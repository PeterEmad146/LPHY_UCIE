`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_tx_top;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [1:0]  link_width;
    logic        degrade_x8;
    logic        degrade_upper;
    logic        lane_reversal;
    logic        is_retimer;
    logic        free_run_mode;
    logic        txtrk_en;
    
    logic        tx_training_en;
    logic [1:0]  pattern_sel;
    logic        scrambler_en;
    logic        load_seed;
    logic [22:0] lane_seeds [63:0];
    
    logic        repair_en;
    logic [63:0] ext_lane_failed_map;
    wire         unrepairable;
    
    logic        lp_valid;
    logic        lp_irdy;
    wire         pl_trdy;
    logic [511:0] lp_data;
    logic        credit_return;
    
    wire [7:0]   TXDATA [63:0];
    wire [63:0]  TXDATA_OE;
    wire [7:0]   TXVLD;
    wire [7:0]   TXRD [3:0];
    wire [3:0]   TXRD_OE;
    wire         tx_clock_en;
    wire         tx_track_en;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_tx_top #(
        .NUM_LANES(64)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .link_width(link_width), .degrade_x8(degrade_x8), .degrade_upper(degrade_upper),
        .lane_reversal(lane_reversal), .is_retimer(is_retimer), .free_run_mode(free_run_mode), .txtrk_en(txtrk_en),
        .tx_training_en(tx_training_en), .pattern_sel(pattern_sel), .scrambler_en(scrambler_en),
        .load_seed(load_seed), .lane_seeds(lane_seeds),
        .repair_en(repair_en), .ext_lane_failed_map(ext_lane_failed_map), .unrepairable(unrepairable),
        .lp_valid(lp_valid), .lp_irdy(lp_irdy), .pl_trdy(pl_trdy), .lp_data(lp_data), .credit_return(credit_return),
        .TXDATA(TXDATA), .TXDATA_OE(TXDATA_OE), .TXVLD(TXVLD), .TXRD(TXRD), .TXRD_OE(TXRD_OE),
        .tx_clock_en(tx_clock_en), .tx_track_en(tx_track_en)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        link_width = 2'b10; degrade_x8 = 0; degrade_upper = 0; lane_reversal = 0;
        is_retimer = 0; free_run_mode = 0; txtrk_en = 0;
        tx_training_en = 0; pattern_sel = 2'b00; scrambler_en = 0; load_seed = 0;
        repair_en = 0; ext_lane_failed_map = 64'h0;
        lp_valid = 0; lp_irdy = 0; lp_data = '0; credit_return = 0;
        
        for (int i=0; i<64; i++) lane_seeds[i] = 23'h1DBFBC;
    endtask

    task load_incremental_payload();
        for (int i = 0; i < 64; i++) begin
            lp_data[i*8 +: 8] = i[7:0]; 
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_tx_top.fsdb");
        $fsdbDumpvars(0, tb_lphy_tx_top);

        $display("==================================================");
        $display("Starting Mainband TX Top Integration Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Standard x64 Datapath (Adapter -> AFE)
        // =========================================================
        $display("Running TC1: Adapter to AFE Pipeline (x64, No Repair, No Scramble)");
        load_incremental_payload();
        lp_valid = 1'b1;
        lp_irdy  = 1'b1;
        
        // Wait 2 pipeline cycles for data to reach the AFE registers
        @(negedge clk);
        lp_valid = 1'b0; lp_irdy = 1'b0;
        @(negedge clk);
        
        if (TXVLD !== 8'h0F) $error("[FAIL] TC1: Valid Framer integration failed. Expected 0F, Got %0h", TXVLD);
        if (tx_clock_en !== 1'b1) $error("[FAIL] TC1: AFE Clock Enable failed to assert.");
        
        for (int i = 0; i < 64; i++) begin
            if (TXDATA[i] !== i[7:0]) $error("[FAIL] TC1: Datapath corrupted at Lane %0d. Expected %0d, Got %0h", i, i, TXDATA[i]);
        end
        $display("[PASS] TC1: Clean Adapter-to-AFE datapath pipelined perfectly.");
        
        // Wait for clock postamble to expire
        repeat(4) @(negedge clk);

        // =========================================================
        // TC2: LTSSM Training Override (VALTRAIN)
        // =========================================================
        $display("Running TC2: LTSSM Training Override (VALTRAIN Pattern)");
        
        tx_training_en = 1'b1;
        pattern_sel    = 2'b10; // VALTRAIN
        
        @(negedge clk);
        @(negedge clk); // Allow pipeline to register
        
        // During VALTRAIN, data lanes should output 0F and scrambler must be bypassed automatically
        if (TXDATA[0] !== 8'h0F) $error("[FAIL] TC2: Pattern Generator override failed on Data lanes.");
        if (TXVLD !== 8'h0F) $error("[FAIL] TC2: Valid Framer failed to override into training mode.");
        
        $display("[PASS] TC2: LTSSM successfully commandeered the datapath for VALTRAIN.");
        
        tx_training_en = 1'b0;
        repeat(4) @(negedge clk);

        // =========================================================
        // TC3: Scrambler Integration
        // =========================================================
        $display("Running TC3: Scrambler Datapath Integration");
        
        scrambler_en = 1'b1;
        load_incremental_payload();
        lp_valid = 1'b1;
        lp_irdy  = 1'b1;
        
        @(negedge clk);
        lp_valid = 1'b0; lp_irdy = 1'b0;
        @(negedge clk);
        
        // Because scrambler is active, TXDATA[0] should NO LONGER equal its logical input (8'h00)
        if (TXDATA[0] === 8'h00) $error("[FAIL] TC3: Scrambler failed to XOR the datapath.");
        else $display("[PASS] TC3: Scrambler successfully engaged and modified datapath.");

        scrambler_en = 1'b0;
        repeat(4) @(negedge clk);

        // =========================================================
        // TC4: Physical Repair & Tri-State Propagation
        // =========================================================
        $display("Running TC4: AFE Tri-State (OE) Propagation");
        
        repair_en = 1'b1;
        ext_lane_failed_map = 64'h0;
        ext_lane_failed_map[12] = 1'b1; // Inject fault at physical lane 12
        
        load_incremental_payload();
        lp_valid = 1'b1; lp_irdy = 1'b1;
        
        @(negedge clk);
        lp_valid = 1'b0; lp_irdy = 1'b0;
        @(negedge clk);
        
        if (TXDATA_OE[12] !== 1'b0 || TXDATA[12] !== 8'h00) 
            $error("[FAIL] TC4: Tri-State Output Enable failed to propagate to AFE boundary.");
            
        if (TXRD_OE[0] !== 1'b1) 
            $error("[FAIL] TC4: Redundant Bump Output Enable failed to propagate to AFE boundary.");
            
        $display("[PASS] TC4: Analog Output Enables (Tri-States) successfully registered at AFE boundary.");

        $display("==================================================");
        $display("Mainband TX Top Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire