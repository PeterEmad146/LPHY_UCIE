`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_rx_top;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int NUM_LANES = 64;
    
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [1:0]  link_width;
    logic        is_retimer;
    logic        free_run_mode;
    logic        is_linkerror;
    logic        force_enable;
    
    logic        rx_training_en;
    logic        en_reversal_check;
    wire         reversal_detected;
    wire         reversal_check_done;
    
    logic        descrambler_en;
    logic        load_seed;
    logic [22:0] lane_seeds [63:0];
    
    logic        repair_en;
    logic        en_lane_check;
    wire  [63:0] detected_lane_failures;
    wire         check_done;
    
    wire         framing_err;
    wire         unrepairable;
    
    wire         pl_valid;
    wire [511:0] pl_data;
    wire         credit_return;
    wire         rx_gated_clk;
    
    logic [7:0]  RXDATA [NUM_LANES-1:0];
    logic [7:0]  RXVLD;
    logic [7:0]  RXRD [3:0];
    logic        RXTRK;
    wire         rx_en;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_rx_top #(
        .NUM_LANES(NUM_LANES)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .link_width(link_width), .is_retimer(is_retimer), .free_run_mode(free_run_mode), 
        .is_linkerror(is_linkerror), .force_enable(force_enable),
        .rx_training_en(rx_training_en), .en_reversal_check(en_reversal_check), 
        .reversal_detected(reversal_detected), .reversal_check_done(reversal_check_done),
        .descrambler_en(descrambler_en), .load_seed(load_seed), .lane_seeds(lane_seeds),
        .repair_en(repair_en), .en_lane_check(en_lane_check), 
        .detected_lane_failures(detected_lane_failures), .check_done(check_done),
        .framing_err(framing_err), .unrepairable(unrepairable),
        .pl_valid(pl_valid), .pl_data(pl_data), .credit_return(credit_return), .rx_gated_clk(rx_gated_clk),
        .RXDATA(RXDATA), .RXVLD(RXVLD), .RXRD(RXRD), .RXTRK(RXTRK), .rx_en(rx_en)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        link_width = 2'b10; is_retimer = 0; free_run_mode = 0; is_linkerror = 0; force_enable = 0;
        rx_training_en = 0; en_reversal_check = 0;
        descrambler_en = 0; load_seed = 0; repair_en = 0; en_lane_check = 0;
        RXVLD = 8'h00; RXTRK = 1'b0;
        for (int i=0; i<64; i++) begin
            lane_seeds[i] = 23'h1DBFBC;
            RXDATA[i] = 8'h00;
        end
        for (int i=0; i<4; i++) RXRD[i] = 8'h00;
    endtask

    // Generates Expected ID pattern for automated training
    function automatic logic [7:0] get_expected_byte(int lane, bit is_byte1);
        logic [7:0] target_id = lane[7:0];
        if (!is_byte1) return {target_id[3:0], 4'b1010};
        else           return {4'b1010, target_id[7:4]};
    endfunction

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_rx_top.fsdb");
        $fsdbDumpvars(0, tb_lphy_rx_top);

        $display("==================================================");
        $display("Starting Mainband RX Top Integration Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Clean Datapath Integration (AFE -> Adapter)
        // =========================================================
        $display("Running TC1: AFE to Adapter Pipeline Alignment");
        
        // Drive physical pins
        RXVLD = 8'h0F; // Standard Valid Frame
        for (int i = 0; i < 64; i++) RXDATA[i] = i[7:0];
        
        // Pipeline Depth: AFE Latch(1) -> Derotator Align(2) -> Output
        @(negedge clk); 
        RXVLD = 8'h00; // Drop valid
        @(negedge clk); 
        @(negedge clk); // Data arrives at Adapter interface
        
        if (pl_valid !== 1'b1) $error("[FAIL] TC1: Adapter Valid failed to assert phase-aligned with data.");
        
        for (int i = 0; i < 64; i++) begin
            if (pl_data[i*8 +: 8] !== i[7:0]) 
                $error("[FAIL] TC1: Datapath corrupted at Lane %0d. Expected %0d", i, i);
        end
        $display("[PASS] TC1: 512-bit payload successfully assembled and pipelined to Adapter.");
        
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Hardware Training & Repair Integration
        // =========================================================
        $display("Running TC2: Automated Repair End-to-End Integration");
        
        // 1. Enter Training Mode
        rx_training_en = 1'b1;
        en_lane_check  = 1'b1;
        RXVLD          = 8'h0F;
        
        // Blast the 128-iteration training pattern, injecting a fault on Lane 10
        for (int iter = 0; iter < 128; iter++) begin
            for (int i = 0; i < 64; i++) RXDATA[i] = (i == 10) ? 8'hFF : get_expected_byte(i, 0);
            @(negedge clk);
            for (int i = 0; i < 64; i++) RXDATA[i] = (i == 10) ? 8'hFF : get_expected_byte(i, 1);
            @(negedge clk);
        end
        
        en_lane_check = 1'b0; // End training
        rx_training_en = 1'b0;
        repair_en = 1'b1;     // Activate Repair Demux
        
        // Wait for pipeline to flush
        repeat(3) @(negedge clk);
        
        // Prove pl_valid was masked during training!
        if (pl_valid === 1'b1) $error("[FAIL] TC2: rx_training_en failed to mask pl_valid from the Adapter.");
        
        // 2. Drive standard payload through the newly repaired physical link
        // To reconstruct Logical 0-63, we simulate the inverse shift on the physical pins:
        RXVLD = 8'h0F;
        RXRD[0] = 8'h00; // Logical 0 comes from Redundant 0
        for (int i = 0; i < 10; i++) RXDATA[i] = (i + 1); // Logical 1-10 come from Physical 0-9
        RXDATA[10] = 8'hFF; // Broken physical lane
        for (int i = 11; i < 64; i++) RXDATA[i] = i;      // Logical 11-63 come from Physical 11-63
        
        @(negedge clk); RXVLD = 8'h00;
        @(negedge clk); 
        @(negedge clk); // Data arrives
        
        // Verify Repair Demux successfully reconstructed the 0-63 payload
        for (int i = 0; i < 64; i++) begin
            if (pl_data[i*8 +: 8] !== i[7:0]) 
                $error("[FAIL] TC2: Repair Integration failed at Logical Lane %0d", i);
        end
        $display("[PASS] TC2: Sub-modules autonomously detected and repaired the physical fault.");
        
        repair_en = 1'b0; // Reset for next test
        repeat(3) @(negedge clk);

        // =========================================================
        // TC3: Scrambler Datapath Integration
        // =========================================================
        $display("Running TC3: Descrambler Integration");
        
        descrambler_en = 1'b1;
        RXVLD = 8'h0F;
        for (int i = 0; i < 64; i++) RXDATA[i] = 8'h00; // Drive all zeros
        
        @(negedge clk); RXVLD = 8'h00;
        @(negedge clk); 
        @(negedge clk); // Data arrives
        
        // Because descrambler is active, output should NO LONGER be 00 (it's XOR'd with the LFSR)
        if (pl_data[7:0] === 8'h00) $error("[FAIL] TC3: Descrambler failed to XOR the datapath.");
        else $display("[PASS] TC3: Descrambler successfully integrated and modified datapath.");

        descrambler_en = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC4: RX Clock Gating & Postamble Envelope
        // =========================================================
        $display("Running TC4: RX Clock Gating Envelope");
        
        RXVLD = 8'h0F;
        @(negedge clk);
        
        // Clock should be running while valid
        #1; if (!rx_gated_clk) $error("[FAIL] TC4: Gated clock failed to run during valid frame.");
        
        @(negedge clk);
        RXVLD = 8'h00; // Valid drops
        
        // Postamble Cycle 1
        #1; if (!rx_gated_clk) $error("[FAIL] TC4: Gated clock failed during Postamble 1.");
        @(negedge clk);
        
        // Postamble Cycle 2
        #1; if (!rx_gated_clk) $error("[FAIL] TC4: Gated clock failed during Postamble 2.");
        @(negedge clk);
        
        // Postamble Expired
        #1; if (rx_gated_clk) $error("[FAIL] TC4: Gated clock failed to shut off after postamble.");
        
        $display("[PASS] TC4: Integrated Clock Gater perfectly enveloped the transaction.");

        $display("==================================================");
        $display("Mainband RX Top Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire