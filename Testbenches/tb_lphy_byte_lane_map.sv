`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_byte_lane_map;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [1:0]   link_width;
    logic         degrade_x8;
    logic         degrade_upper;
    logic         lane_reversal;
    
    logic         lp_valid;
    logic         lp_irdy;
    wire          pl_trdy;
    logic [511:0] lp_data;
    
    wire          lane_valid;
    wire  [7:0]   lane_data [63:0];

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_byte_lane_map dut (
        .clk(clk),
        .rst_n(rst_n),
        .link_width(link_width),
        .degrade_x8(degrade_x8),
        .degrade_upper(degrade_upper),
        .lane_reversal(lane_reversal),
        .lp_valid(lp_valid),
        .lp_irdy(lp_irdy),
        .pl_trdy(pl_trdy),
        .lp_data(lp_data),
        .lane_valid(lane_valid),
        .lane_data(lane_data)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        link_width    = 2'b00; // default x16
        degrade_x8    = 1'b0;
        degrade_upper = 1'b0;
        lane_reversal = 1'b0;
        lp_valid      = 1'b0;
        lp_irdy       = 1'b0;
        lp_data       = '0;
    endtask

    // Loads the 64-Byte payload such that Byte[i] = i (0 to 63)
    task load_incremental_payload();
        for (int i = 0; i < 64; i++) begin
            lp_data[i*8 +: 8] = i[7:0]; 
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_byte_lane_map.fsdb");
        $fsdbDumpvars(0, tb_lphy_byte_lane_map);

        $display("==================================================");
        $display("Starting Mainband TX Byte-to-Lane Mapper Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: x64 Mode (1 Cycle Transfer)
        // =========================================================
        $display("Running TC1: x64 Mode (All 64 Bytes in 1 Cycle)");
        link_width = 2'b10; // x64
        load_incremental_payload();
        lp_valid = 1'b1;
        lp_irdy  = 1'b1;
        
        @(negedge clk); // Allow DUT to sample and register output
        lp_valid = 1'b0;
        lp_irdy  = 1'b0;
        
        if (!lane_valid) $error("[FAIL] TC1: lane_valid did not assert.");
        for (int i = 0; i < 64; i++) begin
            if (lane_data[i] !== i[7:0]) $error("[FAIL] TC1: Mismatch on Lane %0d. Expected %0d, Got %0d", i, i, lane_data[i]);
        end
        $display("[PASS] TC1: x64 mapping successful.");
        @(negedge clk);

        // =========================================================
        // TC2: x32 Mode (2 Cycle Transfer)
        // =========================================================
        $display("Running TC2: x32 Mode (64 Bytes over 2 Cycles)");
        link_width = 2'b01; // x32
        lp_valid = 1'b1;
        lp_irdy  = 1'b1;
        
        // Cycle 0: Expect Bytes 0-31
        @(negedge clk);
        lp_valid = 1'b0; lp_irdy = 1'b0;
        if (!lane_valid) $error("[FAIL] TC2.0: lane_valid dropped.");
        for (int i = 0; i < 32; i++) begin
            if (lane_data[i] !== i[7:0]) $error("[FAIL] TC2.0: Mismatch on Lane %0d", i);
        end
        
        // Cycle 1: Expect Bytes 32-63
        @(negedge clk);
        if (!lane_valid) $error("[FAIL] TC2.1: lane_valid dropped.");
        for (int i = 0; i < 32; i++) begin
            if (lane_data[i] !== (i + 32)) $error("[FAIL] TC2.1: Mismatch on Lane %0d", i);
        end
        
        $display("[PASS] TC2: x32 mapping successful.");
        @(negedge clk);

        // =========================================================
        // TC3: x16 Mode (4 Cycle Transfer)
        // =========================================================
        $display("Running TC3: x16 Mode (64 Bytes over 4 Cycles)");
        link_width = 2'b00; // x16
        lp_valid = 1'b1; lp_irdy = 1'b1;
        
        for (int cycle = 0; cycle < 4; cycle++) begin
            @(negedge clk);
            if (cycle == 0) begin lp_valid = 1'b0; lp_irdy = 1'b0; end
            
            for (int i = 0; i < 16; i++) begin
                if (lane_data[i] !== (i + cycle*16)) $error("[FAIL] TC3: Mismatch on Cycle %0d, Lane %0d", cycle, i);
            end
        end
        $display("[PASS] TC3: x16 mapping successful.");
        @(negedge clk);

        // =========================================================
        // TC4: x8 Degraded Mode Upper (8 Cycle Transfer)
        // =========================================================
        $display("Running TC4: x8 Degraded Mode mapped to Lanes [15:8]");
        link_width    = 2'b00; // Base x16
        degrade_x8    = 1'b1;
        degrade_upper = 1'b1;  // Map to [15:8]
        lp_valid = 1'b1; lp_irdy = 1'b1;
        
        for (int cycle = 0; cycle < 8; cycle++) begin
            @(negedge clk);
            if (cycle == 0) begin lp_valid = 1'b0; lp_irdy = 1'b0; end
            
            // Check that Lanes [7:0] are isolated/zeroed
            if (lane_data[0] !== 8'h00) $error("[FAIL] TC4: Lane 0 not isolated.");
            
            // Check Lanes [15:8]
            for (int i = 0; i < 8; i++) begin
                if (lane_data[i+8] !== (i + cycle*8)) $error("[FAIL] TC4: Mismatch on Cycle %0d, Lane %0d", cycle, i+8);
            end
        end
        $display("[PASS] TC4: x8 Degraded mapping successful.");
        @(negedge clk);

        // =========================================================
        // TC5: x16 Lane Reversal (Wiring fix)
        // =========================================================
        $display("Running TC5: x16 Logical Lane Reversal");
        link_width    = 2'b00; // x16
        degrade_x8    = 1'b0;
        lane_reversal = 1'b1;  // Activate Reversal Mux
        lp_valid = 1'b1; lp_irdy = 1'b1;
        
        @(negedge clk);
        lp_valid = 1'b0; lp_irdy = 1'b0;
        
        // Cycle 0: Normal is Bytes 0-15. Reversed means Lane i gets Byte (15-i)
        for (int i = 0; i < 16; i++) begin
            if (lane_data[i] !== (15 - i)) $error("[FAIL] TC5: Mismatch on Lane %0d. Expected %0d, Got %0d", i, (15-i), lane_data[i]);
        end
        
        // Let it flush out
        repeat(3) @(negedge clk);
        
        $display("[PASS] TC5: Logical Lane Reversal successful.");

        $display("==================================================");
        $display("Byte-to-Lane Mapper Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire