`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_pattern_gen;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [7:0]  lane_id;
    logic [1:0]  pattern_sel;
    logic        enable;
    wire  [7:0]  pattern_out;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_pattern_gen dut (
        .clk(clk),
        .rst_n(rst_n),
        .lane_id(lane_id),
        .pattern_sel(pattern_sel),
        .enable(enable),
        .pattern_out(pattern_out)
    );

    // ---------------------------------------------------------
    // Helper State
    // ---------------------------------------------------------
    task clear_inputs();
        lane_id     = 8'h00;
        pattern_sel = 2'b00;
        enable      = 1'b0;
    endtask

    logic [7:0] expected_cr [4];

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_pattern_gen.fsdb");
        $fsdbDumpvars(0, tb_lphy_pattern_gen);

        $display("==================================================");
        $display("Starting Mainband TX Pattern Generator Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Idle / None Pattern (2'b00)
        // =========================================================
        $display("Running TC1: Idle State");
        pattern_sel = 2'b00;
        enable      = 1'b1;
        
        #1; // Yield to combinatorial logic
        if (pattern_out !== 8'h00)
            $error("[FAIL] TC1: Expected 8'h00, Got %0h", pattern_out);
        else
            $display("[PASS] TC1: Idle pattern generated correctly (8'h00).");
            
        @(negedge clk);

        // =========================================================
        // TC2: VALTRAIN Pattern (2'b10) - 1 Phase Loop
        // =========================================================
        $display("Running TC2: VALTRAIN Pattern Streaming");
        pattern_sel = 2'b10; // VALTRAIN
        enable      = 1'b1;
        
        for (int i = 0; i < 3; i++) begin
            #1;
            if (pattern_out !== 8'h0F) begin
                $error("[FAIL] TC2: Expected 8'h0F, Got %0h at cycle %0d", pattern_out, i);
                $finish;
            end
            @(negedge clk);
        end
        $display("[PASS] TC2: VALTRAIN streamed correctly (0F -> 0F -> 0F).");

        // =========================================================
        // TC3: Per-Lane ID Pattern (2'b01) - 2 Phase Loop
        // =========================================================
        $display("Running TC3: Per-Lane ID Streaming (Lane ID = 0xC3)");
        // 16-bit encoding for 8'hC3 (1100_0011): 
        // Phase 0: {lane_id[3:0], 4'b1010} = {4'h3, 4'hA} = 8'h3A
        // Phase 1: {4'b1010, lane_id[7:4]} = {4'hA, 4'hC} = 8'hAC
        
        clear_inputs(); // Forces phase_cnt to reset back to 0
        @(negedge clk); 
        
        lane_id     = 8'hC3;
        pattern_sel = 2'b01; 
        enable      = 1'b1;
        
        // Cycle 0: Expect Phase 0
        #1;
        if (pattern_out !== 8'h3A) $error("[FAIL] TC3: Phase 0 failed. Got %0h", pattern_out);
        @(negedge clk);
        
        // Cycle 1: Expect Phase 1
        #1;
        if (pattern_out !== 8'hAC) $error("[FAIL] TC3: Phase 1 failed. Got %0h", pattern_out);
        @(negedge clk);
        
        // Cycle 2: Expect Phase 0 again (Loop back)
        #1;
        if (pattern_out !== 8'h3A) $error("[FAIL] TC3: Loopback failed. Got %0h", pattern_out);
        else $display("[PASS] TC3: Per-Lane ID streamed perfectly (3A -> AC -> 3A).");
        @(negedge clk);

        // =========================================================
        // TC4: Clock Repair Pattern (2'b11) - 3 Phase Loop
        // =========================================================
        $display("Running TC4: Clock Repair Streaming");
        // 24-bit encoding: AA -> AA -> 00
        
        clear_inputs();
        @(negedge clk);
        
        pattern_sel = 2'b11;
        enable      = 1'b1;
        
        // Stream out 4 cycles to prove the 3-phase loopback
        expected_cr= '{8'hAA, 8'hAA, 8'h00, 8'hAA};
        
        for (int i = 0; i < 4; i++) begin
            #1;
            if (pattern_out !== expected_cr[i]) begin
                $error("[FAIL] TC4: Phase %0d mismatch. Expected %0h, Got %0h", i, expected_cr[i], pattern_out);
                $finish;
            end
            @(negedge clk);
        end
        $display("[PASS] TC4: Clock Repair streamed correctly (AA -> AA -> 00 -> AA).");

        // =========================================================
        // TC5: Pipeline Stall (Enable Pin Toggle)
        // =========================================================
        $display("Running TC5: Stream Pausing (Enable Pin)");
        
        clear_inputs();
        @(negedge clk);
        
        pattern_sel = 2'b11; // Clock repair again
        enable      = 1'b1;
        
        // Cycle 0: Phase 0 (AA)
        #1;
        @(negedge clk); 
        
        // Cycle 1: We are now at Phase 1. Let's drop enable.
        enable = 1'b0;
        
        #1;
        if (pattern_out !== 8'hAA) $error("[FAIL] TC5: Combinatorial output shifted during disable.");
        
        @(negedge clk); // Clock ticks, but phase_cnt should not advance because enable=0
        
        #1;
        if (pattern_out !== 8'hAA) $error("[FAIL] TC5: Phase advanced despite enable=0.");
        
        // Re-enable
        enable = 1'b1;
        @(negedge clk);
        
        // Cycle 2: We should now advance to Phase 2 (00)
        #1;
        if (pattern_out !== 8'h00) $error("[FAIL] TC5: Phase failed to resume correctly. Got %0h", pattern_out);
        else $display("[PASS] TC5: Pattern generation successfully paused and resumed.");

        $display("==================================================");
        $display("Pattern Generator Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire