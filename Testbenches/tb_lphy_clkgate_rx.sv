`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_clkgate_rx;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic free_run_mode;
    logic is_linkerror;
    logic force_enable;
    logic valid_in;
    wire  gated_clk;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_clkgate_rx dut (
        .clk(clk),
        .rst_n(rst_n),
        .free_run_mode(free_run_mode),
        .is_linkerror(is_linkerror),
        .force_enable(force_enable),
        .valid_in(valid_in),
        .gated_clk(gated_clk)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        free_run_mode = 1'b0;
        is_linkerror  = 1'b0;
        force_enable  = 1'b0;
        valid_in      = 1'b0;
    endtask

    // This task checks the state of the gated clock exactly when the 
    // main source clock is at its peak (high).
    task check_clock_pulse(input logic expected_running, input string tc_name);
        @(posedge clk);
        #1; // Yield to allow the AND gate to evaluate
        
        if (expected_running && gated_clk !== 1'b1) begin
            $error("[FAIL] %s: Clock incorrectly gated! (Expected High, Got Low)", tc_name);
        end else if (!expected_running && gated_clk !== 1'b0) begin
            $error("[FAIL] %s: Clock failed to gate! (Power leak detected)", tc_name);
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_clkgate_rx.fsdb");
        $fsdbDumpvars(0, tb_lphy_clkgate_rx);

        $display("==================================================");
        $display("Starting RX Clock Gater Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;

        // =========================================================
        // TC1: Standard Data Valid & 2-Cycle Postamble
        // =========================================================
        $display("Running TC1: 16 UI (2-Cycle) Postamble Verification");
        
        @(negedge clk);
        valid_in = 1'b1; // Start data transfer
        
        // Check that clock runs while valid
        check_clock_pulse(1'b1, "TC1 (Valid High)"); 
        check_clock_pulse(1'b1, "TC1 (Valid High)");
        
        @(negedge clk);
        valid_in = 1'b0; // End data transfer
        
        // Postamble Cycle 1: Clock should still run
        check_clock_pulse(1'b1, "TC1 (Postamble Cycle 1)");
        
        // Postamble Cycle 2: Clock should still run
        check_clock_pulse(1'b1, "TC1 (Postamble Cycle 2)");
        
        // Cycle 3: Postamble expired, clock must now be gated flat
        check_clock_pulse(1'b0, "TC1 (Gated Cycle 3)");
        check_clock_pulse(1'b0, "TC1 (Gated Cycle 4)");
        
        $display("[PASS] TC1: Clock perfectly gated after the strict 2-cycle postamble.");

        // =========================================================
        // TC2: Free-Run Mode
        // =========================================================
        $display("Running TC2: Free-Run Mode Override");
        
        @(negedge clk);
        free_run_mode = 1'b1;
        valid_in = 1'b0; // Datapath idle
        
        // Check that clock ignores valid_in and runs anyway
        check_clock_pulse(1'b1, "TC2 (Free-Run)");
        check_clock_pulse(1'b1, "TC2 (Free-Run)");
        
        $display("[PASS] TC2: Free-Run mode successfully bypassed dynamic gating.");

        // =========================================================
        // TC3: LinkError Containment Rule
        // =========================================================
        $display("Running TC3: LinkError State Prohibition");
        
        @(negedge clk);
        free_run_mode = 1'b0; 
        is_linkerror  = 1'b1; // LTSSM declares fatal error
        
        // Clock must run to allow sideband/error handlers to flush
        check_clock_pulse(1'b1, "TC3 (LinkError)");
        check_clock_pulse(1'b1, "TC3 (LinkError)");
        
        $display("[PASS] TC3: LinkError rule successfully forced clock on.");

        // =========================================================
        // TC4: Training / Wakeup Override
        // =========================================================
        $display("Running TC4: Force Enable (Training/Wakeup)");
        
        @(negedge clk);
        is_linkerror = 1'b0;
        force_enable = 1'b1; // Adapter/LTSSM asserts wake
        
        // Clock must run
        check_clock_pulse(1'b1, "TC4 (Force Enable)");
        
        // Drop it and watch the postamble catch it safely
        @(negedge clk);
        force_enable = 1'b0;
        check_clock_pulse(1'b1, "TC4 (Force Enable Dropped - Postamble 1)");
        check_clock_pulse(1'b1, "TC4 (Force Enable Dropped - Postamble 2)");
        check_clock_pulse(1'b0, "TC4 (Safely Gated)");

        $display("[PASS] TC4: Training override applied and released cleanly.");

        $display("==================================================");
        $display("RX Clock Gater Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire