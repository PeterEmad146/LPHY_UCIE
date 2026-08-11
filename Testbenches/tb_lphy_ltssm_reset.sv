`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_reset;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int CLK_CYCLES_4MS = 10; // Lowered for fast simulation
    
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic power_stable, sb_clk_stable, mb_clk_stable, mb_clk_slow;
    logic soc_reset_n, start_link_training, sb_rx_wake;
    logic [3:0] lp_state_req;
    logic en_reset;
    
    wire  exit_to_sbinit;
    wire  phy_reset_active;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_reset #(
        .CLK_CYCLES_4MS(CLK_CYCLES_4MS)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .power_stable(power_stable), .sb_clk_stable(sb_clk_stable), 
        .mb_clk_stable(mb_clk_stable), .mb_clk_slow(mb_clk_slow),
        .soc_reset_n(soc_reset_n), .start_link_training(start_link_training), 
        .sb_rx_wake(sb_rx_wake), .lp_state_req(lp_state_req),
        .en_reset(en_reset), .exit_to_sbinit(exit_to_sbinit), 
        .phy_reset_active(phy_reset_active)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        power_stable = 0; sb_clk_stable = 0; mb_clk_stable = 0; mb_clk_slow = 0;
        soc_reset_n = 1; start_link_training = 0; sb_rx_wake = 0;
        lp_state_req = 4'b1111; // Uninitialized garbage
        en_reset = 1;
    endtask

    task assert_power_good();
        power_stable = 1; sb_clk_stable = 1; mb_clk_stable = 1; mb_clk_slow = 1;
    endtask

    // Wait for the 4ms timer to finish
    task wait_for_timer();
        repeat(CLK_CYCLES_4MS + 2) @(negedge clk);
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_ltssm_reset.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_reset);

        $display("==================================================");
        $display("Starting LTSSM RESET State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Analog Timer & Power Glitch Check
        // =========================================================
        $display("Running TC1: Power Stability and Timer Reset");
        assert_power_good();
        repeat(5) @(negedge clk); // Let timer count halfway
        
        power_stable = 1'b0; // SIMULATE POWER GLITCH
        @(negedge clk);
        assert_power_good(); // Power restores
        
        start_link_training = 1'b1; // Assert trigger
        
        // Wait original remaining time (should fail because it reset!)
        repeat(5) @(negedge clk);
        if (exit_to_sbinit === 1'b1) $error("[FAIL] TC1: Timer failed to reset on power glitch.");
        
        // Wait full time
        wait_for_timer();
        if (exit_to_sbinit !== 1'b1 || phy_reset_active !== 1'b0) 
            $error("[FAIL] TC1: Failed to exit after full recovery.");
        else 
            $display("[PASS] TC1: 4ms Timer perfectly enforced and reset on glitch.");
            
        start_link_training = 1'b0;
        en_reset = 0; @(negedge clk); en_reset = 1; // Re-enter reset state

        // =========================================================
        // TC2: Remote Sideband Wake Trigger
        // =========================================================
        $display("Running TC2: Remote SB Wake Trigger");
        assert_power_good();
        wait_for_timer();
        
        sb_rx_wake = 1'b1;
        @(negedge clk);
        
        if (exit_to_sbinit !== 1'b1) $error("[FAIL] TC2: Remote Wake failed to trigger exit.");
        else $display("[PASS] TC2: Remote SB Wake successfully triggered SBINIT transition.");
        
        sb_rx_wake = 1'b0;
        en_reset = 0; @(negedge clk); en_reset = 1; 

        // =========================================================
        // TC3: RDI Adapter Trigger (Enforcing NOP Rule)
        // =========================================================
        $display("Running TC3: RDI NOP-First Rule Enforcement");
        assert_power_good();
        wait_for_timer();
        
        // Adapter tries to jump straight to ACTIVE without sending NOP
        lp_state_req = 4'b0001; 
        @(negedge clk);
        
        if (exit_to_sbinit === 1'b1) $error("[FAIL] TC3: RTL illegally accepted ACTIVE without prior NOP.");
        
        // Adapter complies and sends NOP
        lp_state_req = 4'b0000;
        @(negedge clk);
        
        // Adapter requests ACTIVE again
        lp_state_req = 4'b0001;
        @(negedge clk);
        
        if (exit_to_sbinit !== 1'b1) $error("[FAIL] TC3: RTL failed to exit after valid NOP -> ACTIVE sequence.");
        else $display("[PASS] TC3: Adapter NOP -> ACTIVE request rule perfectly enforced.");

        $display("==================================================");
        $display("LTSSM RESET State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire