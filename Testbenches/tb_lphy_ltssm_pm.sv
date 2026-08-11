`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_pm;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int TIMEOUT_2US_CYCLES = 20; // Lowered for rapid testing
    
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       en_l1, en_l2;
    logic [3:0] lp_state_req;
    logic       rx_req_active, rx_rsp_active;
    
    wire [3:0]  pl_state_sts;
    wire        pl_inband_pres;
    wire        tx_req_active, tx_rsp_active;
    wire [7:0]  pm_log;
    
    wire        exit_to_speedidle;
    wire        exit_to_reset;
    wire        exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_pm #(
        .TIMEOUT_2US_CYCLES(TIMEOUT_2US_CYCLES)
    ) dut (.*); // Using implicit connections

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        en_l1 = 0; en_l2 = 0;
        lp_state_req = 4'b0000;
        rx_req_active = 0; rx_rsp_active = 0;
    endtask

    task automatic pulse(ref logic sig);
        sig = 1'b1;
        @(negedge clk);
        sig = 1'b0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_ltssm_pm.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_pm);

        $display("==================================================");
        $display("Starting LTSSM PM (L1/L2) State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Remote Initiation - Wake up from L1
        // =========================================================
        $display("Running TC1: Remote Wakeup from L1");
        
        @(negedge clk);
        en_l1 = 1'b1;
        
        // Wait for FSM to enter ST_L1
        @(negedge clk);
        
        if (pl_state_sts !== 4'b0100) $error("[FAIL] TC1: RDI Status failed to reflect L1 (0100b).");
        if (pm_log !== 8'h17) $error("[FAIL] TC1: Logging failed. Got %h", pm_log);
        if (pl_inband_pres !== 1'b0) $error("[FAIL] TC1: In-Band Presence was not de-asserted!");
        
        // Remote PHY sends Wakeup Request
        rx_req_active = 1'b1;
        
        // FIX: Check Response immediately!
        #1; 
        if (!tx_rsp_active) $error("[FAIL] TC1: FSM failed to respond to remote wakeup request.");
        
        @(negedge clk);
        rx_req_active = 1'b0;
        
        // FSM is in ST_EXITING. Check routing flag.
        #1; 
        if (!exit_to_speedidle) $error("[FAIL] TC1: L1 exit failed to route to MBTRAIN.SPEEDIDLE.");
        if (exit_to_reset) $error("[FAIL] TC1: L1 exit illegally routed to RESET.");
        else $display("[PASS] TC1: Remote L1 wakeup perfectly executed and routed.");

        en_l1 = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Local Initiation - Wake up from L2
        // =========================================================
        $display("Running TC2: Local Adapter Wakeup from L2");
        
        clear_inputs();
        en_l2 = 1'b1;
        
        @(negedge clk);
        if (pl_state_sts !== 4'b1000) $error("[FAIL] TC2: RDI Status failed to reflect L2 (1000b).");
        
        // Local Adapter requests Active
        lp_state_req = 4'b0001; 
        
        #1; 
        if (!tx_req_active) $error("[FAIL] TC2: FSM failed to transmit Wakeup Request to remote PHY.");
        
        @(negedge clk); // State is now ST_WAKE_REQ
        
        pulse(rx_rsp_active); // Remote PHY acknowledges
        
        // FSM is in ST_EXITING. Check routing flag.
        #1; 
        if (!exit_to_reset) $error("[FAIL] TC2: L2 exit failed to route to RESET.");
        if (exit_to_speedidle) $error("[FAIL] TC2: L2 exit illegally routed to SPEEDIDLE.");
        else $display("[PASS] TC2: Local L2 wakeup perfectly executed and routed.");

        en_l2 = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC3: 2us Response Timeout Escalation
        // =========================================================
        $display("Running TC3: 2us Wakeup Handshake Timeout");
        
        clear_inputs();
        en_l1 = 1'b1;
        @(negedge clk);
        
        lp_state_req = 4'b0001; // Local asks to wake up
        @(negedge clk); // We are now stuck in ST_WAKE_REQ
        
        // Remote PHY is dead. Wait out the timer...
        repeat(TIMEOUT_2US_CYCLES + 3) @(negedge clk);
        
        #1;
        if (!exit_to_trainerror) $error("[FAIL] TC3: Failed to escalate to TRAINERROR on 2us timeout.");
        else $display("[PASS] TC3: 2us timeout correctly escalated to fatal error.");

        $display("==================================================");
        $display("LTSSM PM State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire