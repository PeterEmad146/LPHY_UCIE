`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_active;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       en_active;
    logic [3:0] lp_state_req;
    logic       lp_linkerror, internal_retrain_req, internal_error_req;
    logic       rx_req_l1, rx_rsp_l1;
    logic       rx_req_l2, rx_rsp_l2;
    logic       rx_req_linkreset, rx_rsp_linkreset;
    logic       rx_req_disable, rx_rsp_disable;
    logic       rx_req_retrain, rx_rsp_retrain;
    logic       rx_req_linkerror;
    
    wire        tx_req_l1, tx_rsp_l1;
    wire        tx_req_l2, tx_rsp_l2;
    wire        tx_req_linkreset, tx_rsp_linkreset;
    wire        tx_req_disable, tx_rsp_disable;
    wire        tx_req_retrain, tx_rsp_retrain;
    wire        tx_req_linkerror;
    
    wire [3:0]  pl_state_sts;
    wire [7:0]  active_log;
    wire        scrambling_en;
    wire        exit_to_l1, exit_to_l2;
    wire        exit_to_linkreset, exit_to_disable;
    wire        exit_to_retrain, exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_active dut (.*); // Using implicit connections

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        en_active = 0; lp_state_req = 4'b0000;
        lp_linkerror = 0; internal_retrain_req = 0; internal_error_req = 0;
        rx_req_l1 = 0; rx_rsp_l1 = 0; rx_req_l2 = 0; rx_rsp_l2 = 0;
        rx_req_linkreset = 0; rx_rsp_linkreset = 0;
        rx_req_disable = 0; rx_rsp_disable = 0;
        rx_req_retrain = 0; rx_rsp_retrain = 0; rx_req_linkerror = 0;
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
        $fsdbDumpfile("tb_lphy_ltssm_active.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_active);

        $display("==================================================");
        $display("Starting LTSSM ACTIVE State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Enter Active and Process Remote L1 Request
        // =========================================================
        $display("Running TC1: Remote Sideband Request Handshake (L1)");
        
        @(negedge clk);
        en_active = 1'b1;
        
        #1;
        if (!scrambling_en) $error("[FAIL] TC1: LFSR Scrambler failed to enable upon Active entry.");
        if (pl_state_sts !== 4'b0001) $error("[FAIL] TC1: RDI Status failed to reflect 0001b.");
        if (active_log !== 8'h15) $error("[FAIL] TC1: Logging failed. Got %h", active_log);
        
        @(negedge clk);
        pulse(rx_req_l1); // Remote asks to go to L1
        
        #1;
        if (!tx_rsp_l1) $error("[FAIL] TC1: FSM failed to respond to remote L1 request.");
        
        @(negedge clk);
        if (!exit_to_l1) $error("[FAIL] TC1: Failed to trigger exit to L1.");
        else $display("[PASS] TC1: Clean Remote Sideband Handshake execution.");

        en_active = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Process Local Adapter Request (Retrain)
        // =========================================================
        $display("Running TC2: Local Adapter Request Handshake (Retrain)");
        
        clear_inputs();
        en_active = 1'b1;
        @(negedge clk);
        
        lp_state_req = 4'b1011; // Adapter requests Retrain
        
        @(negedge clk);
        #1; if (!tx_req_retrain) $error("[FAIL] TC2: FSM failed to transmit Retrain sideband request.");
        
        pulse(rx_rsp_retrain); // Remote grants it
        
        @(negedge clk);
        if (!exit_to_retrain) $error("[FAIL] TC2: Failed to trigger exit to Retrain.");
        else $display("[PASS] TC2: Clean Local Adapter Request Handshake execution.");

        en_active = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC3: Fatal Error Override
        // =========================================================
        $display("Running TC3: LinkError Override & Interruption");
        
        clear_inputs();
        en_active = 1'b1;
        @(negedge clk);
        
        // Start a normal handshake...
        lp_state_req = 4'b1000; // Adapter requests L2
        @(negedge clk);
        #1; if (!tx_req_l2) $error("[FAIL] TC3: Failed to start L2 sequence.");
        
        // SUDDENLY, A FATAL ERROR OCCURS BEFORE THE RESPONSE!
        lp_linkerror = 1'b1;
        
        @(negedge clk);
        #1; 
        if (!exit_to_trainerror) $error("[FAIL] TC3: LinkError failed to override pending handshake!");
        if (!tx_req_linkerror) $error("[FAIL] TC3: Failed to transmit LinkError sideband packet.");
        else $display("[PASS] TC3: Fatal LinkError successfully bypassed and aborted pending transactions.");

        $display("==================================================");
        $display("LTSSM ACTIVE State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire