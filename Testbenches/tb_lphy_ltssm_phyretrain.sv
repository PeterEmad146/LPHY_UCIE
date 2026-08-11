`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_phyretrain;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int TIMEOUT_CYCLES = 50; 
    
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       en_phyretrain, local_retrain_trigger;
    logic [2:0] local_retrain_enc;
    logic       lp_stallack;
    logic       rx_retrain_init_req, rx_retrain_init_resp;
    logic       rx_retrain_start_req, rx_retrain_start_resp;
    logic [2:0] rx_retrain_enc;
    
    wire        pl_stallreq;
    wire [3:0]  pl_state_sts;
    wire        tx_retrain_init_req, tx_retrain_init_resp;
    wire        tx_retrain_start_req, tx_retrain_start_resp;
    wire [2:0]  tx_retrain_enc;
    wire [7:0]  phyretrain_log;
    wire        rdi_to_retrain, phy_in_retrain;
    wire        exit_to_txselfcal, exit_to_speedidle, exit_to_repair, exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_phyretrain #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) dut (.*); 

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        en_phyretrain = 0; local_retrain_trigger = 0; local_retrain_enc = 0;
        lp_stallack = 0; 
        rx_retrain_init_req = 0; rx_retrain_init_resp = 0;
        rx_retrain_start_req = 0; rx_retrain_start_resp = 0; rx_retrain_enc = 0;
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
        $fsdbDumpfile("tb_lphy_ltssm_phyretrain.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_phyretrain);

        $display("==================================================");
        $display("Starting LTSSM PHYRETRAIN State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Local Initiated Retrain with 4-Phase Stall
        // =========================================================
        $display("Running TC1: Local Trigger & Stallreq Handshake");
        
        @(negedge clk);
        en_phyretrain = 1'b1;
        local_retrain_trigger = 1'b1;
        local_retrain_enc = 3'b001; // We want TXSELFCAL
        
        // Wait to verify pl_stallreq asserts
        @(negedge clk);
        if (!pl_stallreq) $error("[FAIL] TC1: FSM failed to assert pl_stallreq upon entry.");
        if (phyretrain_log !== 8'h13) $error("[FAIL] TC1: Register logging failed. Got %h", phyretrain_log);
        
        // Adapter takes a few cycles to flush pipeline
        repeat(3) @(negedge clk);
        if (tx_retrain_init_req) $error("[FAIL] TC1: Sideband packet illegally sent before StallAck!");
        
        // Adapter flushes and asserts StallAck
        lp_stallack = 1'b1;
        
        // Check sideband output
        @(negedge clk);
        #1; if (!tx_retrain_init_req) $error("[FAIL] TC1: Failed to transmit Init Request.");
        
        // Check Stallreq de-assertion (Phase 3/4)
        if (pl_stallreq) $error("[FAIL] TC1: Failed to de-assert StallReq after receiving StallAck.");
        
        @(negedge clk);
        pulse(rx_retrain_init_resp); // Remote PHY responds
        
        // We transmit Start Req
        #1; if (!tx_retrain_start_req) $error("[FAIL] TC1: Failed to transmit Start Request.");
        
        @(negedge clk);
        pulse(rx_retrain_start_resp); // Remote PHY acknowledges parameter
        
        // Verify Exit Routing
        #1; 
        if (!exit_to_txselfcal) $error("[FAIL] TC1: Failed to correctly route to TXSELFCAL exit.");
        else $display("[PASS] TC1: Stallreq pipeline flush and local retrain successfully executed.");

        en_phyretrain = 1'b0; local_retrain_trigger = 1'b0; lp_stallack = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Remote Initiated Retrain & Priority Conflict (Table 28)
        // =========================================================
        $display("Running TC2: Remote Trigger & Priority Resolution");
        
        clear_inputs();
        en_phyretrain = 1'b1;
        local_retrain_enc = 3'b001; // We want TXSELFCAL
        
        @(negedge clk);
        rx_retrain_init_req = 1'b1; // Remote initiates the retrain
        
        @(negedge clk);
        lp_stallack = 1'b1; // Local adapter grants stall
        rx_retrain_init_req = 1'b0;
        
        #1; if (!tx_retrain_init_resp) $error("[FAIL] TC2: Failed to return Init Response.");
        
        @(negedge clk);
        rx_retrain_start_req = 1'b1;
        rx_retrain_enc = 3'b010; // Remote demands a SPEED DEGRADE!
        
        #1; 
        if (!tx_retrain_start_resp) $error("[FAIL] TC2: Failed to return Start Response.");
        if (tx_retrain_enc !== 3'b010) $error("[FAIL] TC2: FSM failed Table 28 priority override! Sent %b", tx_retrain_enc);
        
        @(negedge clk);
        rx_retrain_start_req = 1'b0;
        
        // Verify Exit Routing - We must exit to SpeedIdle because Remote demanded it
        #1; 
        if (!exit_to_speedidle) $error("[FAIL] TC2: Priority override failed to route exit to SPEEDIDLE.");
        else $display("[PASS] TC2: Remote trigger and Table 28 Speed Degrade override perfectly executed.");

        $display("==================================================");
        $display("LTSSM PHYRETRAIN State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire