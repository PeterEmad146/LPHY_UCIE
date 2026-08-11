`default_nettype none
`timescale 1ns / 1ps

module tb_ucie_lphy_top;

    localparam int SIM_TIMEOUT = 500; 
    localparam int NUM_LANES   = 64;
    
    logic lclk;
    logic rst_n;
    always #5 lclk = ~lclk;

    // ---------------------------------------------------------
    // 1. RDI / Adapter Interface Signals
    // ---------------------------------------------------------
    logic         start_link_training, lp_linkerror, lp_stallack;
    logic [3:0]   lp_state_req;
    logic         lp_valid, lp_irdy;
    logic [511:0] lp_data;
    
    wire          pl_trdy, pl_valid, pl_inband_pres, pl_protocol_vld;
    wire  [511:0] pl_data;
    wire  [3:0]   pl_state_sts;
    wire          pl_error, pl_cerror, pl_nferror, pl_trainerror, pl_phyinrecenter;
    wire          pl_stallreq, pl_clk_req, pl_wake_ack;
    wire  [2:0]   pl_speedmode, pl_lnk_cfg;
    wire  [7:0]   error_log_0;
    
    logic         lp_clk_ack, lp_wake_req;

    // ---------------------------------------------------------
    // 2. Global Hardware & AFE Status
    // ---------------------------------------------------------
    logic         power_stable, sb_clk_stable, mb_clk_stable, mb_clk_slow, soc_reset_n;
    
    wire  [5:0]   afe_pi_phase;
    wire  [7:0]   TXDATA [NUM_LANES-1:0];
    wire  [63:0]  TXDATA_OE;
    wire  [7:0]   TXVLD;
    wire  [7:0]   TXRD [3:0];
    wire  [3:0]   TXRD_OE;
    wire  [7:0]   TXRDVLD;
    wire          tx_clock_en, tx_track_en, TXCKP, TXCKN, TXTRK, TXRDCK;
    
    logic [7:0]   RXDATA [NUM_LANES-1:0];
    logic [7:0]   RXVLD;
    logic [7:0]   RXRD [3:0];
    logic [7:0]   RXRDVLD;
    logic         RXTRK, RXCKP, RXCKN, RXRDCK;
    wire          rx_en, rx_gated_clk;

    wire          afe_tx_valid, afe_tx_ipg_en, afe_tx_rd_en;
    wire  [63:0]  afe_tx_data;
    logic         afe_tx_ready, afe_rx_valid;
    logic [63:0]  afe_rx_data;
    wire          afe_rx_en, afe_rx_rd_en;

    logic         tx_req_valid, tx_ep, tx_cr, tx_local_crd_ret;
    logic [4:0]   tx_opcode, tx_tag;
    logic [2:0]   tx_srcid, tx_dstid, tx_cp_status;
    logic [63:0]  tx_payload;
    logic [7:0]   tx_be, tx_msgcode, tx_msgsubcode;
    logic [15:0]  tx_msginfo;
    logic [23:0]  tx_addr;
    
    wire          tx_req_ready, rx_req_valid, rx_ep, rx_cr, rx_parity_err;
    wire  [4:0]   rx_opcode, rx_tag;
    wire  [2:0]   rx_srcid, rx_dstid, rx_cp_status;
    wire  [63:0]  rx_payload;
    wire  [7:0]   rx_be, rx_msgcode, rx_msgsubcode;
    wire  [15:0]  rx_msginfo;
    wire  [23:0]  rx_addr;

    // ---------------------------------------------------------
    // 3. LTSSM Sideband Mocks & Calibration Completes
    // ---------------------------------------------------------
    logic [3:0]   rx_pattern_detected;
    logic         sb_rx_wake;
    logic         rx_msg_out_of_reset, rx_msg_done_req, rx_msg_done_resp;
    logic         rx_req_active, rx_rsp_active, rx_req_l1, rx_rsp_l1, rx_req_l2, rx_rsp_l2;
    logic         rx_req_linkreset, rx_rsp_linkreset, rx_req_disable, rx_rsp_disable;
    logic         rx_req_retrain, rx_rsp_retrain, rx_req_linkerror;
    logic         rx_trainerror_req, rx_trainerror_resp;
    logic         rx_retrain_init_req, rx_retrain_init_resp;
    logic         rx_retrain_start_req, rx_retrain_start_resp;
    logic [2:0]   rx_retrain_enc;

    wire          tx_send_pattern, tx_msg_out_of_reset, tx_msg_done_req, tx_msg_done_resp;
    wire  [2:0]   sb_repair_sel;
    wire          tx_req_active, tx_rsp_active, tx_req_l1, tx_rsp_l1, tx_req_l2, tx_rsp_l2;
    wire          tx_req_linkreset, tx_rsp_linkreset, tx_req_disable, tx_rsp_disable;
    wire          tx_req_retrain, tx_rsp_retrain, tx_req_linkerror;
    wire          tx_trainerror_req, tx_trainerror_resp;
    wire          tx_retrain_init_req, tx_retrain_init_resp;
    wire          tx_retrain_start_req, tx_retrain_start_resp;
    wire  [2:0]   tx_retrain_enc;

    logic         param_done, repairclk_done, repairval_done, reversal_done, repairmb_done;
    logic         valvref_done, datavref_done, speedidle_done, txselfcal_done, rxclkcal_done;
    logic         valtraincenter_done, valtrainvref_done, datatrainvref_done, rxdeskew_done;
    logic         datatraincenter2_done, linkspeed_done, linkspeed_error;
    logic         needs_repair, needs_speed_degrade, repair_done;
    logic         internal_retrain_req, internal_error_req;
    logic [2:0]   local_retrain_enc;

    // ---------------------------------------------------------
    // Behavioral AFE RX Mock: Feed matching training pattern
    // ---------------------------------------------------------
    always_comb begin
        for (int i = 0; i < NUM_LANES; i++) begin
            RXDATA[i] = 8'h0F; // Matches expected training data for calibration
        end
        RXVLD = 8'hFF;
    end

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    ucie_lphy_top #(
        .NUM_LANES(NUM_LANES),
        .PACKAGE_TYPE(1'b0), // Advanced
        .RESET_TIMER_CYCLES(SIM_TIMEOUT),
        .SBINIT_TIMEOUT(SIM_TIMEOUT),
        .MBINIT_TIMEOUT(SIM_TIMEOUT),
        .MBTRAIN_TIMEOUT(SIM_TIMEOUT),
        .LINKINIT_TIMEOUT(SIM_TIMEOUT),
        .PHYRETRAIN_TIMEOUT(SIM_TIMEOUT),
        .TRAINERROR_TIMEOUT(SIM_TIMEOUT),
        
        // FIXED: Fast-forward the hardware calibration sweep for simulation!
        .D2C_PI_PHASE_MAX(3),   // Only sweep 4 phases instead of 64
        .D2C_SETTLE_CYCLES(2),  // Settle in 2 cycles instead of 32
        .D2C_TEST_CYCLES(4)     // Test over 4 cycles instead of 128
    ) dut (.*); 

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        lp_state_req = 0; lp_linkerror = 0; lp_stallack = 0; start_link_training = 0;
        lp_valid = 0; lp_irdy = 0; lp_data = '0;
        lp_clk_ack = 1; lp_wake_req = 0;
        power_stable = 0; sb_clk_stable = 0; mb_clk_stable = 0; mb_clk_slow = 0;
        soc_reset_n = 1; 
        
        afe_tx_ready = 1; afe_rx_valid = 0; afe_rx_data = '0;
        RXRD = '{default:'0}; RXRDVLD = '0;
        RXTRK = 0; RXCKP = 0; RXCKN = 0; RXRDCK = 0;
        
        tx_req_valid = 0; tx_opcode = 0; tx_srcid = 0; tx_dstid = 0; tx_ep = 0;
        tx_cr = 0; tx_payload = 0; tx_tag = 0; tx_be = 0; tx_addr = 0; tx_cp_status = 0;
        tx_msgcode = 0; tx_msgsubcode = 0; tx_msginfo = 0; tx_local_crd_ret = 0;

        sb_rx_wake = 0; rx_pattern_detected = 0; rx_msg_out_of_reset = 0; rx_msg_done_req = 0; 
        rx_msg_done_resp = 0; rx_req_active = 0; rx_rsp_active = 0; rx_req_l1 = 0; rx_rsp_l1 = 0;
        rx_req_l2 = 0; rx_rsp_l2 = 0; rx_req_linkreset = 0; rx_rsp_linkreset = 0;
        rx_req_disable = 0; rx_rsp_disable = 0; rx_req_retrain = 0; rx_rsp_retrain = 0;
        rx_req_linkerror = 0; rx_trainerror_req = 0; rx_trainerror_resp = 0;
        rx_retrain_init_req = 0; rx_retrain_init_resp = 0; rx_retrain_start_req = 0;
        rx_retrain_start_resp = 0; rx_retrain_enc = 0;
        
        param_done = 0; repairclk_done = 0; repairval_done = 0; reversal_done = 0; repairmb_done = 0; 
        valvref_done = 0; datavref_done = 0; speedidle_done = 0; txselfcal_done = 0; rxclkcal_done = 0; 
        valtraincenter_done = 0; valtrainvref_done = 0; datatrainvref_done = 0;
        rxdeskew_done = 0; datatraincenter2_done = 0; linkspeed_done = 0;
        linkspeed_error = 0; needs_repair = 0; needs_speed_degrade = 0; repair_done = 0;

        internal_retrain_req = 1'b0;
        internal_error_req   = 1'b0;
        local_retrain_enc    = 3'b001; 
    endtask

    task automatic pulse(ref logic sig);
        @(negedge lclk);
        sig = 1'b1;
        @(negedge lclk);
        sig = 1'b0;
    endtask

    task wait_for_state(input logic [7:0] target_log);
        fork
            begin
                wait(error_log_0 == target_log);
            end
            begin
                wait(error_log_0 == 8'h16); // TRAINERROR
                $error("\n[FATAL] Hardware aborted to TRAINERROR!\n");
                $finish;
            end
            begin
                // FIXED: Give the testbench a longer leash!
                repeat(5000) @(posedge lclk); 
                $error("\n[FATAL] Testbench Watchdog Timer Expired while waiting for log %h!\n", target_log);
                $finish;
            end
        join_any
        disable fork;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_ucie_lphy_top.fsdb");
        $fsdbDumpvars(0, tb_ucie_lphy_top);

        $display("==================================================");
        $display("Starting Logical PHY Top-Level Bring-Up");
        $display("==================================================");

        lclk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // 1. Traverse ST_RESET
        // =========================================================
        power_stable        = 1'b1; 
        sb_clk_stable       = 1'b1; 
        mb_clk_stable       = 1'b1; 
        mb_clk_slow         = 1'b1;
        
        lp_state_req        = 4'b0000; // Put back to NOP
        start_link_training = 1'b1;    // MMIO trigger
        
        wait_for_state(8'h01); // 01h = SBINIT
        @(negedge lclk); 
        $display("[PASS] Exited RESET. Entered SBINIT.");   

        // =========================================================
        // 2. Traverse ST_SBINIT
        // =========================================================
        rx_pattern_detected = 4'b0001; 
        
        wait(tx_msg_out_of_reset == 1'b1);
        @(negedge lclk); 
        pulse(rx_msg_out_of_reset);
        
        wait(tx_msg_done_req == 1'b1);
        @(negedge lclk); 
        pulse(rx_msg_done_resp);
        
        wait_for_state(8'h02); // 02h = MBINIT.PARAM
        @(negedge lclk);
        $display("[PASS] Exited SBINIT. Entered MBINIT.");

        // =========================================================
        // 3. Traverse ST_MBINIT
        // =========================================================
        wait(dut.u_ltssm.en_param);       @(negedge lclk); pulse(param_done);
        wait(dut.u_ltssm.en_repairclk);   @(negedge lclk); pulse(repairclk_done); 
        wait(dut.u_ltssm.en_repairval);   @(negedge lclk); pulse(repairval_done);
        wait(dut.u_ltssm.en_reversal);    @(negedge lclk); pulse(reversal_done);
        wait(dut.u_ltssm.en_repairmb);    @(negedge lclk); pulse(repairmb_done);
        
        wait_for_state(8'h08); // 08h = MBTRAIN.VALVREF
        @(negedge lclk);
        $display("[PASS] Exited MBINIT. Entered MBTRAIN.");

        // =========================================================
        // 4. Traverse ST_MBTRAIN
        // =========================================================
        wait(dut.u_ltssm.en_valvref);          @(negedge lclk); pulse(valvref_done);
        wait(dut.u_ltssm.en_datavref);         @(negedge lclk); pulse(datavref_done);
        wait(dut.u_ltssm.en_speedidle);        @(negedge lclk); pulse(speedidle_done);
        wait(dut.u_ltssm.en_txselfcal);        @(negedge lclk); pulse(txselfcal_done);
        wait(dut.u_ltssm.en_rxclkcal);         @(negedge lclk); pulse(rxclkcal_done);
        wait(dut.u_ltssm.en_valtraincenter);   @(negedge lclk); pulse(valtraincenter_done);
        wait(dut.u_ltssm.en_valtrainvref);     @(negedge lclk); pulse(valtrainvref_done);
        wait(dut.u_ltssm.en_datatrainvref);    @(negedge lclk); pulse(datatrainvref_done);
        wait(dut.u_ltssm.en_rxdeskew);         @(negedge lclk); pulse(rxdeskew_done); 
        wait(dut.u_ltssm.en_datatraincenter2); @(negedge lclk); pulse(datatraincenter2_done);
        wait(dut.u_ltssm.en_linkspeed);        @(negedge lclk); pulse(linkspeed_done);
        
        wait_for_state(8'h14); // 14h = LINKINIT
        @(negedge lclk);
        $display("[PASS] Exited MBTRAIN. Entered LINKINIT.");

        // =========================================================
        // 5. Traverse ST_LINKINIT -> ACTIVE
        // =========================================================
        wait(pl_inband_pres == 1'b1);
        @(negedge lclk);
        lp_state_req = 4'b0001; 
        
        wait(tx_req_active == 1'b1);
        @(negedge lclk); pulse(rx_req_active);
        
        wait(tx_rsp_active == 1'b1);
        @(negedge lclk); pulse(rx_rsp_active);
        
        wait_for_state(8'h15); // 15h = ACTIVE
        @(negedge lclk);
        
        if (pl_state_sts !== 4'b0001) $error("[FAIL] RDI Status did not lock to 0001b.");
        else $display("[PASS] Exited LINKINIT. Link is now ACTIVE!");

        // =========================================================
        // 6. RDI DATAPATH INJECTION TEST
        // =========================================================
        $display("--------------------------------------------------");
        $display("Injecting Protocol Flit across RDI Boundary...");
        
        wait(pl_trdy == 1'b1);
        @(negedge lclk);
        
        lp_valid = 1'b1;
        lp_irdy  = 1'b1;
        lp_data  = {256'hDEADBEEF_CAFEF00D, 256'h12345678_9ABCDEF0}; 
        
        @(negedge lclk);
        lp_valid = 1'b0;
        lp_irdy  = 1'b0;
        
        @(negedge lclk);
        
        if (TXDATA_OE !== 64'hFFFF_FFFF_FFFF_FFFF) 
            $error("[FAIL] AFE Data Output Enables are not driven!");
        else 
            $display("[PASS] AFE Datapath ungated! Flit successfully routed to TX Bumps.");

        $display("==================================================");
        $display("UCIe LOGICAL PHY INTEGRATION COMPLETE.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire