`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_rdi_intf;

    // ---------------------------------------------------------
    // Parameters & Clocks
    // ---------------------------------------------------------
    localparam int NBYTES = 16;
    localparam int NC     = 32;

    logic lclk;
    logic rst_n;
    
    always #5 lclk = ~lclk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals (Adapter Side - Formal RDI)
    // ---------------------------------------------------------
    logic [3:0]  lp_state_req;
    wire  [3:0]  pl_state_sts;
    wire         pl_inband_pres;
    
    logic        lp_linkerror;
    wire         pl_error;
    wire         pl_cerror;
    wire         pl_nferror;
    wire         pl_trainerror;
    wire         pl_phyinrecenter;

    wire         pl_stallreq;
    logic        lp_stallack;

    wire         pl_clk_req;
    logic        lp_clk_ack;
    logic        lp_wake_req;
    wire         pl_wake_ack;

    wire  [2:0]  pl_speedmode;
    wire  [2:0]  pl_lnk_cfg;

    logic [(NBYTES*8)-1:0] lp_data;
    logic        lp_valid;
    logic        lp_irdy;
    wire         pl_trdy;
    logic        lp_retimer_crd;

    wire  [(NBYTES*8)-1:0] pl_data;
    wire         pl_valid;
    wire         pl_retimer_crd;

    logic [NC-1:0] lp_cfg;
    logic        lp_cfg_vld;
    wire         pl_cfg_crd;

    wire  [NC-1:0] pl_cfg;
    wire         pl_cfg_vld;
    logic        lp_cfg_crd;

    // ---------------------------------------------------------
    // DUT Signals (Internal PHY Side)
    // ---------------------------------------------------------
    logic [3:0]  internal_pl_state_sts;
    logic        internal_pl_inband_pres;
    wire  [3:0]  internal_lp_state_req;
    wire         internal_lp_linkerror;
    wire         internal_start_link_training;
    
    logic        internal_stallreq;
    wire         internal_stallack;
    logic        internal_phyinrecenter;
    
    logic        internal_error;
    logic        internal_cerror;
    logic        internal_nferror;
    logic        internal_trainerror;
    logic [2:0]  internal_speedmode;
    logic [2:0]  internal_lnk_cfg;

    wire  [(NBYTES*8)-1:0] internal_lp_data;
    wire         internal_lp_valid;
    wire         internal_lp_irdy;
    logic        internal_pl_trdy;
    wire         internal_lp_retimer_crd;
    
    logic [(NBYTES*8)-1:0] internal_pl_data;
    logic        internal_pl_valid;
    logic        internal_pl_retimer_crd;

    wire  [NC-1:0] internal_lp_cfg;
    wire         internal_lp_cfg_vld;
    logic        internal_pl_cfg_crd;
    
    logic [NC-1:0] internal_pl_cfg;
    logic        internal_pl_cfg_vld;
    wire         internal_lp_cfg_crd;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_rdi_intf #(
        .NBYTES(NBYTES),
        .NC(NC)
    ) dut (
        .lclk(lclk), .rst_n(rst_n),
        
        // Formal RDI
        .lp_state_req(lp_state_req), .pl_state_sts(pl_state_sts), .pl_inband_pres(pl_inband_pres),
        .lp_linkerror(lp_linkerror), .pl_error(pl_error), .pl_cerror(pl_cerror),
        .pl_nferror(pl_nferror), .pl_trainerror(pl_trainerror), .pl_phyinrecenter(pl_phyinrecenter),
        .pl_stallreq(pl_stallreq), .lp_stallack(lp_stallack),
        .pl_clk_req(pl_clk_req), .lp_clk_ack(lp_clk_ack), .lp_wake_req(lp_wake_req), .pl_wake_ack(pl_wake_ack),
        .pl_speedmode(pl_speedmode), .pl_lnk_cfg(pl_lnk_cfg),
        
        .lp_data(lp_data), .lp_valid(lp_valid), .lp_irdy(lp_irdy),
        .pl_trdy(pl_trdy), .lp_retimer_crd(lp_retimer_crd),
        .pl_data(pl_data), .pl_valid(pl_valid), .pl_retimer_crd(pl_retimer_crd),
        
        .lp_cfg(lp_cfg), .lp_cfg_vld(lp_cfg_vld), .pl_cfg_crd(pl_cfg_crd),
        .pl_cfg(pl_cfg), .pl_cfg_vld(pl_cfg_vld), .lp_cfg_crd(lp_cfg_crd),
        
        // Internal PHY
        .internal_pl_state_sts(internal_pl_state_sts), .internal_pl_inband_pres(internal_pl_inband_pres),
        .internal_lp_state_req(internal_lp_state_req), .internal_lp_linkerror(internal_lp_linkerror),
        .internal_start_link_training(internal_start_link_training),
        .internal_stallreq(internal_stallreq), .internal_stallack(internal_stallack),
        .internal_phyinrecenter(internal_phyinrecenter),
        .internal_error(internal_error), .internal_cerror(internal_cerror),
        .internal_nferror(internal_nferror), .internal_trainerror(internal_trainerror),
        .internal_speedmode(internal_speedmode), .internal_lnk_cfg(internal_lnk_cfg),
        
        .internal_lp_data(internal_lp_data), .internal_lp_valid(internal_lp_valid),
        .internal_lp_irdy(internal_lp_irdy), .internal_pl_trdy(internal_pl_trdy),
        .internal_lp_retimer_crd(internal_lp_retimer_crd),
        
        .internal_pl_data(internal_pl_data), .internal_pl_valid(internal_pl_valid),
        .internal_pl_retimer_crd(internal_pl_retimer_crd),
        
        .internal_lp_cfg(internal_lp_cfg), .internal_lp_cfg_vld(internal_lp_cfg_vld),
        .internal_pl_cfg_crd(internal_pl_cfg_crd),
        .internal_pl_cfg(internal_pl_cfg), .internal_pl_cfg_vld(internal_pl_cfg_vld),
        .internal_lp_cfg_crd(internal_lp_cfg_crd)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_all_inputs();
        lp_state_req = 4'h0; lp_linkerror = 1'b0; lp_stallack = 1'b0; lp_clk_ack = 1'b0; lp_wake_req = 1'b0;
        lp_data = 128'h0; lp_valid = 1'b0; lp_irdy = 1'b0; lp_retimer_crd = 1'b0;
        lp_cfg = 32'h0; lp_cfg_vld = 1'b0; lp_cfg_crd = 1'b0;
        
        internal_pl_state_sts = 4'h0; internal_pl_inband_pres = 1'b0; internal_stallreq = 1'b0;
        internal_phyinrecenter = 1'b0; internal_error = 1'b0; internal_cerror = 1'b0;
        internal_nferror = 1'b0; internal_trainerror = 1'b0; internal_speedmode = 3'h0; internal_lnk_cfg = 3'h0;
        internal_pl_trdy = 1'b0; internal_pl_data = 128'h0; internal_pl_valid = 1'b0;
        internal_pl_retimer_crd = 1'b0; internal_pl_cfg_crd = 1'b0;
        internal_pl_cfg = 32'h0; internal_pl_cfg_vld = 1'b0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_rdi_intf.fsdb");
        $fsdbDumpvars(0, tb_lphy_rdi_intf);

        $display("==================================================");
        $display("Starting RDI Formal Interface Verification");
        $display("==================================================");

        lclk = 0; rst_n = 0;
        clear_all_inputs();

        #15 rst_n = 1;
        @(negedge lclk);

        // =========================================================
        // TC1: Mainband & Sideband Combinational Passthrough
        // =========================================================
        $display("Running TC1: Datapath Routing Validation");
        lp_data = 128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_1111_2222;
        lp_valid = 1'b1;
        lp_cfg = 32'hDEADBEEF;
        internal_pl_trdy = 1'b1; // PHY is ready
        
        #1; // Yield for combinational assignments
        
        if (internal_lp_data !== 128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_1111_2222 || 
            internal_lp_cfg !== 32'hDEADBEEF || pl_trdy !== 1'b1)
            $error("[FAIL] TC1: Combinational datapath routing failed.");
        else
            $display("[PASS] TC1: 128-bit Mainband & 32-bit Sideband routed perfectly.");

        // =========================================================
        // TC2: Link Training Trigger (SBINIT Wakeup)
        // =========================================================
        $display("Running TC2: Link Training Master Trigger");
        clear_all_inputs();
        @(negedge lclk);
        
        internal_pl_state_sts = 4'b0000; // PHY is in Reset
        lp_state_req = 4'b0001;          // Adapter requests Active
        
        #1; 
        if (internal_start_link_training !== 1'b1)
            $error("[FAIL] TC2: Link Training trigger failed to assert.");
        else
            $display("[PASS] TC2: SBINIT Master Trigger logic synthesized perfectly.");

        // =========================================================
        // TC3: Clock Gating Handshake (2-Flop Staging)
        // =========================================================
        $display("Running TC3: Clock Gating Wake Handshake");
        clear_all_inputs();
        @(negedge lclk);
        
        lp_wake_req = 1'b1; // Adapter requests to wake PHY
        
        @(negedge lclk);
        if (pl_wake_ack === 1'b1) $error("[FAIL] TC3: Wake Ack asserted too early (Missing Flop 1).");
        
        @(negedge lclk);
        if (pl_wake_ack !== 1'b1) $error("[FAIL] TC3: Wake Ack failed to assert after 2 clock cycles.");
        else $display("[PASS] TC3: Clock Gating Handshake correctly staged through 2 flops.");

        // =========================================================
        // TC4: Stall Handshake Isolation (Rule 8 Compliance)
        // =========================================================
        $display("Running TC4: Stall Handshake Flop Isolation");
        clear_all_inputs();
        @(negedge lclk);
        
        internal_stallreq = 1'b1; // PHY initiates stall
        lp_stallack = 1'b1;       // Adapter acknowledges
        
        #1; // Check combinational output immediately
        if (pl_stallreq === 1'b1 || internal_stallack === 1'b1) 
            $error("[FAIL] TC4: Stall signals are combinational! Rule 8 violated.");
            
        @(negedge lclk); // Wait for clock edge
        if (pl_stallreq !== 1'b1 || internal_stallack !== 1'b1)
            $error("[FAIL] TC4: Stall signals failed to propagate through isolation flops.");
        else
            $display("[PASS] TC4: Stall Handshake is strictly isolated by clock flops.");

        $display("==================================================");
        $display("RDI Interface Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire