`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_sbinit;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int TIMEOUT_CYCLES = 100; // Drastically lowered for testing
    
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       en_sbinit;
    logic       package_type;
    logic [3:0] rx_pattern_detected;
    logic       rx_msg_out_of_reset;
    logic       rx_msg_done_req;
    logic       rx_msg_done_resp;
    
    wire        tx_send_pattern;
    wire        tx_msg_out_of_reset;
    wire        tx_msg_done_req;
    wire        tx_msg_done_resp;
    wire [2:0]  sb_repair_sel;
    wire [3:0]  pl_state_sts;
    wire        pl_inband_pres;
    wire        pl_protocol_vld;
    wire        exit_to_mbinit;
    wire        exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_sbinit #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .en_sbinit(en_sbinit), .package_type(package_type),
        .rx_pattern_detected(rx_pattern_detected), 
        .rx_msg_out_of_reset(rx_msg_out_of_reset), 
        .rx_msg_done_req(rx_msg_done_req), 
        .rx_msg_done_resp(rx_msg_done_resp),
        .tx_send_pattern(tx_send_pattern), 
        .tx_msg_out_of_reset(tx_msg_out_of_reset), 
        .tx_msg_done_req(tx_msg_done_req), 
        .tx_msg_done_resp(tx_msg_done_resp), 
        .sb_repair_sel(sb_repair_sel),
        .pl_state_sts(pl_state_sts), .pl_inband_pres(pl_inband_pres), .pl_protocol_vld(pl_protocol_vld),
        .exit_to_mbinit(exit_to_mbinit), .exit_to_trainerror(exit_to_trainerror)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        en_sbinit = 0; package_type = 0; // Adv Package
        rx_pattern_detected = 0; rx_msg_out_of_reset = 0;
        rx_msg_done_req = 0; rx_msg_done_resp = 0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_ltssm_sbinit.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_sbinit);

        $display("==================================================");
        $display("Starting LTSSM SBINIT State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        clear_inputs();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Perfect Handshake (Advanced Package)
        // =========================================================
        $display("Running TC1: Ideal Handshake execution");
        
        @(negedge clk);
        en_sbinit = 1'b1;
        
        // Wait in pattern state
        repeat(5) @(negedge clk);
        if (!tx_send_pattern) $error("[FAIL] TC1: Failed to drive Training Pattern.");
        
        // Remote detects redundant pattern (Combo 2: DATASBRD / CKSB)
        rx_pattern_detected = 4'b0100;
        @(negedge clk);
        
        // State Machine will now wait 48 cycles (Turnaround mechanism)
        repeat(50) @(negedge clk);
        
        if (!tx_msg_out_of_reset) $error("[FAIL] TC1: Failed to send Out-of-Reset Message.");
        if (sb_repair_sel !== 3'b010) $error("[FAIL] TC1: Priority encoder failed to map redundant combination 2.");
        
        // Remote responds to OOR
        rx_msg_out_of_reset = 1'b1;
        @(negedge clk);
        rx_msg_out_of_reset = 1'b0;
        
        // FIX: Check immediately! tx_msg_done_req is a 1-cycle pulse.
        // We are currently sitting on the exact negedge where the pulse is active.
        if (!tx_msg_done_req) $error("[FAIL] TC1: Failed to initiate Done Request.");
        
        // Move forward 1 cycle so the RTL can enter ST_WAIT_DONE
        @(negedge clk);
        
        // Remote responds to Done
        rx_msg_done_resp = 1'b1;
        @(negedge clk);
        rx_msg_done_resp = 1'b0;
        
        // Wait 1 cycle for RTL to enter ST_DONE and assert exit
        @(negedge clk);
        if (!exit_to_mbinit) $error("[FAIL] TC1: Failed to transition to MBINIT after handshakes.");
        else $display("[PASS] TC1: Handshake sequence and redundant mapping perfectly executed.");

        en_sbinit = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Timeout Escalation
        // =========================================================
        $display("Running TC2: Timeout Escalation (No Pattern Lock)");
        
        clear_inputs();
        en_sbinit = 1'b1;
        
        // Just let it run without responding...
        repeat(TIMEOUT_CYCLES + 5) @(negedge clk);
        
        if (!exit_to_trainerror) $error("[FAIL] TC2: Failed to escalate to TRAINERROR after timeout.");
        else $display("[PASS] TC2: Timeout correctly escalated to TRAINERROR.");

        $display("==================================================");
        $display("LTSSM SBINIT State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire