`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_mbinit;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int TIMEOUT_CYCLES = 20; // Lowered for rapid testing
    
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       en_mbinit;
    logic       package_type;
    
    logic       param_done, cal_done, repairclk_done;
    logic       repairval_done, reversal_done, repairmb_done;
    logic       substate_error;
    
    wire        en_param, en_cal, en_repairclk;
    wire        en_repairval, en_reversal, en_repairmb;
    wire [3:0]  pl_state_sts;
    wire        pl_inband_pres;
    wire [7:0]  mbinit_substate_log;
    wire        exit_to_mbtrain;
    wire        exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_mbinit #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .en_mbinit(en_mbinit), .package_type(package_type),
        .param_done(param_done), .cal_done(cal_done), .repairclk_done(repairclk_done),
        .repairval_done(repairval_done), .reversal_done(reversal_done), .repairmb_done(repairmb_done),
        .substate_error(substate_error),
        .en_param(en_param), .en_cal(en_cal), .en_repairclk(en_repairclk),
        .en_repairval(en_repairval), .en_reversal(en_reversal), .en_repairmb(en_repairmb),
        .pl_state_sts(pl_state_sts), .pl_inband_pres(pl_inband_pres),
        .mbinit_substate_log(mbinit_substate_log),
        .exit_to_mbtrain(exit_to_mbtrain), .exit_to_trainerror(exit_to_trainerror)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_flags();
        param_done = 0; cal_done = 0; repairclk_done = 0;
        repairval_done = 0; reversal_done = 0; repairmb_done = 0;
        substate_error = 0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_ltssm_mbinit.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_mbinit);

        $display("==================================================");
        $display("Starting LTSSM MBINIT State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        en_mbinit = 0; package_type = 0;
        clear_flags();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Perfect Sub-state Sequence
        // =========================================================
        $display("Running TC1: Sequential Sub-state Handshakes");
        
        @(negedge clk); en_mbinit = 1'b1;
        
        @(negedge clk); if(!en_param) $error("[FAIL] TC1: Failed to enter PARAM.");
        param_done = 1'b1; @(negedge clk); param_done = 1'b0;
        
        @(negedge clk); if(!en_cal) $error("[FAIL] TC1: Failed to enter CAL.");
        cal_done = 1'b1; @(negedge clk); cal_done = 1'b0;
        
        @(negedge clk); if(!en_repairclk) $error("[FAIL] TC1: Failed to enter REPAIRCLK.");
        // Verify register logging correctly outputs spec encoding 04h!
        if(mbinit_substate_log !== 8'h04) $error("[FAIL] TC1: Logging failed. Got %h", mbinit_substate_log);
        repairclk_done = 1'b1; @(negedge clk); repairclk_done = 1'b0;
        
        @(negedge clk); if(!en_repairval) $error("[FAIL] TC1: Failed to enter REPAIRVAL.");
        repairval_done = 1'b1; @(negedge clk); repairval_done = 1'b0;
        
        @(negedge clk); if(!en_reversal) $error("[FAIL] TC1: Failed to enter REVERSALMB.");
        reversal_done = 1'b1; @(negedge clk); reversal_done = 1'b0;
        
        @(negedge clk); if(!en_repairmb) $error("[FAIL] TC1: Failed to enter REPAIRMB.");
        repairmb_done = 1'b1; @(negedge clk); repairmb_done = 1'b0;
        
        // Wait 1 cycle for FSM to enter DONE
        @(negedge clk);
        if (!exit_to_mbtrain) $error("[FAIL] TC1: Failed to exit to MBTRAIN.");
        else $display("[PASS] TC1: MBINIT smoothly traversed all sub-states and exited.");

        en_mbinit = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Timeout Escalation (Per Sub-State Check)
        // =========================================================
        $display("Running TC2: 8ms Sub-State Timeout Check");
        
        clear_flags();
        en_mbinit = 1'b1;
        
        @(negedge clk);
        param_done = 1'b1; @(negedge clk); param_done = 1'b0;
        
        // FSM is now stuck in ST_CAL. We wait until timeout.
        repeat(TIMEOUT_CYCLES + 3) @(negedge clk);
        
        if (!exit_to_trainerror) $error("[FAIL] TC2: Failed to escalate timeout in ST_CAL.");
        else $display("[PASS] TC2: Timeout correctly triggered TRAINERROR transition.");

        en_mbinit = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC3: Immediate Sub-state Error
        // =========================================================
        $display("Running TC3: Sub-module Fatal Error Flagging");
        
        clear_flags();
        en_mbinit = 1'b1;
        
        @(negedge clk);
        param_done = 1'b1; @(negedge clk); param_done = 1'b0;
        
        @(negedge clk);
        
        // Simulate ST_CAL hardware discovering an unrepairable short
        substate_error = 1'b1; 
        @(negedge clk);
        
        if (!exit_to_trainerror) $error("[FAIL] TC3: Fatal sub-state error was ignored.");
        else $display("[PASS] TC3: Immediate hardware error correctly bypassed timeout and aborted sequence.");

        $display("==================================================");
        $display("LTSSM MBINIT State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire