`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_ltssm_mbtrain;

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
    logic       en_mbtrain, package_type;
    logic       valvref_done, datavref_done, speedidle_done;
    logic       txselfcal_done, rxclkcal_done, valtraincenter_done;
    logic       valtrainvref_done, datatraincenter1_done, datatrainvref_done;
    logic       rxdeskew_done, datatraincenter2_done;
    
    logic       linkspeed_done, linkspeed_error;
    logic       needs_repair, needs_speed_degrade;
    logic       repair_done, substate_error;
    
    wire        en_valvref, en_datavref, en_speedidle, en_txselfcal;
    wire        en_rxclkcal, en_valtraincenter, en_valtrainvref;
    wire        en_datatraincenter1, en_datatrainvref, en_rxdeskew;
    wire        en_datatraincenter2, en_linkspeed, en_repair;
    
    wire [3:0]  pl_state_sts;
    wire        pl_inband_pres;
    wire [7:0]  mbtrain_substate_log;
    wire        exit_to_linkinit;
    wire        exit_to_trainerror;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_ltssm_mbtrain #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) dut (.*); // Using SV implicit port connection for brevity

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_flags();
        valvref_done = 0; datavref_done = 0; speedidle_done = 0;
        txselfcal_done = 0; rxclkcal_done = 0; valtraincenter_done = 0;
        valtrainvref_done = 0; datatraincenter1_done = 0; datatrainvref_done = 0;
        rxdeskew_done = 0; datatraincenter2_done = 0;
        linkspeed_done = 0; linkspeed_error = 0; needs_repair = 0; needs_speed_degrade = 0;
        repair_done = 0; substate_error = 0;
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
        $fsdbDumpfile("tb_lphy_ltssm_mbtrain.fsdb");
        $fsdbDumpvars(0, tb_lphy_ltssm_mbtrain);

        $display("==================================================");
        $display("Starting LTSSM MBTRAIN State Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        en_mbtrain = 0; package_type = 0; // Advanced
        clear_flags();
        #15 rst_n = 1;

        // =========================================================
        // TC1: Perfect Bring-up (Advanced Package)
        // =========================================================
        $display("Running TC1: Sequential Training (Advanced Package)");
        
        @(negedge clk); en_mbtrain = 1'b1;
        
        @(negedge clk); pulse(valvref_done);
        @(negedge clk); pulse(datavref_done);
        @(negedge clk); pulse(speedidle_done);
        @(negedge clk); pulse(txselfcal_done);
        @(negedge clk); pulse(rxclkcal_done);
        @(negedge clk); pulse(valtraincenter_done);
        @(negedge clk); pulse(valtrainvref_done);
        @(negedge clk); pulse(datatraincenter1_done);
        @(negedge clk); pulse(datatrainvref_done);
        @(negedge clk); pulse(rxdeskew_done);
        @(negedge clk); pulse(datatraincenter2_done);
        
        @(negedge clk); 
        if (!en_linkspeed) $error("[FAIL] TC1: Failed to reach LINKSPEED test.");
        
        pulse(linkspeed_done);
        
        @(negedge clk);
        if (!exit_to_linkinit) $error("[FAIL] TC1: Failed to exit to LINKINIT.");
        else $display("[PASS] TC1: Perfect training sequence completed.");

        en_mbtrain = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC2: Standard Package Bypass Test
        // =========================================================
        $display("Running TC2: Standard Package Bypass Sequence");
        clear_flags();
        en_mbtrain = 1'b1;
        package_type = 1'b1; // Switch to Standard!
        
        @(negedge clk);
        if (!en_speedidle) $error("[FAIL] TC2: Standard pkg failed to skip initial VREF states.");
        
        pulse(speedidle_done);
        pulse(txselfcal_done);
        pulse(rxclkcal_done);
        pulse(valtraincenter_done);
        
        @(negedge clk);
        if (!en_datatraincenter1) $error("[FAIL] TC2: Standard pkg failed to skip High-Speed VREF states.");
        else $display("[PASS] TC2: Standard package correctly bypassed all VREF phases.");

        en_mbtrain = 1'b0;
        repeat(3) @(negedge clk);

        // =========================================================
        // TC3: Degradation Loop (Link Speed Failure & Recovery)
        // =========================================================
        $display("Running TC3: Link Failure and Loop-Back Recovery");
        clear_flags();
        en_mbtrain = 1'b1;
        package_type = 1'b0; 
        
        // Fast-forward to LINKSPEED...
        pulse(valvref_done); pulse(datavref_done); pulse(speedidle_done); pulse(txselfcal_done);
        pulse(rxclkcal_done); pulse(valtraincenter_done); pulse(valtrainvref_done); pulse(datatraincenter1_done);
        pulse(datatrainvref_done); pulse(rxdeskew_done); pulse(datatraincenter2_done);
        
        @(negedge clk);
        // INJECT ERROR: Linkspeed point test fails, but repair is possible!
        linkspeed_error = 1'b1;
        needs_repair = 1'b1;
        @(negedge clk);
        
        if (!en_repair) $error("[FAIL] TC3: FSM failed to enter REPAIR state on link error.");
        
        linkspeed_error = 1'b0;
        needs_repair = 1'b0;
        pulse(repair_done);
        
        @(negedge clk);
        if (!en_txselfcal) $error("[FAIL] TC3: FSM failed to loop back to TXSELFCAL after repair.");
        else $display("[PASS] TC3: FSM successfully invoked repair and looped back to recalibrate.");

        $display("==================================================");
        $display("LTSSM MBTRAIN State Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire