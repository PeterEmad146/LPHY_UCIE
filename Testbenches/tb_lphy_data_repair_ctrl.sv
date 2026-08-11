`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_data_repair_ctrl;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        package_type;
    logic [63:0] lane_failed;
    logic        check_done;

    wire [7:0]   trd_repair_addr [3:0];
    wire [1:0]   lane_map;
    wire         is_unrepairable;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_data_repair_ctrl dut (
        .clk(clk),
        .rst_n(rst_n),
        .package_type(package_type),
        .lane_failed(lane_failed),
        .check_done(check_done),
        .trd_repair_addr(trd_repair_addr),
        .lane_map(lane_map),
        .is_unrepairable(is_unrepairable)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task trigger_check();
        @(negedge clk);
        check_done = 1'b1;
        @(negedge clk);
        check_done = 1'b0;
    endtask

    task clear_faults();
        lane_failed = 64'h0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_data_repair_ctrl.fsdb");
        $fsdbDumpvars(0, tb_lphy_data_repair_ctrl);

        $display("==================================================");
        $display("Starting Data Repair Controller Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        package_type = 1'b0; // Start with Advanced Package
        check_done = 0;
        clear_faults();

        #15 rst_n = 1;

        // =========================================================
        // TC1: Advanced Package - Perfect Link
        // =========================================================
        $display("Running TC1: Adv Package - Clean Link");
        trigger_check();
        
        if (lane_map !== 2'b11 || is_unrepairable !== 1'b0 || trd_repair_addr[0] !== 8'hFF)
            $error("[FAIL] TC1: Clean link incorrectly altered mapping.");
        else
            $display("[PASS] TC1: Clean link maintained x64 width with no redundant mapping.");

        // =========================================================
        // TC2: Advanced Package - Successful Redundancy Repair
        // =========================================================
        $display("Running TC2: Adv Package - Repairable Faults (Lanes 5, 20, 50)");
        clear_faults();
        // 2 in lower, 1 in upper
        lane_failed[5]  = 1'b1; 
        lane_failed[20] = 1'b1;
        lane_failed[50] = 1'b1;
        trigger_check();
        
        if (lane_map !== 2'b11 || is_unrepairable !== 1'b0)
            $error("[FAIL] TC2: Degraded width despite having enough redundant pins.");
            
        if (trd_repair_addr[0] !== 8'd5 || trd_repair_addr[1] !== 8'd20 || trd_repair_addr[2] !== 8'd50)
            $error("[FAIL] TC2: Redundant pin addresses mapped incorrectly.");
        else
            $display("[PASS] TC2: Hardware mapped physical faults to redundant sideband addresses perfectly.");

        // =========================================================
        // TC3: Advanced Package - Half-Width Degradation
        // =========================================================
        $display("Running TC3: Adv Package - Lower Half Fatal (Degrade to Upper x32)");
        clear_faults();
        // 3 in lower (fatal for lower), 0 in upper
        lane_failed[1] = 1'b1; lane_failed[2] = 1'b1; lane_failed[3] = 1'b1;
        trigger_check();
        
        if (lane_map !== 2'b10 || is_unrepairable !== 1'b0)
            $error("[FAIL] TC3: Failed to trigger x32 Upper-Half degradation.");
        else
            $display("[PASS] TC3: Hardware successfully degraded link width to survive local fatalities.");

        // =========================================================
        // TC4: Standard Package - Half-Width Degradation
        // =========================================================
        $display("Running TC4: Std Package - Upper Half Fatal (Degrade to Lower x8)");
        package_type = 1'b1; // Switch to Standard
        clear_faults();
        // Standard has no redundancy. Any fault is fatal to that half.
        lane_failed[12] = 1'b1; // Fault in upper 8 lanes
        trigger_check();
        
        if (lane_map !== 2'b01 || is_unrepairable !== 1'b0)
            $error("[FAIL] TC4: Standard Package failed to degrade to Lower x8.");
        else
            $display("[PASS] TC4: Standard Package bypassed redundancy and correctly degraded width.");

        // =========================================================
        // TC5: Fatal (Unrepairable) Check
        // =========================================================
        $display("Running TC5: Fatal Damage Check");
        clear_faults();
        // Both halves damaged on Standard Package
        lane_failed[2] = 1'b1;  // Lower fault
        lane_failed[10] = 1'b1; // Upper fault
        trigger_check();
        
        if (!is_unrepairable || lane_map !== 2'b00)
            $error("[FAIL] TC5: Link failed to declare itself unrepairable.");
        else
            $display("[PASS] TC5: Fatal damage successfully triggered link failure state.");

        $display("==================================================");
        $display("Data Repair Controller Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire