`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_repair_rx;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [7:0] rx_physical_data [63:0];
    logic [7:0] rx_redundant_data [3:0];
    logic [63:0] lane_failed;

    wire  [7:0] rx_logical_data [63:0];
    wire        unrepairable;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_repair_rx dut (
        .rx_physical_data(rx_physical_data),
        .rx_redundant_data(rx_redundant_data),
        .lane_failed(lane_failed),
        .rx_logical_data(rx_logical_data),
        .unrepairable(unrepairable)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task load_trace_data();
        for (int i = 0; i < 64; i++) begin
            rx_physical_data[i] = i[7:0]; // Physical Lane 'i' carries marker 'i'
        end
        // Load recognizable hex markers on the redundant analog bumps
        rx_redundant_data[0] = 8'hA0;
        rx_redundant_data[1] = 8'hA1;
        rx_redundant_data[2] = 8'hA2;
        rx_redundant_data[3] = 8'hA3;
    endtask

    task clear_faults();
        lane_failed = 64'h0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_repair_rx.fsdb");
        $fsdbDumpvars(0, tb_lphy_repair_rx);

        $display("==================================================");
        $display("Starting RX Redundancy Demultiplexer Verification");
        $display("==================================================");

        // Initialize Datapath
        load_trace_data();
        clear_faults();
        
        #10; // Yield for combinatorial stabilization

        // =========================================================
        // TC1: Clean Link (No Failures)
        // =========================================================
        $display("Running TC1: Clean Link (1:1 Mapping)");
        
        for (int i = 0; i < 64; i++) begin
            if (rx_logical_data[i] !== i[7:0]) $error("[FAIL] TC1: Mismatch on Lane %0d", i);
        end
        if (unrepairable) $error("[FAIL] TC1: Unrepairable flag asserted falsely.");
        else $display("[PASS] TC1: Clean link perfectly routed 1:1.");

        // =========================================================
        // TC2: Single Failure - Group 1 Lower (Shift Left Undo)
        // =========================================================
        $display("Running TC2: Single Failure at Physical Lane 5");
        
        clear_faults();
        lane_failed[5] = 1'b1; // Inject physical failure
        
        #10; 
        
        // Logical Lane 0 must pull from Redundant Bump 0 (A0)
        if (rx_logical_data[0] !== 8'hA0)
            $error("[FAIL] TC2: Logical Lane 0 failed to pull from Redundant Bump 0.");
            
        // Logical Lanes 1-5 must pull from Physical Lanes 0-4 (Inverse of Shift Left)
        for (int i = 1; i <= 5; i++) begin
            if (rx_logical_data[i] !== (i - 1)) $error("[FAIL] TC2: Shift-Left Undo failed at Logical Lane %0d", i);
        end
        
        // Logical Lanes 6-31 must remain 1:1 mapped to Physical Lanes 6-31
        for (int i = 6; i < 32; i++) begin
            if (rx_logical_data[i] !== i) $error("[FAIL] TC2: 1:1 Mapping corrupted above failure at Lane %0d", i);
        end
        $display("[PASS] TC2: Single failure correctly reconstructed logical datapath.");

        // =========================================================
        // TC3: Double Failure - Group 2 Upper (Split Shift Undo)
        // =========================================================
        $display("Running TC3: Double Failure at Physical Lane 40 and 50");
        
        clear_faults();
        lane_failed[40] = 1'b1;
        lane_failed[50] = 1'b1;
        
        #10;
        
        // Check Redundant Bumps
        if (rx_logical_data[32] !== 8'hA2 || rx_logical_data[63] !== 8'hA3)
            $error("[FAIL] TC3: Logical boundary lanes failed to pull from Redundant Bumps 2 and 3.");
            
        // Shift Left (< f0): Logical 33-40 pull from Physical 32-39
        for (int i = 33; i <= 40; i++) begin
            if (rx_logical_data[i] !== (i - 1)) $error("[FAIL] TC3: Shift-Left Undo failed at Logical Lane %0d", i);
        end
        
        // Middle Pass-through (> f0 and < f1): Logical 41-49 pull from Physical 41-49
        for (int i = 41; i <= 49; i++) begin
            if (rx_logical_data[i] !== i) $error("[FAIL] TC3: Inner 1:1 mapping corrupted at Lane %0d", i);
        end
        
        // Shift Right (> f1): Logical 50-62 pull from Physical 51-63
        for (int i = 50; i <= 62; i++) begin
            if (rx_logical_data[i] !== (i + 1)) $error("[FAIL] TC3: Shift-Right Undo failed at Logical Lane %0d", i);
        end
        $display("[PASS] TC3: Double failure Split-Shift correctly reconstructed logical datapath.");

        // =========================================================
        // TC4: Fatal Escalation (>2 Failures in a Group)
        // =========================================================
        $display("Running TC4: Unrepairable Escalation (3 Failures)");
        
        clear_faults();
        lane_failed[40] = 1'b1;
        lane_failed[50] = 1'b1;
        lane_failed[55] = 1'b1; // Third failure in upper group
        
        #10;
        
        if (!unrepairable)
            $error("[FAIL] TC4: Unrepairable flag failed to assert on 3rd failure.");
        else
            $display("[PASS] TC4: Unrepairable flag successfully escalated fatal damage.");

        $display("==================================================");
        $display("RX Repair Demultiplexer Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire