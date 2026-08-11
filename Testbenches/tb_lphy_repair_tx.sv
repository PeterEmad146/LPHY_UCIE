`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_repair_tx;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [7:0]  tx_logical_data [63:0];
    logic [63:0] lane_failed;

    wire  [7:0]  tx_physical_data [63:0];
    wire  [7:0]  tx_redundant_data [3:0];
    
    wire  [63:0] tx_physical_oe;
    wire  [3:0]  tx_redundant_oe;
    wire         unrepairable;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_repair_tx dut (
        .tx_logical_data(tx_logical_data),
        .lane_failed(lane_failed),
        .tx_physical_data(tx_physical_data),
        .tx_redundant_data(tx_redundant_data),
        .tx_physical_oe(tx_physical_oe),
        .tx_redundant_oe(tx_redundant_oe),
        .unrepairable(unrepairable)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task load_incremental_data();
        for (int i = 0; i < 64; i++) begin
            tx_logical_data[i] = i[7:0]; // Logical Lane 'i' carries payload 'i'
        end
    endtask

    task clear_faults();
        lane_failed = 64'h0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_repair_tx.fsdb");
        $fsdbDumpvars(0, tb_lphy_repair_tx);

        $display("==================================================");
        $display("Starting TX Redundancy Remapping Verification");
        $display("==================================================");

        // Initialize Datapath
        load_incremental_data();
        clear_faults();
        
        #10; // Yield for combinatorial stabilization

        // =========================================================
        // TC1: Clean Link (No Failures)
        // =========================================================
        $display("Running TC1: Clean Link (1:1 Mapping)");
        
        if (tx_physical_oe !== 64'hFFFF_FFFF_FFFF_FFFF || tx_redundant_oe !== 4'b0000)
            $error("[FAIL] TC1: Output Enables are incorrect for a clean link.");
            
        for (int i = 0; i < 64; i++) begin
            if (tx_physical_data[i] !== i[7:0]) $error("[FAIL] TC1: Mismatch on Lane %0d", i);
        end
        if (unrepairable) $error("[FAIL] TC1: Unrepairable flag asserted falsely.");
        else $display("[PASS] TC1: Clean link perfectly routed 1:1.");

        // =========================================================
        // TC2: Single Failure - Lower Group (Shift Left)
        // =========================================================
        $display("Running TC2: Single Failure at Lane 5 (Shift Left)");
        
        clear_faults();
        lane_failed[5] = 1'b1; // Inject failure
        
        #10; 
        
        // Check Redundant Bump 0 (Should take Logical Lane 0)
        if (tx_redundant_oe[0] !== 1'b1 || tx_redundant_data[0] !== 8'h00)
            $error("[FAIL] TC2: Redundant Bump 0 failed to pick up Logical Lane 0.");
            
        // Check Tri-state on failed pin
        if (tx_physical_oe[5] !== 1'b0 || tx_physical_data[5] !== 8'h00)
            $error("[FAIL] TC2: Failed Lane 5 was not tri-stated.");
            
        // Check Shift Left (Physical 0-4 should carry Logical 1-5)
        for (int i = 0; i < 5; i++) begin
            if (tx_physical_data[i] !== (i + 1)) $error("[FAIL] TC2: Shift Left failed at Physical Lane %0d", i);
        end
        
        // Check 1:1 passthrough above failure
        for (int i = 6; i < 32; i++) begin
            if (tx_physical_data[i] !== i) $error("[FAIL] TC2: 1:1 Mapping corrupted above failure at Lane %0d", i);
        end
        $display("[PASS] TC2: Single failure Shift-Left routed correctly.");

        // =========================================================
        // TC3: Double Failure - Lower Group (Shift Left & Right)
        // =========================================================
        $display("Running TC3: Double Failure at Lane 5 and Lane 25");
        
        clear_faults();
        lane_failed[5]  = 1'b1;
        lane_failed[25] = 1'b1;
        
        #10;
        
        // Check Redundant Bumps
        if (tx_redundant_oe[1:0] !== 2'b11 || tx_redundant_data[0] !== 8'h00 || tx_redundant_data[1] !== 8'd31)
            $error("[FAIL] TC3: Redundant bumps did not pick up Logical 0 and Logical 31.");
            
        // Check Tri-states
        if (tx_physical_oe[5] !== 1'b0 || tx_physical_oe[25] !== 1'b0)
            $error("[FAIL] TC3: Failed lanes were not tri-stated.");
            
        // Check Shift Right (Physical 26-31 should carry Logical 25-30)
        for (int i = 26; i < 32; i++) begin
            if (tx_physical_data[i] !== (i - 1)) $error("[FAIL] TC3: Shift Right failed at Physical Lane %0d", i);
        end
        
        // Check inner un-shifted bounds (Physical 6-24 should carry Logical 6-24)
        for (int i = 6; i < 25; i++) begin
            if (tx_physical_data[i] !== i) $error("[FAIL] TC3: Inner 1:1 mapping corrupted at Lane %0d", i);
        end
        $display("[PASS] TC3: Double failure Shift-Left & Shift-Right routed correctly.");

        // =========================================================
        // TC4: Fatal Escalation (>2 Failures in a Group)
        // =========================================================
        $display("Running TC4: Unrepairable Escalation (3 Failures)");
        
        clear_faults();
        lane_failed[5]  = 1'b1;
        lane_failed[25] = 1'b1;
        lane_failed[28] = 1'b1; // Third failure in lower group
        
        #10;
        
        if (!unrepairable)
            $error("[FAIL] TC4: Unrepairable flag failed to assert on 3rd failure.");
        else
            $display("[PASS] TC4: Unrepairable flag successfully escalated fatal damage.");

        $display("==================================================");
        $display("TX Repair Multiplexer Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire