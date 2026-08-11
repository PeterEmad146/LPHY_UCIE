`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_lane_id_detect;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int NUM_LANES = 64;
    
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [7:0] rx_lane_data_in [NUM_LANES - 1:0];
    logic       rx_lane_valid;
    logic       en_lane_check;
    logic       is_reversed;
    
    wire  [NUM_LANES - 1:0] lane_failed;
    wire        check_done;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_lane_id_detect #(
        .NUM_LANES(NUM_LANES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rx_lane_data_in(rx_lane_data_in),
        .rx_lane_valid(rx_lane_valid),
        .en_lane_check(en_lane_check),
        .is_reversed(is_reversed),
        .lane_failed(lane_failed),
        .check_done(check_done)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        rx_lane_valid = 1'b0;
        en_lane_check = 1'b0;
        is_reversed   = 1'b0;
        for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = 8'h00;
    endtask

    // Generates the mathematically expected byte for a Normal or Reversed lane
    function automatic logic [7:0] get_expected_byte(int lane, bit inject_rev, bit is_byte1);
        logic [7:0] target_id;
        if (inject_rev) target_id = (NUM_LANES - 1 - lane);
        else            target_id = lane[7:0];
        
        if (!is_byte1) return {target_id[3:0], 4'b1010};
        else           return {4'b1010, target_id[7:4]};
    endfunction

    // Simulates the LTSSM sending the 128-iteration sequence
    // Allows injecting specific physical faults into designated lanes
    task run_repair_sequence(input bit inject_rev, input int stuck_lane, input int flutter_lane);
        $display("   -> Blasting 128 iterations (256 cycles)...");
        
        en_lane_check = 1'b1;
        rx_lane_valid = 1'b1;
        
        for (int iter = 0; iter < 128; iter++) begin
            // Cycle 0: Byte 0
            for (int i = 0; i < NUM_LANES; i++) begin
                if (i == stuck_lane) 
                    rx_lane_data_in[i] = 8'hFF; // Hard stuck fault
                else if (i == flutter_lane && (iter % 10 == 0)) 
                    rx_lane_data_in[i] = 8'hAA; // Flutters/Drops every 10 iterations
                else 
                    rx_lane_data_in[i] = get_expected_byte(i, inject_rev, 0);
            end
            @(negedge clk);
            
            // Cycle 1: Byte 1
            for (int i = 0; i < NUM_LANES; i++) begin
                if (i == stuck_lane) 
                    rx_lane_data_in[i] = 8'hFF;
                else if (i == flutter_lane && (iter % 10 == 0)) 
                    rx_lane_data_in[i] = 8'hAA;
                else 
                    rx_lane_data_in[i] = get_expected_byte(i, inject_rev, 1);
            end
            @(negedge clk); 
            
            // On final iteration, verify check_done pulsed
            if (iter == 127) begin
                #1; 
                if (!check_done) $error("[FAIL] check_done failed to pulse at 128 iterations.");
            end
        end
        
        // End of Check (LTSSM moves on to MBTRAIN)
        en_lane_check = 1'b0;
        rx_lane_valid = 1'b0;
        @(negedge clk);
    endtask

    logic [63:0] expected_mask;

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_lane_id_detect.fsdb");
        $fsdbDumpvars(0, tb_lphy_lane_id_detect);

        $display("==================================================");
        $display("Starting RX Lane ID (Repair) Detector Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Clean Link (Normal Wiring)
        // =========================================================
        $display("Running TC1: Clean Link (0 Faults)");
        is_reversed = 1'b0;
        
        run_repair_sequence(1'b0, -1, -1); // No reversed data, no stuck lane, no flutter lane
        
        if (lane_failed !== 64'h0)
            $error("[FAIL] TC1: False positive! Clean link flagged failures: %h", lane_failed);
        else
            $display("[PASS] TC1: 64 physical lanes achieved 16 consecutive hits and passed.");

        // =========================================================
        // TC2: Clean Link (Reversed Wiring)
        // =========================================================
        $display("Running TC2: Reversed Link (0 Faults)");
        is_reversed = 1'b1; // LTSSM asserts the package is reversed
        
        run_repair_sequence(1'b1, -1, -1); // Blast reversed data
        
        if (lane_failed !== 64'h0)
            $error("[FAIL] TC2: False positive! Reversed link failed to decode expected IDs.");
        else
            $display("[PASS] TC2: Reversed lane mapping successfully decoded and passed.");

        // =========================================================
        // TC3: Physical Fault Detection
        // =========================================================
        $display("Running TC3: Fault Detection (Stuck Lane 12, Fluttering Lane 45)");
        is_reversed = 1'b0; 
        
        // Inject a stuck-at fault on Lane 12, and a fluttering fault on Lane 45
        run_repair_sequence(1'b0, 12, 45);
        
        // Check exact failure mask
        expected_mask = 64'h0;
        expected_mask[12] = 1'b1;
        expected_mask[45] = 1'b1;
        
        if (lane_failed !== expected_mask) begin
            $error("[FAIL] TC3: Fault mask incorrect. Expected %h, Got %h", expected_mask, lane_failed);
        end else begin
            $display("[PASS] TC3: Successfully identified both permanent and intermittent physical faults.");
        end

        // =========================================================
        // TC4: Mask Persistence (LTSSM Idle)
        // =========================================================
        $display("Running TC4: Repair Mask Persistence");
        
        // The check finished in TC3. en_lane_check is now 0. 
        // We wait 10 clock cycles to simulate the LTSSM moving into MBTRAIN or ACTIVE states.
        repeat(10) @(negedge clk);
        
        if (lane_failed !== expected_mask) begin
            $error("[FAIL] TC4: Volatile bug detected! The lane_failed mask cleared itself when en_lane_check dropped.");
        end else begin
            $display("[PASS] TC4: The lane_failed mask securely held its state for the downstream repair muxes.");
        end

        $display("==================================================");
        $display("RX Lane ID Detector Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire