`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_lane_derotate;

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
    logic [7:0] rx_lane_data_in  [NUM_LANES-1:0];
    logic       rx_lane_valid;
    logic       en_reversal_check;
    
    wire        reversal_detected;
    wire        reversal_check_done;
    wire  [7:0] rx_lane_data_out [NUM_LANES-1:0];

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_lane_derotate #(
        .NUM_LANES(NUM_LANES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rx_lane_data_in(rx_lane_data_in),
        .rx_lane_valid(rx_lane_valid),
        .en_reversal_check(en_reversal_check),
        .reversal_detected(reversal_detected),
        .reversal_check_done(reversal_check_done),
        .rx_lane_data_out(rx_lane_data_out)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        rx_lane_valid     = 1'b0;
        en_reversal_check = 1'b0;
        for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = 8'h00;
    endtask

    // Generates the mathematically expected byte for a Normal or Reversed lane
    function automatic logic [7:0] get_expected_byte(int lane, bit is_reversed, bit is_byte1);
        logic [7:0] target_id;
        if (is_reversed) target_id = (NUM_LANES - 1 - lane);
        else             target_id = lane[7:0];
        
        // Byte 0: {ID[3:0], 4'b1010}
        // Byte 1: {4'b1010, ID[7:4]}
        if (!is_byte1) return {target_id[3:0], 4'b1010};
        else           return {4'b1010, target_id[7:4]};
    endfunction

    // Simulates the LTSSM sending the 128-iteration sequence
    task run_training_sequence(input bit inject_reversal);
        $display("   -> Blasting 128 iterations (256 cycles) of %s pattern...", 
                 inject_reversal ? "REVERSED" : "NORMAL");
                 
        en_reversal_check = 1'b1;
        rx_lane_valid     = 1'b1;
        
        for (int iter = 0; iter < 128; iter++) begin
            // Cycle 0: Byte 0
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_in[i] = get_expected_byte(i, inject_reversal, 0);
            end
            @(negedge clk);
            
            // Cycle 1: Byte 1
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_in[i] = get_expected_byte(i, inject_reversal, 1);
            end
            @(negedge clk); // FIX: Wait for the 256th clock edge to actually hit the hardware!
            
            // On the final iteration, check the 'done' flag AFTER the clock ticks
            if (iter == 127) begin
                #1; // Yield to ensure signal updates in simulator
                if (!reversal_check_done) 
                    $error("[FAIL] reversal_check_done failed to pulse at 128 iterations.");
            end
        end
        
        // Clean up
        en_reversal_check = 1'b0;
        rx_lane_valid     = 1'b0;
        @(negedge clk);
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_lane_derotate.fsdb");
        $fsdbDumpvars(0, tb_lphy_lane_derotate);

        $display("==================================================");
        $display("Starting RX Reversal Detector Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: 1-Cycle Datapath Passthrough
        // =========================================================
        $display("Running TC1: Strict 1-Cycle Alignment Pipeline");
        
        rx_lane_valid = 1'b1;
        for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = i[7:0]; // Payload = Lane index
        
        @(negedge clk);
        rx_lane_valid = 1'b0;
        
        // Data should have passed through 1:1, without spatial reversal!
        for (int i = 0; i < NUM_LANES; i++) begin
            if (rx_lane_data_out[i] !== i[7:0]) begin
                $error("[FAIL] TC1: Datapath pipelining failed/corrupted at Lane %0d", i);
            end
        end
        $display("[PASS] TC1: Datapath successfully acted as a straight pipeline.");
        clear_inputs();
        @(negedge clk);

        // =========================================================
        // TC2: Normal Wiring Detection
        // =========================================================
        $display("Running TC2: Normal Package Routing Check");
        
        run_training_sequence(1'b0); // 0 = Normal
        
        if (reversal_detected !== 1'b0)
            $error("[FAIL] TC2: False positive! Normal sequence flagged as reversed.");
        else
            $display("[PASS] TC2: Normal wiring successfully detected and cleared.");

        // =========================================================
        // TC3: Reversed Wiring Detection
        // =========================================================
        $display("Running TC3: Reversed Package Routing Check");
        
        run_training_sequence(1'b1); // 1 = Reversed
        
        if (reversal_detected !== 1'b1)
            $error("[FAIL] TC3: False negative! Reversed sequence was not flagged.");
        else
            $display("[PASS] TC3: Reversed wiring successfully detected and flagged.");

        // =========================================================
        // TC4: Broken Consecutive Chain (Robustness Check)
        // =========================================================
        $display("Running TC4: Broken Hit Chain Isolation");
        // We will assert enable, send 10 good hits, then send garbage.
        // It should NOT lock early.
        
        en_reversal_check = 1'b1;
        rx_lane_valid     = 1'b1;
        
        for (int iter = 0; iter < 10; iter++) begin
            for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = get_expected_byte(i, 0, 0);
            @(negedge clk);
            for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = get_expected_byte(i, 0, 1);
            @(negedge clk);
        end
        
        // Send Garbage
        for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = 8'hFF;
        @(negedge clk);
        @(negedge clk);
        
        // Send 6 more good hits (which would sum to 16 if it wasn't cleared)
        for (int iter = 0; iter < 6; iter++) begin
            for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = get_expected_byte(i, 0, 0);
            @(negedge clk);
            for (int i = 0; i < NUM_LANES; i++) rx_lane_data_in[i] = get_expected_byte(i, 0, 1);
            @(negedge clk);
        end
        
        // Check internal locks
        #1;
        if (dut.norm_locked[0] === 1'b1) 
            $error("[FAIL] TC4: Hit counter failed to reset on garbage data. Locked prematurely.");
        else
            $display("[PASS] TC4: Hit counters successfully reset upon receiving garbage data.");

        $display("==================================================");
        $display("RX Reversal Detector Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire