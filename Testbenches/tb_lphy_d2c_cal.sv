`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_d2c_cal;

    // ---------------------------------------------------------
    // Parameters, Clock and Reset
    // ---------------------------------------------------------
    localparam int NUM_LANES = 64;
    localparam int PI_PHASE_MAX = 63;
    localparam int SETTLE_CYCLES = 4; // Drastically lowered to speed up simulation!
    localparam int TEST_CYCLES = 16;  // Lowered for simulation speed
    
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        start_cal;
    logic [15:0] error_threshold;
    
    logic [7:0]  rx_data [NUM_LANES-1:0];
    logic [7:0]  expected_data [NUM_LANES-1:0];
    
    wire [5:0]   pi_phase;
    wire         cal_done;
    wire         cal_error;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_d2c_cal #(
        .NUM_LANES(NUM_LANES),
        .PI_PHASE_MAX(PI_PHASE_MAX),
        .SETTLE_CYCLES(SETTLE_CYCLES),
        .TEST_CYCLES(TEST_CYCLES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start_cal(start_cal),
        .error_threshold(error_threshold),
        .rx_data(rx_data),
        .expected_data(expected_data),
        .pi_phase(pi_phase),
        .cal_done(cal_done),
        .cal_error(cal_error)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task load_expected();
        for (int i = 0; i < NUM_LANES; i++) begin
            expected_data[i] = 8'(i);
        end
    endtask

    // Drives the RX data stream. Simulates an analog channel where the eye is only 
    // open (0 errors) between the defined left_phase and right_phase.
    task simulate_channel(input int left_phase, input int right_phase);
        int is_open;
        
        while (!cal_done) begin
            @(negedge clk);
            
            // Check if the current PI phase commanded by the DUT is inside our simulated eye
            if (left_phase <= right_phase) begin
                is_open = (pi_phase >= left_phase && pi_phase <= right_phase);
            end else begin
                // Wrapped eye logic
                is_open = (pi_phase >= left_phase || pi_phase <= right_phase);
            end
            
            for (int i = 0; i < NUM_LANES; i++) begin
                if (is_open) begin
                    rx_data[i] = expected_data[i]; // Perfect match (0 errors)
                end else begin
                    // Inject jitter/errors outside the eye
                    rx_data[i] = 8'hFF; 
                end
            end
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_d2c_cal.fsdb");
        $fsdbDumpvars(0, tb_lphy_d2c_cal);

        $display("==================================================");
        $display("Starting D2D Calibration (Phase Interpolator) Verification");
        $display("==================================================");

        clk = 0; rst_n = 0;
        start_cal = 0;
        error_threshold = 16'd0; // Require perfect match
        load_expected();
        
        // Initialize rx_data with garbage
        for (int i = 0; i < NUM_LANES; i++) rx_data[i] = 8'hFF;

        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Standard Contiguous Eye (Phase 10 to Phase 40)
        // =========================================================
        $display("Running TC1: Contiguous Eye Centering (Open: 10 to 40)");
        
        start_cal = 1'b1;
        fork
            simulate_channel(10, 40); // Simulates the AFE responding to pi_phase
        join
        
        // Eye Width = 30. Center = 10 + 15 = 25.
        if (cal_error || pi_phase !== 6'd25)
            $error("[FAIL] TC1: Failed to center eye. Expected Phase: 25, Got: %0d", pi_phase);
        else
            $display("[PASS] TC1: Standard eye beautifully centered at Phase 25.");
            
        start_cal = 1'b0;
        @(negedge clk); // Allow state machine to reset to IDLE
        @(negedge clk);

        // =========================================================
        // TC2: Circular Wrap-Around Eye (Phase 50 to Phase 10)
        // =========================================================
        $display("Running TC2: Wrap-Around Eye Centering (Open: 50 to 10)");
        
        start_cal = 1'b1;
        fork
            simulate_channel(50, 10);
        join
        
        // Wrapped Width = (64 - 50) + 10 = 24. Half = 12. Center = (50 + 12) % 64 = 62.
        if (cal_error || pi_phase !== 6'd62)
            $error("[FAIL] TC2: Wrap-around math failed. Expected Phase: 62, Got: %0d", pi_phase);
        else
            $display("[PASS] TC2: Wrap-around circular math perfectly calculated Phase 62.");

        start_cal = 1'b0;
        @(negedge clk);
        @(negedge clk);

        // =========================================================
        // TC3: Dead Link (Completely Closed Eye)
        // =========================================================
        $display("Running TC3: Dead Link Failure");
        
        start_cal = 1'b1;
        fork
            simulate_channel(100, 100); // Impossible phases force continuous errors
        join
        
        if (!cal_error)
            $error("[FAIL] TC3: Calibrator failed to flag the dead link.");
        else
            $display("[PASS] TC3: Fatal link damage successfully flagged.");

        $display("==================================================");
        $display("D2D Calibration Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire