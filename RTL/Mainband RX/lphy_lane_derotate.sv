`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe RX Reversal Detector & Alignment Pipeline
/// @description Datapath acts as a straight 1-cycle pipeline (no spatial muxing per spec).
/// Control logic detects Per-Lane ID patterns to report physical wiring reversal to the LTSSM.
module lphy_lane_derotate #(
    parameter int NUM_LANES = 64 // 16 for Standard Package, 64 for Advanced Package
)(
    input  wire        clk,
    input  wire        rst_n,

    // Data from RX Valid Deframer / Deserializer
    input  wire [7:0]  rx_lane_data_in [NUM_LANES-1:0],
    input  wire        rx_lane_valid,

    // Control Signals from LTSSM (MBINIT.REVERSALMB state)
    input  wire        en_reversal_check,
    output logic       reversal_detected,   // 1: Wire is reversed, 0: Normal
    output logic       reversal_check_done, // Pulses high when 128-iteration check is complete

    // Deskewed and Aligned Data to RX Top (Strictly 1:1 Passthrough)
    output logic [7:0] rx_lane_data_out [NUM_LANES-1:0]
);

    // =========================================================================
    // 1. RX Alignment Pipeline (NO SPATIAL MUXING PERMITTED)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_out[i] <= 8'h00;
            end
        end else if (rx_lane_valid) begin
            // The RX datapath must remain perfectly straight. 
            // Reversal multiplexing is handled exclusively by the TX partner.
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_out[i] <= rx_lane_data_in[i];
            end
        end
    end

    // =========================================================================
    // 2. Elaboration-Time Expected Pattern Generation
    // =========================================================================
    logic [7:0] exp_norm_b0 [NUM_LANES-1:0];
    logic [7:0] exp_norm_b1 [NUM_LANES-1:0];
    logic [7:0] exp_rev_b0  [NUM_LANES-1:0];
    logic [7:0] exp_rev_b1  [NUM_LANES-1:0];

    generate
        for (genvar i = 0; i < NUM_LANES; i++) begin : gen_exp_patterns
            wire [7:0] norm_id = i[7:0];
            wire [7:0] rev_id  = (NUM_LANES - 1 - i);

            // Pattern: 0 1 0 1 <Lane ID> 0 1 0 1 (LSB First)
            assign exp_norm_b0[i] = {norm_id[3:0], 4'b1010};
            assign exp_norm_b1[i] = {4'b1010, norm_id[7:4]};
            
            assign exp_rev_b0[i]  = {rev_id[3:0], 4'b1010};
            assign exp_rev_b1[i]  = {4'b1010, rev_id[7:4]};
        end
    endgenerate

    // =========================================================================
    // 3. Per-Lane ID Reversal Detection (MBINIT.REVERSALMB)
    // =========================================================================
    logic [7:0] prev_byte [NUM_LANES-1:0];
    logic [7:0] cycle_cnt;
    
    // We only need 5 bits to count up to 16 consecutive hits
    logic [4:0] consecutive_norm [NUM_LANES-1:0];
    logic [4:0] consecutive_rev  [NUM_LANES-1:0];
    
    // Lock flags: Set to 1 once a lane achieves 16 consecutive hits
    logic norm_locked [NUM_LANES-1:0];
    logic rev_locked  [NUM_LANES-1:0];
    
    // Combinatorial Majority Vote Tally
    logic [7:0] total_normal;
    logic [7:0] total_reversed;
    
    always_comb begin
        total_normal = 8'h00;
        total_reversed = 8'h00;
        for (int i = 0; i < NUM_LANES; i++) begin
            if (norm_locked[i]) total_normal   = total_normal + 1'b1;
            if (rev_locked[i])  total_reversed = total_reversed + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_LANES; i++) begin
                prev_byte[i]        <= 8'h00;
                consecutive_norm[i] <= 5'd0;
                consecutive_rev[i]  <= 5'd0;
                norm_locked[i]      <= 1'b0;
                rev_locked[i]       <= 1'b0;
            end
            cycle_cnt           <= 8'h00;
            reversal_detected   <= 1'b0;
            reversal_check_done <= 1'b0;
        end else begin
            reversal_check_done <= 1'b0; // Default pulse

            if (en_reversal_check && rx_lane_valid) begin
                cycle_cnt <= cycle_cnt + 1'b1;

                for (int i = 0; i < NUM_LANES; i++) begin
                    prev_byte[i] <= rx_lane_data_in[i];

                    // Because the pattern is 16 bits (2 bytes), we evaluate every odd clock cycle
                    if (cycle_cnt[0] == 1'b1) begin
                        // Check for Normal ID
                        if (prev_byte[i] == exp_norm_b0[i] && rx_lane_data_in[i] == exp_norm_b1[i]) begin
                            if (consecutive_norm[i] < 5'd16) consecutive_norm[i] <= consecutive_norm[i] + 1'b1;
                            if (consecutive_norm[i] == 5'd15) norm_locked[i] <= 1'b1; // 16th hit locks the lane
                        end else begin
                            consecutive_norm[i] <= 5'd0; // FIX: Break consecutive chain on mismatch
                        end

                        // Check for Reversed ID
                        if (prev_byte[i] == exp_rev_b0[i] && rx_lane_data_in[i] == exp_rev_b1[i]) begin
                            if (consecutive_rev[i] < 5'd16) consecutive_rev[i] <= consecutive_rev[i] + 1'b1;
                            if (consecutive_rev[i] == 5'd15) rev_locked[i] <= 1'b1; // 16th hit locks the lane
                        end else begin
                            consecutive_rev[i] <= 5'd0; // FIX: Break consecutive chain on mismatch
                        end
                    end
                end

                // Evaluate results after 128 iterations (256 clock cycles = 8'hFF)
                if (cycle_cnt == 8'hFF) begin
                    if (total_reversed > total_normal) begin
                        reversal_detected <= 1'b1; // Tell LTSSM to send Reversed Result
                    end else begin
                        reversal_detected <= 1'b0; // Tell LTSSM to send Normal Result
                    end
                    reversal_check_done <= 1'b1;
                end
                
            end else if (!en_reversal_check) begin
                // Clear state when check is disabled
                cycle_cnt <= 8'h00;
                for (int i = 0; i < NUM_LANES; i++) begin
                    consecutive_norm[i] <= 5'd0;
                    consecutive_rev[i]  <= 5'd0;
                    norm_locked[i]      <= 1'b0;
                    rev_locked[i]       <= 1'b0;
                end
            end
        end
    end
endmodule
`default_nettype wire