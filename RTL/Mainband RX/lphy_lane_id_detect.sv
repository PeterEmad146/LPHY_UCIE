`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe RX Lane ID Detector (Data Lane Repair)
/// @description Scans all physical lanes for the Per-Lane ID pattern.
/// Flags lanes that fail to achieve 16 consecutive hits as broken.
module lphy_lane_id_detect #(
    parameter int NUM_LANES = 64 // 64 for Advanced Package, 16 for Standard Package
)(
    input  wire        clk,
    input  wire        rst_n, 
    
    // Data from RX Valid Deframer / Deserializer
    input  wire [7:0]  rx_lane_data_in [NUM_LANES - 1:0], 
    input  wire        rx_lane_valid, 
    
    // Control Signals from LTSSM (MBINIT.REPAIRMB)
    input  wire        en_lane_check, 
    input  wire        is_reversed,    // 1 if LTSSM determined lanes are reversed
    
    // Outputs to Data Repair Controller (Persistent Mask)
    output logic [NUM_LANES - 1:0] lane_failed, // 1: Broken (needs repair), 0: Passed
    output logic       check_done
);

    // =========================================================================
    // 1. Elaboration-Time Expected Pattern Generation
    // =========================================================================
    logic [7:0] exp_norm_b0 [NUM_LANES - 1:0];
    logic [7:0] exp_norm_b1 [NUM_LANES - 1:0];
    logic [7:0] exp_rev_b0  [NUM_LANES - 1:0];
    logic [7:0] exp_rev_b1  [NUM_LANES - 1:0];
    
    generate 
        for (genvar i = 0; i < NUM_LANES; i++) begin: gen_exp_patterns
            wire [7:0] norm_id = i[7:0];
            wire [7:0] rev_id  = (NUM_LANES - 1 - i);
            
            // Pattern: 0101 <Lane ID> 0101 (LSB First)
            assign exp_norm_b0[i] = {norm_id[3:0], 4'b1010};
            assign exp_norm_b1[i] = {4'b1010, norm_id[7:4]};
            
            assign exp_rev_b0[i]  = {rev_id[3:0], 4'b1010};
            assign exp_rev_b1[i]  = {4'b1010, rev_id[7:4]};
        end
    endgenerate
    
    // =========================================================================
    // 2. Detection & Consecutive Hit Logic
    // =========================================================================
    logic [7:0] prev_byte [NUM_LANES-1:0];
    logic [7:0] cycle_cnt;
    
    // Counters to track the mandatory 16 consecutive hits
    logic [4:0] consec_hits [NUM_LANES-1:0]; 
    logic       lane_passed [NUM_LANES-1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_LANES; i++) begin
                prev_byte[i]   <= 8'h00;
                consec_hits[i] <= 5'd0;
                lane_passed[i] <= 1'b0;
            end
            lane_failed <= '0;
            cycle_cnt   <= 8'h00;
            check_done  <= 1'b0;
        end else begin
            check_done <= 1'b0; // Default pulse to 0

            if (en_lane_check && rx_lane_valid) begin
                cycle_cnt <= cycle_cnt + 1'b1;

                for (int i = 0; i < NUM_LANES; i++) begin
                    prev_byte[i] <= rx_lane_data_in[i];

                    // Evaluate on every odd cycle (once 2 bytes are captured)
                    if (cycle_cnt[0] == 1'b1) begin
                        logic match;
                        if (is_reversed) begin
                            match = (prev_byte[i] == exp_rev_b0[i] && rx_lane_data_in[i] == exp_rev_b1[i]);
                        end else begin
                            match = (prev_byte[i] == exp_norm_b0[i] && rx_lane_data_in[i] == exp_norm_b1[i]);
                        end

                        if (match) begin
                            // Increment consecutive hits, capping at 16
                            if (consec_hits[i] < 5'd16) begin
                                consec_hits[i] <= consec_hits[i] + 1'b1;
                            end
                            // If we hit the 16th consecutive match, flag the lane as passed
                            if (consec_hits[i] == 5'd15) begin 
                                lane_passed[i] <= 1'b1;
                            end
                        end else begin
                            // A single bit error resets the consecutive counter
                            consec_hits[i] <= 5'd0; 
                        end
                    end
                end

                // Evaluate results after 128 iterations (256 clock cycles)
                if (cycle_cnt == 8'hFF) begin
                    for (int i = 0; i < NUM_LANES; i++) begin
                        // Lock in the failure mask. This holds its state permanently 
                        // until another check overrides it.
                        lane_failed[i] <= ~lane_passed[i];
                    end
                    check_done <= 1'b1;
                end
                
            end else if (!en_lane_check) begin
                // Clear state when not testing, BUT LEAVE lane_failed ALONE!
                cycle_cnt <= 8'h00;
                for (int i = 0; i < NUM_LANES; i++) begin
                    consec_hits[i] <= 5'd0;
                    lane_passed[i] <= 1'b0;
                end
            end
        end
    end
endmodule
`default_nettype wire