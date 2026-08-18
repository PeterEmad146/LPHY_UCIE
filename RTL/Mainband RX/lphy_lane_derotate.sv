`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe RX Reversal Detector & Alignment Pipeline
/// @description Datapath acts as a straight 1-cycle pipeline. Control logic 
/// detects Per-Lane ID patterns to report physical wiring reversal to the LTSSM.
/// (Optimized with a Strict Binary Tree Pipeline for 32nm 2GHz Closure)
module lphy_lane_derotate #(
    parameter int NUM_LANES = 64 
)(
    input  wire         clk,
    input  wire         rst_n,

    // Data from RX Valid Deframer / Deserializer
    input  wire [7:0]   rx_lane_data_in [NUM_LANES-1:0],
    input  wire         rx_lane_valid,

    // Control Signals from LTSSM (MBINIT.REVERSALMB state)
    input  wire         en_reversal_check,
    output logic        reversal_detected,   
    output logic        reversal_check_done, 

    // Deskewed and Aligned Data to RX Top 
    output logic [7:0]  rx_lane_data_out [NUM_LANES-1:0]
);

    // =========================================================================
    // 1. RX Alignment Pipeline
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_out[i] <= 8'h00;
            end
        end else if (rx_lane_valid) begin
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

            assign exp_norm_b0[i] = {norm_id[3:0], 4'b1010};
            assign exp_norm_b1[i] = {4'b1010, norm_id[7:4]};
            
            assign exp_rev_b0[i]  = {rev_id[3:0], 4'b1010};
            assign exp_rev_b1[i]  = {4'b1010, rev_id[7:4]};
        end
    endgenerate

    // =========================================================================
    // 3. Sliced Popcount Function
    // =========================================================================
    function automatic logic [3:0] popcount8(input logic [7:0] vec);
        logic [1:0] s1 [0:3];
        logic [2:0] s2 [0:1];
        int i;
        for (i = 0; i < 4; i++) s1[i] = {1'b0, vec[2*i]} + {1'b0, vec[2*i+1]};
        for (i = 0; i < 2; i++) s2[i] = s1[2*i] + s1[2*i+1];
        return s2[0] + s2[1];
    endfunction

    // =========================================================================
    // 4. Per-Lane ID Reversal Detection 
    // =========================================================================
    logic [7:0] prev_byte [NUM_LANES-1:0];
    logic [7:0] cycle_cnt;
    logic [4:0] consecutive_norm [NUM_LANES-1:0];
    logic [4:0] consecutive_rev  [NUM_LANES-1:0];
    
    logic [NUM_LANES-1:0] norm_locked;
    logic [NUM_LANES-1:0] rev_locked;

    logic [63:0] safe_norm_locked;
    logic [63:0] safe_rev_locked;
    
    always_comb begin
        safe_norm_locked = '0;
        safe_rev_locked  = '0;
        safe_norm_locked[NUM_LANES-1:0] = norm_locked;
        safe_rev_locked[NUM_LANES-1:0]  = rev_locked;
    end

    // -------------------------------------------------------------------------
    // STRICT BINARY TREE PIPELINE REGISTERS
    // Every single addition is isolated by a hard lock.
    // -------------------------------------------------------------------------
    (* dont_touch = "true" *) logic [3:0] norm_p0, norm_p1, norm_p2, norm_p3;
    (* dont_touch = "true" *) logic [3:0] norm_p4, norm_p5, norm_p6, norm_p7;
    (* dont_touch = "true" *) logic [3:0] rev_p0,  rev_p1,  rev_p2,  rev_p3;
    (* dont_touch = "true" *) logic [3:0] rev_p4,  rev_p5,  rev_p6,  rev_p7;

    // First layer of sums (5 bits to prevent overflow)
    (* dont_touch = "true" *) logic [4:0] norm_sum_01, norm_sum_23, norm_sum_45, norm_sum_67;
    (* dont_touch = "true" *) logic [4:0] rev_sum_01,  rev_sum_23,  rev_sum_45,  rev_sum_67;

    // Second layer of sums (6 bits)
    (* dont_touch = "true" *) logic [5:0] norm_half_0, norm_half_1;
    (* dont_touch = "true" *) logic [5:0] rev_half_0,  rev_half_1;

    // Final sums (7 bits)
    (* dont_touch = "true" *) logic [6:0] total_normal_q;
    (* dont_touch = "true" *) logic [6:0] total_reversed_q;
    (* dont_touch = "true" *) logic       reversal_decision_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_LANES; i++) begin
                prev_byte[i]        <= 8'h00;
                consecutive_norm[i] <= 5'd0;
                consecutive_rev[i]  <= 5'd0;
            end
            norm_locked         <= '0;
            rev_locked          <= '0;
            cycle_cnt           <= 8'h00;
            reversal_detected   <= 1'b0;
            reversal_check_done <= 1'b0;
            
            norm_p0 <= 4'd0; norm_p1 <= 4'd0; norm_p2 <= 4'd0; norm_p3 <= 4'd0;
            norm_p4 <= 4'd0; norm_p5 <= 4'd0; norm_p6 <= 4'd0; norm_p7 <= 4'd0;
            rev_p0  <= 4'd0; rev_p1  <= 4'd0; rev_p2  <= 4'd0; rev_p3  <= 4'd0;
            rev_p4  <= 4'd0; rev_p5  <= 4'd0; rev_p6  <= 4'd0; rev_p7  <= 4'd0;

            norm_sum_01 <= 5'd0; norm_sum_23 <= 5'd0; norm_sum_45 <= 5'd0; norm_sum_67 <= 5'd0;
            rev_sum_01  <= 5'd0; rev_sum_23  <= 5'd0; rev_sum_45  <= 5'd0; rev_sum_67  <= 5'd0;

            norm_half_0 <= 6'd0; norm_half_1 <= 6'd0;
            rev_half_0  <= 6'd0; rev_half_1  <= 6'd0;

            total_normal_q      <= 7'd0;
            total_reversed_q    <= 7'd0;
            reversal_decision_q <= 1'b0;
        end else begin
            reversal_check_done <= 1'b0; 

            if (en_reversal_check && rx_lane_valid) begin
                cycle_cnt <= cycle_cnt + 1'b1;

                for (int i = 0; i < NUM_LANES; i++) begin
                    prev_byte[i] <= rx_lane_data_in[i];

                    if (cycle_cnt[0] == 1'b1) begin
                        if (prev_byte[i] == exp_norm_b0[i] && rx_lane_data_in[i] == exp_norm_b1[i]) begin
                            if (consecutive_norm[i] < 5'd16) consecutive_norm[i] <= consecutive_norm[i] + 1'b1;
                            if (consecutive_norm[i] == 5'd15) norm_locked[i] <= 1'b1; 
                        end else begin
                            consecutive_norm[i] <= 5'd0; 
                        end

                        if (prev_byte[i] == exp_rev_b0[i] && rx_lane_data_in[i] == exp_rev_b1[i]) begin
                            if (consecutive_rev[i] < 5'd16) consecutive_rev[i] <= consecutive_rev[i] + 1'b1;
                            if (consecutive_rev[i] == 5'd15) rev_locked[i] <= 1'b1; 
                        end else begin
                            consecutive_rev[i] <= 5'd0; 
                        end
                    end
                end

                // -------------------------------------------------------------
                // THE STRICT BINARY TREE (1 Adder Per Cycle)
                // -------------------------------------------------------------
                
                // Stage 1 (Cycle 249): 8-bit Partial Sums 
                if (cycle_cnt == 8'hF9) begin
                    norm_p0 <= popcount8(safe_norm_locked[7:0]);
                    norm_p1 <= popcount8(safe_norm_locked[15:8]);
                    norm_p2 <= popcount8(safe_norm_locked[23:16]);
                    norm_p3 <= popcount8(safe_norm_locked[31:24]);
                    norm_p4 <= popcount8(safe_norm_locked[39:32]);
                    norm_p5 <= popcount8(safe_norm_locked[47:40]);
                    norm_p6 <= popcount8(safe_norm_locked[55:48]);
                    norm_p7 <= popcount8(safe_norm_locked[63:56]);

                    rev_p0  <= popcount8(safe_rev_locked[7:0]);
                    rev_p1  <= popcount8(safe_rev_locked[15:8]);
                    rev_p2  <= popcount8(safe_rev_locked[23:16]);
                    rev_p3  <= popcount8(safe_rev_locked[31:24]);
                    rev_p4  <= popcount8(safe_rev_locked[39:32]);
                    rev_p5  <= popcount8(safe_rev_locked[47:40]);
                    rev_p6  <= popcount8(safe_rev_locked[55:48]);
                    rev_p7  <= popcount8(safe_rev_locked[63:56]);
                end

                // Stage 2 (Cycle 250): Add pairs (Strictly A + B)
                if (cycle_cnt == 8'hFA) begin
                    norm_sum_01 <= norm_p0 + norm_p1;
                    norm_sum_23 <= norm_p2 + norm_p3;
                    norm_sum_45 <= norm_p4 + norm_p5;
                    norm_sum_67 <= norm_p6 + norm_p7;

                    rev_sum_01  <= rev_p0 + rev_p1;
                    rev_sum_23  <= rev_p2 + rev_p3;
                    rev_sum_45  <= rev_p4 + rev_p5;
                    rev_sum_67  <= rev_p6 + rev_p7;
                end

                // Stage 3 (Cycle 251): Add quadrants (Strictly A + B)
                if (cycle_cnt == 8'hFB) begin
                    norm_half_0 <= norm_sum_01 + norm_sum_23;
                    norm_half_1 <= norm_sum_45 + norm_sum_67;
                    rev_half_0  <= rev_sum_01  + rev_sum_23;
                    rev_half_1  <= rev_sum_45  + rev_sum_67;
                end

                // Stage 4 (Cycle 252): Add halves (Strictly A + B)
                if (cycle_cnt == 8'hFC) begin
                    total_normal_q   <= norm_half_0 + norm_half_1;
                    total_reversed_q <= rev_half_0  + rev_half_1;
                end

                // Stage 5 (Cycle 253): Compare 
                if (cycle_cnt == 8'hFD) begin
                    reversal_decision_q <= (total_reversed_q > total_normal_q);
                end

                // Stage 6 (Cycle 254): Route
                if (cycle_cnt == 8'hFE) begin
                    reversal_detected   <= reversal_decision_q;
                    reversal_check_done <= 1'b1;
                end
                
            end else if (!en_reversal_check) begin
                cycle_cnt   <= 8'h00;
                norm_locked <= '0;
                rev_locked  <= '0;
                for (int i = 0; i < NUM_LANES; i++) begin
                    consecutive_norm[i] <= 5'd0;
                    consecutive_rev[i]  <= 5'd0;
                end
            end
        end
    end
endmodule
`default_nettype wire