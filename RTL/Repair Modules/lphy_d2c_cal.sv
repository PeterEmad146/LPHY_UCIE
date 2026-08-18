`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe D2D Calibration Controller (Data Eye Centering)
/// @description Sweeps the Analog Phase Interpolator (PI) across 64 steps.
/// Accumulates errors against a known pattern to find the data eye edges.
/// (Optimized with a 3-Stage Locked Pipeline and Pipelined Accumulator for 2 GHz)
module lphy_d2c_cal #(
    parameter int NUM_LANES = 64,
    parameter int PI_PHASE_MAX = 63,        // 64 total PI phase steps
    parameter int SETTLE_CYCLES = 32,       // Wait time for Analog PI to lock
    parameter int TEST_CYCLES = 128         // Number of UI to test per phase
)(
    input  wire         clk, 
    input  wire         rst_n, 
    
    // Control from LTSSM 
    input  wire         start_cal, 
    input  wire  [15:0] error_threshold,    
    
    // Data stream from RX Mainband 
    input  wire [7:0]   rx_data [NUM_LANES-1:0], 
    input  wire [7:0]   expected_data [NUM_LANES-1:0], 
    
    // Calibration Outputs 
    output logic [5:0]  pi_phase,           
    output logic        cal_done, 
    output logic        cal_error           
);

    typedef enum logic [2:0] {
        ST_IDLE        = 3'b000, 
        ST_SET_PHASE   = 3'b001, 
        ST_WAIT_SETTLE = 3'b010, 
        ST_ACCUMULATE  = 3'b011, 
        ST_FLUSH_PIPE  = 3'b100, 
        ST_EVALUATE    = 3'b101, 
        ST_CALC_CENTER = 3'b110, 
        ST_DONE        = 3'b111
    } state_t;

    state_t state, next_state;
    
    logic [5:0]  current_phase;
    logic [15:0] cycle_cnt;
    
    // -------------------------------------------------------------------------
    // PIPELINED COUNTER (Kills the 16-bit Ripple-Carry Delay)
    // -------------------------------------------------------------------------
    (* dont_touch = "true" *) logic [7:0] error_cnt_lo;
    (* dont_touch = "true" *) logic [7:0] error_cnt_hi;
    (* dont_touch = "true" *) logic       error_cnt_carry;
    logic [15:0] final_error_cnt;

    assign final_error_cnt = {error_cnt_hi, error_cnt_lo};
    
    logic [5:0]  left_edge;
    logic [5:0]  right_edge;
    logic        in_eye;
    logic        eye_found;
    
    logic [5:0]  first_eye_right;
    logic        first_eye_closed;
    logic        phase_0_passed;
    logic        phase_max_passed;
    
    // =========================================================================
    // PIPELINED ERROR DETECTION 
    // =========================================================================
    (* dont_touch = "true" *) logic [NUM_LANES-1:0] st1_lane_err;
    (* dont_touch = "true" *) logic                 st1_accum_en;
    
    (* dont_touch = "true" *) logic                 st2_cycle_err;
    (* dont_touch = "true" *) logic                 st2_accum_en;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_lane_err  <= '0;
            st1_accum_en  <= 1'b0;
            st2_cycle_err <= 1'b0;
            st2_accum_en  <= 1'b0;
        end else begin
            // Stage 1: Parallel Lane Comparisons
            for (int i = 0; i < NUM_LANES; i++) begin
                st1_lane_err[i] <= (rx_data[i] !== expected_data[i]);
            end
            st1_accum_en <= (state == ST_ACCUMULATE);
            
            // Stage 2: Global OR Reduction
            st2_cycle_err <= |st1_lane_err;
            st2_accum_en  <= st1_accum_en;
        end
    end
    
    // =========================================================================
    // SEQUENTIAL STATE MACHINE
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            current_phase    <= '0;
            cycle_cnt        <= '0;
            
            error_cnt_lo     <= '0;
            error_cnt_hi     <= '0;
            error_cnt_carry  <= 1'b0;

            left_edge        <= '0;
            right_edge       <= '0;
            in_eye           <= 1'b0;
            eye_found        <= 1'b0;
            pi_phase         <= '0;
            first_eye_right  <= '0;
            first_eye_closed <= 1'b0;
            phase_0_passed   <= 1'b0;
            phase_max_passed <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                ST_IDLE: begin
                    current_phase    <= '0;
                    left_edge        <= '0;
                    right_edge       <= '0;
                    in_eye           <= 1'b0;
                    eye_found        <= 1'b0;
                    cycle_cnt        <= '0;
                    
                    first_eye_right  <= '0;
                    first_eye_closed <= 1'b0;
                    phase_0_passed   <= 1'b0;
                    phase_max_passed <= 1'b0;
                end
                
                ST_SET_PHASE: begin
                    pi_phase  <= current_phase; 
                    cycle_cnt <= '0;
                end
                
                ST_WAIT_SETTLE: begin
                    cycle_cnt <= cycle_cnt + 1'b1;
                end
                
                ST_ACCUMULATE: begin
                    cycle_cnt <= cycle_cnt + 1'b1;
                end
                
                ST_FLUSH_PIPE: begin
                    cycle_cnt <= cycle_cnt + 1'b1;
                end
                
                ST_EVALUATE: begin
                    if (final_error_cnt <= error_threshold) begin
                        if (!in_eye) begin
                            left_edge <= current_phase;
                            in_eye    <= 1'b1;
                        end
                        right_edge <= current_phase;    
                        eye_found  <= 1'b1;
                        
                        if (current_phase == 6'd0)              phase_0_passed   <= 1'b1;
                        if (current_phase == PI_PHASE_MAX[5:0]) phase_max_passed <= 1'b1;
                    end else begin
                        if (in_eye) begin
                            in_eye <= 1'b0;     
                            if (!first_eye_closed) begin
                                first_eye_right  <= right_edge;
                                first_eye_closed <= 1'b1;
                            end
                        end
                    end
                    
                    current_phase <= current_phase + 1'b1;
                end
                
                ST_CALC_CENTER: begin
                    if (eye_found) begin
                        if (phase_0_passed && phase_max_passed) begin
                            pi_phase <= (left_edge + ((7'd64 - left_edge + first_eye_right) >> 1)) % 64;
                        end else begin
                            pi_phase <= left_edge + ((right_edge - left_edge) >> 1);
                        end
                    end
                end
            endcase
            
            // -----------------------------------------------------------------
            // INDEPENDENT PIPELINED ERROR ACCUMULATOR
            // -----------------------------------------------------------------
            if (state == ST_SET_PHASE) begin
                error_cnt_lo    <= '0;
                error_cnt_hi    <= '0;
                error_cnt_carry <= 1'b0;
            end else begin
                // Lower 8-bit increment (Half the carry-chain depth!)
                if (st2_accum_en && st2_cycle_err) begin
                    error_cnt_lo    <= error_cnt_lo + 1'b1;
                    error_cnt_carry <= (error_cnt_lo == 8'hFF); // Trigger overflow
                end else begin
                    error_cnt_carry <= 1'b0;
                end
                
                // Upper 8-bit increment executes 1 cycle later
                if (error_cnt_carry) begin
                    error_cnt_hi <= error_cnt_hi + 1'b1;
                end
            end
        end
    end
    
    // =========================================================================
    // COMBINATIONAL NEXT STATE LOGIC
    // =========================================================================
    always_comb begin
        next_state = state;
        cal_done   = 1'b0;
        cal_error  = 1'b0;
        
        case (state)
            ST_IDLE: begin
                if (start_cal) next_state = ST_SET_PHASE;
            end
            
            ST_SET_PHASE: begin
                next_state = ST_WAIT_SETTLE;
            end
            
            ST_WAIT_SETTLE: begin
                if (cycle_cnt == SETTLE_CYCLES - 1) begin
                    next_state = ST_ACCUMULATE;
                end
            end
            
            ST_ACCUMULATE: begin
                if (cycle_cnt == (SETTLE_CYCLES + TEST_CYCLES - 1)) begin
                    next_state = ST_FLUSH_PIPE;
                end
            end
            
            ST_FLUSH_PIPE: begin
                if (cycle_cnt == (SETTLE_CYCLES + TEST_CYCLES + 1)) begin
                    next_state = ST_EVALUATE;
                end
            end
            
            ST_EVALUATE: begin
                if (current_phase == PI_PHASE_MAX[5:0]) next_state = ST_CALC_CENTER;
                else next_state = ST_SET_PHASE;
            end
            
            ST_CALC_CENTER: begin
                next_state = ST_DONE;
            end
            
            ST_DONE: begin
                cal_done = 1'b1;
                if (!eye_found) cal_error = 1'b1; 
                if (!start_cal) next_state = ST_IDLE;
            end 
        endcase
    end
endmodule
`default_nettype wire