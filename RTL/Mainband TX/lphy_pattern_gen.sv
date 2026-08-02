`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband Training Pattern Generator
/// @description Generates the unscrambled, static training patterns required for 
/// MBINIT phase (Lane ID, VALTRAIN, Clock Repair). Streams output in 8-bit chunks.
module lphy_pattern_gen (
    input  wire         clk,
    input  wire         rst_n,          // Active-low reset
    
    // Configuration
    input  wire [7:0]   lane_id,        // 8-bit Lane ID for REVERSALMB
    
    // Pattern Selection (From LTSSM)
    // 00 = None (Output 0), 01 = Per-Lane ID, 10 = VALTRAIN, 11 = Clock Repair
    input  wire [1:0]   pattern_sel,    
    input  wire         enable,         // High to advance the pattern stream
    
    // Datapath Output
    output logic [7:0]  pattern_out     // 8-bit slice of the requested pattern
);

    // =========================================================================
    // Internal State
    // =========================================================================
    // The longest pattern is the Clock Repair pattern (24 bits = 3 chunks of 8)
    // A 2-bit counter is sufficient to track the streaming phase (0 to 2)
    logic [1:0] phase_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_cnt <= 2'b00;
        end else if (pattern_sel == 2'b00) begin
            // Reset phase counter when no pattern is active
            phase_cnt <= 2'b00;
        end else if (enable) begin
            // Advance phase based on the length of the selected pattern
            if (pattern_sel == 2'b01) begin
                // Per-Lane ID is 16 bits (2 phases)
                phase_cnt <= (phase_cnt == 2'b01) ? 2'b00 : (phase_cnt + 1'b1);
            end else if (pattern_sel == 2'b11) begin
                // Clock Repair is 24 bits (3 phases)
                phase_cnt <= (phase_cnt == 2'b10) ? 2'b00 : (phase_cnt + 1'b1);
            end else begin
                // VALTRAIN is 8 bits (1 phase, constantly repeats)
                phase_cnt <= 2'b00;
            end
        end
    end

    // =========================================================================
    // Pattern Generation Logic (Combinatorial)
    // =========================================================================
    always_comb begin
        pattern_out = 8'h00; // Default zero

        case (pattern_sel)
            2'b01: begin
                // Per-Lane ID Pattern (16 bits): 0101 <Lane ID [0:7]> 0101
                // Phase 0 (Lower 8 bits): 0101 (4'b1010 LSB first) + Lane ID [3:0]
                // Phase 1 (Upper 8 bits): Lane ID [7:4] + 0101 (4'b1010 LSB first)
                if (phase_cnt == 2'b00) begin
                    pattern_out = {lane_id[3:0], 4'b1010};
                end else begin
                    pattern_out = {4'b1010, lane_id[7:4]};
                end
            end
            
            2'b10: begin
                // VALTRAIN Pattern (8 bits): Four 1's followed by Four 0's.
                // Transmitted LSB-first -> 8'b00001111 -> 8'h0F
                pattern_out = 8'h0F;
            end
            
            2'b11: begin
                // Clock Repair Pattern (24 bits): 16 clock cycles (toggling) then 8 low.
                // "Toggling" starting with 0 is 0101... -> 8'b10101010 (8'hAA LSB first)
                // Phase 0: 8'hAA
                // Phase 1: 8'hAA
                // Phase 2: 8'h00
                if (phase_cnt == 2'b00) begin
                    pattern_out = 8'hAA;
                end else if (phase_cnt == 2'b01) begin
                    pattern_out = 8'hAA;
                end else begin
                    pattern_out = 8'h00;
                end
            end
            
            default: begin
                pattern_out = 8'h00;
            end
        endcase
    end

endmodule
`default_nettype wire