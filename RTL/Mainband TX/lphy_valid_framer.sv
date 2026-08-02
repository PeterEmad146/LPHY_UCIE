`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband Valid Framer
/// @description Generates the 8-UI Valid envelope for mainband data framing, 
/// handles Retimer credit overloading, and provides VALTRAIN overrides.
module lphy_valid_framer (
    input  wire        clk,
    input  wire        rst_n,
    
    // Configuration & State
    input  wire        is_retimer,    // High if the remote partner is a UCIe Retimer
    input  wire        train_mode,    // High during MBINIT.REPAIRVAL or MBTRAIN.VALTRAINCENTER
    
    // Inputs from TX Byte-to-Lane Mapper and Flow Control
    input  wire        lane_valid,    // High if datapath has a valid Flit payload
    input  wire        credit_return, // High if a Retimer credit needs to be released
    
    // 8-bit Parallel Output to the Serializer (Bit 0 transmitted first)
    output logic [7:0] valid_frame_out
);

    // =========================================================================
    // Safety Masking
    // =========================================================================
    // Strictly enforce the rule: Never transmit credit encodings to non-retimers.
    wire effective_credit_return = is_retimer ? credit_return : 1'b0;

    // =========================================================================
    // Framing State Machine
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_frame_out <= 8'h00;
        end else if (train_mode) begin
            // Override datapath: Continuously output VALTRAIN pattern
            // Four 1's followed by Four 0's (LSB first) -> 8'b00001111 (8'h0F)
            valid_frame_out <= 8'h0F;
        end else begin
            // Standard runtime framing and Retimer credit multiplexing
            case ({effective_credit_return, lane_valid})
                2'b11: valid_frame_out <= 8'hFF; // Data Valid + 1 Credit
                2'b01: valid_frame_out <= 8'h0F; // Data Valid + No Credit
                2'b10: valid_frame_out <= 8'hF0; // No Data + 1 Credit
                2'b00: valid_frame_out <= 8'h00; // No Data + No Credit
            endcase
        end
    end

endmodule
`default_nettype wire