`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Valid Deframer
/// @description Decodes the 8-UI Valid envelope from the physical lane.
/// Extracts Flit validity, Retimer credits, and flags physical bit-flip framing errors.
module lphy_valid_deframer (
    input  wire        clk, 
    input  wire        rst_n, 
    
    // Configuration
    input  wire        is_retimer,      // 1: Remote partner is a UCIe Retimer
    
    // 8-bit Parallel Input from the Deserializer (Bit 0 received first on wire)
    input  wire [7:0]  valid_frame_in,
    
    // Extracted Outputs (Registered)
    output logic       lane_valid,      // High if data is valid for this 8-UI block
    output logic       credit_return,   // High if a Retimer credit is released
    output logic       framing_err      // High if an illegal/corrupted frame is received
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_valid    <= 1'b0;
            credit_return <= 1'b0;
            framing_err   <= 1'b0;
        end else begin
            // Default: clean state
            framing_err <= 1'b0;
            
            // Decode the 8-UI Valid frame (Table 17 of the UCIe Spec)
            case (valid_frame_in)
                8'h0F: begin
                    // Wire: 1111_0000 -> RTL: 8'b0000_1111
                    // Data Valid + No Credit
                    lane_valid    <= 1'b1;
                    credit_return <= 1'b0;
                end
                
                8'h00: begin
                    // Wire: 0000_0000 -> RTL: 8'b0000_0000
                    // No Data + No Credit (Electrical Idle)
                    lane_valid    <= 1'b0;
                    credit_return <= 1'b0;
                end
                
                8'hFF: begin
                    // Wire: 1111_1111 -> RTL: 8'b1111_1111
                    // Data Valid + 1 Credit
                    if (is_retimer) begin
                        lane_valid    <= 1'b1;
                        credit_return <= 1'b1;
                    end else begin
                        // Illegal frame for a non-retimer link
                        lane_valid    <= 1'b0;
                        credit_return <= 1'b0;
                        framing_err   <= 1'b1;
                    end
                end
                
                8'hF0: begin
                    // Wire: 0000_1111 -> RTL: 8'b1111_0000
                    // No Data + 1 Credit
                    if (is_retimer) begin
                        lane_valid    <= 1'b0;
                        credit_return <= 1'b1;
                    end else begin
                        // Illegal frame for a non-retimer link
                        lane_valid    <= 1'b0;
                        credit_return <= 1'b0;
                        framing_err   <= 1'b1;
                    end
                end
                
                default: begin
                    // Any other bit pattern implies a bit flip / Channel error
                    lane_valid    <= 1'b0;
                    credit_return <= 1'b0;
                    framing_err   <= 1'b1;
                end
            endcase
        end
    end

endmodule
`default_nettype wire