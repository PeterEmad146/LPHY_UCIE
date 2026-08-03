`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Descrambler
/// @description 23-bit Fibonacci LFSR identical to the TX Scrambler.
/// Recovers the original payload by XORing with the synchronized PRBS key.
module lphy_descrambler (
    input  wire         clk,
    input  wire         rst_n,
    
    // Control Signals
    input  wire         enable,         // High to advance LFSR and descramble
    input  wire         load_seed,      // High to load the initial per-lane seed
    input  wire [22:0]  seed_in,        // 23-bit Lane-specific seed value
    
    // Datapath
    input  wire [7:0]   data_in,        // Scrambled 8-bit payload
    output wire [7:0]   data_out        // Descrambled original 8-bit payload
);

    logic [22:0] lfsr_reg;
    logic [22:0] next_lfsr;
    logic [22:0] temp_lfsr;
    logic [7:0]  descramble_key;
    logic        feedback;
    
    // =========================================================================
    // Combinatorial block: Advance LFSR by 8 steps (Fibonacci Topology)
    // =========================================================================
    always_comb begin
        temp_lfsr = lfsr_reg;
        
        for (int i = 0; i < 8; i++) begin
            // The output bit of the LFSR is D22 (the MSB)
            descramble_key[i] = temp_lfsr[22];
            
            // UCIe/PCIe Fibonacci Polynomial: X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1
            // Taps are located at D22, D20, D15, D7, D4, and D1
            feedback = temp_lfsr[22] ^ temp_lfsr[20] ^ temp_lfsr[15] ^ 
                       temp_lfsr[7]  ^ temp_lfsr[4]  ^ temp_lfsr[1];
                       
            // Shift left and append the calculated feedback bit to D0
            temp_lfsr = {temp_lfsr[21:0], feedback};
        end
        
        next_lfsr = temp_lfsr;
    end     
    
    // =========================================================================
    // Sequential block: Update the LFSR register
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr_reg <= 23'h1DBFBC;     // Default to Lane 0 Seed
        end else if (load_seed) begin
            lfsr_reg <= seed_in;
        end else if (enable) begin
            lfsr_reg <= next_lfsr;
        end
    end
    
    // =========================================================================
    // Datapath XOR
    // =========================================================================
    // XOR the input data with the generated 8-bit key.
    // If enable is low, data passes through unmodified (Bypass Mode).
    assign data_out = enable ? (data_in ^ descramble_key) : data_in;

endmodule
`default_nettype wire