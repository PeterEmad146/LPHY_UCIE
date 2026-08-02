`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband Scrambler (Option A: Per-Lane LFSR)
/// @description 23-bit Fibonacci LFSR for UCIe data scrambling and PRBS generation.
module lphy_scrambler (
    input  wire         clk,
    input  wire         rst_n,          // Active-low reset
    
    // Control Signals
    input  wire         enable,         // High to advance LFSR and apply scrambling
    input  wire         load_seed,      // High to load the initial per-lane seed (e.g., during LINKINIT)
    input  wire [22:0]  seed_in,        // 23-bit Lane-specific seed value
    
    // Datapath
    input  wire [7:0]   data_in,        // 8-bit payload from datapath
    output wire [7:0]   data_out        // Scrambled 8-bit payload
);

    logic [22:0] lfsr_reg;
    logic [22:0] next_lfsr;
    logic [22:0] temp_lfsr;
    logic [7:0]  scramble_key;
    logic        feedback;
    
    // =========================================================================
    // Combinatorial block: Advance LFSR by 8 steps (Fibonacci Topology)
    // =========================================================================
    always_comb begin
        temp_lfsr = lfsr_reg;
        
        for (int i = 0; i < 8; i++) begin
            // The output bit of the LFSR is D22 (the MSB)
            scramble_key[i] = temp_lfsr[22];
            
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
            // LFSR only advances when enable is high. 
            // During un-scrambled training patterns, it correctly pauses.
            lfsr_reg <= next_lfsr;
        end
    end
    
    // =========================================================================
    // Datapath XOR
    // =========================================================================
    // XOR the input data with the generated 8-bit scramble key.
    // If enable is low, data passes through unmodified (Bypass Mode).
    assign data_out = enable ? (data_in ^ scramble_key) : data_in;

endmodule
`default_nettype wire