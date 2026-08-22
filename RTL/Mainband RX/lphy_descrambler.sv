`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Descrambler
/// @description 23-bit Fibonacci LFSR identical to the TX Scrambler.
/// (Optimized with XOR3-Chunked Parallel Math for 2GHz Closure)
module lphy_descrambler (
    input  wire         clk,
    input  wire         rst_n,
    
    // Control Signals
    input  wire         enable,         
    input  wire         load_seed,      
    input  wire [22:0]  seed_in,        
    
    // Datapath
    input  wire [7:0]   data_in,        
    output wire [7:0]   data_out        
);

    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic        enable_q;
    (* dont_touch = "true" *) logic        load_seed_q;
    (* dont_touch = "true" *) logic [22:0] seed_in_q;
    (* dont_touch = "true" *) logic [7:0]  data_in_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable_q    <= 1'b0;
            load_seed_q <= 1'b0;
            seed_in_q   <= '0;
            data_in_q   <= '0;
        end else begin
            enable_q    <= enable;
            load_seed_q <= load_seed;
            seed_in_q   <= seed_in;
            data_in_q   <= data_in;
        end
    end

    // =========================================================================
    // 2. XOR3-CHUNKED UNROLLED LFSR
    // Grouped by 3 to explicitly map to high-speed XOR3/XNOR3 standard cells
    // THIS is the logic that collapses the 6-level chain into 3 levels!
    // =========================================================================
    logic [22:0] lfsr_reg;
    logic [22:0] next_lfsr;
    logic [7:0]  descramble_key;
    
    wire [22:0] S = lfsr_reg;

    always_comb begin
        next_lfsr[22:8] = S[14:0];
        
        next_lfsr[7] = (S[22] ^ S[20] ^ S[15]) ^ (S[7] ^ S[4] ^ S[1]);
        
        next_lfsr[6] = (S[21] ^ S[19] ^ S[14]) ^ (S[6] ^ S[3] ^ S[0]);
        
        next_lfsr[5] = ((S[22] ^ S[18] ^ S[15]) ^ (S[13] ^ S[7] ^ S[5])) ^ (S[4] ^ S[2] ^ S[1]);
        
        next_lfsr[4] = ((S[21] ^ S[17] ^ S[14]) ^ (S[12] ^ S[6] ^ S[4])) ^ (S[3] ^ S[1] ^ S[0]);
        
        next_lfsr[3] = ((S[22] ^ S[16] ^ S[15]) ^ (S[13] ^ S[11] ^ S[7])) ^ ((S[5] ^ S[4] ^ S[3]) ^ (S[2] ^ S[1] ^ S[0]));
        
        next_lfsr[2] = ((S[22] ^ S[21] ^ S[20]) ^ (S[14] ^ S[12] ^ S[10])) ^ ((S[7] ^ S[6] ^ S[3]) ^ (S[2] ^ S[0]));
        
        next_lfsr[1] = ((S[22] ^ S[21] ^ S[19]) ^ (S[15] ^ S[13] ^ S[11])) ^ ((S[9] ^ S[7] ^ S[6]) ^ (S[5] ^ S[4] ^ S[2]));
        
        next_lfsr[0] = ((S[21] ^ S[20] ^ S[18]) ^ (S[14] ^ S[12] ^ S[10])) ^ ((S[8] ^ S[6] ^ S[5]) ^ (S[4] ^ S[3] ^ S[1]));

        descramble_key[0] = S[22];
        descramble_key[1] = S[21];
        descramble_key[2] = S[20];
        descramble_key[3] = S[19];
        descramble_key[4] = S[18];
        descramble_key[5] = S[17];
        descramble_key[6] = S[16];
        descramble_key[7] = S[15];
    end     
    
    // =========================================================================
    // 3. DATAPATH UPDATES & MUX
    // =========================================================================
    wire update_en = enable_q | load_seed_q;
    wire [22:0] lfsr_data = load_seed_q ? seed_in_q : next_lfsr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr_reg <= 23'h1DBFBC;     
        end else if (update_en) begin
            lfsr_reg <= lfsr_data;
        end
    end
    
    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // =========================================================================
    (* dont_touch = "true" *) logic [7:0] data_out_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) data_out_q <= 8'h00;
        else data_out_q <= enable_q ? (data_in_q ^ descramble_key) : data_in_q;
    end

    assign data_out = data_out_q;

endmodule
`default_nettype wire