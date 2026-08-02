`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Byte-to-Lane Mapper
/// @description Slices 64-Byte protocol transfers into 8, 16, 32, or 64 active lanes.
/// Handles x8 width degradation routing and logical Lane Reversal.
module lphy_byte_lane_map (
    input  wire        clk,
    input  wire        rst_n, 
    
    // Configuration & LTSSM Overrides
    input  wire [1:0]  link_width,    // 2'b00: x16, 2'b01: x32, 2'b10: x64
    input  wire        degrade_x8,    // High if operating in degraded x8 mode (Standard Pkg)
    input  wire        degrade_upper, // High: map to lanes [15:8], Low: map to lanes [7:0]
    input  wire        lane_reversal, // High to reverse logical lane ordering
    
    // Interface from D2D Adapter (64 Bytes per transfer)
    input  wire        lp_valid,
    input  wire        lp_irdy, 
    output wire        pl_trdy, 
    input  wire [511:0] lp_data,
    
    // Output to Physical Lanes (up to 64 active lanes, 1 Byte per lane)
    output logic       lane_valid, 
    output logic [7:0] lane_data [63:0] // Unpacked array for per-lane routing
);

    // =========================================================================
    // Internal State
    // =========================================================================
    logic [511:0] buffer;
    logic [3:0]   chunk_cnt;     // FIX: Widened to 4 bits to hold up to 8
    logic         busy;
    
    logic [3:0]   max_chunks;    // FIX: Widened to 4 bits
    logic [511:0] current_src;
    logic [3:0]   current_chunk; // FIX: Widened to 4 bits

    // Combinatorial Output Muxes
    logic [7:0]   next_active_bytes [63:0];
    logic [7:0]   next_mapped_lanes [63:0];
    logic [7:0]   next_final_lanes  [63:0];

    // The PHY is ready to accept a new payload when not sequencing a previous one
    assign pl_trdy = !busy;

    // Determine total serialization depth based on configuration
    always_comb begin
        if (link_width == 2'b10)      max_chunks = 4'd1; // x64: 1 cycle
        else if (link_width == 2'b01) max_chunks = 4'd2; // x32: 2 cycles
        else if (degrade_x8)          max_chunks = 4'd8; // x8:  8 cycles
        else                          max_chunks = 4'd4; // x16: 4 cycles
    end

    // Select datapath source 
    always_comb begin
        if (!busy && lp_valid && lp_irdy) begin
            current_src   = lp_data;
            current_chunk = 4'd0;
        end else begin
            current_src   = buffer;
            current_chunk = chunk_cnt;
        end
    end

    // =========================================================================
    // Combinatorial Mapping Pipeline
    // =========================================================================
    always_comb begin
        // ---------------------------------------------------------------------
        // Stage 1: Byte Extraction
        // ---------------------------------------------------------------------
        for (int i = 0; i < 64; i++) next_active_bytes[i] = 8'h00;

        if (link_width == 2'b10) begin
            for (int i = 0; i < 64; i++) next_active_bytes[i] = current_src[i*8 +: 8];
        end else if (link_width == 2'b01) begin
            for (int i = 0; i < 32; i++) next_active_bytes[i] = current_src[(i + current_chunk*32)*8 +: 8];
        end else if (degrade_x8) begin
            for (int i = 0; i < 8; i++)  next_active_bytes[i] = current_src[(i + current_chunk*8)*8 +: 8];
        end else begin
            for (int i = 0; i < 16; i++) next_active_bytes[i] = current_src[(i + current_chunk*16)*8 +: 8];
        end

        // ---------------------------------------------------------------------
        // Stage 2: Spatial Lane Assignment
        // ---------------------------------------------------------------------
        for (int i = 0; i < 64; i++) next_mapped_lanes[i] = 8'h00;

        if (link_width == 2'b10) begin
            for (int i = 0; i < 64; i++) next_mapped_lanes[i] = next_active_bytes[i];
        end else if (link_width == 2'b01) begin
            for (int i = 0; i < 32; i++) next_mapped_lanes[i] = next_active_bytes[i];
        end else if (degrade_x8) begin
            if (degrade_upper) begin
                for (int i = 0; i < 8; i++) next_mapped_lanes[i+8] = next_active_bytes[i]; // Map to [15:8]
            end else begin
                for (int i = 0; i < 8; i++) next_mapped_lanes[i]   = next_active_bytes[i]; // Map to [7:0]
            end
        end else begin
            for (int i = 0; i < 16; i++) next_mapped_lanes[i] = next_active_bytes[i];
        end

        // ---------------------------------------------------------------------
        // Stage 3: Logical Lane Reversal Mux
        // ---------------------------------------------------------------------
        for (int i = 0; i < 64; i++) next_final_lanes[i] = 8'h00;

        if (lane_reversal) begin
            if (link_width == 2'b10) begin
                for (int i = 0; i < 64; i++) next_final_lanes[i] = next_mapped_lanes[63-i];
            end else if (link_width == 2'b01) begin
                for (int i = 0; i < 32; i++) next_final_lanes[i] = next_mapped_lanes[31-i];
            end else if (degrade_x8) begin
                if (degrade_upper) begin
                    for (int i = 0; i < 8; i++) next_final_lanes[i+8] = next_mapped_lanes[15-i];
                end else begin
                    for (int i = 0; i < 8; i++) next_final_lanes[i]   = next_mapped_lanes[7-i];
                end
            end else begin
                for (int i = 0; i < 16; i++) next_final_lanes[i] = next_mapped_lanes[15-i];
            end
        end else begin
            for (int i = 0; i < 64; i++) next_final_lanes[i] = next_mapped_lanes[i];
        end
    end

    // =========================================================================
    // Sequential State Update
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer     <= '0;
            chunk_cnt  <= '0;
            busy       <= 1'b0;
            lane_valid <= 1'b0;
            for (int i = 0; i < 64; i++) lane_data[i] <= 8'h00;
        end else begin
            if (!busy) begin
                if (lp_valid && lp_irdy) begin
                    // Capture full 64-Byte payload
                    buffer     <= lp_data;
                    lane_valid <= 1'b1;
                    
                    // Drive first chunk immediately
                    for (int i = 0; i < 64; i++) lane_data[i] <= next_final_lanes[i];

                    // Lock FSM if more chunks are required
                    if (max_chunks > 4'd1) begin
                        busy      <= 1'b1;
                        chunk_cnt <= 4'd1;
                    end
                end else begin
                    lane_valid <= 1'b0;
                    for (int i = 0; i < 64; i++) lane_data[i] <= 8'h00; // Power isolation
                end
            end else begin
                // Drive subsequent chunks
                lane_valid <= 1'b1;
                for (int i = 0; i < 64; i++) lane_data[i] <= next_final_lanes[i];

                if (chunk_cnt == (max_chunks - 4'd1)) begin
                    busy <= 1'b0; // Serialization complete, ready for next flit
                end else begin
                    chunk_cnt <= chunk_cnt + 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire