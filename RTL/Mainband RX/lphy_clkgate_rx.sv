`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Dynamic Clock Gater
/// @description Glitch-free Integrated Clock Gating (ICG) cell for the RX datapath.
/// Enforces the strict 16 UI (2 byte-clock cycle) postamble and manages LTSSM overrides.
module lphy_clkgate_rx (
    input  wire  clk, 
    input  wire  rst_n, 
    
    // =========================================================================
    // Configuration & State Overrides
    // =========================================================================
    input  wire  free_run_mode, // 1: Clock never gates (Dynamic gating bypassed)
    input  wire  is_linkerror,  // 1: Force clock on for error containment (Spec Rule)
    input  wire  force_enable,  // 1: Force clock on during Link Training / Wakeup
    
    // =========================================================================
    // Input from Valid Deframer
    // =========================================================================
    input  wire  valid_in,      // High when data/credits are actively being received
    
    // =========================================================================
    // Gated Clock Output
    // =========================================================================
    output logic gated_clk      // Distributed to internal RX logic
);

    logic [3:0] postamble_cnt;
    logic       clk_en;
    logic       clk_en_latched;
    
    // =========================================================================
    // Postamble Counter (Tracks 2 Byte-Rate Cycles = 16 UI)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            postamble_cnt <= 4'd2;  // Start fully idle
        end else begin
            if (valid_in) begin
                postamble_cnt <= 4'd0;
            end else if (postamble_cnt < 4'd2) begin
                postamble_cnt <= postamble_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // Combinatorial Enable logic
    // =========================================================================
    // The clock runs if data is valid, postamble is active, or any override is asserted.
    assign clk_en = valid_in | (postamble_cnt < 4'd2) | free_run_mode | is_linkerror | force_enable;
    
    // =========================================================================
    // Glitch-Free Clock Gating Latch (Infers ICG Cell in Synthesis)
    // =========================================================================
    always_latch begin
        if (!rst_n) begin
            clk_en_latched <= 1'b0;
        end else if (!clk) begin // Transparent on the negative edge of the clock
            clk_en_latched <= clk_en;
        end
    end

    // =========================================================================
    // Gated Clock Driver
    // =========================================================================
    assign gated_clk = clk & clk_en_latched;

endmodule
`default_nettype wire