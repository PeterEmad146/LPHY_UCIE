`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Sideband Parity Generator and Checker
/// @description Computes and verifies Control Parity (CP) and Data Parity (DP) 
/// for the 64-bit sideband header according to the UCIe 1.0 Specification.
module lphy_sb_parity (
    // TX Sideband Parity Generation
    input  wire  [63:0] tx_header_in,   // 64-bit Header (Phase 0 and Phase 1)
    input  wire  [63:0] tx_data_in,     // 64-bit Data Payload (Phase 2 and Phase 3)
    input  wire         tx_has_data,    // High if the packet type includes a data payload
    input  wire         tx_data_is_64b, // High for 64-bit payload, Low for 32-bit payload
    
    output wire  [63:0] tx_header_out,  // Header with DP (Bit 63) and CP (Bit 62) populated
    
    // RX Sideband Parity Checking
    input  wire  [63:0] rx_header_in,
    input  wire  [63:0] rx_data_in,
    input  wire         rx_has_data,
    input  wire         rx_data_is_64b,
    
    // Error flags (Combinational - must be registered by top-level FSM)
    output wire         rx_cp_err,      // High if Control Parity mismatch
    output wire         rx_dp_err       // High if Data Parity mismatch
);

    // Physical bit locations within the 64-bit header (Phase 1)
    localparam int DP_BIT = 63;
    localparam int CP_BIT = 62;

    // =========================================================================
    // TX Logic: Parity Generation
    // =========================================================================
    wire        tx_cp_gen;
    wire        tx_dp_gen;
    wire [63:0] masked_tx_data;

    // Mask the upper 32 bits if the payload is only 32 bits to prevent parity corruption
    assign masked_tx_data = tx_data_is_64b ? tx_data_in : {32'd0, tx_data_in[31:0]};

    // DP: Even parity of data payload. 0b if no data payload.
    assign tx_dp_gen = tx_has_data ? ^masked_tx_data : 1'b0;
    
    // CP: Even parity of all header bits excluding DP and CP themselves (Bits 61:0)
    // Note: Bit 4 (EP/Poison) is inherently included in this calculation per spec.
    assign tx_cp_gen = ^tx_header_in[61:0];
    
    // Assemble final header
    assign tx_header_out = {tx_dp_gen, tx_cp_gen, tx_header_in[61:0]};
    

    // =========================================================================
    // RX Logic: Parity Verification
    // =========================================================================
    wire        rx_cp_calc;
    wire        rx_dp_calc;
    wire [63:0] masked_rx_data;

    // Mask incoming data identically to TX logic
    assign masked_rx_data = rx_data_is_64b ? rx_data_in : {32'd0, rx_data_in[31:0]};

    // Calculate expected parities from incoming streams
    assign rx_dp_calc = rx_has_data ? ^masked_rx_data : 1'b0;
    assign rx_cp_calc = ^rx_header_in[61:0];
    
    // Check calculated parity against received parity bits using logical inequality
    assign rx_dp_err = (rx_header_in[DP_BIT] != rx_dp_calc);
    assign rx_cp_err = (rx_header_in[CP_BIT] != rx_cp_calc);

endmodule
`default_nettype wire