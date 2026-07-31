`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Sideband Packet Encoder
/// @description Assembles Phase 0 and Phase 1 headers based on opcode and 
/// instantiates the Parity Generator before passing to the TX pipeline.
module lphy_sb_pkt_enc(
    input  wire         clk,
    input  wire         rst_n,
    
    // Handshake (Upstream from Adapter via RDI)
    input  wire         req_valid, 
    output wire         req_ready,
    
    // Handshake (Downstream to TX Serializer / Credit Manager)
    input  wire         tx_ready,
    
    // Common Packet Fields
    input  wire  [4:0]  opcode,       // 5-bit opcode (Table 47)
    input  wire  [2:0]  srcid,        // 3-bit Source ID
    input  wire  [2:0]  dstid,        // 3-bit Destination ID
    input  wire         ep,           // Data Poison bit
    input  wire         cr,           // Credit return bit
    input  wire  [63:0] payload_in,   // 64-bit Data Payload (if applicable)
    
    // Register Access Specific Fields
    input  wire  [4:0]  tag,          // 5-bit Request Tag
    input  wire  [7:0]  be,           // 8-bit Byte Enables
    input  wire  [23:0] addr,         // 24-bit Address (Requests only)
    input  wire  [2:0]  cp_status,    // 3-bit Completion Status (Completions Only)
    
    // Message Specific Fields
    input  wire  [7:0]  msgcode,      // 8-bit Message Code
    input  wire  [7:0]  msgsubcode,   // 8-bit Message Subcode
    input  wire  [15:0] msginfo,      // 16-bit Message Info
    
    // Formatted Output to Sideband TX Serializer
    output logic        pkt_valid,
    output logic [63:0] pkt_header,
    output logic [63:0] pkt_data,
    output logic        pkt_has_data,
    output logic        pkt_data_is_64b 
);

    logic is_reg_req;
    logic is_reg_cpl;
    logic is_msg;
    logic has_data;
    logic is_64b_data;
    
    logic [31:0] phase0_reg;
    logic [31:0] phase1_reg;
    logic [63:0] raw_header;
    logic [63:0] calc_header;
    
    // Decode Opcode to determine packet type and data sizing (Table 47)
    always_comb begin
        is_msg = (opcode == 5'b10010) || (opcode == 5'b11011);
        
        has_data = (opcode == 5'b00001) ||  // 32b Mem Write
                   (opcode == 5'b00101) ||  // 32b Cfg Write
                   (opcode == 5'b01001) ||  // 64b Mem Write
                   (opcode == 5'b01101) ||  // 64b Cfg Write
                   (opcode == 5'b10001) ||  // Cpl with 32b Data
                   (opcode == 5'b11001) ||  // Cpl with 64b Data
                   (opcode == 5'b11011);    // Message with 64b Data
                   
        is_64b_data = (opcode == 5'b01001) || // 64b Mem Write
                      (opcode == 5'b01101) || // 64b Cfg Write
                      (opcode == 5'b11001) || // Cpl with 64b Data
                      (opcode == 5'b11011);   // Message with 64b Data
                   
        is_reg_req = (opcode == 5'b00000) || (opcode == 5'b00001) || 
                     (opcode == 5'b00100) || (opcode == 5'b00101) ||
                     (opcode == 5'b01000) || (opcode == 5'b01001) ||
                     (opcode == 5'b01100) || (opcode == 5'b01101);
       
        is_reg_cpl = (opcode == 5'b10000) || (opcode == 5'b10001) ||
                     (opcode == 5'b11001);
    end
    
    // Assemble the 64-bit Header (Phase 0 and Phase 1)
    always_comb begin
        if (is_msg) begin
            // Message Format
            phase0_reg = {srcid, 7'h00, msgcode, 9'h000, opcode};
            phase1_reg = {2'b00, 3'b0, dstid, msginfo, msgsubcode};
        end else if (is_reg_cpl) begin
            // Register Access Completions
            phase0_reg = {srcid, 2'b00, tag, be, 8'h00, ep, opcode};
            phase1_reg = {2'b00, cr, 2'b00, dstid, 21'b0, cp_status};
        end else begin
            // Register Access Requests 
            phase0_reg = {srcid, 2'b00, tag, be, 8'h00, ep, opcode};
            phase1_reg = {2'b00, cr, 2'b00, dstid, addr};
        end
        raw_header = {phase1_reg, phase0_reg};
    end
    
    // Instantiate Phase 1 Parity Calculator (from previous module)
    lphy_sb_parity parity_calc (
        .tx_header_in   (raw_header),
        .tx_data_in     (payload_in),
        .tx_has_data    (has_data),
        .tx_data_is_64b (is_64b_data),
        .tx_header_out  (calc_header),
        
        // RX signals tied off (this instance is TX only)
        .rx_header_in   (64'h0),
        .rx_data_in     (64'h0),
        .rx_has_data    (1'b0),
        .rx_data_is_64b (1'b0),
        .rx_cp_err      (), // Left open explicitly
        .rx_dp_err      ()  // Left open explicitly
    );
    
    // Pipeline output assignments
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pkt_valid       <= 1'b0;
            pkt_header      <= 64'b0;
            pkt_data        <= 64'h0;
            pkt_has_data    <= 1'b0;
            pkt_data_is_64b <= 1'b0;
        end else if (tx_ready) begin
            pkt_valid       <= req_valid;
            pkt_header      <= req_valid ? calc_header : 64'b0;
            pkt_data        <= req_valid ? payload_in  : 64'h0;
            pkt_has_data    <= req_valid ? has_data    : 1'b0;
            pkt_data_is_64b <= req_valid ? is_64b_data : 1'b0;
        end
    end
    
    // Backpressure enforcement
    assign req_ready = tx_ready;
    
endmodule
`default_nettype wire