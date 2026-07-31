`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Sideband Packet Decoder
/// @description Extracts header fields, routes payload, and verifies parity
/// for incoming 800 MT/s sideband packets.
module lphy_sb_pkt_dec (
    input  wire         clk,
    input  wire         rst_n,
    
    // Input from RX Deserializer (via Wrapper)
    input  wire         pkt_valid,
    input  wire [63:0]  pkt_header,
    input  wire [63:0]  pkt_data,
    
    // Common Decoded Fields
    output logic        req_valid,
    output logic [4:0]  opcode,
    output logic [2:0]  srcid,
    output logic [2:0]  dstid,
    output logic        ep,
    output logic        cr,
    output logic [63:0] payload_out,
    
    // Register Access Fields
    output logic [4:0]  tag,
    output logic [7:0]  be,
    output logic [23:0] addr,
    output logic [2:0]  cp_status,
    
    // Message Specific Fields
    output logic [7:0]  msgcode,
    output logic [7:0]  msgsubcode,
    output logic [15:0] msginfo,
    
    // Error Flags 
    output logic        parity_err
);

    // Internal signals for combinatorial decoding 
    wire [31:0] phase0;
    wire [31:0] phase1;
    
    wire [4:0]  dec_opcode;
    logic       is_reg_req;
    logic       is_reg_cpl;
    logic       is_msg;
    logic       has_data;
    logic       is_64b_data;
    
    wire        rx_cp_err;
    wire        rx_dp_err;
    
    // Split header into phases
    assign phase1 = pkt_header[63:32];
    assign phase0 = pkt_header[31:0];
    
    // Opcode is always at the same location of phase 0
    assign dec_opcode = phase0[4:0];
    
    // Decode Opcode groups
    always_comb begin
        is_reg_req = (dec_opcode == 5'b00000) || (dec_opcode == 5'b00001) || 
                     (dec_opcode == 5'b00100) || (dec_opcode == 5'b00101) || 
                     (dec_opcode == 5'b01000) || (dec_opcode == 5'b01001) || 
                     (dec_opcode == 5'b01100) || (dec_opcode == 5'b01101);
                     
        is_reg_cpl = (dec_opcode == 5'b10000) || (dec_opcode == 5'b10001) || 
                     (dec_opcode == 5'b11001);
                     
        is_msg     = (dec_opcode == 5'b10010) || (dec_opcode == 5'b11011);
        
        has_data   = (dec_opcode == 5'b00001) || (dec_opcode == 5'b00101) || 
                     (dec_opcode == 5'b01001) || (dec_opcode == 5'b01101) || 
                     (dec_opcode == 5'b10001) || (dec_opcode == 5'b11001) || 
                     (dec_opcode == 5'b11011);
                     
        is_64b_data = (dec_opcode == 5'b01001) || // 64b Mem Write
                      (dec_opcode == 5'b01101) || // 64b Cfg Write
                      (dec_opcode == 5'b11001) || // Cpl with 64b Data
                      (dec_opcode == 5'b11011);   // Message with 64b Data
    end
    
    // Instantiate Parity Checker
    lphy_sb_parity parity_checker (
        // TX ports tied off (this instance is RX only)
        .tx_header_in   (64'h0),
        .tx_data_in     (64'h0),
        .tx_has_data    (1'b0),
        .tx_data_is_64b (1'b0),
        .tx_header_out  (),
        
        // RX Evaluation
        .rx_header_in   (pkt_header),
        .rx_data_in     (pkt_data),
        .rx_has_data    (has_data),
        .rx_data_is_64b (is_64b_data),
        .rx_cp_err      (rx_cp_err),
        .rx_dp_err      (rx_dp_err)
    );
    
    // Synchronous output assignment 
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            req_valid   <= 1'b0;
            opcode      <= 5'h0;
            srcid       <= 3'h0;
            dstid       <= 3'h0;
            ep          <= 1'b0;
            cr          <= 1'b0;
            payload_out <= 64'h0;
            tag         <= 5'h0;
            be          <= 8'h0;
            addr        <= 24'h0;
            cp_status   <= 3'h0;
            msgcode     <= 8'h0;
            msgsubcode  <= 8'h0;
            msginfo     <= 16'h0;
            parity_err  <= 1'b0;
        end else if (pkt_valid) begin
            req_valid   <= 1'b1;
            opcode      <= dec_opcode;
            payload_out <= pkt_data;
            parity_err  <= rx_cp_err | rx_dp_err; // Consolidate parity errors
            
            // Common decode for srcid
            srcid <= phase0[31:29];
            
            if (is_msg) begin
                // Message Format 
                dstid      <= phase1[26:24];
                msgcode    <= phase0[21:14];
                msginfo    <= phase1[23:8];
                msgsubcode <= phase1[7:0];
                
                // Zero out unused fields to prevent ghost data propagation
                ep         <= 1'b0; 
                cr         <= 1'b0; 
                tag        <= 5'h0; 
                be         <= 8'h0; 
                addr       <= 24'h0;
                cp_status  <= 3'h0;
            end else if (is_reg_cpl) begin
                // Register Access Completions
                dstid      <= phase1[26:24];
                cr         <= phase1[29];
                tag        <= phase0[26:22];
                be         <= phase0[21:14];
                ep         <= phase0[5];
                cp_status  <= phase1[2:0];
                
                // Zero out unused fields
                addr       <= 24'h0;
                msgcode    <= 8'h0;
                msginfo    <= 16'h0;
                msgsubcode <= 8'h0;
            end else begin
                // Register Access Requests 
                dstid      <= phase1[26:24];
                cr         <= phase1[29];
                tag        <= phase0[26:22];
                be         <= phase0[21:14];
                ep         <= phase0[5];
                addr       <= phase1[23:0];
                
                // Zero out unused fields
                cp_status  <= 3'h0;
                msgcode    <= 8'h0;
                msginfo    <= 16'h0;
                msgsubcode <= 8'h0;
            end
        end else begin
            // Clear valid and errors; hold data payload to reduce switching power
            req_valid  <= 1'b0;
            parity_err <= 1'b0;
        end
    end
endmodule
`default_nettype wire