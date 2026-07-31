`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Sideband Controller (Logical PHY Top)
/// @description Top-level wrapper integrating Flow Control, Encoding, Decoding, 
/// and AFE sequencing for the 800 MT/s Sideband Interface.
module lphy_sb_ctrl (
    input  wire         sb_clk,         // FIXED: Dedicated 800 MHz Sideband Clock
    input  wire         rst_n,          // Active-low reset (VCCAON domain)
    input  wire         rdi_in_reset,   // RDI Reset state indicator for Flow Control
    
    // Local TX Interface (from D2D Adapter via RDI)
    input  wire         tx_req_valid,
    output wire         tx_req_ready,
    input  wire [4:0]   tx_opcode,
    input  wire [2:0]   tx_srcid,
    input  wire [2:0]   tx_dstid,
    input  wire         tx_ep,
    input  wire         tx_cr,
    input  wire [63:0]  tx_payload,
    input  wire [4:0]   tx_tag,
    input  wire [7:0]   tx_be,
    input  wire [23:0]  tx_addr,
    input  wire [2:0]   tx_cp_status,
    input  wire [7:0]   tx_msgcode, 
    input  wire [7:0]   tx_msgsubcode,
    input  wire [15:0]  tx_msginfo,
    input  wire         tx_local_crd_ret,
    
    // Local RX Interface (to D2D Adapter via RDI)
    output logic        rx_req_valid,
    output logic [4:0]  rx_opcode, 
    output logic [2:0]  rx_srcid,
    output logic [2:0]  rx_dstid,
    output logic        rx_ep, 
    output logic        rx_cr, 
    output logic [63:0] rx_payload, 
    output logic [4:0]  rx_tag,
    output logic [7:0]  rx_be, 
    output logic [23:0] rx_addr, 
    output logic [2:0]  rx_cp_status, 
    output logic [7:0]  rx_msgcode,
    output logic [7:0]  rx_msgsubcode, 
    output logic [15:0] rx_msginfo,
    output logic        rx_parity_err,
    
    // PARALLEL SIDEBAND INTERFACE (To discrete AFE Serializer)
    output logic        afe_tx_valid,
    output logic [63:0] afe_tx_data,    
    input  wire         afe_tx_ready,   
    output logic        afe_tx_ipg_en,  // NEW: Commands AFE to insert 32 UI gap

    input  wire         afe_rx_valid,
    input  wire [63:0]  afe_rx_data,    
    output logic        afe_rx_en,
    
    // Advanced Package Redundancy Enables (Compliance)
    output logic        afe_tx_rd_en,   // Enable redundant TX pin
    output logic        afe_rx_rd_en    // Enable redundant RX pin
);

    // =========================================================================
    // Internal Interconnects
    // =========================================================================
    logic tx_allowed;
    logic seq_tx_ready;
    logic fire_encoder;
    
    logic        enc_pkt_valid;
    logic [63:0] enc_pkt_header;
    logic [63:0] enc_pkt_data;
    logic        enc_pkt_has_data;
    logic        enc_pkt_data_is_64b; // FIXED: Added missing wire
    
    logic        dec_pkt_valid;
    logic [63:0] dec_pkt_header;
    logic [63:0] dec_pkt_data;
    
    // Default redundancy to 0 (Software configures this during SBINIT)
    assign afe_tx_rd_en = 1'b0;
    assign afe_rx_rd_en = 1'b0;

    // Decode Opcode for Flow Control
    logic is_reg_req, is_reg_cpl, is_msg;
    always_comb begin
        is_reg_req = (tx_opcode == 5'b00000) || (tx_opcode == 5'b00001) || 
                     (tx_opcode == 5'b00100) || (tx_opcode == 5'b00101) || 
                     (tx_opcode == 5'b01000) || (tx_opcode == 5'b01001) || 
                     (tx_opcode == 5'b01100) || (tx_opcode == 5'b01101);
                 
        is_reg_cpl = (tx_opcode == 5'b10000) || (tx_opcode == 5'b10001) || 
                     (tx_opcode == 5'b11001);
                     
        is_msg     = (tx_opcode == 5'b10010) || (tx_opcode == 5'b11011);
    end
    
    // =========================================================================
    // 1. Flow Control Instance
    // =========================================================================
    lphy_sb_flow_ctrl #(
        .LOCAL_CREDITS_INIT(32),
        .REMOTE_CREDITS_INIT(4),
        .MAX_REMOTE_CREDITS(32)
    ) fc_inst (
        .clk(sb_clk),
        .rst_n(rst_n),
        .rdi_in_reset(rdi_in_reset), 
        .req_valid(tx_req_valid),
        .is_reg_req(is_reg_req),
        .is_reg_cpl(is_reg_cpl), 
        .is_msg(is_msg), 
        .tx_allowed(tx_allowed), 
        .local_crd_ret(tx_local_crd_ret), 
        // Note: Mechanism B (Nop.Crd) extraction logic goes here eventually
        .remote_crd_ret_val({2'b00, (rx_req_valid & rx_cr)}) 
    );
    
    // =========================================================================
    // 2. Packet Encoder Instance
    // =========================================================================
    assign fire_encoder = tx_req_valid & tx_allowed & seq_tx_ready;
    assign tx_req_ready = tx_allowed & seq_tx_ready;
    
    lphy_sb_pkt_enc enc_inst (
        .clk(sb_clk), 
        .rst_n(rst_n), 
        .req_valid(fire_encoder), 
        .tx_ready(seq_tx_ready),   // Wired downstream backpressure
        .req_ready(),              // Ignored at wrapper level
        .opcode(tx_opcode),
        .srcid(tx_srcid), 
        .dstid(tx_dstid),
        .ep(tx_ep),
        .cr(tx_cr), 
        .payload_in(tx_payload), 
        .tag(tx_tag), 
        .be(tx_be), 
        .addr(tx_addr), 
        .cp_status(tx_cp_status), 
        .msgcode(tx_msgcode), 
        .msgsubcode(tx_msgsubcode), 
        .msginfo(tx_msginfo), 
        .pkt_valid(enc_pkt_valid), 
        .pkt_header(enc_pkt_header), 
        .pkt_data(enc_pkt_data), 
        .pkt_has_data(enc_pkt_has_data),
        .pkt_data_is_64b(enc_pkt_data_is_64b) // FIXED
    );
    
    // =========================================================================
    // 3. TX Word Sequencer (With 32 UI Gap Enforcement)
    // =========================================================================
    typedef enum logic [1:0] {ST_TX_IDLE, ST_TX_DATA, ST_TX_IPG} tx_st_t;
    tx_st_t tx_state, tx_next_state;
    
    logic [63:0] hold_tx_data;

    always_ff @(posedge sb_clk or negedge rst_n) begin
        if (!rst_n) tx_state <= ST_TX_IDLE;
        else tx_state <= tx_next_state;
    end
    
    always_comb begin
        tx_next_state = tx_state;
        case (tx_state)
            ST_TX_IDLE: begin
                if (enc_pkt_valid && seq_tx_ready) 
                    tx_next_state = enc_pkt_has_data ? ST_TX_DATA : ST_TX_IPG;
            end
            ST_TX_DATA: begin
                if (afe_tx_ready) 
                    tx_next_state = ST_TX_IPG; // Force gap after payload
            end
            ST_TX_IPG: begin
                if (afe_tx_ready)
                    tx_next_state = ST_TX_IDLE; // Gap satisfied, return to ready
            end
            default: tx_next_state = ST_TX_IDLE;
        endcase
    end
    
    always_ff @(posedge sb_clk or negedge rst_n) begin
        if (!rst_n) begin
            afe_tx_valid  <= 1'b0;
            afe_tx_data   <= 64'h0;
            afe_tx_ipg_en <= 1'b0;
            seq_tx_ready  <= 1'b1;
            hold_tx_data  <= 64'h0;
        end else begin
            case (tx_state)
                ST_TX_IDLE: begin
                    afe_tx_ipg_en <= 1'b0; // Clear gap flag
                    if (enc_pkt_valid && seq_tx_ready) begin
                        afe_tx_valid <= 1'b1;
                        afe_tx_data  <= enc_pkt_header;
                        
                        if (enc_pkt_has_data) begin
                            hold_tx_data <= enc_pkt_data;
                            seq_tx_ready <= 1'b0; 
                        end else begin
                            seq_tx_ready <= 1'b0; // Wait for IPG
                        end
                    end else if (afe_tx_ready) begin
                        afe_tx_valid <= 1'b0;
                    end
                end
                
                ST_TX_DATA: begin
                    if (afe_tx_ready) begin
                        afe_tx_valid <= 1'b1;
                        afe_tx_data  <= hold_tx_data;
                        // Stay blocked, moving to IPG next
                    end
                end
                
                ST_TX_IPG: begin
                    if (afe_tx_ready) begin
                        afe_tx_valid  <= 1'b0;
                        afe_tx_ipg_en <= 1'b1; // Command AFE to hold 0 for 32 UI
                        seq_tx_ready  <= 1'b1; // Ready for next packet
                    end
                end
            endcase
        end
    end
    
    // =========================================================================
    // 4. RX Word Sequencer 
    // =========================================================================
    assign afe_rx_en = 1'b1; 
    
    typedef enum logic {ST_RX_HDR, ST_RX_DATA} rx_st_t;
    rx_st_t rx_state;
    
    logic [63:0] hold_rx_header;
    logic [4:0]  raw_opcode;

    always_ff @(posedge sb_clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state       <= ST_RX_HDR;
            dec_pkt_valid  <= 1'b0;
            dec_pkt_header <= 64'h0;
            dec_pkt_data   <= 64'h0;
            hold_rx_header <= 64'h0;
        end else begin
            dec_pkt_valid <= 1'b0; 
            
            case (rx_state)
                ST_RX_HDR: begin
                    if (afe_rx_valid) begin
                        hold_rx_header <= afe_rx_data;
                        
                        // Peek into the raw header directly
                        if (afe_rx_data[4:0] == 5'b00001 || afe_rx_data[4:0] == 5'b00101 || 
                            afe_rx_data[4:0] == 5'b01001 || afe_rx_data[4:0] == 5'b01101 || 
                            afe_rx_data[4:0] == 5'b10001 || afe_rx_data[4:0] == 5'b11001 || 
                            afe_rx_data[4:0] == 5'b11011) begin
                            rx_state <= ST_RX_DATA; 
                        end else begin
                            dec_pkt_header <= afe_rx_data;
                            dec_pkt_data   <= 64'h0;
                            dec_pkt_valid  <= 1'b1;
                        end
                    end
                end
                
                ST_RX_DATA: begin
                    if (afe_rx_valid) begin
                        dec_pkt_header <= hold_rx_header;
                        dec_pkt_data   <= afe_rx_data;
                        dec_pkt_valid  <= 1'b1;
                        rx_state       <= ST_RX_HDR;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // 5. Packet Decoder Instance (Black-box for now)
    // =========================================================================
    lphy_sb_pkt_dec dec_inst (
        .clk(sb_clk),
        .rst_n(rst_n),
        .pkt_valid(dec_pkt_valid),
        .pkt_header(dec_pkt_header),
        .pkt_data(dec_pkt_data),
        .req_valid(rx_req_valid),
        .opcode(rx_opcode), 
        .srcid(rx_srcid), 
        .dstid(rx_dstid),
        .ep(rx_ep), 
        .cr(rx_cr), 
        .payload_out(rx_payload),
        .tag(rx_tag), 
        .be(rx_be), 
        .addr(rx_addr), 
        .cp_status(rx_cp_status),
        .msgcode(rx_msgcode), 
        .msgsubcode(rx_msgsubcode), 
        .msginfo(rx_msginfo),
        .parity_err(rx_parity_err)
    );

endmodule
`default_nettype wire