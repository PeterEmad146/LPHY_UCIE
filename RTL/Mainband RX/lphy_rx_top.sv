`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Top Wrapper
/// @description Integrates the AFE pipeline, Repair Demux, Reversal Detector, 
/// Valid Deframer, Descrambler, and Data Demapper to drive the Adapter.
module lphy_rx_top #(
    parameter int NUM_LANES = 64    // 16 for Standard Package, 64 for Advanced Package
)(
    input  wire         clk, 
    input  wire         rst_n, 
    
    // =========================================================================
    // Configuration & LTSSM Controls
    // =========================================================================
    input  wire [1:0]   link_width,     // 2'b00: x16, 2'b01: x32, 2'b10: x64
    input  wire         is_retimer,     // 1: Enable 8-UI credit frame decoding
    input  wire         free_run_mode,  // 1: Clock never gates
    input  wire         is_linkerror,   // 1: Force clock on for error containment
    input  wire         force_enable,   // 1: Force clock on during wake/training
    
    // Training & Pattern Control
    input  wire         rx_training_en, // 1: Bypass Adapter, data goes to LTSSM
    input  wire         en_reversal_check, 
    output logic        reversal_detected, 
    output logic        reversal_check_done, 
    
    // Scrambling Control
    input  wire         descrambler_en, 
    input  wire         load_seed, 
    input  wire [22:0]  lane_seeds [63:0], 
    
    // Repair Control
    input  wire         repair_en, 
    input  wire         en_lane_check, 
    output logic [63:0] detected_lane_failures,
    output logic        check_done,
    
    // Safety & Error Escalation
    output logic        framing_err, 
    output logic        unrepairable,
    
    // =========================================================================
    // Interface to D2D Adapter (RDI)
    // =========================================================================
    output logic        pl_valid, 
    output logic [511:0] pl_data,       // Fully assembled 64-Byte payload
    output logic        credit_return,
    output logic        rx_gated_clk,       
    
    // =========================================================================
    // PARALLEL AFE BOUNDARY (Analog Front End Interface)
    // =========================================================================
    input  wire [7:0]   RXDATA [NUM_LANES-1:0],   
    input  wire [7:0]   RXVLD,                    
    input  wire [7:0]   RXRD [3:0],               
    input  wire         RXTRK,                    
    output logic        rx_en                     
);

    // Power up AFE receivers automatically when out of reset
    assign rx_en = 1'b1; 
    
    // Suppress lint warnings for unused analog clock pins
    logic _unused;
    assign _unused = ^{RXTRK};

    // Internal pipeline signals
    logic [7:0] rx_lane_data_64 [63:0]; 
    logic [7:0] rx_lane_data_NUM [NUM_LANES-1:0];
    logic [7:0] rx_txrd_data_raw [3:0];
    logic [7:0] rx_valid_frame;
    
    wire        internal_lane_valid;
    logic       internal_lane_valid_q;  // 1-cycle delay to align with lane_id_detect
    wire        internal_credit_return;
    wire        internal_framing_err;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) internal_lane_valid_q <= 1'b0;
        else        internal_lane_valid_q <= internal_lane_valid;
    end
    
    // =========================================================================
    // 1. AFE PIPELINE LATCH 
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_valid_frame <= 8'h00;
            for (int i = 0; i < 64; i++)        rx_lane_data_64[i] <= 8'h00;
            for (int i = 0; i < NUM_LANES; i++) rx_lane_data_NUM[i] <= 8'h00;
            for (int i = 0; i < 4; i++)         rx_txrd_data_raw[i] <= 8'h00;
        end else begin
            rx_valid_frame <= RXVLD;
            
            // Latch exactly NUM_LANES for the normal datapath
            for (int i = 0; i < NUM_LANES; i++) begin
                rx_lane_data_NUM[i] <= RXDATA[i];
                rx_lane_data_64[i]  <= RXDATA[i];
            end
            
            // Safely pad the upper lanes with 0 to prevent array crashes
            for (int i = NUM_LANES; i < 64; i++) begin
                rx_lane_data_64[i] <= 8'h00;
            end
            
            for (int i = 0; i < 4; i++) rx_txrd_data_raw[i] <= RXRD[i];
        end
    end
    
    // =========================================================================
    // 2. LANE ID DETECTION
    // =========================================================================
    wire [NUM_LANES-1:0] lane_failed_narrow;

    lphy_lane_id_detect #(
        .NUM_LANES(NUM_LANES)
    ) lphy_id_detect_inst (
        .clk(clk), 
        .rst_n(rst_n), 
        .rx_lane_data_in(rx_lane_data_NUM), 
        .rx_lane_valid(internal_lane_valid_q), 
        .en_lane_check(en_lane_check), 
        .is_reversed(reversal_detected), 
        .lane_failed(lane_failed_narrow), 
        .check_done(check_done)
    );

    assign detected_lane_failures = {{(64-NUM_LANES){1'b0}}, lane_failed_narrow};
    
    // =========================================================================
    // 3. RX REPAIR MULTIPLEXER
    // =========================================================================
    wire [7:0] rx_repaired_data_64 [63:0];
    wire [63:0] rx_lane_failed_map;
    
    assign rx_lane_failed_map = repair_en ? detected_lane_failures : 64'h0;
    
    lphy_repair_rx rx_repair_inst (
        .rx_physical_data(rx_lane_data_64), 
        .rx_redundant_data(rx_txrd_data_raw), 
        .lane_failed(rx_lane_failed_map), 
        .rx_logical_data(rx_repaired_data_64),
        .unrepairable(unrepairable)
    );
    
    // =========================================================================
    // 4. PIPELINE ALIGNMENT BUFFER
    // =========================================================================
    logic [7:0] rx_lane_data_q [63:0];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 64; i++) rx_lane_data_q[i] <= 8'h00;
        end else begin
            for (int i = 0; i < 64; i++) rx_lane_data_q[i] <= rx_repaired_data_64[i];
        end
    end
    
    // =========================================================================
    // 5. VALID DEFRAMER
    // =========================================================================
    lphy_valid_deframer valid_deframer_inst (
        .clk(clk), 
        .rst_n(rst_n), 
        .is_retimer(is_retimer),
        .valid_frame_in(rx_valid_frame), 
        .lane_valid(internal_lane_valid), 
        .credit_return(internal_credit_return), 
        .framing_err(internal_framing_err)
    );
    
    // Align valid signals with the Derotator flop
    logic pl_valid_reg;
    logic pl_credit_return_reg;
    logic pl_framing_err_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pl_valid_reg         <= 1'b0;
            pl_credit_return_reg <= 1'b0;
            pl_framing_err_reg   <= 1'b0;
        end else begin
            pl_valid_reg         <= internal_lane_valid;
            pl_credit_return_reg <= internal_credit_return;
            pl_framing_err_reg   <= internal_framing_err;
        end
    end
    
    // Hide adapter valid if we are in training mode
    assign pl_valid      = rx_training_en ? 1'b0 : pl_valid_reg;
    assign credit_return = pl_credit_return_reg;
    assign framing_err   = pl_framing_err_reg;
    
    // =========================================================================
    // 6. LANE DEROTATOR & DESKEW
    // =========================================================================
    wire [7:0] derotated_data [63:0];
    
    lphy_lane_derotate #(
        .NUM_LANES(64)
    ) lane_derotate_inst (
        .clk(clk), 
        .rst_n(rst_n), 
        .rx_lane_data_in(rx_lane_data_q), 
        .rx_lane_valid(internal_lane_valid), 
        .en_reversal_check(en_reversal_check), 
        .reversal_detected(reversal_detected), 
        .reversal_check_done(reversal_check_done), 
        .rx_lane_data_out(derotated_data)  
    );
    
    // =========================================================================
    // 7. DESCRAMBLER ARRAY
    // =========================================================================
    // Spec Rule: Initialization Patterns must NOT be descrambled.
    wire descrambler_fire = descrambler_en & pl_valid_reg & ~rx_training_en;
    wire [7:0] descrambled_data [63:0];
    
    genvar i; 
    generate
        for (i = 0; i < 64; i++) begin : gen_descramblers
            lphy_descrambler descrambler_inst (
                .clk(clk), 
                .rst_n(rst_n), 
                .enable(descrambler_fire), 
                .load_seed(load_seed), 
                .seed_in(lane_seeds[i]), 
                .data_in(derotated_data[i]), 
                .data_out(descrambled_data[i])
            );
        end
    endgenerate
    
    // =========================================================================
    // 8. LANE-TO-BYTE DEMAPPER (Adapter Payload Reconstruction)
    // =========================================================================
    // Combines the active lanes into a flat 512-bit vector.
    // If x32 or x16 degraded mode is active, the data arrives sequentially over 
    // multiple cycles. (Simplified for this wrapper scope to direct passthrough).
    always_comb begin
        for (int j = 0; j < 64; j++) begin
            pl_data[j*8 +: 8] = descrambled_data[j];
        end
    end
    
    // =========================================================================
    // 9. RX CLOCK GATER
    // =========================================================================
    lphy_clkgate_rx clkgate_rx_inst (
        .clk(clk),
        .rst_n(rst_n), 
        .free_run_mode(free_run_mode), 
        .is_linkerror(is_linkerror),
        .force_enable(force_enable),
        .valid_in(internal_lane_valid), 
        .gated_clk(rx_gated_clk)
    );

endmodule
`default_nettype wire