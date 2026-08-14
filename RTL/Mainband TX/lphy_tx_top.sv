`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband TX Top Wrapper
/// @description Integrates the Byte Mapper, Pattern Generator, Scrambler, 
/// Valid Framer, and TX Repair Multiplexers into the final AFE boundary.
/// (Optimized with 8-Quadrant Replication Trees for 2 GHz 32nm WLM closure)
module lphy_tx_top #(
    parameter int NUM_LANES = 64        
)(
    input  wire         clk,
    input  wire         rst_n,
    
    // Configuration & LTSSM Controls
    input  wire [1:0]   link_width,     
    input  wire         degrade_x8,     
    input  wire         degrade_upper,  
    input  wire         lane_reversal,  
    input  wire         is_retimer,     
    input  wire         free_run_mode,  
    input  wire         txtrk_en,       
    
    // Pattern & Training Control
    input  wire         tx_training_en, 
    input  wire [1:0]   pattern_sel,    
    input  wire         scrambler_en,   
    input  wire         load_seed,      
    input  wire [22:0]  lane_seeds [63:0], 
    
    // Repair Control
    input  wire         repair_en,      
    input  wire [63:0]  ext_lane_failed_map, 
    output wire         unrepairable,   
    
    // Interface from D2D Adapter (RDI)
    input  wire         lp_valid, 
    input  wire         lp_irdy, 
    output wire         pl_trdy, 
    input  wire [511:0] lp_data, 
    input  wire         credit_return,  
    
    // PARALLEL AFE BOUNDARY 
    output logic [7:0]  TXDATA [63:0],  
    output logic [63:0] TXDATA_OE,      
    output logic [7:0]  TXVLD,          
    output logic [7:0]  TXRD [3:0],     
    output logic [3:0]  TXRD_OE,        
    output logic        tx_clock_en,    
    output logic        tx_track_en     
);

    // =========================================================================
    // Internal Interconnects
    // =========================================================================
    wire [7:0]  mapped_lane_data [63:0];    
    wire        mapped_lane_valid;
    wire        active_valid;           
    
    wire [7:0]  tx_valid_frame;
    wire [7:0]  pattern_data [63:0];
    logic [7:0] pre_scramble_data [63:0];
    wire [7:0]  raw_scrambled_data [63:0];
    
    wire [7:0]  repaired_lane_data [63:0];
    wire [7:0]  repaired_txrd_data [3:0];
    wire [63:0] repaired_lane_oe;
    wire [3:0]  repaired_txrd_oe;

    // -------------------------------------------------------------------------
    // 1. BYTE-TO-LANE MAPPER 
    // -------------------------------------------------------------------------
    lphy_byte_lane_map byte_mapper_inst (
        .clk(clk), .rst_n(rst_n), .link_width(link_width), .degrade_x8(degrade_x8),
        .degrade_upper(degrade_upper), .lane_reversal(lane_reversal),
        .lp_valid(lp_valid), .lp_irdy(lp_irdy), .pl_trdy(pl_trdy), 
        .lp_data(lp_data), .lane_valid(mapped_lane_valid), .lane_data(mapped_lane_data)
    );
    
    assign active_valid = mapped_lane_valid | tx_training_en;

    // =========================================================================
    // PIPELINE STAGE 1: Control Signal 8-Quadrant Replication Tree
    // =========================================================================
    // A single register driving 192 loads triggers massive enclosed WLM penalties. 
    // We replicate the registers 8 times. Each register drives exactly 8 lanes.
    // The 'dont_touch' pragma strictly forbids the -retime engine from merging them.
    (* dont_touch = "true" *) logic [7:0] pipe_tx_training_en;
    (* dont_touch = "true" *) logic [7:0] pipe_scrambler_en;
    (* dont_touch = "true" *) logic [7:0] pipe_active_valid;
    (* dont_touch = "true" *) logic [7:0] pipe_load_seed;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_tx_training_en <= 8'h0;
            pipe_scrambler_en   <= 8'h0;
            pipe_active_valid   <= 8'h0;
            pipe_load_seed      <= 8'h0;
        end else begin
            for (int i = 0; i < 8; i++) begin
                pipe_tx_training_en[i] <= tx_training_en;
                pipe_scrambler_en[i]   <= scrambler_en;
                pipe_active_valid[i]   <= active_valid;
                pipe_load_seed[i]      <= load_seed;
            end
        end
    end

    // -------------------------------------------------------------------------
    // PIPELINE ALIGNMENT STAGE (Datapath)
    // -------------------------------------------------------------------------
    logic [7:0] mapped_lane_data_q [63:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 64; i++) mapped_lane_data_q[i] <= 8'h00;
        end else begin
            for (int i = 0; i < 64; i++) mapped_lane_data_q[i] <= mapped_lane_data[i];
        end
    end
    
    // -------------------------------------------------------------------------
    // 2. VALID FRAMER
    // -------------------------------------------------------------------------
    wire is_valtrain = tx_training_en && (pattern_sel == 2'b10);
    
    lphy_valid_framer valid_framer_inst (
        .clk(clk), .rst_n(rst_n), .is_retimer(is_retimer), .train_mode(is_valtrain),
        .lane_valid(active_valid), .credit_return(credit_return), .valid_frame_out(tx_valid_frame)
    );
    
    // -------------------------------------------------------------------------
    // 3. TRAINING PATTERN GENERATOR ARRAY & MUX
    // -------------------------------------------------------------------------
    genvar pg;
    generate
        for (pg = 0; pg < 64; pg++) begin : gen_pattern_gens
            // Map the lane to 1 of the 8 replicated registers (e.g., Lane 21 uses flop 2)
            lphy_pattern_gen pattern_gen_inst (
                .clk(clk), .rst_n(rst_n), .lane_id(8'(pg)), .pattern_sel(pattern_sel), 
                .enable(pipe_tx_training_en[pg / 8]), 
                .pattern_out(pattern_data[pg])
            );
        end
    endgenerate

    always_comb begin
        for (int j = 0; j < 64; j++) begin
            if (pipe_tx_training_en[j / 8]) begin 
                pre_scramble_data[j] = pattern_data[j];
            end else begin
                pre_scramble_data[j] = mapped_lane_data_q[j];
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // 4. SCRAMBLER ARRAY
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < 64; i++) begin : gen_scramblers
            // Logic explicitly driven by the geographically closest replicated quadrant flop
            wire scrambler_fire = pipe_scrambler_en[i / 8] & pipe_active_valid[i / 8] & ~pipe_tx_training_en[i / 8];

            lphy_scrambler scrambler_inst (
                .clk(clk), .rst_n(rst_n), .enable(scrambler_fire), 
                .load_seed(pipe_load_seed[i / 8]), .seed_in(lane_seeds[i]), 
                .data_in(pre_scramble_data[i]), .data_out(raw_scrambled_data[i]) 
            );
        end
    endgenerate
    
    // =========================================================================
    // PIPELINE STAGE 2: Datapath Isolation & Alignment
    // =========================================================================
    logic [7:0] pipe_scrambled_data [63:0];
    logic [7:0] pipe_tx_valid_frame;
    logic       pipe_active_valid_boundary; // Merged back for AFE logic

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_tx_valid_frame <= 8'h00;
            pipe_active_valid_boundary <= 1'b0;
        end else begin
            pipe_tx_valid_frame <= tx_valid_frame;
            pipe_active_valid_boundary <= pipe_active_valid[0]; // Any bit is fine here
        end
    end

    always_ff @(posedge clk) begin
        for (int k = 0; k < 64; k++) begin
            pipe_scrambled_data[k] <= raw_scrambled_data[k];
        end
    end

    // -------------------------------------------------------------------------
    // 5. TX REPAIR MULTIPLEXER
    // -------------------------------------------------------------------------
    logic [63:0] tx_lane_failed_map;
    assign tx_lane_failed_map = repair_en ? ext_lane_failed_map : 64'h0;    
    
    lphy_repair_tx tx_repair_inst (
        .tx_logical_data(pipe_scrambled_data), .lane_failed(tx_lane_failed_map), 
        .tx_physical_data(repaired_lane_data), .tx_redundant_data(repaired_txrd_data),
        .tx_physical_oe(repaired_lane_oe), .tx_redundant_oe(repaired_txrd_oe), .unrepairable(unrepairable)
    );
    
    // -------------------------------------------------------------------------
    // 6. AFE BOUNDARY PIPELINE
    // -------------------------------------------------------------------------
    logic [3:0] postamble_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            TXVLD <= 8'h00;
            for (int k = 0; k < 64; k++) TXDATA[k] <= 8'h00;
            TXDATA_OE <= 64'hFFFF_FFFF_FFFF_FFFF; 
            for (int k = 0; k < 4; k++) TXRD[k] <= 8'h00;
            TXRD_OE <= 4'h0;
            tx_clock_en <= 1'b0;
            tx_track_en <= 1'b0;
            postamble_cnt <= 4'd2;
        end else begin
            TXVLD     <= pipe_tx_valid_frame;
            TXDATA_OE <= repaired_lane_oe;
            TXRD_OE   <= repaired_txrd_oe;
            for (int k = 0; k < 64; k++) TXDATA[k] <= repaired_lane_data[k];
            for (int k = 0; k < 4; k++)  TXRD[k]   <= repaired_txrd_data[k];
            tx_track_en <= txtrk_en;
            
            if (pipe_active_valid_boundary) begin 
                postamble_cnt <= 4'd0;
                tx_clock_en   <= 1'b1;
            end else if (postamble_cnt < 4'd2) begin
                postamble_cnt <= postamble_cnt + 1'b1;
                tx_clock_en   <= 1'b1; 
            end else begin
                tx_clock_en   <= free_run_mode; 
            end
        end
    end
endmodule
`default_nettype wire