`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband TX Top Wrapper
/// @description Integrates the Byte Mapper, Pattern Generator, Scrambler, 
/// Valid Framer, and TX Repair Multiplexers into the final AFE boundary.
module lphy_tx_top #(
    parameter int NUM_LANES = 64        // 64 for Advanced Package, 16 for Standard
)(
    input  wire         clk,
    input  wire         rst_n,
    
    // =========================================================================
    // Configuration & LTSSM Controls
    // =========================================================================
    input  wire [1:0]   link_width,     // 2'b00: x16, 2'b01: x32, 2'b10: x64
    input  wire         degrade_x8,     // High to enable x8 degraded mode
    input  wire         degrade_upper,  // High to map to upper lanes [15:8]
    input  wire         lane_reversal,  // High to reverse logical lane order
    input  wire         is_retimer,     // High to enable 8-UI credit overloading
    input  wire         free_run_mode,  // 1: Clock never gates, 0: Dynamic gating
    input  wire         txtrk_en,       // 1: TXTRK carries Phase-1 replica
    
    // Pattern & Training Control
    input  wire         tx_training_en, // 1: Send training pattern, 0: Adapter data
    input  wire [1:0]   pattern_sel,    // 00:None, 01:LaneID, 10:VALTRAIN, 11:ClkRepair
    input  wire         scrambler_en,   // High during MBTRAIN and ACTIVE
    input  wire         load_seed,      // Pulled high to load initial seeds
    input  wire [22:0]  lane_seeds [63:0], 
    
    // Repair Control
    input  wire         repair_en,      // 1: Redundancy routing active
    input  wire [63:0]  ext_lane_failed_map, 
    output wire         unrepairable,   // Fatal error escalation to LTSSM
    
    // =========================================================================
    // Interface from D2D Adapter (RDI)
    // =========================================================================
    input  wire         lp_valid, 
    input  wire         lp_irdy, 
    output wire         pl_trdy, 
    input  wire [511:0] lp_data, 
    input  wire         credit_return,  // From Sideband Flow Control
    
    // =========================================================================
    // PARALLEL AFE BOUNDARY (Analog Front End Interface)
    // =========================================================================
    output logic [7:0]  TXDATA [63:0],  // 8-bit parallel data per lane
    output logic [63:0] TXDATA_OE,      // 1: Drive, 0: Tri-State (Broken Bump)
    
    output logic [7:0]  TXVLD,          // 8-bit parallel valid frame
    
    output logic [7:0]  TXRD [3:0],     // 8-bit parallel redundant data
    output logic [3:0]  TXRD_OE,        // 1: Drive Redundant bump, 0: Tri-State
    
    output logic        tx_clock_en,    // AFE Clock Driver Enable
    output logic        tx_track_en     // AFE Track Driver Enable
);

    // =========================================================================
    // Internal Interconnects
    // =========================================================================
    wire [7:0]  mapped_lane_data [63:0];    
    wire        mapped_lane_valid;
    wire        active_valid;           // Merges adapter valid and training valid
    
    wire [7:0]  tx_valid_frame;
    
    wire [7:0]  pattern_data [63:0];
    logic [7:0] pre_scramble_data [63:0];
    wire [7:0]  scrambled_lane_data [63:0];
    
    wire [7:0]  repaired_lane_data [63:0];
    wire [7:0]  repaired_txrd_data [3:0];
    wire [63:0] repaired_lane_oe;
    wire [3:0]  repaired_txrd_oe;

    // -------------------------------------------------------------------------
    // 1. BYTE-TO-LANE MAPPER 
    // -------------------------------------------------------------------------
    lphy_byte_lane_map byte_mapper_inst (
        .clk(clk), 
        .rst_n(rst_n), 
        .link_width(link_width), 
        .degrade_x8(degrade_x8),
        .degrade_upper(degrade_upper),
        .lane_reversal(lane_reversal),
        .lp_valid(lp_valid), 
        .lp_irdy(lp_irdy), 
        .pl_trdy(pl_trdy), 
        .lp_data(lp_data), 
        .lane_valid(mapped_lane_valid), 
        .lane_data(mapped_lane_data)
    );
    
    // If we are training, we simulate a 'valid' pipeline so the framer and 
    // pattern generators fire without needing Adapter data.
    assign active_valid = mapped_lane_valid | tx_training_en;
    
    // -------------------------------------------------------------------------
    // PIPELINE ALIGNMENT STAGE
    // -------------------------------------------------------------------------
    logic [7:0] mapped_lane_data_q [63:0];
    logic       active_valid_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_valid_q <= 1'b0;
            for (int i = 0; i < 64; i++) mapped_lane_data_q[i] <= 8'h00;
        end else begin
            active_valid_q <= active_valid;
            for (int i = 0; i < 64; i++) mapped_lane_data_q[i] <= mapped_lane_data[i];
        end
    end
    
    // -------------------------------------------------------------------------
    // 2. VALID FRAMER
    // -------------------------------------------------------------------------
    // train_mode forces VALTRAIN (0F) when pattern_sel is 2'b10
    wire is_valtrain = tx_training_en && (pattern_sel == 2'b10);
    
    lphy_valid_framer valid_framer_inst (
        .clk(clk), 
        .rst_n(rst_n),
        .is_retimer(is_retimer),
        .train_mode(is_valtrain),
        .lane_valid(active_valid), // Uses un-delayed valid to match 1-cycle latency
        .credit_return(credit_return),
        .valid_frame_out(tx_valid_frame)
    );
    
    // -------------------------------------------------------------------------
    // 3. TRAINING PATTERN GENERATOR ARRAY & MUX
    // -------------------------------------------------------------------------
    genvar pg;
    generate
        for (pg = 0; pg < 64; pg++) begin : gen_pattern_gens
            lphy_pattern_gen pattern_gen_inst (
                .clk(clk),
                .rst_n(rst_n),
                .lane_id       (8'(pg)), 
                .pattern_sel   (pattern_sel), 
                .enable        (tx_training_en), // Streams pattern when training
                .pattern_out   (pattern_data[pg])
            );
        end
    endgenerate

    always_comb begin
        for (int j = 0; j < 64; j++) begin
            if (tx_training_en) begin
                pre_scramble_data[j] = pattern_data[j];
            end else begin
                pre_scramble_data[j] = mapped_lane_data_q[j];
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // 4. SCRAMBLER ARRAY
    // -------------------------------------------------------------------------
    // Spec Rule: Initialization Patterns (LaneID, VALTRAIN, ClkRepair) must NOT 
    // be scrambled. We bypass the scrambler entirely when tx_training_en is high.
    wire scrambler_fire = scrambler_en & active_valid_q & ~tx_training_en;

    genvar i;
    generate
        for (i = 0; i < 64; i++) begin : gen_scramblers
            lphy_scrambler scrambler_inst (
                .clk(clk), 
                .rst_n(rst_n), 
                .enable(scrambler_fire), 
                .load_seed(load_seed), 
                .seed_in(lane_seeds[i]), 
                .data_in(pre_scramble_data[i]), 
                .data_out(scrambled_lane_data[i])
            );
        end
    endgenerate
    
    // -------------------------------------------------------------------------
    // 5. TX REPAIR MULTIPLEXER
    // -------------------------------------------------------------------------
    logic [63:0] tx_lane_failed_map;
    assign tx_lane_failed_map = repair_en ? ext_lane_failed_map : 64'h0;    
    
    lphy_repair_tx tx_repair_inst (
        .tx_logical_data(scrambled_lane_data), 
        .lane_failed(tx_lane_failed_map), 
        .tx_physical_data(repaired_lane_data), 
        .tx_redundant_data(repaired_txrd_data),
        .tx_physical_oe(repaired_lane_oe),
        .tx_redundant_oe(repaired_txrd_oe),
        .unrepairable(unrepairable)
    );
    
    // -------------------------------------------------------------------------
    // 6. AFE BOUNDARY PIPELINE (Clock Gating & Tri-State Registers)
    // -------------------------------------------------------------------------
    logic [3:0] postamble_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            TXVLD <= 8'h00;
            for (int i = 0; i < 64; i++) begin
                TXDATA[i] <= 8'h00;
            end
            TXDATA_OE <= 64'hFFFF_FFFF_FFFF_FFFF; // Default driving
            
            for (int i = 0; i < 4; i++) begin
                TXRD[i] <= 8'h00;
            end
            TXRD_OE <= 4'h0;
            
            tx_clock_en <= 1'b0;
            tx_track_en <= 1'b0;
            postamble_cnt <= 4'd2;
        end else begin
            // 1. Route physical data and Tri-State OEs to Analog Pads
            TXVLD     <= tx_valid_frame;
            TXDATA_OE <= repaired_lane_oe;
            TXRD_OE   <= repaired_txrd_oe;
            
            for (int i = 0; i < 64; i++) TXDATA[i] <= repaired_lane_data[i];
            for (int i = 0; i < 4; i++)  TXRD[i]   <= repaired_txrd_data[i];
            
            tx_track_en <= txtrk_en;
            
            // 2. Generate AFE Logical Envelope (with strictly 16 UI postamble)
            if (active_valid_q) begin
                postamble_cnt <= 4'd0;
                tx_clock_en   <= 1'b1;
            end else if (postamble_cnt < 4'd2) begin
                postamble_cnt <= postamble_cnt + 1'b1;
                tx_clock_en   <= 1'b1; // Hold clock high for exactly 2 cycles
            end else begin
                tx_clock_en   <= free_run_mode; // Gate clock to save power
            end
        end
    end
    
endmodule
`default_nettype wire