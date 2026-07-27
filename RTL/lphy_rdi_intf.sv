`default_nettype none
`timescale 1ps / 1ps

/// @title UCIe Raw Die-to-Die Interface (RDI)
/// @description Standardized logical boundary between the D2D Adapter and the Logical PHY.
/// Decouples the upper protocol layers from the physical analog macros.
module lphy_rdi_intf #(
    parameter int NBYTES = 16, // Mainband data width in bytes (e.g., 16 = 128-bit)
    parameter int NC     = 32  // Sideband data width in bits (8, 16, or 32)
)(
    input  wire  lclk,         // SOC provided global link clock
    input  wire  rst_n,        // Active-low reset

    // =========================================================================
    // 1. RDI Formal Interface (Exposed to D2D Adapter)
    // =========================================================================
    
    // --- State and Status Management ---
    input  wire [3:0]  lp_state_req,
    output wire [3:0]  pl_state_sts,
    output wire        pl_inband_pres,
    
    // --- Error Management ---
    input  wire        lp_linkerror,
    output wire        pl_error,
    output wire        pl_cerror,
    output wire        pl_nferror,
    output wire        pl_trainerror,
    output wire        pl_phyinrecenter,

    // --- Stall Handshake ---
    output wire        pl_stallreq,
    input  wire        lp_stallack,

    // --- Clock Gating Handshakes ---
    output wire        pl_clk_req,
    input  wire        lp_clk_ack,
    input  wire        lp_wake_req,
    output wire        pl_wake_ack,

    // --- Configuration ---
    output wire [2:0]  pl_speedmode,
    output wire [2:0]  pl_lnk_cfg,

    // --- Mainband Data Transmit (Adapter to PHY) ---
    input  wire [(NBYTES*8)-1:0] lp_data,
    input  wire        lp_valid,
    input  wire        lp_irdy,
    output wire        pl_trdy,
    input  wire        lp_retimer_crd,

    // --- Mainband Data Receive (PHY to Adapter) ---
    output wire [(NBYTES*8)-1:0] pl_data,
    output wire        pl_valid,
    output wire        pl_retimer_crd,

    // --- Sideband Transmit (Adapter to PHY) ---
    input  wire [NC-1:0] lp_cfg,
    input  wire        lp_cfg_vld,
    output wire        pl_cfg_crd,

    // --- Sideband Receive (PHY to Adapter) ---
    output wire [NC-1:0] pl_cfg,
    output wire        pl_cfg_vld,
    input  wire        lp_cfg_crd,

    // =========================================================================
    // 2. Internal PHY Connections (Wired to internal sub-modules)
    // =========================================================================
    
    // Internal State Machine (LTSSM)
    input  wire [3:0]  internal_pl_state_sts,
    input  wire        internal_pl_inband_pres,
    output wire [3:0]  internal_lp_state_req,
    output wire        internal_lp_linkerror,
    output wire        internal_start_link_training, 
    input  wire        internal_stallreq,
    output wire        internal_stallack,
    input  wire        internal_phyinrecenter,
    input  wire        internal_error,
    input  wire        internal_cerror,
    input  wire        internal_nferror,
    input  wire        internal_trainerror,
    input  wire [2:0]  internal_speedmode,
    input  wire [2:0]  internal_lnk_cfg,

    // Internal Mainband Datapath
    output wire [(NBYTES*8)-1:0] internal_lp_data,
    output wire        internal_lp_valid,
    output wire        internal_lp_irdy,
    input  wire        internal_pl_trdy,
    output wire        internal_lp_retimer_crd,
    
    input  wire [(NBYTES*8)-1:0] internal_pl_data,
    input  wire        internal_pl_valid,
    input  wire        internal_pl_retimer_crd,

    // Internal Sideband Controller
    output wire [NC-1:0] internal_lp_cfg,
    output wire        internal_lp_cfg_vld,
    input  wire        internal_pl_cfg_crd,
    
    input  wire [NC-1:0] internal_pl_cfg,
    input  wire        internal_pl_cfg_vld,
    output wire        internal_lp_cfg_crd
);

    // -------------------------------------------------------------------------
    // Direct Passthrough Assignments (State & Errors)
    // -------------------------------------------------------------------------
    assign internal_lp_state_req = lp_state_req;
    assign internal_lp_linkerror = lp_linkerror;
    
    assign pl_state_sts          = internal_pl_state_sts;
    assign pl_inband_pres        = internal_pl_inband_pres;
    assign pl_speedmode          = internal_speedmode;
    assign pl_lnk_cfg            = internal_lnk_cfg;

    assign pl_error              = internal_error;
    assign pl_cerror             = internal_cerror;
    assign pl_nferror            = internal_nferror;
    assign pl_trainerror         = internal_trainerror;
    assign pl_phyinrecenter      = internal_phyinrecenter;

    // -------------------------------------------------------------------------
    // Direct Passthrough Assignments (Datapath & Sideband)
    // -------------------------------------------------------------------------
    assign internal_lp_data      = lp_data;
    assign internal_lp_valid     = lp_valid;
    assign internal_lp_irdy      = lp_irdy;
    assign pl_trdy               = internal_pl_trdy;
    assign internal_lp_retimer_crd = lp_retimer_crd;

    assign pl_data               = internal_pl_data;
    assign pl_valid              = internal_pl_valid;
    assign pl_retimer_crd        = internal_pl_retimer_crd;

    assign internal_lp_cfg       = lp_cfg;
    assign internal_lp_cfg_vld   = lp_cfg_vld;
    assign pl_cfg_crd            = internal_pl_cfg_crd;

    assign pl_cfg                = internal_pl_cfg;
    assign pl_cfg_vld            = internal_pl_cfg_vld;
    assign internal_lp_cfg_crd   = lp_cfg_crd;

    // -------------------------------------------------------------------------
    // Link Training Trigger Synthesis
    // -------------------------------------------------------------------------
    // Translates the Adapter's request to exit Reset into a single cycle pulse 
    // to kick off the internal PHY SBINIT sequence.
    assign internal_start_link_training = (internal_pl_state_sts == 4'b0000) && (lp_state_req == 4'b0001);

    // -------------------------------------------------------------------------
    // Clock Gating Handshakes (UCIe Spec Section 8.1.3)
    // -------------------------------------------------------------------------
    // Rule: If dynamic clock gating is not supported by the PHY, it must stage
    // lp_wake_req internally for one or more clock cycles and turn it around as pl_wake_ack.
    logic wake_req_q1, wake_req_q2;
    always_ff @(posedge lclk or negedge rst_n) begin
        if (!rst_n) begin
            wake_req_q1 <= 1'b0;
            wake_req_q2 <= 1'b0;
        end else begin
            wake_req_q1 <= lp_wake_req;
            wake_req_q2 <= wake_req_q1;
        end
    end
    assign pl_wake_ack = wake_req_q2;

    // Bypass dynamic clock gating complexity on the Adapter side for this implementation
    assign pl_clk_req = 1'b1; 

    // -------------------------------------------------------------------------
    // Stall Handshake (UCIe Spec Section 8.3.1)
    // -------------------------------------------------------------------------
    // Rule 8: The logic path between pl_stallreq and lp_stallack must contain at 
    // least one flop to prevent a combinatorial loop.
    logic stallreq_q;
    logic stallack_q;

    always_ff @(posedge lclk or negedge rst_n) begin
        if (!rst_n) begin
            stallreq_q <= 1'b0;
            stallack_q <= 1'b0;
        end else begin
            // Forward internal PHY stall request up to Adapter
            stallreq_q <= internal_stallreq;
            
            // Capture Adapter's stall ack and safely route it down to PHY LTSSM
            stallack_q <= lp_stallack;
        end
    end

    assign pl_stallreq       = stallreq_q;
    assign internal_stallack = stallack_q;

endmodule
`default_nettype wire