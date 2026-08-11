`default_nettype none
`timescale 1ns / 1ps

module ucie_lphy_top #(
    parameter int NUM_LANES = 64,       
    parameter logic PACKAGE_TYPE = 1'b0, 
    parameter int RESET_TIMER_CYCLES  = 400000,
    parameter int SBINIT_TIMEOUT      = 800000,
    parameter int MBINIT_TIMEOUT      = 800000,
    parameter int MBTRAIN_TIMEOUT     = 800000,
    parameter int LINKINIT_TIMEOUT    = 800000,
    parameter int PHYRETRAIN_TIMEOUT  = 800000,
    parameter int TRAINERROR_TIMEOUT  = 800000,
    parameter int D2C_PI_PHASE_MAX    = 63,
    parameter int D2C_SETTLE_CYCLES   = 32,
    parameter int D2C_TEST_CYCLES     = 128
)(
    // 1. GLOBAL CLOCKS AND RESET
    input  wire        lclk,             
    input  wire        rst_n,            
    input  wire        soc_reset_n,      

    // 2. ADAPTER INTERFACE (RDI)
    input  wire        lp_valid,
    input  wire        lp_irdy,
    output logic       pl_trdy,
    input  wire [511:0] lp_data,
    output logic       pl_valid,
    output logic [511:0] pl_data,
    input  wire [3:0]  lp_state_req,
    output logic [3:0] pl_state_sts,
    output logic       pl_inband_pres,
    output logic       pl_protocol_vld,  
    input  wire        lp_linkerror,
    output logic       pl_error,
    output logic       pl_cerror,
    output logic       pl_nferror,
    output logic       pl_trainerror,
    output logic       pl_phyinrecenter,
    output logic       pl_stallreq,
    input  wire        lp_stallack,
    output logic       pl_clk_req,
    input  wire        lp_clk_ack,
    input  wire        lp_wake_req,
    output logic       pl_wake_ack,
    output logic [2:0] pl_speedmode,
    output logic [2:0] pl_lnk_cfg,
    output logic [7:0] error_log_0,      

    // Sideband MAC Interface
    input  wire        tx_req_valid,
    output logic       tx_req_ready,
    input  wire [4:0]  tx_opcode,
    input  wire [2:0]  tx_srcid,
    input  wire [2:0]  tx_dstid,
    input  wire        tx_ep,
    input  wire        tx_cr,
    input  wire [63:0] tx_payload,
    input  wire [4:0]  tx_tag,
    input  wire [7:0]  tx_be,
    input  wire [23:0] tx_addr,
    input  wire [2:0]  tx_cp_status,
    input  wire [7:0]  tx_msgcode, 
    input  wire [7:0]  tx_msgsubcode,
    input  wire [15:0] tx_msginfo,
    input  wire        tx_local_crd_ret,
    output logic       rx_req_valid,
    output logic [4:0] rx_opcode, 
    output logic [2:0] rx_srcid,
    output logic [2:0] rx_dstid,
    output logic       rx_ep, 
    output logic       rx_cr, 
    output logic [63:0] rx_payload, 
    output logic [4:0] rx_tag,
    output logic [7:0] rx_be, 
    output logic [23:0] rx_addr, 
    output logic [2:0] rx_cp_status, 
    output logic [7:0] rx_msgcode,
    output logic [7:0] rx_msgsubcode, 
    output logic [15:0] rx_msginfo,
    output logic       rx_parity_err,

    // 3. PHYSICAL BUMP INTERFACE
    input  wire        power_stable,
    input  wire        sb_clk_stable,
    input  wire        mb_clk_stable,
    input  wire        mb_clk_slow,
    output logic [5:0] afe_pi_phase,     
    output logic [7:0] TXDATA [NUM_LANES-1:0],
    output logic [63:0] TXDATA_OE,       
    output logic [7:0] TXVLD,
    output logic [7:0] TXRD [3:0],
    output logic [3:0] TXRD_OE,          
    output logic [7:0] TXRDVLD,
    output logic       tx_clock_en,
    output logic       tx_track_en,
    output logic       TXCKP, TXCKN, TXTRK, TXRDCK,
    input  wire  [7:0] RXDATA [NUM_LANES-1:0],
    input  wire  [7:0] RXVLD,
    input  wire  [7:0] RXRD [3:0],
    input  wire  [7:0] RXRDVLD,
    input  wire        RXTRK, RXCKP, RXCKN, RXRDCK,
    output logic       rx_en,
    output logic       rx_gated_clk,
    output logic       afe_tx_valid,
    output logic [63:0] afe_tx_data,
    input  wire        afe_tx_ready,
    output logic       afe_tx_ipg_en,    
    output logic       afe_tx_rd_en,     
    input  wire        afe_rx_valid,
    input  wire  [63:0] afe_rx_data,
    output logic       afe_rx_en,
    output logic       afe_rx_rd_en,     

    // 4. EXTERNAL LTSSM / HANDSHAKES
    input  wire        start_link_training,
    input  wire        sb_rx_wake,       
    input  wire [3:0]  rx_pattern_detected,
    input  wire        rx_msg_out_of_reset, rx_msg_done_req, rx_msg_done_resp,
    input  wire        rx_req_active, rx_rsp_active,
    input  wire        rx_req_l1, rx_rsp_l1, rx_req_l2, rx_rsp_l2,
    input  wire        rx_req_linkreset, rx_rsp_linkreset,
    input  wire        rx_req_disable, rx_rsp_disable,
    input  wire        rx_req_retrain, rx_rsp_retrain,
    input  wire        rx_req_linkerror, rx_trainerror_req, rx_trainerror_resp,
    input  wire        rx_retrain_init_req, rx_retrain_init_resp,
    input  wire        rx_retrain_start_req, rx_retrain_start_resp,
    input  wire [2:0]  rx_retrain_enc,

    output logic       tx_send_pattern, tx_msg_out_of_reset, tx_msg_done_req, tx_msg_done_resp,
    output logic [2:0] sb_repair_sel,
    output logic       tx_req_active, tx_rsp_active,
    output logic       tx_req_l1, tx_rsp_l1, tx_req_l2, tx_rsp_l2,
    output logic       tx_req_linkreset, tx_rsp_linkreset,
    output logic       tx_req_disable, tx_rsp_disable,
    output logic       tx_req_retrain, tx_rsp_retrain,
    output logic       tx_req_linkerror, tx_trainerror_req, tx_trainerror_resp,
    output logic       tx_retrain_init_req, tx_retrain_init_resp,
    output logic       tx_retrain_start_req, tx_retrain_start_resp,
    output logic [2:0] tx_retrain_enc,

    // =========================================================================
    // 5. INTERNAL REPAIR & CALIBRATION STATUS 
    // =========================================================================
    input  wire        param_done, repairclk_done, repairval_done, reversal_done, repairmb_done,
    input  wire        valvref_done, datavref_done, speedidle_done, txselfcal_done, rxclkcal_done,
    input  wire        valtraincenter_done, valtrainvref_done, datatrainvref_done, rxdeskew_done,
    input  wire        datatraincenter2_done, linkspeed_done, linkspeed_error,
    input  wire        needs_repair, needs_speed_degrade, repair_done,
    // FIXED: Added these back so the testbench can bind cleanly
    input  wire        internal_retrain_req, 
    input  wire        internal_error_req,
    input  wire [2:0]  local_retrain_enc
);

    // =========================================================================
    // INTERNAL INTERCONNECT NETS
    // =========================================================================
    wire [3:0] int_pl_state_sts, int_lp_state_req;
    wire       int_pl_inband_pres, int_lp_linkerror, internal_start_link_training;
    wire       internal_stallreq, internal_stallack, internal_phyinrecenter;
    wire       internal_trainerror;
    
    wire       tx_training_en, scrambler_en, descrambler_en, repair_en;
    wire       lfsr_reset, clear_start_training, phy_reset_active;
    wire       en_reversal_check;
    
    wire       en_param_int, en_repairclk_int, en_repairval_int, en_repairmb_int;
    wire       en_valvref_int, en_datavref_int, en_speedidle_int;
    wire       en_txselfcal_int, en_rxclkcal_int;
    wire       en_datatrainvref_int, en_rxdeskew_int, en_datatraincenter2_int;
    wire       en_linkspeed_int, en_repair_state_int;
    
    wire       en_cal_int, en_datatrain_int;
    wire       start_cal;
    assign start_cal = en_cal_int | en_datatrain_int;

    // FIXED: Declare internal wires for sub-module driven signals
    wire       cal_done;
    wire       cal_error;
    wire       is_unrepairable;

    wire       int_select_valtrain;
    wire       int_en_valtraincenter, int_en_valtrainvref;
    assign int_select_valtrain = int_en_valtraincenter | int_en_valtrainvref;

    wire       framing_err;
    wire       rx_credit_return;
    wire       check_done_rx;
    wire [63:0] rx_detected_failures;
    
    wire [1:0] lane_map_ctrl;
    wire [7:0] pre_degrade_tx_data [NUM_LANES-1:0];
    wire [7:0] pre_degrade_rx_data [NUM_LANES-1:0];
    wire [7:0] raw_tx_vld;
    wire [7:0] raw_rx_vld;

    wire [7:0] trd_repair_addr [3:0];
    wire [1:0] tx_vld_repair_addr = 2'h3; 
    wire [1:0] rx_vld_repair_addr = 2'h3; 
    
    wire [22:0] lane_seeds [63:0];
    assign lane_seeds[ 0] = 23'h1DBFBC;  assign lane_seeds[ 1] = 23'h1DBFC0;
    assign lane_seeds[ 2] = 23'h1DBFC6;  assign lane_seeds[ 3] = 23'h0B7EF5;
    assign lane_seeds[ 4] = 23'h05BF7B;  assign lane_seeds[ 5] = 23'h162FC2;
    assign lane_seeds[ 6] = 23'h0B17E1;  assign lane_seeds[ 7] = 23'h058BF1;
    assign lane_seeds[ 8] = 23'h02C5F9;  assign lane_seeds[ 9] = 23'h016300;
    assign lane_seeds[10] = 23'h00B180;  assign lane_seeds[11] = 23'h0058C1;
    assign lane_seeds[12] = 23'h002C61;  assign lane_seeds[13] = 23'h001631;
    assign lane_seeds[14] = 23'h000B19;  assign lane_seeds[15] = 23'h00058D;
    assign lane_seeds[16] = 23'h1002C7;  assign lane_seeds[17] = 23'h080164;
    assign lane_seeds[18] = 23'h0400B2;  assign lane_seeds[19] = 23'h02005A;
    assign lane_seeds[20] = 23'h01002D;  assign lane_seeds[21] = 23'h008017;
    assign lane_seeds[22] = 23'h00400C;  assign lane_seeds[23] = 23'h002006;
    assign lane_seeds[24] = 23'h001003;  assign lane_seeds[25] = 23'h000802;
    assign lane_seeds[26] = 23'h000401;  assign lane_seeds[27] = 23'h1FFC4B;
    assign lane_seeds[28] = 23'h0FFE26;  assign lane_seeds[29] = 23'h07FF14;
    assign lane_seeds[30] = 23'h03FF8B;  assign lane_seeds[31] = 23'h01FFC6;
    assign lane_seeds[32] = 23'h00FFE3;  assign lane_seeds[33] = 23'h007FF2;
    assign lane_seeds[34] = 23'h003FFA;  assign lane_seeds[35] = 23'h001FFD;
    assign lane_seeds[36] = 23'h000FFF;  assign lane_seeds[37] = 23'h000800;
    assign lane_seeds[38] = 23'h000400;  assign lane_seeds[39] = 23'h000200;
    assign lane_seeds[40] = 23'h000100;  assign lane_seeds[41] = 23'h000080;
    assign lane_seeds[42] = 23'h000040;  assign lane_seeds[43] = 23'h000020;
    assign lane_seeds[44] = 23'h000010;  assign lane_seeds[45] = 23'h000008;
    assign lane_seeds[46] = 23'h000004;  assign lane_seeds[47] = 23'h000002;
    assign lane_seeds[48] = 23'h000001;  assign lane_seeds[49] = 23'h1FFFC1;
    assign lane_seeds[50] = 23'h0FFFE1;  assign lane_seeds[51] = 23'h07FFF1;
    assign lane_seeds[52] = 23'h03FFF9;  assign lane_seeds[53] = 23'h01FFFD;
    assign lane_seeds[54] = 23'h00FFFE;  assign lane_seeds[55] = 23'h007FFF;
    assign lane_seeds[56] = 23'h1C3FFF;  assign lane_seeds[57] = 23'h0E1FFF;
    assign lane_seeds[58] = 23'h070FFF;  assign lane_seeds[59] = 23'h0387FF;
    assign lane_seeds[60] = 23'h01C3FF;  assign lane_seeds[61] = 23'h00E1FF;
    assign lane_seeds[62] = 23'h0070FF;  assign lane_seeds[63] = 23'h00387F;

    lphy_rdi_intf u_rdi_intf (
        .lclk                    (lclk),
        .rst_n                   (rst_n),
        .lp_state_req            (lp_state_req),
        .pl_state_sts            (pl_state_sts),
        .pl_inband_pres          (pl_inband_pres),
        .lp_linkerror            (lp_linkerror),
        .pl_error                (pl_error),
        .pl_cerror               (pl_cerror),
        .pl_nferror              (pl_nferror),
        .pl_trainerror           (pl_trainerror),
        .pl_phyinrecenter        (pl_phyinrecenter),
        .pl_stallreq             (pl_stallreq),
        .lp_stallack             (lp_stallack),
        .pl_clk_req              (pl_clk_req),
        .lp_clk_ack              (lp_clk_ack),
        .lp_wake_req             (lp_wake_req),
        .pl_wake_ack             (pl_wake_ack),
        .pl_speedmode            (pl_speedmode),
        .pl_lnk_cfg              (pl_lnk_cfg),

        .internal_pl_state_sts   (int_pl_state_sts),
        .internal_pl_inband_pres (int_pl_inband_pres),
        .internal_lp_state_req   (int_lp_state_req),
        .internal_lp_linkerror   (int_lp_linkerror),
        .internal_start_link_training(internal_start_link_training),
        .internal_stallreq       (internal_stallreq),
        .internal_stallack       (internal_stallack),
        .internal_phyinrecenter  (internal_phyinrecenter),
        
        .internal_error          (1'b0),
        .internal_cerror         (1'b0),
        .internal_nferror        (1'b0),
        .internal_trainerror     (internal_trainerror),
        .internal_speedmode      (3'b111), 
        .internal_lnk_cfg        (3'b101)  
    );

    assign internal_trainerror = (int_pl_state_sts == 4'b1010); 

    lphy_ltssm #(
        .RESET_TIMER_CYCLES (RESET_TIMER_CYCLES),
        .SBINIT_TIMEOUT     (SBINIT_TIMEOUT),
        .MBINIT_TIMEOUT     (MBINIT_TIMEOUT),
        .MBTRAIN_TIMEOUT    (MBTRAIN_TIMEOUT),
        .LINKINIT_TIMEOUT   (LINKINIT_TIMEOUT),
        .PHYRETRAIN_TIMEOUT (PHYRETRAIN_TIMEOUT),
        .TRAINERROR_TIMEOUT (TRAINERROR_TIMEOUT)
    ) u_ltssm (
        .clk                     (lclk),
        .rst_n                   (rst_n),
        // FIXED: Connect the external MMIO pin to the LTSSM!
        .start_link_training     (start_link_training | internal_start_link_training),
        .lp_state_req            (int_lp_state_req),
        .lp_linkerror            (int_lp_linkerror),
        .lp_stallack             (internal_stallack),
        .pl_state_sts            (int_pl_state_sts),
        .pl_inband_pres          (int_pl_inband_pres),
        .pl_protocol_vld         (pl_protocol_vld),
        .pl_stallreq             (internal_stallreq),
        .rdi_to_retrain          (), 
        .phy_in_retrain          (internal_phyinrecenter),
        .power_stable            (power_stable),
        .sb_clk_stable           (sb_clk_stable),
        .mb_clk_stable           (mb_clk_stable),
        .mb_clk_slow             (mb_clk_slow),
        .soc_reset_n             (soc_reset_n),
        .package_type            (PACKAGE_TYPE),
        .sb_rx_wake              (sb_rx_wake),
        .rx_pattern_detected     (rx_pattern_detected),
        .rx_msg_out_of_reset     (rx_msg_out_of_reset),
        .rx_msg_done_req         (rx_msg_done_req),
        .rx_msg_done_resp        (rx_msg_done_resp),
        .rx_req_active           (rx_req_active),
        .rx_rsp_active           (rx_rsp_active),
        .rx_req_l1               (rx_req_l1),
        .rx_rsp_l1               (rx_rsp_l1),
        .rx_req_l2               (rx_req_l2),
        .rx_rsp_l2               (rx_rsp_l2),
        .rx_req_linkreset        (rx_req_linkreset),
        .rx_rsp_linkreset        (rx_rsp_linkreset),
        .rx_req_disable          (rx_req_disable),
        .rx_rsp_disable          (rx_rsp_disable),
        .rx_req_retrain          (rx_req_retrain),
        .rx_rsp_retrain          (rx_rsp_retrain),
        .rx_req_linkerror        (rx_req_linkerror),
        .rx_trainerror_req       (rx_trainerror_req),
        .rx_trainerror_resp      (rx_trainerror_resp),
        .rx_retrain_init_req     (rx_retrain_init_req),
        .rx_retrain_init_resp    (rx_retrain_init_resp),
        .rx_retrain_start_req    (rx_retrain_start_req),
        .rx_retrain_start_resp   (rx_retrain_start_resp),
        .rx_retrain_enc          (rx_retrain_enc),
        .tx_send_pattern         (tx_send_pattern),
        .tx_msg_out_of_reset     (tx_msg_out_of_reset),
        .tx_msg_done_req         (tx_msg_done_req),
        .tx_msg_done_resp        (tx_msg_done_resp),
        .sb_repair_sel           (sb_repair_sel),
        .tx_req_active           (tx_req_active),
        .tx_rsp_active           (tx_rsp_active),
        .tx_req_l1               (tx_req_l1),
        .tx_rsp_l1               (tx_rsp_l1),
        .tx_req_l2               (tx_req_l2),
        .tx_rsp_l2               (tx_rsp_l2),
        .tx_req_linkreset        (tx_req_linkreset),
        .tx_rsp_linkreset        (tx_rsp_linkreset),
        .tx_req_disable          (tx_req_disable),
        .tx_rsp_disable          (tx_rsp_disable),
        .tx_req_retrain          (tx_req_retrain),
        .tx_rsp_retrain          (tx_rsp_retrain),
        .tx_req_linkerror        (tx_req_linkerror),
        .tx_trainerror_req       (tx_trainerror_req),
        .tx_trainerror_resp      (tx_trainerror_resp),
        .tx_retrain_init_req     (tx_retrain_init_req),
        .tx_retrain_init_resp    (tx_retrain_init_resp),
        .tx_retrain_start_req    (tx_retrain_start_req),
        .tx_retrain_start_resp   (tx_retrain_start_resp),
        .tx_retrain_enc          (tx_retrain_enc),
        .param_done              (param_done), 
        .cal_done                (cal_done),        // Mapped to internal wire
        .repairclk_done          (repairclk_done), 
        .repairval_done          (repairval_done), 
        .reversal_done           (reversal_done), 
        .repairmb_done           (repairmb_done),
        .valvref_done            (valvref_done), 
        .datavref_done           (datavref_done), 
        .speedidle_done          (speedidle_done), 
        .txselfcal_done          (txselfcal_done), 
        .rxclkcal_done           (rxclkcal_done), 
        .valtraincenter_done     (valtraincenter_done), 
        .valtrainvref_done       (valtrainvref_done), 
        .datatraincenter1_done   (cal_done),        // Mapped to internal wire
        .datatrainvref_done      (datatrainvref_done), 
        .rxdeskew_done           (rxdeskew_done), 
        .datatraincenter2_done   (datatraincenter2_done),
        .linkspeed_done          (linkspeed_done), 
        .linkspeed_error         (linkspeed_error), 
        .needs_repair            (needs_repair), 
        .needs_speed_degrade     (needs_speed_degrade), 
        .repair_done             (repair_done),
        .cal_error               (cal_error),       // Mapped to internal wire
        .is_unrepairable         (is_unrepairable), // Mapped to internal wire
        .internal_retrain_req    (1'b0), 
        .internal_error_req      (framing_err),
        .local_retrain_enc       (3'b001),         
        .phy_reset_active        (phy_reset_active),
        .lfsr_reset              (lfsr_reset),
        .clear_start_training    (clear_start_training),
        .tx_training_en          (tx_training_en),
        .scrambler_en            (scrambler_en),
        .descrambler_en          (descrambler_en),
        .repair_en               (repair_en),
        .error_log_0             (error_log_0),
        .en_param                (en_param_int), 
        .en_cal                  (en_cal_int), 
        .en_repairclk            (en_repairclk_int), 
        .en_repairval            (en_repairval_int), 
        .en_reversal             (en_reversal_check), 
        .en_repairmb             (en_repairmb_int), 
        .en_valvref              (en_valvref_int), 
        .en_datavref             (en_datavref_int), 
        .en_speedidle            (en_speedidle_int), 
        .en_txselfcal            (en_txselfcal_int), 
        .en_rxclkcal             (en_rxclkcal_int), 
        .en_valtraincenter       (int_en_valtraincenter), 
        .en_valtrainvref         (int_en_valtrainvref), 
        .en_datatraincenter1     (en_datatrain_int), 
        .en_datatrainvref        (en_datatrainvref_int), 
        .en_rxdeskew             (en_rxdeskew_int), 
        .en_datatraincenter2     (en_datatraincenter2_int), 
        .en_linkspeed            (en_linkspeed_int), 
        .en_repair_state         (en_repair_state_int)
    );

    lphy_sb_ctrl u_sb_ctrl (
        .sb_clk        (lclk),
        .rst_n         (rst_n),
        .rdi_in_reset  (int_pl_state_sts == 4'b0000),
        .tx_req_valid  (tx_req_valid),      .tx_req_ready  (tx_req_ready),
        .tx_opcode     (tx_opcode),         .tx_srcid      (tx_srcid),
        .tx_dstid      (tx_dstid),          .tx_ep         (tx_ep),
        .tx_cr         (tx_cr),             .tx_payload    (tx_payload),
        .tx_tag        (tx_tag),            .tx_be         (tx_be),
        .tx_addr       (tx_addr),           .tx_cp_status  (tx_cp_status),
        .tx_msgcode    (tx_msgcode),        .tx_msgsubcode (tx_msgsubcode),
        .tx_msginfo    (tx_msginfo),        .tx_local_crd_ret(tx_local_crd_ret),
        .rx_req_valid  (rx_req_valid),      .rx_opcode     (rx_opcode),
        .rx_srcid      (rx_srcid),          .rx_dstid      (rx_dstid),
        .rx_ep         (rx_ep),             .rx_cr         (rx_cr),
        .rx_payload    (rx_payload),        .rx_tag        (rx_tag),
        .rx_be         (rx_be),             .rx_addr       (rx_addr),
        .rx_cp_status  (rx_cp_status),      .rx_msgcode    (rx_msgcode),
        .rx_msgsubcode (rx_msgsubcode),     .rx_msginfo    (rx_msginfo),
        .rx_parity_err (rx_parity_err),
        .afe_tx_valid  (afe_tx_valid),      .afe_tx_data   (afe_tx_data),
        .afe_tx_ready  (afe_tx_ready),      .afe_tx_ipg_en (afe_tx_ipg_en),
        .afe_tx_rd_en  (afe_tx_rd_en),
        .afe_rx_valid  (afe_rx_valid),      .afe_rx_data   (afe_rx_data),
        .afe_rx_en     (afe_rx_en),         .afe_rx_rd_en  (afe_rx_rd_en)
    );

    lphy_tx_top #(
        .NUM_LANES(NUM_LANES)
    ) u_tx_top (
        .clk                 (lclk),
        .rst_n               (rst_n),
        .link_width          (2'b10), 
        .degrade_x8          (1'b0),
        .degrade_upper       (1'b0),
        .lane_reversal       (1'b0),
        .is_retimer          (1'b0),
        .free_run_mode       (1'b0),
        .txtrk_en            (1'b0),
        .tx_training_en      (tx_training_en),
        .pattern_sel         ({int_select_valtrain, 1'b0}),
        .scrambler_en        (scrambler_en),
        .load_seed           (lfsr_reset),
        .lane_seeds          (lane_seeds),
        .repair_en           (repair_en), 
        .ext_lane_failed_map (rx_detected_failures), 
        .unrepairable        (),
        .lp_valid            (lp_valid),
        .lp_irdy             (lp_irdy),
        .pl_trdy             (pl_trdy),
        .lp_data             (lp_data),
        .credit_return       (rx_credit_return),
        .TXDATA              (pre_degrade_tx_data),
        .TXDATA_OE           (TXDATA_OE),
        .TXVLD               (raw_tx_vld),
        .TXRD                (TXRD),
        .TXRD_OE             (TXRD_OE),
        .tx_clock_en         (tx_clock_en),
        .tx_track_en         (tx_track_en)
    );

    assign TXCKP  = tx_clock_en;
    assign TXCKN  = ~tx_clock_en;
    assign TXTRK  = tx_track_en;
    assign TXRDCK = 1'b0;

    lphy_rx_top #(
        .NUM_LANES(NUM_LANES)
    ) u_rx_top (
        .clk                 (lclk),
        .rst_n               (rst_n),
        .link_width          (2'b10),
        .is_retimer          (1'b0),
        .free_run_mode       (1'b0),
        .is_linkerror        (lp_linkerror),
        .force_enable        (1'b0),
        .rx_training_en      (tx_training_en),
        .en_reversal_check   (en_reversal_check),
        .reversal_detected   (),
        .reversal_check_done (),
        .descrambler_en      (descrambler_en),
        .load_seed           (lfsr_reset),
        .lane_seeds          (lane_seeds),
        .repair_en           (repair_en), 
        .en_lane_check       (en_reversal_check), 
        .detected_lane_failures(rx_detected_failures),
        .check_done          (check_done_rx),
        .framing_err         (framing_err),
        .unrepairable        (),
        .pl_valid            (pl_valid),
        .pl_data             (pl_data), 
        .credit_return       (rx_credit_return),
        .rx_gated_clk        (rx_gated_clk),
        .RXDATA              (pre_degrade_rx_data),
        .RXVLD               (raw_rx_vld),
        .RXRD                (RXRD),
        .RXTRK               (RXTRK),
        .rx_en               (rx_en)
    );
    
    lphy_data_repair_ctrl u_repair_ctrl (
        .clk            (lclk), 
        .rst_n          (rst_n), 
        .package_type   (PACKAGE_TYPE), 
        .lane_failed    (rx_detected_failures), 
        .check_done     (check_done_rx), 
        .trd_repair_addr(trd_repair_addr),  
        .lane_map       (lane_map_ctrl),    
        .is_unrepairable(is_unrepairable)   // Internal wire
    );  
    
    logic [7:0] d2c_expected_data [NUM_LANES-1:0];
    generate
        for (genvar i = 0; i < NUM_LANES; i++) begin : gen_expected_data
            assign d2c_expected_data[i] = 8'h0F;
        end
    endgenerate

    lphy_d2c_cal #(
        .NUM_LANES(NUM_LANES),
        .PI_PHASE_MAX(D2C_PI_PHASE_MAX), 
        .SETTLE_CYCLES(D2C_SETTLE_CYCLES),
        .TEST_CYCLES(D2C_TEST_CYCLES)
    ) u_d2c_cal (
        .clk            (lclk), 
        .rst_n          (rst_n), 
        .start_cal      (start_cal), 
        .error_threshold(16'd0), 
        .rx_data        (pre_degrade_rx_data), 
        .expected_data  (d2c_expected_data), 
        .pi_phase       (afe_pi_phase), 
        .cal_done       (cal_done),         // Internal wire
        .cal_error      (cal_error)         // Internal wire
    );
    
    lphy_width_degrade #(
        .NUM_LANES (NUM_LANES)
    ) u_width_degrade (
        .lane_map           (lane_map_ctrl),
        .tx_logical_data    (pre_degrade_tx_data),
        .tx_physical_data   (TXDATA),              
        .rx_physical_data   (RXDATA),              
        .rx_logical_data    (pre_degrade_rx_data)
    );

    lphy_valid_repair u_valid_repair (
        .tvld_l             (raw_tx_vld),
        .trdvld_l           (8'h00),     
        .rvld_l             (raw_rx_vld),
        .rrdvld_l           (),         
        .tvld_p             (TXVLD),    
        .trdvld_p           (TXRDVLD),  
        .rvld_p             (RXVLD),    
        .rrdvld_p           (RXRDVLD),  
        .tx_repair_addr     (tx_vld_repair_addr),
        .rx_repair_addr     (rx_vld_repair_addr) 
    );

endmodule
`default_nettype wire