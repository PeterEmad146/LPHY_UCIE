`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Mainband Initialization State (MBINIT)
/// @description Orchestrates Stage 2 of Link Bring-up at 4 GT/s. Sweeps through
/// parameter exchange, calibration, clock/valid repair, reversal, and data repair.
module lphy_ltssm_mbinit #(
    // Scaled down 8ms timeout for simulation
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_mbinit,       // Triggered by exit from SBINIT state
    input  wire        package_type,    // 0: Advanced, 1: Standard
    
    // Handshake Status Inputs from Sideband / Repair Logic
    input  wire        param_done,
    input  wire        cal_done, 
    input  wire        repairclk_done, 
    input  wire        repairval_done, 
    input  wire        reversal_done, 
    input  wire        repairmb_done, 
    input  wire        substate_error,  // Triggers immediate exit to TRAINERROR
    
    // Sub-state enables to trigger underlying hardware modules
    output logic       en_param, 
    output logic       en_cal, 
    output logic       en_repairclk, 
    output logic       en_repairval, 
    output logic       en_reversal, 
    output logic       en_repairmb, 
    
    // RDI / Protocol Isolation Outputs
    output logic [3:0] pl_state_sts,    // Must be held at 4'b0000 (RESET)
    output logic       pl_inband_pres,  // Must be held at 0
    
    // Status Logging Output
    output logic [7:0] mbinit_substate_log, // For Error Log 0 Register
    
    // State Machine Exits
    output logic       exit_to_mbtrain, 
    output logic       exit_to_trainerror
);

    // Enforce RDI Isolation Rules unconditionally during this state
    assign pl_state_sts   = 4'b0000;
    assign pl_inband_pres = 1'b0;

    // Direct mapping to UCIe Specification Error Log 0 Encodings
    typedef enum logic [7:0] {
        ST_IDLE       = 8'h00, 
        ST_PARAM      = 8'h02, 
        ST_CAL        = 8'h03, 
        ST_REPAIRCLK  = 8'h04, 
        ST_REPAIRVAL  = 8'h05, 
        ST_REVERSALMB = 8'h06, 
        ST_REPAIRMB   = 8'h07, 
        ST_DONE       = 8'h0F, 
        ST_ERROR      = 8'hEE
    } state_t;
    
    state_t state, next_state;
    logic [31:0] timeout_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            timeout_cnt <= 32'd0;
        end else begin
            state <= next_state;
            
            // ARCHITECTURAL FIX: 8ms Timeout is PER SUB-STATE.
            // Reset the timer whenever the state changes.
            if (state != next_state) begin
                timeout_cnt <= 32'd0;
            end else if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                if (timeout_cnt < TIMEOUT_CYCLES)
                    timeout_cnt <= timeout_cnt + 1'b1;
            end
        end
    end
    
    always_comb begin
        next_state         = state;
        en_param           = 1'b0;
        en_cal             = 1'b0;
        en_repairclk       = 1'b0;
        en_repairval       = 1'b0;
        en_reversal        = 1'b0;
        en_repairmb        = 1'b0;
        exit_to_mbtrain    = 1'b0;
        exit_to_trainerror = 1'b0;
        
        // Output the spec-compliant state encoding to the register block
        mbinit_substate_log = state;
        
        // State Machine execution
        case (state)
            ST_IDLE: begin
                if (en_mbinit) next_state = ST_PARAM;
            end
            
            ST_PARAM: begin
                en_param = 1'b1;
                if(param_done) next_state = ST_CAL;
            end
            
            ST_CAL: begin
                en_cal = 1'b1;
                // ARCHITECTURAL FIX: Standard packages must NOT skip the continuity checks.
                if(cal_done) next_state = ST_REPAIRCLK;
            end
            
            ST_REPAIRCLK: begin
                en_repairclk = 1'b1;
                if(repairclk_done) next_state = ST_REPAIRVAL;
            end
            
            ST_REPAIRVAL: begin
                en_repairval = 1'b1;
                if (repairval_done) next_state = ST_REVERSALMB;
            end
            
            ST_REVERSALMB: begin
                en_reversal = 1'b1;
                if (reversal_done) next_state = ST_REPAIRMB;
            end
            
            ST_REPAIRMB: begin
                en_repairmb = 1'b1;
                if(repairmb_done) next_state = ST_DONE;
            end
            
            ST_DONE: begin
                exit_to_mbtrain = 1'b1;
                if (!en_mbinit) next_state = ST_IDLE; // Reset when Master LTSSM moves on
            end
            
            ST_ERROR: begin
                exit_to_trainerror = 1'b1;
                if(!en_mbinit) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
        
        // Timeout or Substate Error overrides normal transitions
        if ((timeout_cnt == TIMEOUT_CYCLES) || substate_error) begin
            if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                next_state = ST_ERROR;
            end
        end
    end
endmodule
`default_nettype wire