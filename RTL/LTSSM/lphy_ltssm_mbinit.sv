`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Mainband Initialization State (MBINIT)
/// @description Orchestrates Stage 2 of Link Bring-up at 4 GT/s. 
/// (Optimized with Full I/O Boundary Shielding and Pipelined Counters for 2GHz)
module lphy_ltssm_mbinit #(
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_mbinit,       
    input  wire        package_type,    // 0: Advanced, 1: Standard
    
    // Handshake Status Inputs
    input  wire        param_done,
    input  wire        cal_done, 
    input  wire        repairclk_done, 
    input  wire        repairval_done, 
    input  wire        reversal_done, 
    input  wire        repairmb_done, 
    input  wire        substate_error,  
    
    // Sub-state enables
    output logic       en_param, 
    output logic       en_cal, 
    output logic       en_repairclk, 
    output logic       en_repairval, 
    output logic       en_reversal, 
    output logic       en_repairmb, 
    
    // RDI Isolation Outputs
    output logic [3:0] pl_state_sts,    
    output logic       pl_inband_pres,  
    
    // Status Logging Output
    output logic [7:0] mbinit_substate_log, 
    
    // State Machine Exits
    output logic       exit_to_mbtrain, 
    output logic       exit_to_trainerror
);

    assign pl_state_sts   = 4'b0000;
    assign pl_inband_pres = 1'b0;

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
    
    state_t state, next_state, state_q;

    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic en_mbinit_q;
    (* dont_touch = "true" *) logic param_done_q;
    (* dont_touch = "true" *) logic cal_done_q;
    (* dont_touch = "true" *) logic repairclk_done_q;
    (* dont_touch = "true" *) logic repairval_done_q;
    (* dont_touch = "true" *) logic reversal_done_q;
    (* dont_touch = "true" *) logic repairmb_done_q;
    (* dont_touch = "true" *) logic substate_error_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_mbinit_q      <= 1'b0;
            param_done_q     <= 1'b0;
            cal_done_q       <= 1'b0;
            repairclk_done_q <= 1'b0;
            repairval_done_q <= 1'b0;
            reversal_done_q  <= 1'b0;
            repairmb_done_q  <= 1'b0;
            substate_error_q <= 1'b0;
        end else begin
            en_mbinit_q      <= en_mbinit;
            param_done_q     <= param_done;
            cal_done_q       <= cal_done;
            repairclk_done_q <= repairclk_done;
            repairval_done_q <= repairval_done;
            reversal_done_q  <= reversal_done;
            repairmb_done_q  <= repairmb_done;
            substate_error_q <= substate_error;
        end
    end

    // =========================================================================
    // 2. PIPELINED COUNTER (Kills 32-bit Ripple-Carry Trap)
    // =========================================================================
    localparam logic [19:0] TARGET_CYCLES = 20'(TIMEOUT_CYCLES);

    (* dont_touch = "true" *) logic [7:0] cnt_lo;
    (* dont_touch = "true" *) logic [7:0] cnt_mid;
    (* dont_touch = "true" *) logic [3:0] cnt_hi;
    (* dont_touch = "true" *) logic       carry_lo;
    (* dont_touch = "true" *) logic       carry_mid;
    (* dont_touch = "true" *) logic       timeout_reached;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            state_q         <= ST_IDLE;
            cnt_lo          <= '0;
            cnt_mid         <= '0;
            cnt_hi          <= '0;
            carry_lo        <= 1'b0;
            carry_mid       <= 1'b0;
            timeout_reached <= 1'b0;
        end else begin
            state   <= next_state;
            state_q <= state;
            
            if (state != state_q) begin
                cnt_lo          <= '0;
                cnt_mid         <= '0;
                cnt_hi          <= '0;
                carry_lo        <= 1'b0;
                carry_mid       <= 1'b0;
                timeout_reached <= 1'b0;
            end else if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                if (!timeout_reached) begin
                    cnt_lo   <= cnt_lo + 1'b1;
                    carry_lo <= (cnt_lo == 8'hFF);
                    
                    if (carry_lo) begin
                        cnt_mid   <= cnt_mid + 1'b1;
                        carry_mid <= (cnt_mid == 8'hFF);
                    end else carry_mid <= 1'b0;
                    
                    if (carry_mid) cnt_hi <= cnt_hi + 1'b1;
                end

                if ({cnt_hi, cnt_mid, cnt_lo} == TARGET_CYCLES) begin
                    timeout_reached <= 1'b1;
                end
            end
        end
    end
    
    // =========================================================================
    // 3. COMBINATIONAL NEXT-STATE & INTERNAL OUTPUTS
    // =========================================================================
    logic c_en_param, c_en_cal, c_en_repairclk, c_en_repairval, c_en_reversal, c_en_repairmb;
    logic c_exit_to_mbtrain, c_exit_to_trainerror;
    logic [7:0] c_mbinit_substate_log;

    always_comb begin
        next_state           = state;
        c_en_param           = 1'b0;
        c_en_cal             = 1'b0;
        c_en_repairclk       = 1'b0;
        c_en_repairval       = 1'b0;
        c_en_reversal        = 1'b0;
        c_en_repairmb        = 1'b0;
        c_exit_to_mbtrain    = 1'b0;
        c_exit_to_trainerror = 1'b0;
        
        c_mbinit_substate_log = state;
        
        // Evaluate strictly using the shielded input registers
        case (state)
            ST_IDLE: begin
                if (en_mbinit_q) next_state = ST_PARAM;
            end
            
            ST_PARAM: begin
                c_en_param = 1'b1;
                if(param_done_q) next_state = ST_CAL;
            end
            
            ST_CAL: begin
                c_en_cal = 1'b1;
                if(cal_done_q) next_state = ST_REPAIRCLK;
            end
            
            ST_REPAIRCLK: begin
                c_en_repairclk = 1'b1;
                if(repairclk_done_q) next_state = ST_REPAIRVAL;
            end
            
            ST_REPAIRVAL: begin
                c_en_repairval = 1'b1;
                if (repairval_done_q) next_state = ST_REVERSALMB;
            end
            
            ST_REVERSALMB: begin
                c_en_reversal = 1'b1;
                if (reversal_done_q) next_state = ST_REPAIRMB;
            end
            
            ST_REPAIRMB: begin
                c_en_repairmb = 1'b1;
                if(repairmb_done_q) next_state = ST_DONE;
            end
            
            ST_DONE: begin
                c_exit_to_mbtrain = 1'b1;
                if (!en_mbinit_q) next_state = ST_IDLE; 
            end
            
            ST_ERROR: begin
                c_exit_to_trainerror = 1'b1;
                if(!en_mbinit_q) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
        
        if (timeout_reached || substate_error_q) begin
            if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                next_state = ST_ERROR;
            end
        end
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_param            <= 1'b0;
            en_cal              <= 1'b0;
            en_repairclk        <= 1'b0;
            en_repairval        <= 1'b0;
            en_reversal         <= 1'b0;
            en_repairmb         <= 1'b0;
            exit_to_mbtrain     <= 1'b0;
            exit_to_trainerror  <= 1'b0;
            mbinit_substate_log <= 8'h00;
        end else begin
            en_param            <= c_en_param;
            en_cal              <= c_en_cal;
            en_repairclk        <= c_en_repairclk;
            en_repairval        <= c_en_repairval;
            en_reversal         <= c_en_reversal;
            en_repairmb         <= c_en_repairmb;
            exit_to_mbtrain     <= c_exit_to_mbtrain;
            exit_to_trainerror  <= c_exit_to_trainerror;
            mbinit_substate_log <= c_mbinit_substate_log;
        end
    end
endmodule
`default_nettype wire