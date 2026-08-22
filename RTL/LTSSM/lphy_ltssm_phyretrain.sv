`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Physical Retrain State Controller (PHYRETRAIN)
/// @description Orchestrates the mid-flight pipeline stall and sideband negotiation.
/// (Optimized with Full I/O Boundary Shielding and Pipelined Counters for 2GHz)
module lphy_ltssm_phyretrain #(
    parameter int TIMEOUT_CYCLES = 800000 
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         en_phyretrain,       
    
    // Internal PHY Triggers
    input  wire         local_retrain_trigger, 
    input  wire [2:0]   local_retrain_enc,   

    // RDI Interface Control & Status
    output logic        pl_stallreq,         
    input  wire         lp_stallack,         
    output logic [3:0]  pl_state_sts,        

    // Handshake Status Inputs from Sideband RX
    input  wire         rx_retrain_init_req,
    input  wire         rx_retrain_init_resp,
    input  wire         rx_retrain_start_req,
    input  wire [2:0]   rx_retrain_enc,
    input  wire         rx_retrain_start_resp,

    // Handshake Triggers to Sideband TX
    output logic        tx_retrain_init_req,
    output logic        tx_retrain_init_resp,
    output logic        tx_retrain_start_req,
    output logic        tx_retrain_start_resp,
    output logic [2:0]  tx_retrain_enc,
    
    // Status Logging Output
    output logic [7:0]  phyretrain_log,      

    // State Machine Exits & Control
    output logic        rdi_to_retrain,      
    output logic        phy_in_retrain,      
    output logic        exit_to_txselfcal,
    output logic        exit_to_speedidle,
    output logic        exit_to_repair,
    output logic        exit_to_trainerror
);

    typedef enum logic [3:0] {
        ST_IDLE            = 4'h0,
        ST_LOC_STALL_WAIT  = 4'h1,
        ST_LOC_INIT_REQ    = 4'h2,
        ST_LOC_WAIT_RESP   = 4'h3,
        ST_LOC_START_REQ   = 4'h4,
        ST_LOC_WAIT_START  = 4'h5,
        ST_REM_STALL_WAIT  = 4'h6,
        ST_REM_INIT_RESP   = 4'h7,
        ST_REM_WAIT_REQ    = 4'h8,
        ST_REM_START_RESP  = 4'h9,
        ST_DONE            = 4'hA,
        ST_ERROR           = 4'hB
    } state_t;

    (* fsm_encoding = "one_hot" *) state_t state, next_state, state_q;
    
    logic [2:0]  remote_enc_reg;
    logic [2:0]  resolved_enc;

    // =========================================================================
    // 1. BOUNDARY INPUT PIPELINE (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic en_phyretrain_q;
    (* dont_touch = "true" *) logic local_retrain_trigger_q;
    (* dont_touch = "true" *) logic lp_stallack_q;
    (* dont_touch = "true" *) logic rx_init_req_q;
    (* dont_touch = "true" *) logic rx_init_resp_q;
    (* dont_touch = "true" *) logic rx_start_req_q;
    (* dont_touch = "true" *) logic rx_start_resp_q;
    (* dont_touch = "true" *) logic [2:0] rx_enc_q;

    // =========================================================================
    // 2. PIPELINED COUNTER 
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
            remote_enc_reg  <= 3'b001;
            
            en_phyretrain_q         <= 1'b0;
            local_retrain_trigger_q <= 1'b0;
            lp_stallack_q           <= 1'b0;
            rx_init_req_q           <= 1'b0;
            rx_init_resp_q          <= 1'b0;
            rx_start_req_q          <= 1'b0;
            rx_start_resp_q         <= 1'b0;
            rx_enc_q                <= 3'b000;
            
            cnt_lo          <= '0;
            cnt_mid         <= '0;
            cnt_hi          <= '0;
            carry_lo        <= 1'b0;
            carry_mid       <= 1'b0;
            timeout_reached <= 1'b0;
        end else begin
            state   <= next_state;
            state_q <= state;
            
            // Flop the inputs 
            en_phyretrain_q         <= en_phyretrain;
            local_retrain_trigger_q <= local_retrain_trigger;
            lp_stallack_q           <= lp_stallack;
            rx_init_req_q           <= rx_retrain_init_req;
            rx_init_resp_q          <= rx_retrain_init_resp;
            rx_start_req_q          <= rx_retrain_start_req;
            rx_start_resp_q         <= rx_retrain_start_resp;
            rx_enc_q                <= rx_retrain_enc;

            // Timer Logic
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

            if (rx_start_req_q || rx_start_resp_q) begin
                remote_enc_reg <= rx_enc_q;
            end
        end
    end

    // Resolution Logic 
    always_comb begin
        logic [2:0] remote_val;
        remote_val = (rx_start_req_q || rx_start_resp_q) ? rx_enc_q : remote_enc_reg;
        
        if (local_retrain_enc == 3'b010 || remote_val == 3'b010) begin
            resolved_enc = 3'b010; 
        end else if (local_retrain_enc == 3'b100 || remote_val == 3'b100) begin
            resolved_enc = 3'b100; 
        end else begin
            resolved_enc = 3'b001; 
        end
    end

    // =========================================================================
    // 3. COMBINATIONAL NEXT-STATE
    // =========================================================================
    logic c_pl_stallreq;
    logic [3:0] c_pl_state_sts;
    logic c_tx_retrain_init_req, c_tx_retrain_init_resp;
    logic c_tx_retrain_start_req, c_tx_retrain_start_resp;
    logic [2:0] c_tx_retrain_enc;
    logic [7:0] c_phyretrain_log;
    logic c_rdi_to_retrain, c_phy_in_retrain;
    logic c_exit_to_txselfcal, c_exit_to_speedidle, c_exit_to_repair, c_exit_to_trainerror;

    always_comb begin
        next_state             = state;
        c_pl_stallreq          = 1'b0;
        c_tx_retrain_init_req  = 1'b0;
        c_tx_retrain_init_resp = 1'b0;
        c_tx_retrain_start_req = 1'b0;
        c_tx_retrain_start_resp= 1'b0;
        c_tx_retrain_enc       = 3'b000;
        c_rdi_to_retrain       = 1'b0;
        
        c_exit_to_txselfcal    = 1'b0;
        c_exit_to_speedidle    = 1'b0;
        c_exit_to_repair       = 1'b0;
        c_exit_to_trainerror   = 1'b0;

        c_phyretrain_log = (state != ST_IDLE) ? 8'h13 : 8'h00;
        
        c_phy_in_retrain = en_phyretrain_q; 
        c_pl_state_sts   = en_phyretrain_q ? 4'b1011 : 4'b0000;

        case (state)
            ST_IDLE: begin
                if (en_phyretrain_q) begin
                    if (local_retrain_trigger_q) next_state = ST_LOC_STALL_WAIT;
                    else if (rx_init_req_q) next_state = ST_REM_STALL_WAIT;
                end
            end

            ST_LOC_STALL_WAIT: begin
                c_pl_stallreq = 1'b1; 
                if (lp_stallack_q) next_state = ST_LOC_INIT_REQ;
            end
            
            ST_LOC_INIT_REQ: begin
                c_tx_retrain_init_req = 1'b1;
                next_state = ST_LOC_WAIT_RESP;
            end
            
            ST_LOC_WAIT_RESP: begin
                if (rx_init_resp_q) begin
                    next_state = ST_LOC_START_REQ;
                end else if (rx_init_req_q) begin
                    c_tx_retrain_init_resp = 1'b1; 
                    next_state = ST_LOC_START_REQ;
                end
            end
            
            ST_LOC_START_REQ: begin
                c_rdi_to_retrain = 1'b1; 
                c_tx_retrain_start_req = 1'b1;
                c_tx_retrain_enc = local_retrain_enc;
                next_state = ST_LOC_WAIT_START;
            end
            
            ST_LOC_WAIT_START: begin
                c_rdi_to_retrain = 1'b1;
                if (rx_start_resp_q || rx_start_req_q) begin
                    if (rx_start_req_q) begin
                        c_tx_retrain_start_resp = 1'b1;
                        c_tx_retrain_enc = resolved_enc;
                    end
                    next_state = ST_DONE;
                end
            end

            ST_REM_STALL_WAIT: begin
                c_pl_stallreq = 1'b1; 
                if (lp_stallack_q) next_state = ST_REM_INIT_RESP;
            end
            
            ST_REM_INIT_RESP: begin
                c_tx_retrain_init_resp = 1'b1;
                c_rdi_to_retrain = 1'b1;
                next_state = ST_REM_WAIT_REQ;
            end
            
            ST_REM_WAIT_REQ: begin
                c_rdi_to_retrain = 1'b1;
                if (rx_start_req_q) next_state = ST_REM_START_RESP;
            end
            
            ST_REM_START_RESP: begin
                c_rdi_to_retrain = 1'b1;
                c_tx_retrain_start_resp = 1'b1;
                c_tx_retrain_enc = resolved_enc;
                next_state = ST_DONE;
            end

            ST_DONE: begin
                c_rdi_to_retrain = 1'b1; 
                if (resolved_enc == 3'b010) c_exit_to_speedidle = 1'b1;
                else if (resolved_enc == 3'b100) c_exit_to_repair = 1'b1;
                else c_exit_to_txselfcal = 1'b1;
                
                if (!en_phyretrain_q) next_state = ST_IDLE;
            end

            ST_ERROR: begin
                c_exit_to_trainerror = 1'b1;
                if (!en_phyretrain_q) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase

        // The bug was here: `substate_error` is safely removed
        if (timeout_reached) begin
            if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                next_state = ST_ERROR;
            end
        end
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // =========================================================================
    (* dont_touch = "true" *) logic        pl_stallreq_q;
    (* dont_touch = "true" *) logic [3:0]  pl_state_sts_q;
    (* dont_touch = "true" *) logic        tx_retrain_init_req_q;
    (* dont_touch = "true" *) logic        tx_retrain_init_resp_q;
    (* dont_touch = "true" *) logic        tx_retrain_start_req_q;
    (* dont_touch = "true" *) logic        tx_retrain_start_resp_q;
    (* dont_touch = "true" *) logic [2:0]  tx_retrain_enc_q;
    (* dont_touch = "true" *) logic [7:0]  phyretrain_log_q;
    (* dont_touch = "true" *) logic        rdi_to_retrain_q;
    (* dont_touch = "true" *) logic        phy_in_retrain_q;
    (* dont_touch = "true" *) logic        exit_to_txselfcal_q;
    (* dont_touch = "true" *) logic        exit_to_speedidle_q;
    (* dont_touch = "true" *) logic        exit_to_repair_q;
    (* dont_touch = "true" *) logic        exit_to_trainerror_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pl_stallreq_q           <= 1'b0;
            pl_state_sts_q          <= 4'b0000;
            tx_retrain_init_req_q   <= 1'b0;
            tx_retrain_init_resp_q  <= 1'b0;
            tx_retrain_start_req_q  <= 1'b0;
            tx_retrain_start_resp_q <= 1'b0;
            tx_retrain_enc_q        <= 3'b000;
            phyretrain_log_q        <= 8'h00;
            rdi_to_retrain_q        <= 1'b0;
            phy_in_retrain_q        <= 1'b0;
            exit_to_txselfcal_q     <= 1'b0;
            exit_to_speedidle_q     <= 1'b0;
            exit_to_repair_q        <= 1'b0;
            exit_to_trainerror_q    <= 1'b0;
        end else begin
            pl_stallreq_q           <= c_pl_stallreq;
            pl_state_sts_q          <= c_pl_state_sts;
            tx_retrain_init_req_q   <= c_tx_retrain_init_req;
            tx_retrain_init_resp_q  <= c_tx_retrain_init_resp;
            tx_retrain_start_req_q  <= c_tx_retrain_start_req;
            tx_retrain_start_resp_q <= c_tx_retrain_start_resp;
            tx_retrain_enc_q        <= c_tx_retrain_enc;
            phyretrain_log_q        <= c_phyretrain_log;
            rdi_to_retrain_q        <= c_rdi_to_retrain;
            phy_in_retrain_q        <= c_phy_in_retrain;
            exit_to_txselfcal_q     <= c_exit_to_txselfcal;
            exit_to_speedidle_q     <= c_exit_to_speedidle;
            exit_to_repair_q        <= c_exit_to_repair;
            exit_to_trainerror_q    <= c_exit_to_trainerror;
        end
    end

    // Route purely from the locked registers
    assign pl_stallreq           = pl_stallreq_q;
    assign pl_state_sts          = pl_state_sts_q;
    assign tx_retrain_init_req   = tx_retrain_init_req_q;
    assign tx_retrain_init_resp  = tx_retrain_init_resp_q;
    assign tx_retrain_start_req  = tx_retrain_start_req_q;
    assign tx_retrain_start_resp = tx_retrain_start_resp_q;
    assign tx_retrain_enc        = tx_retrain_enc_q;
    assign phyretrain_log        = phyretrain_log_q;
    assign rdi_to_retrain        = rdi_to_retrain_q;
    assign phy_in_retrain        = phy_in_retrain_q;
    assign exit_to_txselfcal     = exit_to_txselfcal_q;
    assign exit_to_speedidle     = exit_to_speedidle_q;
    assign exit_to_repair        = exit_to_repair_q;
    assign exit_to_trainerror    = exit_to_trainerror_q;

endmodule
`default_nettype wire