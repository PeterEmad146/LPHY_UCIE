`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Active State Controller
/// @description Orchestrates the fully operational link state. 
/// (Optimized with Full I/O Boundary Shielding for 2GHz Timing Closure)
module lphy_ltssm_active (
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_active,           
    
    // Adapter Interface (RDI) 
    input  wire [3:0]  lp_state_req,        
    input  wire        lp_linkerror,        
    output logic [3:0] pl_state_sts,        
    
    // Handshake Status Inputs from Sideband RX 
    input  wire        rx_req_l1,        input wire rx_rsp_l1,
    input  wire        rx_req_l2,        input wire rx_rsp_l2,
    input  wire        rx_req_linkreset, input wire rx_rsp_linkreset,
    input  wire        rx_req_disable,   input wire rx_rsp_disable,
    input  wire        rx_req_retrain,   input wire rx_rsp_retrain,
    input  wire        rx_req_linkerror, 
    
    // Handshake Triggers to Sideband TX 
    output logic       tx_req_l1,        output logic tx_rsp_l1,
    output logic       tx_req_l2,        output logic tx_rsp_l2,
    output logic       tx_req_linkreset, output logic tx_rsp_linkreset,
    output logic       tx_req_disable,   output logic tx_rsp_disable,
    output logic       tx_req_retrain,   output logic tx_rsp_retrain,
    output logic       tx_req_linkerror,
    
    // Internal PHY Triggers
    input  wire        internal_retrain_req, 
    input  wire        internal_error_req,  
    
    // Status Logging Output
    output logic [7:0] active_log,          
    
    // State Machine Exits & Control
    output logic       scrambling_en,       
    output logic       exit_to_l1, 
    output logic       exit_to_l2, 
    output logic       exit_to_linkreset, 
    output logic       exit_to_disable, 
    output logic       exit_to_retrain, 
    output logic       exit_to_trainerror
);

    typedef enum logic [1:0] {
        ST_ACTIVE_STEADY = 2'b00,
        ST_WAIT_RSP      = 2'b01,  
        ST_EXITING       = 2'b10
    } state_t;
    
    (* fsm_encoding = "one_hot" *) state_t state, next_state;
    logic [3:0] target_exit_state; 

    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // Absorbs external timing delays before evaluating the priority tree
    // =========================================================================
    (* dont_touch = "true" *) logic       en_active_q;
    (* dont_touch = "true" *) logic [3:0] lp_state_req_q;
    (* dont_touch = "true" *) logic       lp_linkerror_q;
    
    (* dont_touch = "true" *) logic rx_req_l1_q,        rx_rsp_l1_q;
    (* dont_touch = "true" *) logic rx_req_l2_q,        rx_rsp_l2_q;
    (* dont_touch = "true" *) logic rx_req_linkreset_q, rx_rsp_linkreset_q;
    (* dont_touch = "true" *) logic rx_req_disable_q,   rx_rsp_disable_q;
    (* dont_touch = "true" *) logic rx_req_retrain_q,   rx_rsp_retrain_q;
    (* dont_touch = "true" *) logic rx_req_linkerror_q;
    
    (* dont_touch = "true" *) logic internal_retrain_req_q;
    (* dont_touch = "true" *) logic internal_error_req_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_active_q            <= 1'b0;
            lp_state_req_q         <= 4'h0;
            lp_linkerror_q         <= 1'b0;
            
            rx_req_l1_q            <= 1'b0; rx_rsp_l1_q        <= 1'b0;
            rx_req_l2_q            <= 1'b0; rx_rsp_l2_q        <= 1'b0;
            rx_req_linkreset_q     <= 1'b0; rx_rsp_linkreset_q <= 1'b0;
            rx_req_disable_q       <= 1'b0; rx_rsp_disable_q   <= 1'b0;
            rx_req_retrain_q       <= 1'b0; rx_rsp_retrain_q   <= 1'b0;
            rx_req_linkerror_q     <= 1'b0;
            
            internal_retrain_req_q <= 1'b0;
            internal_error_req_q   <= 1'b0;
        end else begin
            en_active_q            <= en_active;
            lp_state_req_q         <= lp_state_req;
            lp_linkerror_q         <= lp_linkerror;
            
            rx_req_l1_q            <= rx_req_l1;        rx_rsp_l1_q        <= rx_rsp_l1;
            rx_req_l2_q            <= rx_req_l2;        rx_rsp_l2_q        <= rx_rsp_l2;
            rx_req_linkreset_q     <= rx_req_linkreset; rx_rsp_linkreset_q <= rx_rsp_linkreset;
            rx_req_disable_q       <= rx_req_disable;   rx_rsp_disable_q   <= rx_rsp_disable;
            rx_req_retrain_q       <= rx_req_retrain;   rx_rsp_retrain_q   <= rx_rsp_retrain;
            rx_req_linkerror_q     <= rx_req_linkerror;
            
            internal_retrain_req_q <= internal_retrain_req;
            internal_error_req_q   <= internal_error_req;
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_ACTIVE_STEADY;
            target_exit_state <= 4'h0;
        end else begin
            state <= next_state;
            
            if (!en_active_q) begin
                target_exit_state <= 4'h0;
            end else if (state == ST_ACTIVE_STEADY) begin
                // Evaluate strictly using the shielded registers!
                if (rx_req_disable_q)           target_exit_state <= 4'b1100;
                else if (rx_req_linkreset_q)    target_exit_state <= 4'b1001;
                else if (rx_req_retrain_q)      target_exit_state <= 4'b1011;
                else if (rx_req_l1_q)           target_exit_state <= 4'b0100;
                else if (rx_req_l2_q)           target_exit_state <= 4'b1000;
                
                else if (lp_state_req_q == 4'b1100) target_exit_state <= 4'b1100; 
                else if (lp_state_req_q == 4'b1001) target_exit_state <= 4'b1001; 
                else if (lp_state_req_q == 4'b1011 || internal_retrain_req_q) target_exit_state <= 4'b1011; 
                else if (lp_state_req_q == 4'b0100) target_exit_state <= 4'b0100; 
                else if (lp_state_req_q == 4'b1000) target_exit_state <= 4'b1000; 
            end
        end
    end

    // =========================================================================
    // HANDSHAKE ARBITRATION 
    // =========================================================================
    localparam int WIN_DISABLE = 0, WIN_LINKRESET = 1, WIN_RETRAIN = 2,
                   WIN_L1 = 3, WIN_L2 = 4, WIN_LP_DISABLE = 5,
                   WIN_LP_LINKRESET = 6, WIN_LP_RETRAIN = 7,
                   WIN_LP_L1 = 8, WIN_LP_L2 = 9;

    logic [15:0] req_vec_steady;
    logic [3:0]  winner_idx;
    logic        winner_valid;

    function automatic void find_first_one16(input  logic [15:0] vec,
                                              output logic [3:0]  idx,
                                              output logic        valid);
        logic [1:0] n_idx [0:3]; logic n_val [0:3];
        logic [2:0] g_idx [0:1]; logic g_val [0:1];
        int i;
        for (i = 0; i < 4; i++) begin
            casez (vec[i*4 +: 4])
                4'b???1: begin n_idx[i] = 2'd0; n_val[i] = 1'b1; end
                4'b??10: begin n_idx[i] = 2'd1; n_val[i] = 1'b1; end
                4'b?100: begin n_idx[i] = 2'd2; n_val[i] = 1'b1; end
                4'b1000: begin n_idx[i] = 2'd3; n_val[i] = 1'b1; end
                default: begin n_idx[i] = 2'd0; n_val[i] = 1'b0; end
            endcase
        end
        for (i = 0; i < 2; i++) begin
            if (n_val[2*i])        begin g_idx[i] = {1'b0, n_idx[2*i]};   g_val[i] = 1'b1; end
            else if (n_val[2*i+1]) begin g_idx[i] = {1'b1, n_idx[2*i+1]}; g_val[i] = 1'b1; end
            else                   begin g_idx[i] = 3'd0; g_val[i] = 1'b0; end
        end
        if (g_val[0])      begin idx = {1'b0, g_idx[0]}; valid = 1'b1; end
        else if (g_val[1]) begin idx = {1'b1, g_idx[1]}; valid = 1'b1; end
        else               begin idx = 4'd0; valid = 1'b0; end
    endfunction

    always_comb begin
        req_vec_steady = 16'd0;
        // Evaluate strictly using the shielded registers
        req_vec_steady[WIN_DISABLE]      = rx_req_disable_q;
        req_vec_steady[WIN_LINKRESET]    = rx_req_linkreset_q;
        req_vec_steady[WIN_RETRAIN]      = rx_req_retrain_q;
        req_vec_steady[WIN_L1]           = rx_req_l1_q;
        req_vec_steady[WIN_L2]           = rx_req_l2_q;
        req_vec_steady[WIN_LP_DISABLE]   = (lp_state_req_q == 4'b1100);
        req_vec_steady[WIN_LP_LINKRESET] = (lp_state_req_q == 4'b1001);
        req_vec_steady[WIN_LP_RETRAIN]   = (lp_state_req_q == 4'b1011) || internal_retrain_req_q;
        req_vec_steady[WIN_LP_L1]        = (lp_state_req_q == 4'b0100);
        req_vec_steady[WIN_LP_L2]        = (lp_state_req_q == 4'b1000);
        find_first_one16(req_vec_steady, winner_idx, winner_valid);
    end

    // =========================================================================
    // COMBINATIONAL DECISION 
    // =========================================================================
    logic c_tx_req_l1,        c_tx_rsp_l1;
    logic c_tx_req_l2,        c_tx_rsp_l2;
    logic c_tx_req_linkreset, c_tx_rsp_linkreset;
    logic c_tx_req_disable,   c_tx_rsp_disable;
    logic c_tx_req_retrain,   c_tx_rsp_retrain;
    logic c_tx_req_linkerror;
    logic c_exit_to_l1, c_exit_to_l2, c_exit_to_linkreset;
    logic c_exit_to_disable, c_exit_to_retrain, c_exit_to_trainerror;

    always_comb begin
        next_state = state;
        
        c_tx_req_l1 = 0; c_tx_rsp_l1 = 0;
        c_tx_req_l2 = 0; c_tx_rsp_l2 = 0;
        c_tx_req_linkreset = 0; c_tx_rsp_linkreset = 0;
        c_tx_req_disable = 0;   c_tx_rsp_disable = 0;
        c_tx_req_retrain = 0;   c_tx_rsp_retrain = 0;
        c_tx_req_linkerror = 0;
        
        c_exit_to_l1 = 0; c_exit_to_l2 = 0;
        c_exit_to_linkreset = 0; c_exit_to_disable = 0;
        c_exit_to_retrain = 0; c_exit_to_trainerror = 0;
        
        if (en_active_q) begin
            // Evaluate fatal overrides using the shielded registers
            if (lp_linkerror_q || internal_error_req_q || rx_req_linkerror_q) begin
                c_tx_req_linkerror   = 1'b1; 
                c_exit_to_trainerror = 1'b1;
                next_state           = ST_ACTIVE_STEADY;
            end 
            else begin
                case (state)
                    ST_ACTIVE_STEADY: begin
                        if (winner_valid) begin
                            next_state = (winner_idx <= WIN_L2) ? ST_EXITING : ST_WAIT_RSP;
                            case (winner_idx)
                                WIN_DISABLE:      begin c_tx_rsp_disable   = 1; c_exit_to_disable   = 1; end
                                WIN_LINKRESET:    begin c_tx_rsp_linkreset = 1; c_exit_to_linkreset = 1; end
                                WIN_RETRAIN:      begin c_tx_rsp_retrain   = 1; c_exit_to_retrain   = 1; end
                                WIN_L1:           begin c_tx_rsp_l1        = 1; c_exit_to_l1        = 1; end
                                WIN_L2:           begin c_tx_rsp_l2        = 1; c_exit_to_l2        = 1; end
                                WIN_LP_DISABLE:   begin c_tx_req_disable   = 1; end
                                WIN_LP_LINKRESET: begin c_tx_req_linkreset = 1; end
                                WIN_LP_RETRAIN:   begin c_tx_req_retrain   = 1; end
                                WIN_LP_L1:        begin c_tx_req_l1        = 1; end
                                WIN_LP_L2:        begin c_tx_req_l2        = 1; end
                                default: ;
                            endcase
                        end
                    end
                    
                    ST_WAIT_RSP: begin
                        if      (target_exit_state == 4'b1100 && rx_rsp_disable_q)   begin c_exit_to_disable = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1001 && rx_rsp_linkreset_q) begin c_exit_to_linkreset = 1; next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1011 && rx_rsp_retrain_q)   begin c_exit_to_retrain = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b0100 && rx_rsp_l1_q)        begin c_exit_to_l1 = 1;        next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1000 && rx_rsp_l2_q)        begin c_exit_to_l2 = 1;        next_state = ST_EXITING; end
                    end
                    
                    ST_EXITING: begin
                        if      (target_exit_state == 4'b1100) c_exit_to_disable = 1'b1;
                        else if (target_exit_state == 4'b1001) c_exit_to_linkreset = 1'b1;
                        else if (target_exit_state == 4'b1011) c_exit_to_retrain = 1'b1;
                        else if (target_exit_state == 4'b0100) c_exit_to_l1 = 1'b1;
                        else if (target_exit_state == 4'b1000) c_exit_to_l2 = 1'b1;

                        if (!en_active_q) next_state = ST_ACTIVE_STEADY;
                    end
                endcase
            end
        end else begin
            next_state = ST_ACTIVE_STEADY;
        end
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // explicitly locked with dont_touch so Design Compiler cannot melt the boundaries
    // =========================================================================
    (* dont_touch = "true" *) logic tx_req_l1_q,        tx_rsp_l1_q;
    (* dont_touch = "true" *) logic tx_req_l2_q,        tx_rsp_l2_q;
    (* dont_touch = "true" *) logic tx_req_linkreset_q, tx_rsp_linkreset_q;
    (* dont_touch = "true" *) logic tx_req_disable_q,   tx_rsp_disable_q;
    (* dont_touch = "true" *) logic tx_req_retrain_q,   tx_rsp_retrain_q;
    (* dont_touch = "true" *) logic tx_req_linkerror_q;
    
    (* dont_touch = "true" *) logic exit_to_l1_q, exit_to_l2_q;
    (* dont_touch = "true" *) logic exit_to_linkreset_q, exit_to_disable_q;
    (* dont_touch = "true" *) logic exit_to_retrain_q, exit_to_trainerror_q;
    
    (* dont_touch = "true" *) logic scrambling_en_q;
    (* dont_touch = "true" *) logic [3:0] pl_state_sts_q;
    (* dont_touch = "true" *) logic [7:0] active_log_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_req_l1_q <= 1'b0;        tx_rsp_l1_q <= 1'b0;
            tx_req_l2_q <= 1'b0;        tx_rsp_l2_q <= 1'b0;
            tx_req_linkreset_q <= 1'b0; tx_rsp_linkreset_q <= 1'b0;
            tx_req_disable_q <= 1'b0;   tx_rsp_disable_q <= 1'b0;
            tx_req_retrain_q <= 1'b0;   tx_rsp_retrain_q <= 1'b0;
            tx_req_linkerror_q <= 1'b0;
            exit_to_l1_q <= 1'b0; exit_to_l2_q <= 1'b0;
            exit_to_linkreset_q <= 1'b0; exit_to_disable_q <= 1'b0;
            exit_to_retrain_q <= 1'b0; exit_to_trainerror_q <= 1'b0;
            scrambling_en_q <= 1'b0;
            pl_state_sts_q <= 4'b0000;
            active_log_q <= 8'h00;
        end else begin
            tx_req_l1_q <= c_tx_req_l1;               tx_rsp_l1_q <= c_tx_rsp_l1;
            tx_req_l2_q <= c_tx_req_l2;               tx_rsp_l2_q <= c_tx_rsp_l2;
            tx_req_linkreset_q <= c_tx_req_linkreset; tx_rsp_linkreset_q <= c_tx_rsp_linkreset;
            tx_req_disable_q <= c_tx_req_disable;     tx_rsp_disable_q <= c_tx_rsp_disable;
            tx_req_retrain_q <= c_tx_req_retrain;     tx_rsp_retrain_q <= c_tx_rsp_retrain;
            tx_req_linkerror_q <= c_tx_req_linkerror;
            exit_to_l1_q <= c_exit_to_l1;             exit_to_l2_q <= c_exit_to_l2;
            exit_to_linkreset_q <= c_exit_to_linkreset;
            exit_to_disable_q <= c_exit_to_disable;
            exit_to_retrain_q <= c_exit_to_retrain;
            exit_to_trainerror_q <= c_exit_to_trainerror;
            scrambling_en_q <= en_active_q;
            pl_state_sts_q <= en_active_q ? 4'b0001 : 4'b0000;
            active_log_q <= en_active_q ? 8'h15 : 8'h00;
        end
    end

    // Route purely from the locked registers
    assign tx_req_l1        = tx_req_l1_q;
    assign tx_rsp_l1        = tx_rsp_l1_q;
    assign tx_req_l2        = tx_req_l2_q;
    assign tx_rsp_l2        = tx_rsp_l2_q;
    assign tx_req_linkreset = tx_req_linkreset_q;
    assign tx_rsp_linkreset = tx_rsp_linkreset_q;
    assign tx_req_disable   = tx_req_disable_q;
    assign tx_rsp_disable   = tx_rsp_disable_q;
    assign tx_req_retrain   = tx_req_retrain_q;
    assign tx_rsp_retrain   = tx_rsp_retrain_q;
    assign tx_req_linkerror = tx_req_linkerror_q;
    
    assign exit_to_l1         = exit_to_l1_q;
    assign exit_to_l2         = exit_to_l2_q;
    assign exit_to_linkreset  = exit_to_linkreset_q;
    assign exit_to_disable    = exit_to_disable_q;
    assign exit_to_retrain    = exit_to_retrain_q;
    assign exit_to_trainerror = exit_to_trainerror_q;
    
    assign scrambling_en = scrambling_en_q;
    assign pl_state_sts  = pl_state_sts_q;
    assign active_log    = active_log_q;

endmodule
`default_nettype wire