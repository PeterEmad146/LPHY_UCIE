`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Active State Controller
/// @description Orchestrates the fully operational link state. Enforces mandatory
/// mainband scrambling, reports status to RDI, and coordinates sideband handshakes
/// for exiting to Power Management (L1/L2), Retrain, Reset, Disable, or LinkError.
module lphy_ltssm_active (
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_active,           // Triggered by exit from LINKINIT state
    
    // Adapter Interface (RDI) 
    input  wire [3:0]  lp_state_req,        // 0100b: L1, 1000b: L2, 1001b: LinkReset, 1011b: Retrain, 1100b: Disabled
    input  wire        lp_linkerror,        // Immediate transition to LinkError
    output logic [3:0] pl_state_sts,        // Tell Adapter we are in ACTIVE (0001b)
    
    // Handshake Status Inputs from Sideband RX (1-Cycle Pulses)
    input  wire        rx_req_l1,        input wire rx_rsp_l1,
    input  wire        rx_req_l2,        input wire rx_rsp_l2,
    input  wire        rx_req_linkreset, input wire rx_rsp_linkreset,
    input  wire        rx_req_disable,   input wire rx_rsp_disable,
    input  wire        rx_req_retrain,   input wire rx_rsp_retrain,
    input  wire        rx_req_linkerror, 
    
    // Handshake Triggers to Sideband TX (1-Cycle Pulses)
    output logic       tx_req_l1,        output logic tx_rsp_l1,
    output logic       tx_req_l2,        output logic tx_rsp_l2,
    output logic       tx_req_linkreset, output logic tx_rsp_linkreset,
    output logic       tx_req_disable,   output logic tx_rsp_disable,
    output logic       tx_req_retrain,   output logic tx_rsp_retrain,
    output logic       tx_req_linkerror,
    
    // Internal PHY Triggers
    input  wire        internal_retrain_req, 
    input  wire        internal_error_req,  // Triggered by uncorrectable PHY faults
    
    // Status Logging Output
    output logic [7:0] active_log,          // Output to Error Log 0 Register (15h)
    
    // State Machine Exits & Control
    output logic        scrambling_en,       // Mandatory LFSR scrambling during ACTIVE
    output logic       exit_to_l1, 
    output logic       exit_to_l2, 
    output logic       exit_to_linkreset, 
    output logic       exit_to_disable, 
    output logic       exit_to_retrain, 
    output logic       exit_to_trainerror
);

    typedef enum logic [1:0] {
        ST_ACTIVE_STEADY = 2'b00,
        ST_WAIT_RSP      = 2'b01,  // We asked to leave, waiting for remote permission
        ST_EXITING       = 2'b10
    } state_t;
    
    state_t state, next_state;
    
    // Latches to remember what state we are trying to exit to
    logic [3:0] target_exit_state; 
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_ACTIVE_STEADY;
            target_exit_state <= 4'h0;
        end else begin
            state <= next_state;
            
            if (!en_active) begin
                target_exit_state <= 4'h0;
            end else if (state == ST_ACTIVE_STEADY) begin
                // Latch Remote Requests first (highest priority)
                if (rx_req_disable)           target_exit_state <= 4'b1100;
                else if (rx_req_linkreset)    target_exit_state <= 4'b1001;
                else if (rx_req_retrain)      target_exit_state <= 4'b1011;
                else if (rx_req_l1)           target_exit_state <= 4'b0100;
                else if (rx_req_l2)           target_exit_state <= 4'b1000;
                
                // Then Local Requests from RDI Adapter or Internal PHY monitors
                else if (lp_state_req == 4'b1100) target_exit_state <= 4'b1100; // Disable
                else if (lp_state_req == 4'b1001) target_exit_state <= 4'b1001; // LinkReset
                else if (lp_state_req == 4'b1011 || internal_retrain_req) target_exit_state <= 4'b1011; // Retrain
                else if (lp_state_req == 4'b0100) target_exit_state <= 4'b0100; // L1
                else if (lp_state_req == 4'b1000) target_exit_state <= 4'b1000; // L2
            end
        end
    end

    // =========================================================================
    // HANDSHAKE ARBITRATION (Balanced priority tree, replaces serial if/else-if)
    // =========================================================================
    // The original ST_ACTIVE_STEADY block picked a winner from 10 mutually
    // exclusive conditions (5 remote requests, then 5 local requests) using a
    // linear if/else-if chain. That's a genuine 10-level serial dependency
    // (to grant a low-priority request you must first disprove every
    // higher-priority one), which alone (~0.6 ns) already blew the ~0.46 ns
    // per-cycle budget at 2 GHz -- before even considering the extra module
    // hops downstream. Rebuilt below as a 16-bit request vector (in priority
    // order, zero-padded) resolved by a balanced priority-encoder tree
    // (depth ~4 instead of ~10).
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
        logic [1:0] n_idx [0:3];
        logic       n_val [0:3];
        logic [2:0] g_idx [0:1];
        logic       g_val [0:1];
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
        else                begin idx = 4'd0; valid = 1'b0; end
    endfunction

    always_comb begin
        req_vec_steady = 16'd0;
        req_vec_steady[WIN_DISABLE]      = rx_req_disable;
        req_vec_steady[WIN_LINKRESET]    = rx_req_linkreset;
        req_vec_steady[WIN_RETRAIN]      = rx_req_retrain;
        req_vec_steady[WIN_L1]           = rx_req_l1;
        req_vec_steady[WIN_L2]           = rx_req_l2;
        req_vec_steady[WIN_LP_DISABLE]   = (lp_state_req == 4'b1100);
        req_vec_steady[WIN_LP_LINKRESET] = (lp_state_req == 4'b1001);
        req_vec_steady[WIN_LP_RETRAIN]   = (lp_state_req == 4'b1011) || internal_retrain_req;
        req_vec_steady[WIN_LP_L1]        = (lp_state_req == 4'b0100);
        req_vec_steady[WIN_LP_L2]        = (lp_state_req == 4'b1000);
        find_first_one16(req_vec_steady, winner_idx, winner_valid);
    end

    // =========================================================================
    // COMBINATIONAL DECISION (drives next_state directly -- unchanged timing
    // contract for the internal FSM register -- and drives the pre-register
    // "c_*" values that get pipelined below for external consumers)
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
        
        // ARCHITECTURAL FIX: Scrambling must be continuously enabled for the ENTIRE DURATION 
        // of en_active, regardless of the internal handshake sub-state.
        scrambling_en = en_active; 
        pl_state_sts  = en_active ? 4'b0001 : 4'b0000; 
        
        // Spec Compliance: Output 15h to Error Log 0 while ACTIVE
        active_log    = en_active ? 8'h15 : 8'h00;
        
        c_tx_req_l1 = 0; c_tx_rsp_l1 = 0;
        c_tx_req_l2 = 0; c_tx_rsp_l2 = 0;
        c_tx_req_linkreset = 0; c_tx_rsp_linkreset = 0;
        c_tx_req_disable = 0;   c_tx_rsp_disable = 0;
        c_tx_req_retrain = 0;   c_tx_rsp_retrain = 0;
        c_tx_req_linkerror = 0;
        
        c_exit_to_l1 = 0; c_exit_to_l2 = 0;
        c_exit_to_linkreset = 0; c_exit_to_disable = 0;
        c_exit_to_retrain = 0; c_exit_to_trainerror = 0;
        
        if (en_active) begin
            // -----------------------------------------------------------------
            // FATAL ERROR OVERRIDE: Bypasses all handshakes immediately
            // -----------------------------------------------------------------
            if (lp_linkerror || internal_error_req || rx_req_linkerror) begin
                c_tx_req_linkerror   = 1'b1; 
                c_exit_to_trainerror = 1'b1;
                next_state           = ST_ACTIVE_STEADY;
            end 
            else begin
                case (state)
                    ST_ACTIVE_STEADY: begin
                        // Winner resolved by the balanced priority tree above
                        // (same priority order as the original if/else-if chain)
                        if (winner_valid) begin
                            next_state = (winner_idx <= WIN_L2) ? ST_EXITING : ST_WAIT_RSP;
                            case (winner_idx)
                                WIN_DISABLE:      begin c_tx_rsp_disable   = 1; c_exit_to_disable   = 1; end
                                WIN_LINKRESET:    begin c_tx_rsp_linkreset = 1; c_exit_to_linkreset = 1; end
                                WIN_RETRAIN:      begin c_tx_rsp_retrain   = 1; c_exit_to_retrain   = 1; end
                                WIN_L1:           begin c_tx_rsp_l1        = 1; c_exit_to_l1         = 1; end
                                WIN_L2:           begin c_tx_rsp_l2        = 1; c_exit_to_l2         = 1; end
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
                        // Wait for the remote PHY to grant our request
                        if      (target_exit_state == 4'b1100 && rx_rsp_disable)   begin c_exit_to_disable = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1001 && rx_rsp_linkreset) begin c_exit_to_linkreset = 1; next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1011 && rx_rsp_retrain)   begin c_exit_to_retrain = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b0100 && rx_rsp_l1)        begin c_exit_to_l1 = 1;        next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1000 && rx_rsp_l2)        begin c_exit_to_l2 = 1;        next_state = ST_EXITING; end
                    end
                    
                    ST_EXITING: begin
                        // Hold the exit flag continuously until the master LTSSM acknowledges and drops en_active
                        if      (target_exit_state == 4'b1100) c_exit_to_disable = 1'b1;
                        else if (target_exit_state == 4'b1001) c_exit_to_linkreset = 1'b1;
                        else if (target_exit_state == 4'b1011) c_exit_to_retrain = 1'b1;
                        else if (target_exit_state == 4'b0100) c_exit_to_l1 = 1'b1;
                        else if (target_exit_state == 4'b1000) c_exit_to_l2 = 1'b1;

                        if (!en_active) next_state = ST_ACTIVE_STEADY;
                    end
                endcase
            end
        end else begin
            next_state = ST_ACTIVE_STEADY;
        end
    end

    // =========================================================================
    // OUTPUT PIPELINE REGISTER
    // =========================================================================
    // Breaks the combinational chain at the module boundary: external
    // consumers (the top-level LTSSM's exit-routing logic, several module
    // hops downstream) now see a clean register launch instead of the raw
    // ripple from the priority tree above. This adds one clk of latency
    // between a handshake decision and its external visibility; the
    // "hold continuously during ST_EXITING" behavior is unaffected since
    // these are level signals held for many cycles once asserted.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_req_l1 <= 1'b0;        tx_rsp_l1 <= 1'b0;
            tx_req_l2 <= 1'b0;        tx_rsp_l2 <= 1'b0;
            tx_req_linkreset <= 1'b0; tx_rsp_linkreset <= 1'b0;
            tx_req_disable <= 1'b0;   tx_rsp_disable <= 1'b0;
            tx_req_retrain <= 1'b0;   tx_rsp_retrain <= 1'b0;
            tx_req_linkerror <= 1'b0;
            exit_to_l1 <= 1'b0; exit_to_l2 <= 1'b0;
            exit_to_linkreset <= 1'b0; exit_to_disable <= 1'b0;
            exit_to_retrain <= 1'b0; exit_to_trainerror <= 1'b0;
        end else begin
            tx_req_l1 <= c_tx_req_l1;               tx_rsp_l1 <= c_tx_rsp_l1;
            tx_req_l2 <= c_tx_req_l2;               tx_rsp_l2 <= c_tx_rsp_l2;
            tx_req_linkreset <= c_tx_req_linkreset; tx_rsp_linkreset <= c_tx_rsp_linkreset;
            tx_req_disable <= c_tx_req_disable;     tx_rsp_disable <= c_tx_rsp_disable;
            tx_req_retrain <= c_tx_req_retrain;     tx_rsp_retrain <= c_tx_rsp_retrain;
            tx_req_linkerror <= c_tx_req_linkerror;
            exit_to_l1 <= c_exit_to_l1;             exit_to_l2 <= c_exit_to_l2;
            exit_to_linkreset <= c_exit_to_linkreset;
            exit_to_disable <= c_exit_to_disable;
            exit_to_retrain <= c_exit_to_retrain;
            exit_to_trainerror <= c_exit_to_trainerror;
        end
    end

endmodule
`default_nettype wire