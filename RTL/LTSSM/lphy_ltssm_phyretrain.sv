`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Physical Retrain State Controller (PHYRETRAIN)
/// @description Orchestrates the mid-flight pipeline stall (pl_stallreq) and the 
/// bidirectional sideband negotiation required to recover a failing link. Resolves 
/// repair vs speed degrade conflicts using the strict Priority Table 28.
module lphy_ltssm_phyretrain #(
    // Scaled down 8ms timeout for simulation
    parameter int TIMEOUT_CYCLES = 800000 
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en_phyretrain,       // Asserted when LTSSM is in PHYRETRAIN
    
    // Internal PHY Triggers
    input  wire        local_retrain_trigger, 
    input  wire [2:0]  local_retrain_enc,   // 001b: TXSELFCAL, 010b: SPEEDIDLE, 100b: REPAIR

    // RDI Interface Control & Status
    output logic       pl_stallreq,         // Phase 1 of 4-Phase Stall Handshake
    input  wire        lp_stallack,         // Phase 2 of 4-Phase Stall Handshake
    output logic [3:0] pl_state_sts,        // RDI Status (Must be 1011b for Retrain)

    // Handshake Status Inputs from Sideband RX (1-cycle Pulses)
    input  wire        rx_retrain_init_req,
    input  wire        rx_retrain_init_resp,
    input  wire        rx_retrain_start_req,
    input  wire [2:0]  rx_retrain_enc,
    input  wire        rx_retrain_start_resp,

    // Handshake Triggers to Sideband TX (1-cycle Pulses)
    output logic       tx_retrain_init_req,
    output logic       tx_retrain_init_resp,
    output logic       tx_retrain_start_req,
    output logic       tx_retrain_start_resp,
    output logic [2:0] tx_retrain_enc,
    
    // Status Logging Output
    output logic [7:0] phyretrain_log,      // Output to Error Log 0 Register (13h)

    // State Machine Exits & Control
    output logic       rdi_to_retrain,      // Instructs RDI Wrapper logic to transition
    output logic       phy_in_retrain,      // Mandatory PHY_IN_RETRAIN status variable
    
    output logic       exit_to_txselfcal,
    output logic       exit_to_speedidle,
    output logic       exit_to_repair,
    output logic       exit_to_trainerror
);

    typedef enum logic [3:0] {
        ST_IDLE            = 4'h0,
        // Local Initiated Flow
        ST_LOC_STALL_WAIT  = 4'h1,
        ST_LOC_INIT_REQ    = 4'h2,
        ST_LOC_WAIT_RESP   = 4'h3,
        ST_LOC_START_REQ   = 4'h4,
        ST_LOC_WAIT_START  = 4'h5,
        // Remote Initiated Flow
        ST_REM_STALL_WAIT  = 4'h6,
        ST_REM_INIT_RESP   = 4'h7,
        ST_REM_WAIT_REQ    = 4'h8,
        ST_REM_START_RESP  = 4'h9,
        // Common Exit
        ST_DONE            = 4'hA,
        ST_ERROR           = 4'hB
    } state_t;

    state_t state, next_state;
    logic [31:0] timeout_cnt;
    logic [2:0]  remote_enc_reg;
    logic [2:0]  resolved_enc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            timeout_cnt    <= 32'd0;
            remote_enc_reg <= 3'b001;
        end else begin
            state <= next_state;
            
            // 8ms Residency Timeout is PER SUB-STATE
            if (state != next_state) begin
                timeout_cnt <= 32'd0;
            end else if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                if (timeout_cnt < TIMEOUT_CYCLES)
                    timeout_cnt <= timeout_cnt + 1'b1;
            end

            // Capture remote encoding when start req/resp is received
            if (rx_retrain_start_req || rx_retrain_start_resp) begin
                remote_enc_reg <= rx_retrain_enc;
            end
        end
    end

    // Resolution Logic based on UCIe Priority Table 28
    // 010b (Speed Degrade) > 100b (Repair) > 001b (No Lane Errors)
    always_comb begin
        logic [2:0] remote_val;
        remote_val = (rx_retrain_start_req || rx_retrain_start_resp) ? rx_retrain_enc : remote_enc_reg;
        
        if (local_retrain_enc == 3'b010 || remote_val == 3'b010) begin
            resolved_enc = 3'b010; 
        end else if (local_retrain_enc == 3'b100 || remote_val == 3'b100) begin
            resolved_enc = 3'b100; 
        end else begin
            resolved_enc = 3'b001; 
        end
    end

    always_comb begin
        next_state            = state;
        pl_stallreq           = 1'b0;
        tx_retrain_init_req   = 1'b0;
        tx_retrain_init_resp  = 1'b0;
        tx_retrain_start_req  = 1'b0;
        tx_retrain_start_resp = 1'b0;
        tx_retrain_enc        = 3'b000;
        rdi_to_retrain        = 1'b0;
        
        exit_to_txselfcal     = 1'b0;
        exit_to_speedidle     = 1'b0;
        exit_to_repair        = 1'b0;
        exit_to_trainerror    = 1'b0;

        // Spec Compliance: Output 13h to Error Log 0
        phyretrain_log = (state != ST_IDLE) ? 8'h13 : 8'h00;
        
        // Internal tracking and RDI status override
        phy_in_retrain = en_phyretrain; 
        pl_state_sts   = en_phyretrain ? 4'b1011 : 4'b0000;

        case (state)
            ST_IDLE: begin
                if (en_phyretrain) begin
                    if (local_retrain_trigger) next_state = ST_LOC_STALL_WAIT;
                    else if (rx_retrain_init_req) next_state = ST_REM_STALL_WAIT;
                end
            end

            // =========================================================
            // LOCAL INITIATED FLOW (We asked to retrain)
            // =========================================================
            ST_LOC_STALL_WAIT: begin
                pl_stallreq = 1'b1; // Phase 1: Request Stall
                if (lp_stallack) begin
                    // Phase 2 complete (Pipeline empty). Transition.
                    next_state = ST_LOC_INIT_REQ;
                end
            end
            
            ST_LOC_INIT_REQ: begin
                tx_retrain_init_req = 1'b1;
                // Phase 3 & 4 (De-assert Stallreq) occurs naturally because 
                // pl_stallreq defaults to 0 in this state!
                next_state = ST_LOC_WAIT_RESP;
            end
            
            ST_LOC_WAIT_RESP: begin
                if (rx_retrain_init_resp) begin
                    next_state = ST_LOC_START_REQ;
                end else if (rx_retrain_init_req) begin
                    // CROSSOVER: Both sides asked to retrain simultaneously!
                    // Acknowledge their request and move straight to resolving parameters.
                    tx_retrain_init_resp = 1'b1; 
                    next_state = ST_LOC_START_REQ;
                end
            end
            
            ST_LOC_START_REQ: begin
                rdi_to_retrain = 1'b1; 
                tx_retrain_start_req = 1'b1;
                tx_retrain_enc = local_retrain_enc;
                next_state = ST_LOC_WAIT_START;
            end
            
            ST_LOC_WAIT_START: begin
                rdi_to_retrain = 1'b1;
                if (rx_retrain_start_resp || rx_retrain_start_req) begin
                    if (rx_retrain_start_req) begin
                        // CROSSOVER on Start Req: Acknowledge and resolve
                        tx_retrain_start_resp = 1'b1;
                        tx_retrain_enc = resolved_enc;
                    end
                    next_state = ST_DONE;
                end
            end

            // =========================================================
            // REMOTE INITIATED FLOW (They asked to retrain)
            // =========================================================
            ST_REM_STALL_WAIT: begin
                pl_stallreq = 1'b1; // Phase 1: Request Stall
                if (lp_stallack) begin
                    // Phase 2 complete. Transition.
                    next_state = ST_REM_INIT_RESP;
                end
            end
            
            ST_REM_INIT_RESP: begin
                tx_retrain_init_resp = 1'b1;
                rdi_to_retrain = 1'b1;
                // Phase 3 & 4 (Stallreq drop) occurs naturally
                next_state = ST_REM_WAIT_REQ;
            end
            
            ST_REM_WAIT_REQ: begin
                rdi_to_retrain = 1'b1;
                if (rx_retrain_start_req) begin
                    next_state = ST_REM_START_RESP;
                end
            end
            
            ST_REM_START_RESP: begin
                rdi_to_retrain = 1'b1;
                tx_retrain_start_resp = 1'b1;
                tx_retrain_enc = resolved_enc;
                next_state = ST_DONE;
            end

            // =========================================================
            // COMMON EXITS
            // =========================================================
            ST_DONE: begin
                rdi_to_retrain = 1'b1; 
                // Route LTSM to correct training sub-state
                if (resolved_enc == 3'b010) exit_to_speedidle = 1'b1;
                else if (resolved_enc == 3'b100) exit_to_repair = 1'b1;
                else exit_to_txselfcal = 1'b1;
                
                if (!en_phyretrain) next_state = ST_IDLE;
            end

            ST_ERROR: begin
                exit_to_trainerror = 1'b1;
                if (!en_phyretrain) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase

        // Timeout Escalation
        if (timeout_cnt == TIMEOUT_CYCLES) begin
            if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                next_state = ST_ERROR;
            end
        end
    end
endmodule
`default_nettype wire