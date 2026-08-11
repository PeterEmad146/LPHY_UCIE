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
    output logic       scrambling_en,       // Mandatory LFSR scrambling during ACTIVE
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

    always_comb begin
        next_state = state;
        
        // ARCHITECTURAL FIX: Scrambling must be continuously enabled for the ENTIRE DURATION 
        // of en_active, regardless of the internal handshake sub-state.
        scrambling_en = en_active; 
        pl_state_sts  = en_active ? 4'b0001 : 4'b0000; 
        
        // Spec Compliance: Output 15h to Error Log 0 while ACTIVE
        active_log    = en_active ? 8'h15 : 8'h00;
        
        tx_req_l1 = 0; tx_rsp_l1 = 0;
        tx_req_l2 = 0; tx_rsp_l2 = 0;
        tx_req_linkreset = 0; tx_rsp_linkreset = 0;
        tx_req_disable = 0;   tx_rsp_disable = 0;
        tx_req_retrain = 0;   tx_rsp_retrain = 0;
        tx_req_linkerror = 0;
        
        exit_to_l1 = 0; exit_to_l2 = 0;
        exit_to_linkreset = 0; exit_to_disable = 0;
        exit_to_retrain = 0; exit_to_trainerror = 0;
        
        if (en_active) begin
            // -----------------------------------------------------------------
            // FATAL ERROR OVERRIDE: Bypasses all handshakes immediately
            // -----------------------------------------------------------------
            if (lp_linkerror || internal_error_req || rx_req_linkerror) begin
                tx_req_linkerror   = 1'b1; 
                exit_to_trainerror = 1'b1;
                next_state         = ST_ACTIVE_STEADY;
            end 
            else begin
                case (state)
                    ST_ACTIVE_STEADY: begin
                        // 1. REMOTE Initiation: They ask, we respond, we transition to EXITING to hold the flag.
                        if      (rx_req_disable)   begin tx_rsp_disable = 1;   exit_to_disable = 1;   next_state = ST_EXITING; end
                        else if (rx_req_linkreset) begin tx_rsp_linkreset = 1; exit_to_linkreset = 1; next_state = ST_EXITING; end
                        else if (rx_req_retrain)   begin tx_rsp_retrain = 1;   exit_to_retrain = 1;   next_state = ST_EXITING; end
                        else if (rx_req_l1)        begin tx_rsp_l1 = 1;        exit_to_l1 = 1;        next_state = ST_EXITING; end
                        else if (rx_req_l2)        begin tx_rsp_l2 = 1;        exit_to_l2 = 1;        next_state = ST_EXITING; end
                        
                        // 2. LOCAL Initiation: We ask, wait for response.
                        else if (lp_state_req == 4'b1100) begin tx_req_disable = 1;   next_state = ST_WAIT_RSP; end
                        else if (lp_state_req == 4'b1001) begin tx_req_linkreset = 1; next_state = ST_WAIT_RSP; end
                        else if (lp_state_req == 4'b1011 || internal_retrain_req) begin tx_req_retrain = 1; next_state = ST_WAIT_RSP; end
                        else if (lp_state_req == 4'b0100) begin tx_req_l1 = 1;        next_state = ST_WAIT_RSP; end
                        else if (lp_state_req == 4'b1000) begin tx_req_l2 = 1;        next_state = ST_WAIT_RSP; end
                    end
                    
                    ST_WAIT_RSP: begin
                        // Wait for the remote PHY to grant our request
                        if      (target_exit_state == 4'b1100 && rx_rsp_disable)   begin exit_to_disable = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1001 && rx_rsp_linkreset) begin exit_to_linkreset = 1; next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1011 && rx_rsp_retrain)   begin exit_to_retrain = 1;   next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b0100 && rx_rsp_l1)        begin exit_to_l1 = 1;        next_state = ST_EXITING; end
                        else if (target_exit_state == 4'b1000 && rx_rsp_l2)        begin exit_to_l2 = 1;        next_state = ST_EXITING; end
                    end
                    
                    ST_EXITING: begin
                        // Hold the exit flag continuously until the master LTSSM acknowledges and drops en_active
                        if      (target_exit_state == 4'b1100) exit_to_disable = 1'b1;
                        else if (target_exit_state == 4'b1001) exit_to_linkreset = 1'b1;
                        else if (target_exit_state == 4'b1011) exit_to_retrain = 1'b1;
                        else if (target_exit_state == 4'b0100) exit_to_l1 = 1'b1;
                        else if (target_exit_state == 4'b1000) exit_to_l2 = 1'b1;

                        if (!en_active) next_state = ST_ACTIVE_STEADY;
                    end
                endcase
            end
        end else begin
            next_state = ST_ACTIVE_STEADY;
        end
    end
endmodule
`default_nettype wire