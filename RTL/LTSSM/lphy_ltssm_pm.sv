`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Power Management State (L1/L2)
/// @description Orchestrates physical layer sleep states. Manages the wake-up 
/// sideband handshakes initiated either locally by the Adapter or remotely by the 
/// Link Partner, enforcing the 2us handshake timeout rule.
module lphy_ltssm_pm #(
    // Number of cycles for 2us timeout. 
    // Default: 100MHz clock (10ns) = 200 cycles.
    parameter int TIMEOUT_2US_CYCLES = 200 
)(
    input  wire        clk, 
    input  wire        rst_n,
    
    // Entry Triggers from Master LTSSM
    input  wire        en_l1, 
    input  wire        en_l2, 
    
    // Adapter Interface (RDI) State Requests & Status
    input  wire [3:0]  lp_state_req,        // Looking for 4'b0001 (Active) to initiate wake-up
    output logic [3:0] pl_state_sts,        // Tell Adapter we are successfully asleep
    output logic       pl_inband_pres,      // De-asserted during sleep
    
    // Handshake Status Inputs from Sideband RX (1-Cycle Pulses)
    input  wire        rx_req_active,       // Remote Link partner requesting PM exit
    input  wire        rx_rsp_active,       // Remote Link partner acknowledging our wake-up
    
    // Handshake Triggers to Sideband TX (1-Cycle Pulses)
    output logic       tx_req_active, 
    output logic       tx_rsp_active, 
    
    // Status Logging Output
    output logic [7:0] pm_log,              // Output to Error Log 0 Register (17h)
    
    // State Machine Exits
    output logic       exit_to_speedidle,   // L1 wake-up routes to MBTRAIN.SPEEDIDLE
    output logic       exit_to_reset,       // L2 wake-up routes to RESET
    output logic       exit_to_trainerror   // 2us Timeout
);

    typedef enum logic [2:0] {
        ST_IDLE     = 3'h0, 
        ST_L1       = 3'h1, 
        ST_L2       = 3'h2,
        ST_WAKE_REQ = 3'h3,  // Waiting for Remote PHY to acknowledge wake-up
        ST_EXITING  = 3'h4,  // Holding exit flags high
        ST_ERROR    = 3'h5
    } state_t;
    
    state_t state, next_state;
    
    // Track which sleep state we are waking up from
    logic was_in_l2; 
    
    // 2us Handshake Timeout Counter
    logic [15:0] timeout_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            was_in_l2 <= 1'b0;
            timeout_cnt <= 16'd0;
        end else begin
            state <= next_state;
            
            // Track the origin state for routing the exit path
            if (state == ST_L2) was_in_l2 <= 1'b1;
            else if (state == ST_IDLE) was_in_l2 <= 1'b0;
            
            // 2us Timeout Timer (Active ONLY when waiting for remote response)
            if (state == ST_WAKE_REQ) begin
                timeout_cnt <= timeout_cnt + 1'b1;
            end else begin
                timeout_cnt <= 16'd0;
            end
        end
    end
    
    always_comb begin
        next_state = state;
        
        exit_to_speedidle  = 1'b0;
        exit_to_reset      = 1'b0;
        exit_to_trainerror = 1'b0;
        
        tx_req_active = 1'b0;
        tx_rsp_active = 1'b0;
        
        pl_state_sts   = 4'b0000;
        pl_inband_pres = 1'b0; // Presence is 0 while asleep!
        
        // Spec Compliance: Output 17h to Error Log 0
        pm_log = (state != ST_IDLE) ? 8'h17 : 8'h00;
        
        case (state)
            ST_IDLE: begin
                if (en_l1) next_state = ST_L1;
                else if (en_l2) next_state = ST_L2;
            end
            
            ST_L1: begin
                pl_state_sts = 4'b0100; // Tell Adapter we are in L1
                
                // 1. Remote PHY wakes us up
                if (rx_req_active) begin
                    tx_rsp_active = 1'b1;
                    next_state = ST_EXITING;
                end 
                // 2. Local Adapter wakes us up
                else if (lp_state_req == 4'b0001) begin
                    tx_req_active = 1'b1;
                    next_state = ST_WAKE_REQ;
                end
            end
            
            ST_L2: begin
                pl_state_sts = 4'b1000; // Tell Adapter we are in L2
                
                // 1. Remote PHY wakes us up
                if (rx_req_active) begin
                    tx_rsp_active = 1'b1;
                    next_state = ST_EXITING;
                end 
                // 2. Local Adapter wakes us up
                else if (lp_state_req == 4'b0001) begin
                    tx_req_active = 1'b1;
                    next_state = ST_WAKE_REQ;
                end 
            end
            
            ST_WAKE_REQ: begin
                // Hold the sleep status while waking up
                pl_state_sts = was_in_l2 ? 4'b1000 : 4'b0100;
                
                if (rx_rsp_active) begin
                    next_state = ST_EXITING;
                end else if (timeout_cnt >= TIMEOUT_2US_CYCLES) begin
                    // 2us rule violation!
                    next_state = ST_ERROR;
                end
            end
            
            ST_EXITING: begin
                if (was_in_l2) exit_to_reset = 1'b1;
                else exit_to_speedidle = 1'b1;
                
                if (!en_l1 && !en_l2) next_state = ST_IDLE;
            end
            
            ST_ERROR: begin
                exit_to_trainerror = 1'b1;
                if (!en_l1 && !en_l2) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
endmodule
`default_nettype wire