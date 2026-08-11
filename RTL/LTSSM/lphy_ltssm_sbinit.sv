`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Sideband Initialization State (SBINIT)
/// @description Coordinates the 800 MT/s sideband clock-and-idle pattern training, 
/// 4-way redundancy evaluation for Advanced Packages, and the Out-Of-Reset 
/// sideband message handshake to establish the low-speed communications channel.
module lphy_ltssm_sbinit #(
    // Scaled down 8ms timeout for simulation.
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_sbinit,           // Triggered by exit from RESET state
    input  wire        package_type,        // 0: Advanced, 1: Standard
    
    // Status inputs from sideband RX Logic
    // Result mapping (Advanced): [3]: CKSBRD/DATASBRD, [2]: CKSB/DATASBRD, [1]: CKSBRD/DATASB, [0]: CKSB/DATASB
    input  wire [3:0]  rx_pattern_detected, 
    input  wire        rx_msg_out_of_reset, 
    input  wire        rx_msg_done_req,     
    input  wire        rx_msg_done_resp,    
    
    // Control Outputs to Sideband TX/RX Logic
    output logic       tx_send_pattern,     // 1: Send 64UI clock + 32UI low
    output logic       tx_msg_out_of_reset, // 1: Send {SBINIT Out of Reset}
    output logic       tx_msg_done_req,     // 1: Send {SBINIT done req}
    output logic       tx_msg_done_resp,    // 1: Send {SBINIT done resp}
    output logic [2:0] sb_repair_sel,       // 0: No Repair, 1-3: Mux routing for Advanced Package
    
    // RDI / Protocol Isolation Outputs
    output logic [3:0] pl_state_sts,        // Must be held at 4'b0000 (RESET)
    output logic       pl_inband_pres,      // Must be held at 0
    output logic       pl_protocol_vld,     // Must be held at 0
    
    // State Machine Exits
    output logic       exit_to_mbinit, 
    output logic       exit_to_trainerror       
);

    // Enforce RDI Isolation Rules unconditionally during this state
    assign pl_state_sts    = 4'b0000;
    assign pl_inband_pres  = 1'b0;
    assign pl_protocol_vld = 1'b0;

    typedef enum logic [3:0] {
        ST_IDLE          = 4'b0000, 
        ST_SEND_PATTERN  = 4'b0001, 
        ST_WAIT_4_ITER   = 4'b0010, 
        ST_OUT_OF_RESET  = 4'b0011, 
        ST_WAIT_OOR_RESP = 4'b0100, // Wait for remote to send OOR
        ST_DONE_REQ      = 4'b0101, 
        ST_WAIT_DONE     = 4'b0110, // Wait for Done Resp
        ST_SEND_RESP     = 4'b0111, // Send Done Resp to remote
        ST_DONE          = 4'b1000, 
        ST_ERROR         = 4'b1001
    } state_t;

    state_t state, next_state;
    logic [31:0] timeout_cnt;
    logic [5:0]  wait_cnt;          // 6 bits to count up to 48 iterations
    logic [3:0]  latched_rx_pattern;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= ST_IDLE;
            timeout_cnt        <= 32'd0;
            wait_cnt           <= 6'd0;
            latched_rx_pattern <= 4'd0;
        end else begin
            state <= next_state;
            
            // 8ms Timeout Counter (Resets on major phase transitions)
            if (state == ST_IDLE || state == ST_WAIT_4_ITER || state == ST_DONE || state == ST_ERROR) begin
                timeout_cnt <= 32'd0;
            end else begin
                if (timeout_cnt < TIMEOUT_CYCLES) begin
                    timeout_cnt <= timeout_cnt + 1'b1;
                end
            end
            
            // Latch the prioritized successful RX pattern combination
            if (state == ST_SEND_PATTERN && rx_pattern_detected != 4'b0000) begin
                latched_rx_pattern <= rx_pattern_detected;
            end
            
            // 4-Iteration (48 clock cycles) wait counter
            if (state == ST_WAIT_4_ITER) begin
                wait_cnt <= wait_cnt + 1'b1;
            end else begin
                wait_cnt <= 6'd0;
            end
        end
    end

    always_comb begin
        next_state = state;
        
        tx_send_pattern     = 1'b0;
        tx_msg_out_of_reset = 1'b0;
        tx_msg_done_req     = 1'b0;
        tx_msg_done_resp    = 1'b0; 
        
        exit_to_mbinit      = 1'b0;
        exit_to_trainerror  = 1'b0;
        sb_repair_sel       = 3'b000;
        
        // -----------------------------------------------------------------
        // Resolve Sideband Repair Routing for Advanced Packages
        // -----------------------------------------------------------------
        if (package_type == 1'b0) begin
            // Strict Priority Encoder for functional sideband lock
            if (latched_rx_pattern[0])      sb_repair_sel = 3'b000; // DATASB / CKSB
            else if (latched_rx_pattern[1]) sb_repair_sel = 3'b001; // DATASB / CKSBRD
            else if (latched_rx_pattern[2]) sb_repair_sel = 3'b010; // DATASBRD / CKSB
            else if (latched_rx_pattern[3]) sb_repair_sel = 3'b011; // DATASBRD / CKSBRD
        end 
        
        // Timeout Escalation
        if (timeout_cnt == TIMEOUT_CYCLES && state != ST_IDLE) begin
            next_state = ST_ERROR;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (en_sbinit) next_state = ST_SEND_PATTERN;
                end
                
                ST_SEND_PATTERN: begin
                    tx_send_pattern = 1'b1;
                    if(rx_pattern_detected != 4'b0000) begin
                        next_state = ST_WAIT_4_ITER;
                    end
                end
                
                ST_WAIT_4_ITER: begin
                    tx_send_pattern = 1'b1;
                    // Wait 48 byte-clock cycles (4 iterations of 96 UI) for turnaround alignment
                    if (wait_cnt == 6'd47) begin
                        next_state = ST_OUT_OF_RESET;
                    end
                end
                
                ST_OUT_OF_RESET: begin
                    // Send Local Message
                    tx_msg_out_of_reset = 1'b1;
                    next_state = ST_WAIT_OOR_RESP;
                end
                
                ST_WAIT_OOR_RESP: begin
                    // Keep sending local message until remote acknowledges with their OOR
                    tx_msg_out_of_reset = 1'b1;
                    if(rx_msg_out_of_reset) begin
                        next_state = ST_DONE_REQ;
                    end 
                end
                
                ST_DONE_REQ: begin
                    tx_msg_done_req = 1'b1;
                    next_state = ST_WAIT_DONE;
                end
                
                ST_WAIT_DONE: begin
                    // If we receive a Request, we must send a Response.
                    // If we receive a Response, we exit.
                    if (rx_msg_done_req) begin
                        next_state = ST_SEND_RESP;
                    end else if (rx_msg_done_resp) begin
                        next_state = ST_DONE;
                    end
                end
                
                ST_SEND_RESP: begin
                    tx_msg_done_resp = 1'b1;
                    // After sending the response to unblock the remote PHY, we wait for ours.
                    if (rx_msg_done_resp) begin
                        next_state = ST_DONE;
                    end else begin
                        next_state = ST_WAIT_DONE;
                    end
                end
                
                ST_DONE: begin
                    exit_to_mbinit = 1'b1;
                    if(!en_sbinit) next_state = ST_IDLE; 
                end
                
                ST_ERROR: begin
                    exit_to_trainerror = 1'b1;
                    if(!en_sbinit) next_state = ST_IDLE;
                end
            endcase
        end
    end
endmodule
`default_nettype wire