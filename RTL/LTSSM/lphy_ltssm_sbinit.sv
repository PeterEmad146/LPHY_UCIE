`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Sideband Initialization State (SBINIT)
/// @description Coordinates the 800 MT/s sideband clock-and-idle pattern training.
/// (Optimized with Full I/O Boundary Shielding and Pipelined Counters for 2GHz)
module lphy_ltssm_sbinit #(
    // Scaled down 8ms timeout for simulation.
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_sbinit,           
    input  wire        package_type,        
    
    // Status inputs from sideband RX Logic
    input  wire [3:0]  rx_pattern_detected, 
    input  wire        rx_msg_out_of_reset, 
    input  wire        rx_msg_done_req,     
    input  wire        rx_msg_done_resp,    
    
    // Control Outputs to Sideband TX/RX Logic
    output logic       tx_send_pattern,     
    output logic       tx_msg_out_of_reset, 
    output logic       tx_msg_done_req,     
    output logic       tx_msg_done_resp,    
    output logic [2:0] sb_repair_sel,       
    
    // RDI / Protocol Isolation Outputs
    output logic [3:0] pl_state_sts,        
    output logic       pl_inband_pres,      
    output logic       pl_protocol_vld,     
    
    // State Machine Exits
    output logic       exit_to_mbinit, 
    output logic       exit_to_trainerror       
);

    // Enforce RDI Isolation Rules unconditionally during this state
    assign pl_state_sts    = 4'b0000;
    assign pl_inband_pres  = 1'b0;
    assign pl_protocol_vld = 1'b0;

    // Stripped hardcoded values to allow One-Hot synthesis
    typedef enum logic [3:0] {
        ST_IDLE, ST_SEND_PATTERN, ST_WAIT_4_ITER, ST_OUT_OF_RESET, 
        ST_WAIT_OOR_RESP, ST_DONE_REQ, ST_WAIT_DONE, ST_SEND_RESP, 
        ST_DONE, ST_ERROR
    } state_t;

    (* fsm_encoding = "one_hot" *) state_t state, next_state, state_q;
    
    logic [5:0]  wait_cnt;          
    logic [3:0]  latched_rx_pattern;

    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic en_sbinit_q;
    (* dont_touch = "true" *) logic package_type_q;
    (* dont_touch = "true" *) logic [3:0] rx_pattern_detected_q;
    (* dont_touch = "true" *) logic rx_msg_out_of_reset_q;
    (* dont_touch = "true" *) logic rx_msg_done_req_q;
    (* dont_touch = "true" *) logic rx_msg_done_resp_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_sbinit_q           <= 1'b0;
            package_type_q        <= 1'b0;
            rx_pattern_detected_q <= 4'b0000;
            rx_msg_out_of_reset_q <= 1'b0;
            rx_msg_done_req_q     <= 1'b0;
            rx_msg_done_resp_q    <= 1'b0;
        end else begin
            en_sbinit_q           <= en_sbinit;
            package_type_q        <= package_type;
            rx_pattern_detected_q <= rx_pattern_detected;
            rx_msg_out_of_reset_q <= rx_msg_out_of_reset;
            rx_msg_done_req_q     <= rx_msg_done_req;
            rx_msg_done_resp_q    <= rx_msg_done_resp;
        end
    end

    // =========================================================================
    // 2. 3-STAGE PIPELINED COUNTER
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
            state              <= ST_IDLE;
            state_q            <= ST_IDLE;
            wait_cnt           <= 6'd0;
            latched_rx_pattern <= 4'd0;
            cnt_lo             <= '0;
            cnt_mid            <= '0;
            cnt_hi             <= '0;
            carry_lo           <= 1'b0;
            carry_mid          <= 1'b0;
            timeout_reached    <= 1'b0;
        end else begin
            state   <= next_state;
            state_q <= state;
            
            // Timer resets on major phase transitions
            if (state == ST_IDLE || state == ST_WAIT_4_ITER || state == ST_DONE || state == ST_ERROR || state != state_q) begin
                cnt_lo          <= '0;
                cnt_mid         <= '0;
                cnt_hi          <= '0;
                carry_lo        <= 1'b0;
                carry_mid       <= 1'b0;
                timeout_reached <= 1'b0;
            end else begin
                if (!timeout_reached) begin
                    cnt_lo   <= cnt_lo + 1'b1;
                    carry_lo <= (cnt_lo == 8'hFF);
                    if (carry_lo) begin
                        cnt_mid   <= cnt_mid + 1'b1;
                        carry_mid <= (cnt_mid == 8'hFF);
                    end else carry_mid <= 1'b0;
                    if (carry_mid) cnt_hi <= cnt_hi + 1'b1;
                end

                if ({cnt_hi, cnt_mid, cnt_lo} == TARGET_CYCLES) timeout_reached <= 1'b1;
            end
            
            // Latch the prioritized successful RX pattern combination
            if (state == ST_SEND_PATTERN && rx_pattern_detected_q != 4'b0000) begin
                latched_rx_pattern <= rx_pattern_detected_q;
            end
            
            // 4-Iteration (48 clock cycles) wait counter
            if (state == ST_WAIT_4_ITER) wait_cnt <= wait_cnt + 1'b1;
            else wait_cnt <= 6'd0;
        end
    end

    // =========================================================================
    // 3. COMBINATIONAL NEXT-STATE & INTERNAL OUTPUTS
    // =========================================================================
    logic c_tx_send_pattern, c_tx_msg_out_of_reset;
    logic c_tx_msg_done_req, c_tx_msg_done_resp;
    logic [2:0] c_sb_repair_sel;
    logic c_exit_to_mbinit, c_exit_to_trainerror;

    always_comb begin
        next_state            = state;
        c_tx_send_pattern     = 1'b0;
        c_tx_msg_out_of_reset = 1'b0;
        c_tx_msg_done_req     = 1'b0;
        c_tx_msg_done_resp    = 1'b0; 
        c_exit_to_mbinit      = 1'b0;
        c_exit_to_trainerror  = 1'b0;
        c_sb_repair_sel       = 3'b000;
        
        if (package_type_q == 1'b0) begin
            if (latched_rx_pattern[0])      c_sb_repair_sel = 3'b000; 
            else if (latched_rx_pattern[1]) c_sb_repair_sel = 3'b001; 
            else if (latched_rx_pattern[2]) c_sb_repair_sel = 3'b010; 
            else if (latched_rx_pattern[3]) c_sb_repair_sel = 3'b011; 
        end 
        
        if (timeout_reached && state != ST_IDLE) begin
            next_state = ST_ERROR;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (en_sbinit_q) next_state = ST_SEND_PATTERN;
                end
                
                ST_SEND_PATTERN: begin
                    c_tx_send_pattern = 1'b1;
                    if(rx_pattern_detected_q != 4'b0000) begin
                        next_state = ST_WAIT_4_ITER;
                    end
                end
                
                ST_WAIT_4_ITER: begin
                    c_tx_send_pattern = 1'b1;
                    if (wait_cnt == 6'd47) begin
                        next_state = ST_OUT_OF_RESET;
                    end
                end
                
                ST_OUT_OF_RESET: begin
                    c_tx_msg_out_of_reset = 1'b1;
                    next_state = ST_WAIT_OOR_RESP;
                end
                
                ST_WAIT_OOR_RESP: begin
                    c_tx_msg_out_of_reset = 1'b1;
                    if(rx_msg_out_of_reset_q) begin
                        next_state = ST_DONE_REQ;
                    end 
                end
                
                ST_DONE_REQ: begin
                    c_tx_msg_done_req = 1'b1;
                    next_state = ST_WAIT_DONE;
                end
                
                ST_WAIT_DONE: begin
                    if (rx_msg_done_req_q) begin
                        next_state = ST_SEND_RESP;
                    end else if (rx_msg_done_resp_q) begin
                        next_state = ST_DONE;
                    end
                end
                
                ST_SEND_RESP: begin
                    c_tx_msg_done_resp = 1'b1;
                    if (rx_msg_done_resp_q) begin
                        next_state = ST_DONE;
                    end else begin
                        next_state = ST_WAIT_DONE;
                    end
                end
                
                ST_DONE: begin
                    c_exit_to_mbinit = 1'b1;
                    if(!en_sbinit_q) next_state = ST_IDLE; 
                end
                
                ST_ERROR: begin
                    c_exit_to_trainerror = 1'b1;
                    if(!en_sbinit_q) next_state = ST_IDLE;
                end
                
                default: next_state = ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // Absorbs external routing delays completely
    // =========================================================================
    (* dont_touch = "true" *) logic tx_pattern_q, tx_oor_q, tx_dreq_q, tx_dresp_q;
    (* dont_touch = "true" *) logic [2:0] repair_sel_q;
    (* dont_touch = "true" *) logic exit_mbinit_q, exit_error_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_pattern_q  <= 1'b0;
            tx_oor_q      <= 1'b0;
            tx_dreq_q     <= 1'b0;
            tx_dresp_q    <= 1'b0;
            repair_sel_q  <= 3'b000;
            exit_mbinit_q <= 1'b0;
            exit_error_q  <= 1'b0;
        end else begin
            tx_pattern_q  <= c_tx_send_pattern;
            tx_oor_q      <= c_tx_msg_out_of_reset;
            tx_dreq_q     <= c_tx_msg_done_req;
            tx_dresp_q    <= c_tx_msg_done_resp;
            repair_sel_q  <= c_sb_repair_sel;
            exit_mbinit_q <= c_exit_to_mbinit;
            exit_error_q  <= c_exit_to_trainerror;
        end
    end

    assign tx_send_pattern     = tx_pattern_q;
    assign tx_msg_out_of_reset = tx_oor_q;
    assign tx_msg_done_req     = tx_dreq_q;
    assign tx_msg_done_resp    = tx_dresp_q;
    assign sb_repair_sel       = repair_sel_q;
    assign exit_to_mbinit      = exit_mbinit_q;
    assign exit_to_trainerror  = exit_error_q;

endmodule
`default_nettype wire