`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Train Error State Controller (TRAINERROR)
/// @description Orchestrates the graceful teardown of the physical link following 
/// a fatal training or operational failure.
/// (Optimized with Full I/O Boundary Shielding and Pipelined Counters for 2GHz)
module lphy_ltssm_trainerror #(
    // Scaled down 8ms timeout for simulation
    parameter int TIMEOUT_CYCLES = 800000 
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en_trainerror,     // Triggered by Master LTSSM

    // Handshake Status Inputs from Sideband RX (1-Cycle Pulses)
    input  wire        rx_trainerror_req,
    input  wire        rx_trainerror_resp,

    // Adapter / RDI Status
    input  wire        rdi_in_linkerror,  // 1b if RDI pl_state_sts == LinkError

    // Handshake Triggers to Sideband TX
    output logic       tx_trainerror_req,
    output logic       tx_trainerror_resp,

    // Status Logging Output
    output logic [7:0] trainerror_log,    // Output to Error Log 0 Register (16h)

    // State Machine Exits & Control
    output logic       exit_to_reset
);

    typedef enum logic [2:0] {
        ST_IDLE      = 3'h0,
        ST_SEND_REQ  = 3'h1, // Local initiated error
        ST_WAIT_RESP = 3'h2,
        ST_SEND_RESP = 3'h3, // Remote initiated error
        ST_WAIT_RDI  = 3'h4, // Hold if RDI is in LinkError
        ST_DONE      = 3'h5
    } state_t;

    state_t state, next_state, state_q;
    
    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic en_trainerror_q;
    (* dont_touch = "true" *) logic rx_trainerror_req_q;
    (* dont_touch = "true" *) logic rx_trainerror_resp_q;
    (* dont_touch = "true" *) logic rdi_in_linkerror_q;
    
    // Latches to catch 1-cycle sideband pulses
    logic rcvd_req;
    logic rcvd_resp;

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
            state                <= ST_IDLE;
            state_q              <= ST_IDLE;
            en_trainerror_q      <= 1'b0;
            rx_trainerror_req_q  <= 1'b0;
            rx_trainerror_resp_q <= 1'b0;
            rdi_in_linkerror_q   <= 1'b0;
            rcvd_req             <= 1'b0;
            rcvd_resp            <= 1'b0;
            cnt_lo               <= '0;
            cnt_mid              <= '0;
            cnt_hi               <= '0;
            carry_lo             <= 1'b0;
            carry_mid            <= 1'b0;
            timeout_reached      <= 1'b0;
        end else begin
            // Absorb External Delays
            en_trainerror_q      <= en_trainerror;
            rx_trainerror_req_q  <= rx_trainerror_req;
            rx_trainerror_resp_q <= rx_trainerror_resp;
            rdi_in_linkerror_q   <= rdi_in_linkerror;

            state   <= next_state;
            state_q <= state;
            
            // Pulse Latching Logic (Evaluated strictly on the shielded signals)
            if (state != ST_IDLE && next_state == ST_IDLE) begin
                rcvd_req  <= 1'b0;
                rcvd_resp <= 1'b0;
            end else begin
                if (rx_trainerror_req_q)  rcvd_req  <= 1'b1;
                if (rx_trainerror_resp_q) rcvd_resp <= 1'b1;
            end

            // Timer runs ONLY during ST_WAIT_RESP
            if (state != state_q || state != ST_WAIT_RESP) begin
                cnt_lo          <= '0;
                cnt_mid         <= '0;
                cnt_hi          <= '0;
                carry_lo        <= 1'b0;
                carry_mid       <= 1'b0;
                timeout_reached <= 1'b0;
            end else if (state == ST_WAIT_RESP) begin
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
    logic c_tx_trainerror_req, c_tx_trainerror_resp, c_exit_to_reset;
    logic [7:0] c_trainerror_log;

    always_comb begin
        next_state           = state;
        c_tx_trainerror_req  = 1'b0;
        c_tx_trainerror_resp = 1'b0;
        c_exit_to_reset      = 1'b0;
        c_trainerror_log     = (state != ST_IDLE) ? 8'h16 : 8'h00;

        // Evaluate using ONLY the internal, registered signals
        case (state)
            ST_IDLE: begin
                if (en_trainerror_q) begin
                    if (rx_trainerror_req_q || rcvd_req) next_state = ST_SEND_RESP;
                    else next_state = ST_SEND_REQ;
                end
            end

            ST_SEND_REQ: begin
                c_tx_trainerror_req = 1'b1;
                next_state = ST_WAIT_RESP;
            end
            
            ST_WAIT_RESP: begin
                if (rx_trainerror_resp_q || rcvd_resp || timeout_reached) begin
                    next_state = ST_WAIT_RDI;
                end 
                else if (rx_trainerror_req_q || rcvd_req) begin
                    c_tx_trainerror_resp = 1'b1;
                    next_state = ST_WAIT_RDI;
                end
            end

            ST_SEND_RESP: begin
                c_tx_trainerror_resp = 1'b1;
                next_state = ST_WAIT_RDI;
            end

            ST_WAIT_RDI: begin
                if (!rdi_in_linkerror_q) begin
                    next_state = ST_DONE;
                end
            end

            ST_DONE: begin
                c_exit_to_reset = 1'b1;
                if (!en_trainerror_q) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_trainerror_req  <= 1'b0;
            tx_trainerror_resp <= 1'b0;
            exit_to_reset      <= 1'b0;
            trainerror_log     <= 8'h00;
        end else begin
            tx_trainerror_req  <= c_tx_trainerror_req;
            tx_trainerror_resp <= c_tx_trainerror_resp;
            exit_to_reset      <= c_exit_to_reset;
            trainerror_log     <= c_trainerror_log;
        end
    end
endmodule
`default_nettype wire