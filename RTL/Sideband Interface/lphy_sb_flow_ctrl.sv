`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Sideband Flow Controller
/// @description Manages local RDI/FDI credits and E2E remote credits for sideband 
/// packet transmission, enforcing strict consumption and bypass rules.
module lphy_sb_flow_ctrl #(
    parameter int LOCAL_CREDITS_INIT  = 32, // Maximum 32 local credits per spec
    parameter int REMOTE_CREDITS_INIT = 4,  // Initial remote credits for Reg Access
    parameter int MAX_REMOTE_CREDITS  = 32  // Saturating limit for remote credits
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rdi_in_reset,   // High when RDI state is Reset
    
    // Request from Sideband Controller/Encoder
    input  wire        req_valid,
    input  wire        is_reg_req,     // High for Register Access Request
    input  wire        is_reg_cpl,     // High for Register Access Completion
    input  wire        is_msg,         // High for Messages (with or without data)
    
    // Authorization Output
    output logic       tx_allowed,     // High if credits are available
    
    // Credit Return Inputs
    input  wire        local_crd_ret,      // pl_cfg_crd: Single credit return
    input  wire [2:0]  remote_crd_ret_val  // Accommodates 1 (Cr bit) or up to 4 (Nop.Crd)
);

    // Credit Counters (Widened to 6 bits to support up to 63 credits safely)
    logic [5:0] local_crd_count;
    logic [5:0] remote_crd_count;
    
    // Next-state arithmetic vectors (7 bits to safely catch overflow during calculation)
    logic [6:0] next_local_crd;
    logic [6:0] next_remote_crd;
    
    // Credit Consumption Flags
    logic consume_local;
    logic consume_remote;
    
    // =========================================================================
    // Consumption & Authorization Logic
    // =========================================================================
    always_comb begin
        consume_local  = 1'b0;
        consume_remote = 1'b0;
        
        // Only consume if the request is valid AND authorized to transmit
        if (req_valid && tx_allowed) begin
            if (is_reg_req) begin
                consume_local  = 1'b1;
                consume_remote = 1'b1;
            end else if (is_msg) begin
                consume_local  = 1'b1;
                // Messages do not consume remote E2E credits per spec
            end
        end
    end
    
    always_comb begin
        if (is_reg_cpl) begin
            // Completions bypass credit checks entirely (Guaranteed Forward Progress)
            tx_allowed = 1'b1;
        end else if (is_reg_req) begin
            // Register requests need BOTH local RDI space and Remote E2E space
            tx_allowed = (local_crd_count > 0) && (remote_crd_count > 0);
        end else if (is_msg) begin
            // Messages only need local RDI space
            tx_allowed = (local_crd_count > 0);
        end else begin
            tx_allowed = 1'b0;
        end
    end

    // =========================================================================
    // Next-State Arithmetic
    // =========================================================================
    always_comb begin
        // Parallel math: Current + Returns - Consumption
        // Padding with 0s ensures exact width matching for the synthesis tool
        next_local_crd  = {1'b0, local_crd_count}  + {6'b0, local_crd_ret}      - {6'b0, consume_local};
        next_remote_crd = {1'b0, remote_crd_count} + {4'b0, remote_crd_ret_val} - {6'b0, consume_remote};
    end
    
    // =========================================================================
    // Sequential Counter Updates
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            local_crd_count  <= LOCAL_CREDITS_INIT[5:0];
            remote_crd_count <= REMOTE_CREDITS_INIT[5:0];
        end else if (rdi_in_reset) begin
            // Re-initialize counters when RDI FSM drops to Reset
            local_crd_count  <= LOCAL_CREDITS_INIT[5:0];
            remote_crd_count <= REMOTE_CREDITS_INIT[5:0];
        end else begin
            // Update Local Counter with Saturation
            if (next_local_crd > LOCAL_CREDITS_INIT[6:0])
                local_crd_count <= LOCAL_CREDITS_INIT[5:0];
            else
                local_crd_count <= next_local_crd[5:0];
                
            // Update Remote Counter with Saturation
            if (next_remote_crd > MAX_REMOTE_CREDITS[6:0])
                remote_crd_count <= MAX_REMOTE_CREDITS[5:0];
            else
                remote_crd_count <= next_remote_crd[5:0];
        end
    end

endmodule
`default_nettype wire