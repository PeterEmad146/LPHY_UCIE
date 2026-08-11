`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Reset State Controller
/// @description Enforces the mandatory 4ms analog stabilization wait time. 
/// Evaluates the Software, Remote, and Adapter (RDI) triggers to exit to SBINIT.
module lphy_ltssm_reset #(
    // Number of clock cycles required to achieve a 4ms hold time. 
    // Default: For a 100 MHz processing clock (10ns period), 4ms = 400,000 cycles.
    parameter int CLK_CYCLES_4MS = 400000
)(
    input  wire        clk, 
    input  wire        rst_n,
    
    // Status inputs from Analog Front End (AFE) / Clocking logic
    input  wire        power_stable, 
    input  wire        sb_clk_stable,         // Sideband clock running at 800 MHz
    input  wire        mb_clk_stable,         // Mainband clock stable
    input  wire        mb_clk_slow,           // Mainband clock set to slowest rate (4 GT/s)
    
    // Control inputs from SoC / Software / Sideband
    input  wire        soc_reset_n,           // 0: SoC forces PHY reset, 1: SoC releases PHY
    input  wire        start_link_training,   // Trigger from UCIe Link Control register (Software)
    input  wire        sb_rx_wake,            // Trigger from Remote Link Partner (Sideband)
    
    // Interface to D2D Adapter (RDI)
    input  wire [3:0]  lp_state_req,          // Adapter State Request
    
    // LTSSM Control
    input  wire        en_reset,              // High while master LTSSM is in ST_RESET
    
    // State Machine output
    output logic       exit_to_sbinit,        // Asserts high when all conditions met to transition  
    output logic       phy_reset_active       // Clamps PHY to electrical quiet state
);

    // UCIe RDI State Encodings
    localparam logic [3:0] RDI_STATE_NOP    = 4'b0000;
    localparam logic [3:0] RDI_STATE_ACTIVE = 4'b0001;

    logic [31:0] timer_4ms;             
    logic        timer_done;
    
    logic        en_reset_q;            
    logic        reset_reentry;         
    assign reset_reentry = en_reset & ~en_reset_q;
    
    // RDI Rule Enforcement: Adapter must present NOP while in Reset
    logic        rdi_seen_nop;
    logic        rdi_trigger_valid;
    
    // RDI trigger is only valid if we saw a NOP during this reset cycle, 
    // and the Adapter is now requesting ACTIVE.
    assign rdi_trigger_valid = rdi_seen_nop & (lp_state_req == RDI_STATE_ACTIVE);
    
    // Any of the three permitted triggers can initiate link training
    logic        training_trigger_active;
    assign training_trigger_active = start_link_training | sb_rx_wake | rdi_trigger_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer_4ms        <= 32'd0;
            timer_done       <= 1'b0;
            exit_to_sbinit   <= 1'b0;
            phy_reset_active <= 1'b1;   // Active High electrical clamp
            en_reset_q       <= 1'b0;
            rdi_seen_nop     <= 1'b0;
        end else begin
            en_reset_q <= en_reset;

            // On LTSSM re-entry to RESET, clear timers and rule trackers
            if (reset_reentry) begin
                timer_4ms    <= 32'd0;
                timer_done   <= 1'b0;
                rdi_seen_nop <= 1'b0;
            end

            // If the SoC forces a hard reset, clamp everything immediately
            if (!soc_reset_n) begin
                timer_4ms        <= 32'd0;
                timer_done       <= 1'b0;
                exit_to_sbinit   <= 1'b0;
                phy_reset_active <= 1'b1;
                rdi_seen_nop     <= 1'b0;
            end
            else if (en_reset) begin
                
                // Track the RDI "NOP-First" rule while residing in the Reset state
                if (lp_state_req == RDI_STATE_NOP) begin
                    rdi_seen_nop <= 1'b1;
                end

                // 1. Enforce the mandatory 4ms hold for PLLs to stabilize
                if (power_stable && sb_clk_stable && mb_clk_stable) begin
                    if (timer_4ms < CLK_CYCLES_4MS) begin
                        timer_4ms  <= timer_4ms + 1'b1;
                        timer_done <= 1'b0;
                    end else begin
                        timer_done <= 1'b1;
                    end
                end else begin
                    // Reset timer if power/clocks glitch or drop
                    timer_4ms  <= 32'd0;
                    timer_done <= 1'b0;
                end

                // 2. Evaluate exit conditions to SBINIT
                if (timer_done && mb_clk_slow && training_trigger_active) begin
                    exit_to_sbinit   <= 1'b1;  // Trigger master LTSSM
                    phy_reset_active <= 1'b0;  // Release electrical clamps
                end else begin
                    exit_to_sbinit   <= 1'b0;
                    phy_reset_active <= 1'b1;  // Keep PHY clamped
                end
                
            end else begin
                // If LTSSM is not in reset, clear the exit trigger 
                exit_to_sbinit <= 1'b0;
            end
        end
    end
endmodule
`default_nettype wire