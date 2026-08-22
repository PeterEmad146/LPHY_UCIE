`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Reset State Controller
/// @description Enforces the mandatory 4ms analog stabilization wait time.
/// (Optimized with a 3-Stage Pipelined Counter for 2GHz Timing Closure)
module lphy_ltssm_reset #(
    // Number of clock cycles required to achieve a 4ms hold time. 
    parameter int CLK_CYCLES_4MS = 400000
)(
    input  wire        clk, 
    input  wire        rst_n,
    
    // Status inputs from Analog Front End (AFE) / Clocking logic
    input  wire        power_stable, 
    input  wire        sb_clk_stable,        
    input  wire        mb_clk_stable,        
    input  wire        mb_clk_slow,          
    
    // Control inputs from SoC / Software / Sideband
    input  wire        soc_reset_n,          
    input  wire        start_link_training,  
    input  wire        sb_rx_wake,           
    
    // Interface to D2D Adapter (RDI)
    input  wire [3:0]  lp_state_req,         
    
    // LTSSM Control
    input  wire        en_reset,             
    
    // State Machine output
    output logic       exit_to_sbinit,       
    output logic       phy_reset_active      
);

    localparam logic [3:0] RDI_STATE_NOP    = 4'b0000;
    localparam logic [3:0] RDI_STATE_ACTIVE = 4'b0001;

    logic        timer_done;
    logic        en_reset_q;            
    logic        reset_reentry;         
    assign reset_reentry = en_reset & ~en_reset_q;
    
    logic        rdi_seen_nop;
    logic        rdi_trigger_valid;
    
    assign rdi_trigger_valid = rdi_seen_nop & (lp_state_req == RDI_STATE_ACTIVE);
    
    logic        training_trigger_active;
    assign training_trigger_active = start_link_training | sb_rx_wake | rdi_trigger_valid;

    // =========================================================================
    // 3-STAGE PIPELINED 20-BIT COUNTER (Replacing the 32-bit Ripple-Carry)
    // =========================================================================
    localparam logic [19:0] TARGET_CYCLES = 20'(CLK_CYCLES_4MS);

    (* dont_touch = "true" *) logic [7:0] cnt_lo;
    (* dont_touch = "true" *) logic [7:0] cnt_mid;
    (* dont_touch = "true" *) logic [3:0] cnt_hi;
    (* dont_touch = "true" *) logic       carry_lo;
    (* dont_touch = "true" *) logic       carry_mid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_lo           <= '0;
            cnt_mid          <= '0;
            cnt_hi           <= '0;
            carry_lo         <= 1'b0;
            carry_mid        <= 1'b0;
            timer_done       <= 1'b0;
            exit_to_sbinit   <= 1'b0;
            phy_reset_active <= 1'b1; 
            en_reset_q       <= 1'b0;
            rdi_seen_nop     <= 1'b0;
        end else begin
            en_reset_q <= en_reset;

            if (reset_reentry || !soc_reset_n) begin
                cnt_lo           <= '0;
                cnt_mid          <= '0;
                cnt_hi           <= '0;
                carry_lo         <= 1'b0;
                carry_mid        <= 1'b0;
                timer_done       <= 1'b0;
                rdi_seen_nop     <= 1'b0;
                if (!soc_reset_n) begin
                    exit_to_sbinit   <= 1'b0;
                    phy_reset_active <= 1'b1;
                end
            end
            else if (en_reset) begin
                
                if (lp_state_req == RDI_STATE_NOP) begin
                    rdi_seen_nop <= 1'b1;
                end

                if (power_stable && sb_clk_stable && mb_clk_stable) begin
                    if (!timer_done) begin
                        cnt_lo   <= cnt_lo + 1'b1;
                        carry_lo <= (cnt_lo == 8'hFF);
                        
                        if (carry_lo) begin
                            cnt_mid   <= cnt_mid + 1'b1;
                            carry_mid <= (cnt_mid == 8'hFF);
                        end else carry_mid <= 1'b0;
                        
                        if (carry_mid) cnt_hi <= cnt_hi + 1'b1;
                    end

                    // Isolated Comparator
                    if ({cnt_hi, cnt_mid, cnt_lo} == TARGET_CYCLES) begin
                        timer_done <= 1'b1;
                    end
                end else begin
                    cnt_lo     <= '0;
                    cnt_mid    <= '0;
                    cnt_hi     <= '0;
                    carry_lo   <= 1'b0;
                    carry_mid  <= 1'b0;
                    timer_done <= 1'b0;
                end

                if (timer_done && mb_clk_slow && training_trigger_active) begin
                    exit_to_sbinit   <= 1'b1;  
                    phy_reset_active <= 1'b0;  
                end else begin
                    exit_to_sbinit   <= 1'b0;
                    phy_reset_active <= 1'b1;  
                end
                
            end else begin
                exit_to_sbinit <= 1'b0;
            end
        end
    end
endmodule
`default_nettype wire