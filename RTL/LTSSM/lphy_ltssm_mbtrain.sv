`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Mainband Training State (MBTRAIN)
/// @description Orchestrates Stage 2 high-speed training. 
/// (Optimized with Flop-In/Out Shields and Forced One-Hot Encoding for 2GHz)
module lphy_ltssm_mbtrain #(
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire         clk, 
    input  wire         rst_n, 
    input  wire         en_mbtrain,          
    input  wire         package_type,        
    
    // Handshake Status Inputs
    input  wire         valvref_done, 
    input  wire         datavref_done, 
    input  wire         speedidle_done, 
    input  wire         txselfcal_done, 
    input  wire         rxclkcal_done, 
    input  wire         valtraincenter_done, 
    input  wire         valtrainvref_done, 
    input  wire         datatraincenter1_done, 
    input  wire         datatrainvref_done, 
    input  wire         rxdeskew_done, 
    input  wire         datatraincenter2_done, 
    
    // LINKSPEED Status & Decision Logic
    input  wire         linkspeed_done, 
    input  wire         linkspeed_error, 
    input  wire         needs_repair, 
    input  wire         needs_speed_degrade, 
    
    input  wire         repair_done, 
    input  wire         substate_error,      
    
    // Sub-state enables
    output logic        en_valvref, 
    output logic        en_datavref, 
    output logic        en_speedidle, 
    output logic        en_txselfcal, 
    output logic        en_rxclkcal, 
    output logic        en_valtraincenter, 
    output logic        en_valtrainvref, 
    output logic        en_datatraincenter1,
    output logic        en_datatrainvref, 
    output logic        en_rxdeskew, 
    output logic        en_datatraincenter2, 
    output logic        en_linkspeed, 
    output logic        en_repair, 
    
    // RDI Isolation Outputs
    output logic [3:0]  pl_state_sts,        
    output logic        pl_inband_pres,      
    
    // Status Logging
    output logic [7:0]  mbtrain_substate_log, 
    
    // State Machine Exits
    output logic        exit_to_linkinit, 
    output logic        exit_to_trainerror
);

    assign pl_state_sts   = 4'b0000;
    assign pl_inband_pres = 1'b0;

    // ARCHITECTURAL FIX: Stripped the hardcoded 8-bit values!
    // This allows Design Compiler to freely optimize the internal state encoding.
    typedef enum logic [3:0] {
        ST_IDLE, ST_VALVREF, ST_DATAVREF, ST_SPEEDIDLE, ST_TXSELFCAL,
        ST_RXCLKCAL, ST_VALTRAINCENTER, ST_VALTRAINVREF, ST_DATATRAINCENTER1,
        ST_DATATRAINVREF, ST_RXDESKEW, ST_DATATRAINCENTER2, ST_LINKSPEED,
        ST_REPAIR, ST_DONE, ST_ERROR
    } state_t;

    // ARCHITECTURAL FIX: Force One-Hot encoding to completely crush the logic depth
    (* fsm_encoding = "one_hot" *) state_t state, next_state, state_q;

    // =========================================================================
    // 1. INPUT BOUNDARY SHIELD (Flop-In)
    // =========================================================================
    (* dont_touch = "true" *) logic en_mbtrain_q, package_type_q;
    (* dont_touch = "true" *) logic valvref_done_q, datavref_done_q, speedidle_done_q;
    (* dont_touch = "true" *) logic txselfcal_done_q, rxclkcal_done_q, valtraincenter_done_q;
    (* dont_touch = "true" *) logic valtrainvref_done_q, datatraincenter1_done_q, datatrainvref_done_q;
    (* dont_touch = "true" *) logic rxdeskew_done_q, datatraincenter2_done_q, linkspeed_done_q;
    (* dont_touch = "true" *) logic linkspeed_error_q, needs_repair_q, needs_speed_degrade_q;
    (* dont_touch = "true" *) logic repair_done_q, substate_error_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_mbtrain_q <= 0; package_type_q <= 0;
            valvref_done_q <= 0; datavref_done_q <= 0; speedidle_done_q <= 0;
            txselfcal_done_q <= 0; rxclkcal_done_q <= 0; valtraincenter_done_q <= 0;
            valtrainvref_done_q <= 0; datatraincenter1_done_q <= 0; datatrainvref_done_q <= 0;
            rxdeskew_done_q <= 0; datatraincenter2_done_q <= 0; linkspeed_done_q <= 0;
            linkspeed_error_q <= 0; needs_repair_q <= 0; needs_speed_degrade_q <= 0;
            repair_done_q <= 0; substate_error_q <= 0;
        end else begin
            en_mbtrain_q <= en_mbtrain; package_type_q <= package_type;
            valvref_done_q <= valvref_done; datavref_done_q <= datavref_done; speedidle_done_q <= speedidle_done;
            txselfcal_done_q <= txselfcal_done; rxclkcal_done_q <= rxclkcal_done; valtraincenter_done_q <= valtraincenter_done;
            valtrainvref_done_q <= valtrainvref_done; datatraincenter1_done_q <= datatraincenter1_done; datatrainvref_done_q <= datatrainvref_done;
            rxdeskew_done_q <= rxdeskew_done; datatraincenter2_done_q <= datatraincenter2_done; linkspeed_done_q <= linkspeed_done;
            linkspeed_error_q <= linkspeed_error; needs_repair_q <= needs_repair; needs_speed_degrade_q <= needs_speed_degrade;
            repair_done_q <= repair_done; substate_error_q <= substate_error;
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
            state           <= ST_IDLE;
            state_q         <= ST_IDLE;
            cnt_lo          <= '0;
            cnt_mid         <= '0;
            cnt_hi          <= '0;
            carry_lo        <= 1'b0;
            carry_mid       <= 1'b0;
            timeout_reached <= 1'b0;
        end else begin
            state   <= next_state;
            state_q <= state;
            
            if (state != state_q) begin
                cnt_lo          <= '0;
                cnt_mid         <= '0;
                cnt_hi          <= '0;
                carry_lo        <= 1'b0;
                carry_mid       <= 1'b0;
                timeout_reached <= 1'b0;
            end else if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
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
    logic c_en_valvref, c_en_datavref, c_en_speedidle, c_en_txselfcal, c_en_rxclkcal;
    logic c_en_valtraincenter, c_en_valtrainvref, c_en_datatraincenter1, c_en_datatrainvref;
    logic c_en_rxdeskew, c_en_datatraincenter2, c_en_linkspeed, c_en_repair;
    logic c_exit_to_linkinit, c_exit_to_trainerror;
    logic [7:0] c_mbtrain_substate_log;

    // DECOUPLED OUTPUT LOGGING: Re-assigns the Hex values for the Spec Register
    always_comb begin
        case (state)
            ST_IDLE:             c_mbtrain_substate_log = 8'h00;
            ST_VALVREF:          c_mbtrain_substate_log = 8'h08;
            ST_DATAVREF:         c_mbtrain_substate_log = 8'h09;
            ST_SPEEDIDLE:        c_mbtrain_substate_log = 8'h0A;
            ST_TXSELFCAL:        c_mbtrain_substate_log = 8'h0B;
            ST_RXCLKCAL:         c_mbtrain_substate_log = 8'h0C;
            ST_VALTRAINCENTER:   c_mbtrain_substate_log = 8'h0D;
            ST_VALTRAINVREF:     c_mbtrain_substate_log = 8'h0E;
            ST_DATATRAINCENTER1: c_mbtrain_substate_log = 8'h0F;
            ST_DATATRAINVREF:    c_mbtrain_substate_log = 8'h10;
            ST_RXDESKEW:         c_mbtrain_substate_log = 8'h11;
            ST_DATATRAINCENTER2: c_mbtrain_substate_log = 8'h12;
            ST_LINKSPEED:        c_mbtrain_substate_log = 8'h13;
            ST_REPAIR:           c_mbtrain_substate_log = 8'h14;
            ST_DONE:             c_mbtrain_substate_log = 8'h1F;
            ST_ERROR:            c_mbtrain_substate_log = 8'hEE;
            default:             c_mbtrain_substate_log = 8'h00;
        endcase
    end

    always_comb begin
        next_state            = state;
        c_en_valvref          = 1'b0;
        c_en_datavref         = 1'b0;
        c_en_speedidle        = 1'b0;
        c_en_txselfcal        = 1'b0;
        c_en_rxclkcal         = 1'b0;
        c_en_valtraincenter   = 1'b0;
        c_en_valtrainvref     = 1'b0;
        c_en_datatraincenter1 = 1'b0;
        c_en_datatrainvref    = 1'b0;
        c_en_rxdeskew         = 1'b0;
        c_en_datatraincenter2 = 1'b0;
        c_en_linkspeed        = 1'b0;
        c_en_repair           = 1'b0;
        c_exit_to_linkinit    = 1'b0;
        c_exit_to_trainerror  = 1'b0;
        
        // Evaluate strictly using the shielded input registers
        case (state)
            ST_IDLE: begin
                if(en_mbtrain_q) begin
                    if (package_type_q == 1'b1) next_state = ST_SPEEDIDLE;
                    else next_state = ST_VALVREF;
                end
            end
                
            ST_VALVREF: begin
                c_en_valvref = 1'b1;
                if(valvref_done_q) next_state = ST_DATAVREF;
            end
            
            ST_DATAVREF: begin
                c_en_datavref = 1'b1;
                if(datavref_done_q) next_state = ST_SPEEDIDLE;
            end
            
            ST_SPEEDIDLE: begin
                c_en_speedidle = 1'b1;
                if (speedidle_done_q) next_state = ST_TXSELFCAL;
            end
            
            ST_TXSELFCAL: begin
                c_en_txselfcal = 1'b1;
                if (txselfcal_done_q) next_state = ST_RXCLKCAL;
            end
            
            ST_RXCLKCAL: begin
                c_en_rxclkcal = 1'b1;
                if (rxclkcal_done_q) next_state = ST_VALTRAINCENTER;
            end
            
            ST_VALTRAINCENTER: begin
                c_en_valtraincenter = 1'b1;
                if(valtraincenter_done_q) begin
                    if (package_type_q == 1'b1) next_state = ST_DATATRAINCENTER1;
                    else next_state = ST_VALTRAINVREF;
                end
            end
            
            ST_VALTRAINVREF: begin
                c_en_valtrainvref = 1'b1;
                if (valtrainvref_done_q) next_state = ST_DATATRAINCENTER1;
            end
            
            ST_DATATRAINCENTER1: begin
                c_en_datatraincenter1 = 1'b1;
                if (datatraincenter1_done_q) begin
                    if (package_type_q == 1'b1) next_state = ST_RXDESKEW;
                    else next_state = ST_DATATRAINVREF;
                end
            end
            
            ST_DATATRAINVREF: begin
                c_en_datatrainvref = 1'b1;
                if(datatrainvref_done_q) next_state = ST_RXDESKEW;
            end
            
            ST_RXDESKEW: begin
                c_en_rxdeskew = 1'b1;
                if (rxdeskew_done_q) next_state = ST_DATATRAINCENTER2;
            end
            
            ST_DATATRAINCENTER2: begin
                c_en_datatraincenter2 = 1'b1;
                if (datatraincenter2_done_q) next_state = ST_LINKSPEED;
            end
            
            ST_LINKSPEED: begin
                c_en_linkspeed = 1'b1;
                if (linkspeed_done_q) begin
                    next_state = ST_DONE;
                end else if (linkspeed_error_q) begin
                    if (needs_repair_q) next_state = ST_REPAIR;
                    else if (needs_speed_degrade_q) next_state = ST_SPEEDIDLE;
                    else next_state = ST_ERROR; 
                end
            end
            
            ST_REPAIR: begin
                c_en_repair = 1'b1;
                if (repair_done_q) next_state = ST_TXSELFCAL; 
            end
            
            ST_DONE: begin
                c_exit_to_linkinit = 1'b1;
                if (!en_mbtrain_q) next_state = ST_IDLE;
            end
            
            ST_ERROR: begin
                c_exit_to_trainerror = 1'b1;
                if (!en_mbtrain_q) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase

        if (timeout_reached || substate_error_q) begin
            if (state != ST_IDLE && state != ST_ERROR && state != ST_DONE) 
                next_state = ST_ERROR;
        end
    end

    // =========================================================================
    // 4. OUTPUT BOUNDARY SHIELD (Flop-Out)
    // Prevents cross-module logic melting from downstream consumers
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_valvref          <= 1'b0;
            en_datavref         <= 1'b0;
            en_speedidle        <= 1'b0;
            en_txselfcal        <= 1'b0;
            en_rxclkcal         <= 1'b0;
            en_valtraincenter   <= 1'b0;
            en_valtrainvref     <= 1'b0;
            en_datatraincenter1 <= 1'b0;
            en_datatrainvref    <= 1'b0;
            en_rxdeskew         <= 1'b0;
            en_datatraincenter2 <= 1'b0;
            en_linkspeed        <= 1'b0;
            en_repair           <= 1'b0;
            exit_to_linkinit    <= 1'b0;
            exit_to_trainerror  <= 1'b0;
            mbtrain_substate_log <= 8'h00;
        end else begin
            en_valvref          <= c_en_valvref;
            en_datavref         <= c_en_datavref;
            en_speedidle        <= c_en_speedidle;
            en_txselfcal        <= c_en_txselfcal;
            en_rxclkcal         <= c_en_rxclkcal;
            en_valtraincenter   <= c_en_valtraincenter;
            en_valtrainvref     <= c_en_valtrainvref;
            en_datatraincenter1 <= c_en_datatraincenter1;
            en_datatrainvref    <= c_en_datatrainvref;
            en_rxdeskew         <= c_en_rxdeskew;
            en_datatraincenter2 <= c_en_datatraincenter2;
            en_linkspeed        <= c_en_linkspeed;
            en_repair           <= c_en_repair;
            exit_to_linkinit    <= c_exit_to_linkinit;
            exit_to_trainerror  <= c_exit_to_trainerror;
            mbtrain_substate_log <= c_mbtrain_substate_log;
        end
    end
endmodule
`default_nettype wire