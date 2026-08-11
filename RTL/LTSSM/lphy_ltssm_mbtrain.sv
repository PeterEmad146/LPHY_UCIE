`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe LTSSM Mainband Training State (MBTRAIN)
/// @description Orchestrates Stage 2 high-speed training. Sweeps through 
/// Vref optimization, clock-to-data centering, receiver deskew, and speed stability checks.
module lphy_ltssm_mbtrain #(
    // Scaled down 8ms timeout for simulation
    parameter int TIMEOUT_CYCLES = 800000
)(
    input  wire        clk, 
    input  wire        rst_n, 
    input  wire        en_mbtrain,          // Triggered by exit from MBINIT state
    input  wire        package_type,        // 0: Advanced, 1: Standard
    
    // Handshake Status Inputs from Sideband / Calibration logic
    input  wire        valvref_done, 
    input  wire        datavref_done, 
    input  wire        speedidle_done, 
    input  wire        txselfcal_done, 
    input  wire        rxclkcal_done, 
    input  wire        valtraincenter_done, 
    input  wire        valtrainvref_done, 
    input  wire        datatraincenter1_done, 
    input  wire        datatrainvref_done, 
    input  wire        rxdeskew_done, 
    input  wire        datatraincenter2_done, 
    
    // LINKSPEED Status & Decision Logic
    input  wire        linkspeed_done, 
    input  wire        linkspeed_error, 
    input  wire        needs_repair, 
    input  wire        needs_speed_degrade, 
    
    input  wire        repair_done, 
    input  wire        substate_error,      // Triggers immediate exit to TRAINERROR
    
    // Sub-state enables to trigger underlying hardware modules
    output logic       en_valvref, 
    output logic       en_datavref, 
    output logic       en_speedidle, 
    output logic       en_txselfcal, 
    output logic       en_rxclkcal, 
    output logic       en_valtraincenter, 
    output logic       en_valtrainvref, 
    output logic       en_datatraincenter1,
    output logic       en_datatrainvref, 
    output logic       en_rxdeskew, 
    output logic       en_datatraincenter2, 
    output logic       en_linkspeed, 
    output logic       en_repair, 
    
    // RDI / Protocol Isolation Outputs
    output logic [3:0] pl_state_sts,        // Must be held at 4'b0000 (RESET)
    output logic       pl_inband_pres,      // Must be held at 0
    
    // Status Logging Output
    output logic [7:0] mbtrain_substate_log, // For Error Log 0 Register
    
    // State Machine Exits
    output logic       exit_to_linkinit, 
    output logic       exit_to_trainerror
);

    // Enforce RDI Isolation Rules unconditionally during this state
    assign pl_state_sts   = 4'b0000;
    assign pl_inband_pres = 1'b0;

    // Direct mapping to UCIe Specification Error Log 0 Encodings
    typedef enum logic [7:0] {
        ST_IDLE             = 8'h00,
        ST_VALVREF          = 8'h08, 
        ST_DATAVREF         = 8'h09, 
        ST_SPEEDIDLE        = 8'h0A, 
        ST_TXSELFCAL        = 8'h0B,
        ST_RXCLKCAL         = 8'h0C, 
        ST_VALTRAINCENTER   = 8'h0D, 
        ST_VALTRAINVREF     = 8'h0E, // Using 0E per generic progression, though spec maps differently depending on version
        ST_DATATRAINCENTER1 = 8'h0F, // (Offset mapping)
        ST_DATATRAINVREF    = 8'h10, 
        ST_RXDESKEW         = 8'h11, 
        ST_DATATRAINCENTER2 = 8'h12, 
        ST_LINKSPEED        = 8'h13, 
        ST_REPAIR           = 8'h14, 
        ST_DONE             = 8'h1F, 
        ST_ERROR            = 8'hEE
    } state_t;

    state_t state, next_state;
    logic [31:0] timeout_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            timeout_cnt <= 32'd0;
        end else begin
            state <= next_state;
            
            // ARCHITECTURAL FIX: 8ms Timeout is PER SUB-STATE.
            // Reset the timer whenever the state changes.
            if (state != next_state) begin
                timeout_cnt <= 32'd0;
            end else if (state != ST_IDLE && state != ST_DONE && state != ST_ERROR) begin
                if (timeout_cnt < TIMEOUT_CYCLES)
                    timeout_cnt <= timeout_cnt + 1'b1;
            end
        end
    end

    always_comb begin
        next_state          = state;
        en_valvref          = 1'b0;
        en_datavref         = 1'b0;
        en_speedidle        = 1'b0;
        en_txselfcal        = 1'b0;
        en_rxclkcal         = 1'b0;
        en_valtraincenter   = 1'b0;
        en_valtrainvref     = 1'b0;
        en_datatraincenter1 = 1'b0;
        en_datatrainvref    = 1'b0;
        en_rxdeskew         = 1'b0;
        en_datatraincenter2 = 1'b0;
        en_linkspeed        = 1'b0;
        en_repair           = 1'b0;
        exit_to_linkinit    = 1'b0;
        exit_to_trainerror  = 1'b0;
        
        // Output the spec-compliant state encoding to the register block
        mbtrain_substate_log = state;
        
        // State Machine execution
        case (state)
            ST_IDLE: begin
                if(en_mbtrain) begin
                    // Standard Package bypasses initial 4 GT/s Vref sweeps
                    if (package_type == 1'b1) next_state = ST_SPEEDIDLE;
                    else next_state = ST_VALVREF;
                end
            end
                
            ST_VALVREF: begin
                en_valvref = 1'b1;
                if(valvref_done) next_state = ST_DATAVREF;
            end
            
            ST_DATAVREF: begin
                en_datavref = 1'b1;
                if(datavref_done) next_state = ST_SPEEDIDLE;
            end
            
            ST_SPEEDIDLE: begin
                en_speedidle = 1'b1;
                if (speedidle_done) next_state = ST_TXSELFCAL;
            end
            
            ST_TXSELFCAL: begin
                en_txselfcal = 1'b1;
                if (txselfcal_done) next_state = ST_RXCLKCAL;
            end
            
            ST_RXCLKCAL: begin
                en_rxclkcal = 1'b1;
                if (rxclkcal_done) next_state = ST_VALTRAINCENTER;
            end
            
            ST_VALTRAINCENTER: begin
                en_valtraincenter = 1'b1;
                if(valtraincenter_done) begin
                    // Standard Packages bypass high-speed Valid Vref
                    if (package_type == 1'b1) next_state = ST_DATATRAINCENTER1;
                    else next_state = ST_VALTRAINVREF;
                end
            end
            
            ST_VALTRAINVREF: begin
                en_valtrainvref = 1'b1;
                if (valtrainvref_done) next_state = ST_DATATRAINCENTER1;
            end
            
            ST_DATATRAINCENTER1: begin
                en_datatraincenter1 = 1'b1;
                if (datatraincenter1_done) begin
                    // Standard Packages bypass high-speed Data Vref
                    if (package_type == 1'b1) next_state = ST_RXDESKEW;
                    else next_state = ST_DATATRAINVREF;
                end
            end
            
            ST_DATATRAINVREF: begin
                en_datatrainvref = 1'b1;
                if(datatrainvref_done) next_state = ST_RXDESKEW;
            end
            
            ST_RXDESKEW: begin
                en_rxdeskew = 1'b1;
                if (rxdeskew_done) next_state = ST_DATATRAINCENTER2;
            end
            
            ST_DATATRAINCENTER2: begin
                en_datatraincenter2 = 1'b1;
                if (datatraincenter2_done) next_state = ST_LINKSPEED;
            end
            
            ST_LINKSPEED: begin
                en_linkspeed = 1'b1;
                if (linkspeed_done) begin
                    next_state = ST_DONE;
                end else if (linkspeed_error) begin
                    // Crucial Decision Gate for Link Failure Recovery
                    if (needs_repair) next_state = ST_REPAIR;
                    else if (needs_speed_degrade) next_state = ST_SPEEDIDLE;
                    else next_state = ST_ERROR; // Fatal
                end
            end
            
            ST_REPAIR: begin
                en_repair = 1'b1;
                if (repair_done) next_state = ST_TXSELFCAL; // Loop back to retrain newly mapped channels
            end
            
            ST_DONE: begin
                exit_to_linkinit = 1'b1;
                if (!en_mbtrain) next_state = ST_IDLE;
            end
            
            ST_ERROR: begin
                exit_to_trainerror = 1'b1;
                if (!en_mbtrain) next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase

        // Timeout or Substate Error overrides normal transitions
        if ((timeout_cnt == TIMEOUT_CYCLES) || substate_error) begin
            if (state != ST_IDLE && state != ST_ERROR && state != ST_DONE) 
                next_state = ST_ERROR;
        end
    end
endmodule
`default_nettype wire