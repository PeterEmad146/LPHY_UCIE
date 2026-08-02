`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband TX Redundancy Remapping (Lane Repair)
/// @description Implements Shift-Left and Shift-Right multiplexing to bypass 
/// broken physical micro-bumps using 4 redundant lanes on Advanced Packages.
module lphy_repair_tx (
    // Logical Data from Byte-to-Lane Mapper
    input  wire [7:0]  tx_logical_data [63:0],
    input  wire [63:0] lane_failed,          // Error mask supplied by LTSSM

    // Physical Data to Analog Front End (AFE)
    output logic [7:0] tx_physical_data [63:0],
    output logic [7:0] tx_redundant_data [3:0],
    
    // Output Enables for AFE Tri-state Control (1 = Drive, 0 = Hi-Z)
    output logic [63:0] tx_physical_oe,
    output logic [3:0]  tx_redundant_oe,
    
    // Status Escalation to LTSSM
    output wire         unrepairable
);

    // =========================================================================
    // Group 1: Lower 32 Lanes [31:0] repaired by Redundant [1:0]
    // =========================================================================
    logic [2:0] fail_cnt_lower; // Widen to 3 bits to prevent overflow
    logic [4:0] f0_l, f1_l;

    always_comb begin
        fail_cnt_lower = 3'd0;
        f0_l = '0;
        f1_l = '0;
        
        // Default 1:1 Mapping and OE Initialization
        tx_physical_oe[31:0] = 32'hFFFF_FFFF;
        tx_redundant_oe[1:0] = 2'b00;
        for (int i = 0; i < 32; i++) begin
            tx_physical_data[i] = tx_logical_data[i];
        end
        tx_redundant_data[0] = 8'h00;
        tx_redundant_data[1] = 8'h00;

        // Priority Encoder: Locate up to 2 faults
        for (int i = 0; i < 32; i++) begin
            if (lane_failed[i]) begin
                if (fail_cnt_lower == 3'd0) f0_l = i[4:0];
                else if (fail_cnt_lower == 3'd1) f1_l = i[4:0];
                fail_cnt_lower++;
            end
        end

        // Apply Shift Multiplexers
        if (fail_cnt_lower == 3'd1) begin
            // Single Failure: Shift Left
            tx_redundant_oe[0]   = 1'b1;
            tx_redundant_data[0] = tx_logical_data[0];
            
            for (int i = 0; i < 32; i++) begin
                if (i < f0_l) begin
                    tx_physical_data[i] = tx_logical_data[i+1];
                end else if (i == f0_l) begin
                    tx_physical_data[i] = 8'h00;
                    tx_physical_oe[i]   = 1'b0; // Command AFE to Tri-state
                end
            end
        end 
        else if (fail_cnt_lower == 3'd2) begin
            // Double Failure: Shift Left (< f0) and Shift Right (> f1)
            tx_redundant_oe[1:0] = 2'b11;
            tx_redundant_data[0] = tx_logical_data[0];
            tx_redundant_data[1] = tx_logical_data[31];
            
            for (int i = 0; i < 32; i++) begin
                if (i < f0_l) begin
                    tx_physical_data[i] = tx_logical_data[i+1];
                end else if (i == f0_l || i == f1_l) begin
                    tx_physical_data[i] = 8'h00;
                    tx_physical_oe[i]   = 1'b0; // Command AFE to Tri-state
                end else if (i > f1_l) begin
                    tx_physical_data[i] = tx_logical_data[i-1];
                end
            end
        end
    end

    // =========================================================================
    // Group 2: Upper 32 Lanes [63:32] repaired by Redundant [3:2]
    // =========================================================================
    logic [2:0] fail_cnt_upper;
    logic [5:0] f0_u, f1_u;

    always_comb begin
        fail_cnt_upper = 3'd0;
        f0_u = '0;
        f1_u = '0;
        
        // Default 1:1 Mapping and OE Initialization
        tx_physical_oe[63:32] = 32'hFFFF_FFFF;
        tx_redundant_oe[3:2]  = 2'b00;
        for (int i = 32; i < 64; i++) begin
            tx_physical_data[i] = tx_logical_data[i];
        end
        tx_redundant_data[2] = 8'h00;
        tx_redundant_data[3] = 8'h00;

        // Priority Encoder: Locate up to 2 faults
        for (int i = 32; i < 64; i++) begin
            if (lane_failed[i]) begin
                if (fail_cnt_upper == 3'd0) f0_u = i[5:0];
                else if (fail_cnt_upper == 3'd1) f1_u = i[5:0];
                fail_cnt_upper++;
            end
        end

        // Apply Shift Multiplexers
        if (fail_cnt_upper == 3'd1) begin
            // Single Failure: Shift Left
            tx_redundant_oe[2]   = 1'b1;
            tx_redundant_data[2] = tx_logical_data[32];
            
            for (int i = 32; i < 64; i++) begin
                if (i < f0_u) begin
                    tx_physical_data[i] = tx_logical_data[i+1];
                end else if (i == f0_u) begin
                    tx_physical_data[i] = 8'h00;
                    tx_physical_oe[i]   = 1'b0;
                end
            end
        end 
        else if (fail_cnt_upper == 3'd2) begin
            // Double Failure: Shift Left (< f0) and Shift Right (> f1)
            tx_redundant_oe[3:2] = 2'b11;
            tx_redundant_data[2] = tx_logical_data[32];
            tx_redundant_data[3] = tx_logical_data[63];
            
            for (int i = 32; i < 64; i++) begin
                if (i < f0_u) begin
                    tx_physical_data[i] = tx_logical_data[i+1];
                end else if (i == f0_u || i == f1_u) begin
                    tx_physical_data[i] = 8'h00;
                    tx_physical_oe[i]   = 1'b0;
                end else if (i > f1_u) begin
                    tx_physical_data[i] = tx_logical_data[i-1];
                end
            end
        end
    end

    // =========================================================================
    // Fatal Error Escalation
    // =========================================================================
    // If either group exceeds its maximum repair capacity of 2 lanes, flag the LTSSM.
    assign unrepairable = (fail_cnt_lower > 3'd2) || (fail_cnt_upper > 3'd2);

endmodule
`default_nettype wire