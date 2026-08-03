`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Mainband RX Redundancy Demultiplexer (Lane Repair)
/// @description Inverses the TX redundancy shifts to reconstruct the logical 
/// datapath from the physical analog bumps using 4 redundant lanes.
module lphy_repair_rx (
    // Physical Data from Analog Front End (AFE)
    input  wire [7:0]  rx_physical_data [63:0], 
    input  wire [7:0]  rx_redundant_data [3:0],  
    
    // Failure Map from RX Lane ID Detector
    input  wire [63:0] lane_failed, 
    
    // Logical Data to RX Top
    output logic [7:0] rx_logical_data [63:0],
    
    // Status Escalation to LTSSM
    output wire        unrepairable
);

    // =========================================================================
    // Group 1: Lower 32 Lanes (0 to 31) using Redundant [1:0] 
    // =========================================================================
    logic [2:0] fail_cnt_lower; // FIX: Widened to 3 bits
    logic [4:0] f0_l, f1_l;
    
    always_comb begin
        fail_cnt_lower = 3'd0;
        f0_l = '0;
        f1_l = '0;
        
        // Count failures and record their physical indices
        for (int i = 0; i < 32; i++) begin
            if (lane_failed[i]) begin
                if (fail_cnt_lower == 3'd0) f0_l = i[4:0];
                else if (fail_cnt_lower == 3'd1) f1_l = i[4:0];
                fail_cnt_lower++;
            end
        end
        
        // Default 1:1 Mapping
        for (int i = 0; i < 32; i++) begin
            rx_logical_data[i] = rx_physical_data[i];
        end
        
        if (fail_cnt_lower == 3'd1) begin
            // Single Failure: Reconstruct shift-right mapping
            rx_logical_data[0] = rx_redundant_data[0];    
            for (int i = 1; i <= 31; i++) begin
                if (i <= f0_l) begin
                    rx_logical_data[i] = rx_physical_data[i-1];
                end
            end
        end 
        else if (fail_cnt_lower == 3'd2) begin
            // Two Failures: Reconstruct split shift mapping
            rx_logical_data[0]  = rx_redundant_data[0];   
            rx_logical_data[31] = rx_redundant_data[1];  
            
            for (int i = 1; i <= 30; i++) begin
                if (i <= f0_l) begin
                    rx_logical_data[i] = rx_physical_data[i-1];
                end else if (i >= f1_l) begin
                    rx_logical_data[i] = rx_physical_data[i+1];
                end
            end
        end
    end
    
    // =========================================================================
    // Group 2: Upper 32 Lanes (32 to 63) using Redundant [3:2]
    // =========================================================================
    logic [2:0] fail_cnt_upper; // FIX: Widened to 3 bits
    logic [5:0] f0_u, f1_u;
    
    always_comb begin
        fail_cnt_upper = 3'd0;
        f0_u = '0;
        f1_u = '0;
        
        // Count failures and record their physical indices
        for (int i = 32; i < 64; i++) begin
            if (lane_failed[i]) begin
                if (fail_cnt_upper == 3'd0) f0_u = i[5:0];
                else if (fail_cnt_upper == 3'd1) f1_u = i[5:0];
                fail_cnt_upper++;
            end
        end
        
        // Default 1:1 mapping 
        for (int i = 32; i < 64; i++) begin
            rx_logical_data[i] = rx_physical_data[i];
        end
        
        if (fail_cnt_upper == 3'd1) begin
            // Single Failure: Reconstruct shift-right mapping
            rx_logical_data[32] = rx_redundant_data[2];  
            for (int i = 33; i <= 63; i++) begin
                if (i <= f0_u) begin
                    rx_logical_data[i] = rx_physical_data[i-1];
                end
            end
        end 
        else if (fail_cnt_upper == 3'd2) begin
            // Two Failures: Reconstruct split shift mapping
            rx_logical_data[32] = rx_redundant_data[2]; 
            rx_logical_data[63] = rx_redundant_data[3]; 
            
            for (int i = 33; i <= 62; i++) begin
                if (i <= f0_u) begin
                    rx_logical_data[i] = rx_physical_data[i-1];
                end else if (i >= f1_u) begin
                    rx_logical_data[i] = rx_physical_data[i+1];
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