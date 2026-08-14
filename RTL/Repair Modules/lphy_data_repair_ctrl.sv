`default_nettype none
`timescale 1ns / 1ps

/// @title UCIe Data Repair & Degradation Controller
/// @description Evaluates the physical failure map. Assigns redundant pins for 
/// Advanced Packages, or triggers half-width degradation / fatal errors.
module lphy_data_repair_ctrl (
    input  wire         clk,
    input  wire         rst_n,
    
    // Configuration & Triggers
    input  wire         package_type, 
    input  wire [63:0]  lane_failed,
    input  wire         check_done,

    // Outputs to Datapath and LTSSM
    output logic [7:0]  trd_repair_addr [3:0],
    output logic [1:0]  lane_map, 
    output logic        is_unrepairable 
);

    // =========================================================================
    // HELPER FUNCTIONS (Rewritten as true balanced trees, O(log2 N) depth)
    // =========================================================================
    //
    // NOTE ON THE FIX:
    // The previous implementations used a for-loop with a loop-carried
    // dependency ("sum = sum + ...", and an early "return" inside a scan).
    // That pattern forces the synthesizer to build a 32-stage *sequential*
    // chain (a ripple accumulator / cascaded priority chain), NOT a parallel
    // tree, regardless of comments claiming otherwise. That 32-deep chain is
    // exactly what showed up on the violating timing path (lane_failed[42]
    // through ~20 XOR/XNOR/NAND stages into the Stage-2 register).
    //
    // Below, both functions are restructured with explicit tree levels so
    // that all additions/comparisons *within* a level are independent of
    // each other, giving a true critical path depth of log2(32) = 5 levels.

    // Priority Encoder: Finds the lowest index '1' in a 32-bit vector.
    // Built as a 4 -> 8 -> 16 -> 32 balanced combining tree (depth 5).
    function automatic logic [4:0] get_first_one(input logic [31:0] vec);
        logic [1:0] enc4  [0:7];   // index within each 4-bit nibble
        logic       val4  [0:7];
        logic [2:0] enc8  [0:3];   // index within each 8-bit group
        logic       val8  [0:3];
        logic [3:0] enc16 [0:1];   // index within each 16-bit half
        logic       val16 [0:1];
        logic [4:0] enc32;
        int i;

        // Level 1: 8 independent 4-bit priority encoders
        for (i = 0; i < 8; i++) begin
            casez (vec[i*4 +: 4])
                4'b???1: begin enc4[i] = 2'd0; val4[i] = 1'b1; end
                4'b??10: begin enc4[i] = 2'd1; val4[i] = 1'b1; end
                4'b?100: begin enc4[i] = 2'd2; val4[i] = 1'b1; end
                4'b1000: begin enc4[i] = 2'd3; val4[i] = 1'b1; end
                default: begin enc4[i] = 2'd0; val4[i] = 1'b0; end
            endcase
        end

        // Level 2: combine pairs of nibbles -> 4 groups of 8
        for (i = 0; i < 4; i++) begin
            if (val4[2*i])
                begin enc8[i] = {1'b0, enc4[2*i]};   val8[i] = 1'b1; end
            else if (val4[2*i+1])
                begin enc8[i] = {1'b1, enc4[2*i+1]}; val8[i] = 1'b1; end
            else
                begin enc8[i] = 3'd0; val8[i] = 1'b0; end
        end

        // Level 3: combine pairs of 8-bit groups -> 2 groups of 16
        for (i = 0; i < 2; i++) begin
            if (val8[2*i])
                begin enc16[i] = {1'b0, enc8[2*i]};   val16[i] = 1'b1; end
            else if (val8[2*i+1])
                begin enc16[i] = {1'b1, enc8[2*i+1]}; val16[i] = 1'b1; end
            else
                begin enc16[i] = 4'd0; val16[i] = 1'b0; end
        end

        // Level 4: combine the two 16-bit halves -> final 32-bit result
        if (val16[0])      enc32 = {1'b0, enc16[0]};
        else if (val16[1]) enc32 = {1'b1, enc16[1]};
        else                enc32 = 5'd0;

        return enc32;
    endfunction

    // Popcount: Counts total '1's using a genuine balanced adder tree
    // (16 -> 8 -> 4 -> 2 -> 1 additions per level, depth 5).
    function automatic logic [5:0] popcount32(input logic [31:0] vec);
        logic [1:0] s1 [0:15];
        logic [2:0] s2 [0:7];
        logic [3:0] s3 [0:3];
        logic [4:0] s4 [0:1];
        logic [5:0] s5;
        int i;

        for (i = 0; i < 16; i++) s1[i] = {1'b0, vec[2*i]} + {1'b0, vec[2*i+1]};
        for (i = 0; i < 8;  i++) s2[i] = s1[2*i] + s1[2*i+1];
        for (i = 0; i < 4;  i++) s3[i] = s2[2*i] + s2[2*i+1];
        for (i = 0; i < 2;  i++) s4[i] = s3[2*i] + s3[2*i+1];
        s5 = s4[0] + s4[1];

        return s5;
    endfunction

    // =========================================================================
    // STAGE 1: Input Latching
    // =========================================================================
    logic [31:0] st1_lower_vec, st1_upper_vec;
    logic        st1_std_lower_fail, st1_std_upper_fail;
    logic        st1_check_done;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_lower_vec      <= '0;
            st1_upper_vec      <= '0;
            st1_std_lower_fail <= 1'b0;
            st1_std_upper_fail <= 1'b0;
            st1_check_done     <= 1'b0;
        end else begin
            st1_lower_vec      <= lane_failed[31:0];
            st1_upper_vec      <= lane_failed[63:32];
            // Standard package only cares about lanes 0-7 and 8-15
            st1_std_lower_fail <= |lane_failed[7:0]; 
            st1_std_upper_fail <= |lane_failed[15:8];
            st1_check_done     <= check_done;
        end
    end

    // =========================================================================
    // STAGE 2: Popcount & First Failure Detection (Parallel Trees)
    // =========================================================================
    logic [5:0]  st2_lower_cnt, st2_upper_cnt;
    logic [4:0]  st2_lower_f0,  st2_upper_f0;
    logic [31:0] st2_lower_vec, st2_upper_vec; 
    logic        st2_std_lower_fail, st2_std_upper_fail;
    logic        st2_check_done;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st2_lower_cnt <= '0; st2_upper_cnt <= '0;
            st2_lower_f0  <= '0; st2_upper_f0  <= '0;
            st2_lower_vec <= '0; st2_upper_vec <= '0;
            st2_std_lower_fail <= 1'b0; st2_std_upper_fail <= 1'b0;
            st2_check_done <= 1'b0;
        end else begin
            st2_lower_cnt <= popcount32(st1_lower_vec);
            st2_upper_cnt <= popcount32(st1_upper_vec);
            st2_lower_f0  <= get_first_one(st1_lower_vec);
            st2_upper_f0  <= get_first_one(st1_upper_vec);
            
            st2_lower_vec <= st1_lower_vec;
            st2_upper_vec <= st1_upper_vec;
            st2_std_lower_fail <= st1_std_lower_fail;
            st2_std_upper_fail <= st1_std_upper_fail;
            st2_check_done <= st1_check_done;
        end
    end

    // =========================================================================
    // STAGE 3: Second Failure Detection (Masking)
    // =========================================================================
    logic [5:0]  st3_lower_cnt, st3_upper_cnt;
    logic [4:0]  st3_lower_f0,  st3_upper_f0;
    logic [4:0]  st3_lower_f1,  st3_upper_f1;
    logic        st3_std_lower_fail, st3_std_upper_fail;
    logic        st3_check_done;
    
    logic [31:0] lower_mask, upper_mask;
    logic [31:0] lower_masked_vec, upper_masked_vec;

    always_comb begin
        // Shift a '1' to the index of the first failure, invert, and bitwise AND to mask it out
        lower_mask = ~(32'd1 << st2_lower_f0);
        upper_mask = ~(32'd1 << st2_upper_f0);
        lower_masked_vec = st2_lower_vec & lower_mask;
        upper_masked_vec = st2_upper_vec & upper_mask;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st3_lower_cnt <= '0; st3_upper_cnt <= '0;
            st3_lower_f0  <= '0; st3_upper_f0  <= '0;
            st3_lower_f1  <= '0; st3_upper_f1  <= '0;
            st3_std_lower_fail <= 1'b0; st3_std_upper_fail <= 1'b0;
            st3_check_done <= 1'b0;
        end else begin
            // Pass along Stage 2 data
            st3_lower_cnt <= st2_lower_cnt;
            st3_upper_cnt <= st2_upper_cnt;
            st3_lower_f0  <= st2_lower_f0;
            st3_upper_f0  <= st2_upper_f0;
            st3_std_lower_fail <= st2_std_lower_fail;
            st3_std_upper_fail <= st2_std_upper_fail;
            
            // Find second failure index using the masked vectors
            st3_lower_f1  <= get_first_one(lower_masked_vec);
            st3_upper_f1  <= get_first_one(upper_masked_vec);
            
            st3_check_done <= st2_check_done;
        end
    end

    // =========================================================================
    // STAGE 4: Final Output Routing (Degradation & Repair Maps)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i=0; i<4; i++) trd_repair_addr[i] <= 8'hFF;
            lane_map        <= 2'b11;
            is_unrepairable <= 1'b0;
        end else if (st3_check_done) begin
            
            if (package_type == 1'b0) begin
                // -------------------------------------------------------------
                // ADVANCED PACKAGE
                // -------------------------------------------------------------
                logic lower_fatal, upper_fatal;
                lower_fatal = (st3_lower_cnt > 6'd2);
                upper_fatal = (st3_upper_cnt > 6'd2);
                
                // Route Redundant Pins
                trd_repair_addr[0] <= (st3_lower_cnt >= 6'd1) ? {3'b000, st3_lower_f0} : 8'hFF;
                trd_repair_addr[1] <= (st3_lower_cnt == 6'd2) ? {3'b000, st3_lower_f1} : 8'hFF;
                
                // Upper lanes offset by 32
                trd_repair_addr[2] <= (st3_upper_cnt >= 6'd1) ? (8'd32 + {3'b000, st3_upper_f0}) : 8'hFF;
                trd_repair_addr[3] <= (st3_upper_cnt == 6'd2) ? (8'd32 + {3'b000, st3_upper_f1}) : 8'hFF;
                
                // Degradation Logic
                if (lower_fatal && upper_fatal) begin
                    is_unrepairable <= 1'b1; lane_map <= 2'b00;
                end else if (upper_fatal) begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b01; // Degrade to lower
                end else if (lower_fatal) begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b10; // Degrade to upper
                end else begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b11; // Full x64
                end

            end else begin
                // -------------------------------------------------------------
                // STANDARD PACKAGE (No Repair, just degrade)
                // -------------------------------------------------------------
                for (int i=0; i<4; i++) trd_repair_addr[i] <= 8'hFF;
                
                if (st3_std_lower_fail && st3_std_upper_fail) begin
                    is_unrepairable <= 1'b1; lane_map <= 2'b00;
                end else if (st3_std_upper_fail) begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b01;
                end else if (st3_std_lower_fail) begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b10;
                end else begin
                    is_unrepairable <= 1'b0; lane_map <= 2'b11;
                end
            end
        end
    end

endmodule
`default_nettype wire