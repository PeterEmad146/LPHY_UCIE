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
    // HELPER FUNCTIONS (Balanced trees, O(log2 N) depth)
    // =========================================================================
    //
    // NOTE ON THE FIX:
    // The original code counted failures with "fail_cnt++" inside a 32-iteration
    // for-loop and captured f0/f1 via if/else-if checks against that *same*
    // running counter. Both the increment and the index capture are therefore
    // one long loop-carried dependency chain (up to 32 sequential levels), and
    // because this module has no clock/registers of its own, that whole chain
    // sits directly on the combinational path into the downstream register --
    // exactly what the STA report shows (lane_failed[14] alone costs 18 gates).
    //
    // Below, counting and indexing are rebuilt as independent balanced trees
    // (popcount tree + priority-encoder tree), so the critical path depth is a
    // small constant (~5-6 levels) no matter which bit(s) failed.

    // Popcount: use the built-in population-count construct rather than a
    // hand-rolled tree. NOTE: an earlier version of this function chained
    // five stages of growing-width ripple-carry adds (2b->3b->4b->5b->6b).
    // That looked like a "balanced tree" by stage COUNT, but each stage's
    // own carry-propagate depth grows with its width, so the real critical
    // path was ~1+2+3+4+5 = 15 gate levels -- barely better than the
    // original ripple accumulator (exactly what showed up in the last STA
    // report as a long FADDX/HADDX chain). A true O(log n) popcount needs
    // carry-SAVE compression (3:2 compressors) with carry resolution
    // deferred to a single final adder, which synthesis tools implement
    // correctly for $countones far more reliably than a hand-rolled tree.
    function automatic logic [5:0] popcount32(input logic [31:0] vec);
        return 6'($countones(vec));
    endfunction

    // Find-first-two: a SINGLE balanced tree pass that produces the index of
    // the lowest AND second-lowest set bit at once, depth 5 (no dynamic
    // barrel shift, no dependency between idx0 and idx1). Each tree node
    // carries (idx0, has0, idx1, has1) for its subrange; two children merge
    // in O(1) gates:
    //   has0 = L.has0 | R.has0
    //   idx0 = L.has0 ? L.idx0 : R.idx0                      (with side tag)
    //   has1 = L.has1 | (L.has0 & R.has0) | (!L.has0 & R.has1)
    //   idx1 = L.has1 ? L.idx1 : (L.has0&R.has0) ? R.idx0 : R.idx1
    function automatic void find_first_two(input  logic [31:0] vec,
                                            output logic [4:0]  idx0,
                                            output logic        has0,
                                            output logic [4:0]  idx1,
                                            output logic        has1);
        // ---- Leaf level: 8 independent 4-bit nibbles ----
        logic [1:0] n_idx0 [0:7];
        logic       n_has0 [0:7];
        logic [1:0] n_idx1 [0:7];
        logic       n_has1 [0:7];
        // ---- Level 1 merge: 4 groups of 8 bits ----
        logic [2:0] g8_idx0 [0:3];
        logic       g8_has0 [0:3];
        logic [2:0] g8_idx1 [0:3];
        logic       g8_has1 [0:3];
        // ---- Level 2 merge: 2 groups of 16 bits ----
        logic [3:0] g16_idx0 [0:1];
        logic       g16_has0 [0:1];
        logic [3:0] g16_idx1 [0:1];
        logic       g16_has1 [0:1];
        int i;

        for (i = 0; i < 8; i++) begin : nibble_encode
            logic [3:0] nb, masked;
            nb = vec[i*4 +: 4];
            casez (nb)
                4'b???1: begin n_idx0[i] = 2'd0; n_has0[i] = 1'b1; end
                4'b??10: begin n_idx0[i] = 2'd1; n_has0[i] = 1'b1; end
                4'b?100: begin n_idx0[i] = 2'd2; n_has0[i] = 1'b1; end
                4'b1000: begin n_idx0[i] = 2'd3; n_has0[i] = 1'b1; end
                default: begin n_idx0[i] = 2'd0; n_has0[i] = 1'b0; end
            endcase
            masked = n_has0[i] ? (nb & ~(4'd1 << n_idx0[i])) : 4'd0;
            casez (masked)
                4'b???1: begin n_idx1[i] = 2'd0; n_has1[i] = 1'b1; end
                4'b??10: begin n_idx1[i] = 2'd1; n_has1[i] = 1'b1; end
                4'b?100: begin n_idx1[i] = 2'd2; n_has1[i] = 1'b1; end
                4'b1000: begin n_idx1[i] = 2'd3; n_has1[i] = 1'b1; end
                default: begin n_idx1[i] = 2'd0; n_has1[i] = 1'b0; end
            endcase
        end

        for (i = 0; i < 4; i++) begin : merge_l1
            logic l_has0, r_has0, l_has1, r_has1;
            l_has0 = n_has0[2*i];   r_has0 = n_has0[2*i+1];
            l_has1 = n_has1[2*i];   r_has1 = n_has1[2*i+1];
            g8_has0[i] = l_has0 | r_has0;
            g8_idx0[i] = l_has0 ? {1'b0, n_idx0[2*i]} : {1'b1, n_idx0[2*i+1]};
            g8_has1[i] = l_has1 | (l_has0 & r_has0) | (!l_has0 & r_has1);
            if (l_has1)              g8_idx1[i] = {1'b0, n_idx1[2*i]};
            else if (l_has0 & r_has0) g8_idx1[i] = {1'b1, n_idx0[2*i+1]};
            else                      g8_idx1[i] = {1'b1, n_idx1[2*i+1]};
        end

        for (i = 0; i < 2; i++) begin : merge_l2
            logic l_has0, r_has0, l_has1, r_has1;
            l_has0 = g8_has0[2*i];   r_has0 = g8_has0[2*i+1];
            l_has1 = g8_has1[2*i];   r_has1 = g8_has1[2*i+1];
            g16_has0[i] = l_has0 | r_has0;
            g16_idx0[i] = l_has0 ? {1'b0, g8_idx0[2*i]} : {1'b1, g8_idx0[2*i+1]};
            g16_has1[i] = l_has1 | (l_has0 & r_has0) | (!l_has0 & r_has1);
            if (l_has1)               g16_idx1[i] = {1'b0, g8_idx1[2*i]};
            else if (l_has0 & r_has0) g16_idx1[i] = {1'b1, g8_idx0[2*i+1]};
            else                       g16_idx1[i] = {1'b1, g8_idx1[2*i+1]};
        end

        // ---- Level 3 merge: root, 32 bits ----
        begin : merge_root
            logic l_has0, r_has0, l_has1, r_has1;
            l_has0 = g16_has0[0]; r_has0 = g16_has0[1];
            l_has1 = g16_has1[0]; r_has1 = g16_has1[1];
            has0 = l_has0 | r_has0;
            idx0 = l_has0 ? {1'b0, g16_idx0[0]} : {1'b1, g16_idx0[1]};
            has1 = l_has1 | (l_has0 & r_has0) | (!l_has0 & r_has1);
            if (l_has1)               idx1 = {1'b0, g16_idx1[0]};
            else if (l_has0 & r_has0) idx1 = {1'b1, g16_idx0[1]};
            else                       idx1 = {1'b1, g16_idx1[1]};
        end
    endfunction

    // =========================================================================
    // Group 1: Lower 32 Lanes (0 to 31) using Redundant [1:0] 
    // =========================================================================
    logic [5:0] fail_cnt_lower_full;   // FIX: full-width count, no wraparound
    logic [4:0] f0_l, f1_l;
    logic       f0_l_valid, f1_l_valid;
    logic [7:0] rx_logical_data_comb [63:0];

    // =========================================================================
    // Group 2: Upper 32 Lanes (32 to 63) using Redundant [3:2]
    // =========================================================================
    logic [5:0] fail_cnt_upper_full;   // FIX: full-width count, no wraparound
    logic [4:0] f0_u_raw, f1_u_raw;
    logic [5:0] f0_u, f1_u;
    logic       f0_u_valid, f1_u_valid;

    always_comb begin
        // Count (independent tree) and both failure indices (single parallel
        // find-first-two tree -- no serial dependency between f0_l and f1_l)
        fail_cnt_lower_full = popcount32(lane_failed[31:0]);
        find_first_two(lane_failed[31:0], f0_l, f0_l_valid, f1_l, f1_l_valid);

        // Count (independent tree) and both failure indices (single parallel
        // find-first-two tree), relative to bit 0 of the upper half
        fail_cnt_upper_full = popcount32(lane_failed[63:32]);
        find_first_two(lane_failed[63:32], f0_u_raw, f0_u_valid, f1_u_raw, f1_u_valid);

        // Rebase indices to absolute lane numbers (32..63), matching original semantics
        f0_u = {1'b0, f0_u_raw} + 6'd32;
        f1_u = {1'b0, f1_u_raw} + 6'd32;

        // Default 1:1 mapping
        for (int i = 0; i < 64; i++) begin
            rx_logical_data_comb[i] = rx_physical_data[i];
        end

        if (fail_cnt_lower_full == 6'd1) begin
            // Single Failure: Reconstruct shift-right mapping
            rx_logical_data_comb[0] = rx_redundant_data[0];
            for (int i = 1; i <= 31; i++) begin
                if (i <= f0_l) begin
                    rx_logical_data_comb[i] = rx_physical_data[i-1];
                end
            end
        end else if (fail_cnt_lower_full == 6'd2) begin
            // Two Failures: Reconstruct split shift mapping
            rx_logical_data_comb[0] = rx_redundant_data[0];
            rx_logical_data_comb[31] = rx_redundant_data[1];

            for (int i = 1; i <= 30; i++) begin
                if (i <= f0_l) begin
                    rx_logical_data_comb[i] = rx_physical_data[i-1];
                end else if (i >= f1_l) begin
                    rx_logical_data_comb[i] = rx_physical_data[i+1];
                end
            end
        end

        if (fail_cnt_upper_full == 6'd1) begin
            // Single Failure: Reconstruct shift-right mapping
            rx_logical_data_comb[32] = rx_redundant_data[2];
            for (int i = 33; i <= 63; i++) begin
                if (i <= f0_u) begin
                    rx_logical_data_comb[i] = rx_physical_data[i-1];
                end
            end
        end else if (fail_cnt_upper_full == 6'd2) begin
            // Two Failures: Reconstruct split shift mapping
            rx_logical_data_comb[32] = rx_redundant_data[2];
            rx_logical_data_comb[63] = rx_redundant_data[3];

            for (int i = 33; i <= 62; i++) begin
                if (i <= f0_u) begin
                    rx_logical_data_comb[i] = rx_physical_data[i-1];
                end else if (i >= f1_u) begin
                    rx_logical_data_comb[i] = rx_physical_data[i+1];
                end
            end
        end

        rx_logical_data = rx_logical_data_comb;
    end

    // =========================================================================
    // Fatal Error Escalation
    // =========================================================================
    // If either group exceeds its maximum repair capacity of 2 lanes, flag the LTSSM.
    assign unrepairable = (fail_cnt_lower_full > 6'd2) || (fail_cnt_upper_full > 6'd2);

endmodule
`default_nettype wire