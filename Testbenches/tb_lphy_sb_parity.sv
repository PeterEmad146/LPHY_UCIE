`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_sb_parity;

    // ---------------------------------------------------------
    // Signals
    // ---------------------------------------------------------
    logic [63:0] tx_header_in;
    logic [63:0] tx_data_in;
    logic        tx_has_data;
    logic        tx_data_is_64b;
    wire  [63:0] tx_header_out;

    logic [63:0] rx_header_in;
    logic [63:0] rx_data_in;
    logic        rx_has_data;
    logic        rx_data_is_64b;
    wire         rx_cp_err;
    wire         rx_dp_err;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_sb_parity dut (
        .tx_header_in(tx_header_in),
        .tx_data_in(tx_data_in),
        .tx_has_data(tx_has_data),
        .tx_data_is_64b(tx_data_is_64b),
        .tx_header_out(tx_header_out),
        .rx_header_in(rx_header_in),
        .rx_data_in(rx_data_in),
        .rx_has_data(rx_has_data),
        .rx_data_is_64b(rx_data_is_64b),
        .rx_cp_err(rx_cp_err),
        .rx_dp_err(rx_dp_err)
    );

    // ---------------------------------------------------------
    // Stimulus & Checking
    // ---------------------------------------------------------
    initial begin
        // Enable waveform dumping for Verdi
        $fsdbDumpfile("tb_lphy_sb_parity.fsdb");
        $fsdbDumpvars(0, tb_lphy_sb_parity);

        $display("==================================================");
        $display("Starting UCIe Sideband Parity Verification");
        $display("==================================================");

        // Test Case 1: 64-bit Payload (Normal Operation)
        tx_header_in   = 64'h0000_0000_0000_000F; // Arbitrary header
        tx_data_in     = 64'hAAAA_BBBB_CCCC_DDDD; // Arbitrary 64-bit data
        tx_has_data    = 1'b1;
        tx_data_is_64b = 1'b1;
        #10;
        // Loopback TX output to RX input for verification
        rx_header_in   = tx_header_out;
        rx_data_in     = tx_data_in;
        rx_has_data    = tx_has_data;
        rx_data_is_64b = tx_data_is_64b;
        #10;
        if (rx_cp_err || rx_dp_err) $error("[FAIL] TC1: Unexpected Error on 64b Payload");
        else $display("[PASS] TC1: 64-bit Payload Parity generation and check successful.");

        // Test Case 2: 32-bit Payload (Testing the Masking Logic)
        // We put garbage in the upper 32 bits to ensure the RTL masks it out.
        tx_data_in     = 64'hDEAD_BEEF_1234_5678; 
        tx_data_is_64b = 1'b0; // Only lower 32 bits are valid
        #10;
        rx_header_in   = tx_header_out;
        rx_data_in     = tx_data_in;
        rx_data_is_64b = 1'b0;
        #10;
        if (rx_cp_err || rx_dp_err) $error("[FAIL] TC2: Masking logic failed on 32b Payload");
        else $display("[PASS] TC2: 32-bit Payload Parity masking successful.");

        // Test Case 3: Message Without Data
        tx_has_data    = 1'b0; // No data payload
        #10;
        rx_header_in   = tx_header_out;
        rx_has_data    = 1'b0;
        #10;
        if (tx_header_out[63] !== 1'b0) $error("[FAIL] TC3: DP bit not driven to 0 on no-data message.");
        if (rx_cp_err || rx_dp_err) $error("[FAIL] TC3: Unexpected Error on No-Data message");
        else $display("[PASS] TC3: No-Data Payload logic successful.");

        // Test Case 4: Forced Control Parity (CP) Error (UIE Escalation)
        tx_has_data    = 1'b1;
        tx_data_is_64b = 1'b1;
        #10;
        rx_header_in   = tx_header_out;
        rx_has_data    = 1'b1;         // FIX: Clear state leakage from TC3
        rx_data_is_64b = 1'b1;         // FIX: Clear state leakage from TC3
        rx_header_in[62] = ~rx_header_in[62]; // Flip the CP bit in transit
        #10;
        if (!rx_cp_err) $error("[FAIL] TC4: Failed to detect Control Parity mismatch.");
        else $display("[PASS] TC4: Control Parity UIE correctly detected.");

        // Test Case 5: Forced Data Parity (DP) Error (UIE Escalation)
        rx_header_in   = tx_header_out; // Restore correct header
        rx_has_data    = 1'b1;          // Ensure state remains correct
        rx_data_is_64b = 1'b1;          // Ensure state remains correct
        rx_header_in[63] = ~rx_header_in[63]; // Flip the DP bit in transit
        #10;
        if (!rx_dp_err) $error("[FAIL] TC5: Failed to detect Data Parity mismatch.");
        else $display("[PASS] TC5: Data Parity UIE correctly detected.");
        $display("==================================================");
        $display("Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire