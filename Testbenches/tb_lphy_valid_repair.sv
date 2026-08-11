`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_valid_repair;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [7:0] tvld_l;
    logic [7:0] trdvld_l;
    logic [7:0] rvld_p;
    logic [7:0] rrdvld_p;
    
    logic [1:0] tx_repair_addr;
    logic [1:0] rx_repair_addr;
    
    wire  [7:0] rvld_l;
    wire  [7:0] rrdvld_l;
    wire  [7:0] tvld_p;
    wire  [7:0] trdvld_p;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_valid_repair dut (
        .tvld_l(tvld_l),
        .trdvld_l(trdvld_l),
        .rvld_l(rvld_l),
        .rrdvld_l(rrdvld_l),
        .tvld_p(tvld_p),
        .trdvld_p(trdvld_p),
        .rvld_p(rvld_p),
        .rrdvld_p(rrdvld_p),
        .tx_repair_addr(tx_repair_addr),
        .rx_repair_addr(rx_repair_addr)
    );

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_valid_repair.fsdb");
        $fsdbDumpvars(0, tb_lphy_valid_repair);

        $display("==================================================");
        $display("Starting Valid Pin Repair Mux Verification");
        $display("==================================================");

        // Baseline logical/physical inputs
        tvld_l   = 8'h0F; // Standard Valid Frame going OUT
        trdvld_l = 8'h00; // Redundant TX usually held at 0
        
        rvld_p   = 8'h0F; // Standard Valid Frame coming IN
        rrdvld_p = 8'hAA; // Distinguishable garbage on redundant analog pin

        // =========================================================
        // TC1: Clean Link (1:1 Mapping)
        // =========================================================
        $display("Running TC1: Clean Link (No Repair)");
        tx_repair_addr = 2'h3; // No Repair
        rx_repair_addr = 2'h3; // No Repair
        #5; 
        
        // TX Check
        if (tvld_p !== 8'h0F || trdvld_p !== 8'h00) 
            $error("[FAIL] TC1 TX: 1:1 Mapping failed.");
            
        // RX Check
        if (rvld_l !== 8'h0F || rrdvld_l !== 8'hAA) 
            $error("[FAIL] TC1 RX: 1:1 Mapping failed.");
            
        $display("[PASS] TC1: Valid pins routed perfectly 1:1.");

        // =========================================================
        // TC2: TX Repair (Fail-over)
        // =========================================================
        $display("Running TC2: TX Pin Repair");
        tx_repair_addr = 2'h0; // Repair TVLD
        rx_repair_addr = 2'h3;
        #5;
        
        if (tvld_p !== 8'h00) 
            $error("[FAIL] TC2 TX: Dead primary bump failed to park at 0.");
        if (trdvld_p !== 8'h0F)
            $error("[FAIL] TC2 TX: Valid frame failed to route to Redundant Pin.");
            
        $display("[PASS] TC2: TX Valid envelope safely routed to Redundant Pin.");

        // =========================================================
        // TC3: RX Repair (Recovery)
        // =========================================================
        $display("Running TC3: RX Pin Repair");
        tx_repair_addr = 2'h3;
        rx_repair_addr = 2'h0; // Repair RVLD
        
        // To simulate a real failure, we kill the primary RX pin, 
        // and put the expected valid frame on the redundant RX pin.
        rvld_p   = 8'h00; // Dead analog pin
        rrdvld_p = 8'h0F; // Recovered valid envelope
        #5;
        
        if (rvld_l !== 8'h0F)
            $error("[FAIL] TC3 RX: Failed to recover valid envelope from Redundant Pin.");
        if (rrdvld_l !== 8'h00)
            $error("[FAIL] TC3 RX: Internal redundant lane failed to clear after consumption.");
            
        $display("[PASS] TC3: RX Valid envelope successfully recovered from Redundant Pin.");

        $display("==================================================");
        $display("Valid Repair Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire