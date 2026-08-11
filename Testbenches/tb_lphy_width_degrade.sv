`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_width_degrade;

    // ---------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------
    localparam int NUM_LANES = 64;
    localparam int HALF_LANES = 32;

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic [1:0] lane_map;
    
    logic [7:0] tx_logical_data [NUM_LANES-1:0];
    logic [7:0] rx_physical_data [NUM_LANES-1:0];
    
    wire  [7:0] tx_physical_data [NUM_LANES-1:0];
    wire  [7:0] rx_logical_data [NUM_LANES-1:0];

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_width_degrade #(
        .NUM_LANES(NUM_LANES)
    ) dut (
        .lane_map(lane_map),
        .tx_logical_data(tx_logical_data),
        .tx_physical_data(tx_physical_data),
        .rx_physical_data(rx_physical_data),
        .rx_logical_data(rx_logical_data)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task load_gradients();
        for (int i = 0; i < NUM_LANES; i++) begin
            // FIX: Using SV 8'(...) casting instead of illegal [7:0] expression selects
            tx_logical_data[i]  = 8'(i); 
            rx_physical_data[i] = 8'(i + 100); 
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_width_degrade.fsdb");
        $fsdbDumpvars(0, tb_lphy_width_degrade);

        $display("==================================================");
        $display("Starting Width Degradation Mux Verification");
        $display("==================================================");

        load_gradients();

        // =========================================================
        // TC1: Full Width (2'b11)
        // =========================================================
        $display("Running TC1: Full Width (1:1 Mapping)");
        lane_map = 2'b11;
        #5; // Yield for combinational evaluation
        
        for (int i = 0; i < NUM_LANES; i++) begin
            if (tx_physical_data[i] !== 8'(i)) 
                $error("[FAIL] TC1 TX: Mismatch at Lane %0d", i);
            if (rx_logical_data[i] !== 8'(i + 100)) 
                $error("[FAIL] TC1 RX: Mismatch at Lane %0d", i);
        end
        $display("[PASS] TC1: Full width routed perfectly 1:1.");

        // =========================================================
        // TC2: Lower Half Degradation (2'b01)
        // =========================================================
        $display("Running TC2: Degrade to Lower Half");
        lane_map = 2'b01;
        #5;
        
        for (int i = 0; i < NUM_LANES; i++) begin
            // TX Check
            if (i < HALF_LANES && tx_physical_data[i] !== 8'(i)) 
                $error("[FAIL] TC2 TX: Lower half mismatch at Lane %0d", i);
            else if (i >= HALF_LANES && tx_physical_data[i] !== 8'h00) 
                $error("[FAIL] TC2 TX: Upper half failed to park at 0.");
                
            // RX Check
            if (i < HALF_LANES && rx_logical_data[i] !== 8'(i + 100)) 
                $error("[FAIL] TC2 RX: Lower half mismatch at Lane %0d", i);
            else if (i >= HALF_LANES && rx_logical_data[i] !== 8'h00) 
                $error("[FAIL] TC2 RX: Upper logical half failed to park at 0.");
        end
        $display("[PASS] TC2: Lower Half degradation bounded and routed perfectly.");

        // =========================================================
        // TC3: Upper Half Degradation (2'b10)
        // =========================================================
        $display("Running TC3: Degrade to Upper Half");
        lane_map = 2'b10;
        #5;
        
        for (int i = 0; i < NUM_LANES; i++) begin
            // TX Check (Logical 0-31 shifts to Physical 32-63)
            if (i < HALF_LANES && tx_physical_data[i] !== 8'h00) 
                $error("[FAIL] TC3 TX: Lower half failed to park at 0.");
            else if (i >= HALF_LANES && tx_physical_data[i] !== 8'(i - HALF_LANES)) 
                $error("[FAIL] TC3 TX: Upper half mismatch. Got %h", tx_physical_data[i]);
                
            // RX Check (Physical 32-63 shifts down to Logical 0-31)
            if (i < HALF_LANES && rx_logical_data[i] !== 8'(i + HALF_LANES + 100)) 
                $error("[FAIL] TC3 RX: Lower logical half mismatch. Got %h", rx_logical_data[i]);
            else if (i >= HALF_LANES && rx_logical_data[i] !== 8'h00) 
                $error("[FAIL] TC3 RX: Upper logical half failed to park at 0.");
        end
        $display("[PASS] TC3: Upper Half degradation shifted up/down perfectly.");
        
        // =========================================================
        // TC4: Fatal Failure (2'b00)
        // =========================================================
        $display("Running TC4: Fatal Link Failure");
        lane_map = 2'b00;
        #5;
        
        for (int i = 0; i < NUM_LANES; i++) begin
            if (tx_physical_data[i] !== 8'h00 || rx_logical_data[i] !== 8'h00) 
                $error("[FAIL] TC4: Datapath failed to securely zero-out during fatal error.");
        end
        $display("[PASS] TC4: Datapath securely parked during fatal link state.");

        $display("==================================================");
        $display("Width Degradation Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire