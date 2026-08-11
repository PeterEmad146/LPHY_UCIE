`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_descrambler;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // Shared Control Signals
    // ---------------------------------------------------------
    logic        enable;
    logic        load_seed;
    logic [22:0] seed_in;

    // ---------------------------------------------------------
    // Datapath Signals
    // ---------------------------------------------------------
    logic [7:0]  tx_plaintext_in;
    wire  [7:0]  scrambled_wire;
    wire  [7:0]  rx_plaintext_out;

    // ---------------------------------------------------------
    // Golden TX Instantiation
    // ---------------------------------------------------------
    lphy_scrambler tx_scrambler (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .load_seed(load_seed),
        .seed_in(seed_in),
        .data_in(tx_plaintext_in),
        .data_out(scrambled_wire)
    );

    // ---------------------------------------------------------
    // DUT RX Instantiation
    // ---------------------------------------------------------
    lphy_descrambler rx_descrambler (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .load_seed(load_seed),
        .seed_in(seed_in),
        .data_in(scrambled_wire),
        .data_out(rx_plaintext_out)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        enable          = 1'b0;
        load_seed       = 1'b0;
        seed_in         = 23'h0;
        tx_plaintext_in = 8'h0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_descrambler.fsdb");
        $fsdbDumpvars(0, tb_lphy_descrambler);

        $display("==================================================");
        $display("Starting E2E Scrambler/Descrambler Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Seed Synchronization
        // =========================================================
        $display("Running TC1: Seed Synchronization (Lane 2)");
        
        load_seed = 1'b1;
        seed_in   = 23'h1EC760; // UCIe Table 20 (Lane 2 Seed)
        
        @(negedge clk);
        clear_inputs();
        
        // Peek into both modules to ensure perfect state alignment
        if (tx_scrambler.lfsr_reg !== 23'h1EC760 || rx_descrambler.lfsr_reg !== 23'h1EC760)
            $error("[FAIL] TC1: LFSRs failed to synchronize seeds.");
        else
            $display("[PASS] TC1: TX and RX LFSRs synchronized perfectly.");

        // =========================================================
        // TC2: Bypass Mode Check
        // =========================================================
        $display("Running TC2: Bypass Mode Passthrough");
        
        enable = 1'b0; // Scrambling paused
        tx_plaintext_in = 8'hAA; 
        
        #1; // Yield for combinational path
        if (scrambled_wire !== 8'hAA || rx_plaintext_out !== 8'hAA)
            $error("[FAIL] TC2: Bypass mode corrupted the datapath.");
        else
            $display("[PASS] TC2: Bypass Mode successfully passed data unmodified.");
            
        @(negedge clk);

        // =========================================================
        // TC3: Continuous E2E Recovery
        // =========================================================
        $display("Running TC3: Continuous E2E Cipher Recovery (20 Cycles)");
        
        enable = 1'b1;
        
        for (int cycle = 0; cycle < 20; cycle++) begin
            // Feed a recognizable incrementing payload
            tx_plaintext_in = cycle[7:0]; 
            
            #1; // Wait for TX XOR -> Wire -> RX XOR to settle
            
            // 1. Prove the data on the wire is actually scrambled (Ciphertext != Plaintext)
            // (Exception: Cycle 0 might mathematically evaluate to the same value depending on the seed bit)
            if (cycle > 0 && scrambled_wire === tx_plaintext_in) begin
                $error("[FAIL] TC3: Wire data is not being scrambled!");
            end
            
            // 2. Prove the RX successfully recovered it
            if (rx_plaintext_out !== tx_plaintext_in) begin
                $error("[FAIL] TC3: Descramble mismatch at cycle %0d. Sent: %0h, Cipher: %0h, Recv: %0h", 
                       cycle, tx_plaintext_in, scrambled_wire, rx_plaintext_out);
                $finish;
            end
            
            @(negedge clk); // Advance both LFSRs
        end
        
        $display("[PASS] TC3: RX successfully recovered 100%% of the TX plaintext.");

        $display("==================================================");
        $display("Descrambler Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire