`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_scrambler;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        enable;
    logic        load_seed;
    logic [22:0] seed_in;
    logic [7:0]  data_in;
    wire  [7:0]  data_out;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_scrambler dut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .load_seed(load_seed),
        .seed_in(seed_in),
        .data_in(data_in),
        .data_out(data_out)
    );

    // ---------------------------------------------------------
    // Behavioral Reference Model (Predictor)
    // ---------------------------------------------------------
    // This function mimics the exact mathematical requirement of the 
    // UCIe LFSR to predict what the hardware *should* output.
    function automatic logic [7:0] predict_key(ref logic [22:0] current_state);
        logic [7:0] expected_key;
        logic feedback;
        
        for (int i = 0; i < 8; i++) begin
            expected_key[i] = current_state[22];
            feedback = current_state[22] ^ current_state[20] ^ current_state[15] ^ 
                       current_state[7]  ^ current_state[4]  ^ current_state[1];
            current_state = {current_state[21:0], feedback};
        end
        return expected_key;
    endfunction

    // ---------------------------------------------------------
    // Helper State
    // ---------------------------------------------------------
    logic [22:0] tb_lfsr_state;
    logic [7:0]  expected_key;

    task clear_inputs();
        enable    = 1'b0;
        load_seed = 1'b0;
        seed_in   = 23'h0;
        data_in   = 8'h0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_scrambler.fsdb");
        $fsdbDumpvars(0, tb_lphy_scrambler);

        $display("==================================================");
        $display("Starting Mainband TX Scrambler Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Seed Initialization (Lane 1 Seed)
        // =========================================================
        $display("Running TC1: Seed Initialization (Lane 1)");
        
        load_seed = 1'b1;
        seed_in   = 23'h0607BB; // UCIe Table 20 (Lane 1 Seed)
        tb_lfsr_state = 23'h0607BB; // Sync the testbench predictor
        
        @(negedge clk);
        clear_inputs(); // Drop load_seed
        
        // Peek into the DUT via hierarchical path to ensure it took the seed
        if (dut.lfsr_reg !== 23'h0607BB)
            $error("[FAIL] TC1: LFSR failed to load Lane 1 seed.");
        else
            $display("[PASS] TC1: Lane-specific seed successfully loaded.");

        // =========================================================
        // TC2: Scrambling Bypass (MBINIT.REPAIRCLK Rule)
        // =========================================================
        $display("Running TC2: Datapath Bypass & LFSR Pause");
        
        enable  = 1'b0;
        data_in = 8'hA5; // 10100101
        
        @(negedge clk);
        
        // Data should pass through un-XORed, and LFSR should NOT advance
        if (data_out !== 8'hA5)
            $error("[FAIL] TC2: Datapath corrupted during bypass mode.");
        else if (dut.lfsr_reg !== 23'h0607BB)
            $error("[FAIL] TC2: LFSR illegally advanced while enable was low.");
        else
            $display("[PASS] TC2: Bypass Mode correctly paused LFSR and preserved datapath.");

        // =========================================================
        // TC3: Continuous Scrambling & PRBS Generation
        // =========================================================
        $display("Running TC3: Continuous Fibonacci Scrambling (10 Cycles)");
        
        enable = 1'b1;
        
        for (int cycle = 0; cycle < 10; cycle++) begin
            data_in = 8'hFF; // Continuous stream of 1s
            
            #1; // FIX: Yield 1 delta cycle so combinational data_out resolves BEFORE the clock edge!
            
            // Generate the expected key using our software reference model
            // (Note: This also advances the tb_lfsr_state for the next loop iteration)
            expected_key = predict_key(tb_lfsr_state);
            
            // FIX: Check immediately! Do not wait for the clock edge.
            if (data_out !== (8'hFF ^ expected_key)) begin
                $error("[FAIL] TC3: Scramble mismatch at cycle %0d. Expected: %0h, Got: %0h", 
                       cycle, (8'hFF ^ expected_key), data_out);
                $finish;
            end
            
            // NOW wait for the clock edge to advance the hardware LFSR for the next cycle
            @(negedge clk);
        end
        
        $display("[PASS] TC3: Fibonacci Scrambling perfectly matched mathematical reference model.");

        $display("==================================================");
        $display("Scrambler Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire