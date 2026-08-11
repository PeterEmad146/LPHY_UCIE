`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_valid_framer;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Link Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic       is_retimer;
    logic       train_mode;
    logic       lane_valid;
    logic       credit_return;
    wire  [7:0] valid_frame_out;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_valid_framer dut (
        .clk(clk),
        .rst_n(rst_n),
        .is_retimer(is_retimer),
        .train_mode(train_mode),
        .lane_valid(lane_valid),
        .credit_return(credit_return),
        .valid_frame_out(valid_frame_out)
    );

    // ---------------------------------------------------------
    // Helper State
    // ---------------------------------------------------------
    task clear_inputs();
        is_retimer    = 1'b0;
        train_mode    = 1'b0;
        lane_valid    = 1'b0;
        credit_return = 1'b0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_valid_framer.fsdb");
        $fsdbDumpvars(0, tb_lphy_valid_framer);

        $display("==================================================");
        $display("Starting Mainband Valid Framer Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Training Mode Override (MBINIT.REPAIRVAL)
        // =========================================================
        $display("Running TC1: Training Mode Override");
        
        train_mode    = 1'b1;
        lane_valid    = 1'b0; // Datapath is idle
        credit_return = 1'b1; // Simulate rogue credit return
        
        @(negedge clk); // Allow register update
        if (valid_frame_out !== 8'h0F)
            $error("[FAIL] TC1: Training override failed. Expected 0F, Got %0h", valid_frame_out);
        else
            $display("[PASS] TC1: Training mode successfully forced VALTRAIN pattern (0F).");

        clear_inputs();
        @(negedge clk);

        // =========================================================
        // TC2: Standard Link (Non-Retimer) Safety Masking
        // =========================================================
        $display("Running TC2: Standard Link Safety Masking");
        
        is_retimer = 1'b0; // Standard D2D Link
        
        // Scenario A: Data transfer + Rogue Credit
        lane_valid    = 1'b1;
        credit_return = 1'b1;
        
        @(negedge clk);
        if (valid_frame_out !== 8'h0F) 
            $error("[FAIL] TC2A: Masking failed on Data. Expected 0F, Got %0h", valid_frame_out);
            
        // Scenario B: Idle + Rogue Credit
        lane_valid    = 1'b0;
        credit_return = 1'b1;
        
        @(negedge clk);
        if (valid_frame_out !== 8'h00) 
            $error("[FAIL] TC2B: Masking failed on Idle. Expected 00, Got %0h", valid_frame_out);
        else 
            $display("[PASS] TC2: Non-Retimer link successfully masked out credit signals.");

        clear_inputs();
        @(negedge clk);

        // =========================================================
        // TC3: Retimer Link Flow Control (All 4 Encodings)
        // =========================================================
        $display("Running TC3: Retimer Link Credit Overloading");
        
        is_retimer = 1'b1;
        
        // 1. Idle (No Data, No Credit)
        lane_valid    = 1'b0;
        credit_return = 1'b0;
        @(negedge clk);
        if (valid_frame_out !== 8'h00) $error("[FAIL] TC3.1: Expected 00, Got %0h", valid_frame_out);
        
        // 2. Data Only (Data, No Credit)
        lane_valid    = 1'b1;
        credit_return = 1'b0;
        @(negedge clk);
        if (valid_frame_out !== 8'h0F) $error("[FAIL] TC3.2: Expected 0F, Got %0h", valid_frame_out);
        
        // 3. Data + Credit
        lane_valid    = 1'b1;
        credit_return = 1'b1;
        @(negedge clk);
        if (valid_frame_out !== 8'hFF) $error("[FAIL] TC3.3: Expected FF, Got %0h", valid_frame_out);
        
        // 4. Credit Only (No Data, Credit)
        lane_valid    = 1'b0;
        credit_return = 1'b1;
        @(negedge clk);
        if (valid_frame_out !== 8'hF0) $error("[FAIL] TC3.4: Expected F0, Got %0h", valid_frame_out);
        else $display("[PASS] TC3: Retimer perfectly overloaded the 8-UI framing envelope.");

        $display("==================================================");
        $display("Valid Framer Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire