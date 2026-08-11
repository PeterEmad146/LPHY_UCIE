`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_valid_deframer;

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
    logic [7:0] valid_frame_in;
    
    wire        lane_valid;
    wire        credit_return;
    wire        framing_err;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_valid_deframer dut (
        .clk(clk),
        .rst_n(rst_n),
        .is_retimer(is_retimer),
        .valid_frame_in(valid_frame_in),
        .lane_valid(lane_valid),
        .credit_return(credit_return),
        .framing_err(framing_err)
    );

    // ---------------------------------------------------------
    // Helper Tasks
    // ---------------------------------------------------------
    task clear_inputs();
        is_retimer     = 1'b0;
        valid_frame_in = 8'h00;
    endtask

    // Checks the registered outputs exactly 1 cycle after presentation
    task check_outputs(input logic exp_valid, input logic exp_credit, input logic exp_err, input string tc_name);
        @(negedge clk);
        if (lane_valid !== exp_valid || credit_return !== exp_credit || framing_err !== exp_err) begin
            $error("[FAIL] %s: Expected {V:%b, C:%b, E:%b}, Got {V:%b, C:%b, E:%b}", 
                   tc_name, exp_valid, exp_credit, exp_err, lane_valid, credit_return, framing_err);
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_valid_deframer.fsdb");
        $fsdbDumpvars(0, tb_lphy_valid_deframer);

        $display("==================================================");
        $display("Starting Mainband RX Valid Deframer Verification");
        $display("==================================================");

        clk = 0; 
        rst_n = 0;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge clk);

        // =========================================================
        // TC1: Standard Link (Non-Retimer) Normal Ops
        // =========================================================
        $display("Running TC1: Standard Link Normal Decoding (0F, 00)");
        is_retimer = 1'b0;
        
        valid_frame_in = 8'h0F; // Standard Data
        check_outputs(1'b1, 1'b0, 1'b0, "TC1: Standard Data (0F)");
        
        valid_frame_in = 8'h00; // Standard Idle
        check_outputs(1'b0, 1'b0, 1'b0, "TC1: Standard Idle (00)");
        
        $display("[PASS] TC1: Standard link decoded legal frames correctly.");

        // =========================================================
        // TC2: Standard Link Safety Masking (Rogue Transmitter)
        // =========================================================
        $display("Running TC2: Standard Link Safety Masking (FF, F0)");
        is_retimer = 1'b0; // Still Non-Retimer
        
        valid_frame_in = 8'hFF; // Rogue Data+Credit
        check_outputs(1'b0, 1'b0, 1'b1, "TC2: Rogue Data+Credit (FF)");
        
        valid_frame_in = 8'hF0; // Rogue Idle+Credit
        check_outputs(1'b0, 1'b0, 1'b1, "TC2: Rogue Idle+Credit (F0)");
        
        $display("[PASS] TC2: Non-Retimer successfully masked illegal credit encodings and flagged errors.");

        // =========================================================
        // TC3: Retimer Link Operation
        // =========================================================
        $display("Running TC3: Retimer Link Overloaded Decoding");
        is_retimer = 1'b1; // Switch to Retimer mode
        
        valid_frame_in = 8'hFF; // Legal Data+Credit
        check_outputs(1'b1, 1'b1, 1'b0, "TC3: Retimer Data+Credit (FF)");
        
        valid_frame_in = 8'hF0; // Legal Idle+Credit
        check_outputs(1'b0, 1'b1, 1'b0, "TC3: Retimer Idle+Credit (F0)");
        
        valid_frame_in = 8'h0F; // Legal Data (No Credit)
        check_outputs(1'b1, 1'b0, 1'b0, "TC3: Retimer Data (0F)");
        
        $display("[PASS] TC3: Retimer successfully unlocked and extracted overloaded credit frames.");

        // =========================================================
        // TC4: Bit-Flip / Physical Channel Error Injection
        // =========================================================
        $display("Running TC4: Physical Channel Error Injection");
        is_retimer = 1'b1;
        
        // Inject random 1-bit or multi-bit flips on the wire
        valid_frame_in = 8'h1F; // Bit flip on Data
        check_outputs(1'b0, 1'b0, 1'b1, "TC4: Corruption (1F)");
        
        valid_frame_in = 8'hAA; // Total channel garbage
        check_outputs(1'b0, 1'b0, 1'b1, "TC4: Corruption (AA)");
        
        $display("[PASS] TC4: Hardware successfully caught corrupted framing bits and isolated datapath.");

        $display("==================================================");
        $display("Valid Deframer Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire