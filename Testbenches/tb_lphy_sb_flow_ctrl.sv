`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_sb_flow_ctrl;

    // ---------------------------------------------------------
    // Clock and Reset Generation
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        rdi_in_reset;
    logic        req_valid;
    logic        is_reg_req;
    logic        is_reg_cpl;
    logic        is_msg;
    wire         tx_allowed;
    logic        local_crd_ret;
    logic [2:0]  remote_crd_ret_val;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_sb_flow_ctrl #(
        .LOCAL_CREDITS_INIT(32),
        .REMOTE_CREDITS_INIT(4),
        .MAX_REMOTE_CREDITS(32)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rdi_in_reset(rdi_in_reset),
        .req_valid(req_valid),
        .is_reg_req(is_reg_req),
        .is_reg_cpl(is_reg_cpl),
        .is_msg(is_msg),
        .tx_allowed(tx_allowed),
        .local_crd_ret(local_crd_ret),
        .remote_crd_ret_val(remote_crd_ret_val)
    );

    // ---------------------------------------------------------
    // Helper Task: Clear Inputs
    // ---------------------------------------------------------
    task clear_inputs();
        req_valid          = 1'b0;
        is_reg_req         = 1'b0;
        is_reg_cpl         = 1'b0;
        is_msg             = 1'b0;
        local_crd_ret      = 1'b0;
        remote_crd_ret_val = 3'b000;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_sb_flow_ctrl.fsdb");
        $fsdbDumpvars(0, tb_lphy_sb_flow_ctrl);

        $display("==================================================");
        $display("Starting Sideband Flow Controller Verification");
        $display("==================================================");

        // Initialize
        clk          = 0;
        rst_n        = 0;
        rdi_in_reset = 1'b1;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;

        // ---------------------------------------------------------
        // TC1: Initialization Check
        // ---------------------------------------------------------
        @(negedge clk);
        rdi_in_reset = 1'b0; // Bring RDI to Active
        
        @(negedge clk);
        // At this point, local = 32, remote = 4. 
        if (dut.local_crd_count !== 6'd32 || dut.remote_crd_count !== 6'd4)
            $error("[FAIL] TC1: Counters did not initialize correctly.");
        else
            $display("[PASS] TC1: Initialization successful (Local: 32, Remote: 4).");

        // ---------------------------------------------------------
        // TC2: Message Consumption (Local Only)
        // ---------------------------------------------------------
        req_valid = 1'b1;
        is_msg    = 1'b1;
        
        @(negedge clk);
        clear_inputs(); // Clear stimulus
        
        @(negedge clk); // Allow register update
        if (dut.local_crd_count !== 6'd31 || dut.remote_crd_count !== 6'd4)
            $error("[FAIL] TC2: Message consumed incorrect credits.");
        else
            $display("[PASS] TC2: Message correctly consumed only 1 local credit.");

        // ---------------------------------------------------------
        // TC3: Register Access Request (Local + Remote)
        // ---------------------------------------------------------
        req_valid  = 1'b1;
        is_reg_req = 1'b1;
        
        @(negedge clk);
        clear_inputs();
        
        @(negedge clk);
        if (dut.local_crd_count !== 6'd30 || dut.remote_crd_count !== 6'd3)
            $error("[FAIL] TC3: Register Request consumed incorrect credits.");
        else
            $display("[PASS] TC3: Reg Req correctly consumed 1 local and 1 remote.");

        // ---------------------------------------------------------
        // TC4: Register Access Completion (Bypass Check)
        // ---------------------------------------------------------
        // Completions should unconditionally allow TX and consume NO credits.
        req_valid  = 1'b1;
        is_reg_cpl = 1'b1;
        
        #1; // FIX: Yield 1 time unit to allow combinational logic to propagate
        
        if (tx_allowed !== 1'b1)
            $error("[FAIL] TC4: Completion was not authorized immediately.");
            
        @(negedge clk);
        clear_inputs();
        
        @(negedge clk);
        if (dut.local_crd_count !== 6'd30 || dut.remote_crd_count !== 6'd3)
            $error("[FAIL] TC4: Completion incorrectly consumed a credit.");
        else
            $display("[PASS] TC4: Reg Completion bypassed flow control and consumed 0 credits.");

        // ---------------------------------------------------------
        // TC5: Simultaneous Return and Consume
        // ---------------------------------------------------------
        // We will send a Reg Request (consumes 1L, 1R) while simultaneously
        // returning 1 Local and 3 Remote credits.
        req_valid          = 1'b1;
        is_reg_req         = 1'b1;
        local_crd_ret      = 1'b1;
        remote_crd_ret_val = 3'd3; 
        
        @(negedge clk);
        clear_inputs();
        
        @(negedge clk);
        // Local: 30 - 1 + 1 = 30
        // Remote: 3 - 1 + 3 = 5
        if (dut.local_crd_count !== 6'd30 || dut.remote_crd_count !== 6'd5)
            $error("[FAIL] TC5: Parallel arithmetic failed (Local: %0d, Remote: %0d).", dut.local_crd_count, dut.remote_crd_count);
        else
            $display("[PASS] TC5: Simultaneous Return and Consume calculated correctly.");

        // ---------------------------------------------------------
        // TC6: Saturation/Overflow Protection
        // ---------------------------------------------------------
        // Force a massive remote credit return to hit the ceiling.
        remote_crd_ret_val = 3'd4; // Loop this for a few cycles
        
        repeat(10) @(negedge clk);
        clear_inputs();
        
        @(negedge clk);
        if (dut.remote_crd_count > 6'd32)
            $error("[FAIL] TC6: Remote credits overflowed past MAX_REMOTE_CREDITS limit.");
        else if (dut.remote_crd_count !== 6'd32)
            $error("[FAIL] TC6: Remote credits failed to saturate at MAX_REMOTE_CREDITS.");
        else
            $display("[PASS] TC6: Counter saturation successfully prevented overflow.");

        // ---------------------------------------------------------
        // TC7: RDI Reset Re-initialization
        // ---------------------------------------------------------
        rdi_in_reset = 1'b1;
        @(negedge clk);
        rdi_in_reset = 1'b0;
        @(negedge clk);
        
        if (dut.remote_crd_count !== 6'd4 || dut.local_crd_count !== 6'd32)
            $error("[FAIL] TC7: RDI Reset failed to re-initialize counters.");
        else
            $display("[PASS] TC7: RDI Reset successfully re-initialized credits.");

        $display("==================================================");
        $display("Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire