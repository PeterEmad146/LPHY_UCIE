`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_sb_pkt_dec;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        pkt_valid;
    logic [63:0] pkt_header;
    logic [63:0] pkt_data;
    
    wire         req_valid;
    wire  [4:0]  opcode;
    wire  [2:0]  srcid;
    wire  [2:0]  dstid;
    wire         ep;
    wire         cr;
    wire  [63:0] payload_out;
    wire  [4:0]  tag;
    wire  [7:0]  be;
    wire  [23:0] addr;
    wire  [2:0]  cp_status;
    wire  [7:0]  msgcode;
    wire  [7:0]  msgsubcode;
    wire  [15:0] msginfo;
    wire         parity_err;

    // ---------------------------------------------------------
    // Testbench Golden Parity Generator
    // ---------------------------------------------------------
    logic [63:0] tb_raw_header;
    logic        tb_has_data;
    logic        tb_is_64b;
    wire  [63:0] tb_calc_header;

    lphy_sb_parity tb_parity_gen (
        .tx_header_in   (tb_raw_header),
        .tx_data_in     (pkt_data),
        .tx_has_data    (tb_has_data),
        .tx_data_is_64b (tb_is_64b),
        .tx_header_out  (tb_calc_header),
        
        .rx_header_in   (64'h0),
        .rx_data_in     (64'h0),
        .rx_has_data    (1'b0),
        .rx_data_is_64b (1'b0),
        .rx_cp_err      (),
        .rx_dp_err      ()
    );

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_sb_pkt_dec dut (
        .clk(clk),
        .rst_n(rst_n),
        .pkt_valid(pkt_valid),
        .pkt_header(pkt_header),
        .pkt_data(pkt_data),
        .req_valid(req_valid),
        .opcode(opcode),
        .srcid(srcid),
        .dstid(dstid),
        .ep(ep),
        .cr(cr),
        .payload_out(payload_out),
        .tag(tag),
        .be(be),
        .addr(addr),
        .cp_status(cp_status),
        .msgcode(msgcode),
        .msgsubcode(msgsubcode),
        .msginfo(msginfo),
        .parity_err(parity_err)
    );

    // Helper Variables for Assembly
    logic [31:0] phase0;
    logic [31:0] phase1;

    task clear_inputs();
        pkt_valid = 1'b0;
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_sb_pkt_dec.fsdb");
        $fsdbDumpvars(0, tb_lphy_sb_pkt_dec);

        $display("==================================================");
        $display("Starting Sideband Packet Decoder Verification");
        $display("==================================================");

        clk = 0;
        rst_n = 0;
        clear_inputs();

        #15 rst_n = 1;
        @(negedge clk);

        // ---------------------------------------------------------
        // TC1: Message with 64b Data (e.g., Parameter Exchange)
        // ---------------------------------------------------------
        $display("Running TC1: Message with 64b Data");
        
        phase0 = {3'b010, 7'h00, 8'hAA, 9'h000, 5'b11011}; 
        phase1 = {2'b00, 3'b0, 3'b001, 16'hBEEF, 8'hCC};   
        tb_raw_header = {phase1, phase0};
        
        pkt_data    = 64'h11223344_55667788;
        tb_has_data = 1'b1;
        tb_is_64b   = 1'b1;
        
        #1; 
        
        pkt_valid  = 1'b1;
        pkt_header = tb_calc_header; 
        
        @(negedge clk); // Wait 1 cycle for DUT to sample and update outputs
        
        // CHECK FIRST
        if (!req_valid || opcode !== 5'b11011 || msgcode !== 8'hAA || msginfo !== 16'hBEEF || addr !== 24'h0 || parity_err)
            $error("[FAIL] TC1: Message decoding failed or ghost data present on addr.");
        else
            $display("[PASS] TC1: Message decoded perfectly. Ghost data zeroed out.");

        // THEN CLEAR
        clear_inputs();
        @(negedge clk); // Flush pipeline

        // ---------------------------------------------------------
        // TC2: 64-bit Memory Write Request
        // ---------------------------------------------------------
        $display("Running TC2: 64-bit Memory Write Request");
        
        phase0 = {3'b001, 2'b00, 5'h1F, 8'hFF, 8'h00, 1'b1, 5'b01001}; 
        phase1 = {2'b00, 1'b1, 2'b00, 3'b010, 24'hAABBCC};             
        tb_raw_header = {phase1, phase0};
        
        pkt_data    = 64'hDEAD_BEEF_CAFE_BABE;
        tb_has_data = 1'b1;
        tb_is_64b   = 1'b1;
        
        #1; 
        
        pkt_valid  = 1'b1;
        pkt_header = tb_calc_header; 
        
        @(negedge clk); // Wait 1 cycle
        
        // CHECK FIRST
        if (!req_valid || tag !== 5'h1F || addr !== 24'hAABBCC || ep !== 1'b1 || payload_out !== 64'hDEAD_BEEF_CAFE_BABE || parity_err)
            $error("[FAIL] TC2: Mem Write decoding failed.");
        else
            $display("[PASS] TC2: 64-bit Memory Write decoded correctly.");
            
        // THEN CLEAR
        clear_inputs();
        @(negedge clk); // Flush pipeline

        // ---------------------------------------------------------
        // TC3: Control Parity Error Injection (UIE Escalation)
        // ---------------------------------------------------------
        $display("Running TC3: Control Parity Error Injection");
        
        tb_raw_header = {phase1, phase0}; 
        #1;
        
        pkt_valid  = 1'b1;
        pkt_header = tb_calc_header;
        pkt_header[62] = ~pkt_header[62]; // INJECT CORRUPTION
        
        @(negedge clk); // Wait 1 cycle
        
        // CHECK FIRST
        if (!parity_err)
            $error("[FAIL] TC3: Decoder failed to detect parity corruption.");
        else
            $display("[PASS] TC3: Control Parity UIE successfully detected.");

        // THEN CLEAR
        clear_inputs();
        @(negedge clk); // Flush pipeline

        $display("==================================================");
        $display("Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire