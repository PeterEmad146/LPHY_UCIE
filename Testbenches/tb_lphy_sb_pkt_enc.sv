`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_sb_pkt_enc;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic clk;
    logic rst_n;

    always #5 clk = ~clk; // 100MHz clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        req_valid;
    wire         req_ready;
    logic [4:0]  opcode;
    logic [2:0]  srcid;
    logic [2:0]  dstid;
    logic        ep;
    logic        cr;
    logic [63:0] payload_in;
    logic [4:0]  tag;
    logic [7:0]  be;
    logic [23:0] addr;
    logic [2:0]  cp_status;
    logic [7:0]  msgcode;
    logic [7:0]  msgsubcode;
    logic [15:0] msginfo;

    logic        tx_ready;
    wire         pkt_valid;
    wire  [63:0] pkt_header;
    wire  [63:0] pkt_data;
    wire         pkt_has_data;
    wire         pkt_data_is_64b;

    // ---------------------------------------------------------
    // DUT Instantiation
    // ---------------------------------------------------------
    lphy_sb_pkt_enc dut (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_valid),
        .req_ready(req_ready),
        .tx_ready(tx_ready),
        .opcode(opcode),
        .srcid(srcid),
        .dstid(dstid),
        .ep(ep),
        .cr(cr),
        .payload_in(payload_in),
        .tag(tag),
        .be(be),
        .addr(addr),
        .cp_status(cp_status),
        .msgcode(msgcode),
        .msgsubcode(msgsubcode),
        .msginfo(msginfo),
        .pkt_valid(pkt_valid),
        .pkt_header(pkt_header),
        .pkt_data(pkt_data),
        .pkt_has_data(pkt_has_data),
        .pkt_data_is_64b(pkt_data_is_64b)
    );

    task clear_inputs();
        req_valid  = 1'b0;
        opcode     = 5'b0;
        srcid      = 3'b0;
        dstid      = 3'b0;
        ep         = 1'b0;
        cr         = 1'b0;
        payload_in = 64'h0;
        tag        = 5'b0;
        be         = 8'b0;
        addr       = 24'b0;
        cp_status  = 3'b0;
        msgcode    = 8'b0;
        msgsubcode = 8'b0;
        msginfo    = 16'b0;
    endtask

    initial begin
        $fsdbDumpfile("tb_lphy_sb_pkt_enc.fsdb");
        $fsdbDumpvars(0, tb_lphy_sb_pkt_enc);

        $display("==================================================");
        $display("Starting UCIe Sideband Encoder Verification");
        $display("==================================================");

        // Initialize
        clk = 0;
        rst_n = 0;
        tx_ready = 1'b1;
        clear_inputs();

        // Release Reset
        #15 rst_n = 1;

        // Align to the NEGATIVE edge to safely drive inputs without race conditions
        @(negedge clk);

        // ---------------------------------------------------------
        // Test Case 1: Message Without Data
        // ---------------------------------------------------------
        $display("Running TC1: Message Without Data");
        req_valid  = 1'b1;
        opcode     = 5'b10010; // Message without data
        srcid      = 3'b010;   // Physical Layer
        dstid      = 3'b010;   // Remote Physical Layer
        msgcode    = 8'h01;    // Active Req
        msgsubcode = 8'h01;
        msginfo    = 16'hABCD;
        
        @(negedge clk); // Allow 1 cycle for DUT to sample (on posedge) and process
        if (!pkt_valid || pkt_has_data || pkt_data_is_64b) 
            $error("[FAIL] TC1: Pipeline valid or data flags incorrect.");
        else 
            $display("[PASS] TC1: Message Without Data encoded correctly.");
            
        clear_inputs(); 
        @(negedge clk); // Flush pipeline

        // ---------------------------------------------------------
        // Test Case 2: 64-bit Memory Write Request
        // ---------------------------------------------------------
        $display("Running TC2: 64-bit Memory Write");
        req_valid  = 1'b1;
        opcode     = 5'b01001; // 64b Mem Write
        srcid      = 3'b001;   // Adapter
        dstid      = 3'b001;   // Remote Adapter
        tag        = 5'h1F;
        be         = 8'hFF;
        addr       = 24'hAABBCC;
        payload_in = 64'h11223344_55667788;

        @(negedge clk); 
        if (!pkt_valid || !pkt_has_data || !pkt_data_is_64b || pkt_data !== 64'h11223344_55667788) 
            $error("[FAIL] TC2: 64-bit Mem Write payload or flags failed.");
        else 
            $display("[PASS] TC2: 64-bit Mem Write encoded correctly.");
            
        clear_inputs();
        @(negedge clk);

        // ---------------------------------------------------------
        // Test Case 3: Backpressure Check
        // ---------------------------------------------------------
        $display("Running TC3: Backpressure Check");
        tx_ready   = 1'b0;     // SERIALIZER IS BUSY!
        req_valid  = 1'b1;
        opcode     = 5'b00000; // 32b Mem Read

        #1; // Wait 1 delta cycle for combinational req_ready to drop
        if (req_ready !== 1'b0) 
            $error("[FAIL] TC3: req_ready did not drop in response to tx_ready.");
        
        @(negedge clk); // Pipeline remains stalled...
        
        tx_ready   = 1'b1;     // SERIALIZER IS FREE!
        
        @(negedge clk); // Wait 1 cycle for pipeline to register
        if (!pkt_valid) 
            $error("[FAIL] TC3: Packet dropped during stall.");
        else 
            $display("[PASS] TC3: Backpressure handshake and pipeline stall successful.");

        @(negedge clk);
        $display("==================================================");
        $display("Verification Complete.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire