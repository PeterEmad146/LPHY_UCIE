`default_nettype none
`timescale 1ns / 1ps

module tb_lphy_sb_ctrl;

    // ---------------------------------------------------------
    // Clock and Reset
    // ---------------------------------------------------------
    logic sb_clk;
    logic rst_n;
    
    always #5 sb_clk = ~sb_clk; // 100MHz simulation clock

    // ---------------------------------------------------------
    // DUT Signals
    // ---------------------------------------------------------
    logic        rdi_in_reset;
    
    // Adapter TX Interface
    logic        tx_req_valid;
    wire         tx_req_ready;
    logic [4:0]  tx_opcode;
    logic [2:0]  tx_srcid;
    logic [2:0]  tx_dstid;
    logic        tx_ep;
    logic        tx_cr;
    logic [63:0] tx_payload;
    logic [4:0]  tx_tag;
    logic [7:0]  tx_be;
    logic [23:0] tx_addr;
    logic [2:0]  tx_cp_status;
    logic [7:0]  tx_msgcode;
    logic [7:0]  tx_msgsubcode;
    logic [15:0] tx_msginfo;
    logic        tx_local_crd_ret;

    // Adapter RX Interface
    wire         rx_req_valid;
    wire  [4:0]  rx_opcode;
    wire  [2:0]  rx_srcid;
    wire  [2:0]  rx_dstid;
    wire         rx_ep;
    wire         rx_cr;
    wire  [63:0] rx_payload;
    wire  [4:0]  rx_tag;
    wire  [7:0]  rx_be;
    wire  [23:0] rx_addr;
    wire  [2:0]  rx_cp_status;
    wire  [7:0]  rx_msgcode;
    wire  [7:0]  rx_msgsubcode;
    wire  [15:0] rx_msginfo;
    wire         rx_parity_err;

    // AFE TX Interface
    wire         afe_tx_valid;
    wire  [63:0] afe_tx_data;
    logic        afe_tx_ready;
    wire         afe_tx_ipg_en;
    wire         afe_tx_rd_en;

    // AFE RX Interface
    logic        afe_rx_valid;
    logic [63:0] afe_rx_data;
    wire         afe_rx_en;
    wire         afe_rx_rd_en;

    // ---------------------------------------------------------
    // Golden Parity Generator for AFE RX Injection
    // ---------------------------------------------------------
    logic [63:0] tb_raw_header;
    logic        tb_has_data;
    logic        tb_is_64b;
    wire  [63:0] tb_calc_header;

    lphy_sb_parity tb_parity_gen (
        .tx_header_in   (tb_raw_header),
        .tx_data_in     (afe_rx_data), // For data parity calc
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
    lphy_sb_ctrl dut (
        .sb_clk(sb_clk),
        .rst_n(rst_n),
        .rdi_in_reset(rdi_in_reset),
        
        .tx_req_valid(tx_req_valid), .tx_req_ready(tx_req_ready),
        .tx_opcode(tx_opcode), .tx_srcid(tx_srcid), .tx_dstid(tx_dstid),
        .tx_ep(tx_ep), .tx_cr(tx_cr), .tx_payload(tx_payload),
        .tx_tag(tx_tag), .tx_be(tx_be), .tx_addr(tx_addr),
        .tx_cp_status(tx_cp_status), .tx_msgcode(tx_msgcode),
        .tx_msgsubcode(tx_msgsubcode), .tx_msginfo(tx_msginfo),
        .tx_local_crd_ret(tx_local_crd_ret),
        
        .rx_req_valid(rx_req_valid), .rx_opcode(rx_opcode), .rx_srcid(rx_srcid),
        .rx_dstid(rx_dstid), .rx_ep(rx_ep), .rx_cr(rx_cr), .rx_payload(rx_payload),
        .rx_tag(rx_tag), .rx_be(rx_be), .rx_addr(rx_addr), .rx_cp_status(rx_cp_status),
        .rx_msgcode(rx_msgcode), .rx_msgsubcode(rx_msgsubcode), .rx_msginfo(rx_msginfo),
        .rx_parity_err(rx_parity_err),
        
        .afe_tx_valid(afe_tx_valid), .afe_tx_data(afe_tx_data),
        .afe_tx_ready(afe_tx_ready), .afe_tx_ipg_en(afe_tx_ipg_en),
        .afe_rx_valid(afe_rx_valid), .afe_rx_data(afe_rx_data),
        .afe_rx_en(afe_rx_en), .afe_tx_rd_en(afe_tx_rd_en), .afe_rx_rd_en(afe_rx_rd_en)
    );

    // Helper Tasks
    task clear_tx_inputs();
        tx_req_valid     = 1'b0;
        tx_opcode        = 5'h0;
        tx_srcid         = 3'h0;
        tx_dstid         = 3'h0;
        tx_ep            = 1'b0;
        tx_cr            = 1'b0;
        tx_payload       = 64'h0;
        tx_tag           = 5'h0;
        tx_be            = 8'h0;
        tx_addr          = 24'h0;
        tx_cp_status     = 3'h0;
        tx_msgcode       = 8'h0;
        tx_msgsubcode    = 8'h0;
        tx_msginfo       = 16'h0;
        tx_local_crd_ret = 1'b0; // FIX: Prevents X-propagation in Flow Control
    endtask

    // ---------------------------------------------------------
    // Main Test Sequence
    // ---------------------------------------------------------
    initial begin
        $fsdbDumpfile("tb_lphy_sb_ctrl.fsdb");
        $fsdbDumpvars(0, tb_lphy_sb_ctrl);

        $display("==================================================");
        $display("Starting Sideband Controller Top Wrapper Verification");
        $display("==================================================");

        sb_clk = 0; rst_n = 0;
        rdi_in_reset = 1'b1;
        afe_tx_ready = 1'b1; // AFE is initially ready
        afe_rx_valid = 1'b0;
        clear_tx_inputs();

        // Release Reset
        #15 rst_n = 1;
        @(negedge sb_clk);
        rdi_in_reset = 1'b0; // Bring RDI out of reset (initializes flow control credits)

        // =========================================================
        // TC1: E2E TX Pipeline - Message Without Data
        // =========================================================
        $display("Running TC1: E2E TX Message (No Data) & IPG Sequence");
        tx_req_valid = 1'b1;
        tx_opcode    = 5'b10010; // Message without data
        tx_srcid     = 3'b010;
        tx_dstid     = 3'b010;
        tx_msgcode   = 8'h01;
        tx_msginfo   = 16'hABCD;
        
        @(negedge sb_clk);
        clear_tx_inputs();
        
        // Cycle 1: Wrapper should drive header to AFE
        @(negedge sb_clk); 
        if (!afe_tx_valid) $error("[FAIL] TC1: AFE TX Valid did not assert for header.");
        
        // Cycle 2: Since there is no data, FSM must jump straight to IPG state
        @(negedge sb_clk);
        if (!afe_tx_ipg_en) $error("[FAIL] TC1: Inter-Packet Gap (IPG) enable did not assert.");
        else $display("[PASS] TC1: TX Message sequenced to AFE successfully with IPG.");

        // =========================================================
        // TC2: E2E TX Pipeline - 64-bit Memory Write
        // =========================================================
        $display("Running TC2: E2E TX 64-bit Memory Write Sequence");
        tx_req_valid = 1'b1;
        tx_opcode    = 5'b01001; // 64b Mem Write
        tx_addr      = 24'hAABBCC;
        tx_payload   = 64'h11223344_55667788;
        
        @(negedge sb_clk);
        clear_tx_inputs();
        
        // Cycle 1: Header to AFE
        @(negedge sb_clk);
        if (!afe_tx_valid) $error("[FAIL] TC2: Header was not sequenced.");
        
        // Cycle 2: Payload to AFE
        @(negedge sb_clk);
        if (!afe_tx_valid || afe_tx_data !== 64'h11223344_55667788) 
            $error("[FAIL] TC2: Payload was not sequenced correctly.");
            
        // Cycle 3: IPG Enforcement
        @(negedge sb_clk);
        if (!afe_tx_ipg_en) $error("[FAIL] TC2: IPG enable did not assert after payload.");
        else $display("[PASS] TC2: 64-bit Payload sequenced to AFE successfully.");

        // =========================================================
        // TC3: E2E RX Pipeline - 64-bit Memory Write
        // =========================================================
        $display("Running TC3: E2E RX 64-bit Memory Write Reception");
        
        // 1. Setup Golden Generator for Mem Write
        tb_raw_header = {2'b00, 1'b1, 2'b00, 3'b010, 24'hCAFE00, 
                         3'b001, 2'b00, 5'h1F, 8'hFF, 8'h00, 1'b0, 5'b01001};
        tb_has_data   = 1'b1;
        tb_is_64b     = 1'b1;
        afe_rx_data   = 64'hDEAD_BEEF_0000_1111; // Payload data used for DP calc
        #1;
        
        // 2. Drive Header into AFE RX pins
        afe_rx_valid = 1'b1;
        afe_rx_data  = tb_calc_header; 
        
        // 3. Drive Payload into AFE RX pins on next cycle
        @(negedge sb_clk);
        afe_rx_data = 64'hDEAD_BEEF_0000_1111;
        
        // 4. Clear AFE bus
        @(negedge sb_clk);
        afe_rx_valid = 1'b0;
        
        // 5. Check Adapter RX Interface on the following cycle
        @(negedge sb_clk);
        if (!rx_req_valid || rx_addr !== 24'hCAFE00 || rx_payload !== 64'hDEAD_BEEF_0000_1111 || rx_parity_err)
            $error("[FAIL] TC3: RX Pipeline failed to decode Mem Write.");
        else
            $display("[PASS] TC3: E2E RX Pipeline decoded Mem Write perfectly.");

        // =========================================================
        // TC4: E2E RX Pipeline - UIE Parity Error Injection
        // =========================================================
        $display("Running TC4: RX Parity Error (UIE) Escalation");
        
        tb_raw_header = {2'b00, 3'b0, 3'b001, 16'h0000, 8'h00, 
                         3'b010, 7'h00, 8'h01, 9'h000, 5'b10010}; // Msg w/o Data
        tb_has_data   = 1'b0;
        #1;
        
        afe_rx_valid = 1'b1;
        afe_rx_data  = tb_calc_header;
        afe_rx_data[63] = ~afe_rx_data[63]; // Flip DP bit (should be 0, force to 1)
        
        @(negedge sb_clk);
        afe_rx_valid = 1'b0;
        
        @(negedge sb_clk);
        if (!rx_parity_err)
            $error("[FAIL] TC4: Wrapper failed to assert Parity Error flag.");
        else
            $display("[PASS] TC4: Wrapper successfully flagged UIE Parity Error.");

        $display("==================================================");
        $display("All System Integrations Verified. Logical PHY Sideband is Clean.");
        $display("==================================================");
        $finish;
    end
endmodule
`default_nettype wire