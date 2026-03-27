//  12 comprehensive test cases covering:
//    1) Single BID add
//    2) Two BIDs (BBO update to higher price)
//    3) Two BIDs + one ASK (both sides populated)
//    4) ASK improves (lower price)
//    5) 4-order Verilog TB replay (from Python verify_against_verilog_tb)
//    6) Cancel non-BBO order (O(1) path)
//    7) Cancel BBO bid → rebuild
//    8) Cancel BBO ask → rebuild
//    9) Cancel all orders → empty book
//   10) Fill to capacity (64 orders)
//   11) Same price, multiple orders (level aggregation)
//   12) Interleaved add/cancel storm (20 operations)

`timescale 1ns / 1ps

module tb_hybrid_adaptive_order_book;

    // ════════════════════════════════════════════════════════════════════
    //  Parameters
    // ════════════════════════════════════════════════════════════════════
    parameter ORDER_DEPTH    = 64;
    parameter LEVEL_DEPTH    = 64;
    parameter PRICE_WIDTH    = 32;
    parameter QTY_WIDTH      = 16;
    parameter ORDER_ID_WIDTH = 8;
    parameter DEFAULT_ASK    = 32'h000003E7;  // 999

    parameter CLK_PERIOD     = 10;  // 10 ns → 100 MHz

    // ════════════════════════════════════════════════════════════════════
    //  Signals
    // ════════════════════════════════════════════════════════════════════
    reg                        clk;
    reg                        rst_n;

    reg                        cmd_valid;
    reg                        cmd_is_cancel;
    reg  [PRICE_WIDTH-1:0]     cmd_price;
    reg  [QTY_WIDTH-1:0]       cmd_qty;
    reg                        cmd_side;
    reg  [ORDER_ID_WIDTH-1:0]  cmd_cancel_id;

    wire                       resp_valid;
    wire                       resp_ready;
    wire [PRICE_WIDTH-1:0]     best_bid_price;
    wire [QTY_WIDTH-1:0]       best_bid_qty;
    wire                       best_bid_present;
    wire [PRICE_WIDTH-1:0]     best_ask_price;
    wire [QTY_WIDTH-1:0]       best_ask_qty;
    wire                       best_ask_present;
    wire [ORDER_ID_WIDTH-1:0]  assigned_order_id;
    wire [31:0]                o1_hit_count;
    wire [31:0]                logn_hit_count;

    // Test tracking
    integer test_num;
    integer pass_count;
    integer fail_count;
    integer total_tests;

    // ════════════════════════════════════════════════════════════════════
    //  DUT Instantiation
    // ════════════════════════════════════════════════════════════════════
    hybrid_adaptive_order_book #(
        .ORDER_DEPTH    (ORDER_DEPTH),
        .LEVEL_DEPTH    (LEVEL_DEPTH),
        .PRICE_WIDTH    (PRICE_WIDTH),
        .QTY_WIDTH      (QTY_WIDTH),
        .ORDER_ID_WIDTH (ORDER_ID_WIDTH),
        .DEFAULT_ASK    (DEFAULT_ASK)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .cmd_valid        (cmd_valid),
        .cmd_is_cancel    (cmd_is_cancel),
        .cmd_price        (cmd_price),
        .cmd_qty          (cmd_qty),
        .cmd_side         (cmd_side),
        .cmd_cancel_id    (cmd_cancel_id),
        .resp_valid       (resp_valid),
        .resp_ready       (resp_ready),
        .best_bid_price   (best_bid_price),
        .best_bid_qty     (best_bid_qty),
        .best_bid_present (best_bid_present),
        .best_ask_price   (best_ask_price),
        .best_ask_qty     (best_ask_qty),
        .best_ask_present (best_ask_present),
        .assigned_order_id(assigned_order_id),
        .o1_hit_count     (o1_hit_count),
        .logn_hit_count   (logn_hit_count)
    );

    // ════════════════════════════════════════════════════════════════════
    //  Clock Generation
    // ════════════════════════════════════════════════════════════════════
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ════════════════════════════════════════════════════════════════════
    //  VCD Dump for Waveform
    // ════════════════════════════════════════════════════════════════════
    initial begin
        $dumpfile("tb_hybrid_ob.vcd");
        $dumpvars(0, tb_hybrid_adaptive_order_book);
    end

    // ════════════════════════════════════════════════════════════════════
    //  Helper Tasks
    // ════════════════════════════════════════════════════════════════════

    // ── Reset the DUT ─────────────────────────────────────────────────
    task do_reset;
    begin
        rst_n       <= 0;
        cmd_valid   <= 0;
        cmd_is_cancel <= 0;
        cmd_price   <= 0;
        cmd_qty     <= 0;
        cmd_side    <= 0;
        cmd_cancel_id <= 0;
        repeat(5) @(posedge clk);
        rst_n <= 1;
        repeat(2) @(posedge clk);
    end
    endtask

    // ── Send ADD command and wait for response ────────────────────────
    task add_order;
        input [PRICE_WIDTH-1:0]  price;
        input [QTY_WIDTH-1:0]    qty;
        input                    side;
    begin
        // Wait until ready
        wait(resp_ready);
        @(posedge clk);

        cmd_valid     <= 1;
        cmd_is_cancel <= 0;
        cmd_price     <= price;
        cmd_qty       <= qty;
        cmd_side      <= side;
        cmd_cancel_id <= 0;
        @(posedge clk);
        cmd_valid     <= 0;

        // Wait for response
        wait(resp_valid);
        @(posedge clk);
    end
    endtask

    // ── Send CANCEL command and wait for response ─────────────────────
    task cancel_order;
        input [ORDER_ID_WIDTH-1:0] order_id;
    begin
        wait(resp_ready);
        @(posedge clk);

        cmd_valid     <= 1;
        cmd_is_cancel <= 1;
        cmd_price     <= 0;
        cmd_qty       <= 0;
        cmd_side      <= 0;
        cmd_cancel_id <= order_id;
        @(posedge clk);
        cmd_valid     <= 0;

        // Wait for response
        wait(resp_valid);
        @(posedge clk);
    end
    endtask

    // ── Check BBO against expected values ─────────────────────────────
    task check_bbo;
        input [PRICE_WIDTH-1:0]  exp_bid_price;
        input [QTY_WIDTH-1:0]    exp_bid_qty;
        input                    exp_bid_present;
        input [PRICE_WIDTH-1:0]  exp_ask_price;
        input [QTY_WIDTH-1:0]    exp_ask_qty;
        input                    exp_ask_present;
        input [255:0]            test_desc;
    begin
        if (best_bid_price   == exp_bid_price   &&
            best_bid_qty     == exp_bid_qty     &&
            best_bid_present == exp_bid_present &&
            best_ask_price   == exp_ask_price   &&
            best_ask_qty     == exp_ask_qty     &&
            best_ask_present == exp_ask_present) begin
            $display("  [PASS] Test %0d: %0s", test_num, test_desc);
            $display("         BBO: bid=%0d x%0d (%s)  ask=%0d x%0d (%s)",
                     best_bid_price, best_bid_qty,
                     best_bid_present ? "Y" : "N",
                     best_ask_price, best_ask_qty,
                     best_ask_present ? "Y" : "N");
            pass_count = pass_count + 1;
        end
        else begin
            $display("  [FAIL] Test %0d: %0s", test_num, test_desc);
            $display("         Expected: bid=%0d x%0d (%s)  ask=%0d x%0d (%s)",
                     exp_bid_price, exp_bid_qty,
                     exp_bid_present ? "Y" : "N",
                     exp_ask_price, exp_ask_qty,
                     exp_ask_present ? "Y" : "N");
            $display("         Got:      bid=%0d x%0d (%s)  ask=%0d x%0d (%s)",
                     best_bid_price, best_bid_qty,
                     best_bid_present ? "Y" : "N",
                     best_ask_price, best_ask_qty,
                     best_ask_present ? "Y" : "N");
            fail_count = fail_count + 1;
        end
        test_num = test_num + 1;
    end
    endtask

    // ════════════════════════════════════════════════════════════════════
    //  Main Test Sequence
    // ════════════════════════════════════════════════════════════════════
    initial begin
        $display("");
        $display("======================================================================");
        $display("  HYBRID ADAPTIVE ORDER BOOK — COMPREHENSIVE TESTBENCH");
        $display("  Target: Zedboard Zynq-7000 (100 MHz)");
        $display("======================================================================");

        test_num   = 1;
        pass_count = 0;
        fail_count = 0;
        total_tests = 0;

        // ══════════════════════════════════════════════════════════════
        //  TEST 1: Single BID add
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 1: Single BID add ---");
        do_reset;
        add_order(32'd100, 16'd5, 1'b0);  // BID 100 x 5
        check_bbo(
            32'd100, 16'd5, 1'b1,          // bid: price=100, qty=5, present
            DEFAULT_ASK, 16'd0, 1'b0,      // ask: default, no ask yet
            "Single BID add"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 2: Two BIDs, second higher price → BBO updates
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 2: Two BIDs, BBO updates to higher ---");
        add_order(32'd105, 16'd2, 1'b0);  // BID 105 x 2
        check_bbo(
            32'd105, 16'd2, 1'b1,
            DEFAULT_ASK, 16'd0, 1'b0,
            "Two BIDs, higher updates BBO"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 3: Add ASK → both sides populated
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 3: Two BIDs + one ASK ---");
        add_order(32'd110, 16'd1, 1'b1);  // ASK 110 x 1
        check_bbo(
            32'd105, 16'd2, 1'b1,
            32'd110, 16'd1, 1'b1,
            "Both sides populated"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 4: ASK improves (lower price)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 4: ASK improves to lower price ---");
        add_order(32'd108, 16'd4, 1'b1);  // ASK 108 x 4
        check_bbo(
            32'd105, 16'd2, 1'b1,
            32'd108, 16'd4, 1'b1,
            "ASK improves to 108"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 5: 4-order replay (matches Python verify_against_verilog_tb)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 5: Python TB replay (4-order sequence) ---");
        do_reset;

        // Order 1: BID 100 x 5
        add_order(32'd100, 16'd5, 1'b0);
        check_bbo(
            32'd100, 16'd5, 1'b1,
            DEFAULT_ASK, 16'd0, 1'b0,
            "Replay: BID 100x5"
        );

        // Order 2: BID 105 x 2
        add_order(32'd105, 16'd2, 1'b0);
        check_bbo(
            32'd105, 16'd2, 1'b1,
            DEFAULT_ASK, 16'd0, 1'b0,
            "Replay: BID 105x2"
        );

        // Order 3: ASK 110 x 1
        add_order(32'd110, 16'd1, 1'b1);
        check_bbo(
            32'd105, 16'd2, 1'b1,
            32'd110, 16'd1, 1'b1,
            "Replay: ASK 110x1"
        );

        // Order 4: ASK 108 x 4
        add_order(32'd108, 16'd4, 1'b1);
        check_bbo(
            32'd105, 16'd2, 1'b1,
            32'd108, 16'd4, 1'b1,
            "Replay: ASK 108x4"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 6: Cancel non-BBO order (O(1) path, BBO unchanged)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 6: Cancel non-BBO order ---");
        // Order ID 0 is BID 100x5, which is NOT the best bid (105 is best)
        cancel_order(8'd0);  // Cancel order 0 (BID 100 x 5)
        check_bbo(
            32'd105, 16'd2, 1'b1,
            32'd108, 16'd4, 1'b1,
            "Cancel non-BBO: BBO unchanged"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 7: Cancel BBO bid → triggers rebuild
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 7: Cancel BBO bid, triggers rebuild ---");
        // Order ID 1 is BID 105x2 — this IS the best bid
        // After cancel, no more bids → bid_present should go to 0
        cancel_order(8'd1);  // Cancel order 1 (BID 105 x 2)
        check_bbo(
            32'd0, 16'd0, 1'b0,           // No bids remaining
            32'd108, 16'd4, 1'b1,
            "Cancel BBO bid: rebuild, no bids left"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 8: Cancel BBO ask → triggers rebuild
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 8: Cancel BBO ask, triggers rebuild ---");
        // Order ID 3 is ASK 108x4 — this IS the best ask
        // After cancel, ASK 110x1 remains → best ask = 110
        cancel_order(8'd3);  // Cancel order 3 (ASK 108 x 4)
        check_bbo(
            32'd0, 16'd0, 1'b0,
            32'd110, 16'd1, 1'b1,
            "Cancel BBO ask: rebuild to 110"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 9: Cancel all remaining → empty book
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 9: Cancel all → empty book ---");
        // Only order 2 (ASK 110x1) remains
        cancel_order(8'd2);
        check_bbo(
            32'd0, 16'd0, 1'b0,
            DEFAULT_ASK, 16'd0, 1'b0,
            "Empty book: defaults"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 10: Fill to capacity (64 orders)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 10: Fill to capacity (64 orders) ---");
        do_reset;
        begin : fill_block
            integer j;
            for (j = 0; j < 32; j = j + 1) begin
                add_order(32'd1000 + j, 16'd1, 1'b0);  // BIDs 1000-1031
            end
            for (j = 0; j < 32; j = j + 1) begin
                add_order(32'd2000 + j, 16'd1, 1'b1);  // ASKs 2000-2031
            end
        end
        check_bbo(
            32'd1031, 16'd1, 1'b1,       // Best bid = highest = 1031
            32'd2000, 16'd1, 1'b1,       // Best ask = lowest = 2000
            "64 orders: bid=1031, ask=2000"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 11: Same price, multiple orders (level aggregation)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 11: Level aggregation (same price) ---");
        do_reset;
        add_order(32'd500, 16'd10, 1'b0);  // BID 500 x 10
        add_order(32'd500, 16'd20, 1'b0);  // BID 500 x 20 (same price)
        add_order(32'd500, 16'd5,  1'b0);  // BID 500 x 5  (same price)
        check_bbo(
            32'd500, 16'd35, 1'b1,         // Total qty = 10+20+5 = 35
            DEFAULT_ASK, 16'd0, 1'b0,
            "Level aggregation: qty=35"
        );

        // ══════════════════════════════════════════════════════════════
        //  TEST 12: Interleaved add/cancel storm (20 ops)
        // ══════════════════════════════════════════════════════════════
        $display("");
        $display("--- Test 12: Interleaved add/cancel storm ---");
        do_reset;

        // Build initial book
        add_order(32'd200, 16'd10, 1'b0);  // ID=0: BID 200x10
        add_order(32'd210, 16'd5,  1'b0);  // ID=1: BID 210x5
        add_order(32'd220, 16'd8,  1'b0);  // ID=2: BID 220x8  ← best bid
        add_order(32'd300, 16'd3,  1'b1);  // ID=3: ASK 300x3
        add_order(32'd290, 16'd7,  1'b1);  // ID=4: ASK 290x7  ← best ask
        add_order(32'd310, 16'd2,  1'b1);  // ID=5: ASK 310x2

        // Cancel non-BBO
        cancel_order(8'd0);  // Cancel BID 200x10 (not BBO)
        check_bbo(
            32'd220, 16'd8, 1'b1,
            32'd290, 16'd7, 1'b1,
            "Storm: cancel non-BBO bid"
        );

        // Add better bid
        add_order(32'd225, 16'd12, 1'b0);  // ID=6: BID 225x12 → new best bid
        check_bbo(
            32'd225, 16'd12, 1'b1,
            32'd290, 16'd7, 1'b1,
            "Storm: new best bid 225"
        );

        // Add better ask
        add_order(32'd285, 16'd6, 1'b1);  // ID=7: ASK 285x6 → new best ask
        check_bbo(
            32'd225, 16'd12, 1'b1,
            32'd285, 16'd6, 1'b1,
            "Storm: new best ask 285"
        );

        // Cancel the best bid (225) → rebuild finds 220
        cancel_order(8'd6);
        check_bbo(
            32'd220, 16'd8, 1'b1,
            32'd285, 16'd6, 1'b1,
            "Storm: cancel BBO bid, rebuild to 220"
        );

        // Cancel the best ask (285) → rebuild finds 290
        cancel_order(8'd7);
        check_bbo(
            32'd220, 16'd8, 1'b1,
            32'd290, 16'd7, 1'b1,
            "Storm: cancel BBO ask, rebuild to 290"
        );

        // Add equal-price orders (test aggregation under stress)
        add_order(32'd220, 16'd4, 1'b0);  // ID=8: same as best bid price
        check_bbo(
            32'd220, 16'd12, 1'b1,         // 8 + 4 = 12
            32'd290, 16'd7, 1'b1,
            "Storm: aggregate at best bid"
        );

        // ══════════════════════════════════════════════════════════════
        //  RESULTS SUMMARY
        // ══════════════════════════════════════════════════════════════
        total_tests = pass_count + fail_count;

        $display("");
        $display("======================================================================");
        $display("  RESULTS SUMMARY");
        $display("======================================================================");
        $display("  Total tests: %0d", total_tests);
        $display("  Passed:      %0d", pass_count);
        $display("  Failed:      %0d", fail_count);
        $display("  O(1) hits:   %0d", o1_hit_count);
        $display("  O(logN) hits:%0d", logn_hit_count);
        $display("======================================================================");

        if (fail_count == 0)
            $display("  >>> ALL TESTS PASSED <<< ");
        else
            $display("  >>> SOME TESTS FAILED <<< ");

        $display("======================================================================");
        $display("");

        #100;
        $finish;
    end

    // ════════════════════════════════════════════════════════════════════
    //  Timeout watchdog (prevent infinite hang)
    // ════════════════════════════════════════════════════════════════════
    initial begin
        #5000000;  // 5 ms timeout
        $display("");
        $display("  [ERROR] SIMULATION TIMEOUT — possible FSM deadlock!");
        $display("");
        $finish;
    end

endmodule
