`timescale 1ns / 1ps

// =============================================================================
// tb.v - Vivado xsim testbench for bitmap.v
//
// Tests:
//   1. BASIC      - single bid + ask, verify correct BBO price and qty
//   2. CANCEL     - add two bids, cancel best, verify BBO moves to next level
//   3. BYPASS     - back-to-back same-block updates, stress-test forwarding mux
//   4. BURST      - 8 updates spaced 5 cycles apart, verify all commits
//   5. QTY_CHANGE - same price different qty, verify bbo_valid still pulses
//   6. LATENCY    - 20 samples, cancel+add pattern guarantees BBO change each time
//
// Expected latency: 9 cycles (36 ns at 250 MHz) deterministic.
//
// Vivado: set sim runtime to 500us before running.
// =============================================================================

module order_book_tb;

    // =========================================================================
    // PARAMETERS
    // =========================================================================

    parameter IDX_WIDTH  = 12;
    parameter QTY_WIDTH  = 32;
    parameter CLK_PERIOD = 4;           // 4 ns = 250 MHz
    parameter BASE_PRICE = 32'd10000;   // price at index 0

    // =========================================================================
    // DUT SIGNALS
    // =========================================================================

    reg                    clk;
    reg                    reset;
    reg  [IDX_WIDTH-1:0]   update_idx;
    reg  [QTY_WIDTH-1:0]   update_qty;
    reg                    update_valid;
    reg                    side;
    wire [31:0]            best_bid_price;
    wire [31:0]            best_ask_price;
    wire [QTY_WIDTH-1:0]   best_bid_qty;
    wire [QTY_WIDTH-1:0]   best_ask_qty;
    wire                   bbo_valid;

    // =========================================================================
    // DUT INSTANTIATION
    // =========================================================================

    bitmap #(
        .IDX_WIDTH (IDX_WIDTH),
        .QTY_WIDTH (QTY_WIDTH)
    ) dut (
        .clk           (clk),
        .reset         (reset),
        .update_idx    (update_idx),
        .update_qty    (update_qty),
        .update_valid  (update_valid),
        .side          (side),
        .base_price    (BASE_PRICE),
        .best_bid_price(best_bid_price),
        .best_ask_price(best_ask_price),
        .best_bid_qty  (best_bid_qty),
        .best_ask_qty  (best_ask_qty),
        .bbo_valid     (bbo_valid)
    );

    // =========================================================================
    // CLOCK - 250 MHz
    // =========================================================================

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // CYCLE COUNTER
    // =========================================================================

    integer cycle_count;
    initial cycle_count = 0;
    always @(posedge clk) cycle_count = cycle_count + 1;

    // =========================================================================
    // TRACKING VARIABLES
    // =========================================================================

    integer t_send;
    integer t_recv;
    integer measured_latency;
    integer latency_sum;
    integer latency_count;
    integer latency_min;
    integer latency_max;
    integer pass_count;
    integer fail_count;

    // =========================================================================
    // TASK: do_reset
    // =========================================================================

    task do_reset;
        begin
            reset        = 1'b1;
            update_valid = 1'b0;
            update_idx   = {IDX_WIDTH{1'b0}};
            update_qty   = {QTY_WIDTH{1'b0}};
            side         = 1'b0;
            repeat(4) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
            repeat(2) @(posedge clk);
        end
    endtask

    // =========================================================================
    // TASK: wait_and_check
    //   Waits for bbo_valid using while loop (xsim safe).
    //   Prints PASS/FAIL with corrected latency.
    // =========================================================================

    task wait_and_check;
        input [8*32-1:0]      test_name;
        input [31:0]          exp_bid_price;
        input [31:0]          exp_ask_price;
        input [QTY_WIDTH-1:0] exp_bid_qty;
        input [QTY_WIDTH-1:0] exp_ask_qty;
        integer tout;
        begin
            tout = 0;
            @(posedge clk);

            while (!bbo_valid && tout < 50) begin
                @(posedge clk);
                tout = tout + 1;
            end

            if (tout >= 50) begin
                $display("TIMEOUT  [%0s]  bbo_valid never asserted", test_name);
                fail_count = fail_count + 1;
            end else begin
                t_recv           = cycle_count;
                // +1 corrects for negedge drive vs posedge sample offset
                measured_latency = t_recv - t_send + 1;
                latency_sum      = latency_sum + measured_latency;
                latency_count    = latency_count + 1;
                if (measured_latency < latency_min) latency_min = measured_latency;
                if (measured_latency > latency_max) latency_max = measured_latency;

                if (best_bid_price == exp_bid_price &&
                    best_ask_price == exp_ask_price &&
                    best_bid_qty   == exp_bid_qty   &&
                    best_ask_qty   == exp_ask_qty) begin
                    $display("PASS     [%0s]  latency=%0d cycles (%0d ns) | BID qty=%0d price=%0d  ASK qty=%0d price=%0d",
                        test_name,
                        measured_latency, measured_latency * CLK_PERIOD,
                        best_bid_qty, best_bid_price,
                        best_ask_qty, best_ask_price);
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL     [%0s]  latency=%0d cycles (%0d ns)",
                        test_name, measured_latency, measured_latency * CLK_PERIOD);
                    $display("         GOT   BID qty=%0d price=%0d  ASK qty=%0d price=%0d",
                        best_bid_qty, best_bid_price,
                        best_ask_qty, best_ask_price);
                    $display("         WANT  BID qty=%0d price=%0d  ASK qty=%0d price=%0d",
                        exp_bid_qty, exp_bid_price,
                        exp_ask_qty, exp_ask_price);
                    fail_count = fail_count + 1;
                end
            end

            repeat(15) @(posedge clk);
        end
    endtask

    // =========================================================================
    // MAIN
    // =========================================================================

    initial begin

        latency_sum   = 0;
        latency_count = 0;
        latency_min   = 999999;
        latency_max   = 0;
        pass_count    = 0;
        fail_count    = 0;

        $display("");
        $display("=================================================================");
        $display("  bitmap.v - Behavioral Simulation  (%0d MHz)", 1000/CLK_PERIOD);
        $display("  BASE_PRICE=%0d  IDX_WIDTH=%0d  QTY_WIDTH=%0d",
                  BASE_PRICE, IDX_WIDTH, QTY_WIDTH);
        $display("=================================================================");

        // -----------------------------------------------------------------
        // TEST 1: BASIC
        //   bid idx=5  ($100.05) qty=500
        //   ask idx=10 ($100.10) qty=200
        //   Both driven back-to-back then BBO checked.
        // -----------------------------------------------------------------
        $display("\n--- TEST 1: BASIC ---");
        do_reset;

        @(negedge clk);
        update_idx=12'd5; update_qty=32'd500; side=1'b0; update_valid=1'b1;
        t_send = cycle_count;
        @(negedge clk);
        update_idx=12'd10; update_qty=32'd200; side=1'b1;
        @(negedge clk);
        update_valid=1'b0;

        wait_and_check("BASIC",
            BASE_PRICE+32'd5,  BASE_PRICE+32'd10,
            32'd500,           32'd200);

        // -----------------------------------------------------------------
        // TEST 2: CANCEL
        //   Seed: bid@3 qty=100, bid@5 qty=500, ask@10 qty=200
        //   Cancel bid@5 (qty=0) -> best bid falls to idx 3
        // -----------------------------------------------------------------
        $display("\n--- TEST 2: CANCEL ---");
        do_reset;

        @(negedge clk); update_idx=12'd3;  update_qty=32'd100; side=1'b0; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(3) @(posedge clk);

        @(negedge clk); update_idx=12'd5;  update_qty=32'd500; side=1'b0; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(3) @(posedge clk);

        @(negedge clk); update_idx=12'd10; update_qty=32'd200; side=1'b1; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(20) @(posedge clk);

        @(negedge clk);
        update_idx=12'd5; update_qty=32'd0; side=1'b0; update_valid=1'b1;
        t_send = cycle_count;
        @(negedge clk);
        update_valid=1'b0;

        wait_and_check("CANCEL",
            BASE_PRICE+32'd3,  BASE_PRICE+32'd10,
            32'd100,           32'd200);

        // -----------------------------------------------------------------
        // TEST 3: BYPASS
        //   idx 0 and idx 1 are in the same L0 block (block 0, bits 0 and 1).
        //   Two consecutive-cycle bid updates exercise the RMW forwarding mux.
        //   Expected: best bid = idx 1 (higher price), qty 999
        // -----------------------------------------------------------------
        $display("\n--- TEST 3: BYPASS (same-block consecutive) ---");
        do_reset;

        @(negedge clk); update_idx=12'd60; update_qty=32'd300; side=1'b1; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(20) @(posedge clk);

        @(negedge clk);
        update_idx=12'd0; update_qty=32'd100; side=1'b0; update_valid=1'b1;
        t_send = cycle_count;
        @(negedge clk);
        update_idx=12'd1; update_qty=32'd999; side=1'b0;
        @(negedge clk);
        update_valid=1'b0;

        wait_and_check("BYPASS",
            BASE_PRICE+32'd1,  BASE_PRICE+32'd60,
            32'd999,           32'd300);

        // -----------------------------------------------------------------
        // TEST 4: BURST
        //   8 updates spaced 5 cycles apart so each RMW fully commits.
        //   Even i = bid at idx i*10  (i=0,2,4,6 -> idx 0,20,40,60)
        //   Odd  i = ask at idx i*10  (i=1,3,5,7 -> idx 10,30,50,70)
        //   Expected best bid: idx 60 qty 700
        //   Expected best ask: idx 10 qty 200
        // -----------------------------------------------------------------
        $display("\n--- TEST 4: BURST (8 updates, 5 cycles apart) ---");
        do_reset;

        begin : burst_block
            integer bi;
            for (bi = 0; bi < 8; bi = bi + 1) begin
                @(negedge clk);
                update_idx   = bi * 10;
                update_qty   = (bi + 1) * 100;
                side         = bi[0];
                update_valid = 1'b1;
                if (bi == 0) t_send = cycle_count;
                @(negedge clk);
                update_valid = 1'b0;
                repeat(3) @(posedge clk); // 5 cycles total: 1 drive + 1 deassert + 3 drain
            end
        end

        wait_and_check("BURST",
            BASE_PRICE+32'd60, BASE_PRICE+32'd10,
            32'd700,           32'd200);

        // -----------------------------------------------------------------
        // TEST 5: QTY_CHANGE
        //   Best bid stays at idx 5, qty changes 500 -> 123.
        //   bbo_valid must pulse (qty tracked in edge detector).
        // -----------------------------------------------------------------
        $display("\n--- TEST 5: QTY_CHANGE ---");
        do_reset;

        @(negedge clk); update_idx=12'd5;  update_qty=32'd500; side=1'b0; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(3) @(posedge clk);
        @(negedge clk); update_idx=12'd10; update_qty=32'd200; side=1'b1; update_valid=1'b1;
        @(negedge clk); update_valid=1'b0;
        repeat(20) @(posedge clk);

        @(negedge clk);
        update_idx=12'd5; update_qty=32'd123; side=1'b0; update_valid=1'b1;
        t_send = cycle_count;
        @(negedge clk);
        update_valid=1'b0;

        wait_and_check("QTY_CHANGE",
            BASE_PRICE+32'd5,  BASE_PRICE+32'd10,
            32'd123,           32'd200);

        // -----------------------------------------------------------------
        // TEST 6: LATENCY MEASUREMENT - 20 samples
        //
        //   Strategy: cancel+add pattern on each sample.
        //   Alternates best bid between idx 10 and idx 20.
        //   Cancel the current best first (qty=0), wait for RMW to commit,
        //   then add new best at the other price. This guarantees a genuine
        //   BBO price change every sample so bbo_valid fires reliably.
        //
        //   t_send records the ADD cycle (not the cancel).
        //   Latency = cycles from ADD update_valid to bbo_valid output.
        //   Expected: 9 cycles every sample.
        // -----------------------------------------------------------------
        $display("\n--- TEST 6: LATENCY MEASUREMENT (20 samples) ---");
        do_reset;

        begin : latency_block
            integer ls;
            integer lw;
            integer l_min, l_max, l_sum;
            l_min = 999999; l_max = 0; l_sum = 0;

            // Seed: standing ask at idx 50, initial bid at idx 10
            @(negedge clk); update_idx=12'd50; update_qty=32'd999; side=1'b1; update_valid=1'b1;
            @(negedge clk); update_valid=1'b0;
            repeat(6) @(posedge clk);

            @(negedge clk); update_idx=12'd10; update_qty=32'd500; side=1'b0; update_valid=1'b1;
            @(negedge clk); update_valid=1'b0;
            repeat(20) @(posedge clk);

            for (ls = 0; ls < 20; ls = ls + 1) begin

                // Step 1: cancel the current best bid
                @(negedge clk);
                update_idx   = (ls[0]) ? 12'd20 : 12'd10; // cancel whichever is current best
                update_qty   = 32'd0;
                side         = 1'b0;
                update_valid = 1'b1;
                @(negedge clk); update_valid = 1'b0;
                repeat(6) @(posedge clk); // wait for cancel to fully commit

                // Step 2: add new best bid at the other price
                @(negedge clk);
                update_idx   = (ls[0]) ? 12'd10 : 12'd20;
                update_qty   = ls + 1;
                side         = 1'b0;
                update_valid = 1'b1;
                t_send       = cycle_count;
                @(negedge clk);
                update_valid = 1'b0;

                // Wait for bbo_valid
                lw = 0;
                @(posedge clk);
                while (!bbo_valid && lw < 50) begin
                    @(posedge clk);
                    lw = lw + 1;
                end

                if (bbo_valid) begin
                    // +1 corrects negedge/posedge offset
                    measured_latency = cycle_count - t_send + 1;
                    l_sum = l_sum + measured_latency;
                    if (measured_latency < l_min) l_min = measured_latency;
                    if (measured_latency > l_max) l_max = measured_latency;
                    $display("  sample %02d : %0d cycles  (%0d ns)",
                             ls, measured_latency, measured_latency * CLK_PERIOD);
                end else begin
                    $display("  sample %02d : TIMEOUT", ls);
                end

                repeat(20) @(posedge clk);
            end

            $display("  -------------------------------------------------------");
            $display("  min : %0d cycles  (%0d ns)", l_min, l_min * CLK_PERIOD);
            $display("  max : %0d cycles  (%0d ns)", l_max, l_max * CLK_PERIOD);
            $display("  avg : %0d cycles  (%0d ns)", l_sum/20, (l_sum/20)*CLK_PERIOD);
        end

        // -----------------------------------------------------------------
        // SUMMARY
        // -----------------------------------------------------------------
        $display("");
        $display("=================================================================");
        $display("  SIMULATION COMPLETE");
        $display("  Tests passed : %0d / %0d", pass_count, pass_count+fail_count);
        $display("  Tests failed : %0d", fail_count);
        if (latency_count > 0) begin
            $display("  Latency (tests 1-5, %0d samples):", latency_count);
            $display("    min : %0d cycles  (%0d ns)", latency_min, latency_min*CLK_PERIOD);
            $display("    max : %0d cycles  (%0d ns)", latency_max, latency_max*CLK_PERIOD);
            $display("    avg : %0d cycles  (%0d ns)",
                latency_sum/latency_count, (latency_sum/latency_count)*CLK_PERIOD);
        end
        $display("=================================================================");
        $display("");

        $finish;
    end

    // =========================================================================
    // WATCHDOG
    // =========================================================================

    initial begin
        #500000000; // 500ms wall-clock limit (sim time)
        $display("WATCHDOG: sim exceeded limit - forcing exit");
        $finish;
    end

endmodule