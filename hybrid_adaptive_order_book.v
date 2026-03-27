`timescale 1ns / 1ps

module hybrid_adaptive_order_book #(
    parameter ORDER_DEPTH    = 64,
    parameter LEVEL_DEPTH    = 64,
    parameter PRICE_WIDTH    = 32,
    parameter QTY_WIDTH      = 16,
    parameter ORDER_ID_WIDTH = 8,
    parameter DEFAULT_ASK    = 32'h000003E7  // 999
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // ── Command Interface ──────────────────────────────────────────────
    input  wire                       cmd_valid,
    input  wire                       cmd_is_cancel,   // 0=ADD, 1=CANCEL
    input  wire [PRICE_WIDTH-1:0]     cmd_price,
    input  wire [QTY_WIDTH-1:0]       cmd_qty,
    input  wire                       cmd_side,        // 0=BID, 1=ASK
    input  wire [ORDER_ID_WIDTH-1:0]  cmd_cancel_id,

    // ── Response Interface ─────────────────────────────────────────────
    output reg                        resp_valid,
    output reg                        resp_ready,
    output reg  [PRICE_WIDTH-1:0]     best_bid_price,
    output reg  [QTY_WIDTH-1:0]       best_bid_qty,
    output reg                        best_bid_present,
    output reg  [PRICE_WIDTH-1:0]     best_ask_price,
    output reg  [QTY_WIDTH-1:0]       best_ask_qty,
    output reg                        best_ask_present,
    output reg  [ORDER_ID_WIDTH-1:0]  assigned_order_id,

    // ── Telemetry ──────────────────────────────────────────────────────
    output reg  [31:0]                o1_hit_count,
    output reg  [31:0]                logn_hit_count
);

    // ====================================================================
    //  Local Parameters
    // ====================================================================
    localparam OD_BITS = $clog2(ORDER_DEPTH);  // 6
    localparam LD_BITS = $clog2(LEVEL_DEPTH);  // 6

    // ====================================================================
    //  FSM States
    // ====================================================================
    localparam [4:0]
        S_IDLE              = 5'd0,
        S_ADD_WRITE         = 5'd1,
        S_ADD_LEVEL_SCAN    = 5'd2,
        S_ADD_LEVEL_UPDATE  = 5'd3,
        S_ADD_LEVEL_CREATE  = 5'd4,
        S_ADD_BBO_CHECK     = 5'd5,
        S_CANCEL_SCAN       = 5'd6,
        S_CANCEL_FOUND      = 5'd7,
        S_CANCEL_LEVEL_SCAN = 5'd8,
        S_CANCEL_LEVEL_UPD  = 5'd9,
        S_CANCEL_BBO_CHECK  = 5'd10,
        S_CANCEL_BBO_WAIT   = 5'd11,  // wait for need_rebuild NBA
        S_REBUILD_SCAN      = 5'd12,
        S_REBUILD_DONE      = 5'd13,
        S_DONE              = 5'd14;

    reg [4:0] state, state_next;

    // ====================================================================
    //  Order Store
    // ====================================================================
    reg                       order_valid  [0:ORDER_DEPTH-1];
    reg                       order_side   [0:ORDER_DEPTH-1];
    reg [PRICE_WIDTH-1:0]     order_price  [0:ORDER_DEPTH-1];
    reg [QTY_WIDTH-1:0]       order_qty    [0:ORDER_DEPTH-1];
    reg [ORDER_ID_WIDTH-1:0]  order_id_mem [0:ORDER_DEPTH-1];

    reg [OD_BITS-1:0]         wr_ptr;
    reg [ORDER_ID_WIDTH-1:0]  next_order_id;

    // ====================================================================
    //  Price Level Store (with side field)
    // ====================================================================
    reg                       level_valid     [0:LEVEL_DEPTH-1];
    reg                       level_side      [0:LEVEL_DEPTH-1];
    reg [PRICE_WIDTH-1:0]     level_price     [0:LEVEL_DEPTH-1];
    reg [QTY_WIDTH-1:0]       level_total_qty [0:LEVEL_DEPTH-1];
    reg [LD_BITS:0]           level_count;

    // ====================================================================
    //  Working Registers
    // ====================================================================
    reg [PRICE_WIDTH-1:0]     w_price;
    reg [QTY_WIDTH-1:0]       w_qty;
    reg                       w_side;
    reg [ORDER_ID_WIDTH-1:0]  w_cancel_id;

    // Scan counters (extra bit to avoid overflow)
    reg [OD_BITS:0]           scan_idx;
    reg [LD_BITS:0]           level_scan_idx;
    reg                       scan_done;

    // Level lookup results
    reg                       level_found;
    reg [LD_BITS-1:0]         level_found_idx;

    // Cancel lookup results
    reg                       cancel_found;
    reg [OD_BITS-1:0]         cancel_found_idx;
    reg [PRICE_WIDTH-1:0]     cancel_price;
    reg [QTY_WIDTH-1:0]       cancel_qty;
    reg                       cancel_side;
    reg                       need_rebuild;

    // Saved level qty before update (for BBO check)
    reg [QTY_WIDTH-1:0]       saved_level_qty;

    // Rebuild
    reg [PRICE_WIDTH-1:0]     rebuild_best_price;
    reg [QTY_WIDTH-1:0]       rebuild_best_qty;
    reg                       rebuild_found;

    integer i;

    // ====================================================================
    //  FSM: State Register
    // ====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= state_next;
    end

    // ====================================================================
    //  FSM: Next-State Logic
    // ====================================================================
    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE: begin
                if (cmd_valid)
                    state_next = cmd_is_cancel ? S_CANCEL_SCAN : S_ADD_WRITE;
            end

            S_ADD_WRITE:        state_next = S_ADD_LEVEL_SCAN;
            S_ADD_LEVEL_SCAN: begin
                if (level_found)
                    state_next = S_ADD_LEVEL_UPDATE;
                else if (scan_done)
                    state_next = S_ADD_LEVEL_CREATE;
            end
            S_ADD_LEVEL_UPDATE: state_next = S_ADD_BBO_CHECK;
            S_ADD_LEVEL_CREATE: state_next = S_ADD_BBO_CHECK;
            S_ADD_BBO_CHECK:    state_next = S_DONE;

            S_CANCEL_SCAN: begin
                if (cancel_found)
                    state_next = S_CANCEL_FOUND;
                else if (scan_done)
                    state_next = S_DONE;
            end
            S_CANCEL_FOUND:     state_next = S_CANCEL_LEVEL_SCAN;
            S_CANCEL_LEVEL_SCAN: begin
                if (level_found)
                    state_next = S_CANCEL_LEVEL_UPD;
                else if (scan_done)
                    state_next = S_CANCEL_BBO_CHECK;
            end
            S_CANCEL_LEVEL_UPD: state_next = S_CANCEL_BBO_CHECK;
            S_CANCEL_BBO_CHECK: state_next = S_CANCEL_BBO_WAIT;
            S_CANCEL_BBO_WAIT: begin
                state_next = need_rebuild ? S_REBUILD_SCAN : S_DONE;
            end

            S_REBUILD_SCAN: begin
                if (scan_done)
                    state_next = S_REBUILD_DONE;
            end
            S_REBUILD_DONE:     state_next = S_DONE;
            S_DONE:             state_next = S_IDLE;
            default:            state_next = S_IDLE;
        endcase
    end

    // ====================================================================
    //  FSM: Datapath
    // ====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr           <= 0;
            next_order_id    <= 0;
            level_count      <= 0;
            best_bid_price   <= 0;
            best_bid_qty     <= 0;
            best_bid_present <= 0;
            best_ask_price   <= DEFAULT_ASK;
            best_ask_qty     <= 0;
            best_ask_present <= 0;
            resp_valid       <= 0;
            resp_ready       <= 1;
            assigned_order_id<= 0;
            o1_hit_count     <= 0;
            logn_hit_count   <= 0;
            scan_idx         <= 0;
            level_scan_idx   <= 0;
            scan_done        <= 0;
            level_found      <= 0;
            level_found_idx  <= 0;
            cancel_found     <= 0;
            cancel_found_idx <= 0;
            need_rebuild     <= 0;
            rebuild_best_price <= 0;
            rebuild_best_qty <= 0;
            rebuild_found    <= 0;
            w_price          <= 0;
            w_qty            <= 0;
            w_side           <= 0;
            w_cancel_id      <= 0;
            cancel_price     <= 0;
            cancel_qty       <= 0;
            cancel_side      <= 0;
            saved_level_qty  <= 0;

            for (i = 0; i < ORDER_DEPTH; i = i + 1) begin
                order_valid[i]  <= 0;
                order_side[i]   <= 0;
                order_price[i]  <= 0;
                order_qty[i]    <= 0;
                order_id_mem[i] <= 0;
            end
            for (i = 0; i < LEVEL_DEPTH; i = i + 1) begin
                level_valid[i]     <= 0;
                level_side[i]      <= 0;
                level_price[i]     <= 0;
                level_total_qty[i] <= 0;
            end
        end
        else begin
            resp_valid <= 0;

            case (state)
                // ── IDLE ────────────────────────────────────────────────
                S_IDLE: begin
                    if (cmd_valid) begin
                        resp_ready      <= 0;
                        w_price         <= cmd_price;
                        w_qty           <= cmd_qty;
                        w_side          <= cmd_side;
                        w_cancel_id     <= cmd_cancel_id;
                        level_found     <= 0;
                        level_found_idx <= 0;
                        cancel_found    <= 0;
                        cancel_found_idx<= 0;
                        need_rebuild    <= 0;
                        scan_idx        <= 0;
                        level_scan_idx  <= 0;
                        scan_done       <= 0;
                        rebuild_found   <= 0;
                        saved_level_qty <= 0;
                    end
                end

                // ── ADD: Write order to store ───────────────────────────
                S_ADD_WRITE: begin
                    order_valid[wr_ptr]  <= 1;
                    order_side[wr_ptr]   <= w_side;
                    order_price[wr_ptr]  <= w_price;
                    order_qty[wr_ptr]    <= w_qty;
                    order_id_mem[wr_ptr] <= next_order_id;
                    assigned_order_id    <= next_order_id;
                    next_order_id        <= next_order_id + 1;
                    wr_ptr               <= wr_ptr + 1;
                    level_scan_idx       <= 0;
                    level_found          <= 0;
                    scan_done            <= 0;
                end

                // ── ADD: Scan for existing price level ──────────────────
                S_ADD_LEVEL_SCAN: begin
                    if (!level_found && !scan_done) begin
                        if (level_scan_idx >= level_count) begin
                            scan_done <= 1;
                        end
                        else if (level_valid[level_scan_idx[LD_BITS-1:0]] &&
                                 level_side[level_scan_idx[LD_BITS-1:0]] == w_side &&
                                 level_price[level_scan_idx[LD_BITS-1:0]] == w_price) begin
                            level_found     <= 1;
                            level_found_idx <= level_scan_idx[LD_BITS-1:0];
                        end
                        else begin
                            level_scan_idx <= level_scan_idx + 1;
                        end
                    end
                end

                // ── ADD: Update existing level ──────────────────────────
                S_ADD_LEVEL_UPDATE: begin
                    level_total_qty[level_found_idx] <=
                        level_total_qty[level_found_idx] + w_qty;
                end

                // ── ADD: Create new level ───────────────────────────────
                S_ADD_LEVEL_CREATE: begin
                    level_valid[level_count[LD_BITS-1:0]]     <= 1;
                    level_side[level_count[LD_BITS-1:0]]      <= w_side;
                    level_price[level_count[LD_BITS-1:0]]     <= w_price;
                    level_total_qty[level_count[LD_BITS-1:0]] <= w_qty;
                    level_count <= level_count + 1;
                end

                // ── ADD: O(1) BBO check ─────────────────────────────────
                S_ADD_BBO_CHECK: begin
                    if (w_side == 0) begin
                        // BID: update if new price >= cached best bid
                        if (!best_bid_present || w_price >= best_bid_price) begin
                            best_bid_price   <= w_price;
                            // Qty: for existing level, read the UPDATED qty
                            // (level_total_qty was updated last cycle via NBA)
                            if (level_found)
                                best_bid_qty <= level_total_qty[level_found_idx];
                            else
                                best_bid_qty <= w_qty;
                            best_bid_present <= 1;
                        end
                        o1_hit_count <= o1_hit_count + 1;
                    end
                    else begin
                        // ASK: update if new price <= cached best ask
                        if (!best_ask_present || w_price <= best_ask_price) begin
                            best_ask_price   <= w_price;
                            if (level_found)
                                best_ask_qty <= level_total_qty[level_found_idx];
                            else
                                best_ask_qty <= w_qty;
                            best_ask_present <= 1;
                        end
                        o1_hit_count <= o1_hit_count + 1;
                    end
                end

                // ── CANCEL: Scan order store ────────────────────────────
                S_CANCEL_SCAN: begin
                    if (!cancel_found && !scan_done) begin
                        if (scan_idx >= ORDER_DEPTH) begin
                            scan_done <= 1;
                        end
                        else if (order_valid[scan_idx[OD_BITS-1:0]] &&
                                 order_id_mem[scan_idx[OD_BITS-1:0]] == w_cancel_id) begin
                            cancel_found     <= 1;
                            cancel_found_idx <= scan_idx[OD_BITS-1:0];
                            cancel_price     <= order_price[scan_idx[OD_BITS-1:0]];
                            cancel_qty       <= order_qty[scan_idx[OD_BITS-1:0]];
                            cancel_side      <= order_side[scan_idx[OD_BITS-1:0]];
                        end
                        else begin
                            scan_idx <= scan_idx + 1;
                        end
                    end
                end

                // ── CANCEL: Invalidate order ────────────────────────────
                S_CANCEL_FOUND: begin
                    order_valid[cancel_found_idx] <= 0;
                    level_scan_idx <= 0;
                    level_found    <= 0;
                    scan_done      <= 0;
                end

                // ── CANCEL: Find matching price level ───────────────────
                S_CANCEL_LEVEL_SCAN: begin
                    if (!level_found && !scan_done) begin
                        if (level_scan_idx >= level_count) begin
                            scan_done <= 1;
                        end
                        else if (level_valid[level_scan_idx[LD_BITS-1:0]] &&
                                 level_side[level_scan_idx[LD_BITS-1:0]] == cancel_side &&
                                 level_price[level_scan_idx[LD_BITS-1:0]] == cancel_price) begin
                            level_found     <= 1;
                            level_found_idx <= level_scan_idx[LD_BITS-1:0];
                            // Save qty BEFORE decrement for BBO comparison
                            saved_level_qty <= level_total_qty[level_scan_idx[LD_BITS-1:0]];
                        end
                        else begin
                            level_scan_idx <= level_scan_idx + 1;
                        end
                    end
                end

                // ── CANCEL: Decrement level qty ─────────────────────────
                S_CANCEL_LEVEL_UPD: begin
                    if (level_total_qty[level_found_idx] <= cancel_qty) begin
                        level_total_qty[level_found_idx] <= 0;
                        level_valid[level_found_idx]     <= 0;
                    end
                    else begin
                        level_total_qty[level_found_idx] <=
                            level_total_qty[level_found_idx] - cancel_qty;
                    end
                end

                // ── CANCEL: BBO check ───────────────────────────────────
                //  Uses saved_level_qty (pre-decrement) for comparison
                S_CANCEL_BBO_CHECK: begin
                    if (cancel_side == 0) begin
                        // BID cancel
                        if (cancel_price == best_bid_price) begin
                            if (level_found && saved_level_qty > cancel_qty) begin
                                // Level still has orders after cancel
                                best_bid_qty <= saved_level_qty - cancel_qty;
                                o1_hit_count <= o1_hit_count + 1;
                                need_rebuild <= 0;
                            end
                            else begin
                                // Level emptied — need full rebuild
                                logn_hit_count <= logn_hit_count + 1;
                                need_rebuild   <= 1;
                                level_scan_idx <= 0;
                                scan_done      <= 0;
                                rebuild_found  <= 0;
                                rebuild_best_price <= 0;
                            end
                        end
                        else begin
                            // Not at BBO
                            o1_hit_count <= o1_hit_count + 1;
                            need_rebuild <= 0;
                        end
                    end
                    else begin
                        // ASK cancel
                        if (cancel_price == best_ask_price) begin
                            if (level_found && saved_level_qty > cancel_qty) begin
                                best_ask_qty <= saved_level_qty - cancel_qty;
                                o1_hit_count <= o1_hit_count + 1;
                                need_rebuild <= 0;
                            end
                            else begin
                                logn_hit_count <= logn_hit_count + 1;
                                need_rebuild   <= 1;
                                level_scan_idx <= 0;
                                scan_done      <= 0;
                                rebuild_found  <= 0;
                                rebuild_best_price <= DEFAULT_ASK;
                            end
                        end
                        else begin
                            o1_hit_count <= o1_hit_count + 1;
                            need_rebuild <= 0;
                        end
                    end
                end

                // ── REBUILD: Scan levels to find new best ───────────────
                S_REBUILD_SCAN: begin
                    if (!scan_done) begin
                        if (level_scan_idx >= level_count) begin
                            scan_done <= 1;
                        end
                        else begin
                            // Only consider levels matching the cancelled side
                            if (level_valid[level_scan_idx[LD_BITS-1:0]] &&
                                level_side[level_scan_idx[LD_BITS-1:0]] == cancel_side) begin
                                if (cancel_side == 0) begin
                                    // Best BID = max price
                                    if (!rebuild_found ||
                                        level_price[level_scan_idx[LD_BITS-1:0]] > rebuild_best_price) begin
                                        rebuild_found      <= 1;
                                        rebuild_best_price <= level_price[level_scan_idx[LD_BITS-1:0]];
                                        rebuild_best_qty   <= level_total_qty[level_scan_idx[LD_BITS-1:0]];
                                    end
                                end
                                else begin
                                    // Best ASK = min price
                                    if (!rebuild_found ||
                                        level_price[level_scan_idx[LD_BITS-1:0]] < rebuild_best_price) begin
                                        rebuild_found      <= 1;
                                        rebuild_best_price <= level_price[level_scan_idx[LD_BITS-1:0]];
                                        rebuild_best_qty   <= level_total_qty[level_scan_idx[LD_BITS-1:0]];
                                    end
                                end
                            end
                            level_scan_idx <= level_scan_idx + 1;
                        end
                    end
                end

                // ── REBUILD DONE: Apply ─────────────────────────────────
                S_REBUILD_DONE: begin
                    if (cancel_side == 0) begin
                        if (rebuild_found) begin
                            best_bid_price   <= rebuild_best_price;
                            best_bid_qty     <= rebuild_best_qty;
                            best_bid_present <= 1;
                        end
                        else begin
                            best_bid_price   <= 0;
                            best_bid_qty     <= 0;
                            best_bid_present <= 0;
                        end
                    end
                    else begin
                        if (rebuild_found) begin
                            best_ask_price   <= rebuild_best_price;
                            best_ask_qty     <= rebuild_best_qty;
                            best_ask_present <= 1;
                        end
                        else begin
                            best_ask_price   <= DEFAULT_ASK;
                            best_ask_qty     <= 0;
                            best_ask_present <= 0;
                        end
                    end
                end

                // ── DONE ────────────────────────────────────────────────
                S_DONE: begin
                    resp_valid <= 1;
                    resp_ready <= 1;
                end
            endcase
        end
    end

endmodule
