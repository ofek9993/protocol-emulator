/*
 * pemu_shifter - one shifter from the shifter pool.
 *
 * ===========================================================================
 * WHAT THIS BLOCK IS
 * ===========================================================================
 * Two registers, not one - matching AN12174's own diagrams (Figure 4 UART,
 * Figure 7 SPI, Figure 2 I2C), which all draw a labelled `Buffer` block
 * feeding a separate `Shifter` block, joined by a load arrow:
 *
 *   BUFFER (sh_buf)  ---load--->  SHIFTER (sr)  ---1 bit/edge--->  pin
 *
 *   TRANSMIT:  write lands in the BUFFER. The instant the shifter is free,
 *              hardware copies buffer -> shifter and starts draining it,
 *              one bit per timer edge, onto pin_out.
 *   RECEIVE:   pin_in is sampled into the shifter one bit per timer edge;
 *              the assembled byte lands in rdata when the frame completes.
 *
 * The timer is the only thing that makes bits move. This block never counts
 * clocks itself - see pemu_timer.v for where timing comes from.
 *
 * ===========================================================================
 * WHY A SEPARATE BUFFER (not folded into the shift register)
 * ===========================================================================
 * AN12174, SPI section, states the mechanism directly: "The shifter status
 * flag is set and cleared each time the SHIFTBUF register is written and
 * read, which means the data in the SHIFTBUF has been transferred to the
 * Shifter (SHIFTBUF is empty)." In other words: `status_flag` tracks the
 * BUFFER, not the shifter, and it is what triggers the timer.
 *
 * Without this, there is exactly one legal instant to write the next byte -
 * the moment the current one finishes, no earlier, no later - because there
 * is nowhere else for a new byte to go while the shifter is draining. With
 * it, the safe window to write is the ENTIRE current frame: the buffer
 * absorbs the write regardless of when it lands, and the handoff to the
 * shifter happens automatically, with no gap.
 *
 * ===========================================================================
 * THE TIMER RELATIONSHIP
 * ===========================================================================
 *   timer_out   IN  - the timer's square wave. Each selected edge (rising or
 *                     falling, per `timpol`) moves the shift register by one.
 *
 *   status_flag OUT - BUFFER-empty flag (see above). Drops the instant a
 *                     write lands in the buffer; rises again the instant
 *                     the buffer is drained into the shifter. The timer
 *                     watches this as its trigger.
 *
 *   timer_done  IN  - one-cycle pulse when the timer finishes its frame.
 *                     The frame boundary; see B2/B4 below.
 *
 * ===========================================================================
 * CODING LUT (13 bits) - the one thing FlexIO does not have
 * ===========================================================================
 *   lut[12]    pattern length: 0 = 1 bit out, 1 = 2 bits out
 *              (2-bit / Manchester path is decoded but NOT built - see NOTE)
 *   lut[2:0]   entry for (state=0, bit=0)  as {next_state, pat1, pat0}
 *   lut[5:3]   entry for (state=0, bit=1)
 *   lut[8:6]   entry for (state=1, bit=0)
 *   lut[11:9]  entry for (state=1, bit=1)
 *
 *   identity = 13'b0_001_000_001_000   (reset default - UART/SPI/I2C)
 *   NRZI     = 13'b0_101_000_000_101   (USB)
 *
 * NOTE: lut[12] is decoded but the 2-bit output serialiser is not built.
 * Only 1-bit codings work today. Manchester changes the shifter/timer rate
 * relationship and is its own piece of work. Stated, not faked.
 *
 * ===========================================================================
 * BUGS FIXED HERE (see AUDIT.md for the full reasoning)
 * ===========================================================================
 *   B1  ORIGINAL FIX: a write mid-transfer was rejected outright, with an
 *       `overrun` flag. That was the safe subset of the real answer.
 *       REAL FIX (this revision): a write mid-transfer now lands in the
 *       buffer instead, exactly as AN12174 describes. `overrun` now means
 *       what it should - the buffer itself was still full from a PREVIOUS
 *       unconsumed write, which is the genuine error case.
 *   B2  if the timer stopped while bits remained, this block stayed `active`
 *       forever - a permanent deadlock. `timer_done` now force-completes
 *       the transfer, which also lets a buffered byte behind it start.
 *   B4  receive never reset its bit counter or LUT state at frame start, so
 *       one bad frame misaligned every frame after it. `timer_done` clears
 *       both.
 *   B5  the received stop bit was discarded unchecked. Now compared against
 *       the expected value; `frame_err` latches on mismatch.
 *
 * SCOPE: the buffer built here is TX only, matching where AN12174's own
 * diagrams draw it as the answer to back-to-back sending. A symmetric RX
 * buffer would need a discrete "software has read rdata" pulse to know when
 * it is safe to accept the next frame; this design's config port is a live
 * combinational read (GOUT), not a consuming register read, so there is no
 * such pulse to build it on. RX still correctly captures one frame at a
 * time (B4), which is what is verified. Documented, not silently assumed.
 */

`default_nettype none

module pemu_shifter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    // ---- configuration -------------------------------------------------
    input  wire [1:0]  smod,      // 0 disabled, 1 receive, 2 transmit
    input  wire        timpol,    // 0 shift on rising edge of timer_out,
                                  // 1 on falling
    input  wire [1:0]  sstart,    // 0/1 no start bit, 2 start=0, 3 start=1
    input  wire [1:0]  sstop,     // 0/1 no stop bit,  2 stop=0,  3 stop=1
    input  wire        dir,       // 0 LSB first, 1 MSB first
    input  wire [12:0] lut,       // coding table

    // ---- from the selected timer ---------------------------------------
    input  wire        timer_out,  // the shift clock
    input  wire        timer_done, // frame boundary (one-cycle pulse)
    input  wire        abort,      // controller: a timeout ended the transfer (AUDIT B27)
    input  wire        clr_err,    // pulse: the host clears overrun / frame_err (CCTL[2], AUDIT B31)

    // ---- data interface ------------------------------------------------
    input  wire [7:0]  wdata,
    input  wire        we,          // pulse: write a byte into the BUFFER
    output reg  [7:0]  rdata,       // last received byte
    output reg         status_flag, // TX: BUFFER empty (write freely).
                                    // RX: byte available.
    output reg         overrun,     // sticky: wrote while buffer still full
    output reg         frame_err,   // sticky: RX stop bit was wrong
    output reg         rx_lastbit,  // RX: the value of the frame's LAST bit
                                    //     (I2C: the ACK slot - 0 = ACK)

    // ---- pin -----------------------------------------------------------
    output wire        pin_out,
    output wire        pin_oe,
    input  wire        pin_in
);

    localparam ST_TX = 2'd2;
    localparam ST_RX = 2'd1;

    // ------------------------------------------------------------------
    // framing: how many bits actually move, and what the extra ones are
    // ------------------------------------------------------------------
    wire has_start = sstart[1];
    wire has_stop  = sstop[1];
    wire start_val = sstart[0];
    wire stop_val  = sstop[0];

    wire [3:0] nbits = 4'd8 + {3'd0, has_start} + {3'd0, has_stop};

    // ------------------------------------------------------------------
    // shift-clock edge detection
    //
    // timer_out is registered once so a level can be turned into an edge.
    // Which edge counts is `timpol` - this is what puts data on one edge of
    // SPI's clock while the receiver samples on the other.
    // ------------------------------------------------------------------
    reg tout_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tout_d <= 1'b0;
        else        tout_d <= timer_out;
    end
    wire tout_rise  = timer_out & ~tout_d;
    wire tout_fall  = ~timer_out & tout_d;
    wire shift_edge = timpol ? tout_fall : tout_rise;

    // ------------------------------------------------------------------
    // bit ordering, applied on the way in (TX) and on the way out (RX)
    // ------------------------------------------------------------------
    wire [7:0] ord_data = dir ? {sh_buf[0], sh_buf[1], sh_buf[2], sh_buf[3],
                                 sh_buf[4], sh_buf[5], sh_buf[6], sh_buf[7]}
                              : sh_buf;

    // Arranged so the first bit to leave is ALWAYS sr[0], whatever the
    // framing. Unused high bits pad with 1 (the idle level). Built from the
    // BUFFER now, since that is what auto-load copies into the shifter.
    reg [9:0] sr_load;
    always @* begin
        case ({has_start, has_stop})
            2'b11:   sr_load = {stop_val, ord_data, start_val};
            2'b10:   sr_load = {1'b1,     ord_data, start_val};
            2'b01:   sr_load = {1'b1,     stop_val, ord_data};
            default: sr_load = {2'b11,              ord_data};
        endcase
    end

    // ------------------------------------------------------------------
    // coding LUT - a 4-entry state machine, not a plain mapping
    // ------------------------------------------------------------------
    reg lut_state;

    function [2:0] lut_entry;
        input        s;
        input        b;
        input [11:0] tbl;
        begin
            case ({s, b})
                2'b00:   lut_entry = tbl[2:0];
                2'b01:   lut_entry = tbl[5:3];
                2'b10:   lut_entry = tbl[8:6];
                default: lut_entry = tbl[11:9];
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // THE BUFFER - a genuinely separate register from the shift register,
    // matching AN12174's own diagram shape. `we` only ever touches this.
    // ------------------------------------------------------------------
    reg [7:0] sh_buf;
    reg       buf_full;

    // ------------------------------------------------------------------
    // shift register and bit counter - the SHIFTER half of the diagram
    // ------------------------------------------------------------------
    reg [9:0] sr;
    reg [3:0] bitcnt;
    reg       rx_void;   // B19: this frame's start bit was wrong - ignore it until the frame ends
    reg       active;

    wire       tx_raw  = sr[0];
    wire [2:0] tx_code = lut_entry(lut_state, tx_raw, lut[11:0]);
    wire       tx_bit  = tx_code[0];

    wire [2:0] rx_code = lut_entry(lut_state, pin_in, lut[11:0]);
    wire       rx_bit  = rx_code[0];

    assign pin_out = (smod == ST_TX) ? tx_bit : 1'b1;
    assign pin_oe  = (smod == ST_TX);

    // Value the shift register will hold after this edge, plus where the 8
    // data bits sit inside it once the frame completes. rdata is latched
    // from this exactly once, on the edge the last bit lands.
    wire [9:0] sr_next_rx = {rx_bit, sr[9:1]};
    wire [3:0] rx_lsb     = (4'd10 - nbits) + {3'd0, has_start};
    wire [9:0] rx_shr     = sr_next_rx >> rx_lsb;
    wire [7:0] rx_raw     = rx_shr[7:0];
    wire [7:0] rx_ord     = dir ? {rx_raw[0], rx_raw[1], rx_raw[2], rx_raw[3],
                                   rx_raw[4], rx_raw[5], rx_raw[6], rx_raw[7]}
                                : rx_raw;

    // B5: the stop bit is the LAST bit shifted in, so after this edge it is
    // sitting at the top of the register.
    wire rx_stop_bit = sr_next_rx[9];
    wire rx_last     = (bitcnt >= nbits - 4'd1);

    // Auto-load condition: the shifter is free AND the buffer holds a byte.
    // This is the mechanism the diagram draws as the arrow between Buffer
    // and Shifter. It fires whether the shifter just finished a previous
    // frame (continuous streaming, zero gap) or was already idle when the
    // write happened (the ordinary single-byte case).
    wire auto_load = (smod == ST_TX) && buf_full && !active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sr          <= 10'h3FF;
            bitcnt      <= 4'd0;
            active      <= 1'b0;
            status_flag <= 1'b1;
            rdata       <= 8'd0;
            rx_lastbit  <= 1'b1;
            rx_void     <= 1'b0;
            lut_state   <= 1'b0;
            overrun     <= 1'b0;
            frame_err   <= 1'b0;
            sh_buf      <= 8'd0;
            buf_full    <= 1'b0;
        end else begin
        // AUDIT B31: the host clears the sticky error flags (CCTL[2]) WITHOUT
        // switching the shifter off - a full-duplex UART can clear an RX
        // framing error while its TX keeps sending. Written FIRST, so an error
        // raised on the same clock (assigned further down) still wins: a clear
        // never swallows a new error.
        if (clr_err) begin overrun <= 1'b0; frame_err <= 1'b0; end
        if (!enable || smod == 2'd0) begin
            // Disabled: hold everything in a clean idle state. Sticky error
            // flags clear here too, so re-enabling gives a fresh start.
            sr          <= 10'h3FF;
            bitcnt      <= 4'd0;
            active      <= 1'b0;
            status_flag <= 1'b1;
            lut_state   <= 1'b0;
            overrun     <= 1'b0;
            frame_err   <= 1'b0;
            buf_full    <= 1'b0;
            rx_void     <= 1'b0;

        end else if (abort) begin
            // AUDIT B27: the transfer was abandoned. Drop the frame in
            // progress AND the byte waiting in the buffer (it belongs to the
            // failed transfer - it must not go out first in the next one).
            // The sticky error flags are KEPT, so the host can still read
            // what went wrong; in receive the "byte arrived" flag is left
            // alone (setting it would look like a byte arriving).
            sr          <= 10'h3FF;         // TX output back to its idle 1 (open drain: released)
            bitcnt      <= 4'd0;
            active      <= 1'b0;
            buf_full    <= 1'b0;
            lut_state   <= 1'b0;
            rx_void     <= 1'b0;
            if (smod == ST_TX) status_flag <= 1'b1;   // buffer empty

        end else if (smod == ST_TX) begin
            // =========================== TRANSMIT ===========================
            // ---- 1. accept a write into the BUFFER -------------------------
            // Evaluated against this cycle's PRE-edge buf_full, same as the
            // auto-load block below, so a write and an auto-load landing on
            // the same clock never race each other: if the buffer was full
            // entering this cycle, the write is a genuine overrun; if it
            // was empty, the write succeeds and auto-load (which also read
            // the pre-edge value) simply runs next cycle instead.
            if (we) begin
                if (buf_full) begin
                    overrun <= 1'b1;          // buffer itself was still full
                end else begin
                    sh_buf      <= wdata;
                    buf_full    <= 1'b1;
                    status_flag <= 1'b0;      // buffer no longer empty
                end
            end

            // ---- 2. auto-load: buffer -> shifter, the instant it's free ----
            if (auto_load) begin
                sr          <= sr_load;
                bitcnt      <= nbits;
                active      <= 1'b1;
                buf_full    <= 1'b0;
                status_flag <= 1'b1;          // buffer empty again - write more
                lut_state   <= 1'b0;

            end else if (active && timer_done) begin
                // B2: the timer finished but bits remain - the two bit-count
                // registers disagree. Force-complete rather than sit here
                // forever waiting for an edge that will never arrive. This
                // also clears `active`, so a byte already waiting in the
                // buffer can auto-load on the very next cycle.
                sr     <= 10'h3FF;
                bitcnt <= 4'd0;
                active <= 1'b0;

            end else if (active && shift_edge) begin
                lut_state <= tx_code[2];
                if (bitcnt <= 4'd1) begin
                    sr     <= 10'h3FF;   // back to the idle level
                    bitcnt <= 4'd0;
                    active <= 1'b0;
                end else begin
                    sr     <= {1'b1, sr[9:1]};
                    bitcnt <= bitcnt - 4'd1;
                end
            end

        end else if (smod == ST_RX) begin
            // =========================== RECEIVE ============================
            if (rx_void) begin
                // B19: a false start. Ignore every edge until the frame
                // boundary, then listen again. Nothing is latched, no flag.
                if (timer_done) begin
                    rx_void   <= 1'b0;
                    bitcnt    <= 4'd0;
                    lut_state <= 1'b0;
                end
            end else if (shift_edge && has_start && bitcnt == 4'd0 && rx_bit != start_val) begin
                // B19: the first sample of a framed receive is the start bit,
                // taken mid-bit. If it is not the start value, the edge that
                // began the frame was noise, not a start bit - a real UART
                // re-checks here and throws the frame away. Before this, a
                // spike on an idle line delivered a phantom 0xFF.
                rx_void <= 1'b1;
            end else if (shift_edge) begin
                lut_state <= rx_code[2];
                sr        <= sr_next_rx;
                if (rx_last) begin
                    bitcnt      <= 4'd0;
                    status_flag <= 1'b1;
                    rdata       <= rx_ord;     // latch once, here
                    rx_lastbit  <= sr_next_rx[9];   // the bit just sampled
                    // B5: check the stop bit instead of discarding it. A bad
                    // stop bit is the classic symptom of a baud mismatch.
                    if (has_stop && (rx_stop_bit != stop_val))
                        frame_err <= 1'b1;
                end else begin
                    bitcnt      <= bitcnt + 4'd1;
                    status_flag <= 1'b0;
                end
            end else if (timer_done) begin
                // B4: frame boundary. Clear the counter and the coding state
                // so a truncated or glitched frame cannot misalign the next
                // one. Without this, one bad frame corrupted every frame
                // that followed.
                bitcnt    <= 4'd0;
                lut_state <= 1'b0;
            end
        end
        end   // the B31 clear's else
    end

    // lut[12] and the LUT's pat1 output feed the 1-bit -> 2-symbol path
    // (Manchester / FM0), which is not built yet
    wire _unused = &{lut[12], tx_code[1], rx_code[1], rx_shr[9:8], 1'b0};

endmodule

`default_nettype wire
