/*
 * pemu_timer - one timer from the timer pool.
 *
 * Modelled on the NXP FlexIO timer (AN5133 section 3.2), trimmed to the
 * fields UART / SPI / I2C actually use.
 *
 * The timer produces a square wave on timer_out. Shifters shift on its
 * edges, so this block *is* the protocol's bit clock. In dual 8-bit
 * baud/bit mode it also counts how many bits have gone by and switches
 * itself off when the transfer is done, which is what lets a whole UART
 * transmitter cost one timer and one shifter.
 *
 * Nothing in here names a protocol - see rule 1 in ARCHITECTURE.md.
 *
 * ===========================================================================
 * DECSRC - decrement on the real pin instead of the clock (AUDIT.md B14)
 * ===========================================================================
 * AN12174's I2C section (5.3.4, Timer 1) configures a SECOND timer whose
 * "Timer decrement source" is "decrement on pin input" rather than the
 * FlexIO clock: *"Each SCL edge makes timer 1 to decrease by 1. Two SCL
 * edges make timer 1 to go through one period, which results in the data
 * in the shifters to shift by one bit."*
 *
 * This is not a speed detail - it is the actual fix for two real problems
 * discovered building the I2C stress tests:
 *
 *   - B14 (dropped first bit): when one timer both generates SCL AND
 *     directly triggers the shift, its first toggle away from idle can
 *     coincide with the shift that consumes bit 7 before anything external
 *     ever sees it (specific to idle-HIGH + shift-on-FALL, which real I2C
 *     requires). Deriving the shift from a SEPARATE timer that only counts
 *     real, observed pin edges removes that coincidence entirely - bit 7
 *     gets a full real half-period on the wire before anything shifts it.
 *
 *   - clock stretching (previously a documented, unfixed gap): if a slave
 *     holds SCL low, the real pin simply does not transition, so a timer
 *     counting real edges does not decrement either - the whole transfer
 *     naturally pauses until the real line catches up. No separate
 *     mechanism needed; it falls out of counting the pin instead of the
 *     clock.
 *
 * DECSRC (1 bit): 0 = decrement on clk (default, everything before this
 * used only this mode). 1 = decrement only on a detected edge (either
 * direction) of pin_in. Everything else about the timer - dual/single
 * mode, enable/disable conditions, output polarity - is unchanged; DECSRC
 * only gates WHEN a tick counts.
 */

`default_nettype none

module pemu_timer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,       // global run/stop from the config block

    // ---- configuration -------------------------------------------------
    input  wire [1:0]  timod,        // 0 disabled, 1 dual 8-bit baud/bit,
                                     // 2 single 16-bit
    input  wire [15:0] timcmp,       // dual8: [7:0] baud div, [15:8] 2*N-1
    input  wire [1:0]  timena,       // 0 always, 1 trigger high,
                                     // 2 trigger rising, 3 pin rising
    input  wire [1:0]  timdis,       // 0 never, 1 on compare, 2 trigger falling
    input  wire        timout,       // output level taken at enable
    input  wire        trgpol,       // 0 active high, 1 active low
    input  wire        pinpol,       // 0 active high, 1 invert pin_in
    input  wire        decsrc,       // 0 decrement on clk, 1 on a real pin edge
    input  wire [7:0]  prediv,       // TIMPRE: count only every (prediv+1)-th clock
                                     // (AUDIT B28: slow UART, PS/2); 0 = every clock
    // ---- controller options (TIMPOL[5:2]); all 0 = the original behaviour
    input  wire        ctrig,        // 1: start on the controller's frame pulse, not trigger_in
    input  wire        startlow,     // 1: the output goes LOW the instant the timer starts
    input  wire        park,         // 1: at stop, KEEP the last output level (don't go idle)
    input  wire        waitpin,      // 1: while the output is high, only count once the pin
                                     //    is really high too - the clock-stretch reflex
    input  wire        ctrl_trig,    // controller: one-clock "run a frame" pulse
    input  wire        release_clk,  // controller: return a parked output to its idle level
    input  wire        abort,        // controller: a timeout ended the transfer - stop NOW,
                                     // go idle, no park (AUDIT B27)

    // ---- inputs (already selected by the mux in the top level) ---------
    input  wire        trigger_in,
    input  wire        pin_in,

    // ---- outputs -------------------------------------------------------
    output reg         timer_out,    // the shift clock
    output wire        timer_active, // currently running
    output reg         timer_done,   // one-cycle pulse when it stops on compare
    output wire        timer_eff     // the clock the shifters see: with waitpin, it only
                                     // goes high once the WIRE is high (samples on the
                                     // real rise, not on the moment we let go)
);

    // ------------------------------------------------------------------
    // trigger / pin edge detection
    // ------------------------------------------------------------------
    wire trg_src = ctrig ? ctrl_trig : trigger_in;
    wire trg     = trgpol ? ~trg_src : trg_src;

    // UART RX has to start on the FALLING edge of the line (the start
    // bit), so the pin used as an enable source needs a polarity control.
    // This is PINPOL in the architecture register map - it was documented
    // but not wired up until the first RX recipe needed it.
    wire pin_eff = pinpol ? ~pin_in : pin_in;

    reg trg_d, pin_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trg_d <= 1'b0;
            pin_d <= 1'b0;
        end else begin
            trg_d <= trg;
            pin_d <= pin_eff;
        end
    end

    wire trg_rise = trg & ~trg_d;
    wire trg_fall = ~trg & trg_d;
    wire pin_rise = pin_eff & ~pin_d;
    // DECSRC needs BOTH directions ("each SCL edge", not just rising).
    wire pin_fall = ~pin_eff & pin_d;
    wire pin_edge = pin_rise | pin_fall;

    // ------------------------------------------------------------------
    // run / stop
    // ------------------------------------------------------------------
    reg running;
    assign timer_active = running;

    wire cfg_on = enable && (timod != 2'd0);

    wire start_cond = (timena == 2'd0) ? 1'b1      :
                      (timena == 2'd1) ? trg       :
                      (timena == 2'd2) ? trg_rise  :
                                         pin_rise;

    // compare_hit is asserted on the toggle that exhausts the bit counter
    wire compare_hit;

    wire stop_cond  = (timdis == 2'd0) ? 1'b0        :
                      (timdis == 2'd1) ? compare_hit :
                      (timdis == 2'd2) ? trg_fall    :
                                         1'b0;

    // Whether THIS cycle counts as a tick at all. Normally every clock
    // does; in DECSRC mode, only a cycle where the real pin just
    // transitioned does. This is the whole mechanism - everything else
    // below is unchanged from before DECSRC existed.
    // waitpin (the clock-stretch reflex): we have let go of the line (output
    // high) but the wire is still low - someone is holding it. Don't count.
    // So the high phase is timed from the REAL rise, not from the moment we
    // released, which also absorbs slow pull-up rise times (tb_realism R5).
    wire hold_high = waitpin && timer_out && !pin_in;
    // TIMPRE (AUDIT B28): with prediv = N the clock-counting tick comes only
    // every N+1 clocks, so one half bit can be up to 256 x 256 clocks - 9600
    // baud and slower fit, and the dual mode still counts the frame's bits.
    // prediv = 0: pre_cnt stays 0, every clock is a tick - exactly as before.
    reg  [7:0] pre_cnt;
    wire pre_zero  = (pre_cnt == 8'd0);
    wire tick      = running && !hold_high && (decsrc ? pin_edge : pre_zero);
    // What the shifters see as their clock. Two rules:
    //  * waitpin: it only goes high once the WIRE is high, so a receiver
    //    samples on the real rise, not the moment we let go;
    //  * startlow: while the timer is not running it reads LOW. Otherwise the
    //    forced low at frame start would look like a falling edge and a
    //    shift-on-fall transmitter would throw its first bit away (the B14
    //    trap), and releasing a parked clock would fake a receive sample.
    //    With this, shifters only ever see edges made DURING a frame.
    assign timer_eff = (startlow && !running) ? 1'b0
                                              : (timer_out && (!waitpin || pin_in));

    // ------------------------------------------------------------------
    // counters
    //   lo: divides the clock. Output toggles every timcmp[7:0]+1 ticks.
    //   hi: counts those toggles. Total toggles = timcmp[15:8]+1.
    // ------------------------------------------------------------------
    reg [7:0]  lo_cnt;
    reg [7:0]  hi_cnt;
    reg [15:0] wide_cnt;   // single 16-bit mode

    wire lo_zero   = (lo_cnt == 8'd0);
    wire wide_zero = (wide_cnt == 16'd0);

    // a toggle happens this cycle
    wire toggle = tick && ((timod == 2'd1) ? lo_zero : wide_zero);

    assign compare_hit = toggle && (timod == 2'd1) && (hi_cnt == 8'd0);

    reg parked;   // stopped with park=1: holding the last level, not idle

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parked     <= 1'b0;
            running    <= 1'b0;
            timer_out  <= 1'b0;
            timer_done <= 1'b0;
            lo_cnt     <= 8'd0;
            hi_cnt     <= 8'd0;
            wide_cnt   <= 16'd0;
            pre_cnt    <= 8'd0;
        end else begin
            // the prescaler: reloads at a frame's start, then counts down to 0
            // and reloads - one tick each time it is at 0 (see `tick`)
            if (!running)                    pre_cnt <= prediv;
            else if (!hold_high && !decsrc)  pre_cnt <= pre_zero ? prediv : pre_cnt - 8'd1;
            timer_done <= 1'b0;

            if (!cfg_on) begin
                running   <= 1'b0;
                parked    <= 1'b0;
                timer_out <= timout;
            end else if (abort) begin
                // AUDIT B27: a timeout ends the whole transfer, not just the
                // table. Without this the frame ran on (waiting for a held
                // SCL) and then PARKED low when the slave let go - holding
                // the bus with nothing left to release it.
                running   <= 1'b0;
                parked    <= 1'b0;
                timer_out <= timout;          // I2C: SCL released
            end else if (!running) begin
                // idle: hold the configured output level, wait to start -
                // unless parked, in which case hold the level we stopped at
                // until the controller releases it (I2C: SCL stays LOW
                // between bytes, so a slave's ACK clock can finish).
                if (!parked || release_clk) begin
                    timer_out <= timout;
                    parked    <= 1'b0;
                end
                if (start_cond) begin
                    running  <= 1'b1;
                    parked   <= 1'b0;
                    lo_cnt   <= timcmp[7:0];
                    hi_cnt   <= timcmp[15:8];
                    wide_cnt <= timcmp;
                    if (startlow) timer_out <= 1'b0;
                end
            end else begin
                // running - everything here now only advances on a tick.
                // With DECSRC=0, tick==running every cycle, so this is
                // bit-for-bit the same behaviour as before DECSRC existed.
                if (tick) begin
                    if (timod == 2'd1) begin
                        if (lo_zero) begin
                            timer_out <= ~timer_out;
                            lo_cnt    <= timcmp[7:0];
                            if (hi_cnt != 8'd0)
                                hi_cnt <= hi_cnt - 8'd1;
                        end else begin
                            lo_cnt <= lo_cnt - 8'd1;
                        end
                    end else begin
                        if (wide_zero) begin
                            timer_out <= ~timer_out;
                            wide_cnt  <= timcmp;
                        end else begin
                            wide_cnt <= wide_cnt - 16'd1;
                        end
                    end
                end

                if (stop_cond) begin
                    running    <= 1'b0;
                    timer_done <= 1'b1;
                    if (park) parked <= 1'b1;   // keep this clock's level
                    // AUDIT B29: the LAST toggle is allowed to happen. The idle
                    // branch returns timer_out to timout on the next clock by
                    // itself. Forcing it here overrode that toggle on the same
                    // clock, so a frame could never end ON a sampling edge (a
                    // UART receiver stopping in the middle of the stop bit
                    // lost that sample). Frames of an even number of toggles
                    // (SPI, I2C, UART TX) already end at the idle level:
                    // unchanged.
                end
            end
        end
    end

endmodule

`default_nettype wire
