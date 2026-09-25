/*
 * pemu_pins - the pin layer: the border between the chip and the real world.
 *
 * Everything that crosses it is made safe here, and ONLY here:
 *
 *   INPUTS  (protocol pins: uio[7:0] and ui_in[6:3])
 *     1. a 2-flop synchroniser on every bit (AUDIT B18). An outside signal
 *        can change at any instant; without this, two flops sampling the
 *        same raw wire in the same clock could see different values.
 *     2. a 3-sample glitch filter (AUDIT B20, part of B19): a new level is
 *        only accepted once it has been seen on 3 consecutive clocks, so a
 *        spike shorter than 2 clocks (40 ns at 50 MHz) never gets in.
 *        filt_long = 1 makes it 4 samples (AUDIT B36): spikes under 3 clocks
 *        (60 ns) never get in - the I2C spec's tSP for Fast / Fast+ mode is
 *        "spikes under 50 ns must be suppressed", which 3 samples do NOT
 *        guarantee (a 49 ns spike can cover 3 samples). One more clock of
 *        latency, so only the configurations that need it switch it on.
 *        filt_on = 0 bypasses the filter (lower latency, for fast buses).
 *   OUTPUTS
 *     3. every pad output is a flop - no combinational glitches reach a pin
 *        (on an open-drain SCL a glitch would be a phantom clock);
 *     3b. one chosen pad can have an OUTPUT HOLD DELAY (AUDIT B24): every
 *        change of its value and drive reaches the pad hold_n clocks late.
 *        I2C: a transmitter must hold SDA >= 300 ns after SCL falls, and SDA
 *        is changed by the shifter AND by controller pin actions - delaying
 *        the pad covers both. hold_n must stay below SCL's low time.
 *        hold_n = 0: off (default).
 *     4. defined reset levels: bidirectional pads released, output-only pads
 *        high (a UART TX line must idle high), SDO and READY low.
 *
 * Nothing else in the design may read uio_in or ui_in[6:3]; the structural
 * check (test/tools/structural_check.py) enforces that - B18 cannot be shown in a
 * zero-delay simulation, so it is checked by structure instead.
 *
 * Logical pin space (4-bit pin numbers used by timers, shifters, controller):
 *    0-7   uio[0..7]      bidirectional
 *    8-11  ui_in[3..6]    input only   (8 = UART RX, Tiny Tapeout standard)
 *   12-15  uo_out[4..7]   output only  (12 = UART TX, Tiny Tapeout standard)
 */

`default_nettype none

module pemu_pins (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        filt_on,
    input  wire        filt_long,      // 4 samples instead of 3 (AUDIT B36)

    // raw pads
    input  wire [7:0]  uio_in,
    input  wire [3:0]  ui_proto,       // ui_in[6:3]
    output reg  [7:0]  uio_out,
    output reg  [7:0]  uio_oe,
    output reg  [7:0]  uo_out,

    // clean inside view
    output wire [15:0] pin_in,         // what every logical pin reads
    input  wire [15:0] pin_val,        // what the pin mux wants on each logical pin
    input  wire [15:0] pin_drv,        //   ...and whether it drives it
    input  wire        sdo,            // config read-back data
    input  wire        ready,          // controller "done"
    input  wire [3:0]  hold_pin,       // the pad with an output hold delay
    input  wire [3:0]  hold_n          // its delay in clocks, 0 = off
);

    // ---------------------------------------------------------- inputs
    wire [11:0] raw = {ui_proto, uio_in};
    reg  [11:0] s1, s2, h1, h2, h3, filt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 12'hFFF; s2 <= 12'hFFF; h1 <= 12'hFFF; h2 <= 12'hFFF; h3 <= 12'hFFF; filt <= 12'hFFF;
        end else begin
            s1 <= raw;                     // synchroniser, stage 1
            s2 <= s1;                      // synchroniser, stage 2
            h1 <= s2;                      // samples of history
            h2 <= h1;
            h3 <= h2;
            // per pin: accept a level only after 3 (filt_long: 4) consecutive identical samples
            if (filt_long) filt <= (s2 & h1 & h2 & h3) | (filt & ~(~s2 & ~h1 & ~h2 & ~h3));
            else           filt <= (s2 & h1 & h2)      | (filt & ~(~s2 & ~h1 & ~h2));
        end
    end

    wire [11:0] clean = filt_on ? filt : s2;

    // output-only pads read back their own (registered) value
    assign pin_in = {uo_out[7:4], clean};

    // ---------------------------------------------------------- output hold delay
    // a 15-stage delay line of {value, drive} for the one selected pad
    reg  [29:0] hdl;
    wire [1:0]  hnow = {pin_val[hold_pin], pin_drv[hold_pin]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hdl <= {15{2'b10}};             // released
        else        hdl <= {hdl[27:0], hnow};
    end
    wire [1:0]  hdel = hdl[(hold_n - 4'd1) * 2 +: 2];
    wire        hon  = (hold_n != 4'd0);
    wire [15:0] hmask = 16'd1 << hold_pin;
    wire [15:0] val_e = hon ? ((pin_val & ~hmask) | ({16{hdel[1]}} & hmask)) : pin_val;
    wire [15:0] drv_e = hon ? ((pin_drv & ~hmask) | ({16{hdel[0]}} & hmask)) : pin_drv;

    // ---------------------------------------------------------- outputs
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uio_out <= 8'hFF;
            uio_oe  <= 8'h00;              // bidirectional pads: released
            uo_out  <= 8'hF0;              // output-only pads high, SDO/READY low
        end else begin
            uio_out <= val_e[7:0];
            uio_oe  <= drv_e[7:0];
            for (k = 0; k < 4; k = k + 1)  // undriven output pads idle high
                uo_out[4 + k] <= drv_e[12 + k] ? val_e[12 + k] : 1'b1;
            uo_out[3:2] <= 2'b00;
            uo_out[1]   <= ready;
            uo_out[0]   <= sdo;
        end
    end

endmodule

`default_nettype wire
