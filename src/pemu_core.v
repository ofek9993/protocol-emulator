/*
 * pemu_core - the protocol emulator: a pool of timers, a pool of shifters,
 * a pin mux, a serial configuration port, the controller, and the pin layer.
 *
 * The pool is 2 timers + 2 shifters (NT / NS below). The design runs one
 * protocol at a time, and 2 + 2 covers UART full duplex, SPI with MISO, I2C,
 * JTAG IDCODE and SWD (refmodel/checks/JTAG_SWD_PAPER.md). Full JTAG would need a third
 * shifter: NS = 3, plus widening the 1-bit shifter-select fields.
 *
 * Nothing in here knows what a protocol is. Two ways to use it:
 *   * config-only: the host writes timer/shifter settings and feeds bytes by
 *     writing SHBUF directly;
 *   * controller mode: the host also loads a STATE TABLE into pemu_ctrl and
 *     says "go"; the controller then sequences the datapath by itself
 *     (chip-select, START/STOP, ACK checks, retries, timeouts).
 * See ARCHITECTURE.md.
 *
 * ------------------------------------------------------------------ pins
 * Tiny Tapeout standard pin map; everything protocol-facing goes through
 * pemu_pins (synchroniser, glitch filter, registered outputs).
 *
 *   ui_in[0]   CFG_SCK   config shift clock          (own synchroniser)
 *   ui_in[1]   CFG_SDI   config data in, MSB first
 *   ui_in[2]   CFG_CS    low while shifting; rising edge commits the write
 *   ui_in[6:3]           protocol inputs  = logical pins 8..11 (8 = UART RX)
 *   ui_in[7]             unused
 *   uio[7:0]             bidirectional    = logical pins 0..7
 *   uo_out[0]  CFG_SDO   config read-back data
 *   uo_out[1]  READY     the controller has finished ("done")
 *   uo_out[3:2]          0
 *   uo_out[7:4]          protocol outputs = logical pins 12..15 (12 = UART TX)
 *
 * --------------------------------------------------------- config port
 * A config write is 16 bits shifted in as {addr[7:0], data[7:0]}; it commits
 * on CS rising only if exactly 16 bits arrived (B7).
 *
 * READ-BACK: when CS falls, the byte selected by GOUT is copied into a read
 * register, and SDO shows its MSB. Each SCK rise moves the next bit onto SDO
 * (about 4 clocks later), so a host that samples SDO at the moment it raises
 * SCK - SPI mode 0 - reads the byte MSB first in the first 8 bits of the
 * word. A read that needs no write is simply an 8-bit word: it is too short
 * to commit, so B7 drops it. Reading the host RX FIFO and popping it can be
 * one word: write 0x04 while reading GOUT = 8 (the copy is taken at CS fall,
 * the pop happens at CS rise).
 *
 * -------------------------------------------------------- register map
 *   0x00  GCTL     [0] enable  [1] input glitch filter OFF (default on)
 *   0x01  GOUT     [3:0] which byte the read-back returns:
 *                    0..1 = shifter n received data
 *                    4    = status flags   (TX ready / RX byte available)
 *                    5    = overrun flags  (a write arrived while busy)
 *                    6    = framing errors (RX stop bit was wrong)
 *                    7    = timer active flags
 *                    8    = host RX FIFO: the oldest byte
 *                    9    = controller status {state[3:0], tx_full,
 *                           rx_empty, timed_out, done}
 *                    10   = controller errors {6'b0, rx_overflow, tx_underflow}
 *                           (sticky; CCTL[2] clears them)
 *                    11   = the controller's live counter (e.g. I2C: write
 *                           bytes NOT yet sent - after a slave's NACK the
 *                           host learns how far the write got)
 *                    others = 0
 *   0x02  CCTL     [0] controller run  [1] write 1 = "go" (a pulse)
 *                  [2] write 1 = clear done / underflow / overflow
 *                  [3] write 1 = flush both host FIFOs (e.g. the unsent
 *                      bytes left behind when a slave NACKs a write)
 *   0x03  HTX      push a byte into the host TX FIFO (4 deep)
 *   0x04  HRXPOP   any write: drop the oldest byte of the host RX FIFO
 *   0x05  CSEL     [0] the controller's TX shifter  [2] its RX shifter
 *   0x06  PHOLD    [3:0] a logical pad, [7:4] its output hold delay in
 *                  clocks (0 = off). I2C: SDA, 15 = 300 ns (AUDIT B24)
 *   0x08-0x0B CPMAP n: controller pin line n  [3:0] logical pin
 *                  [5:4] 0 read-only, 1 open-drain, 3 push-pull
 *   0x60-0x7F      the controller's own registers and table (pemu_ctrl.v)
 *
 *   timer n at 0x10 + n*8   (n = 0, 1):
 *     +0  TIMCTL   [7:6] timod  [5:4] timena  [3:2] timdis
 *                  [1] trgpol   [0] timout
 *     +1  TIMCMPL  timcmp[7:0]
 *     +2  TIMCMPH  timcmp[15:8]
 *     +3  TIMPIN   [7:6] trgsel  [5:2] pinsel  [1:0] pincfg
 *                  trgsel 0..1 = shifter n's status flag, 2..3 = timer n's output
 *     +4  TIMPOL   [0] pinpol  [1] decsrc
 *                  [2] ctrig    start on the controller's frame pulse
 *                  [3] startlow output goes low the instant it starts
 *                  [4] park     at stop, keep the last level
 *                  [5] waitpin  count the high phase from the REAL rise
 *     +5  TIMPRE   tick prescaler: the timer counts every (N+1)-th clock;
 *                  0 = every clock (AUDIT B28: UART below 97.6 kbaud, PS/2)
 *
 *   shifter n at 0x30 + n*8   (n = 0, 1):
 *     +0  SHCTL    [7:6] smod  [4] timsel  [3] timpol  [2] dir
 *                  [1:0] pincfg
 *     +1  SHCFG    [7:6] sstart  [5:4] sstop  [3:0] pinsel
 *     +2  SHLUTL   lut[7:0]     (reset 0x08 = identity)
 *     +3  SHLUTH   lut[12:8]    (reset 0x02 = identity)
 *     +4  SHBUF    write a byte here to transmit it
 *
 *   pincfg: 0 off, 1 open-drain, 2 bidirectional (today = push-pull), 3 push-pull
 *   pinsel: a LOGICAL pin 0..15 (see the pin table above)
 */

`default_nettype none

module pemu_core (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);

    localparam integer NT = 2;          // timers
    localparam integer NS = 2;          // shifters

    // ==================================================================
    // config port: 3 async inputs, synchronised then edge-detected
    // ==================================================================
    reg [1:0] sck_sync, sdi_sync, cs_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync <= 2'b00; sdi_sync <= 2'b00; cs_sync <= 2'b11;
        end else begin
            sck_sync <= {sck_sync[0], ui_in[0]};
            sdi_sync <= {sdi_sync[0], ui_in[1]};
            cs_sync  <= {cs_sync[0],  ui_in[2]};
        end
    end

    reg sck_d, cs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sck_d <= 1'b0; cs_d <= 1'b1; end
        else        begin sck_d <= sck_sync[1]; cs_d <= cs_sync[1]; end
    end

    wire sck_rise = sck_sync[1] & ~sck_d;
    wire cs_rise  = cs_sync[1]  & ~cs_d;
    wire cs_fall  = ~cs_sync[1] &  cs_d;

    // B7: count the bits. Without this, raising CS early committed whatever
    // leftover mix of old and new bits happened to be sitting in cfg_sr, to
    // an effectively random register address.
    reg [15:0] cfg_sr;
    reg [4:0]  cfg_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_sr  <= 16'd0;
            cfg_cnt <= 5'd0;
        end else if (cs_sync[1]) begin
            cfg_cnt <= 5'd0;              // CS high: idle, arm for next word
        end else if (sck_rise) begin
            cfg_sr <= {cfg_sr[14:0], sdi_sync[1]};
            if (cfg_cnt != 5'd31) cfg_cnt <= cfg_cnt + 5'd1;
        end
    end

    // commit only on a complete 16-bit word; short words are dropped
    wire       cfg_we   = cs_rise && (cfg_cnt == 5'd16);
    wire [7:0] cfg_addr = cfg_sr[15:8];
    wire [7:0] cfg_data = cfg_sr[7:0];

    // ==================================================================
    // configuration registers
    // ==================================================================
    reg [7:0] gctl, gout, phold;
    reg [7:0] tim_ctl  [0:NT-1];
    reg [7:0] tim_cmpl [0:NT-1];
    reg [7:0] tim_cmph [0:NT-1];
    reg [7:0] tim_pin  [0:NT-1];
    reg [7:0] tim_pol  [0:NT-1];
    reg [7:0] tim_pre  [0:NT-1];      // TIMPRE (timer + 5): tick prescaler, AUDIT B28
    reg [7:0] sh_ctl   [0:NS-1];
    reg [7:0] sh_cfg   [0:NS-1];
    reg [7:0] sh_lutl  [0:NS-1];
    reg [7:0] sh_luth  [0:NS-1];

    // writing SHBUF pulses the matching shifter's we for one clock
    reg [NS-1:0] sh_we;
    reg [7:0]    sh_wdata;

    integer n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gctl <= 8'h00;
            gout <= 8'h00;
            phold <= 8'h00;
            for (n = 0; n < NT; n = n + 1) begin
                tim_ctl [n] <= 8'h00;
                tim_cmpl[n] <= 8'h00;
                tim_cmph[n] <= 8'h00;
                tim_pin [n] <= 8'h00;
                tim_pol [n] <= 8'h00;
                tim_pre [n] <= 8'h00;
            end
            for (n = 0; n < NS; n = n + 1) begin
                sh_ctl  [n] <= 8'h00;
                sh_cfg  [n] <= 8'h00;
                sh_lutl [n] <= 8'h08;   // identity coding
                sh_luth [n] <= 8'h02;
            end
            sh_we    <= {NS{1'b0}};
            sh_wdata <= 8'd0;
        end else begin
            sh_we <= {NS{1'b0}};
            if (cfg_we) begin
                sh_wdata <= cfg_data;
                if (cfg_addr == 8'h00) gctl <= cfg_data;
                if (cfg_addr == 8'h01) gout <= cfg_data;
                if (cfg_addr == 8'h06) phold <= cfg_data;
                for (n = 0; n < NT; n = n + 1) begin
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 0) tim_ctl [n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 1) tim_cmpl[n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 2) tim_cmph[n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 3) tim_pin [n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 4) tim_pol [n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h10 + n*8 + 5) tim_pre [n] <= cfg_data;
                end
                for (n = 0; n < NS; n = n + 1) begin
                    if ({24'd0, cfg_addr} == 32'h30 + n*8 + 0) sh_ctl [n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h30 + n*8 + 1) sh_cfg [n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h30 + n*8 + 2) sh_lutl[n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h30 + n*8 + 3) sh_luth[n] <= cfg_data;
                    if ({24'd0, cfg_addr} == 32'h30 + n*8 + 4) sh_we[n]   <= 1'b1;
                end
            end
        end
    end

    wire enable  = gctl[0];
    wire filt_on = ~gctl[1];

    // ==================================================================
    // the pin layer's clean view of every logical pin
    // ==================================================================
    wire [15:0] pin_in;
    reg  [15:0] pin_val, pin_drv;

    // ==================================================================
    // the controller and its plumbing (all idle unless CCTL.run = 1)
    // ==================================================================
    reg        c_run, c_go, c_done, c_under, c_over;
    reg        c_tx, c_rx;            // which shifter is the controller's TX / RX
    reg [5:0]  cp_map [0:3];          // [3:0] pin, [5:4] 0 ro, 1 od, 3 pp
    reg [3:0]  cp_val;                // each controller pin line: 0 low, 1 released/high
    reg [NS-1:0] c_we;                // the controller writing a shifter's buffer
    reg [7:0]  c_wdata;
    reg        c_trig, c_release;     // one-clock pulses to the timers
    reg        c_abort;               // one-clock pulse: a timeout ended the transfer (AUDIT B27)
    reg        frame_done, rx_avail, rx_stat_d;
    reg        stop_ovr_en, stop_ovr_val, frame_pend;

    // host FIFOs, 4 bytes each
    reg [7:0] htx [0:3];
    reg [7:0] hrx [0:3];
    reg [2:0] htx_n, hrx_n;
    reg [1:0] htx_rd, htx_wr, hrx_rd, hrx_wr;

    // forward declarations of pool signals the controller uses
    wire [NT-1:0] timer_out, timer_active, timer_done, timer_eff;
    wire [NS-1:0] sh_status, sh_pin_out, sh_pin_oe;
    wire [NS-1:0] sh_overrun, sh_frame_err, sh_rx_last;
    wire [7:0]    sh_rdata [0:NS-1];

    wire [7:0] c_cmp, c_cnt;
    wire [3:0] c_state;
    wire       c_entering, c_timed_out;
    wire       a_pin, a_sh, a_frame, a_pull, a_push, a_done, a_tmo;
    wire [1:0] a_pin_sel, a_pin_op, a_sh_code, a_frame_code;

    // events from outside the controller, indexed by event code
    reg [31:0] ev_ext;
    integer q;
    always @* begin
        ev_ext = 32'd0;
        for (q = 0; q < 4; q = q + 1)
            ev_ext[2 + q] = pin_in[cp_map[q][3:0]];       // CP0-CP3: the (clean) wire
        ev_ext[6]  = frame_done;                          // SH0_DONE
        ev_ext[7]  = rx_avail;                            // SH1_DONE
        ev_ext[9]  = sh_rx_last[c_rx];                    // RX_LASTBIT (I2C: 1 = NACK)
        ev_ext[10] = (sh_rdata[c_rx] == c_cmp);           // RX_EQ
        ev_ext[17] = (htx_n != 3'd0);                     // HOST_DATA
        ev_ext[18] = sh_status[c_tx];                     // TX_EMPTY (buffer free)
        ev_ext[19] = (hrx_n <= 3'd2);                     // RX_ROOM: the RX FIFO can take 2 more
    end

    pemu_ctrl #(.ROWS(16), .SB(4), .DOORS(3)) u_ctrl (
        .clk(clk), .rst_n(rst_n), .run(c_run),
        .we(cfg_we && (cfg_addr[7:5] == 3'b011)), .waddr({3'b000, cfg_addr[4:0]}), .wdata(cfg_data),
        .go(c_go), .ev_ext(ev_ext),
        .a_pin(a_pin), .a_pin_sel(a_pin_sel), .a_pin_op(a_pin_op),
        .a_sh(a_sh), .a_sh_code(a_sh_code), .a_frame(a_frame), .a_frame_code(a_frame_code),
        .a_pull(a_pull), .a_push(a_push), .a_done(a_done), .a_tmo(a_tmo),
        .state(c_state), .entering_o(c_entering), .timed_out(c_timed_out), .cmp(c_cmp), .cnt_o(c_cnt)
    );

    // the controller's actions, turned into datapath and FIFO operations.
    // Its own loop variable: an integer shared by two always blocks is ONE
    // signal with two drivers to synthesis (Yosys: 32 conflicting drivers).
    integer m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_run <= 1'b0; c_go <= 1'b0; c_done <= 1'b0; c_under <= 1'b0; c_over <= 1'b0;
            c_tx <= 1'b0; c_rx <= 1'b0; cp_val <= 4'hF; c_we <= {NS{1'b0}}; c_wdata <= 8'd0;
            c_trig <= 1'b0; c_release <= 1'b0; c_abort <= 1'b0;
            frame_done <= 1'b0; rx_avail <= 1'b0; rx_stat_d <= 1'b1;
            stop_ovr_en <= 1'b0; stop_ovr_val <= 1'b1; frame_pend <= 1'b1;
            htx_n <= 3'd0; htx_rd <= 2'd0; htx_wr <= 2'd0;
            hrx_n <= 3'd0; hrx_rd <= 2'd0; hrx_wr <= 2'd0;
            for (m = 0; m < 4; m = m + 1) cp_map[m] <= 6'd0;
        end else begin
            c_go <= 1'b0; c_we <= {NS{1'b0}}; c_trig <= 1'b0; c_release <= 1'b0;
            c_abort <= a_tmo;                 // a timeout exit was taken: abort the datapath

            // ---- host side
            if (cfg_we) begin
                case (cfg_addr)
                    8'h02: begin
                        c_run <= cfg_data[0];
                        if (cfg_data[1]) begin c_go <= 1'b1; c_done <= 1'b0; end
                        if (cfg_data[2]) begin c_done <= 1'b0; c_under <= 1'b0; c_over <= 1'b0; end
                    end
                    8'h05: begin c_tx <= cfg_data[0]; c_rx <= cfg_data[2]; end
                    8'h08: cp_map[0] <= cfg_data[5:0];
                    8'h09: cp_map[1] <= cfg_data[5:0];
                    8'h0A: cp_map[2] <= cfg_data[5:0];
                    8'h0B: cp_map[3] <= cfg_data[5:0];
                    default: ;
                endcase
            end
            // FIFO bookkeeping: count changes are summed so a push and a pop
            // landing on the same clock are both honoured.
            begin : FIFOS
                reg tx_push, tx_pop, rx_push, rx_pop, flush;
                tx_push = cfg_we && (cfg_addr == 8'h03) && (htx_n != 3'd4);
                tx_pop  = a_pull && (htx_n != 3'd0);
                rx_push = a_push && (hrx_n != 3'd4);
                rx_pop  = cfg_we && (cfg_addr == 8'h04) && (hrx_n != 3'd0);
                if (tx_push) begin htx[htx_wr] <= cfg_data; htx_wr <= htx_wr + 2'd1; end
                if (tx_pop)  htx_rd <= htx_rd + 2'd1;
                htx_n <= htx_n + {2'd0, tx_push} - {2'd0, tx_pop};
                if (rx_push) begin hrx[hrx_wr] <= sh_rdata[c_rx]; hrx_wr <= hrx_wr + 2'd1; end
                if (rx_pop)  hrx_rd <= hrx_rd + 2'd1;
                hrx_n <= hrx_n + {2'd0, rx_push} - {2'd0, rx_pop};
                if (a_pull && htx_n == 3'd0) c_under <= 1'b1;
                if (a_push && hrx_n == 3'd4) c_over  <= 1'b1;
                flush = cfg_we && (cfg_addr == 8'h02) && cfg_data[3];
                if (flush) begin                                        // CCTL[3]: empty both FIFOs
                    htx_n <= 3'd0; htx_rd <= 2'd0; htx_wr <= 2'd0;
                    hrx_n <= 3'd0; hrx_rd <= 2'd0; hrx_wr <= 2'd0;
                end
            end

            // ---- controller actions
            if (a_pin) cp_val[a_pin_sel] <= (a_pin_op == 2'd1) ? 1'b0 : 1'b1;
            if (a_frame) frame_pend <= (a_frame_code == 2'd2);         // 1 = NACK
            if (a_pull && htx_n != 3'd0) begin                          // host byte -> TX buffer
                c_we[c_tx] <= 1'b1;
                c_wdata    <= htx[htx_rd];
            end
            if (a_sh && a_sh_code == 2'd1) begin                        // run a TX frame
                c_trig <= 1'b1; frame_done <= 1'b0; stop_ovr_en <= 1'b0;
            end
            if (a_sh && a_sh_code == 2'd2) begin                        // run an RX frame:
                c_we[c_tx]   <= 1'b1;                                   // release the data line
                c_wdata      <= 8'hFF;                                  // (send all ones)
                stop_ovr_en  <= 1'b1;                                   // and drive only the
                stop_ovr_val <= a_frame ? (a_frame_code == 2'd2)        // last bit: ACK / NACK
                                        : frame_pend;
                c_trig <= 1'b1; frame_done <= 1'b0;
            end
            if (a_sh && a_sh_code == 2'd3) c_release <= 1'b1;           // release a parked clock
            if (a_done) c_done <= 1'b1;

            // ---- events back to the controller
            if (timer_done[sh_ctl[c_tx][4]]) frame_done <= 1'b1;
            rx_stat_d <= sh_status[c_rx];
            if (a_push) rx_avail <= 1'b0;
            if (sh_status[c_rx] && !rx_stat_d) rx_avail <= 1'b1;        // a byte just arrived
        end
    end

    // ==================================================================
    // the pools
    // ==================================================================
    // each timer's trigger: shifter flags 0..1, then timer outputs 0..1
    wire [3:0] trg_bus = {timer_out, sh_status};

    genvar g;
    generate
        for (g = 0; g < NT; g = g + 1) begin : TIMERS
            wire [1:0] trgsel = tim_pin[g][7:6];
            wire [3:0] pinsel = tim_pin[g][5:2];
            pemu_timer u_t (
                .clk(clk), .rst_n(rst_n), .enable(enable),
                .timod  (tim_ctl[g][7:6]),
                .timcmp ({tim_cmph[g], tim_cmpl[g]}),
                .timena (tim_ctl[g][5:4]),
                .timdis (tim_ctl[g][3:2]),
                .timout (tim_ctl[g][0]),
                .trgpol (tim_ctl[g][1]),
                .pinpol (tim_pol[g][0]),
                .decsrc (tim_pol[g][1]),
                .prediv (tim_pre[g]),
                .ctrig  (tim_pol[g][2]),
                .startlow(tim_pol[g][3]),
                .park   (tim_pol[g][4]),
                .waitpin(tim_pol[g][5]),
                .ctrl_trig  (c_trig),
                .release_clk(c_release),
                .abort  (c_abort),
                .trigger_in(trg_bus[trgsel]),
                .pin_in (pin_in[pinsel]),
                .timer_out   (timer_out[g]),
                .timer_active(timer_active[g]),
                .timer_done  (timer_done[g]),
                .timer_eff   (timer_eff[g])
            );
        end

        for (g = 0; g < NS; g = g + 1) begin : SHIFTERS
            wire       timsel = sh_ctl[g][4];
            wire [3:0] pinsel = sh_cfg[g][3:0];
            // the controller's TX shifter may have its last bit (the I2C
            // ACK/NACK slot) overridden for a receive frame
            wire [1:0] sstop  = (stop_ovr_en && c_tx == g) ? (stop_ovr_val ? 2'd3 : 2'd2)
                                                           : sh_cfg[g][5:4];
            pemu_shifter u_s (
                .clk(clk), .rst_n(rst_n), .enable(enable),
                .smod   (sh_ctl[g][7:6]),
                .timpol (sh_ctl[g][3]),
                .sstart (sh_cfg[g][7:6]),
                .sstop  (sstop),
                .dir    (sh_ctl[g][2]),
                .lut    ({sh_luth[g][4:0], sh_lutl[g]}),
                .timer_out (timer_eff[timsel]),    // the clock as the wire sees it
                .timer_done(timer_done[timsel]),   // B3: was never connected
                .abort  (c_abort),
                .wdata  (c_we[g] ? c_wdata : sh_wdata),
                .we     (sh_we[g] | c_we[g]),
                .rdata  (sh_rdata[g]),
                .status_flag(sh_status[g]),
                .overrun    (sh_overrun[g]),
                .frame_err  (sh_frame_err[g]),
                .rx_lastbit (sh_rx_last[g]),
                .pin_out(sh_pin_out[g]),
                .pin_oe (sh_pin_oe[g]),
                .pin_in (pin_in[pinsel])
            );
        end
    endgenerate

    // ==================================================================
    // pin mux - over the 16 LOGICAL pins
    //
    // A pin is driven by whichever shifter or timer selects it and has a
    // non-zero pincfg. Shifters win over timers if both point at the same
    // pin, which is a configuration error rather than a hardware case.
    //
    // Controller pin lines come last. Open-drain ones are WIRED-AND with
    // whatever else drives the pin - they can only pull it low, never fight
    // it - so I2C's SDA can be both the shifter's data and the controller's
    // START/STOP. Push-pull controller lines own their pin (SPI chip-select).
    //
    // Open drain is expressed entirely in the drive enable: pin_drv = ~value,
    // so the pad is released for a 1 and pulled low for a 0. (An extra
    // "od_mask" on the value was found dead by mutation testing and removed.)
    // On an output-only pad (12..15) "released" reads as high.
    // ==================================================================
    integer p, k;
    always @* begin
        pin_val = 16'hFFFF;
        pin_drv = 16'h0000;
        for (p = 0; p < 16; p = p + 1) begin
            for (k = 0; k < NT; k = k + 1) begin
                // timers first, so a shifter on the same pin overrides
                if (tim_pin[k][1:0] != 2'd0 && tim_pin[k][5:2] == p[3:0]) begin
                    pin_val[p] = timer_out[k];
                    pin_drv[p] = (tim_pin[k][1:0] == 2'd1)
                                 ? ~timer_out[k]     // open drain
                                 : 1'b1;
                end
            end
            for (k = 0; k < NS; k = k + 1) begin
                if (sh_ctl[k][1:0] != 2'd0 && sh_cfg[k][3:0] == p[3:0]
                    && sh_pin_oe[k]) begin
                    pin_val[p] = sh_pin_out[k];
                    pin_drv[p] = (sh_ctl[k][1:0] == 2'd1)
                                 ? ~sh_pin_out[k]    // open drain
                                 : 1'b1;
                end
            end
            for (k = 0; k < 4; k = k + 1) begin
                if (cp_map[k][3:0] == p[3:0]) begin
                    if (cp_map[k][5:4] == 2'd1 && !cp_val[k]) begin   // open drain: pull low
                        pin_val[p] = 1'b0;
                        pin_drv[p] = 1'b1;
                    end else if (cp_map[k][5:4] == 2'd3) begin        // push-pull: own it
                        pin_val[p] = cp_val[k];
                        pin_drv[p] = 1'b1;
                    end
                end
            end
        end
    end

    // ==================================================================
    // read-back (B6 history: the old parallel window could not show every
    // shifter; the serial read-back can return any byte)
    // ==================================================================
    reg [7:0] rd_sel;
    always @* begin
        case (gout[3:0])
            4'd0:    rd_sel = sh_rdata[0];
            4'd1:    rd_sel = sh_rdata[1];
            4'd4:    rd_sel = {{(8-NS){1'b0}}, sh_status};
            4'd5:    rd_sel = {{(8-NS){1'b0}}, sh_overrun};
            4'd6:    rd_sel = {{(8-NS){1'b0}}, sh_frame_err};
            4'd7:    rd_sel = {{(8-NT){1'b0}}, timer_active};
            4'd8:    rd_sel = hrx[hrx_rd];
            4'd9:    rd_sel = {c_state, (htx_n == 3'd4), (hrx_n == 3'd0), c_timed_out, c_done};
            4'd10:   rd_sel = {6'd0, c_over, c_under};   // a pull from an empty TX FIFO / a push into a full RX FIFO
            4'd11:   rd_sel = c_cnt;                     // the controller's live counter
            default: rd_sel = 8'd0;
        endcase
    end

    reg [7:0] rd_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                        rd_sr <= 8'd0;
        else if (cs_fall)                  rd_sr <= rd_sel;              // snapshot
        else if (!cs_sync[1] && sck_rise)  rd_sr <= {rd_sr[6:0], 1'b0}; // next bit
    end
    wire sdo = !cs_sync[1] && rd_sr[7];

    // ==================================================================
    // the pin layer: the only place that touches the protocol pads
    // ==================================================================
    pemu_pins u_pins (
        .clk(clk), .rst_n(rst_n), .filt_on(filt_on),
        .uio_in(uio_in), .ui_proto(ui_in[6:3]),
        .uio_out(uio_out), .uio_oe(uio_oe), .uo_out(uo_out),
        .pin_in(pin_in), .pin_val(pin_val), .pin_drv(pin_drv),
        .sdo(sdo), .ready(c_done),
        .hold_pin(phold[3:0]), .hold_n(phold[7:4])
    );

    wire _unused = &{ui_in[7], c_entering, gctl[7:2], gout[7:4], 1'b0};   // reserved bits

endmodule

`default_nettype wire
