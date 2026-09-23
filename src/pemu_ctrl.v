/*
 * pemu_ctrl - the controller: a programmable state machine.
 *
 * Nothing in here knows what a protocol is. The protocol is the TABLE the
 * host loads: one row per state. This module only obeys it:
 *
 *   entering a state   -> fire that row's actions, once, and restart its
 *                         timeout counter
 *   every later clock  -> check the row's exits in order; the FIRST whose
 *                         event is true wins -> go to its next state.
 *                         If none is true and the row's timeout has run
 *                         out, take the timeout exit. Otherwise stay.
 *
 * One transition = one clock. See ARCHITECTURE.md §5.4 and
 * model/TABLE_FORMAT.md. The row layout here must match encode_row() in
 * model/pemu_model.py bit for bit - tb_pemu_ctrl.v replays the Python
 * model's recorded runs and checks this module does exactly the same.
 *
 * ---------------------------------------------------------------- row layout
 * LSB first:
 *   [1:0]   pin_sel   controller pin line CP0-CP3 (also the gated-delay pin)
 *   [3:2]   pin_op    0 none, 1 drive low, 2 release
 *   [5:4]   sh        0 none, 1 run TX frame, 2 run RX frame, 3 release parked clock
 *   [7:6]   dly       0 none, 1-3 load the delay counter from D1-D3
 *   [8]     dly_gate  the delay only counts while pin line pin_sel is high
 *   [10:9]  cnt       0 none, 1 load count A, 2 decrement, 3 load count B
 *   [12:11] flag      0 none, 1 set, 2 clear
 *   [14:13] host      0 none, 1 push RX byte, 2 pull host byte, 3 raise DONE
 *   [16:15] frame     0 none, 1 TX frame's last bit = 0, 2 = 1
 *   then DOORS x { ev[5] inv[1] push[1] nx[SB] }
 *   then { to_en[1] to_nx[SB] }
 *
 * ---------------------------------------------------- local register map
 *   0x00/01 D1 lo/hi   0x02/03 D2   0x04/05 D3   0x06/07 timeout length
 *   0x08 count A   0x09 count B   0x0A compare   0x0B live counter
 *   0x0C live flag (bit 0)   0x0D table row select
 *   0x10-0x1F byte n of the selected row (only as many as a row needs)
 *
 * Events: codes 0/1 (NEVER/ALWAYS), 13 DLY_DONE, 14 CNT_ZERO, 15 FLAG and
 * 16 HOST_GO are made in here; every other code comes in on ev_ext.
 */

`default_nettype none

module pemu_ctrl #(
    parameter integer ROWS  = 16,
    parameter integer SB    = 4,          // bits in a state number
    parameter integer DOORS = 3
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          run,             // 0: held in state 0, about to enter it

    input  wire          we,              // a config write for this module
    input  wire [7:0]    waddr,
    input  wire [7:0]    wdata,
    input  wire          go,              // 1-clock pulse: the host said "go"

    input  wire [31:0]   ev_ext,          // events from outside, indexed by code

    // action strobes, valid in the clock they fire; fields are 0 otherwise
    output reg           a_pin,
    output reg  [1:0]    a_pin_sel,
    output reg  [1:0]    a_pin_op,
    output reg           a_sh,
    output reg  [1:0]    a_sh_code,
    output reg           a_frame,
    output reg  [1:0]    a_frame_code,
    output reg           a_pull,
    output reg           a_push,
    output reg           a_done,
    output reg           a_tmo,           // a timeout exit is taken this clock

    output wire [SB-1:0] state,
    output wire          entering_o,
    output reg           timed_out,       // sticky
    output wire [7:0]    cmp,             // compare value, for the RX_EQ event
    output wire [7:0]    cnt_o            // the live counter, for host read-back
);

    localparam integer AW      = 17;                      // action bits
    localparam integer DW      = 7 + SB;                  // one door
    localparam integer TO_BASE = AW + DOORS * DW;
    localparam integer ROWW    = TO_BASE + 1 + SB;
    localparam integer NBYTES  = (ROWW + 7) / 8;
    localparam integer TBLW    = NBYTES * 8;

    localparam [4:0] E_CP0 = 5'd2, E_GO = 5'd16;

    // ---------------------------------------------------------- storage
    reg [TBLW-1:0] tbl [0:ROWS-1];
    reg [15:0] d1, d2, d3, tmo;
    reg [7:0]  cnt_a, cnt_b, cmp_r, row_sel;

    reg [SB-1:0] cur;
    reg          entering;
    reg [15:0]   dly, tcount;
    reg [7:0]    cnt;
    reg          flag, go_l;

    assign state      = cur;
    assign entering_o = entering;
    assign cmp        = cmp_r;
    assign cnt_o      = cnt;

    // ---------------------------------------------------------- the current row
    wire [ROWW-1:0] row = tbl[cur][ROWW-1:0];

    wire [1:0] f_pin_sel = row[1:0];
    wire [1:0] f_pin_op  = row[3:2];
    wire [1:0] f_sh      = row[5:4];
    wire [1:0] f_dly     = row[7:6];
    wire       f_gate    = row[8];
    wire [1:0] f_cnt     = row[10:9];
    wire [1:0] f_flag    = row[12:11];
    wire [1:0] f_host    = row[14:13];
    wire [1:0] f_frame   = row[16:15];
    wire          f_to_en = row[TO_BASE];
    wire [SB-1:0] f_to_nx = row[TO_BASE+1 +: SB];

    // ---------------------------------------------------------- the delay
    // It counts down every clock after entry - or, if gated, only while the
    // selected pin line is really high (stretch-safe "wait tSU after SCL
    // rises"). The exits see the value AFTER this clock's count.
    wire        gate_ok = !f_gate || ev_ext[E_CP0 + {3'd0, f_pin_sel}];
    wire [15:0] dly_nxt = (dly != 16'd0 && gate_ok) ? dly - 16'd1 : dly;

    // All 32 events as one vector, indexed by event code. The internal
    // events (NEVER, ALWAYS, DLY_DONE, CNT_ZERO, FLAG, HOST_GO) are made
    // here; every other code comes from ev_ext.
    //
    // This is deliberately a WIRE, not a function called from always @*: an
    // earlier version used a function, and @* does not re-run when signals
    // read INSIDE a function change - so a HOST_GO arriving later was never
    // seen. The replay bench caught it on the first clock.
    wire [31:0] ev_all = { ev_ext[31:17],
                           go_l,                    // 16 HOST_GO
                           flag,                    // 15 FLAG
                           (cnt == 8'd0),           // 14 CNT_ZERO
                           (dly_nxt == 16'd0),      // 13 DLY_DONE
                           ev_ext[12:2],
                           1'b1,                    //  1 ALWAYS
                           1'b0 };                  //  0 NEVER

    // ---------------------------------------------------------- the exits
    // First true door wins. Priority order is what gives "AND": a later door
    // is only reached when every earlier one was false.
    reg          taken, take_push, take_go;
    reg [SB-1:0] take_nx;
    integer      d;
    always @* begin
        taken = 1'b0; take_push = 1'b0; take_go = 1'b0; take_nx = {SB{1'b0}};
        for (d = 0; d < DOORS; d = d + 1) begin
            if (!taken && (ev_all[row[AW + d*DW +: 5]] ^ row[AW + d*DW + 5])) begin
                taken     = 1'b1;
                take_push = row[AW + d*DW + 6];
                take_go   = (row[AW + d*DW +: 5] == E_GO);
                take_nx   = row[AW + d*DW + 7 +: SB];
            end
        end
    end

    wire timeout_hit = f_to_en && ((tcount + 16'd1) >= tmo);

    // ---------------------------------------------------------- action strobes
    always @* begin
        a_pin = 1'b0; a_pin_sel = 2'd0; a_pin_op = 2'd0;
        a_sh = 1'b0;  a_sh_code = 2'd0; a_frame = 1'b0; a_frame_code = 2'd0;
        a_pull = 1'b0; a_push = 1'b0; a_done = 1'b0; a_tmo = 1'b0;
        if (run && entering) begin                        // entry: fire the row
            a_pin        = (f_pin_op != 2'd0);
            a_pin_sel    = a_pin ? f_pin_sel : 2'd0;
            a_pin_op     = f_pin_op;
            a_sh         = (f_sh != 2'd0);
            a_sh_code    = f_sh;
            a_frame      = (f_frame != 2'd0);
            a_frame_code = f_frame;
            a_pull       = (f_host == 2'd2);
            a_push       = (f_host == 2'd1);
            a_done       = (f_host == 2'd3);
        end else if (run) begin
            a_push = taken && take_push;                  // push-on-exit
            a_tmo  = !taken && timeout_hit;
        end
    end

    // ---------------------------------------------------------- state
    integer r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur <= {SB{1'b0}}; entering <= 1'b1;
            dly <= 16'd0; tcount <= 16'd0; cnt <= 8'd0; flag <= 1'b0; go_l <= 1'b0;
            timed_out <= 1'b0;
            d1 <= 16'd0; d2 <= 16'd0; d3 <= 16'd0; tmo <= 16'd0;
            cnt_a <= 8'd0; cnt_b <= 8'd0; cmp_r <= 8'd0; row_sel <= 8'd0;
            for (r = 0; r < ROWS; r = r + 1) tbl[r] <= {TBLW{1'b0}};
        end else begin
            // ---- host configuration writes
            if (we) begin
                case (waddr)
                    8'h00: d1[7:0]   <= wdata;   8'h01: d1[15:8]  <= wdata;
                    8'h02: d2[7:0]   <= wdata;   8'h03: d2[15:8]  <= wdata;
                    8'h04: d3[7:0]   <= wdata;   8'h05: d3[15:8]  <= wdata;
                    8'h06: tmo[7:0]  <= wdata;   8'h07: tmo[15:8] <= wdata;
                    8'h08: cnt_a <= wdata;       8'h09: cnt_b <= wdata;
                    8'h0A: cmp_r <= wdata;       8'h0B: cnt   <= wdata;
                    8'h0C: flag  <= wdata[0];    8'h0D: row_sel <= wdata;
                    default:
                        if (waddr[7:4] == 4'h1 && {28'd0, waddr[3:0]} < NBYTES && {24'd0, row_sel} < ROWS)
                            tbl[row_sel[SB-1:0]][waddr[3:0]*8 +: 8] <= wdata;
                endcase
            end

            if (go) begin                       // held until a HOST_GO exit takes it
                go_l      <= 1'b1;
                timed_out <= 1'b0;              // a new transfer starts clean
            end

            // ---- run the table
            if (!run) begin
                cur <= {SB{1'b0}}; entering <= 1'b1; tcount <= 16'd0; dly <= 16'd0;
            end else if (entering) begin
                entering <= 1'b0;
                tcount   <= 16'd0;
                case (f_dly)
                    2'd1: dly <= d1;
                    2'd2: dly <= d2;
                    2'd3: dly <= d3;
                    default: ;
                endcase
                case (f_cnt)
                    2'd1: cnt <= cnt_a;
                    2'd2: cnt <= (cnt != 8'd0) ? cnt - 8'd1 : 8'd0;
                    2'd3: cnt <= cnt_b;
                    default: ;
                endcase
                case (f_flag)
                    2'd1: flag <= 1'b1;
                    2'd2: flag <= 1'b0;
                    default: ;
                endcase
            end else begin
                dly <= dly_nxt;
                if (taken) begin
                    cur      <= take_nx;
                    entering <= 1'b1;
                    if (take_go) go_l <= 1'b0;
                end else begin
                    tcount <= tcount + 16'd1;
                    if (timeout_hit) begin
                        cur       <= f_to_nx;
                        entering  <= 1'b1;
                        timed_out <= 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
