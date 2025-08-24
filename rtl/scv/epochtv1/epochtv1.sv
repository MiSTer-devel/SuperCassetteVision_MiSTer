// Epoch TV-1 - a reasonably accurate implementation
//
// Copyright (c) 2024-2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// . https://github.com/mamedev/mame - src/mame/epoch/scv.cpp
// . https://forums.atariage.com/topic/130365-atari-7800-vs-epoch-super-cassette-vision/ - [takeda.txt]
// . http://takeda-toshiya.my.coocan.jp/scv/index.html
// . https://upsilandre.over-blog.com/2022/10/sprite-hardware-80-s-le-grand-comparatif.html


`timescale 1us / 1ns

import scv_pkg::*;

module epochtv1
  (
   input         CLK, // clock (XTAL * 2)
   input         CE, // pixel clock enable

   // ROM initialization
   input         ROMINIT_SEL_CHR,
   input [9:0]   ROMINIT_ADDR,
   input [7:0]   ROMINIT_DATA,
   input         ROMINIT_VALID,

   // Emulator configuration
   input         palette_t CFG_PALETTE,

   // CPU address / data bus
   input         CP1_POSEDGE, // for CPU bus timing
   input [12:0]  A,
   input [7:0]   DB_I,
   output [7:0]  DB_O,
   output        DB_OE,
   input         RDB,
   input         WRB,
   input         CSB,
   output        WAITB,
   output        SCPUB, // uPD1771C chip select

   // VRAM address / data bus
   output [10:0] VA,
   input [7:0]   VD_I,
   output [7:0]  VD_O,
   output        nVWE,
   output [1:0]  nVCS,

   // video output
   output        VBL,
   output        DE,
   output        HS,
   output        VS,
   output [23:0] RGB // {R,G,B}
   );


// Timing acquired by measuring actual hardware.
localparam [8:0] NUM_ROWS = 9'd263;
localparam [8:0] NUM_COLS = 9'd260;

// Actual render window (incl. overscan): 208 x 232
localparam [8:0] FIRST_ROW_RENDER = 9'd16;
localparam [8:0] LAST_ROW_RENDER = 9'd247;
localparam [8:0] FIRST_COL_RENDER = 9'd23;
localparam [8:0] LAST_COL_RENDER = 9'd230;
localparam [8:0] FIRST_ROW_VSYNC = 9'd257;
localparam [8:0] LAST_ROW_VSYNC = 9'd259;
localparam [8:0] FIRST_COL_HSYNC = 9'd240;
localparam [8:0] LAST_COL_HSYNC = 9'd259;

`ifdef EPOCHTV1_HIDE_OVERSCAN
// Visible window: 204 x 230 = (1,2)-(204,231)
//
// Hide render window edges, which would normally be hidden by
// overscan.
localparam [8:0] FIRST_ROW_VISIBLE = FIRST_ROW_RENDER + 'd2;
localparam [8:0] LAST_ROW_VISIBLE = LAST_ROW_RENDER;
localparam [8:0] FIRST_COL_VISIBLE = FIRST_COL_RENDER + 'd1;
localparam [8:0] LAST_COL_VISIBLE = LAST_COL_RENDER - 'd3;
`else
// Visible window: same as render window
localparam [8:0] FIRST_ROW_VISIBLE = FIRST_ROW_RENDER;
localparam [8:0] LAST_ROW_VISIBLE = LAST_ROW_RENDER;
localparam [8:0] FIRST_COL_VISIBLE = FIRST_COL_RENDER;
localparam [8:0] LAST_COL_VISIBLE = LAST_COL_RENDER;
`endif

`ifdef EPOCHTV1_BORDERS
// left/right borders
localparam [8:0] FIRST_COL_LEFT = FIRST_COL_VISIBLE - 1'd8;
localparam [8:0] LAST_COL_LEFT = FIRST_COL_VISIBLE - 1'd1;
localparam [8:0] FIRST_COL_RIGHT = LAST_COL_VISIBLE + 1'd1;
localparam [8:0] LAST_COL_RIGHT = LAST_COL_VISIBLE + 1'd8;
`endif


reg          ce2;
reg [8:0]    row_p, col_p, row, col;
wire         render_row, render_col, render_px;
wire         visible_row, visible_col, visible_px;
wire         cpu_sel_bgm, cpu_sel_oam, cpu_sel_vram, cpu_sel_reg, cpu_sel_apu;
wire         cpu_rd, cpu_wr, cpu_rdwr;
wire         vram_ce;
reg [11:0]   spr_vram_addr;
reg [7:0]    spr_vram_data;
wire         spr_vram_re_p;
wire         sofp_ce;

//////////////////////////////////////////////////////////////////////
// Control registers ($1400-$1403)

reg [7:0]    ioreg0_p, ioreg1_p, ioreg2_p, ioreg3_p; // shadow registers
reg [7:0]    ioreg0, ioreg1, ioreg2, ioreg3; // active registers

initial begin
  ioreg0_p = 0;
  ioreg1_p = 0;
  ioreg2_p = 0;
  ioreg3_p = 0;

  ioreg0 = 0;
  ioreg1 = 0;
  ioreg2 = 0;
  ioreg3 = 0;
end

always @(posedge CLK) begin
  if (cpu_sel_reg & cpu_wr) begin
    case (A[1:0])
      2'd0: ioreg0_p <= DB_I;
      2'd1: ioreg1_p <= DB_I;
      2'd2: ioreg2_p <= DB_I;
      2'd3: ioreg3_p <= DB_I;
    endcase
  end
end

// Copy from shadow to active registers at start of VSYNC.
always_ff @(posedge CLK) if (CE) begin
  if ((row == FIRST_ROW_VSYNC) & (col == 0)) begin
    ioreg0 <= ioreg0_p;
    ioreg1 <= ioreg1_p;
    ioreg2 <= ioreg2_p;
    ioreg3 <= ioreg3_p;
  end
end

// Handy aliases

wire         bm_ena = ioreg0[0];   // enable bitmap
wire         bm_lores = ioreg0[1]; // bitmap res: 0=lo, 1=hi
wire         sp_hide7 = ioreg0[2]; // hide sprites 64-127
wire         boc_dis = ioreg0_p[3];// disable OAM copy (use shadow)
wire         sp_ena = ioreg0[4];   // enable sprites
wire         sp_2clrm = ioreg0[5]; // 2-color sprite mode
wire         bm_invx = ioreg0[6];  // invert XMAX effect
wire         bm_invy = ioreg0[7];  // invert YMAX effect

// Bitmap FG/BG colors
wire [3:0]   bm_clr_bg = ioreg1[3:0];
wire [3:0]   bm_clr_fg = ioreg1[7:4]; // high-resolution mode only

// Character / graphics window split
wire [3:0]   bm_xmax = ioreg2[3:0];
wire [3:0]   bm_ymax = ioreg2[7:4];

// Character FG/BG colors
wire [3:0]   ch_clr_bg = ioreg3[3:0];
wire [3:0]   ch_clr_fg = ioreg3[7:4];


//////////////////////////////////////////////////////////////////////
// Internal clock generator

// Synthesize a second phase of the pixel clock.  Phase doesn't matter
// for our purposes.

initial
  ce2 = 0;

always @(posedge CLK) begin
  ce2 <= CE;
end


//////////////////////////////////////////////////////////////////////
// Video counter

initial begin
  row = 0;
  col = 0;
end

always_comb begin
  row_p = row;
  col_p = col;

  if (col == NUM_COLS - 1'd1) begin
    col_p = 0;
    if (row == NUM_ROWS - 1'd1) begin
      row_p = 0;
    end
    else begin
      row_p = row_p + 1'd1;
    end
  end
  else begin
    col_p = col_p + 1'd1;
  end
end

always_ff @(posedge CLK) if (CE) begin
  row <= row_p;
  col <= col_p;
end


//////////////////////////////////////////////////////////////////////
// Character pattern ROM (CHR)

reg [7:0] chr [1024];

wire [9:0] chr_a;
reg [7:0]  chr_rbuf;

always_ff @(posedge CLK) begin
  if (ROMINIT_SEL_CHR & ROMINIT_VALID) begin
    chr[ROMINIT_ADDR] <= ROMINIT_DATA;
  end
end

always_ff @(posedge CLK) begin
  chr_rbuf <= chr[chr_a];
end


//////////////////////////////////////////////////////////////////////
// Background memory (BGM)
//
// Separate ports for BGR and CPU allow both processes to access BGM
// in the same pixel clock cycle.

reg [31:0] bgm [128];

wire       bgm_sel_cpu;
reg        bgm_sel_cpu_d;
reg [6:0]  bgm_a;
wire [6:0] bgm_ra_bgr, bgm_a_cpu;
reg [31:0] bgm_rbuf, bgm_rbuf_bgr, bgm_rbuf_cpu;
reg [3:0]  bgm_we;
wire [3:0] bgm_we_cpu;
wire [31:0] bgm_wbuf;

always_ff @(posedge CLK) begin
  bgm_sel_cpu_d <= bgm_sel_cpu;
end

always @* begin
  if (bgm_sel_cpu) begin
    bgm_a = bgm_a_cpu;
    bgm_we = bgm_we_cpu;
  end
  else begin
    bgm_a = bgm_ra_bgr;
    bgm_we = '0;
  end
end

always_ff @(posedge CLK) begin
  bgm_rbuf <= bgm[bgm_a];
  for (int i = 0; i < 4; i++) begin
    if (bgm_we[i]) begin
      bgm[bgm_a][(i*8)+:8] <= bgm_wbuf[(i*8)+:8];
    end
  end
end

always_ff @(posedge CLK) begin
  if (bgm_sel_cpu_d)
    bgm_rbuf_cpu <= bgm_rbuf;
  else
    bgm_rbuf_bgr <= bgm_rbuf;
end


//////////////////////////////////////////////////////////////////////
// Sprite attribute memory, shadow (OAM)

reg [31:0] oam [128];

wire [6:0] oam_a;
wire       oam_a_sel_cpu;
wire [6:0] oam_ra;
reg [31:0] oam_rbuf;
wire [3:0] oam_we;
wire [31:0] oam_wbuf;

assign oam_a = oam_a_sel_cpu ? A[8:2] : oam_ra;
assign oam_wbuf = {4{DB_I}};
assign oam_we = {3'b0, (cpu_sel_oam & cpu_wr)} << A[1:0];

always_ff @(posedge CLK) begin
  oam_rbuf <= oam[oam_a];
  for (int i = 0; i < 4; i++) begin
    if (oam_we[i]) begin
      oam[oam_a][(i*8)+:8] <= oam_wbuf[(i*8)+:8];
    end
  end
end


//////////////////////////////////////////////////////////////////////
// Sprite attribute memory, active (OAM2)

reg [31:0] oam2 [128];

wire [6:0] oam2_ra;
wire       oam2_re;
reg [31:0] oam2_rbuf, oam2_rd;
wire       oam2_wpsel;
wire [6:0] oam2_wa, oam2_wa1, oam2_wa2;
wire       oam2_we, oam2_we1, oam2_we2;
wire [31:0] oam2_wbuf, oam2_wbuf1, oam2_wbuf2;

assign oam2_wa = oam2_wpsel ? oam2_wa2 : oam2_wa1;
assign oam2_we = oam2_wpsel ? oam2_we2 : oam2_we1;
assign oam2_wbuf = oam2_wpsel ? oam2_wbuf2 : oam2_wbuf1;

always_ff @(posedge CLK) begin
  oam2_rbuf <= oam2[oam2_ra];
  if (oam2_we) begin
    oam2[oam2_wa] <= oam2_wbuf;
  end
end

always_ff @(posedge CLK) begin
  if (oam2_re)
    oam2_rd <= oam2_rbuf;
end


//////////////////////////////////////////////////////////////////////
// OAM copier
//
// Copies OAM from shadow to active. Copy starts in VBL and runs to
// completion.

// TODO: Reduce copy speed to match HW

reg [6:0] boc_idx;
reg       boc_active;
wire      boc_we;

initial begin
  boc_active = 0;
end

always_ff @(posedge CLK) if (CE) begin
  if (~boc_active) begin
    if ((row == FIRST_ROW_VSYNC) & (col == 0)) begin
      boc_active <= ~boc_dis;
      boc_idx <= 0;
    end
  end
  else begin
    if (boc_idx == '1) begin
      boc_active <= 0;
    end
    else begin
      boc_idx <= boc_idx + 1'd1;
    end
  end
end

assign boc_we = CE & boc_active;

assign oam_ra = boc_idx;
assign oam2_wpsel = ~boc_active;
assign oam2_wbuf1 = oam_rbuf;
assign oam2_wa1 = boc_idx;
assign oam2_we1 = boc_we;


//////////////////////////////////////////////////////////////////////
// CPU address / data bus interface

reg [7:0] cpu_do;
reg       cpu_csb_d;
reg       cpu_sel;

always_ff @(posedge CLK) if (vram_ce) begin
  cpu_csb_d <= CSB;
  cpu_sel <= (cpu_sel | ~cpu_csb_d) & ~CSB;
end

// Address decoder
assign cpu_sel_vram = ~CSB & (A[12] == 1'b0);     // $0000 - $0FFF
assign cpu_sel_bgm = ~CSB & (A[12:9] == 4'b1000); // $1000 - $11FF
assign cpu_sel_oam = ~CSB & (A[12:9] == 4'b1001); // $1200 - $13FF
assign cpu_sel_reg = ~CSB & (A[12:9] == 4'b1010); // $1400 - $15FF
assign cpu_sel_apu = ~CSB & (A[12:9] == 4'b1011); // $1600 - $17FF

assign cpu_rd = ~(CSB | RDB);
assign cpu_wr = ~(CSB | WRB);
assign cpu_rdwr = cpu_rd | cpu_wr;

always_ff @(posedge CLK) if (CE) begin
  if (cpu_rd) begin
    if (cpu_sel_vram)
      cpu_do <= VD_I;
    else if (cpu_sel_bgm)
      cpu_do <= bgm_rbuf_cpu[(A[1:0]*8)+:8];
    else if (cpu_sel_oam)
      cpu_do <= oam_rbuf[(A[1:0]*8)+:8];
    else if (cpu_sel_reg)
      cpu_do <= 8'hFF;
  end
end

assign DB_O = DB_OE ? cpu_do : 8'hzz;
assign DB_OE = cpu_rd;
assign SCPUB = ~cpu_sel_apu;
// This seems to reproduce WAITB's behavior of asserting on CSB and de-asserting on VRAM bus cycle.
assign WAITB = CSB ^ cpu_sel;

assign bgm_sel_cpu = cpu_sel_bgm & cpu_rdwr & ce2;
assign bgm_a_cpu = A[8:2];
assign bgm_we_cpu = {3'b0, cpu_wr} << A[1:0];
assign bgm_wbuf = {4{DB_I}};

assign oam_a_sel_cpu = cpu_sel_oam & cpu_rdwr;


//////////////////////////////////////////////////////////////////////
// VRAM address / data bus interface

wire [11:0] va;
reg [11:0]  vram_cpu_addr;
reg [7:0]   vram_cpu_data;
reg         vram_cpu_sel_vram_d;
reg         vram_wr;

// VRAM bus cycle rate is 1/2 pixel clock, phase is odd columns (output)
assign vram_ce = CE & col[0];

always @(posedge CLK) if (vram_ce) begin
  vram_cpu_sel_vram_d <= cpu_sel_vram;
  vram_cpu_addr <= A[11:0];
  vram_cpu_data <= DB_I;
  vram_wr <= cpu_wr;
end

wire vram_sel_cpu = cpu_sel & vram_cpu_sel_vram_d;

assign va = vram_sel_cpu ? vram_cpu_addr : spr_vram_addr;

always @*
  spr_vram_data = vram_sel_cpu ? '0 : VD_I;

assign VA = va[10:0];
assign VD_O = vram_wr ? vram_cpu_data : 8'hzz;
assign nVWE = ~(vram_sel_cpu & vram_wr);
assign nVCS[0] = va[11];
assign nVCS[1] = ~va[11];


//////////////////////////////////////////////////////////////////////
// Background (character / bitmap) pipeline

wire [8:0] bgr_row, bgr_col;
wire [4:0] bgr_tx;
wire [4:0] bgr_ty;
wire       bgr_tce;
wire       bgr_xwin, bgr_ywin;
wire       bgr_bm;
wire       bgr_ch;
wire [7:0] bgr_bgm_rd;

wire [3:0] bgr_ch_bgc, bgr_ch_fgc;
reg [7:0]  bgr_ch_pat;

reg [3:0]  bgr_bm_bgc, bgr_bm_fgc;
reg [7:0]  bgr_bm_pat;

reg [3:0]  bgr_bgc, bgr_fgc;
reg [7:0]  bgr_pat, bgr_shift;
reg [3:0]  bgr_px;

assign bgr_row = row;
assign bgr_col = col;

assign bgr_tx = bgr_col[7:3];
assign bgr_ty = bgr_row[7:3];
assign bgr_tce = bgr_col[2:0] == 3'd4;

assign bgr_xwin = (bgr_tx[4:1] < bm_xmax) ^ bm_invx;
assign bgr_ywin = (bgr_ty[4:1] < bm_ymax) ^ bm_invy;

assign bgr_bm = bm_ena & ~bgr_ch;
assign bgr_ch = bgr_xwin & bgr_ywin;

// Read data from BGM
assign bgm_ra_bgr = {bgr_ty[4:1], bgr_tx[4:2]};
assign bgr_bgm_rd = bgm_rbuf_bgr[(bgr_tx[1:0]*8)+:8];

// Read character pattern from ROM
assign chr_a = {bgr_bgm_rd[6:0], bgr_row[2:0]};
assign bgr_ch_bgc = ch_clr_bg;
assign bgr_ch_fgc = ch_clr_fg;
assign bgr_ch_pat = bgr_ty[0] ? 0 : {2'b00, chr_rbuf[7:2]};

// Interpret BGM data as bitmap data
wire [2:0] bgr_bm_hipat_sel = {~bgr_row[3:2], 1'b0};
wire [2:0] bgr_bm_lopat_sel = {~bgr_row[3], 2'b0};
wire [1:0] bgr_bm_hipat = bgr_bgm_rd[bgr_bm_hipat_sel+:2];
wire [3:0] bgr_bm_lopat = bgr_bgm_rd[bgr_bm_lopat_sel+:4];

always @* begin
  bgr_bm_bgc = bm_clr_bg;
  bgr_bm_fgc = bm_clr_fg;
  bgr_bm_pat = 0;
  if (bgr_bm) begin
    if (bm_lores) begin
      if (bgr_bm_lopat != 4'd0) begin // 0 is transparent
        bgr_bm_pat = '1;
        bgr_bm_fgc = bgr_bm_lopat;
      end
    end
    else
      bgr_bm_pat = {{4{bgr_bm_hipat[1]}}, {4{bgr_bm_hipat[0]}}};
  end
end

// Background patterns are reversed
always @* begin
  for (int i = 0; i < 8; i++)
    bgr_pat[7-i] = bgr_ch ? bgr_ch_pat[i] : bgr_bm_pat[i];
end

always @(posedge CLK) if (CE) begin
  if (bgr_tce) begin
    bgr_bgc <= bgr_ch ? bgr_ch_bgc : bgr_bm_bgc;
    bgr_fgc <= bgr_ch ? bgr_ch_fgc : bgr_bm_fgc;
    bgr_shift <= bgr_pat;
  end
  else
    bgr_shift <= {1'b0, bgr_shift[7:1]};
end

always @* begin
  bgr_px = bgr_shift[0] ? bgr_fgc : bgr_bgc;
end


//////////////////////////////////////////////////////////////////////
// Sprite pipeline

typedef struct packed
{
    reg         split;
    reg [6:0]   tile;
    reg [6:0]   x;
    reg         link_x;
    reg [3:0]   start_line;
    reg [3:0]   color;
    reg [6:0]   y;
    reg         link_y;
} s_objattr;

reg [11:0]  spr_vram_addr_d;
reg         spr_vram_re;
wire        spr_rw2;            // reading 2nd half of double-wide
wire        spr_rl, spr_rr;     // reading left/right half (of 16-px pat.)
reg         spr_rnc;            // read nibble counter

reg [6:0]   spr_tile;
reg [7:0]   spr_pat;
reg [6:0]   oam_idx;
s_objattr   spr_oa, spr_oawb;
wire [6:0]  spr_cy;             // current render row
wire        spr_half_w, spr_half_h;
wire        spr_dbl_w, spr_dbl_h;
reg [4:0]   spr_x, spr_y, spr_w, spr_h;
wire        spr_2clr_en;        // 2-color sprite enable (if link_x/y)
wire        spr_2clr;           // 2-color sprite enabled and link_x/y
wire        spr_2halves;        // double-wide or 2-color (actual)
reg [3:0]   spr_color;
wire        spr_y_in_range;
wire        spr_visible;

wire        spr_d0;             // drawing start
wire        spr_d;              // drawing
wire        spr_dw2;            // drawing 2nd half of double-wide or 2-color
wire        spr_dh2;            // drawing bottom half of double-high
wire        spr_skip_dl;        // skip drawing left half
wire        spr_skip_dt;        // skip drawing top half
reg [7:0]   spr_olb_we;

reg         spr_dact;
reg [7:0]   spr_dpat;
reg         spr_dact_d, spr_dact_d2;
reg [15:0]  spr_dsr;            // draw shift register
reg         spr_dnc;            // drawing nibble (4 px) counter
reg [7:0]   spr_dsx;            // current drawing column
reg [3:0]   spr_dclr;           // current sprite color

// TODO: On HW, the VRAM accesses on SST_FETCH and the next cycle have
// the same address.

initial begin
  spr_vram_re = 0;
  spr_rnc = 0;
end

always @* begin
  spr_vram_addr = spr_vram_addr_d;
  if (spr_vram_re_p & spr_vram_re)
    spr_vram_addr = {spr_tile, spr_y[3:1], spr_x[3:2]};
end

always_ff @(posedge CLK) if (sofp_ce) begin
  if (spr_vram_re_p)
    spr_rnc <= spr_vram_re & ~spr_rnc;
  if (spr_vram_re)
    spr_pat <= spr_vram_data;

  spr_vram_re <= spr_vram_re_p;
  spr_vram_addr_d <= spr_vram_addr;
end

assign oam2_ra = oam_idx;
assign oam2_re = sofp_st == SST_EVAL;
assign spr_oa = oam2_rd;

assign spr_half_w = spr_oa.split;
assign spr_half_h = spr_oa.split & spr_oa.tile[6];
assign spr_dbl_w = ~(spr_half_w | spr_2clr_en) & spr_oa.link_x;
assign spr_dbl_h = ~(spr_half_h | spr_2clr_en) & spr_oa.link_y;
assign spr_2clr_en = sp_2clrm & oam_idx[5];
assign spr_2clr = spr_2clr_en & ~spr_half_w & (spr_oa.link_x | spr_oa.link_y);
assign spr_2halves = spr_dbl_w | spr_2clr;

assign spr_w = spr_half_w ? 5'd7 : spr_dbl_w ? 5'd31 : 5'd15;
assign spr_h = spr_half_h ? 5'd7 : spr_dbl_h ? 5'd31 : 5'd15;
assign spr_cy = spr_oa.y + {3'd0, spr_oa.start_line};
assign spr_y_in_range = row[7:1] == spr_cy;
assign spr_visible = |spr_oa.color & spr_y_in_range;

assign spr_skip_dl = spr_half_w & spr_oa.link_x;
assign spr_skip_dt = spr_half_h & spr_oa.link_y;

always @* begin
  spr_tile = spr_oa.tile;
  if (spr_2clr_en & spr_rw2)
    spr_tile = spr_tile ^ {3'b0, spr_oa.link_x, 2'b0, spr_oa.link_y};
  else if (~spr_2clr_en)
    spr_tile = spr_tile | {3'b0, spr_rw2, 2'b0, spr_dh2};
end

function [3:0] spr_2clr_gen(reg [3:0] color, reg link_x, reg link_y);
reg [3:0] out;
  begin
    out[0] = color[0];
    case ({link_y, link_x})
      2'b00:    out[3:1] = color[3:1];
      2'b01:    out[3:1] = {color[1], color[3], color[2]};
      2'b10:    out[3:1] = {color[2], color[1], color[3]};
      2'b11:    out[3:1] = ~color[3:1];
      default:  out[3:1] = 'X;
    endcase
    spr_2clr_gen = out;
  end
endfunction

always @* begin
  spr_color = spr_oa.color;
  if (spr_2clr_en & spr_dw2) begin
    spr_color = spr_2clr_gen(spr_color, spr_oa.link_x, spr_oa.link_y);
  end
end

always_comb begin
  spr_x = 0;
  if (spr_rnc)
    spr_x += 4'd4;
  if (spr_rr | spr_skip_dl)
    spr_x += 4'd8;
end

always @* begin
  spr_y = spr_oa.start_line * 2;
  if (spr_skip_dt)
    spr_y += 4'd8;
end
assign spr_dh2 = spr_y_in_range & spr_dbl_h & spr_y[4];

always @* begin
  spr_dpat = 0;
  spr_dact = 0;
  if (spr_d) begin
    spr_dact = 1'b1;
    for (int i = 0; i < 8; i++) begin
      spr_dpat[i] = spr_pat[3'd7 - i[2:0]];
    end
  end
end

always_ff @(posedge CLK) if (sofp_ce) begin
  spr_dsr <= {spr_dpat, spr_dsr[15:8]};

  spr_dnc <= spr_rnc;

  spr_dact_d <= spr_dact;
  if (spr_d0) begin
    spr_dsx <= spr_oa.x*2;
    spr_dclr <= spr_color;
  end
  else if (spr_dact_d) begin
    spr_dsx <= spr_dsx + 8'd4;
  end

  spr_dact_d2 <= spr_dact_d;
end

function is_dsr_set(reg [15:0] dsr, int off, reg [1:0] x0, reg y);
reg [4:0] p;
  begin
    p = 5'd4 + off[4:0] - 5'(x0);
    p = {p[3:2], y, p[1:0]};    // y selects the nibble
    is_dsr_set = dsr[4'(p)];
  end
endfunction

always @* begin
  spr_olb_we = 0;
  if (spr_dact_d | spr_dact_d2) begin
    for (int i = 0; i < 4; i++) begin
      for (int y = 0; y < 2; y++)
        spr_olb_we[{i[1:0], y[0]}] = is_dsr_set(spr_dsr, i, spr_dsx[1:0], y[0]);
    end
  end
end

// Update start line
always @* begin
  spr_oawb = spr_oa;
  spr_oawb.start_line += 1'd1;
  if (spr_oawb.start_line > spr_h[4:1])
    spr_oawb.start_line = 0;
end

assign oam2_wa2 = oam_idx;
assign oam2_we2 = oam2_re & sofp_ce & spr_visible;
assign oam2_wbuf2 = spr_oawb;


//////////////////////////////////////////////////////////////////////
// Object Line Buffer (OLB)
// - 8 pixels wide to enable writing VRAM fetch (4x2 pixels) in one cycle
// - pixel = 4 bit color
// - two pairs of full rows, used in ping-pong fashion

reg [6:0]   olb_wa;
reg [31:0]  olb_wd;
reg [7:0]   olb_we;
wire [6:0]  olb_ra;
reg [31:0]  olb_rd;
wire        olb_re;
genvar      olb_gi;

reg [31:0]  olb_rbuf [2];

// Declare one array per 2-row. Each array should infer a simple
// dual-port RAM.
generate
  for (olb_gi = 0; olb_gi < 2; olb_gi++) begin :olb_row

  reg [31:0] mem [64];
  reg [5:0]  addr;
  reg [31:0] wbuf;
  reg [7:0]  we;

    always_ff @(posedge CLK) begin
      olb_rbuf[olb_gi] <= mem[addr];
      for (int i = 0; i < 8; i++) begin
        if (we[i]) begin
          mem[addr][(i*4)+:4] <= wbuf[(i*4)+:4];
        end
      end
    end

    always @* begin
      if (olb_wa[6] == olb_gi[0]) begin
        // This 2-row is being written to.
        addr = olb_wa[5:0];
        wbuf = olb_wd;
        we = olb_we;
      end
      else /*if (olb_ra[6] == olb_gi[0])*/ begin
        // This 2-row is being read from.
        // TODO: Clear after reading
        addr = olb_ra[5:0];
        wbuf = 0;
        we = {8{CE & row[0]}}; //XXX
      end
    end
  end
endgenerate

always_ff @(posedge CLK) if (CE) begin
  if (olb_re) begin
    olb_rd <= olb_rbuf[olb_ra[6]]; // select read 2-row
  end
end


//////////////////////////////////////////////////////////////////////
// Sprite OLB fill pipeline

typedef enum reg [2:0]
{
 SST_IDLE,
 SST_EVAL,
 SST_FETCH,
 SST_DRAW_L,
 SST_DRAW_R,
 SST_2CLR_FLUSH,
 SST_DRAW_L2,
 SST_DRAW_R2
} e_sofp_st;

e_sofp_st sofp_st, sofp_st_next;

wire sofp_row;
wire sofp_row_start;
wire sofp_row_end;

wire [6:0] sofp_oam_idx_max;

wire sofp_wsel;

reg [3:0]  sofp_wdc_bg, sofp_wdc_fg;
reg [7:0]  sofp_wds;

assign sofp_ce = vram_ce;

assign sofp_oam_idx_max = sp_hide7 ? 7'd63 : 7'd127;

assign sofp_row = (row_p >= 9'd2) & (row_p <= 9'd253);
assign sofp_row_start = sofp_row & (col_p == 0) & ~row_p[0];
assign sofp_row_end = sofp_row & (col_p >= 9'd241) & row_p[0];

initial begin
  sofp_st = SST_IDLE;
  oam_idx = 0;
  spr_dnc = 0;
end

always @* begin
  sofp_st_next = sofp_st;
  if (~sofp_row | sofp_row_end) begin
    sofp_st_next = SST_IDLE;
  end
  else if (sofp_row_start) begin
    sofp_st_next = e_sofp_st'(sp_ena ? SST_EVAL : SST_IDLE);
  end
  else begin
    if (sofp_st == SST_IDLE) begin
    end
    else if (sofp_st == SST_EVAL) begin
      if (spr_visible) begin
        sofp_st_next = SST_FETCH;
      end
      else begin
        sofp_st_next = e_sofp_st'((oam_idx < sofp_oam_idx_max) ? SST_EVAL : SST_IDLE);
      end
    end
    else if (sofp_st == SST_FETCH) begin
      sofp_st_next = SST_DRAW_L;
    end
    else begin
      if (spr_dnc) begin
        if ((sofp_st == SST_DRAW_L) & ~spr_half_w) begin
          sofp_st_next = SST_DRAW_R;
        end
        else if (((sofp_st == SST_DRAW_L) | (sofp_st == SST_DRAW_R)) &
                 spr_2clr) begin
          // TODO: HW does not exhibit this state. No extra VRAM bus
          // cycles seen on 2-color sprites. Hide it somehow.
          sofp_st_next = SST_2CLR_FLUSH;
        end
        else if (((sofp_st == SST_DRAW_L) | (sofp_st == SST_DRAW_R) |
                  (sofp_st == SST_2CLR_FLUSH)) & spr_2halves) begin
          sofp_st_next = SST_DRAW_L2;
        end
        else if ((sofp_st == SST_DRAW_L2) & ~spr_half_w) begin
          sofp_st_next = SST_DRAW_R2;
        end
        else if (spr_d) begin
          sofp_st_next = e_sofp_st'((oam_idx < 7'd127) ? SST_EVAL : SST_IDLE);
        end
      end
    end
  end
end

always_ff @(posedge CLK) if (sofp_ce) begin
  sofp_st <= sofp_st_next;

  if (sofp_row_start) begin
    oam_idx <= 0;
  end
  else if ((sofp_st >= SST_EVAL) & (sofp_st_next <= SST_EVAL)) begin
    oam_idx <= oam_idx + 1'd1;
  end
end

assign spr_vram_re_p = (sofp_st_next >= SST_FETCH);
assign spr_rw2 = (sofp_st_next == SST_DRAW_L2) | (sofp_st_next == SST_DRAW_R2);
assign spr_rl = (sofp_st_next == SST_DRAW_L) | (sofp_st_next == SST_DRAW_L2);
assign spr_rr = (sofp_st_next == SST_DRAW_R) | (sofp_st_next == SST_DRAW_R2);

assign spr_d0 = ((sofp_st == SST_DRAW_L) | (spr_2clr & (sofp_st == SST_DRAW_L2))) &
                ~spr_dnc;
assign spr_dw2 = (sofp_st == SST_DRAW_L2) | (sofp_st == SST_DRAW_R2);
assign spr_d = (sofp_st == SST_DRAW_L) | (sofp_st == SST_DRAW_R) | spr_dw2;

assign sofp_wsel = ~row[1];

always @* begin
  olb_wa[6] = sofp_wsel;
  olb_wa[5:0] = spr_dsx[7:2];
  sofp_wdc_bg = spr_dclr;
  sofp_wdc_fg = spr_dclr;
  sofp_wds = 0;
  olb_we = spr_olb_we;
end

always @* begin
  for (int i = 0; i < 8; i++) begin
    olb_wd[(i*4)+:4] = sofp_wds[i] ? sofp_wdc_fg : sofp_wdc_bg;
  end
end

wire [7:0] sofp_rx;
reg [2:0]  sofp_rrs;
wire       sofp_rsel;
wire [3:0] sofp_px;

assign sofp_rsel = row[1];
assign sofp_rx = col[7:0];

assign olb_ra = {sofp_rsel, sofp_rx[7:2]};
assign olb_re = ~|sofp_rx[1:0];

always_ff @(posedge CLK) if (CE) begin
  sofp_rrs <= {sofp_rx[1:0], row[0]};
end

assign sofp_px = olb_rd[(sofp_rrs*4)+:4];


//////////////////////////////////////////////////////////////////////
// Sync generator

reg  de, hsync, vsync, vbl;
reg  de_p;

always_comb begin
  // Enable DE for render region...
  de_p = render_px;
`ifdef EPOCHTV1_BORDERS
  // plus right border...
  if (render_row)
    de_p = de_p | ((col >= FIRST_COL_RIGHT) & (col <= LAST_COL_RIGHT));
  // plus left border.
  if (render_row)
    de_p = de_p | ((col >= FIRST_COL_LEFT) & (col <= LAST_COL_LEFT));
`endif
end

always_ff @(posedge CLK) if (CE) begin
  de <= de_p;
/* verilator lint_off UNSIGNED */
  hsync <= (col >= FIRST_COL_HSYNC) & (col <= LAST_COL_HSYNC);
/* verilator lint_on UNSIGNED */
  vsync <= (row >= FIRST_ROW_VSYNC) & (row <= LAST_ROW_VSYNC);
  vbl <= ~render_row;
end

assign VBL = vbl;
assign DE = de;
assign HS = hsync;
assign VS = vsync;


//////////////////////////////////////////////////////////////////////
// Render pipeline

reg [3:0] pd;
reg       render_visible;

always_ff @(posedge CLK) if (CE) begin
  render_visible <= visible_px;
end

always @* begin
  pd = 4'd1; // black borders
  if (render_visible) begin
    pd = |sofp_px ? sofp_px : bgr_px;
  end
end

assign render_row = (row >= FIRST_ROW_RENDER) & (row <= LAST_ROW_RENDER);
assign render_col = (col >= FIRST_COL_RENDER) & (col <= LAST_COL_RENDER);
assign render_px = render_row & render_col;

assign visible_row = (row >= FIRST_ROW_VISIBLE) & (row <= LAST_ROW_VISIBLE);
assign visible_col = (col >= FIRST_COL_VISIBLE) & (col <= LAST_COL_VISIBLE);
assign visible_px = visible_row & visible_col;


//////////////////////////////////////////////////////////////////////
// Color generator

reg [23:0] cg;

always @* begin
  cg = 'X;
  if (CFG_PALETTE == PALETTE_RGB) begin
    // RGB connector signal level is observed to fall off as R+G+B increases.
    case (pd)
	  4'd0 : cg = { 8'd0  , 8'd0  , 8'd160 };
	  4'd1 : cg = { 8'd0  , 8'd0  , 8'd0   };
	  4'd2 : cg = { 8'd0  , 8'd0  , 8'd245 };
	  4'd3 : cg = { 8'd160, 8'd0  , 8'd235 };
	  4'd4 : cg = { 8'd0  , 8'd245, 8'd0   };
	  4'd5 : cg = { 8'd150, 8'd235, 8'd150 };
	  4'd6 : cg = { 8'd0  , 8'd235, 8'd235 };
	  4'd7 : cg = { 8'd0  , 8'd160, 8'd0   };
	  4'd8 : cg = { 8'd245, 8'd0  , 8'd0   };
	  4'd9 : cg = { 8'd235, 8'd160, 8'd0   };
	  4'd10: cg = { 8'd235, 8'd0  , 8'd235 };
	  4'd11: cg = { 8'd235, 8'd150, 8'd150 };
	  4'd12: cg = { 8'd235, 8'd235, 8'd0   };
	  4'd13: cg = { 8'd160, 8'd160, 8'd0   };
	  4'd14: cg = { 8'd150, 8'd150, 8'd150 };
	  4'd15: cg = { 8'd225, 8'd225, 8'd225 };
      default: ;
    endcase
  end
  else if (CFG_PALETTE == PALETTE_RF) begin
    // RF modulator colors are notably different from RGB. Copied from
    // Takeda-san's eSCV emulator [source/src/vm/scv/vdp.cpp].
    case (pd)
	  4'd0 : cg = { 8'd0  , 8'd90 , 8'd156 };
	  4'd1 : cg = { 8'd0  , 8'd0  , 8'd0   };
	  4'd2 : cg = { 8'd58 , 8'd148, 8'd255 };
	  4'd3 : cg = { 8'd0  , 8'd0  , 8'd255 };
	  4'd4 : cg = { 8'd16 , 8'd214, 8'd0   };
	  4'd5 : cg = { 8'd66 , 8'd255, 8'd16  };
	  4'd6 : cg = { 8'd123, 8'd230, 8'd197 };
	  4'd7 : cg = { 8'd0  , 8'd173, 8'd0   };
	  4'd8 : cg = { 8'd255, 8'd41 , 8'd148 };
	  4'd9 : cg = { 8'd255, 8'd49 , 8'd16  };
	  4'd10: cg = { 8'd255, 8'd58 , 8'd255 };
	  4'd11: cg = { 8'd239, 8'd156, 8'd255 };
	  4'd12: cg = { 8'd255, 8'd206, 8'd33  };
	  4'd13: cg = { 8'd74 , 8'd123, 8'd16  };
	  4'd14: cg = { 8'd165, 8'd148, 8'd165 };
	  4'd15: cg = { 8'd255, 8'd255, 8'd255 };
      default: ;
    endcase
  end
end

assign RGB = cg;


endmodule
