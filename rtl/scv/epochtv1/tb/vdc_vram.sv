// Copyright (c) 2024-2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

import scv_pkg::*;

module vdc_vram
  (
   input         clk,
   input         ce,
   input         cp1p,

   input [12:0]  a,
   input [7:0]   db_i,
   output [7:0]  db_o,
   input         rdb,
   input         wrb,
   input         csb,
   output        waitb,

   output        de,
   output        hs,
   output        vs,
   output [23:0] rgb
   );

wire [10:0] va;
wire [7:0]  vd_i, vd_o, vrama_do, vramb_do;
wire        nvwe;
wire [1:0]  nvcs;

palette_t cfg_palette = PALETTE_RGB;
overscan_mask_t cfg_overscan_mask = OVERSCAN_MASK_NONE;

epochtv1 vdc
  (
   .CLK(clk),
   .CE(ce),

   .CFG_PALETTE(cfg_palette),
   .CFG_OVERSCAN_MASK(cfg_overscan_mask),

   .CP1_POSEDGE(cp1p),
   .A(a),
   .DB_I(db_i),
   .DB_O(db_o),
   .DB_OE(),
   .RDB(rdb),
   .WRB(wrb),
   .CSB(csb),
   .WAITB(waitb),

   .VA(va),
   .VD_I(vd_i),
   .VD_O(vd_o),
   .nVWE(nvwe),
   .nVCS(nvcs),

   .DE(de),
   .HS(hs),
   .VS(vs),
   .RGB(rgb)
   );

dpram #(.DWIDTH(8), .AWIDTH(11)) vrama
  (
   .CLK(clk),

   .nCE(nvcs[0]),
   .nWE(nvwe),
   .nOE(~nvwe),
   .A(va),
   .DI(vd_o),
   .DO(vrama_do),

   .nCE2(1'b1),
   .nWE2(1'b1),
   .nOE2(1'b1),
   .A2(),
   .DI2(),
   .DO2()
   );

dpram #(.DWIDTH(8), .AWIDTH(11)) vramb
  (
   .CLK(clk),

   .nCE(nvcs[1]),
   .nWE(nvwe),
   .nOE(~nvwe),
   .A(va),
   .DI(vd_o),
   .DO(vramb_do),

   .nCE2(1'b1),
   .nWE2(1'b1),
   .nOE2(1'b1),
   .A2(),
   .DI2(),
   .DO2()
   );

assign vd_i = (~nvcs[0]) ? vrama_do : (~nvcs[1]) ? vramb_do : 8'hxx;

endmodule
