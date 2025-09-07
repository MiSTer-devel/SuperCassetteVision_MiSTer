// Epoch TV-1 testbench: render
//
// Copyright (c) 2024-2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// From MAME debugger: save vram.bin,2000,1404

`timescale 1us / 1ps

import scv_pkg::*;

module render_tb();

reg         clk, res;
reg [2:0]   ccnt;
reg [12:0]  a;
reg [7:0]   din;
reg         rdb, wrb, csb;
reg [7:0]   ioreg [4];

wire        ce;
wire [7:0]  dout;
wire [10:0] va;
wire [7:0]  vd_i, vd_o, vrama_do, vramb_do;
wire        nvwe;
wire [1:0]  nvcs;
wire        de, hs;
wire [23:0] rgb;

initial begin
  $timeformat(-6, 0, " us", 1);

  $dumpfile("render_tb.vcd");
  $dumpvars();
end

palette_t cfg_palette = PALETTE_RGB;
overscan_mask_t cfg_overscan_mask = OVERSCAN_MASK_NONE;

epochtv1 dut
  (
   .CLK(clk),
   .CE(ce),

   .CFG_PALETTE(cfg_palette),
   .CFG_OVERSCAN_MASK(cfg_overscan_mask),

   .A(a),
   .DB_I(din),
   .DB_O(dout),
   .DB_OE(),
   .RDB(rdb),
   .WRB(wrb),
   .CSB(csb),

   .VA(va),
   .VD_I(vd_i),
   .VD_O(vd_o),
   .nVWE(nvwe),
   .nVCS(nvcs),

   .DE(de),
   .HS(hs),
   .VS(),
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


//////////////////////////////////////////////////////////////////////

initial begin
  ccnt = 0;
  res = 1;
  clk = 1;

  rdb = 1;
  wrb = 1;
  csb = 1;
end

initial forever begin :ckgen
  #(0.25/14.318181) clk = ~clk; // 2 * 14.318181 MHz
end

always @(posedge clk)
  ccnt <= (ccnt == 3'd6) ? 0 : ccnt + 1'd1;

assign ce = (ccnt == 3'd6);

//////////////////////////////////////////////////////////////////////

task load_chr(input string path);
integer fin, code;
  fin = $fopen(path, "r");
  assert(fin != 0) else $fatal(1, "missing CHR ROM %s", path);

  code = $fread(dut.chr, fin, 0, 1024);
endtask

task load_rams(input string path);
reg [7:0] tmp [4];
integer fin, code, i;
  fin = $fopen(path, "r");
  assert(fin != 0) else $fatal(1, "missing RAM %s", path);

  // VRAM: $2000-$3FFF
  code = $fread(vrama.mem, fin, 0, 2048);
  code = $fread(vramb.mem, fin, 0, 2048);

  // BGM: $3000-$31FF
  for (i = 0; i < 128; i++) begin
    code = $fread(tmp, fin, 0, 4);
    dut.bgm[i] = {tmp[3], tmp[2], tmp[1], tmp[0]};
  end

  // OAM: $3200-$33FF
  for (i = 0; i < 128; i++) begin
    code = $fread(tmp, fin, 0, 4);
    dut.oam[i] = {tmp[3], tmp[2], tmp[1], tmp[0]};
  end

  code = $fread(tmp, fin, 0, 4);
  ioreg[0] = tmp[0];
  ioreg[1] = tmp[1];
  ioreg[2] = tmp[2];
  ioreg[3] = tmp[3];

endtask

//////////////////////////////////////////////////////////////////////

task ioreg_write(input [1:0] rs, input [7:0] v);
  while (!ce)
    @(posedge clk) ;
  a <= {11'h500, rs};
  din <= v;
  wrb <= 0;
  csb <= 0;

  @(posedge clk) ;
  while (!ce)
    @(posedge clk) ;
  wrb <= 1;
  csb <= 1;
  
endtask

//////////////////////////////////////////////////////////////////////

integer fpic, pice;
initial begin
  fpic = $fopen("render.hex", "w");
  pice = 0;
end
always @(posedge clk) begin
  if (ce) begin
    if (de) begin
      $fwrite(fpic, "%x", rgb);
      pice = 1;
    end
    else if (pice) begin
      pice = 0;
      $fwrite(fpic, "\n");
    end
  end
end
final
  $fclose(fpic);

//////////////////////////////////////////////////////////////////////

event init_regs;

always @(init_regs) begin
  ioreg_write(0, ioreg[0]);
  ioreg_write(1, ioreg[1]);
  ioreg_write(2, ioreg[2]);
  ioreg_write(3, ioreg[3]);
end

initial #0 begin
  dut.row = dut.FIRST_ROW_VSYNC - 1;

  load_chr("epochtv.chr");
  load_rams("balloons-vram.bin");

  -> init_regs;

  #17500 $finish;
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s render_tb -o render_tb.vvp ../../scv_pkg.sv ../epochtv1.sv ../dpram.sv render_tb.sv && ./render_tb.vvp && make render.png"
// End:
