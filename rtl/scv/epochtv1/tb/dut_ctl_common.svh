// Copyright (c) 2024-2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

reg         clk, res;
reg [8:0]   c4cnt, c4cntn;      // 8.0 MHz
reg [1:0]   ccnt;               // 2.0 MHz
reg [2:0]   vcnt;
reg [7:0]   din;

wire        ce;
wire        cpu4_ce;
wire        cp1p, cp2p;
reg [12:0]  dut_a;
reg [7:0]   dut_db_i;
wire [7:0]  dut_db_o;
reg         dut_rdb, dut_wrb, dut_csb;
wire        dut_waitb;
wire        dut_de, ctl_de;
wire [23:0] dut_rgb, ctl_rgb;

//////////////////////////////////////////////////////////////////////

vdc_vram dut
  (
   .clk(clk),
   .ce(ce),
   .cp1p(cp1p),
   .a(dut_a),
   .db_i(dut_db_i),
   .db_o(dut_db_o),
   .rdb(dut_rdb),
   .wrb(dut_wrb),
   .csb(dut_csb),
   .waitb(dut_waitb),
   .de(dut_de),
   .rgb(dut_rgb)
   );

vdc_vram ctl
  (
   .clk(clk),
   .ce(ce),
   .cp1p(cp1p),
   .a('Z),
   .db_i('Z),
   .rdb(1'b1),
   .wrb(1'b1),
   .csb(1'b1),
   .waitb(),
   .de(ctl_de),
   .rgb(ctl_rgb)
   );

//////////////////////////////////////////////////////////////////////

localparam [8:0] CPU4_MUL = 9'd88;
localparam [8:0] CPU4_DIV = 9'd315;

initial begin
  c4cnt = 0;
  ccnt = 0;
  vcnt = 0;
  res = 1;
  clk = 1;
end

initial forever begin :ckgen
  #(0.25/14.318181) clk = ~clk; // 2 * 14.318181 MHz
end

assign c4cntn = c4cnt + CPU4_MUL;

always @(posedge clk) begin
  vcnt <= ce ? 0 : vcnt + 1'd1;
  c4cnt <= cpu4_ce ? (c4cntn - CPU4_DIV) : c4cntn;
  ccnt <= cpu4_ce ? ccnt + 1'd1 : ccnt;
end

assign cpu4_ce = c4cntn >= CPU4_DIV;
assign cp1p = cpu4_ce & (ccnt == 2'd1);
assign cp2p = cpu4_ce & (ccnt == 2'd3);
assign ce = (vcnt == 3'd6);

//////////////////////////////////////////////////////////////////////

task load_chr(input string path);
integer fin, code;
  fin = $fopen(path, "r");
  assert(fin != 0) else $fatal(1, "missing CHR ROM %s", path);

  code = $fread(dut.vdc.chr, fin, 0, 1024);
endtask

task load_rams(input string path);
reg [7:0] tmp [4];
integer fin, code, i;
  fin = $fopen(path, "r");
  assert(fin != 0) else $fatal(1, "missing RAM %s", path);

  // VRAM: $2000-$3FFF
  code = $fread(dut.vrama.mem, fin, 0, 2048);
  code = $fread(dut.vramb.mem, fin, 0, 2048);

  // BGM: $3000-$31FF
  for (i = 0; i < 128; i++) begin
    code = $fread(tmp, fin, 0, 4);
    dut.vdc.bgm[i] = {tmp[3], tmp[2], tmp[1], tmp[0]};
  end

  // OAM: $3200-$33FF
  for (i = 0; i < 128; i++) begin
    code = $fread(tmp, fin, 0, 4);
    dut.vdc.oam[i] = {tmp[3], tmp[2], tmp[1], tmp[0]};
  end

  code = $fread(tmp, fin, 0, 4);
  dut.vdc.ioreg0_p = tmp[0];
  dut.vdc.ioreg1_p = tmp[1];
  dut.vdc.ioreg2_p = tmp[2];
  dut.vdc.ioreg3_p = tmp[3];
  
endtask

task copy_to_ctl;
int i;
  for (i = 0; i < 2048; i++)
    ctl.vdc.chr[i] = dut.vdc.chr[i];
  for (i = 0; i < 2048; i++) begin
    ctl.vrama.mem[i] = dut.vrama.mem[i];
    ctl.vramb.mem[i] = dut.vramb.mem[i];
  end
  for (i = 0; i < 512; i++)
    ctl.vdc.bgm[i] = dut.vdc.bgm[i];
  for (i = 0; i < 128; i++)
    ctl.vdc.oam[i] = dut.vdc.oam[i];
  ctl.vdc.ioreg0_p = dut.vdc.ioreg0_p;
  ctl.vdc.ioreg1_p = dut.vdc.ioreg1_p;
  ctl.vdc.ioreg2_p = dut.vdc.ioreg2_p;
  ctl.vdc.ioreg3_p = dut.vdc.ioreg3_p;

  ctl.vdc.row = dut.vdc.row;
  ctl.vdc.col = dut.vdc.col;
endtask


//////////////////////////////////////////////////////////////////////

task cpu_init;
  dut_a = 'X;
  dut_db_i = 'Z;
  dut_rdb = 1'b1;
  dut_wrb = 1'b1;
  dut_csb = 1'b1;
endtask

task cpu_rd(input [12:0] a, output [7:0] d);
  while (~cp1p) @(posedge clk) ;
  dut_a <= a;
  dut_csb <= 1'b0;
  while (~cp2p) @(posedge clk) ;
  dut_rdb <= 1'b0;
  do begin
    @(posedge clk) ;
    while (~cp2p) @(posedge clk) ;
  end while (~dut_waitb);
  @(posedge clk) ;
  while (~cp2p) @(posedge clk) ;
  d <= dut_db_o;
  dut_rdb <= 1'b1;
  while (~cp1p) @(posedge clk) ;
  dut_csb <= 1'b1;
  dut_a <= 'X;
endtask

task cpu_wr(input [12:0] a, input [7:0] d);
  while (~cp1p) @(posedge clk) ;
  dut_a <= a;
  dut_csb <= 1'b0;
  while (~cp2p) @(posedge clk) ;
  dut_wrb <= 1'b0;
  dut_db_i <= d;
  do begin
    @(posedge clk) ;
    while (~cp2p) @(posedge clk) ;
  end while (~dut_waitb);
  @(posedge clk) ;
  while (~cp2p) @(posedge clk) ;
  dut_wrb <= 1'b1;
  dut_db_i <= 'Z;
  while (~cp1p) @(posedge clk) ;
  dut_csb <= 1'b1;
  dut_a <= 'X;
endtask

task dut_ctl_init(input string vram_path);
  load_chr("epochtv.chr");
  load_rams(vram_path);
  dut.vdc.row = dut.vdc.FIRST_ROW_VSYNC - 1;
  copy_to_ctl();
endtask
