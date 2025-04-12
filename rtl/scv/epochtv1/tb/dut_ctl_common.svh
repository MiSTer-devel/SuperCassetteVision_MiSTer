// Copyright (c) 2024-2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

reg         clk, res;
reg [2:0]   ccnt;
reg [7:0]   din;

wire        ce;
reg [12:0]  dut_a;
reg [7:0]   dut_db_i;
wire [7:0]  dut_db_o;
reg         dut_rdb, dut_wrb, dut_csb;
wire        dut_de, ctl_de;
wire [23:0] dut_rgb, ctl_rgb;

//////////////////////////////////////////////////////////////////////

vdc_vram dut
  (
   .clk(clk),
   .ce(ce),
   .a(dut_a),
   .db_i(dut_db_i),
   .db_o(dut_db_o),
   .rdb(dut_rdb),
   .wrb(dut_wrb),
   .csb(dut_csb),
   .de(dut_de),
   .rgb(dut_rgb)
   );

vdc_vram ctl
  (
   .clk(clk),
   .ce(ce),
   .a('Z),
   .db_i('Z),
   .rdb(1'b1),
   .wrb(1'b1),
   .csb(1'b1),
   .de(ctl_de),
   .rgb(ctl_rgb)
   );

//////////////////////////////////////////////////////////////////////

initial begin
  ccnt = 0;
  res = 1;
  clk = 1;
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
  dut.vdc.ioreg0 = tmp[0];
  dut.vdc.ioreg1 = tmp[1];
  dut.vdc.ioreg2 = tmp[2];
  dut.vdc.ioreg3 = tmp[3];
  
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
  ctl.vdc.ioreg0 = dut.vdc.ioreg0;
  ctl.vdc.ioreg1 = dut.vdc.ioreg1;
  ctl.vdc.ioreg2 = dut.vdc.ioreg2;
  ctl.vdc.ioreg3 = dut.vdc.ioreg3;

  ctl.vdc.row = dut.vdc.row;
  ctl.vdc.col = dut.vdc.col;
endtask


//////////////////////////////////////////////////////////////////////

task cpu_init;
  dut_a = 0;
  dut_db_i = 'Z;
  dut_rdb = 1'b1;
  dut_wrb = 1'b1;
  dut_csb = 1'b1;
endtask

task cpu_rd(input [12:0] a, output [7:0] d);
  while (~ce) @(posedge clk) ;
  dut_a = a;
  dut_csb = 1'b0;
  repeat (14) @(posedge clk) ;
  dut_rdb = 1'b0;
  repeat (26) @(posedge clk) ;
  d = dut_db_o;
  dut_rdb = 1'b1;
  repeat (2) @(posedge clk) ;
  dut_csb = 1'b1;
endtask

task cpu_wr(input [12:0] a, input [7:0] d);
  while (~ce) @(posedge clk) ;
  dut_a = a;
  dut_csb = 1'b0;
  repeat (14) @(posedge clk) ;
  dut_wrb = 1'b0;
  dut_db_i = d;
  repeat (26) @(posedge clk) ;
  dut_wrb = 1'b1;
  //dut_db_i = 'Z;
  repeat (2) @(posedge clk) ;
  dut_csb = 1'b1;
endtask

task dut_ctl_init(input string vram_path);
  load_chr("epochtv.chr");
  load_rams(vram_path);
  ctl.vdc.row = dut.vdc.FIRST_ROW_BOC_START - 1;
  copy_to_ctl();
endtask

always @(posedge clk) if (dut_de) begin
  assert(dut_rgb === ctl_rgb);
  else begin
    $fatal(1, "output mismatch");
  end
end
