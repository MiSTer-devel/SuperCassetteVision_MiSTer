// NEC uPD7800 testbench: cputest cartridge ROM
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

// Get to the main loop faster, by shorting loops.
`ifndef VERILATOR
`define FAST_MAIN 1
`endif

`define TEST_WAIT 1

module cputest_tb();

reg         clk, res;
reg         cp1p, cp1n, cp2p, cp2n;
reg [7:0]   dut_db_i;
reg         mem_rdb, mem_ready;
reg         vbl;

wire [15:0] a;
wire [7:0]  dut_db_o, vram_db, cart_db;
wire        a_oe;
wire        waitb;
wire        dut_rdb, dut_wrb;
wire        vram_ncs, cart_ncs;

int         tvbl1 = 5000;
int         tvbl0 = 5000;

initial begin
  $timeformat(-6, 0, " us", 1);

`ifndef VERILATOR
  $dumpfile("cputest_tb.vcd");
  $dumpvars();
`else
  $dumpfile("cputest_tb.verilator.vcd");
  $dumpvars();
`endif
end

upd7801 dut
  (
   .CLK(clk),
   .CP1_POSEDGE(cp1p),
   .CP1_NEGEDGE(cp1n),
   .CP2_POSEDGE(cp2p),
   .CP2_NEGEDGE(cp2n),
   .INIT_SEL_BOOT('0),
   .INIT_ADDR('0),
   .INIT_DATA('0),
   .INIT_VALID('0),
   .RESETB(~res),
   .INT0(1'b0),
   .INT1(1'b0),
   .INT2(vbl),
   .A(a),
   .A_OE(a_oe),
   .DB_I(dut_db_i),
   .DB_O(dut_db_o),
   .DB_OE(),
   .WAITB(waitb),
   .M1(),
   .RDB(dut_rdb),
   .WRB(dut_wrb),
   .PA_O(),
   .PB_I(8'hff),                // no buttons pressed
   .PB_O(),
   .PB_OE(),
   .PC_I(8'h01),                // pause switch off
   .PC_O(),
   .PC_OE()
   );

initial
  $readmemh("bootrom.hex", dut.rom.mem);

ram #(10, 8) vram
  (
   .CLK(clk),
   .nCE(vram_ncs),
   .nWE(dut_wrb),
   .nOE(vram_ncs | dut_rdb),
   .A(a[9:0]),
   .DI(dut_db_o),
   .DO(vram_db)
   );

cart cart
  (
   .CLK(clk),
   .A(a[14:0]),
   .DB(cart_db),
   .nCS(cart_ncs | dut_rdb)
   );

always_comb begin
  dut_db_i = 'Z;
  if (~mem_ready)
    ;
  else if (~vram_ncs)
    dut_db_i = vram_db;
  else
    dut_db_i = cart_db;
end

assign vram_ncs = ~(a_oe & ~a[15]);
assign cart_ncs = ~vram_ncs;

`ifdef TEST_WAIT
// HACK: This works because WAITB also affects internal RAM/ROM accesses.
wire mem_cs = ~(mem_rdb & dut.core_wrb);
always @(posedge clk) if (cp1p) begin
  mem_ready <= mem_cs;
  mem_rdb <= dut.core_rdb; // RDB asserts in T1
end
`endif
assign waitb = mem_ready;

initial begin
  vbl = 0;
  cp1p = 0;
  cp1n = 0;
  cp2p = 0;
  cp2n = 0;
  res = 1;
  clk = 1;
`ifdef TEST_WAIT
  mem_ready = 0;
`else
  mem_ready = 1;
`endif
end

always begin :ckgen
  #0.0625 clk = ~clk;
end

always begin :cpgen
  @(posedge clk) cp2n <= 0; cp1p <= 1;
  @(posedge clk) cp1p <= 0; cp1n <= 1;
  @(posedge clk) cp1n <= 0; cp2p <= 1;
  @(posedge clk) cp2p <= 0; cp2n <= 1;
end

wire cp2 = dut.core.cp2;

initial begin :vblgen
  @(negedge res) ;
  forever begin
    repeat (tvbl0) @(posedge clk) ;
    vbl = 1'b1;
    repeat (tvbl1) @(posedge clk) ;
    vbl = 1'b0;
  end
end

task normal_video();
  begin
    // Normal video timing
    tvbl1 = 8 * 1558;
    tvbl0 = 8 * 15109;
  end
endtask


`ifdef FAST_MAIN
initial #0 begin
  #2 @(posedge clk) ;
  res = 0;

  // We're looping until C reaches 0 (inner loop).
  #40 @(posedge clk) ;
  assert(dut.core.pc == 16'h0016);
  dut.core.c = 1;

  // We're also looping until B reaches 0 (outer loop).
`ifdef TEST_WAIT
  #2 ;
`endif
  #40 @(posedge clk) ;
  assert(dut.core.pc == 16'h0018);
  dut.core.b = 0;
  dut.core.c = 1;
  #1 ;

  // Double 'block' in ClearScreen
`ifdef TEST_WAIT
  #76 ;
`endif
  #340 @(posedge clk) ;
  assert(dut.core.pc == 16'h0a25);
  dut.core.c -= 8'hF8;
  dut.core.e += 8'hF8;
  dut.core.l += 8'hF8;
  #26 @(posedge clk) ;
  assert(dut.core.pc == 16'h0a26);
  dut.core.c -= 8'hFD;
  dut.core.e += 8'hFD;
  dut.core.l += 8'hFD;

  normal_video();
end

`else // FAST_MAIN

initial #0 begin
  normal_video();

  #2 @(posedge clk) ;
  res = 0;
end

`endif

logic [15:0] test_num = 'hffff;
always @(posedge clk) begin
  if (cp2n & ~dut.wram_ncs & ~dut.core_wrb & (dut.core_a[6:0] == 7'h01)) begin
  logic [15:0] n;
    n = {dut.wram.mem[1], dut.wram.mem[0]};
    if (test_num != n) begin
      test_num = n;
      $display("%t: Test %4x", $time, test_num);
    end
  end

  // JR $, PSW.SK=0
  if (cp1p && dut.core.ir == 'hFF && dut.core.of_done & ~dut.core.psw[5]) begin
    if (dut.core.pc == 'h812A) begin     // end of success
      $display("Success!");
      $finish;
    end
    else begin
      $error("%t: Infinite loop", $time);
      $fatal(2);
    end
  end
end

endmodule


//////////////////////////////////////////////////////////////////////

module ram
  #(parameter AW,
    parameter DW)
  (
   input           CLK,
   input           nCE,
   input           nWE,
   input           nOE,
   input [AW-1:0]  A,
   input [DW-1:0]  DI,
   output [DW-1:0] DO
   );

reg [DW-1:0] mem [0:((1 << AW) - 1)];
reg [DW-1:0] dor;

// Undefined RAM contents make simulation eventually die.
initial begin
int i;
  for (i = 0; i < (1 << AW); i++)
    mem[i] = 0;
end

always @(posedge CLK)
  dor <= mem[A];

assign DO = ~(nCE | nOE) ? dor : {DW{1'bz}};

always @(negedge CLK) begin
  if (~(nCE | nWE)) begin
    mem[A] <= DI;
  end
end

endmodule


//////////////////////////////////////////////////////////////////////

module cart #(parameter AW=15)
  (
   input            CLK,
   input [AW-1:0]   A,
   output reg [7:0] DB,
   input            nCS
   );

localparam SIZE = 1 << AW;

logic [7:0] mem [SIZE];

initial begin
integer fin, code;
  fin = $fopen("cputest_cart.bin", "r");
  assert(fin != 0) else $finish;
  code = $fread(mem, fin, 0, SIZE);
end

always @(posedge CLK) begin
  DB <= nCS ? 8'hzz : mem[A];
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s cputest_tb -o cputest_tb.vvp ../upd7800.sv ../upd7801.sv cputest_tb.sv && ./cputest_tb.vvp"
// End:
