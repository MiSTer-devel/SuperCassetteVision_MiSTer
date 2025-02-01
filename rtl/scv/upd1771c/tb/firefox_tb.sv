// NEC uPD1771C-011 testbench: play the tone warble heard on PAUSE
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

module firefox_tb();

reg         clk, res;
wire [7:0]  pa_i, pb_i, pb_o;
logic [7:0] din;
int         cycle;

initial begin
  $timeformat(-6, 0, " us", 1);

  $dumpfile("firefox_tb.vcd");
  $dumpvars();
end

upd1771c dut
  (
   .CLK(clk),
   .CKEN('1),
   .RESB(~res),
   .CH1('1),
   .CH2('0),
   .PA_I(pa_i),
   .PA_OE(),
   .PA_O(),
   .PB_I(pb_i),
   .PB_O(pb_o),
   .PB_OE()
   );

assign pa_i = din;
assign pb_i = '1;

task tx(input [7:0] b);
  // Align to PHI2
  while (~dut.phi2p)
    @(posedge clk) ;
  if (~clk)
    @(posedge clk) ;
  din <= b;
  repeat (8)
    @(posedge clk) ;
  repeat (8*9)
    @(posedge clk) ;
endtask

always begin :ckgen
  #(0.5/6) clk = ~clk;
end

always @(posedge clk) begin
  if (~res & dut.phi2p)
    cycle += 1;
end

initial #0 begin
  res = 1;
  clk = 1;
  din = 8'h00;
  cycle = 0;

  #2 @(posedge clk) ;
  res = 0;

  #1000 @(posedge clk) ;

  tx(8'h0f);

  #(100e3) @(posedge clk) ;

  $finish;
end

endmodule


// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -DUPD1771C_ROM_INIT_FROM_HEX -s firefox_tb -o firefox_tb.vvp ../upd1771c.sv firefox_tb.sv && ./firefox_tb.vvp"
// End:
