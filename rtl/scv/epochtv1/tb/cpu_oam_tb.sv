// Epoch TV-1 testbench: CPU accesses OAM during render
//
// OAM RAM is copied from shadow to active RAM during VSYNC.  CPU
// accesses target shadow RAM and have no effect on rendering.
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

module cpu_oam_tb();

`include "dut_ctl_common.svh"

int loops = 0;

initial begin
  $timeformat(-6, 0, " us", 1);

  $dumpfile("cpu_oam_tb.vcd");
  $dumpvars();
end

initial #0 begin
reg [7:0] tmp;
reg [8:0] aoff;
  dut_ctl_init("astro-splash-vram.bin");

  cpu_init();
  aoff = 0;

  // Don't overlap with OAM copy
  while (dut.vdc.row != 0)
    @(posedge clk) ;

  forever begin
    #10 cpu_rd('h1200 + aoff, tmp);
    #10 cpu_wr('h1200 + aoff, tmp);
    aoff += 1;
    if (aoff == 0)
      loops += 1;
  end
end

always @(posedge clk) if (dut_de) begin
  assert(dut_rgb === ctl_rgb);
  else begin
    $fatal(1, "output mismatch");
  end
end

initial #17000 begin
  assert(loops);
  else begin
    $fatal(1, "test did not loop");
  end
  $finish;
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s cpu_oam_tb -o cpu_oam_tb.vvp ../../scv_pkg.sv ../epochtv1.sv ../dpram.sv vdc_vram.sv cpu_oam_tb.sv && ./cpu_oam_tb.vvp"
// End:
