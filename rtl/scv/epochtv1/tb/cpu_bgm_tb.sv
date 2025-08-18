// Epoch TV-1 testbench: CPU accesses BGM during render
//
// CPU accesses to BGM RAM can occur concurrently without affecting
// rendering or the CPU.  BGM writes take effect almost immediately.
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

module cpu_bgm_tb();

`include "dut_ctl_common.svh"

int loops = 0;

initial begin
  $timeformat(-6, 0, " us", 1);

  $dumpfile("cpu_bgm_tb.vcd");
  $dumpvars();
end

initial #0 begin
reg [7:0] tmp;
reg [8:0] aoff;
  dut_ctl_init("astro-splash-vram.bin");

  cpu_init();
  aoff = 0;
  forever begin
    // To prevent our writes from affecting rendering, keep the target
    // CPU address lower than the addresses read by rendering.
    while (dut.vdc.row < 16 * (aoff[8:5] + 'd1))
      #10 ;

    #10 cpu_rd('h1000 + aoff, tmp);
    assert(dut.vdc.bgm[aoff[8:2]][(aoff[1:0]*8)+:8] == tmp);
    #10 cpu_wr('h1000 + aoff, ~tmp);
    assert(dut.vdc.bgm[aoff[8:2]][(aoff[1:0]*8)+:8] == ~tmp);

    aoff += 1;
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
// compile-command: "iverilog -g2012 -grelative-include -s cpu_bgm_tb -o cpu_bgm_tb.vvp ../../scv_pkg.sv ../epochtv1.sv ../dpram.sv vdc_vram.sv cpu_bgm_tb.sv && ./cpu_bgm_tb.vvp"
// End:
