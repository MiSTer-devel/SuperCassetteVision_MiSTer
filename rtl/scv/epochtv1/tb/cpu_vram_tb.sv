// Epoch TV-1 testbench: CPU accesses VRAM during render
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

module cpu_vram_tb();

`include "dut_ctl_common.svh"

int loops = 0;

initial begin
  $timeformat(-6, 0, " us", 1);

  $dumpfile("cpu_vram_tb.vcd");
  $dumpvars();
end

initial #0 begin
reg [7:0] tmp;
reg [11:0] aoff;
  dut_ctl_init("astro-splash-vram.bin");

  cpu_init();
  aoff = 0;
  forever begin
    #3 cpu_rd('h0000 + aoff, tmp);
    if (aoff[11] == 1'b0) begin
      assert(dut.vrama.mem[aoff[10:0]] == tmp);
    end
    else begin
      assert(dut.vramb.mem[aoff[10:0]] == tmp);
    end
    #5 cpu_wr('h0000 + aoff, tmp);
    aoff += 1;
    if (aoff == 0)
      loops += 1;
  end
end

// Zero control VRAM reads when CPU accesses DUT VRAM.
always @(posedge clk) begin
  if (dut.vdc.vram_cpu_sel)
    force ctl.vdc.VD_I = 0;
  else
    release ctl.vdc.VD_I;
end

always @(posedge clk) if (dut_de) begin
  assert(dut_rgb === ctl_rgb);
  else begin
    $fatal(1, "output mismatch");
  end
end

initial #100000 begin
  assert(loops);
  else begin
    $fatal(1, "test did not loop");
  end
  $finish;
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s cpu_vram_tb -o cpu_vram_tb.vvp ../../scv_pkg.sv ../epochtv1.sv ../dpram.sv vdc_vram.sv cpu_vram_tb.sv && ./cpu_vram_tb.vvp"
// End:
