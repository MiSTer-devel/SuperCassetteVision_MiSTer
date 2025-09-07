// Super Cassette Vision testbench: boot with cart, print frames
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ps

// Get to the main loop faster, by shorting loops.
`ifndef VERILATOR
`define FAST_MAIN 1
`endif

module scv_tb();

import scv_pkg::*;

reg         clk, res;
wire        pce, de, vs;
wire [23:0] rgb;
wire [8:0]  aud_pcm;

reg         rominit_active;
integer     rominit_fin;
reg         rominit_sel_boot, rominit_sel_chr, rominit_sel_apu,
            rominit_sel_cart;
reg [24:0]  rominit_addr;
reg [7:0]   rominit_data;
reg         rominit_valid;

mapper_t    mapper;

hmi_t       hmi;

palette_t vdc_palette = PALETTE_RGB;
overscan_mask_t vdc_overscan_mask = OVERSCAN_MASK_NONE;

initial begin
  $timeformat(-6, 0, " us", 1);

`ifndef VERILATOR
  $dumpfile("scv_tb.vcd");
  $dumpvars();
`else
  $dumpfile("scv_tb.verilator.fst");
`endif
end

scv dut
  (
   .CLK(clk),
   .RESB(~res),

   .ROMINIT_SEL_BOOT(rominit_sel_boot),
   .ROMINIT_SEL_CHR(rominit_sel_chr),
   .ROMINIT_SEL_APU(rominit_sel_apu),
   .ROMINIT_SEL_CART(rominit_sel_cart),
   .ROMINIT_ADDR(rominit_addr),
   .ROMINIT_DATA(rominit_data),
   .ROMINIT_VALID(rominit_valid),

   .MAPPER(mapper),
   .VDC_PALETTE(vdc_palette),
   .VDC_OVERSCAN_MASK(vdc_overscan_mask),

   .HMI(hmi),

   .VID_PCE(pce),
   .VID_DE(de),
   .VID_HS(),
   .VID_VS(vs),
   .VID_RGB(rgb),

   .AUD_PCM(aud_pcm)
   );

initial begin
  res = 1;
  clk = 1;

  rominit_active = 0;
  rominit_sel_boot = 0;
  rominit_sel_chr = 0;
  rominit_sel_apu = 0;
  rominit_sel_cart = 0;
  rominit_valid = 0;

  hmi = 0;

  mapper = MAPPER_AUTO;
end

initial forever begin :ckgen
  #(0.25/14.3181818) clk = ~clk; // 2 * 14.3181818 MHz
end

always @(posedge clk) if (rominit_active) begin
integer code;
logic [7:0] data;
  code = $fread(data, rominit_fin, 0, 1);
  if (!$feof(rominit_fin)) begin
    rominit_data <= data;
    if (rominit_valid)
      rominit_addr <= rominit_addr + 1'd1;
    else begin
      rominit_addr <= 0;
      rominit_valid <= '1;
    end
  end
  else begin
    rominit_active <= 0;
    rominit_addr <= 'X;
    rominit_data <= 'X;
    rominit_valid <= 0;
  end
end

task rominit_go(input string fn);
  rominit_fin = $fopen(fn, "r");
  assert(rominit_fin != 0) else $finish;
  rominit_active = '1;
  while (rominit_active)
    @(posedge clk) ;
  @(posedge clk) ;
  $fclose(rominit_fin);
endtask

task rominit_boot;
  rominit_sel_boot = '1;
  rominit_go("upd7801g.s01");
  rominit_sel_boot = '0;
endtask

task rominit_chr;
  rominit_sel_chr = '1;
  rominit_go("epochtv.chr.s02");
  rominit_sel_chr = '0;
endtask

task rominit_apu;
  rominit_sel_apu = '1;
  rominit_go("upd1771c-017.s03");
  rominit_sel_apu = '0;
endtask

task rominit_cart;
  rominit_sel_cart = '1;
  rominit_go("cart.bin");
  rominit_sel_cart = '0;
endtask

//////////////////////////////////////////////////////////////////////

integer frame = 0;
integer fpic;
logic   pice;
string  fname;

initial fpic = -1;
always @(negedge vs) begin
  if (fpic != -1) begin
    $fclose(fpic);
    fpic = -1;
`ifdef VERILATOR
    $system({"python3 ../epochtv1/tb/render2png.py ", fname, {".hex "}, fname, ".png; rm ", fname, ".hex"});
`endif
  end
  $display("%t: Frame %03d", $time, frame);
  $sformat(fname, "frames/render-%03d", frame);
  pice = 0;
`ifdef VERILATOR
  if (frame == 14)
    $dumpvars();
  if (frame == 17)
    $finish();
`else
  if (frame == 5)
    $finish();
`endif
  if ((frame % 1) == 0) begin
    fpic = $fopen({fname, ".hex"}, "w");
  end
  frame = frame + 1;
end
final
  $fclose(fpic);

always @(posedge clk) begin
  if (fpic != -1 && pce) begin
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

//////////////////////////////////////////////////////////////////////

initial #0 begin
  rominit_boot();
  rominit_chr();
  rominit_apu();
  rominit_cart();
  $display("ROMs loaded.");

  #2 @(posedge clk) ;
  res = 0;
  @(posedge dut.cpu.M1) ; // align with first ins.

`ifdef FAST_MAIN
  // We're looping until C reaches 0 (inner loop).
  #40 @(posedge clk) ;
  assert(dut.cpu.core.pc == 16'h0016);
  dut.cpu.core.c = 1;

  // We're also looping until B reaches 0 (outer loop).
  #40 @(posedge clk) ;
  assert(dut.cpu.core.pc == 16'h0018);
  dut.cpu.core.b = 0;
  dut.cpu.core.c = 1;
  #1 ;
`endif

end

initial if (0) begin
  #(300e3);
  $display("Pressing 1...");
  hmi.num[1] = '1;
  #(30e3) hmi.num[1] = 0;

  #(200e3);
  $display("Pressing 1...");
  hmi.num[1] = '1;
  #(30e3) hmi.num[1] = 0;

  #(200e3);
  $display("Pressing T1...");
  hmi.c1.t1 = '1;
  #(30e3) hmi.c1.t1 = 0;

  repeat (4) #(1000e3);
  $display("Pressing T1...");
  hmi.c1.t1 = '1;
  #(30e3) hmi.c1.t1 = 0;
end

endmodule


// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s scv_tb -o scv_tb.vvp -f scv.files scv_tb.sv && ./scv_tb.vvp"
// End:
