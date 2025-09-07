//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [48:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

import scv_pkg::hmi_t;
import scv_pkg::mapper_t;

wire        clk_sys;

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;  

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////
// HPS I/O

// Status Bit Map:
// 0         1         2         3          4         5         6
// 01234567890123456789012345678901 23456789012345678901234567890123
// 0123456789ABCDEFGHIJKLMNOPQRSTUV 0123456789ABCDEFGHIJKLMNOPQRSTUV
// X       XXX     XXXX

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v" 
localparam CONF_STR = {
	"SCV;;",
	"-;",
    "F1,ROMBIN;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	//"O[2],TV Mode,NTSC,PAL;",
	"O[8],Palette,RGB,RF;",
    "O[10:9],Overscan mask,Large,Small,None;",
	"-;",
    "O[19:16],Cartridge mapper,automatic,rom8k,rom16k,rom32k,rom32k_ram,rom64k,rom128k,rom128_ram;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"J,Trig 1,Trig 2,SELECT 1,SELECT 2,SELECT 3,SELECT 4,SELECT EN;",
	"jn,A,B,X,Y,L,R,Start;",
	"v,1;",
	"V,v",`BUILD_DATE 
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire [31:0]  joystick_0, joystick_1;
wire  [10:0] ps2_key;

wire         ioctl_download;
wire [7:0]   ioctl_index;
wire         ioctl_wr;
wire [24:0]  ioctl_addr;
wire [7:0]   ioctl_dout;
wire         ioctl_wait;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask(0),
	
 	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.ps2_key(ps2_key),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

//////////////////////////////////////////////////////////////////////
// Download manager

wire         rominit_active;
wire         rominit_sel_boot, rominit_sel_chr, rominit_sel_apu,
             rominit_sel_cart;
wire [16:0]  rominit_addr;
wire [7:0]   rominit_data;
wire         rominit_valid;

rominit rominit
  (
   .CLK_SYS(clk_sys),

   .IOCTL_DOWNLOAD(ioctl_download),
   .IOCTL_INDEX(ioctl_index),
   .IOCTL_WR(ioctl_wr),
   .IOCTL_ADDR(ioctl_addr),
   .IOCTL_DOUT(ioctl_dout),
   .IOCTL_WAIT(ioctl_wait),

   .ROMINIT_ACTIVE(rominit_active),
   .ROMINIT_SEL_BOOT(rominit_sel_boot),
   .ROMINIT_SEL_CHR(rominit_sel_chr),
   .ROMINIT_SEL_APU(rominit_sel_apu),
   .ROMINIT_SEL_CART(rominit_sel_cart),
   .ROMINIT_ADDR(rominit_addr),
   .ROMINIT_DATA(rominit_data),
   .ROMINIT_VALID(rominit_valid)
   );

//////////////////////////////////////////////////////////////////////
// Connect joysticks and keyboard to HMI

hmi_t       hmi;

joykey joykey
  (
   .CLK_SYS(clk_sys),

   .JOYSTICK_0(joystick_0),
   .JOYSTICK_1(joystick_1),

   .PS2_KEY(ps2_key),

   .HMI(hmi)
   );

////////////////////////////////////////////////////////////////////
// Audio

// TODO: Add CDC for AUDIO_* into CLK_AUDIO

wire signed [8:0]   aud_pcm;
wire signed [15:0]  aud_pcm16, aud_out;

assign aud_pcm16 = {aud_pcm, 7'b0}; // sign-extend to 16 bits
assign aud_out = aud_pcm16 >>> 1;     // add some headroom

assign AUDIO_S = '1; // signed
assign AUDIO_L = aud_out;
assign AUDIO_R = AUDIO_L;
assign AUDIO_MIX = 0; // no mixing

///////////////////////   CLOCKS   ///////////////////////////////

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys)
);

mapper_t mapper;
palette_t vdc_palette;
overscan_mask_t vdc_overscan_mask;

wire reset = RESET | status[0] | buttons[1] | rominit_active;
assign mapper = mapper_t'(status[19:16]);
assign vdc_palette = palette_t'(status[8]);
assign vdc_overscan_mask = overscan_mask_t'(status[10:9]);

scv scv
  (
   .CLK(clk_sys),
   .RESB(~reset),

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

   .VID_PCE(CE_PIXEL),
   .VID_DE(VGA_DE),
   .VID_HS(VGA_HS),
   .VID_VS(VGA_VS),
   .VID_RGB({VGA_R, VGA_G, VGA_B}),

   .AUD_PCM(aud_pcm)
   );

assign CLK_VIDEO = clk_sys;

assign LED_USER = ioctl_download;

endmodule
