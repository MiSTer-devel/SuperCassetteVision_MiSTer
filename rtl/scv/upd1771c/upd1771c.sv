// NEC uPD1771C - a functional implementation
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// . https://github.com/mamedev/mame/blob/master/src/devices/sound/upd1771.cpp - MAME's emulation
// . https://siliconpr0n.org/map/nec/d1771c-017/mcmaster_mz_mit20x/ - photomicrograph of D1771C-017 die
// . http://reverendgumby.gitlab.io/visuald1771c - JavaScript simulator derived from above die shot
// . https://oura.oguchi-rd.com - original LSI design docs

`timescale 1us / 1ns

module upd1771c
  (
   input        CLK,
   input        CKEN,
   input        RESB,

`ifndef UPD1771C_ROM_INIT_FROM_HEX
   input        INIT_SEL,
   input [9:0]  INIT_ADDR,
   input [7:0]  INIT_DATA,
   input        INIT_VALID,
`endif

   input        CH1,
   input        CH2,

   input [7:0]  PA_I,
   output [7:0] PA_O,
   output [7:0] PA_OE,
   input [7:0]  PB_I,
   output [7:0] PB_O,
   output [7:0] PB_OE,

   output [8:0] PCM_OUT
   );


logic [15:0]    db;
logic [11:0]    ab;
logic [15:0]    pdb;

wire            md_64_32, md_tone_ie, md_ns_ie, md_nss,
                md_time_ie, md_ext_ie, md_out, md_if;

logic           fl0;            // Flag 0 (FL0)
logic           ts;             // Tone Sign (TS)
logic           ns;             // Noise Sign (NS)
logic           ss;             // Samle Sign (SS) = DAC out negative (-)
logic [9:0]     da;             // Input to DAC, pre-inversion

logic           cl_t1;
logic [11:0]    int_vec;


//////////////////////////////////////////////////////////////////////
// Reset, etc.

wire  res = ~RESB;
logic resp;
wire  leader, follower, testmode;
logic ch1d, ch2d;

always @(posedge CLK) if (CKEN) begin
  resp <= (resp & ~(phi2p & ~res)) | res;
end

always @(posedge CLK) if (CKEN) begin
  if (phi2)
    ch1d <= CH1;
  if (phi2 | t45)
    ch2d <= CH2;
end

assign leader = ~ch1d & ~ch2d;
assign follower = ch1d & ~ch2d;
assign testmode = ~(leader | follower);


//////////////////////////////////////////////////////////////////////
// Clock generator

// 1 cycle = 8 * CLK
wire phi2p, phi2n;              // 1~3
wire phi1p, phi1n;              // 6~7
wire philsi;                    // 3,5,7
wire phi56;                     // 5~6 (ROM clock)
wire t23;                       // 2~3
wire t45;                       // 4~5
wire t68;                       // 6~8

logic phi2, phi1;

logic [2:0] cgcnt;
logic [8:0] cgo;

initial begin
  phi2 = 0;
  phi1 = 0;
  cgcnt = 0;
end

always @(posedge CLK) if (CKEN) begin
  cgcnt <= cgcnt + 1'd1;
end

always_comb begin
  case (cgcnt)
    3'd0: cgo = 9'b000_0_0_0000;
    3'd1: cgo = 9'b001_0_0_0000;
    3'd2: cgo = 9'b001_0_1_0010;
    3'd3: cgo = 9'b010_0_0_0000;
    3'd4: cgo = 9'b010_1_1_0100;
    3'd5: cgo = 9'b100_1_0_0000;
    3'd6: cgo = 9'b100_0_1_1000;
    3'd7: cgo = 9'b100_0_0_0001;
    default: cgo = 'X;
  endcase
end

// phi*p/n are 1 CLK early, so that phi1/2 align with other clocks.
assign phi2p = CKEN & cgo[0];
assign phi2n = CKEN & cgo[1];
assign phi1p = CKEN & cgo[2];
assign phi1n = CKEN & cgo[3];
assign philsi = CKEN & cgo[4];
assign phi56 = CKEN & cgo[5];
assign t23 = CKEN & cgo[6];
assign t45 = CKEN & cgo[7];
assign t68 = CKEN & cgo[8];

always @(posedge CLK) if (CKEN) begin
  phi2 <= (phi2 | phi2p) & ~phi2n;
  phi1 <= (phi1 | phi1p) & ~phi1n;
end


//////////////////////////////////////////////////////////////////////
// Instruction decoder

logic [15:0] id;

always_comb begin
  id = pdb;
  if (cl_t1)
    id[15:12] = 4'b0110;
end

// The following lines are ordered by vertical position in decoder.

wire id_op_jmpfz = ~|id[15:14] & id[13] & ~|id[12] & id[11]; // 0010_1xxx__xxxx
wire id_op_stf = ~|id[15:11] & id[2] & id[0]; // 0000_00xx__x101
wire id_op_jmp_n4 = ~|id[15:14] & id[13] & ~|id[12:11] & ~id[8]; // 0010_0xx0__xxxx
wire id_op_jmp_call = ~id[15] & &id[14:13] & ~cl_t1; // 011x_xxxx__xxxx
wire id_op_calln = ~|id[15:13] & &id[12:11] & id[3]; // 0001_1xxx__1000
wire id_op_tbln_calln = ~|id[15:13] & &id[12:11]; // 0001_1xxx__xxxx
wire id_op_jmpa = ~|id[15:11] & id[10] & id[8] & id[0]; // 0000_01x1__xxx1
wire id_op_off = ~|id[15:11] & &id[10:9] & id[1]; // 0000_011x__xx1x
wire id_op_reti_ext = ~|id[15:12] & id[11] & id[8] & id[3]; // 0000_1xx1__1xxx
wire id_op_reti_time = ~|id[15:12] & id[11] & id[8] & id[2]; // 0000_1xx1__x1xx
wire id_op_reti_ns = ~|id[15:12] & id[11] & id[8] & id[1]; // 0000_1xx1__xx1x
wire id_op_reti_tone = ~|id[15:12] & id[11] & id[8] & id[0]; // 0000_1xx1__xxx1
wire id_op_mov_N_a = ~|id[15:10] & id[9] & id[0]; // 0000_001x__xxx1
wire id_op_out_da = ~|id[15:11] & id[10] & id[8] & id[1]; // 0000_01x1__xx1x
wire id_op_mvi_md1_n = ~|id[15:14] & &id[13:12] & id[8]; // 0011_xxx1__xxxx
wire id_aluop_notest_md1_n = id[15] & ~|id[14:12] & id[8]; // 1000_xxx1__xxxx
wire id_op_mvi_md0_n = ~|id[15:14] & id[13] & ~id[12] & id[8]; // 0010_xxx1__xxxx
wire id_op_tbln_X_Rr = ~|id[15:13] & &id[12:11] & id[1]; // 0001_1xxx__xx1x
wire id_op_mov_X_RG = ~|id[15:10] & id[3]; // 0000_00xx__1xxx
wire id_op_mul2 = ~|id[15:11] & id[10] & id[8] & id[3]; // 0000_01x1__1xxx
wire id_op_ral = ~|id[15:11] & id[10] & ~id[8] & id[3]; // 0000_01x0__1xxx
wire id_op_rar = ~|id[15:11] & id[10] & ~id[8] & id[2]; // 0000_01x0__x1xx
wire id_op_mul2_b = ~|id[15:11] & id[10] & id[8] & &id[3:2]; // 0000_01x1__11xx
wire id_op_mul1 = ~|id[15:11] & id[10] & id[8] & ~id[3] & id[2] & ~y[0]; // 0000_01x1__01xx & ~y[0]
wire id_op_mov_xchg_a_dst = ~|id[15:13] & id[12] & ~id[11] & id[2]; // 0001_0xxx__x1xx
wire id_op_tbln_a_Rr = ~|id[15:13] & &id[12:11] & id[0]; // 0001_1xxx__xxx1
wire id_aluop_A_Rr = &id[15:14] & ~|id[13:12] & ~id[3]; // 1100_xxxx__0xxx
wire id_aluop_A_n = id[15] & ~|id[14:12] & ~id[8]; // 1000_xxx0__xxxx
wire id_set_a = ~|id[15:14] & id[10] & ~id[9]; // 00xx_x10x__xxxx
wire id_op_reti = ~|id[15:12] & id[11] & id[8]; // 0000_1xx1__xxxx
wire id_get_a_t45 = ~|id[15:11] & ~(~|id[10:9] & id[3]) & ~resp; // 0000_0xxx__xxxx & ~xxxx_x00x__1xxx & ~resp
wire id_get_a_t23_a = ~|id[15:13] & id[12] & ~id[11] & id[0]; // 0001_0xxx__xxx1
wire id_get_a_t23_b = &id[15:14] & ~id[13]; // 110x_xxxx__xxxx
wire id_get_a_t23_c = id[15] & ~|id[14:13] & ~id[8]; // 100x_xxx0__xxxx
wire id_op_tbln_Y_Rr = ~|id[15:13] & &id[12:11] & id[2]; // 0001_1xxx__x1xx
wire id_op_mov_Y_Rr = ~|id[15:13] & id[12] & ~id[11] & ~id[9] & ~id[3] & ~id[0]; // 0001_0x0x__0xx0
wire id_op_muln = ~|id[15:11] & id[10] & id[8] & id[2]; // 0000_01x1__x1xx
wire id_op_jmp_call_n_t2 = ~id[15] & &id[14:13] & ~cl_t1; // 011x_xxxx__xxxx & ~cl_t1
wire id_op_adi5 = &id[15:12] & ~id[11] & ~id[9]; // 1111_0x0x__xxxx
wire id_op_tadi5 = &id[15:11]; // 1111_1xxx__xxxx
wire id_op_adims = &id[15:12] & id[9]; // 1111_xx1x__xxxx
wire id_op_adi_sbi_adi5_Rr_n = &id[15:13] & ~id[11] & ~id[9]; // 111x_0x0x__xxxx
wire id_op_xori = id_aluop_10x_1x0 & ~id[12] & &id[11:9]; // xxx0_111x__xxxx
wire id_op_adi_andi_sbi_ori = id_aluop_10x_1x0 & ~|id[12:11] ; // xxx0_0xxx__xxxx
wire id_op_not_aluop_or_rets = ~id_op_rets & ~id[15]; // 0xxx_xxxx__xxxx
wire id_op_rets = ~|id[15:12] & id[11] & ~id[8] & id[0]; // 0000_1xx0__xxx1
wire id_aluop_10x_1x0_txnx = id_aluop_10x_1x0 & ~id[11]; // xxxx_0xxx__xxxx
wire id_aluop_txnx_Rr_n = &id[15:13] & ~id[12] & ~id[9]; // 1110_xx0x__xxxx
wire id_aluop_10x_1x0_sk_z = id_aluop_10x_1x0 & id[9]; // xxxx_xx1x__xxxx & aluop_10x_1x0
wire id_aluop_Rr_n_not_c5 = &id[15:13] & ~id[12]; // 1110_xxxx__xxxx
wire id_aluop_10x_1x0_sk_cb = id_aluop_10x_1x0 & ~id[9]; // xxxx_xx0x__xxxx & aluop_10x_1x0
wire id_aluop_or_xor_equ = id_aluop_10x_1x0 & &id[10:9]; // xxxx_x11x__xxxx & aluop_10x_1x0
wire id_aluop_or = id_aluop_10x_1x0 & ~|id[12:11] & &id[10:9]; // xxx0_011x__xxxx & aluop_10x_1x0
wire id_aluop_and = id_aluop_10x_1x0 & ~id[10] & id[9]; // xxxx_x01x__xxxx & aluop_10x_1x0
wire id_aluop_10x = id[15] & ~id[14]; // 10xx_xxxx__xxxx
wire id_aluop_10x_1x0 = id_aluop_10x | id_aluop_1x0;
wire id_aluop_1x0 = id[15] & ~id[13]; // 1x0x_xxxx__xxxx
wire id_aluop_sb = id[15] & id[10] & ~id[9]; // 1xxx_x10x__xxxx
wire id_aluop_test_Rr_n = &id[15:13] & id[10]; // 111x_x1xx__xxxx
wire id_aluop_Rr_n = &id[15:13]; // 111x_xxxx__xxxx
wire id_op_not_aluop = ~id[15] & ~((id_op_mul2_b & y[0]) | id_op_mix | id_op_mov_xchg_Hp_Rr_dst);
wire id_op_mix = ~|id[15:13] & id[12] & ~id[11] & id[10] & id[3];
wire id_op_mov_xchg_Hp_Rr_dst = ~|id[15:13] & id[12] & ~id[11] & id[9]; // 0001_0x1x__xxxx
wire id_aluop_A_Rr_Hp = &id[15:14] & ~id[13] & ~id[3]; // 110x_xxxx__0xxx
wire id_aluop_100 = id[15] & ~|id[14:13]; // 100x_xxxx__xxxx
wire id_aluop_Rr_A_Hp_A = &id[15:14] & ~|id[13:12] & id[3]; // 1100_xxxx__1xxx
wire id_aluop_notest_Hp_n = id[15] & ~id[14] & id[13] & ~id[12] & ~id[8]; // 1010_xxx0__xxxx
wire id_aluop_notest_Rr_n = &id[15:13] & ~id[11]; // 111x_0xxx__xxxx
wire id_op_mvi_Rr_n = ~id[15] & id[14] & ~id[13]; // 010x_xxxx__xxxx
wire id_op_mov_xchg_Hp_Rr_x = ~|id[15:14] & id[12] & id[9]; // 00x1_xx1x__xxxx
wire id_op_mov_xchg_mix_Rr_Hp = ~|id[15:13] & id[12] & ~id[11]; // 0001_0xxx__xxxx
wire id_aluop_Hp_n = id[15] & ~id[14] & id[13] & ~id[8]; // 101x_xxx0__xxxx
wire id_aluop_11x = &id[15:14]; // 11xx_xxxx__xxxx
wire id_aluop_notest_H_n = id[15] & ~id[14] & id[13] & ~id[12] & id[8]; // 1010_xxx1__xxxx
wire id_op_mvi_H_n = ~|id[15:14] & &id[13:11]; // 0011_1xxx__xxxx
wire id_op_set_H = ~|id[15:13] & id[12] & ~|id[11:10] & id[3]; // 0001_00xx__1xxx
wire id_op_call = ~id[15] & &id[14:12]; // 0111_xxxx__xxxx
wire id_op_retx = ~|id[15:12] & id[11]; // 0000_1xxx__xxxx
wire id_aluop_H_n = id[15] & ~id[14] & id[13] & id[8]; // 101x_xxx1__xxxx
wire id_get_h_t23 = ~|id[15:13] & id[12] & ~id[11] & id[1]; // 0001_0xxx__xx1x
wire id_op_in_pb = ~|id[15:11] & id[10] & ~id[8] & id[1]; // 0000_01x0__xx1x
wire id_op_in_pa = ~|id[15:11] & id[10] & ~id[8] & id[0]; // 0000_01x0__xxx1
wire id_aluop_1x1 = id[15] & id[13]; // 1x1x_xxxx__xxxx
wire id_op_src_n_a = ~id[15] & id[13]; // 0x1x_xxxx__xxxx
wire id_op_src_n_b = ~id[15] & id[14]; // 01xx_xxxx__xxxx
wire id_op_Rr_pdb8_4 = ~|id[15:13] & id[12] & ~(id[10] & ~id[3]); // 0001_xxxx__xxxx & ~xxxx_x1xx__0xxx
wire id_aluop_Rr_pdb8_4 = &id[15:14] & ~id[13] & ~id[0]; // 110x_xxxx__xxx0
wire id_op_out_pb = ~|id[15:10] & id[2] & ~id[0]; // 0000_00xx__x1x0
wire id_op_out_pa = ~|id[15:10] & id[1]; // 0000_00xx__xx1x


//////////////////////////////////////////////////////////////////////
// Control logic

logic cl_id_op_calln_d, cl_id_op_calln_dd;
logic cl_id_op_tbln_a_Rr_d, cl_id_op_tbln_a_Rr_dd;
logic cl_skip_p, cl_skip, cl_skip_s;
logic cl_id_op_tbln_X_Rr_d, cl_id_op_tbln_X_Rr_dd;
logic cl_id_op_tbln_Y_Rr_d, cl_id_op_tbln_Y_Rr_dd;
logic cl_tbln_PRr_odd, cl_tbln_PRr_odd_p;
logic [4:0] cl_pdb_ram_a;
logic [2:0] cl_sp_ram_a;

wire cl_pdb7_4_to_db11_8 = t45 & (id_op_jmp_n4 & ~testmode);
wire cl_pdb11_8_to_db11_8 = t45 & (resp | cl_t1 | (id_op_jmp_call & ~testmode));
wire cl_pdb_hi_sel = cl_tbln_PRr_odd & ~(pc_ctrl3 | cl_id_op_calln_dd);
wire cl_pdb15_8_to_db7_0 = t45 & cl_pdb_hi_sel;
wire cl_pdb7_0_to_db7_0 = ~cl_pdb_hi_sel & (cl_pdb11_8_to_db11_8 | (t23 & id_aluop_1x1) | (t45 & (id_op_src_n_a | id_op_src_n_b | id_aluop_100)));
wire cl_a_reg_en = ((cl_id_op_tbln_a_Rr_dd | id_aluop_A_Rr | id_op_reti | id_aluop_A_n | id_set_a) & phi1) | (id_op_mov_xchg_a_dst & t45);
wire cl_op_clear_skip = id_op_adi_sbi_adi5_Rr_n | id_op_xori | id_op_adi_andi_sbi_ori | id_op_not_aluop_or_rets;
wire cl_skip_test_if_set = ~(id_op_rets | id_aluop_10x_1x0_txnx | id_aluop_txnx_Rr_n);
wire cl_md0_reg_en = resp | cl_md1_reg_en | id_op_mvi_md0_n; // Yes, cl_md1_reg_en is actually in here.
wire cl_md1_reg_en = resp | id_op_mvi_md1_n | id_aluop_notest_md1_n;
wire cl_sp_to_ram_a = id_op_call | cl_int_load_pc | cl_id_op_calln_dd;
wire cl_spm1_to_ram_a = id_op_retx;
wire cl_pdb12_8_to_ram_a = id_op_mvi_Rr_n;
wire cl_pdb8_4_to_ram_a = id_aluop_Rr_n | id_op_Rr_pdb8_4 | id_aluop_Rr_pdb8_4;
wire cl_id_op_Hp_Rr_dst = id_aluop_Rr_A_Hp_A | id_aluop_notest_Hp_n | id_aluop_notest_Rr_n | id_op_mvi_Rr_n | id_op_mov_xchg_Hp_Rr_x;
wire cl_h_reg_en = philsi & ((phi1 & id_aluop_notest_H_n) | (t45 & (id_op_mvi_H_n | id_op_set_H)));
wire cl_a_to_db = (t23 & (id_get_a_t23_a | id_get_a_t23_b | id_get_a_t23_c)) | (t45 & id_get_a_t45);
wire cl_h_to_db = (t23 & id_get_h_t23) | (t45 & id_aluop_H_n);
wire cl_int_load_pc;
wire cl_alu_c_to_db = t68 & ~(id_op_in_pa | id_op_in_pb | id_op_jmp_call_n_t2 | cl_int_load_pc | cl_id_op_calln_dd | (id_op_jmp_call & ~testmode));
wire cl_pc_load_db11_0 = resp | id_op_retx | (id_op_jmpa & ~testmode) | ((id_op_jmpfz & ~testmode) & ~fl0) | (id_op_jmp_call & ~testmode) | cl_id_op_calln_dd | (id_op_jmp_n4 & ~testmode);
wire cl_id_op_out_pa_d = phi1 & id_op_out_pa;
wire cl_id_op_calln_retx = t45 & (id_op_tbln_calln & ~testmode | id_op_retx);
wire cl_id_Rr_Hp_t45 = t45 & (id_op_mov_xchg_mix_Rr_Hp | id_aluop_Hp_n | id_aluop_11x);
wire cl_n_reg_en = t45 & id_op_mov_N_a;
wire cl_a_in_lsr = id_op_mul2_b | id_op_mul1;
wire cl_a_in_ror_lsr = id_op_rar | cl_a_in_lsr;
wire cl_x_reg_en = t45 & (id_op_mov_X_RG | cl_id_op_tbln_X_Rr_dd);
wire cl_x_to_db = t23 & id_op_mul2;
wire cl_pnc1_to_db = t23 & id_op_mov_X_RG;
wire cl_tbln_PRr_odd_pp = t45 ? ~(pc_load_db12_1 & db[0]) : ~cl_tbln_PRr_odd_p;
wire cl_y_reg_en = cl_id_op_tbln_Y_Rr_dd | id_op_mov_Y_Rr;
wire cl_y_shift = id_op_muln;
wire cl_mix_sub = (ts ^ ns);
wire cl_ts_reg_en = t45 & cl_id_op_tbln_X_Rr_dd;
wire cl_alu_c4inh = (id_op_adims & ~md_64_32) | id_op_adi5;
wire cl_alu_c5inh = id_op_adims & md_64_32;
wire cl_op_tadi5_or_adims_and_not_md0 = (id_op_adims & ~md_64_32) | id_op_tadi5;

assign cl_pdb_ram_a[4:0] = cl_pdb12_8_to_ram_a ? pdb[12:8] : pdb[8:4];
assign cl_sp_ram_a = cl_spm1_to_ram_a ? (sp - 1'd1) : sp;

always @(posedge CLK) if (CKEN) begin
  if (phi1n) begin
    cl_id_op_calln_d <= id_op_calln & ~testmode;
    cl_id_op_tbln_a_Rr_d <= id_op_tbln_a_Rr;
    cl_skip_p <= cl_skip_pp;
    cl_id_op_tbln_X_Rr_d <= id_op_tbln_X_Rr;
    cl_id_op_tbln_Y_Rr_d <= id_op_tbln_Y_Rr;
  end
  if (philsi) begin
    cl_tbln_PRr_odd_p <= ~cl_tbln_PRr_odd_pp;
  end
  if (phi2p) begin
    cl_t1 <= id_op_tbln_calln;
    cl_id_op_calln_dd <= cl_id_op_calln_d;
    cl_id_op_tbln_a_Rr_dd <= cl_id_op_tbln_a_Rr_d;
    cl_skip <= cl_skip_p;
    cl_id_op_tbln_X_Rr_dd <= cl_id_op_tbln_X_Rr_d;
    cl_id_op_tbln_Y_Rr_dd <= cl_id_op_tbln_Y_Rr_d;
    cl_tbln_PRr_odd <= cl_tbln_PRr_odd_p;
  end
end

// Instruction skip logic
wire cl_skip_pp = id_op_reti ? cl_skip_s : cl_skip_a;
wire cl_skip_test_out =
     ((id_aluop_Rr_n_not_c5 | id_aluop_10x_1x0_sk_cb) & alu_co) |
     (id_aluop_10x_1x0_sk_z & alu_c_zero) |
     ((id_op_adims & md_64_32) & alu_co5) |
     (cl_op_tadi5_or_adims_and_not_md0 & alu_co4);
wire cl_skip_test_failed = cl_skip_test_out ^ cl_skip_test_if_set;
wire cl_skip_a = ~(cl_op_clear_skip | cl_skip_test_failed);

always @(posedge CLK) if (phi1n) begin
  if (cl_int_load_pc)
    cl_skip_s <= cl_skip;
end


//////////////////////////////////////////////////////////////////////
// Arithmetic/Logic Unit

logic [7:0] alu_a;              // input A (from TEMP1)
logic [7:0] alu_b;              // input B (from TEMP2)
logic [7:0] alu_c;              // output (S)
logic       alu_co, alu_co4, alu_co5;
wire        alu_c_zero;

wire alu_zero_a_lo = id_op_not_aluop;
wire alu_zero_a_hi = alu_zero_a_lo | id_aluop_Rr_n;
wire alu_zero_b = id_op_mov_xchg_Hp_Rr_dst;
wire alu_op_sub = id_op_mix ? cl_mix_sub : (id_aluop_sb | id_aluop_test_Rr_n);
wire alu_op_no_carry = id_aluop_or_xor_equ;
wire alu_op_or = id_aluop_or;
wire alu_op_and = id_aluop_and;

wire alu_temp1_first = ~(id_aluop_A_Rr_Hp | id_aluop_100);
wire alu_db_to_temp1 = (t23 & alu_temp1_first) | (t45 & ~alu_temp1_first);
wire alu_db_to_temp2 = (t23 & ~alu_temp1_first) | (t45 & alu_temp1_first);

always @(posedge CLK) if (CKEN) begin
  // alu_db_to_temp1/2 are high before and during philsi. And philsi
  // drives TEMP1/2 to A/B.
  if (philsi) begin
    if (alu_db_to_temp1)
      alu_a <= db[7:0];
    else if (alu_db_to_temp2)
      alu_b <= db[7:0];
  end
  if (alu_zero_a_lo)
    alu_a[3:0] <= '0;
  if (alu_zero_a_hi)
    alu_a[7:4] <= '0;
  if (alu_zero_b)
    alu_b <= '0;
end

always @* begin
  alu_co = '0;
  alu_co4 = '0;
  alu_co5 = '0;
  if (alu_op_no_carry) begin
    if (alu_op_or)
      alu_c = alu_a | alu_b;
    else
      alu_c = alu_a ^ alu_b;
  end
  else begin
    if (alu_op_sub)
      {alu_co, alu_c} = alu_b - alu_a;
    else if (alu_op_and)
      alu_c = alu_a & alu_b;
    else begin
      // alu_co[5:4] are only used in certain add ins.
      if (cl_alu_c4inh) begin
        {alu_co4, alu_c[4:0]} = alu_a[4:0] + alu_b[4:0];
        {alu_co, alu_c[7:5]} = alu_a[7:5] + alu_b[7:5];
      end
      else if (cl_alu_c5inh) begin
        {alu_co5, alu_c[5:0]} = alu_a[5:0] + alu_b[5:0];
        {alu_co, alu_c[7:6]} = alu_a[7:6] + alu_b[7:6];
      end
      else
        {alu_co, alu_c} = alu_a + alu_b;
    end
  end
end

assign alu_c_zero = alu_c == '0;


//////////////////////////////////////////////////////////////////////
// pc: Program counter

logic [11:0] pc, pcin, pcn;
logic [11:0] abn, abadd;

wire pc_ctrl1 = t45 & ~(id_op_tbln_calln & ~testmode | cl_int_load_pc | cl_pc_load_db11_0);
wire pc_ctrl3 = ~cl_t1;
wire pc_ctrl4 = ~id_op_jmp_n4;
wire pc_load_int_vec = t45 & cl_int_load_pc;
wire pc_load_db11_0 = t45 & cl_pc_load_db11_0;
wire pc_load_db12_1 = t45 & id_op_tbln_calln & ~testmode;
wire pc_abadd_cin = ~cl_int_load_pc;
wire pc_pcin_to_abn = t45 & pc_ctrl4;
wire pc_out_to_db = t68 & (cl_int_load_pc | (id_op_call & ~testmode) | cl_id_op_calln_dd);

always @(posedge CLK) if (philsi) begin
  pc <= pcn;
  ab <= abn;
end

always @* begin
  pcn = (t23 & pc_ctrl3) ? abadd : pc;
  abn = (t45 & pc_pcin_to_abn) ? pcin : ab;
end

always @(posedge CLK) if (CKEN) begin
  if (pc_load_db11_0)
    pcin <= db[11:0];
  else if (pc_load_db12_1)
    pcin <= db[12:1];
  else if (pc_load_int_vec)
    pcin <= int_vec;
  else if (pc_ctrl1)
    pcin <= pc;
end

always_comb
  abadd = ab + {11'b0, pc_abadd_cin};


//////////////////////////////////////////////////////////////////////
// Program ROM

logic [15:0] rom [1 << 9];
logic [15:0] rom_rbuf, rom_do;
wire         rom_oe, rom_zero;
`ifndef UPD1771C_ROM_INIT_FROM_HEX
logic [7:0]  rom_init_data_lo, rom_init_data_hi;
`endif

`ifdef UPD1771C_ROM_INIT_FROM_HEX
initial begin
  $readmemh("upd1771c_rom.hex", rom);
end
`endif

// Change rom_zero from silicon, to clear PC even in testmode
//assign rom_zero = ~(testmode | ~(resp | cl_int_load_pc | cl_skip));
assign rom_zero = resp | (~testmode & (cl_int_load_pc | cl_skip));
assign rom_oe = ~leader;

always @(posedge CLK) if (CKEN) begin
  rom_rbuf <= rom[ab[8:0]];
end

`ifndef UPD1771C_ROM_INIT_FROM_HEX
// Convert incoming 8-bit data to 16-bit writes. Data are in
// little-endian order (ROM low byte first).
always @(posedge CLK) begin
  if (INIT_SEL & INIT_VALID & ~INIT_ADDR[0])
    rom_init_data_lo <= INIT_DATA;
end
always @* rom_init_data_hi = INIT_DATA;

always @(posedge CLK) begin
  if (INIT_SEL & INIT_VALID & INIT_ADDR[0])
    rom[INIT_ADDR[9:1]] = {rom_init_data_hi, rom_init_data_lo};
end
`endif

always @(posedge CLK) if (phi2p) begin
  rom_do <= (|ab[11:9]) ? '0 : rom_rbuf;
end

assign pdb = (rom_oe & ~rom_zero) ? rom_do : '0;


//////////////////////////////////////////////////////////////////////
// CPU-programmable registers

logic [9:0] md;                 // Mode Register
logic [7:0] a, ap;              // Accumulator
logic [7:0] sa;                 // Shadow Accumulator (A')
logic [5:0] h;                  // Data Pointer
logic [2:0] sp;                 // Stack Pointer
logic [6:0] x;                  // X Multiplier Register
logic [4:0] y, yp;              // Y Multiplier Register

initial begin
  h = '0; // else ram_col[1] is X
  ts = '0; // else pcm_neg / PCM_OUT is X
end

always @(posedge CLK) if (CKEN) begin
  if (cl_md0_reg_en)
    {md[9], md[8], md[4:0]} <= db[6:0];
  if (cl_md1_reg_en)
    md[7:5] <= {(resp | db[7]), db[6:5]};
end

assign md_64_32 = md[0];
assign md_tone_ie = md[1];
assign md_ns_ie = md[2];
assign md_nss = md[3];
assign md_time_ie = md[4];
assign md_ext_ie = md[5];
assign md_out = md[6];
assign md_if = md[7];

// A is copied to A' on interrupt, and copied back on RETI.
always @(posedge CLK) if (phi1n) begin
  if (cl_int_load_pc)
    sa <= a;
end

always @* begin
  ap = db[7:0];
  if (cl_a_in_ror_lsr) begin
    ap = {ap[0], ap[7:1]};
    if (cl_a_in_lsr)
      ap[7] = '0;
  end
  else if (id_op_ral)
    ap = {ap[6:0], ap[7]};
  else if (id_op_reti)
    ap = sa;
end

always @(posedge CLK) if (CKEN) begin
  if (cl_a_reg_en)
    a <= ap;
end

always @(posedge CLK) if (CKEN) begin
  if (cl_h_reg_en)
    h <= db[5:0];
end

always @(posedge CLK) if (phi2p) begin
  if (res)
    sp <= 0;
  else if (cl_sp_to_ram_a)
    sp <= sp + 1'd1;
  else if (cl_spm1_to_ram_a)
    sp <= sp - 1'd1;
end

always @(posedge CLK) if (philsi) begin
  if (cl_x_reg_en)
    x <= db[6:0];
  if (cl_ts_reg_en)
    ts <= db[7];
end

always @(posedge CLK) if (CKEN) begin
  if (phi1n) begin
    if (cl_y_reg_en)
      yp <= db[4:0];
    else if (cl_y_shift)
      yp <= y >> 1;
    else
      yp <= y;
  end
  if (phi2p)
    y <= yp;
end

logic fl0_p;
always @(posedge CLK) if (CKEN) begin
  if (phi2p)
    fl0 <= fl0_p;
  if (phi1n)
    fl0_p <= fl0 & ~(resp | (id_op_jmpfz & ~testmode)) | id_op_stf;
end


//////////////////////////////////////////////////////////////////////
// 64 byte SRAM: 32 columns x 16 rows

// Address inputs
logic [5:0] ram_addr;           // Combined row/column select

always @* begin
  ram_addr = 'X;
  if (cl_spm1_to_ram_a | cl_sp_to_ram_a)
    ram_addr = { 2'b10, cl_sp_ram_a, 1'b0 };
  else if (cl_pdb12_8_to_ram_a | cl_pdb8_4_to_ram_a)
    ram_addr = { 1'b0, cl_pdb_ram_a };
  else /*if (cl_h_to_ram_a)*/
    ram_addr = h;
end

wire [3:0] ram_row;             // Row address select
wire [1:0] ram_col;             // Column address select

assign ram_row[3] = ram_addr[5];
assign ram_col[0] = ram_addr[4];
assign ram_row[2:0] = ram_addr[3:1];
assign ram_col[1] = ram_addr[0];

wire ram_lr_col_sel = ram_col[0]; // left/right col. pair select: 0=left
wire ram_left_sel = ~ram_col[1];  // left array select
wire ram_right_sel = ram_col[1];  // right array select

// Left half of array (cols 0-15): R/W port == db[7:0]
logic [7:0] ram_left_mem [2][16];
logic [7:0] ram_left_rbuf;
logic [7:0] ram_left_wbuf;
wire        ram_left_write_en;
wire        ram_left_db_oe;

always @(posedge CLK) if (CKEN) begin
  if (ram_left_write_en)
    ram_left_mem[ram_lr_col_sel][ram_row] <= ram_left_wbuf;
  ram_left_rbuf <= ram_left_mem[ram_lr_col_sel][ram_row];
end

assign ram_left_write_en = phi1 & (~ram_right_db7_0_ie | (cl_id_op_Hp_Rr_dst & ram_left_sel));
assign ram_left_db_oe = cl_id_op_calln_retx | (ram_left_sel & cl_id_Rr_Hp_t45);

assign ram_left_wbuf = db[7:0];

// Right half of array (cols 16-31): R/W port == db[7:0] or db[15:8]
logic [7:0] ram_right_mem [2][16];
logic [7:0] ram_right_rbuf;
logic [7:0] ram_right_wbuf;
wire        ram_right_write_en;
wire        ram_right_db7_0_ie;
wire        ram_right_db7_0_oe;
wire        ram_right_db15_8_oe;

always @(posedge CLK) if (CKEN) begin
  if (ram_right_write_en)
    ram_right_mem[ram_lr_col_sel][ram_row] <= ram_right_wbuf;
  ram_right_rbuf <= ram_right_mem[ram_lr_col_sel][ram_row];
end

assign ram_right_write_en = phi1 & (~ram_right_db7_0_ie | (cl_id_op_Hp_Rr_dst & ram_right_sel));
assign ram_right_db7_0_ie = ~cl_sp_to_ram_a;
assign ram_right_db7_0_oe = ram_right_sel & cl_id_Rr_Hp_t45;
assign ram_right_db15_8_oe = t45 & ((id_op_tbln_calln & ~testmode) | id_op_retx);
assign ram_right_wbuf = ram_right_db7_0_ie ? db[7:0] : db[15:8];


//////////////////////////////////////////////////////////////////////
// Tone counter

logic [7:0] nc;                 // "NC": Down-counter
logic [7:0] n;                  // "N": Reload value for NC
logic [2:0] nuc;                // NC reload / tone counter

wire nc_eq_01 = nc == 8'd1;     // reload condition
logic nc_eq_01_d;
logic int_tone_cond;
logic int_tone_trig, int_tone_cond_d;

always @(posedge CLK) if (philsi) begin
  n <= cl_n_reg_en ? db[7:0] : n;
end

always @(posedge CLK) if (phi2p) begin
  if (res)
    nc <= 0;
  else if (nc_eq_01)
    nc <= n;
  else
    nc <= nc - 1'd1;
end

// nuc always increments on NC reload.
always @(posedge CLK) if (phi2p) begin
  nc_eq_01_d <= nc_eq_01;

  if (res)
    nuc <= 0;
  else if (nc_eq_01_d)
    nuc <= nuc + 1'd1;
end

// The tone interrupt is driven by select nuc bits or the reload
// signal, depending on the value of N.
wire n_range_00_07 = (n < 8'h08);
wire n_range_08_0f = (n >= 8'h08) & (n < 8'h10);
wire n_range_10_1f = (n >= 8'h10) & (n < 8'h20);
wire n_range_20_3f = (n >= 8'h20) & (n < 8'h40);
wire n_range_40_ff = (n >= 8'h40);

always @* begin
  case (1'b1)
    n_range_00_07: int_tone_cond = 1'b0;
    n_range_08_0f: int_tone_cond = nuc[2];
    n_range_10_1f: int_tone_cond = nuc[1];
    n_range_20_3f: int_tone_cond = nuc[0];
    n_range_40_ff: int_tone_cond = nc_eq_01_d;
    default: int_tone_cond = 1'bx;
  endcase
end

always @(posedge CLK) if (phi2p) begin
  int_tone_cond_d <= int_tone_cond;
  // Interrupt is triggered on int_tone_cond negedge.
  int_tone_trig <= ~int_tone_cond & int_tone_cond_d;
end


//////////////////////////////////////////////////////////////////////
// Timer up-counter, NS interrupt

logic [8:0] tc;                 // timer counter
logic       int_ns_cond;
logic       int_time_cond, int_time_cond_d, int_time_trig;
logic       ns_tc_sel_p, ns_tc_sel;

always @(posedge CLK) if (phi2p) begin
  if (res) begin
    tc <= 0;
  end
  else begin
    // Up-counter tc always runs
    tc <= tc + 1'd1;
  end
end

// 'Time' interrupt is triggered on tc[8] negedge.
assign int_time_cond = tc[8];
always @(posedge CLK) if (phi2p) begin
  int_time_cond_d <= int_time_cond;
end
assign int_time_trig = ~int_time_cond & int_time_cond_d;

// md[9:8] set the 'NS' interrupt trigger rate by selecting a TC bit.
always @* begin
  case (md[9:8])
    2'b00: ns_tc_sel_p = tc[8];
    2'b01: ns_tc_sel_p = tc[7];
    2'b10: ns_tc_sel_p = tc[5];
    2'b11: ns_tc_sel_p = tc[4];
    default: ns_tc_sel_p = 'X;
  endcase
end

always @(posedge CLK) if (phi2p) begin
  ns_tc_sel <= ns_tc_sel_p;
end

// NS interrupt is triggered on falling edge of the selected TC bit.
wire int_ns_trig = ~ns_tc_sel_p & ns_tc_sel;

// The noise generator and RG LFSR are enabled by the NS interrupt trigger.
wire pnc_en = int_ns_trig | res;


//////////////////////////////////////////////////////////////////////
// PNC1, PNC2: Polynomial (?) noise generator LFSRs

logic [6:0] pnc1;
logic [2:0] pnc2;

initial begin // else short RESB means pnc* is X
  pnc1 = '0;
  pnc2 = '0;
end

// PNC1/2 advance on NS interrupt trigger.
always @(posedge CLK) if (phi2p) begin
  if (pnc_en) begin
    pnc1[0] <= ~res & ~(pnc1[5] ^ pnc1[6]);
    pnc1[6:1] <= pnc1[5:0];

    pnc2[0] <= ~res & ~(md_nss ? pnc2[0] : (pnc2[1] ^ ns));
    pnc2[2:1] <= pnc2[1:0];
  end
end

// Clearing NSS immediately forces PNC2[2] / NS to zero.
assign ns = md_nss & pnc2[2];


//////////////////////////////////////////////////////////////////////
// DAC Output

// SS flag is updated by MIX.
always @(posedge CLK) if (CKEN) begin
  if (phi1)                 // needs to update before id_op_out_da ends
    if (id_op_mix)
      ss <= ts ^ alu_co;
end

always @(posedge CLK) if (phi1n) begin
  if (id_op_out_da) begin
    da <= {ss ^ ts, ss, db[7:0]};
  end
end

// pcm_neg, pcm_out: Input to DAC
logic       pcm_neg;
logic [7:0] pcm_out;
always @* begin
  pcm_out = da[7:0];
  pcm_neg = da[8];
  // DA[9] inverts sample input to DAC.
  if (da[9])
    pcm_out = ~pcm_out;
end

// Convert to 9-bit two's complement
assign PCM_OUT[8] = pcm_neg;
assign PCM_OUT[7:0] = pcm_neg ? ~pcm_out : pcm_out;


//////////////////////////////////////////////////////////////////////
// Interrupts

logic [3:0] int_pending, int_pending_d, int_active;
wire [3:0]  int_set_pend;
logic [3:0] int_set_active_prio;
logic [3:0] int_load_pc;

typedef enum reg [1:0]
{
    II_TONE = 2'd0,
    II_NS   = 2'd1,
    II_EXT  = 2'd2,
    II_TIME = 2'd3
} e_int_id;

assign int_set_pend[II_TONE] = int_tone_trig & md_tone_ie;
assign int_set_pend[II_NS]   = int_ns_trig & md_ns_ie;
assign int_set_pend[II_EXT]  = '0;
assign int_set_pend[II_TIME] = int_time_trig & md_time_ie;

always @* begin
  int_pending = int_pending_d;

  if (resp) begin
    int_pending = '0;
  end
  else begin
    if (phi1) begin
      // Set inactive interrupt as pending
      int_pending |= (int_set_pend & ~int_active);
    end

    // Clear pending on going active
    int_pending &= ~int_active;
  end
end

always @(posedge CLK) if (CKEN) begin
  int_pending_d <= int_pending;
end

// Only one interrupt can be active at a time
wire [3:0] int_set_active = |int_active ? '0 : int_pending;
wire [3:0] int_clr_active;

assign int_clr_active[II_TONE] = id_op_reti_tone;
assign int_clr_active[II_NS]   = id_op_reti_ns;
assign int_clr_active[II_EXT]  = id_op_reti_ext;
assign int_clr_active[II_TIME] = id_op_reti_time;

// Priority selection: lowest index is highest priority
always @* begin
  int_set_active_prio = '0;
  if (~|int_active) begin
    case (1'b1)
      int_set_active[0]: int_set_active_prio[0] = '1;
      int_set_active[1]: int_set_active_prio[1] = '1;
      int_set_active[2]: int_set_active_prio[2] = '1;
      int_set_active[3]: int_set_active_prio[3] = '1;
    endcase
  end
end

// Set highest priority pending interrupt as active.
// Clear active on RETI
wire [3:0] int_active_p = (int_active | int_set_active_prio) & ~int_clr_active;

always @(posedge CLK) if (CKEN) begin
  if (resp) begin
    int_active <= '0;
    int_load_pc <= '0;
  end
  else if (phi2p) begin
    int_active <= int_active_p;

    // When an interrupt goes active, load PC with its interrupt vector.
    int_load_pc <= int_active_p & ~int_active;
  end
end

assign cl_int_load_pc = ~testmode & |int_load_pc;

always @* begin
  int_vec = '0;
  if (int_load_pc[II_TONE]) begin
    int_vec |= 'h20;
    case (1'b1)
      n_range_10_1f: int_vec |= 'h04;
      n_range_20_3f: int_vec |= 'h08;
      n_range_40_ff: int_vec |= 'h0c;
      default: ;
    endcase
  end
  if (int_load_pc[II_NS])
    int_vec |= 'h48;
  if (int_load_pc[II_EXT])
    int_vec |= 'h60;
  if (int_load_pc[II_TIME])
    int_vec |= 'h80;
end


//////////////////////////////////////////////////////////////////////
// I/O ports

logic [7:0] pai, pbi;
logic [7:0] pao, pbo;
logic [7:0] pbi_reg;
logic [7:0] pao_reg, pbo_reg;
logic [7:0] pao_reg_d, pbo_reg_p;
logic [7:0] pai_d;

wire io_pabx_pad_pdb_pax_db_out_en, io_pabx_out_reg_pad_en,
     io_pax_out_pbx_pdb_pad_en, io_pax_pdb_out_en, io_pax_db_out_reg_en;
wire io_pax_out_reg_latch_en;
wire io_pbx_out_reg_ce;
wire io_pax_noe, io_pbx_noe, io_pbx7_3_noe, io_pbx2_0_noe;
wire io_pax_out_reg_db_en, io_pbx_pad_db_en;
wire io_ext_cs, io_ext_write, io_ext_write_latch, io_ext_read, io_ext_read_latch;

assign io_pabx_pad_pdb_pax_db_out_en = leader;
assign io_pabx_out_reg_pad_en = ~testmode;
assign io_pax_out_pbx_pdb_pad_en = ~io_pabx_out_reg_pad_en;
assign io_pax_pdb_out_en = ~io_pabx_pad_pdb_pax_db_out_en;
assign io_pax_out_reg_db_en = id_op_in_pa & t68;
assign io_pbx_pad_db_en = id_op_in_pb & t68;
assign io_pax_noe = ~(io_ext_read_latch | leader);
assign io_pbx_noe = ~((md_out & follower) | testmode);
assign io_pbx7_3_noe = io_pbx_noe;
assign io_pbx2_0_noe = io_pbx_noe & ~leader & md_if;

assign io_ext_cs = ~pbi[7];
assign io_ext_write = ~(io_pbx7_3_noe ? pbi[6] : pbo_reg[5]);
assign io_ext_read = ~(io_pbx7_3_noe ? pbi[5] : pbi[6]);
assign io_ext_write_latch = follower & md_if & io_ext_cs & io_ext_write;
assign io_ext_read_latch = follower & md_if & io_ext_cs & io_ext_read;

assign io_pax_db_out_reg_en = ~io_ext_write_latch;
assign io_pax_out_reg_latch_en = cl_id_op_out_pa_d | io_ext_write_latch;
assign io_pbx_out_reg_ce = id_op_out_pb;

always @(posedge CLK) if (CKEN) begin
  // To break circular logic: pao -> PA_O -> pai -> pao_reg -> pao
  pai_d <= pai;

  pao_reg_d <= pao_reg;

  if (phi1n)
    pbo_reg_p <= io_pbx_out_reg_ce ? db[7:0] : pbo_reg;
  if (phi2p)
    pbo_reg <= pbo_reg_p;
end

always @* begin
  pao_reg = pao_reg_d;
  if (io_pax_out_reg_latch_en)
    pao_reg = io_pax_db_out_reg_en ? db[7:0] : pai_d;
end

assign pai = (PA_OE & PA_O) | (~PA_OE & PA_I);
assign pbi = (PB_OE & PB_O) | (~PB_OE & PB_I);

assign pao = io_pabx_out_reg_pad_en ? pao_reg :
             (io_pax_pdb_out_en ? pdb[15:8] : db[7:0]);
assign pbo = io_pabx_out_reg_pad_en ? pbo_reg : pdb[7:0];

assign PA_O = pao;
assign PB_O = pbo;
assign PA_OE = {8{~io_pax_noe}};
assign PB_OE = {{5{~io_pbx7_3_noe}}, {3{~io_pbx2_0_noe}}};


//////////////////////////////////////////////////////////////////////
// Internal buses

logic [15:0]    db_d;

// db: Data bus
always @* begin
  db = db_d;
  if (cl_pdb15_8_to_db7_0)  db[7:0] = pdb[15:8];
  if (cl_pdb7_0_to_db7_0)   db[7:0] = pdb[7:0];
  if (pc_out_to_db)         db[7:0] = pc[7:0];
  //if (n699)                 db[7:0] = md[7:0];
  if (cl_a_to_db)           db[7:0] = a;
  if (cl_h_to_db)           db[7:0] = {2'b0, h};
  if (cl_x_to_db)           db[7:0] = {1'b0, x};
  if (cl_pnc1_to_db)        db[7:0] = {1'b0, pnc1};
  if (cl_alu_c_to_db)       db[7:0] = alu_c;
  if (ram_left_db_oe)       db[7:0] = ram_left_rbuf;
  if (ram_right_db7_0_oe)   db[7:0] = ram_right_rbuf;
  if (io_pax_out_reg_db_en) db[7:0] = pao_reg_d[7:0];
  if (io_pbx_pad_db_en)     db[7:0] = pbi_reg[7:0];

  if (cl_pdb7_4_to_db11_8)  db[11:8] = pdb[7:4];
  if (cl_pdb11_8_to_db11_8) db[11:8] = pdb[11:8];
  if (pc_out_to_db)         db[11:8] = pc[11:8];
  if (ram_right_db15_8_oe)  db[15:8] = ram_right_rbuf;
end

always @(posedge CLK) if (CKEN) begin
  db_d <= db;
end


endmodule
