// NEC uPD7800 - a trivial implementation
//
// Copyright (c) 2024 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// . https://gist.githubusercontent.com/BlockoS/4c4ae33f7571ec48a4b2d3c264aee9f1/raw/80eff532a457ad2cef4dbc78462af69fe8a1efb3/uPD78c06_Instruction_Set.txt
// . https://forums.bannister.org/ubbthreads.php?ubb=showflat&Number=38637 - opcode list, other details
// . https://github.com/mamedev/mame/blob/master/src/devices/cpu/upd7810/upd7810.cpp - MAME's emulation
// . http://www.bitsavers.org/components/nec/_dataBooks/1981_NEC_Microcomputer_Catalog.pdf - includes uDP7801 data sheet, many other chips
// . http://takeda-toshiya.my.coocan.jp/scv/index.html - EPOCH Super Cassette Vision emulator for Win32 / uPD7801 tiny disassembler
// . http://takeda-toshiya.my.coocan.jp/common/index.html - Common Source Code Project - eSCV emulator

// TODO:
// . Unimplemented instructions -- see gen-ucode.py


`timescale 1us / 1ns

module upd7800
  (
   input         CLK,
   input         CP1_POSEDGE, // clock phase 1, +ve edge
   input         CP1_NEGEDGE, //  "             -ve edge
   input         CP2_POSEDGE, // clock phase 2, +ve edge
   input         CP2_NEGEDGE, //  "             -ve edge
   input         RESETB, // reset (active-low)
   input         INT0,
   input         INT1,
   input         INT2,
   output [15:0] A,
   input [7:0]   DB_I,
   output [7:0]  DB_O,
   output        DB_OE,
   input         WAITB, // extend T2 request (memory not ready)
   output        M1, // opcode fetch cycle
   output        RDB, // read
   output        WRB, // write
   output [7:0]  PA_O, // Port A
   input [7:0]   PB_I, // Port B
   output [7:0]  PB_O,
   output [7:0]  PB_OE,
   input [7:0]   PC_I, // Port C
   output [7:0]  PC_O,
   output [7:0]  PC_OE
   );

`define psw_z  psw[6]           // Zero
`define psw_sk psw[5]           // Skip
`define psw_hc psw[4]           // Half Carry
`define psw_l1 psw[3]           // Byte instruction string effect
`define psw_l0 psw[2]           // Word instruction string effect
`define psw_cy psw[0]           // Carry

`define IR_SOFTI    11'h072

`include "uc-types.svh"

typedef enum reg [2:0]
{
    II_INT0 = 3'd0,             // priority 2
    II_INTT = 3'd1,             // priority 3
    II_INT1 = 3'd2,             // priority 4
    II_INT2 = 3'd3,             // priority 5
    II_INTS = 3'd4              // priority 6
} e_int_idx;

wire         resp;
wire         cp1p, cp1n, cp2p, cp2n;
wire [7:0]   pcl, pch;
wire [7:0]   pboe, pcoe;
wire         irf1;
wire [7:0]   pai, pbi, pci;
wire         tc_uf;
wire         t2_wait;

reg          resg;
reg [3:0]    intv1, intv2;
reg [4:0]    intp, intpr, intps, intpm, iack, imsk;
reg [4:0]    irfm;              // Mask selected by SK(N)IT operand
reg          intg;
reg [7:0]    intva;
reg          cp2;
reg [7:0]    v, a, b, c, d, e, h, l;
reg [7:0]    v2, a2, b2, c2, d2, e2, h2, l2; // secondary GP reg. set
reg [7:0]    w;
reg [7:0]    psw;
reg [15:0]   sp, pc;
reg [10:0]   ir;
reg          ie;                // interrupt enable flag
reg [7:0]    mk;                // interrupt mask reg.
reg [7:0]    mb, mc, pao, pbo, pco;
reg [11:0]   tm;
reg [14:0]   tc;           // lower 3 bits are prescaler
reg [15:0]   aor;
reg [7:0]    dor;
reg          rd_ext, wr_ext;
reg [7:0]    rfo, spro, idl, idl_d, idb;
reg [15:0]   ab;
reg [7:0]    ai, bi, ibi, co;
reg [7:0]    alu_co;
reg          alu_cho, alu_cco;
reg          addc, notbi, pdah, pdal, cco, cho;
reg [15:0]   uabi, nabi;
reg          skso;
reg [7:0]    sdg;
reg [1:0]    sefso;
reg          t2_wait_d;

reg [3:0]    oft;
reg [2:0]    of_prefix;
wire         of_start, of_start_d, of_done, of_pc_inc, of_no_skip;
reg          m1, m1ext;
wire         m1_next, oft0_next;
wire         m1_overlap;

s_ird        ird;

s_uc         ucp, uc;
e_uaddr      uptr_p, uptr_next, uptr;
wire         uc_done;
s_nc         ncp, nc;
t_naddr      nptr;

reg          cl_skip;
reg          cl_idb_psw, cl_co_z, cl_cco_c, cl_zero_c, cl_one_c, cl_cho_hc;
reg          cl_sks_sk;
e_sks        cl_pswsk;
reg          cl_sefs_psw;
e_sefm       cl_sefm;
reg          cl_zero_irf;
reg [2:0]    cl_irf;
e_lts        cl_lts;
e_abs        cl_abs, cl_abits;
reg          cl_idb_pcl, cl_idb_pch, cl_pc_inc, cl_pc_dec;
reg          cl_abi_sp, cl_abi_pc;
reg          cl_idb_ir, cl_of_prefix_ir;
reg          cl_ui_ie;
reg          cl_abl_aor, cl_abh_aor, cl_ab_aor;
reg          cl_idb_dor, cl_store_dor;
reg          cl_pre_load_db, cl_load_db;
e_urfs       cl_rfos, cl_rfts;
e_spr        cl_spr;
e_idbs       cl_idbs;
reg          cl_abi_inc, cl_abi_dec;
reg          cl_rpir_inc, cl_rpir_dec;
reg          cl_idb_abil, cl_idb_abih;
reg          cl_carry, cl_one_addc, cl_c_addc, cl_zero_bi,
             cl_bi_not, cl_bi_daa;
reg          cl_clrs, cl_sums, cl_subs, cl_incs, cl_decs, cl_ors, cl_ands,
             cl_eors, cl_lsls, cl_rols, cl_lsrs, cl_rors, cl_dils, cl_dihs,
             cl_diss;
reg          cl_stm;


//////////////////////////////////////////////////////////////////////
// Clocking

assign cp1p = CP1_POSEDGE;
assign cp1n = CP1_NEGEDGE;
assign cp2p = CP2_POSEDGE;
assign cp2n = CP2_NEGEDGE;

initial begin
  cp2 = 0;
end

always_ff @(posedge CLK) begin
  cp2 <= (cp2 & ~cp2n) | cp2p;
end


//////////////////////////////////////////////////////////////////////
// Reset and interrupts

initial begin
  resg = 1'b1;
  intv1 = 0;
  intv2 = 0;
  intp = 0;
end

assign resp = ~RESETB;

always_ff @(posedge CLK) begin
  if (cp2n) begin
    resg <= resp;
  end
end

// Interrupt sampling: Edge-triggered INT1/2 are sampled at
// CP1. Interrupt deemed asserted after 3 consecutive asserted samples
// following at least 1 deasserted sample.

always_ff @(posedge CLK) begin
  if (cp1n) begin
    intv1 <= {intv1[2:0], INT1};
    intv2 <= {intv2[2:0], INT2 ^ ~mk[5]};
  end
end

always @* begin
  intps = 0;
  if (INT0)
    intps[II_INT0] = ~intp[II_INT0];
  if (cp1n & (intv1 == 4'b0111))
    intps[II_INT1] = 1'b1;
  if (cp1n & (intv2 == 4'b0111))
    intps[II_INT2] = 1'b1;
  if (tc_uf)
    intps[II_INTT] = 1'b1;
end

always @* begin
  intpr = 0;
  if (~INT0)
    intpr[II_INT0] = intp[II_INT0];
  // End of interrupt processing
  if (cp2n & intg & of_start) begin
    intpr |= iack;
  end
  if (cp2n & cl_zero_irf) begin
    intpr |= irfm;
  end
end

always_ff @(posedge CLK) begin
  if (resg) begin
    intp <= 0;
  end
  else begin
    intp <= (intp & ~intpr) | intps;
  end
end

always_ff @(posedge CLK) begin
  if (resg) begin
    intpm <= 0;
  end
  else begin
    if (cp2n & of_start) begin
      // Latch interrupt state just before M1 (opcode fetch)
      intpm <= intp & imsk;     // pending and non-masked
    end
  end
end

// Interrupt priority encoder
always @* begin
  casez (intpm)
    5'bzzzz1: iack = 5'b00001;
    5'bzzz10: iack = 5'b00010;
    5'bzz100: iack = 5'b00100;
    5'bz1000: iack = 5'b01000;
    5'b10000: iack = 5'b10000;
    default: iack = 0;
  endcase
end

// imsk: Set to enable
always @* begin
  imsk[II_INT0] = ie & ~mk[0];
  imsk[II_INTT] = ie & ~mk[1];
  imsk[II_INT1] = ie & ~mk[2];
  imsk[II_INT2] = ie & ~mk[3];
  imsk[II_INTS] = ie & ~mk[4];
end

always @* begin
  case (1'b1)
    iack[II_INT0]: intva = 8'h04;
    iack[II_INTT]: intva = 8'h08;
    iack[II_INT1]: intva = 8'h10;
    iack[II_INT2]: intva = 8'h20;
    iack[II_INTS]: intva = 8'h40;
    default: intva = 8'h60;     // SOFTI
  endcase
end

always_comb intg = |intpm;

// Interrupt flag selected by SK(N)IT instruction operand
always_comb begin
  irfm = 0;
  irfm[cl_irf] = cl_zero_irf;
end

assign irf1 = |(irfm & intp);


//////////////////////////////////////////////////////////////////////
// External interface

initial begin
  rd_ext = 1'b1;
  wr_ext = 1'b1;
end

// WAITB is sampled in T2 on CP2 falling. If asserted, the machine
// stays in T2.
assign t2_wait = (cl_load_db | cl_store_dor) & ~WAITB;

always_ff @(posedge CLK) begin
  if (resg)
    t2_wait_d <= 0;
  else if (cp2n)
    t2_wait_d <= t2_wait;
end

always_ff @(posedge CLK) begin
  if (resg) begin
    rd_ext <= 0;
    wr_ext <= 0;
  end
  else begin
    // RDB asserts in T1 @ CP2 rising, de-asserts in T3 @ CP2 rising
    // WRB asserts in T2 @ CP1 rising, de-asserts in T3 @ CP2 rising
    if (cp1p) begin
      wr_ext <= wr_ext | cl_store_dor;
    end
    else if (cp2p) begin
      rd_ext <= cl_pre_load_db | cl_load_db;
      wr_ext <= wr_ext & cl_store_dor;
    end
  end
end

// Latch DB_I into idl on CP2 rising, before RDB de-asserts.
always_comb begin
  idl = idl_d;
  if (~cp2)
    idl = DB_I;
end

always_ff @(posedge CLK) if (cp2p) begin
  idl_d <= idl;
end

assign DB_O = dor;
assign DB_OE = ~WRB;
assign A = aor;
assign RDB = ~rd_ext;
assign WRB = ~wr_ext;

assign M1 = m1ext;


//////////////////////////////////////////////////////////////////////
// Registers

initial begin
  mk = 8'hff;
end

// General-purpose registers
always @(posedge CLK) begin
  if (cp2n & ~(t2_wait | cl_skip)) begin
    if (cl_lts == ULTS_RF) begin
      case (cl_rfts)
        URFS_V: v <= idb;
        URFS_A: a <= idb;
        URFS_B: b <= idb;
        URFS_C: c <= idb;
        URFS_D: d <= idb;
        URFS_E: e <= idb;
        URFS_H: h <= idb;
        URFS_L: l <= idb;
        default: ;
      endcase
    end
    if (cl_abits != UABS_PC) begin
      case (cl_abits)
        UABS_BC: {b, c} <= nabi;
        UABS_DE: {d, e} <= nabi;
        UABS_HL: {h, l} <= nabi;
        default: ;
      endcase
    end
    if (cl_lts == ULTS_SEC) begin
      if (cl_rfts == URFS_V) begin
        v2 <= v;  v <= v2;
        a2 <= a;  a <= a2;
      end
      else if (cl_rfts == URFS_B) begin
        b2 <= b;  b <= b2;
        c2 <= c;  c <= c2;
        d2 <= d;  d <= d2;
        e2 <= e;  e <= e2;
        h2 <= h;  h <= h2;
        l2 <= l;  l <= l2;
      end
    end
  end
end

// Working register
always @(posedge CLK) begin
  if (cp2n & ~(t2_wait | cl_skip)) begin
    if ((cl_lts == ULTS_RF) & (nc.rfts == URFS_W)) begin
      w <= idb;
    end
  end
end

// psw: processor status word
always @(posedge CLK) begin
  if (resg) begin
    psw <= 0;
  end
  else if (cp2n & ~t2_wait) begin
    if (cl_sks_sk)
      `psw_sk <= skso;

    if (~cl_skip) begin
      if (cl_co_z)
        `psw_z <= ~|co;

      if (cl_cco_c)
        `psw_cy <= cco;
      if (cl_zero_c)
        `psw_cy <= 1'b0;
      if (cl_one_c)
        `psw_cy <= 1'b1;

      if (cl_cho_hc)
        `psw_hc <= cho;

      if (cl_sefs_psw)
        {`psw_l1, `psw_l0} <= sefso;

      if (cl_idb_psw)
        psw <= idb;
    end
  end
end

// sp: stack pointer
always @(posedge CLK) begin
  if (resg) begin
    sp <= 0;
  end
  else if (cp2p & ~(t2_wait_d | cl_skip)) begin
    if (cl_lts == ULTS_RF) begin
      case (cl_rfts)
        URFS_SPL: sp[7:0] <= idb;
        URFS_SPH: sp[15:8] <= idb;
        default: ;
      endcase
    end
    if (cl_abits == UABS_SP) begin
      sp <= nabi;
    end
  end
end

// pc: program ("P") counter
always @(posedge CLK) begin
  if (resg) begin
    pc <= 0;
  end
  else if (cp2p & ~t2_wait_d) begin
    if (cl_abi_pc) begin
      pc <= nabi;
    end
  end
end

assign pcl = pc[7:0];
assign pch = pc[15:8];

// ir: instruction (opcode) register
always @(posedge CLK) begin
  if (resg) begin
    ir <= 0;
  end
  if (cp2n) begin
    if (cl_idb_ir) begin
      if (intg) begin
        ir <= `IR_SOFTI;
      end
      else begin
        // HACK: Bypassing idb, because I need it free in T3 for completing
        // the prior instruction.
        ir[7:0] <= idl;
      end
    end
    if (cl_of_prefix_ir) begin
      ir[10:8] <= of_prefix;
    end
  end
end

// ie: interrupt enable flag
always @(posedge CLK) begin
  if (resg) begin
    ie <= 1'b0;
  end
  else if (cp2n) begin
    if (cl_ui_ie) begin
      ie <= nc.idx[0];
    end
  end
end

// mk: interrupt mask register
always @(posedge CLK) begin
  if (resg) begin
    mk <= 8'hff;
  end
  else if (cp2n) begin
    if ((cl_lts == ULTS_SPR) & (cl_spr == USPR_MK)) begin
      mk <= idb;
    end
  end
end

// aor: address bus output register
// register latches on cp1p
always @(posedge CLK) begin
  if (resg) begin
    aor <= '0;
  end
  else if (cp1p) begin
    if (cl_abl_aor)
      aor[7:0] <= ab[7:0];
    else if (cl_abh_aor)
      aor[15:8] <= ab[15:8];
    else if (cl_ab_aor)
      aor <= ab[15:0];
  end
end

// dor: data output register
// register latches on cp1p
always @(posedge CLK) begin
  if (cp1p) begin
    if (cl_idb_dor) begin
      dor <= idb;
    end
  end
end


//////////////////////////////////////////////////////////////////////
// Internal buses

// rfo: register file output
always @* begin
  case (cl_rfos)
    URFS_V: rfo = v;
    URFS_A: rfo = a;
    URFS_B: rfo = b;
    URFS_C: rfo = c;
    URFS_D: rfo = d;
    URFS_E: rfo = e;
    URFS_H: rfo = h;
    URFS_L: rfo = l;
    URFS_PSW: rfo = psw;
    URFS_SPL: rfo = sp[7:0];
    URFS_SPH: rfo = sp[15:8];
    URFS_PCL: rfo = pcl;
    URFS_PCH: rfo = pch;
    URFS_W: rfo = w;
    default: rfo = 8'hxx;
  endcase
end

// spro: special register output
always @* begin
  case (cl_spr)
    USPR_PA: spro = pai;
    USPR_PB: spro = pbi;
    USPR_PC: spro = pci;
    USPR_MK: spro = mk;
    USPR_MB: spro = mb;
    USPR_MC: spro = mc;
    USPR_TM0: spro = tm[7:0];
    USPR_TM1: spro = {4'b0, tm[11:8]};
    default: spro = 8'hxx;
  endcase
end

// idb: internal data bus
always @* begin
  case (cl_idbs)
    UIDBS_0: idb = 0;
    UIDBS_RF: idb = rfo;
    UIDBS_DB: idb = idl;
    UIDBS_CO: idb = co;
    UIDBS_SPR: idb = spro;
    UIDBS_SDG: idb = sdg;
    default: idb = 8'hxx;
  endcase
end

// ab: (internal) address bus
always @* begin
  case (cl_abs)
    UABS_PC: ab = pc;
    UABS_SP: ab = sp;
    UABS_BC: ab = {b, c};
    UABS_DE: ab = {d, e};
    UABS_HL: ab = {h, l};
    UABS_VW: ab = {v, w};
    UABS_IDB_W: ab = {idb, w};
    UABS_AOR: ab = aor;
    UABS_NABI: ab = nabi;
    default: ab = 16'hxxxx;
  endcase
end


//////////////////////////////////////////////////////////////////////
// ALU

// Inputs
always @(posedge CLK) begin
  if (cl_lts == ULTS_AI) begin
    ai <= idb;
  end
  if (cl_lts == ULTS_BI) begin
    bi <= idb;
  end
  if (cl_zero_bi) begin
    bi <= 0;
  end
end

always @* addc = (cl_carry & cco) | (cl_one_addc | cl_incs | cl_decs) | (cl_c_addc & `psw_cy);
always @* notbi = cl_bi_not | cl_decs;
always @* pdah = `psw_cy | (ai[7:4] > 4'h9) |
                 (~`psw_hc & (ai[3:0] > 4'h9) & (ai[7:4] == 4'h9));
always @* pdal = `psw_hc | (ai[3:0] > 4'h9);

always @* begin
  ibi = bi;
  if (cl_incs | cl_decs)
    ibi = 8'h00;
  // DAA adjust constants
  if (cl_bi_daa) begin
    ibi[7:4] = pdah ? 4'h6 : 4'h0;
    ibi[3:0] = pdal ? 4'h6 : 4'h0;
  end
  ibi = ibi ^ {8{notbi}};
end

// Maths
always @* begin :sums_subs
reg [4:0] hsum, lsum;
reg       sub;
  sub = cl_subs | cl_decs;
  lsum = ai[3:0] + ibi[3:0] + {3'b0, addc ^ sub};
  hsum = ai[7:4] + ibi[7:4] + {3'b0, lsum[4]};
  alu_co = {hsum[3:0], lsum[3:0]};
  alu_cho = lsum[4] ^ sub;
  alu_cco = hsum[4] ^ sub;
end

always @(posedge CLK) if (cp2n & ~t2_wait) begin
  if (cl_sums | cl_subs | cl_incs | cl_decs) begin
    co <= alu_co;
    cho <= (cl_bi_daa) ? pdal : alu_cho;
    cco <= (cl_bi_daa) ? pdah : alu_cco;
  end
  else if (cl_ors)
    co <= ai | ibi;
  else if (cl_ands)
    co <= ai & ibi;
  else if (cl_eors)
    co <= ai ^ ibi;
  else if (cl_lsls | cl_rols)
    {cco, co} <= {ai[7:0], addc & cl_rols};
  else if (cl_lsrs | cl_rors)
    {co, cco} <= {addc & cl_rors, ai[7:0]};
  else if (cl_dils)
    co[3:0] <= ai[3:0];
  else if (cl_dihs)
    co[7:4] <= ai[7:4];
  else if (cl_diss)
    co <= {ai[3:0], ai[7:4]};
end


//////////////////////////////////////////////////////////////////////
// Address bus incrementer / decrementer

// update: change part or all
always @(posedge CLK) begin
  if (cp1p & ~t2_wait_d) begin
    uabi <= ab;

    if (~cl_skip) begin
      if (cl_idb_abil)
        uabi[7:0] <= idb;
      if (cl_idb_abih)
        uabi[15:8] <= idb;
    end
  end
end

// next: increment/decrement
always @* begin
  nabi = uabi;

  if (cl_abi_inc)
    nabi = nabi + 1;
  else if (cl_abi_dec)
    nabi = nabi - 1;
end


//////////////////////////////////////////////////////////////////////
// Miscellaneous data sources

// Skip flag source
always @* begin
  case (cl_pswsk)
    USKS_0: skso = 1'b0;
    USKS_1: skso = 1'b1;
    USKS_C: skso = cco;
    USKS_NC: skso = ~cco;
    USKS_Z: skso = ~|co;
    USKS_NZ: skso = |co;
    USKS_I: skso = irf1;
    USKS_NI: skso = ~irf1;
    USKS_PSW_C: skso = `psw_cy;
    USKS_PSW_NC: skso = ~`psw_cy;
    USKS_PSW_Z: skso = `psw_z;
    USKS_PSW_NZ: skso = ~`psw_z;
    default: skso = 1'bx;
  endcase
end

// String effect flag (L0/L1) flag source
always @* begin
  case (cl_sefm)
    ISEFM_NONE: sefso = 0;
    ISEFM_L0: sefso = 2'b01;
    ISEFM_L1: sefso = 2'b10;
    default: sefso = 'X;
  endcase
end

// Special data generator
always @* begin
  case (nc.sdgs)
    USDGS_JRL: sdg = {{3{ir[5]}}, ir[4:0]};
    USDGS_JRH: sdg = {8{ir[5]}};
    USDGS_CALF: sdg = {5'b00001, ir[2:0]};
    USDGS_CALT: sdg = {1'b1, ir[5:0], nc.idx[0]};
    USDGS_INTVA: sdg = intva;
    USDGS_BIT: sdg = 8'b1 << ir[2:0];
    default: sdg = 8'hxx;
  endcase
end


//////////////////////////////////////////////////////////////////////
// Opcode fetch
//
// Instruction execution and opcode fetch can sometimes overlap.

always_ff @(posedge CLK) begin
  if (resg) begin
    oft[3:1] <= 0;
    m1ext <= 0;
  end
  else begin
    if (cp2n & ~t2_wait) begin
      oft[3:1] <= oft[2:0];
    end
    if (cp1p) begin
      m1ext <= (m1 & |oft[2:0]) & ~intg;
    end
  end
end

// M1 cycle should start as soon as resg clears.
always_ff @(posedge CLK) begin
  if (cp2n) begin
    oft[0] <= oft0_next;
    m1 <= m1_next;
  end
end

assign oft0_next = (m1_next & ~|oft[2:0]) | (~resg & m1 & |of_prefix);
assign m1_overlap = of_done & ird.m1_overlap;
assign m1_next = (resg & ~resp) | (~resg & ((m1 & ~oft[3]) | (uc.m1 | m1_overlap)));

// Handle fetching a prefix opcode (1st of 2-byte opcode)
always @* begin
  of_prefix = 0;
  if (~|ir[10:8]) begin
    case (ir[7:0])
      8'h48: of_prefix = 3'd1;
      8'h4C: of_prefix = 3'd2;
      8'h4D: of_prefix = 3'd3;
      8'h60: of_prefix = 3'd4;
      8'h64: of_prefix = 3'd5;
      8'h70: of_prefix = 3'd6;
      8'h74: of_prefix = 3'd7;
      default: ;
    endcase
  end
end

assign of_start = m1_next & oft0_next;
assign of_start_d = m1 & oft[0];
assign of_done = oft[3] & ~(m1 & |of_prefix);
assign of_pc_inc = oft[3] & ~intg;
assign of_no_skip = ird.no_skip;


//////////////////////////////////////////////////////////////////////
// Instruction decode

s_ird ird_lut [2048];

initial begin
int i;
  // Illegal opcode default: fetch new opcode
  for (i = 0; i < 2048; i++) begin
    // default for illegal opcodes
    ird_lut[i] = { UA_IDLE, 1'd1, ISEFM_NONE, 1'd0 };
  end

`include "uc-ird.svh"
end

always @* ird = ird_lut[ir];


//////////////////////////////////////////////////////////////////////
// Microcode

`include "urom.svh"
`include "nrom.svh"

always_ff @(posedge CLK) begin
  ucp <= urom[uptr_p];
end

assign nptr = ucp.naddr;

always_ff @(posedge CLK) begin
  ncp <= nrom[nptr];
end

always_ff @(posedge CLK) begin
  if (cp2n & (resg | ~t2_wait)) begin
    uptr <= uptr_p;
    uc <= ucp;
    nc <= ncp;
  end
end

assign uc_done = (uptr != UA_IDLE) & (uc.bm == UBM_END);

always @* begin
  uptr_next = uptr_p;

  if (of_done) begin
    uptr_next = ird.uaddr;
  end
  else begin
    case (uc.bm)
      UBM_ADV: uptr_next = e_uaddr'(uptr_next + 1'd1);
      UBM_END: uptr_next = UA_IDLE;
      default: ;
    endcase
  end
end

always @(posedge CLK) begin
  if (resg) begin
    uptr_p <= UA_IDLE;
  end
  else if (cp1p & ~t2_wait_d) begin
    uptr_p <= uptr_next;

`ifndef VIVADO
    if (of_done) begin
      assert (ir == 0 || uptr_next != UA_IDLE);
      else begin
        $error("%t: Illegal opcode", $time);
        $fatal(1);
      end
    end
`endif
  end
end


//////////////////////////////////////////////////////////////////////
// Control logic

function e_lts resolve_lts(e_lts in);
e_lts out;
reg sefa;
  begin
    out = in;
    // String effect instruction, and corresponding flag is set?
    sefa = ((cl_sefm == ISEFM_L0) & `psw_l0) |
           ((cl_sefm == ISEFM_L1) & `psw_l1);
    if (sefa) begin
      // Disable register writeback 
      if (in == ULTS_RF) begin
        out = ULTS_NONE;
      end
    end
    resolve_lts = out;
  end
endfunction

function e_urfs resolve_rfs_ir(e_urfs in);
  e_urfs out;
  begin
    out = in;
    if (in == URFS_IR210) begin
      // IR[2:0] encodes V,A,B...L
      out = e_urfs'({2'b00, ir[2:0]});
    end
    resolve_rfs_ir = out;
  end
endfunction

function e_abs resolve_abs_ir(e_abs in);
  e_abs out;
  begin
    out = in;
    if (in == UABS_IR210) begin
      case (ir[2:0])
        3'd0: out = UABS_SP;
        3'd1: out = UABS_BC;
        3'd2, 3'd4, 3'd6: out = UABS_DE;
        3'd3, 3'd5, 3'd7: out = UABS_HL;
        default: ;
      endcase
    end
    // Skipping 'JB' needs a little help.
    if (cl_skip)
      out = UABS_PC;
    resolve_abs_ir = out;
  end
endfunction

function e_spr resolve_sprs(e_sprs in);
  reg [3:0] out;
  begin
    out = 'X;
    if (in == USRS_IR2) begin
      out = {1'b0, ir[2:0]};
    end
    if (in == USRS_IR3) begin
      out = ir[3:0];
    end
    resolve_sprs = e_spr'(out);
  end
endfunction

always @(posedge CLK) begin
  if (resg) begin
    cl_skip <= 0;
  end
  else if (cp2n & of_done) begin
    cl_skip <= `psw_sk & ~(intg | of_no_skip);
  end
end

// cl_load_db (in T2) will always follow cl_pre_load_db (in T1).
always @(posedge CLK) begin
  if (resg) begin
    cl_load_db <= 0;
  end
  else if (cp2n & ~t2_wait) begin
    cl_load_db <= cl_pre_load_db;
  end
end

always @* cl_idb_psw = (cl_rfts == URFS_PSW);
always @* cl_co_z = nc.pswz;
always @* cl_cco_c = nc.pswcy;
always @* cl_zero_c = (nc.lts == ULTS_PSW_CY) & (nc.idx[0] == 1'b0);
always @* cl_one_c = (nc.lts == ULTS_PSW_CY) & (nc.idx[0] == 1'b1);
always @* cl_cho_hc = nc.pswhc;
always @* cl_sks_sk = uc_done;
always @* cl_pswsk = e_sks'(cl_skip ? USKS_0 : nc.pswsk);
always @* cl_sefs_psw = uc_done;
always @* cl_sefm = ird.sefm;
always @* cl_zero_irf = (nc.pswsk == USKS_I) | (nc.pswsk == USKS_NI);
always @* cl_irf = ir[2:0];
always @* cl_lts = resolve_lts(nc.lts);
initial cl_abl_aor = 0;
initial cl_abh_aor = 0;
always @* cl_ab_aor = oft[0] | nc.aout;
always @* cl_idb_dor = (nc.lts == ULTS_DOR);
always @* cl_store_dor = nc.store & ~cl_skip;
always @* cl_pre_load_db = oft[0] | (nc.pre_load & ~cl_skip);
always @* cl_rfos = resolve_rfs_ir(nc.rfos);
always @* cl_rfts = resolve_rfs_ir(nc.rfts);
always @* cl_spr = resolve_sprs(nc.sprs);
always @* cl_idbs = nc.idbs;
always @* cl_idb_pcl = (nc.lts == ULTS_RF) & (nc.rfts == URFS_PCL);
always @* cl_idb_pch = (nc.lts == ULTS_RF) & (nc.rfts == URFS_PCH);
always @* cl_pc_inc = of_pc_inc | nc.pc_inc;
always @* cl_pc_dec = (nc.abs == UABS_PC) & cl_abi_dec;
always @* cl_abi_pc = cl_idb_pcl | cl_idb_pch | cl_pc_inc | cl_pc_dec | (nc.ab_inc & nc.ab_dec & ~cl_skip);
always @* cl_idb_ir = oft[2];
always @* cl_of_prefix_ir = oft[2];
always @* cl_ui_ie = (nc.lts == ULTS_IE) | (intg & of_done);
always @* cl_abs = resolve_abs_ir(nc.abs);
always @* cl_abits = resolve_abs_ir(nc.abits);
always @* cl_idb_abil = cl_idb_pcl;
always @* cl_idb_abih = cl_idb_pch;
always @* cl_abi_inc = cl_pc_inc | (nc.ab_inc & ~nc.ab_dec) | cl_rpir_inc;
always @* cl_abi_dec = ((nc.ab_dec & ~nc.ab_inc) | cl_rpir_dec | (nc.ab_dec_if_nb & ~cco)) & ~cl_skip;
always @* cl_rpir_inc = nc.rpir & (ir[2:1] == 2'b10); // 3'd4-3'd5
always @* cl_rpir_dec = nc.rpir & (ir[2:1] == 2'b11); // 3'd6-3'd7
always @* cl_carry = nc.cis == UCIS_CCO;
always @* cl_one_addc = nc.cis == UCIS_1;
always @* cl_c_addc = nc.cis == UCIS_PSW_CY;
always @* cl_zero_bi = nc.bi0;
always @* cl_bi_not = nc.bin;
always @* cl_bi_daa = nc.daa;
initial cl_clrs = 0;
always @* cl_sums = nc.aluop == UAO_SUM;
always @* cl_subs = nc.aluop == UAO_SUB;
always @* cl_incs = nc.aluop == UAO_INC;
always @* cl_decs = nc.aluop == UAO_DEC;
always @* cl_ors =  nc.aluop == UAO_OR;
always @* cl_ands = nc.aluop == UAO_AND;
always @* cl_eors = nc.aluop == UAO_EOR;
always @* cl_lsls = nc.aluop == UAO_LSL;
always @* cl_rols = nc.aluop == UAO_ROL;
always @* cl_lsrs = nc.aluop == UAO_LSR;
always @* cl_rors = nc.aluop == UAO_ROR;
always @* cl_dils = nc.aluop == UAO_DIL;
always @* cl_dihs = nc.aluop == UAO_DIH;
always @* cl_diss = nc.aluop == UAO_DIS;
always @* cl_stm = (uptr == UA_STM) & ~cl_skip;


//////////////////////////////////////////////////////////////////////
// I/O Ports

initial begin
  pao = 0;
  pbo = 0;
  pco = 0;
  mb = 8'hff;
  mc = 8'hff;
end

// Port and Mode registers
always @(posedge CLK) begin
  if (resg) begin
    pao <= 0;
    pbo <= 0;
    pco <= 0;
    mb <= 8'hff;
    mc <= 8'hff;
  end
  else if (cp2n & ~(t2_wait | cl_skip)) begin
    if (cl_lts == ULTS_SPR) begin
      case (cl_spr)
        USPR_PA: pao <= idb;
        USPR_PB: pbo <= idb;
        USPR_PC: pco <= idb;
        USPR_MB: mb <= idb;
        USPR_MC: mc <= idb;
        default: ;
      endcase
    end
  end
end

assign pai = PA_O;
assign pbi = (PB_I & ~pboe) | (PB_O & pboe);
assign pci = (PC_I & ~pcoe) | (PC_O & pcoe);

assign pboe = ~mb;
assign pcoe = {~mc[7], 5'b11110, ~mc[1:0]};

assign PA_O = pao;
assign PB_O = pbo;
assign PC_O = pco & {mc[7:2], ~mc[1:0]}; // mask control function outputs

assign PB_OE = pboe;
assign PC_OE = pcoe;


//////////////////////////////////////////////////////////////////////
// Timer
//
// . 12-bit down counter, clocked by CP1/2 /8 prescaler
// . When counter underflows or STM instruction executes:
//   . Set interrupt II_INTT (underflow only)
//   . Reload counter from special registers TM0 and TM1
//   . Reset prescaler
// . Not implemented: timer flip-flop, output T0

initial begin
  tm = 12'hfff;
end

// Timer registers
always_ff @(posedge CLK) begin
  if (resg) begin
  end
  else if (cp2n & ~(t2_wait | cl_skip)) begin
    if (cl_lts == ULTS_SPR) begin
      case (cl_spr)
        USPR_TM0: tm[7:0] <= idb;
        USPR_TM1: tm[11:8] <= idb[3:0];
        default: ;
      endcase
    end
  end
end

assign tc_uf = cp2n & ~|tc;

always_ff @(posedge CLK) begin
  if (resg) begin
    tc <= 15'h7fff;
  end
  else if (cp2n) begin
    if (tc_uf | cl_stm) begin
      // Reset prescaler and reload counter
      tc <= {tm, 3'b111};
    end
    else begin
      tc <= tc - 1'd1;
    end
  end
end


endmodule
