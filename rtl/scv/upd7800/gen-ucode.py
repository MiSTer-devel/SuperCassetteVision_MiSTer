# Microcode generator
#
# Copyright (c) 2024 David Hunter
#
# This program is GPL licensed. See COPYING for the full license.

# TODO:
# . SIO, PEN, PEX, PER, IN, OUT (not implemented in MAME)

import yaml


ird_rows = []
uc_rows = []
nc_rows = []


def uc_row(row):
    uc_rows.append(row)

def nc_row(nc):
    if nc in nc_rows:
        i = nc_rows.index(nc)
    else:
        i = len(nc_rows)
        nc_rows.append(nc)
    return i


class ucode_seq():
    def __init__(self, name, str_effect=''):
        name = name.translate(str.maketrans('+-', 'PN'))
        self.name = name
        self.str_effect = 'NONE'
        if str_effect:
            self.str_effect = str_effect
            self.name += '_' + str_effect
        self.steps = []

    def step(self, nc):
        self.steps.append(nc)

    def commit(self, nsteps):
        steps_len = len(self.steps)
        for i in range(steps_len):
            ucrow = {}
            nc = self.steps[i]
            if i == 0:
                ucrow['uaddr'] = self.name
            nsteps = nsteps - 1
            if nsteps == 0:
                ucrow['m1'] = 1
            if i == steps_len - 1:
                ucrow['bm'] = 'END'

            naddr = nc_row(nc)
            ucrow['naddr'] = naddr

            if 'pswsk' in nc:
                assert(ucrow['bm'] == 'END')

            uc_row(ucrow)


def ird_row(ir, nsteps, noper, ucs, no_skip=False):
    if isinstance(ir, list):
        ir0, ir1 = ir
        ats = f'[0x{ir0:02x}, 0x{ir1:02x}]'
    else:
        ir0 = ir
        ats = f'0x{ir:02x}'

    nsteps -= 8 if ir0 >= 0x100 else 4

    ucname = ucs.name
    ucs.commit(nsteps)

    irdrow = {'_at': ats, 'at': ir, 'uaddr': ucname}
    if nsteps == 0:
        irdrow['m1_overlap'] = 1
    if no_skip:
        irdrow['no_skip'] = 1
    irdrow['sefm'] = ucs.str_effect

    ird_rows.append(irdrow)


######################################################################
# Instruction base

def ins(ir, nsteps, ucname, noper, ncs):
    assert(isinstance(ncs, list))
    ucs = ucode_seq(ucname)
    for s in ncs:
        ucs.step(s)
    ird_row(ir, nsteps, noper, ucs)

def idb_sel(rbs, op):
    if rbs in [0, 'AI', 'BI', 'CO', 'DB', 'DOR']:
        bc = {'r': 'idbs', 'w': 'lts'}[op]
        return {bc: rbs}

    if '_' not in rbs:
        rbs = 'RF_' + rbs
    bs, rs = rbs.split('_')
    bc, rc = {
        'RF-w': ('lts', 'rfts'),
        'SPR-w': ('lts', 'sprs'),
        'RF-r': ('idbs', 'rfos'),
        'SPR-r': ('idbs', 'sprs'),
        'SDG-r': ('idbs', 'sdgs'),
    }[bs + '-' + op]
    return {bc: bs, rc: rs}

def idb_rd(rbs):
    return idb_sel(rbs, 'r')

def idb_wr(rbs):
    if rbs == 'DB':
        rbs = 'DOR'                 # write equivalent of idb_rd('DB')
    return idb_sel(rbs, 'w')

def aor_wr(abs):
    return {'abs': abs, 'aout': 1}

def aor_wr_rp():
    # Also enables inc/dec writeback
    return aor_wr('IR210') | {'abits': 'IR210', 'rpir': 1}

######################################################################
# Common nrom operations

nc_idle = {}
nc_preload = {'pre_load': 1}
nc_pc_out_inc = {'aout': 1, 'pc_inc': 1} | nc_preload
nc_load = {}    # load was replaced by preload in T-1
nc_store = {'store': 1}         # dor -> DB
nc_write_db_to_w = {'idbs': 'DB', 'lts': 'RF', 'rfts': 'W'}
# DB -> idb -> AI, 0 -> BI, AI + BI -> CO
nc_write_db_to_co = {'idbs': 'DB', 'lts': 'AI', 'bi0': 1, 'aluop': 'SUM'}
nc_dec_sp = {'abs': 'SP', 'ab_dec': 1, 'abits': 'SP'} # SP - 1 -> SP
nc_inc_sp = {'abs': 'SP', 'ab_inc': 1, 'abits': 'SP'} # SP + 1 -> SP
nc_store_co_to_vw = idb_rd('CO') | idb_wr('DOR') | aor_wr('VW')

# Pre-populate nrom rows
nc_row(nc_idle)
nc_row(nc_pc_out_inc)
#nc_row(nc_load)
nc_row(nc_store)

######################################################################
# Move / load / store data

def move(ir, nsteps, dst, src, str_effect=''):
    imm = src == 'IMM'
    noper = 1 if imm else 0

    ucs = ucode_seq(f'MOV_{dst}_{src}', str_effect)

    if imm:
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        ucs.step({'idbs': 'DB'} | idb_wr(dst))
    else:
        ucs.step(idb_rd(src) | idb_wr(dst))

    ird_row(ir, nsteps, noper, ucs)

def load_wa(ir, nsteps, dst):
    ucs = ucode_seq(f'LD_{dst}_WA')
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    ucs.step(aor_wr('VW') | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr(dst))
    ird_row(ir, nsteps, 1, ucs)

def rpa_to_reglh(rpa):
    regl, regh = {
        'SP': ('SPL', 'SPH'),
        'BC': ('C', 'B'),
        'DE': ('E', 'D'),
        'HL': ('L', 'H'),
    }[rpa]
    return regl, regh

def load_imm16(ir, nsteps, reg, str_effect=''):
    regl, regh = rpa_to_reglh(reg)

    ucs = ucode_seq(f'LDX_{reg}_IMM', str_effect)
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step({'idbs': 'DB', 'rfts': regl, 'lts': 'RF'})
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step({'idbs': 'DB', 'rfts': regh, 'lts': 'RF'})
    ird_row(ir, nsteps, 2, ucs)

def loadx(ir, nsteps):
    ucs = ucode_seq(f'LDAX')

    ucs.step(aor_wr_rp() | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr('A'))

    ird_row(ir, nsteps, 0, ucs)

def load_abs(ir, nsteps):
    ucs = ucode_seq(f'LD_IR210_ABS')
    ucs.step(nc_pc_out_inc)     # Fetch word lo
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    ucs.step(nc_pc_out_inc)     # Fetch word hi
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_co)
    # W -> abl -> aorl, CO -> idb -> abh -> aorh
    ucs.step({'idbs': 'CO'} | aor_wr('IDB_W') | nc_preload)
    ucs.step(nc_load)
    # DB -> idb -> r2
    ucs.step(idb_rd('DB') | idb_wr('RF_IR210'))
    ird_row(ir, nsteps, 2, ucs)

def load_ind(ir, nsteps, reg):
    regl, regh = rpa_to_reglh(reg)

    ucs = ucode_seq(f'L{reg}D')
    # Fetch word lo
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    # Fetch word hi
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_co)
    # rpal <- (word)
    ucs.step({'idbs': 'CO'} | aor_wr('IDB_W') | nc_preload) # {CO, W} -> aor
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr(regl) |
             {'abs': 'AOR'})    # setup for ab_inc
    # rpah <- (word+1)
    # HACK ALERT. I am not proud of this.
    ucs.step(aor_wr('NABI') | nc_preload | {'ab_inc': 1}) # {CO, W} + 1 -> aor
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr(regh))
    ird_row(ir, nsteps, 2, ucs)

def storex(ir, nsteps, src):
    imm = src == 'IMM'
    if imm:
        src = 'RF_W'
    noper = 1 if imm else 0
    nc_write_src_to_dor = idb_rd(src) | {'lts': 'DOR'}
    nc_write_dst_to_aor = aor_wr_rp()

    ucs = ucode_seq(f'STX_{src}')

    if imm:
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        ucs.step({'idbs': 'DB'} | idb_wr(src))
    ucs.step(nc_write_src_to_dor | nc_write_dst_to_aor)
    ucs.step(nc_store)
    ucs.step(nc_idle)

    ird_row(ir, nsteps, noper, ucs)

def storew(ir, nsteps, src):
    imm = src == 'IMM'
    noper = 2 if imm else 1

    ucs = ucode_seq(f'STW_{src}')

    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    if imm:
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        # DB -> idb -> AI, 0 -> BI, AI + BI -> CO
        ucs.step({'idbs': 'DB', 'lts': 'AI', 'bi0': 1, 'aluop': 'SUM'})
        # CO -> idb -> dor
        nc_write_val_to_dor = {'idbs': 'CO', 'lts': 'DOR'}
    else:
        # A -> idb -> dor
        nc_write_val_to_dor = {'rfos': 'A', 'idbs': 'RF', 'lts': 'DOR'}
    # VW -> ab -> aor
    ucs.step(nc_write_val_to_dor | aor_wr('VW'))
    ucs.step(nc_store)
    ucs.step(nc_idle)

    ird_row(ir, nsteps, noper, ucs)

def store_abs(ir, nsteps):
    ucs = ucode_seq(f'ST_IR210_ABS')
    # r2 -> idb -> dor
    ucs.step(nc_pc_out_inc)     # Fetch word lo
    ucs.step(nc_load | idb_rd('RF_IR210') | idb_wr('DOR'))
    ucs.step(nc_write_db_to_w)
    ucs.step(nc_pc_out_inc)     # Fetch word hi
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_co)
    # W -> abl -> aorl, CO -> idb -> abh -> aorh
    ucs.step({'idbs': 'CO'} | aor_wr('IDB_W'))
    ucs.step(nc_store)
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 2, ucs)

def store_ind(ir, nsteps, reg):
    regl, regh = rpa_to_reglh(reg)

    ucs = ucode_seq(f'S{reg}D')
    # Fetch word lo
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    # Fetch word hi
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load | idb_rd(regl) | idb_wr('DOR'))
    ucs.step(nc_write_db_to_co)
    # (word) <- rpal
    ucs.step({'idbs': 'CO'} | aor_wr('IDB_W'))    # {CO, W} -> aor
    ucs.step(nc_store)
    ucs.step({'abs': 'AOR'})    # setup for ab_inc
    # (word+1) <- rpah
    # HACK ALERT. I am not proud of this.
    ucs.step(idb_rd(regh) | idb_wr('DOR') |
             aor_wr('NABI') | {'ab_inc': 1})      # {CO, W} + 1 -> aor
    ucs.step(nc_store)
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 2, ucs)

# TABLE: C <- (PC+2+A), B <- (PC+2+A+1)
def table(ir, nsteps):
    ucs = ucode_seq('TABLE')
    # {CO, W} <- PC + A + 1
    ucs.step(idb_rd('PCL') | idb_wr('AI'))
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(idb_rd('A') | idb_wr('BI') | {'aluop': 'SUM', 'cis': 1})
    ucs.step(idb_rd('CO') | idb_wr('W'))
    ucs.step(nc_idle)
    ucs.step(idb_rd('PCH') | idb_wr('AI') |
             {'aluop': 'SUM', 'bi0': 1, 'cis': 'CCO'})
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    # {CO, W} -> aor
    ucs.step({'idbs': 'CO'} | aor_wr('IDB_W') | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr('C') |
             {'abs': 'AOR'})    # setup for ab_inc
    # {CO, W} + 1 -> aor
    # HACK ALERT. I am not proud of this.
    ucs.step(aor_wr('NABI') | nc_preload | {'ab_inc': 1})
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr('B'))
    ird_row(ir, nsteps, 0, ucs)

# BLOCK: (DE)+ <- (HL)+, C <- C - 1, repeat until borrow
def block(ir, nsteps):
    ucs = ucode_seq('BLOCK')
    # C -> idb -> AI, AI - 1 -> CO
    ucs.step({'rfos': 'C', 'idbs': 'RF', 'lts': 'AI', 'aluop': 'DEC'})
    # CO -> idb -> C
    ucs.step({'rfts': 'C', 'idbs': 'CO', 'lts': 'RF'})
    # Decrement PC if no borrow
    ucs.step({'abs': 'PC', 'ab_dec_if_nb': 1})
    # HL -> ab -> aor
    ucs.step(aor_wr('HL') | {'pre_load': 1})
    # HL + 1 -> HL
    ucs.step({'abs': 'HL', 'ab_inc': 1, 'abits': 'HL'})
    # DB -> idb -> W
    ucs.step({'idbs': 'DB', 'rfts': 'W', 'lts': 'RF'})
    # W -> idb -> dor, DE -> ab -> aor
    ucs.step({'rfos': 'W', 'idbs': 'RF', 'lts': 'DOR'} | aor_wr('DE'))
    # dor -> DB, DE + 1 -> DE
    ucs.step(nc_store | {'abs': 'DE', 'ab_inc': 1, 'abits': 'DE'})
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs, no_skip=True)

# EX: Exchange V, A and V', A'
def ex(ir, nsteps):
    ucs = ucode_seq('EX')
    ucs.step({'lts': 'SEC', 'rfts': 'V'})
    ird_row(ir, nsteps, 0, ucs)

# EXX: Exchange register sets (B,C,D,E,H,L)
def exx(ir, nsteps):
    ucs = ucode_seq('EXX')
    ucs.step({'lts': 'SEC', 'rfts': 'B'})
    ird_row(ir, nsteps, 0, ucs)

######################################################################
# Math / logic / test

def math_logic_test(ir, nsteps, op, dst, src, skip=''):
    imm = src == 'IMM'
    ind = src == 'IND'
    swa = src == 'WA'
    dwa = dst == 'WA'
    spr = dst == 'SPR_IR2'
    noper = imm + (swa or dwa)

    ucname = f'{op}{skip}_{dst}_{src}'
    src = 'DB' if (imm or ind or swa) else src
    dst = 'DB' if dwa else dst
    ncop = {
        'ADD': {'aluop': 'SUM'},
        'ADC': {'aluop': 'SUM', 'cis': 'PSW_CY'},
        'SUB': {'aluop': 'SUB', 'bin': 1},
        'SBB': {'aluop': 'SUB', 'bin': 1, 'cis': 'PSW_CY'},
        'AND': {'aluop': 'AND'},
        'OR': {'aluop': 'OR'},
        'XOR': {'aluop': 'EOR'},
        'SLL': {'aluop': 'LSL'},
        'SLR': {'aluop': 'LSR'},
        'RLL': {'aluop': 'ROL', 'cis': 'PSW_CY'},
        'RLR': {'aluop': 'ROR', 'cis': 'PSW_CY'},
        'BIT': {'aluop': 'AND'},
        'CMP': {'aluop': 'SUB', 'bin': 1, 'cis': 0},
        'CMPB': {'aluop': 'SUB', 'bin': 1, 'cis': 1},
    }[op]
    ncsk = {                                      # Update PSW.SK
        '': {},
        'NC': {'pswsk': 'NC'},  # !CCO -> PSW.SK
        'NB': {'pswsk': 'NC'},  # !CCO -> PSW.SK
        'B': {'pswsk': 'C'},    # CCO -> PSW.SK
        'Z': {'pswsk': 'Z'},    # !CO -> PSW.SK
        'NZ': {'pswsk': 'NZ'},  # CO -> PSW.SK
    }[skip]
    test = op in ['BIT', 'CMP', 'CMPB']
    if op in ['ADD', 'ADC', 'SUB', 'SBB', 'CMP', 'CMPB']:
        ncpsw = {
            'pswz': 1,                            # Update PSW.Z
            'pswcy': 1,                           # Update PSW.CY
            'pswhc': 1,                           # Update PSW.HC
        }
    elif op in ['AND', 'OR', 'XOR', 'BIT']:
        ncpsw = {'pswz': 1}                       # Update PSW.Z
    elif op in ['SLR', 'SLL', 'RLL', 'RLR']:
        ncpsw = {'pswcy': 1}                      # Update PSW.CY
    else:
        raise RuntimeError('unknown op')
    nc_write_dst_to_ai = idb_rd(dst) | idb_wr('AI')
    nc_write_src_to_bi = idb_rd(src) | idb_wr('BI') if src else {}

    ucs = ucode_seq(ucname)
    if imm and dwa:
        # First operand is wa
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        ucs.step(nc_write_db_to_w)
        # VW -> ab -> aor
        ucs.step(aor_wr('VW') | nc_preload)
        ucs.step(nc_load)
        ucs.step(nc_write_dst_to_ai)
        # Second operand is imm
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
    elif imm and spr:
        # Read special reg. (takes a full M-cycle)
        ucs.step(nc_idle)
        ucs.step(nc_idle)
        ucs.step(nc_write_dst_to_ai)
        # Fetch immediate data
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
    elif imm:
        ucs.step(nc_pc_out_inc | nc_write_dst_to_ai | nc_preload)
        ucs.step(nc_load)
    elif ind:
        ucs.step(aor_wr_rp() | nc_preload)
        ucs.step(nc_load | nc_write_dst_to_ai)
    elif swa:
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        ucs.step(nc_write_db_to_w)
        # VW -> ab -> aor
        ucs.step(aor_wr('VW') | nc_write_dst_to_ai | nc_preload)
        ucs.step(nc_load)
    else:
        ucs.step(nc_write_dst_to_ai)
    ucs.step(nc_write_src_to_bi | ncop)

    if test:
        ucs.step(ncpsw | ncsk)
    else:
        if dwa:
            ucs.step(nc_store_co_to_vw)
            ucs.step(nc_store)
            ucs.step(ncpsw | ncsk)
        elif spr:
            # Write special reg. (takes a full M-cycle)
            ucs.step(nc_idle)
            ucs.step(nc_idle)
            ucs.step({'idbs': 'CO'} | idb_wr(dst) | ncpsw | ncsk)
        else:
            ucs.step({'idbs': 'CO'} | idb_wr(dst) | ncpsw | ncsk)
    ird_row(ir, nsteps, noper, ucs)

math = math_logic_test
logic = math_logic_test

def test(ir, nsteps, op, dst, src):
    mtl_op, skip = {
        'EQ':  ('CMP', 'Z'),
        'NEQ': ('CMP', 'NZ'),
        'GT':  ('CMPB','NB'),
        'LT':  ('CMP', 'B'),
        'ON':  ('BIT', 'NZ'),
        'OFF': ('BIT', 'Z'),
    }[op]
    math_logic_test(ir, nsteps, mtl_op, dst, src, skip)

def math_imm(ir, nsteps, op, reg, skip=''):
    math(ir, nsteps, op, reg, 'IMM', skip)

def mathx(ir, nsteps, op, skip=''):
    math(ir, nsteps, op, 'A', 'IND', skip)

def logic_imm(ir, nsteps, op, reg):
    logic(ir, nsteps, op, reg, 'IMM')

def logicx(ir, nsteps, op):
    logic(ir, nsteps, op, 'A', 'IND')

def test_imm(ir, nsteps, op, reg):
    test(ir, nsteps, op, reg, 'IMM')

def testx(ir, nsteps, op):
    test(ir, nsteps, op, 'A', 'IND')

def incdec(ir, nsteps, op, reg):
    wa = reg == 'WA'
    noper = 0 + wa
    ncpsw = {
        'pswsk': 'C',           # Update PSW.SK
        'pswz': 1,              # Update PSW.Z
        'pswhc': 1,             # Update PSW.HC
    }

    ucs = ucode_seq(f'{op}R_{reg}')

    if wa:
        ucs.step(nc_pc_out_inc)
        ucs.step(nc_load)
        ucs.step(nc_write_db_to_w)
        # VW -> ab -> aor
        ucs.step(aor_wr('VW') | nc_preload)
        ucs.step(nc_load)
        reg = 'DB'
    # r2 -> idb -> AI, AI +/- 1 -> CO
    ucs.step(idb_rd(reg) | idb_wr('AI') | {'aluop': op})
    if wa:
        ucs.step(nc_store_co_to_vw)
        ucs.step(nc_store)
        ucs.step(ncpsw)
    else:
        # CO -> idb -> r2
        ucs.step(idb_rd('CO') | idb_wr(reg) | ncpsw)

    ird_row(ir, nsteps, noper, ucs)

def incdecx(ir, nsteps, op, rp):
    abid = 'ab_inc' if op == 'INC' else 'ab_dec'

    ucs = ucode_seq(f'{op}_{rp}')
    ucs.step({'abs': rp, abid: 1, 'abits': rp})
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs)

# DAA: Decimal Adjust A
def daa(ir, nsteps):
    ucs = ucode_seq('DAA')
    ucs.step(idb_rd('A') | idb_wr('AI') | {'daa': 1, 'aluop': 'SUM'})
    ucs.step(idb_rd('CO') | idb_wr('A') | {'pswz': 1, 'pswhc': 1, 'pswcy': 1})
    ird_row(ir, nsteps, 0, ucs)

# RLD: A[3:0] <- (HL)[7:4] <- (HL)[3:0] <- A[3:0]
def rld(ir, nsteps, ucname):
    ucs = ucode_seq(ucname)
    ucs.step(aor_wr('HL') | nc_preload)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    ucs.step(idb_rd('W') | idb_wr('AI') | {'aluop': 'DIS'})
    ucs.step(idb_rd('CO') | idb_wr('W'))
    ucs.step(idb_rd('A') | idb_wr('AI') | {'aluop': 'DIL'})
    ucs.step(aor_wr('HL') | idb_rd('W') | idb_rd('CO') | idb_wr('DOR'))
    ucs.step(nc_store | idb_rd('W') | idb_wr('AI') | {'aluop': 'DIL'})
    ucs.step(idb_rd('A') | idb_wr('AI') | {'aluop': 'DIH'})
    ucs.step(idb_rd('CO') | idb_wr('A'))
    ird_row(ir, nsteps, 0, ucs)

# RRD: A[3:0] -> (HL)[7:4] -> (HL)[3:0] -> A[3:0]
def rrd(ir, nsteps, ucname):
    ucs = ucode_seq(ucname)
    ucs.step(aor_wr('HL') | nc_preload)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    ucs.step(idb_rd('W') | idb_wr('AI') | {'aluop': 'DIH'})
    ucs.step(idb_rd('A') | idb_wr('AI') | {'aluop': 'DIL'})
    ucs.step(idb_rd('CO') | idb_wr('AI') | {'aluop': 'DIS'})
    ucs.step(aor_wr('HL') | idb_rd('W') | idb_rd('CO') | idb_wr('DOR'))
    ucs.step(nc_store | idb_rd('W') | idb_wr('AI') | {'aluop': 'DIL'})
    ucs.step(idb_rd('A') | idb_wr('AI') | {'aluop': 'DIH'})
    ucs.step(idb_rd('CO') | idb_wr('A'))
    ird_row(ir, nsteps, 0, ucs)

######################################################################
# Jump / call / return

def jr(ir, nsteps):
    ucs = ucode_seq('JR')
    # PCL -> idb -> AI
    ucs.step({'rfos': 'PCL', 'idbs': 'RF', 'lts': 'AI'})
    # IR[5:0] -> idb -> BI
    ucs.step({'idbs': 'SDG', 'sdgs': 'JRL', 'lts': 'BI'})
    # AI + BI -> CO
    ucs.step({'aluop': 'SUM', 'cis': 0})
    # CO -> idb -> PCL
    ucs.step({'idbs': 'CO', 'lts': 'RF', 'rfts': 'PCL'})
    # PCH -> idb -> AI
    ucs.step({'rfos': 'PCH', 'idbs': 'RF', 'lts': 'AI'})
    # sign-ext. IR[5] -> idb -> BI
    ucs.step({'idbs': 'SDG', 'sdgs': 'JRH', 'lts': 'BI'})
    # AI + BI + CCO -> CO
    ucs.step({'aluop': 'SUM', 'cis': 'CCO'})
    # CO -> idb -> PCH
    ucs.step({'idbs': 'CO', 'lts': 'RF', 'rfts': 'PCH'})
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs)

def jre(ir, nsteps, sign):
    ucs = ucode_seq(f'JRE_{sign}')
    ucs.step(nc_pc_out_inc)
    # PCL -> idb -> AI
    ucs.step({'rfos': 'PCL', 'idbs': 'RF', 'lts': 'AI'})
    # DB -> idb -> BI, AI + BI -> CO
    ucs.step({'idbs': 'DB', 'lts': 'BI', 'aluop': 'SUM', 'cis': 0})
    # CO -> idb -> PCL
    ucs.step({'idbs': 'CO', 'lts': 'RF', 'rfts': 'PCL'})
    # PCH -> idb -> AI, 0 -> BI, AI +/- BI +/- CCO -> CO
    ucs.step({'rfos': 'PCH', 'idbs': 'RF', 'lts': 'AI', 'bi0': 1,
              'bin': sign == '-', 'aluop': 'SUM', 'cis': 'CCO'})
    # CO -> idb -> PCH
    ucs.step({'idbs': 'CO', 'lts': 'RF', 'rfts': 'PCH'})
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 1, ucs)

def jmp(ir, nsteps):
    ucs = ucode_seq('JMP')
    ucs.step(nc_pc_out_inc)                       # Fetch word lo
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    ucs.step(nc_pc_out_inc)                       # Fetch word hi
    # W -> idb -> PCL
    ucs.step(idb_rd('RF_W') | idb_wr('RF_PCL') | nc_load)
    # DB -> idb -> PCH
    ucs.step({'idbs': 'DB'} | idb_wr('RF_PCH'))
    ird_row(ir, nsteps, 2, ucs)

def jb(ir, nsteps):
    ucs = ucode_seq('JB')
    ucs.step(aor_wr('BC') | {'abits': 'PC', 'ab_inc': 1, 'ab_dec': 1})
    #ucs.step(idb_rd('B') | idb_wr('PCH'))
    #ucs.step(idb_rd('C') | idb_wr('PCL'))
    ird_row(ir, nsteps, 0, ucs)

def call(ir, nsteps):
    ucs = ucode_seq('CALL')
    # Fetch new PCL to W, PC <- PC+1
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    # Fetch new PCH to CO, SP <- SP-1, PC <- PC+1
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_co | nc_dec_sp)
    # (SP) <- PCH, SP <- SP-1
    ucs.step(idb_rd('RF_PCH') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- PCL, W -> abl -> PCL, CO -> idb -> abh -> PCH
    ucs.step(idb_rd('RF_PCL') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store | idb_rd('W') | idb_wr('RF_PCL'))
    ucs.step(idb_rd('CO') | idb_wr('RF_PCH'))
    ird_row(ir, nsteps, 2, ucs)

def calb(ir, nsteps):
    ucs = ucode_seq('CALB')
    # SP <- SP-1
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(nc_dec_sp)
    # (SP) <- PCH, SP <- SP-1, PCH <- B
    ucs.step(idb_rd('RF_PCH') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store | idb_rd('B') | idb_wr('PCH'))
    ucs.step(nc_dec_sp)
    # (SP) <- PCL, PCL <- C
    ucs.step(idb_rd('RF_PCL') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store | idb_rd('C') | idb_wr('PCL'))
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs)

def calf(ir, nsteps):
    ucs = ucode_seq('CALF')
    # SP <- SP-1, PC <- PC+1 (next ins.)
    ucs.step({'pc_inc': 1} | nc_preload)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w | nc_dec_sp)
    # (SP) <- PCH, SP <- SP-1
    ucs.step(idb_rd('RF_PCH') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- PCL, PC <- PC-1 (operand)
    ucs.step(idb_rd('RF_PCL') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step({'abs': 'PC', 'ab_dec': 1, 'abits': 'PC'})
    # PCL <- oper., PCH <- {5'b00001, IR[2:0]}
    ucs.step(aor_wr('PC') | nc_preload) # fetch word lo
    ucs.step(nc_load | idb_rd('SDG_CALF') | idb_wr('RF_PCH'))
    ucs.step(idb_rd('DB') | idb_wr('RF_PCL'))

    ird_row(ir, nsteps, 1, ucs)

def calt(ir, nsteps):
    ucs = ucode_seq('CALT')
    # SP <- SP-1
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(nc_dec_sp)
    # (SP) <- PCH, SP <- SP-1
    ucs.step(idb_rd('RF_PCH') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- PCL
    ucs.step(idb_rd('RF_PCL') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    # effective naddr lo. -> idb -> W
    ucs.step({'idx': 0} | idb_rd('SDG_CALT') | idb_wr('RF_W'))
    # PCL <- (128 + 2ta)
    # W -> abl -> aorl, 0 -> idb -> abh -> aorh
    ucs.step(idb_rd(0) | aor_wr('IDB_W') | nc_preload)
    # effective naddr hi. -> idb -> W
    ucs.step(nc_load | {'idx': 1} | idb_rd('SDG_CALT') | idb_wr('RF_W'))
    ucs.step(idb_rd('DB') | idb_wr('RF_PCL'))
    # PCH <- (129 + 2ta)
    ucs.step(idb_rd(0) | aor_wr('IDB_W') | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr('RF_PCH'))
    ird_row(ir, nsteps, 0, ucs)

# SOFTI/INT (Software/Hardware Interrupt) [19 states]
def softi(ir, nsteps):
    ucs = ucode_seq('INT')
    # SP <- SP-1
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(nc_dec_sp)
    # (SP) <- PSW, SP <- SP-1
    ucs.step(idb_rd('RF_PSW') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- PCH, SP <- SP-1
    ucs.step(idb_rd('RF_PCH') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- PCL
    ucs.step(idb_rd('RF_PCL') | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    # effective naddr lo. -> idb -> W
    ucs.step({'idx': 0} | idb_rd('SDG_CALT') | idb_wr('RF_W'))
    # PCH <- 0, PCL <- int. vector
    ucs.step(idb_rd(0) | idb_wr('RF_PCH'))
    ucs.step(idb_rd('SDG_INTVA') | idb_wr('RF_PCL'))
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs, no_skip=True)

# RET / RETI (Return from Subroutine / Interrupt)
def ret(ir, nsteps, ucname):
    ncsk = {'pswsk': 1} if ucname == 'RETS' else {}
    ucs = ucode_seq(ucname)
    # PCL <- (SP), SP <- SP+1
    ucs.step(aor_wr('SP') | nc_preload)
    ucs.step(nc_load | nc_inc_sp)
    ucs.step(idb_rd('DB') | idb_wr('PCL'))
    # PCH <- (SP), SP <- SP+1
    ucs.step(aor_wr('SP') | nc_preload)
    ucs.step(nc_load | nc_inc_sp)
    ucs.step(idb_rd('DB') | idb_wr('PCH') | ncsk)
    if ucname == 'RETI':
        # PSW <- (SP), SP <- SP+1
        ucs.step(aor_wr('SP') | nc_preload)
        ucs.step(nc_load | nc_inc_sp)
        ucs.step(idb_rd('DB') | idb_wr('PSW'))
    ird_row(ir, nsteps, 0, ucs)

######################################################################
# Stack

# PUSH (Push Register Pair on Stack)
def push16(ir, nsteps, rp):
    ucs = ucode_seq(f'PUSH_{rp}')
    # SP <- SP-1
    ucs.step(nc_idle)
    ucs.step(nc_idle)
    ucs.step(nc_dec_sp)
    # (SP) <- rph <- SP-1
    ucs.step(idb_rd(rp[0]) | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_dec_sp)
    # (SP) <- rpl
    ucs.step(idb_rd(rp[1]) | idb_wr('DOR') | aor_wr('SP'))
    ucs.step(nc_store)
    ucs.step(nc_idle)
    ird_row(ir, nsteps, 0, ucs)

# POP (Pop Register Pair from Stack)
def pop16(ir, nsteps, rp):
    ucs = ucode_seq(f'POP_{rp}')
    # rph <- (SP), SP <- SP+1
    ucs.step(aor_wr('SP') | nc_inc_sp | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr(rp[1]))
    # rpl <- (SP), SP <- SP+1
    ucs.step(aor_wr('SP') | nc_inc_sp | nc_preload)
    ucs.step(nc_load)
    ucs.step(idb_rd('DB') | idb_wr(rp[0]))
    ird_row(ir, nsteps, 0, ucs)


######################################################################
# Skip

def skip(ir, nsteps, sk):
    ucs = ucode_seq(f'SKIP_{sk}')
    ucs.step({'pswsk': sk})
    ird_row(ir, nsteps, 0, ucs)

def bit(ir, nsteps):
    ucs = ucode_seq(f'BIT')
    # Operand is wa
    ucs.step(nc_pc_out_inc)
    ucs.step(nc_load)
    ucs.step(nc_write_db_to_w)
    # VW -> ab -> aor
    ucs.step(aor_wr('VW') | nc_preload)
    ucs.step(nc_load | idb_rd('SDG_BIT') | idb_wr('BI'))
    ucs.step(idb_rd('DB') | idb_wr('AI') | {'aluop': 'AND'})
    ucs.step({'pswsk': 'NZ'})
    ird_row(ir, nsteps, 1, ucs)
    

######################################################################
# Pre-populate urom rows

uc_row({'uaddr': 'IDLE', 'naddr': nc_row(nc_idle), 'bm': 'END'})

######################################################################
# no prefix opcode

move([0x0a, 0x0f], 4, 'A', 'RF_IR210')            # MOV A, r1
move([0x1a, 0x1f], 4, 'RF_IR210', 'A')            # MOV r1, A
move([0x68, 0x6e], 7, 'RF_IR210', 'IMM')          # MVI r, byte
move(0x69, 7, 'RF_IR210', 'IMM', str_effect='L1') # MVI A, byte
move(0x6f, 7, 'RF_IR210', 'IMM', str_effect='L0') # MVI L, byte

load_wa(0x28, 10, 'A')                            # LDAW wa

load_imm16(0x04, 10, 'SP')                        # LXI SP, bbaa
load_imm16(0x14, 10, 'BC')                        # LXI BC, bbaa
load_imm16(0x24, 10, 'DE')                        # LXI DE, bbaa
load_imm16(0x34, 10, 'HL', str_effect='L0')       # LXI HL, bbaa

loadx([0x29, 0x2f], 7)                            # LDAX rpa

storex([0x39, 0x3f], 7, 'A')                      # STAX rpa
storex([0x49, 0x4b], 10, 'IMM')                   # MVIX rpa1, byte

storew(0x38, 10, 'A')                             # STAW wa
storew(0x71, 13, 'IMM')                           # MVIW wa, byte

table(0x21, 19)                                   # TABLE
block(0x31, 13)                                   # BLOCK

ex(0x10, 4)                                       # EX
exx(0x11, 4)                                      # EXX

logic_imm(0x05, 16, 'AND', 'WA')                  # ANIW wa, byte
logic_imm(0x15, 16, 'OR', 'WA')                   # ORIW wa, byte

logic_imm(0x07, 7, 'AND', 'A')                    # ANI A, byte
logic_imm(0x16, 7, 'XOR', 'A')                    # XRI A, byte
logic_imm(0x17, 7, 'OR', 'A')                     # ORI A, byte

test_imm(0x25, 13, 'GT', 'WA')                    # GTIW wa, byte
test_imm(0x35, 13, 'LT', 'WA')                    # LTIW wa, byte
test_imm(0x45, 13, 'ON', 'WA')                    # ONIW wa, byte
test_imm(0x55, 13, 'OFF', 'WA')                   # OFFIW wa, byte
test_imm(0x65, 13, 'NEQ', 'WA')                   # NEIW wa, byte
test_imm(0x75, 13, 'EQ', 'WA')                    # EQIW wa, byte

test_imm(0x27, 7, 'GT', 'A')                      # GTI A, byte
test_imm(0x37, 7, 'LT', 'A')                      # LTI A, byte
test_imm(0x47, 7, 'ON', 'A')                      # ONI A, byte
test_imm(0x57, 7, 'OFF', 'A')                     # OFFI A, byte
test_imm(0x67, 7, 'NEQ', 'A')                     # NEI A, byte
test_imm(0x77, 7, 'EQ', 'A')                      # EQI A, byte

math_imm(0x26, 7, 'ADD', 'A', 'NC')               # ADINC A, byte
math_imm(0x36, 7, 'SUB', 'A', 'NB')               # SUINB A, byte
math_imm(0x46, 7, 'ADD', 'A')                     # ADI A, byte
math_imm(0x56, 7, 'ADC', 'A')                     # ACI A, byte
math_imm(0x66, 7, 'SUB', 'A')                     # SUI A, byte
math_imm(0x76, 7, 'SBB', 'A')                     # SBI A, byte

incdec(0x20, 13, 'INC', 'WA')                     # INRW wa
incdec(0x30, 13, 'DEC', 'WA')                     # DCRW wa

incdec([0x41, 0x43], 4, 'INC', 'RF_IR210')        # INR r2
incdec([0x51, 0x53], 4, 'DEC', 'RF_IR210')        # DCR r2

incdecx(0x02, 7, 'INC', 'SP')                     # INX SP
incdecx(0x12, 7, 'INC', 'BC')                     # INX BC
incdecx(0x22, 7, 'INC', 'DE')                     # INX D
incdecx(0x32, 7, 'INC', 'HL')                     # INX H
incdecx(0x03, 7, 'DEC', 'SP')                     # DCX SP
incdecx(0x13, 7, 'DEC', 'BC')                     # DCX BC
incdecx(0x23, 7, 'DEC', 'DE')                     # DCX D
incdecx(0x33, 7, 'DEC', 'HL')                     # DCX H

daa(0x61, 4)                                      # DAA

jr([0xc0, 0xff], 13)                              # JR
jre(0x4e, 13, '+')                                # JRE (+jdisp)
jre(0x4f, 13, '-')                                # JRE (-jdisp)
jmp(0x54, 10)                                     # JMP word
jb(0x73, 4)                                       # JB
call(0x44, 16)                                    # CALL word
calb(0x63, 13)                                    # CALB
calf([0x78, 0x7f], 16)                            # CALF word
calt([0x80, 0xbf], 19)                            # CALT
softi(0x72, 19)                                   # SOFTI / INT
# Note: Data sheet says 15 cycles, but I think that's a typo.
ret(0x08, 10, 'RET')                              # RET
ret(0x18, 10, 'RETS')                             # RETS
ret(0x62, 13, 'RETI')                             # RETI

bit([0x58, 0x5f], 10)                             # BIT (bit), wa

ins(0x00, 4, 'NOP', 0, [{}])                      # NOP
ins(0x19, 4, 'STM', 0, [{}])                      # STM

######################################################################
# 0x1xx: prefix 0x48

logic(0x130, 8, 'RLL', 'A', '')                   # RLL A
logic(0x131, 8, 'RLR', 'A', '')                   # RLR A
logic(0x132, 8, 'RLL', 'C', '')                   # RLL C
logic(0x133, 8, 'RLR', 'C', '')                   # RLR C
logic(0x134, 8, 'SLL', 'A', '')                   # SLL A
logic(0x135, 8, 'SLR', 'A', '')                   # SLR A
logic(0x136, 8, 'SLL', 'C', '')                   # SLL C
logic(0x137, 8, 'SLR', 'C', '')                   # SLR C

push16(0x10e, 17, 'VA')                           # PUSH V
pop16(0x10f, 14, 'VA')                            # POP V
push16(0x11e, 17, 'BC')                           # PUSH B
pop16(0x11f, 14, 'BC')                            # POP B
push16(0x12e, 17, 'DE')                           # PUSH D
pop16(0x12f, 14, 'DE')                            # POP D
push16(0x13e, 17, 'HL')                           # PUSH H
pop16(0x13f, 14, 'HL')                            # POP H

skip([0x100, 0x104], 8, 'I')                      # SKIT irf
skip(0x10a, 8, 'PSW_C')                           # SKCY
skip(0x10c, 8, 'PSW_Z')                           # SKZ
skip([0x110, 0x114], 8, 'NI')                     # SKNIT irf
skip(0x11a, 8, 'PSW_NC')                          # SKNCY
skip(0x11c, 8, 'PSW_NZ')                          # SKNZ

ins(0x120, 8, 'EI', 0, [{'idx': 1, 'lts': 'IE'}]) # EI
ins(0x124, 8, 'DI', 0, [{'idx': 0, 'lts': 'IE'}]) # DI
ins(0x12A, 8, 'CLC', 0, [{'idx': 0, 'lts': 'PSW_CY'}]) # CLC
ins(0x12B, 8, 'STC', 0, [{'idx': 1, 'lts': 'PSW_CY'}]) # STC

rld(0x138, 17, 'RLD')                             # RLD
rrd(0x139, 17, 'RRD')                             # RRD

######################################################################
# 0x2xx: prefix 0x4C

move([0x2c0, 0x2c9], 8, 'A', 'SPR_IR3')           # MOV A, sr

######################################################################
# 0x3xx: prefix 0x4D

move([0x3c0, 0x3c9], 8, 'SPR_IR3', 'A')           # MOV sr, A

######################################################################
# 0x4xx: prefix 0x60

math([0x420, 0x427], 8, 'ADD', 'RF_IR210', 'A', 'NC')
math([0x430, 0x437], 8, 'SUB', 'RF_IR210', 'A', 'NB')
math([0x440, 0x447], 8, 'ADD', 'RF_IR210', 'A')
math([0x450, 0x457], 8, 'ADC', 'RF_IR210', 'A')
math([0x460, 0x467], 8, 'SUB', 'RF_IR210', 'A')
math([0x470, 0x477], 8, 'SBB', 'RF_IR210', 'A')
math([0x4a0, 0x4a7], 8, 'ADD', 'A', 'RF_IR210', 'NC')
math([0x4b0, 0x4b7], 8, 'SUB', 'A', 'RF_IR210', 'NB')
math([0x4c0, 0x4c7], 8, 'ADD', 'A', 'RF_IR210')
math([0x4d0, 0x4d7], 8, 'ADC', 'A', 'RF_IR210')
math([0x4e0, 0x4e7], 8, 'SUB', 'A', 'RF_IR210')
math([0x4f0, 0x4f7], 8, 'SBB', 'A', 'RF_IR210')

logic([0x408, 0x40f], 8, 'AND', 'RF_IR210', 'A')  # ANA r, A
logic([0x410, 0x417], 8, 'XOR', 'RF_IR210', 'A')  # XRA r, A
logic([0x418, 0x41f], 8, 'OR',  'RF_IR210', 'A')  # ORA r, A
logic([0x488, 0x48f], 8, 'AND', 'A', 'RF_IR210')  # ANA A, r
logic([0x490, 0x497], 8, 'XOR', 'A', 'RF_IR210')  # XRA A, r
logic([0x498, 0x49f], 8, 'OR',  'A', 'RF_IR210')  # ORA A, r

test([0x428, 0x42f], 8, 'GT',  'RF_IR210', 'A')   # GTA r, A
test([0x438, 0x43f], 8, 'LT',  'RF_IR210', 'A')   # LTA r, A
test([0x468, 0x46f], 8, 'NEQ', 'RF_IR210', 'A')   # NEA r, A
test([0x478, 0x47f], 8, 'EQ',  'RF_IR210', 'A')   # EQA r, A
test([0x4a8, 0x4af], 8, 'GT',  'A', 'RF_IR210')   # GTA A, r
test([0x4b8, 0x4bf], 8, 'LT',  'A', 'RF_IR210')   # LTA A, r
test([0x4c8, 0x4cf], 8, 'ON',  'A', 'RF_IR210')   # ONA A, r
test([0x4d8, 0x4df], 8, 'OFF', 'A', 'RF_IR210')   # OFFA A, r
test([0x4e8, 0x4ef], 8, 'NEQ', 'A', 'RF_IR210')   # NEA A, r
test([0x4f8, 0x4ff], 8, 'EQ',  'A', 'RF_IR210')   # EQA A, r

######################################################################
# 0x5xx: prefix 0x64

# ADI(NC)/SUI(NB) r, byte
math_imm([0x520, 0x527], 11, 'ADD', 'RF_IR210', 'NC')
math_imm([0x530, 0x537], 11, 'SUB', 'RF_IR210', 'NB')
math_imm([0x540, 0x547], 11, 'ADD', 'RF_IR210')
math_imm([0x550, 0x557], 11, 'ADC', 'RF_IR210')
math_imm([0x560, 0x567], 11, 'SUB', 'RF_IR210')
math_imm([0x570, 0x577], 11, 'SBB', 'RF_IR210')

logic_imm([0x508, 0x50f], 11, 'AND', 'RF_IR210')  # ANI r, byte
logic_imm([0x510, 0x517], 11, 'XOR', 'RF_IR210')  # XRI r, byte
logic_imm([0x518, 0x51f], 11, 'OR',  'RF_IR210')  # ORI r, byte

math_imm([0x5a0, 0x5a3], 17, 'ADD', 'SPR_IR2', 'NC') # ADINC sr2, byte
math_imm([0x5b0, 0x5b3], 17, 'SUB', 'SPR_IR2', 'NB') # SUINB sr2, byte
math_imm([0x5c0, 0x5c3], 17, 'ADD', 'SPR_IR2')       # ADI sr2, byte
math_imm([0x5d0, 0x5d3], 17, 'ADC', 'SPR_IR2')       # ACI sr2, byte
math_imm([0x5e0, 0x5e3], 17, 'SUB', 'SPR_IR2')       # SUI sr2, byte
math_imm([0x5f0, 0x5f3], 17, 'SBB', 'SPR_IR2')       # SBI sr2, byte

logic_imm([0x588, 0x58b], 17, 'AND', 'SPR_IR2')   # ANI sr2, byte
logic_imm([0x590, 0x593], 17, 'XOR', 'SPR_IR2')   # XRI sr2, byte
logic_imm([0x598, 0x59b], 17, 'OR',  'SPR_IR2')   # ORI sr2, byte

test_imm([0x528, 0x52f], 11, 'GT',  'RF_IR210')   # GTI r, byte
test_imm([0x538, 0x53f], 11, 'LT',  'RF_IR210')   # LTI r, byte
test_imm([0x548, 0x54f], 11, 'ON',  'RF_IR210')   # ONI r, byte
test_imm([0x558, 0x55f], 11, 'OFF', 'RF_IR210')   # OFFI r, byte
test_imm([0x568, 0x56f], 11, 'NEQ', 'RF_IR210')   # NEI r, byte
test_imm([0x578, 0x57f], 11, 'EQ',  'RF_IR210')   # EQI r, byte

test_imm([0x5a8, 0x5ab], 14, 'GT',  'SPR_IR2')    # GTI sr2, byte
test_imm([0x5b8, 0x5bb], 14, 'LT',  'SPR_IR2')    # LTI sr2, byte
test_imm([0x5c8, 0x5cb], 14, 'ON',  'SPR_IR2')    # ONI sr2, byte
test_imm([0x5d8, 0x5db], 14, 'OFF', 'SPR_IR2')    # OFFI sr2, byte
test_imm([0x5e8, 0x5eb], 14, 'NEQ', 'SPR_IR2')    # NEI sr2, byte
test_imm([0x5f8, 0x5fb], 14, 'EQ',  'SPR_IR2')    # EQI sr2, byte

######################################################################
# 0x6xx: prefix 0x70

load_abs([0x668, 0x66f], 17)                      # MOV r, word

load_ind(0x60F, 20, 'SP')                         # LSPD word
load_ind(0x61F, 20, 'BC')                         # LBCD word
load_ind(0x62F, 20, 'DE')                         # LDED word
load_ind(0x63F, 20, 'HL')                         # LHLD word

store_abs([0x678, 0x67f], 17)                     # MOV word, r

store_ind(0x60E, 20, 'SP')                        # SSPD word
store_ind(0x61E, 20, 'BC')                        # SBCD word
store_ind(0x62E, 20, 'DE')                        # SDED word
store_ind(0x63E, 20, 'HL')                        # SHLD word

mathx([0x6a1, 0x6a7], 11, 'ADD', 'NC')            # ADDNCX rpa
mathx([0x6b1, 0x6b7], 11, 'SUB', 'NB')            # SUBNBX rpa
mathx([0x6c1, 0x6c7], 11, 'ADD')                  # ADDX rpa
mathx([0x6d1, 0x6d7], 11, 'ADC')                  # ADCX rpa
mathx([0x6e1, 0x6e7], 11, 'SUB')                  # SUBX rpa
mathx([0x6f1, 0x6f7], 11, 'SBB')                  # SBBX rpa

logicx([0x689, 0x68f], 11, 'AND')                 # ANAX rpa
logicx([0x691, 0x697], 11, 'XOR')                 # XRAX rpa
logicx([0x699, 0x69f], 11, 'OR')                  # ORAX rpa

testx([0x6a9, 0x6af], 11, 'GT')                   # GTAX rpa
testx([0x6b9, 0x6bf], 11, 'LT')                   # LTAX rpa
testx([0x6c9, 0x6cf], 11, 'ON')                   # ONAX rpa
testx([0x6d9, 0x6df], 11, 'OFF')                  # OFFAX rpa
testx([0x6e9, 0x6ef], 11, 'NEQ')                  # NEAX rpa
testx([0x6f9, 0x6ff], 11, 'EQ')                   # EQAX rpa

######################################################################
# 0x7xx: prefix 0x74

math(0x7a0, 14, 'ADD', 'A', 'WA', 'NC')           # ADDNCW wa
math(0x7b0, 14, 'SUB', 'A', 'WA', 'NB')           # SUBNBW wa
math(0x7c0, 14, 'ADD', 'A', 'WA')                 # ADDW wa
math(0x7d0, 14, 'ADC', 'A', 'WA')                 # ADCW wa
math(0x7e0, 14, 'SUB', 'A', 'WA')                 # SUBW wa
math(0x7f0, 14, 'SBB', 'A', 'WA')                 # SBBW wa

logic(0x788, 14, 'AND', 'A', 'WA')                # ANAW wa
logic(0x790, 14, 'XOR', 'A', 'WA')                # XRAW wa
logic(0x798, 14, 'OR', 'A', 'WA')                 # ORAW wa

test(0x7a8, 14, 'GT', 'A', 'WA')                  # GTAW wa
test(0x7b8, 14, 'LT', 'A', 'WA')                  # LTAW wa
test(0x7c8, 14, 'ON', 'A', 'WA')                  # ONAW wa
test(0x7d8, 14, 'OFF', 'A', 'WA')                 # OFFAW wa
test(0x7e8, 14, 'NEQ', 'A', 'WA')                 # NEAW wa
test(0x7f8, 14, 'EQ', 'A', 'WA')                  # EQAW wa

######################################################################

for i in range(len(nc_rows)):
    nc_rows[i] = {'naddr': i} | nc_rows[i] # debugging aid

with open('ucode-gen.yaml', 'w') as f:
    yaml.safe_dump({'ird': {'rows': ird_rows}}, f, sort_keys=False)
    yaml.safe_dump({'urom': {'rows': uc_rows}}, f, sort_keys=False)
    yaml.safe_dump({'nrom': {'rows': nc_rows}}, f, sort_keys=False)
