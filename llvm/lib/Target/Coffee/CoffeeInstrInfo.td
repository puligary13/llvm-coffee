//===-- CoffeeInstrInfo.td - The Coffee Instruction Set ------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the subset of the 32-bit Coffee instruction set, as used
// by the Coffee instruction selector.
//
//===----------------------------------------------------------------------===//

class NeverHasSideEffects {
  bit neverHasSideEffects = 1;
}


include "CoffeeInstrFormats.td"


// Node definition
def SDT_CoffeeJmpLink      : SDTypeProfile<0, 1, [SDTCisVT<0, iPTR>]>;
def SDT_COFFEECallSeqStart : SDCallSeqStart<[ SDTCisVT<0, i32> ]>;
def SDT_COFFEECallSeqEnd   : SDCallSeqEnd<[ SDTCisVT<0, i32>, SDTCisVT<1, i32> ]>;
def SDT_COFFEECmp     : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>]>;
def SDT_COFFEERet          : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_COFFEEDynAlloc    : SDTypeProfile<1, 1, [SDTCisVT<0, iPTR>,
                                               SDTCisSameAs<0, 1>]>;


def SDT_MUL     : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>]>;

def SDT_MUL_64     : SDTypeProfile<0, 1, []>;

//def SDT_COFFEEBrcond  : SDTypeProfile<0, 2,
//                                   [SDTCisVT<0, OtherVT>, SDTCisVT<1, i32>]>;


def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_COFFEECallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;

def callseq_end   : SDNode<"ISD::CALLSEQ_END",   SDT_COFFEECallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

//def SDT_COFFEEcall    : SDTypeProfile<0, -1, [SDTCisPtrTy<0>]>;


// Call
def CoffeeJmpLink : SDNode<"COFFEEISD::JmpLink",SDT_CoffeeJmpLink,
                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                          SDNPVariadic]>;

//def COFFEEcall          : SDNode<"COFFEEISD::CALL", SDT_COFFEEcall,
//                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
//                               SDNPVariadic]>;

def COFFEEret         : SDNode<"COFFEEISD::RET", SDTNone,
                              [SDNPHasChain, SDNPOptInGlue]>;

//guoqing: this might be needed when we need to sup
def MUL              : SDNode<"COFFEEISD::MUL", SDT_MUL, [SDNPOutGlue]>;
def MUL_64           : SDNode<"COFFEEISD::MUL_64", SDT_MUL_64, [SDNPInGlue]>;

//def COFFEEbrcond        : SDNode<"COFFEEISD::BRCOND", SDT_COFFEEBrcond,
//                              [SDNPHasChain, SDNPInGlue]>;

def COFFEEcmp           : SDNode<"COFFEEISD::CMP", SDT_COFFEECmp,
                              [SDNPOutGlue]>;

def COFFEELo    : SDNode<"COFFEEISD::Lo", SDTIntUnaryOp>;
def COFFEEHi    : SDNode<"COFFEEISD::Hi", SDTIntBinOp>;

// Pointer to dynamically allocated stack area.
def COFFEEDynAlloc  : SDNode<"COFFEEISD::DynAlloc", SDT_COFFEEDynAlloc,
                           [SDNPHasChain, SDNPInGlue]>;





def immZExt15  : PatLeaf<(imm), [{ return isInt<15>(N->getZExtValue());  }]>;
def immSExt15  : PatLeaf<(imm), [{ return isInt<15>(N->getSExtValue());  }]>;

def immZExt16  : PatLeaf<(imm), [{ return isUInt<16>(N->getZExtValue());  }]>;
def immSExt16  : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;

def immZExt17  : PatLeaf<(imm), [{ return isUInt<17>(N->getZExtValue());  }]>;
def immSExt17  : PatLeaf<(imm), [{ return isInt<17>(N->getSExtValue()); }]>;


def LO16 : SDNodeXForm<imm, [{
  // Transformation function: get the low 16 bits.
  return getI32Imm((unsigned short)N->getZExtValue());  // unsigned short cast to 16 bit
}]>;

def HI16 : SDNodeXForm<imm, [{
  // Transformation function: shift the immediate value down into the low bits.
  return getI32Imm((unsigned)N->getZExtValue() >> 16);
}]>;


// Addressing modes.
def addr : ComplexPattern<iPTR, 2, "SelectAddr", [frameindex], [SDNPWantParent]>;


/////////////////////////
// Operand definition  //
/////////////////////////

def CoffeeMemAsmOperand : AsmOperandClass {
  let Name = "Mem";
  let ParserMethod = "parseMemOperand";
}


def CoffeeExprAsmOperand : AsmOperandClass {
  let Name = "Expr";
}

def CoffeeAddrAsmOperand : AsmOperandClass {
  let Name = "Addr";
  let ParserMethod = "parseAddrOperand";
}

// address operand is used by ldra pesudo instruction
def address : Operand<iPTR> {
  let ParserMatchClass = CoffeeAddrAsmOperand;
}

def jmptarget   : Operand<OtherVT> {
   let EncoderMethod = "getJumpTargetOpValue";
   let ParserMatchClass = CoffeeExprAsmOperand;
}

def calltarget  : Operand<iPTR> {
  let EncoderMethod = "getJumpTargetOpValue";
  let ParserMatchClass = CoffeeExprAsmOperand;
}

// Operand definition
def bl_target : Operand<i32> {
  let EncoderMethod = "getCoffeeBLTargetOpValue";
  let OperandType = "OPERAND_PCREL";
  let ParserMatchClass = CoffeeExprAsmOperand;
}

def brtarget    : Operand<OtherVT> {
  let EncoderMethod = "getBranchTargetOpValue";
  let OperandType = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranchTarget";
  let ParserMatchClass = CoffeeExprAsmOperand;
}

/// Shift Instructions
def immZExt6 : ImmLeaf<i32, [{return Imm == (Imm & 0x3f);}]>;
def shamt       : Operand<i32>;

def simm15      : Operand<i32>;
def uimm15      : Operand<i32>;

def imm5      : Operand<i32>;
def imm6      : Operand<i32>;
def imm2      : Operand<i32>;
def imm1      : Operand<i32>;

def simm16      : Operand<i32>;
def uimm16      : Operand<i32>;

// the imm operand used in lli lui instruction
def simm16_load  : Operand<i32>, ImmLeaf<i32, [{return isInt<16>(Imm);}]>;
def uimm16_load  : Operand<i32>, ImmLeaf<i32, [{return isUInt<16>(Imm);}]>;

def simm17_cmp  : Operand<i32>, ImmLeaf<i32, [{return isInt<17>(Imm);}]>;
def uimm17_cmp  : Operand<i32>, ImmLeaf<i32, [{return isUInt<17>(Imm);}]>;


def MEMri : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let EncoderMethod = "getMemEncoding";
  let MIOperandInfo = (ops GPRC, simm16);
}

def mem_load : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let EncoderMethod = "getMemLoadEncoding";
  let MIOperandInfo = (ops GPRC, simm15);
  let ParserMatchClass = CoffeeMemAsmOperand;
}

def mem_store : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let EncoderMethod = "getMemStoreEncoding";
  let MIOperandInfo = (ops GPRC, simm15);
  let ParserMatchClass = CoffeeMemAsmOperand;
}

//////////////////////
// class definition //
//////////////////////
class FMem<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
          InstrItinClass itin>: FFI<op, outs, ins, asmstr, pattern>;




// Jump
class JumpFJ<bits<6> op, string instr_asm>:
  FJ<op, (outs), (ins jmptarget:$target),
     !strconcat(instr_asm, "\t$target"), [(br bb:$target)], IIBranch> {
  let isBranch=1;
  let isTerminator=1;
  let isBarrier=1;
  let hasDelaySlot = 1;
  let DecoderMethod = "DecodeJumpTarget";
}

class EffectiveAddress<string instr_asm, RegisterClass RC, Operand Mem> :
  FMem<0b000000, (outs RC:$rt), (ins Mem:$addr),
     instr_asm, [(set RC:$rt, addr:$addr)], IIAlu>;


// Base class for indirect branch and return instruction classes.
let isTerminator=1, isBarrier=1, hasDelaySlot = 1 in
class JumpF3R<bits<6> op, RegisterClass RC, SDPatternOperator operator = null_frag>:
  F3R<op, (outs), (ins RC:$rt), "jmpr\t$rt", [(operator RC:$rt)], IIBranch> {
   let rd = 0;
   let rs = 0;
}

class MovF2R<bits<6> op>:
    F2R<op, (outs GPRC:$rt), (ins GPRC:$rs), "mov\t$rt,\t$rs", [], IIAlu>;

class NotF2R<bits<6> op>:
    F2R<op, (outs GPRC:$rt), (ins GPRC:$rs), "not\t$rt,\t$rs", [], IIAlu>;

// Indirect branch
class IndirectBranch<bits<6> op, RegisterClass RC>: JumpF3R<op, RC, brind> {
  let isBranch = 1;
  let isIndirectBranch = 1;
}

// Return instruction
class RetBase<bits<6> op, RegisterClass RC>: JumpF3R<op, RC> {
  let isReturn = 1;
  let isCodeGenOnly = 1;
  let hasCtrlDep = 1;
  let hasExtraSrcRegAllocReq = 1;
}

// SetCC
class SetCC_R<bits<6> op, string instr_asm, RegisterClass RC>:
  F3R<op, (outs CRRC:$rd), (ins RC:$rt, RC:$rs),
     !strconcat(instr_asm, "\t$rd, $rt, $rs"),
     [/*(set CRRC:$rd, (COFFEEcmp RC:$rt, RC:$rs))*/], IIAlu> {}

class SetCC_I<bits<6> op, string instr_asm, Operand Od, RegisterClass RC>:
  FI<op, (outs CRRC:$rt), (ins RC:$rs, Od:$imm15),
     !strconcat(instr_asm, "\t$rt, $rs, $imm15"),
     [/*(set CRRC:$rd, (COFFEEcmp RC:$rs, imm:$rt))*/], IIAlu> {}


class CBranch<bits<6> op, string instr_asm, PatFrag cond_op>:
  BranchBase<op, (outs), (ins CRRC:$creg, brtarget:$imm22),
             !strconcat(instr_asm, "\t$creg, $imm22"),
             [], IIBranch> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
}

let isCompare = 1 in {
multiclass I_cmp<PatFrag opnode> {
  def ri : CMPI<0b110111, (outs CRRC:$rd ), (ins GPRC:$rt, simm17_cmp:$imm17),
               "cmpi\t$rd\t$rt,\t$imm17", [(set CRRC:$rd, (opnode GPRC:$rt, immSExt17:$imm17))], IIAlu>;

  def riu : CMPI<0b110111, (outs CRRC:$rd ), (ins GPRC:$rt, uimm17_cmp:$imm17),
             "cmpi\t$rd\t$rt,\t$imm17", [(set CRRC:$rd, (opnode GPRC:$rt, immZExt17:$imm17))], IIAlu>;

  def rr : CMPR<0b011001, (outs CRRC:$rd), (ins GPRC:$rt, GPRC:$rs),
               "cmp\t$rd\t$rt,\t$rs", [(set CRRC:$rd, (opnode GPRC:$rt, GPRC:$rs))], IIAlu>;
}
}

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

// class for load & store

class AlignedLoad<PatFrag Node> :
  PatFrag<(ops node:$ptr), (Node node:$ptr), [{
  LoadSDNode *LD = cast<LoadSDNode>(N);
  return LD->getMemoryVT().getSizeInBits()/8 <= LD->getAlignment();
}]>;

class AlignedStore<PatFrag Node> :
  PatFrag<(ops node:$val, node:$ptr), (Node node:$val, node:$ptr), [{
  StoreSDNode *SD = cast<StoreSDNode>(N);
  return SD->getMemoryVT().getSizeInBits()/8 <= SD->getAlignment();
}]>;


// load & store patfrags
def sextloadi16_a   : AlignedLoad<sextloadi16>;
def zextloadi16_a   : AlignedLoad<zextloadi16>;
def extloadi16_a    : AlignedLoad<extloadi16>;
def load_a          : AlignedLoad<load>;
def sextloadi32_a   : AlignedLoad<sextloadi32>;
def zextloadi32_a   : AlignedLoad<zextloadi32>;
def extloadi32_a    : AlignedLoad<extloadi32>;
def truncstorei16_a : AlignedStore<truncstorei16>;
def store_a         : AlignedStore<store>;
def truncstorei32_a : AlignedStore<truncstorei32>;


//load & store instruction

// Memory Load/Store
let canFoldAsLoad = 1 in
class LoadM<bits<6> op, string instr_asm, PatFrag OpNode, RegisterClass RC,
            Operand MemOpnd, bit Pseudo>:
  FMem<op, (outs RC:$rt), (ins MemOpnd:$addr),
     !strconcat(instr_asm, "\t$rt,\t$addr"),
     [(set RC:$rt, (OpNode addr:$addr))], IILoad> {
  bits <5> rt;
  bits<20> addr;

  let Opcode = op;

  let Inst{25} = 0b0;

  let Inst{9-5} = addr{4-0};

  let Inst{24-10} = addr{19-5};
  let Inst{4-0} = rt;
}

class StoreM<bits<6> op, string instr_asm, PatFrag OpNode, RegisterClass RC,
             Operand MemOpnd, bit Pseudo>:
  FMem<op, (outs), (ins RC:$rt, MemOpnd:$addr),
     !strconcat(instr_asm, "\t$rt,\t$addr"),
     [(OpNode RC:$rt, addr:$addr)], IIStore> {

     bits <5> rt;
     bits <20> addr;

    let Opcode = op;
    let Inst{25} = 0b0;
    let Inst{14-10} = rt;

    let Inst{24-15} = addr{19-10};
    let Inst{9-5} = addr{9-5};
    let Inst{4-0} = addr{4-0};

    let Inst{24-20} = addr{14-10};
    let Inst{19-15} = addr{4-0};
    let Inst{9-5} = addr{9-5};
    let Inst{4-0} = addr{19-15};
}


// Arithmetic and logical instructions with 2 register operands.
class ArithLogicI<bits<6> op, string instr_asm, SDNode OpNode,
                  Operand Od, PatLeaf imm_type, RegisterClass RC> :
  FI<op, (outs RC:$rt), (ins RC:$rs, Od:$imm15),
     !strconcat(instr_asm, "\t$rt,\t$rs,\t$imm15"),
     [(set RC:$rt, (OpNode RC:$rs, imm_type:$imm15))], IIAlu> {
  let isReMaterializable = 1;
}

class ArithOverflowI<bits<6> op, string instr_asm, SDNode OpNode,
                     Operand Od, PatLeaf imm_type, RegisterClass RC> :
  FI<op, (outs RC:$rt), (ins RC:$rs, Od:$imm15),
     !strconcat(instr_asm, "\t$rt,\t$rs,\t$imm15"),
     [(set RC:$rt, (OpNode RC:$rs, imm_type:$imm15))], IIAlu>;


class ArithLogicR<bits<6> op, string instr_asm, SDNode OpNode,
                  InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  F3R<op, (outs RC:$rd), (ins RC:$rt, RC:$rs),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$rs"),
     [(set RC:$rd, (OpNode RC:$rt, RC:$rs))], itin> {
  /*let shamt = 0;
  let isCommutable = isComm;
  let isReMaterializable = 1;*/
}

class ArithOverflowR<bits<6> op, string instr_asm, SDNode OpNode,
                    InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  F3R<op, (outs RC:$rd), (ins RC:$rt, RC:$rs),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$rs"),
     [(set RC:$rd, (OpNode RC:$rt, RC:$rs))], itin> {
 /* let shamt = 0;
  let isCommutable = isComm;*/
}


// Load Imediate
class LoadImm<bits<6> op, string instr_asm, RegisterClass RC, Operand Imm>:
  InstCoffee<(outs RC:$rt), (ins Imm:$imm16),
     !strconcat(instr_asm, "\t$rt,\t$imm16"), [], IILoad, FrmI> {


bits<16> imm16;
bits<5> rt;
let Opcode = op;
let Inst{25} = 0b0;
let Inst{4-0} = rt;

let Inst{9} = imm16{15}; //msb
let Inst{24-10} = imm16{14-0};

}


class LoadUpper<bits<6> op, string instr_asm, RegisterClass RC, Operand Imm>:
  InstCoffee<(outs RC:$rd), (ins  RC:$rt, Imm:$imm16),
     !strconcat(instr_asm, "\t$rt,\t$imm16"), [], IILoad, FrmI > {
bits<16> imm16;
bits<5> rt;
let Opcode = op;
let Inst{25} = 0b0;
let Inst{4-0} = rt;

let Inst{9} = imm16{15}; //msb
let Inst{24-10} = imm16{14-0};

let Constraints = "$rd = $rt";

}

class shift_imm<bits<6> opcode, string instr_asm,
                       SDNode OpNode, PatFrag PF, Operand ImmOpnd,
                       RegisterClass RC>:
  F<opcode, (outs RC:$rd), (ins RC:$rt, ImmOpnd:$imm6),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$imm6"),
     [(set RC:$rd, (OpNode RC:$rt, PF:$imm6))], IIAlu> {
bits<6> imm6;
bits<5> rt;
bits<5> rd;
let Inst{15-10} = imm6;
let Inst{9-5} = rt;
let Inst{4-0} = rd;
let Inst{18} = 0b0;
}

// 32-bit shift instructions.
class shift_imm32<bits<6> opcode, string instr_asm,
                         SDNode OpNode>:
  shift_imm<opcode, instr_asm, OpNode, immZExt6, shamt, GPRC>;

class shift_reg<bits<6> opcode, string instr_asm,
                       SDNode OpNode, RegisterClass RC>:
  F3R<opcode, (outs RC:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$rs"),
     [(set RC:$rd, (OpNode RC:$rt, RC:$rs))], IIAlu> {

     let Inst{18} = 0b1; // indicator of regsiter operands instruction for shift
}

class NopF<bits<6> opcode>: F<opcode, (outs), (ins), "nop", [], IIAlu> {
let Inst{15-10} = 0b111010;
}


class RetuF<bits<6> opcode>: F<opcode, (outs), (ins), "retu", [], IIAlu> {
let Inst{9-5} = 0b11111;
}

class RetiF<bits<6> opcode>: F<opcode, (outs), (ins), "reti", [], IIAlu> {
let Inst{24-22} = 0b000;
}

class DiF<bits<6> opcode>: F<opcode, (outs), (ins), "di", [], IIAlu> ;

class EiF<bits<6> opcode>: F<opcode, (outs), (ins), "ei", [], IIAlu> ;

class ScallF<bits<6> opcode>: F<opcode, (outs), (ins), "scall", [], IIAlu> {
let Inst{4-0} = 0b11111;
}

class MulHiF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins), "mulhi\t$rd", [], IIAlu> {
bits <5> rd;
let Inst{4-0} = rd;
}

class SconF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins), "scon\t$rd", [], IIAlu> {
bits <5> rd;
let Inst{4-0} = rd;
}

class RconF<bits<6> opcode>: F<opcode, (outs), (ins GPRC:$rs), "rcon\t$rs", [], IIAlu> {
bits <5> rs;
let Inst{9-5} = rs;
}

class SwmF<bits<6> opcode>: F<opcode, (outs), (ins imm6:$imm), "swm\t$imm", [], IIAlu> {
bits <6> imm;
let Inst{15-10} = imm;
}

class TrapF<bits<6> opcode>: F<opcode, (outs), (ins imm5:$imm), "trap\t$imm", [], IIAlu> {
bits <5> imm;
let Inst{14-10} = imm;
}

class ChrsF<bits<6> opcode>: F<opcode, (outs), (ins imm5:$imm), "chrs\t$imm", [], IIAlu> {
bits <2> imm;
let Inst{11-10} = imm;
}

class ConhF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins GPRC:$rt, GPRC:$rs), "conh\t$rd,\t$rt,\t$rs", [], IIAlu> {
bits<5> rd;
bits<5> rt;
bits<5> rs;
let Inst{14-10} = rt;
let Inst{9-5} = rs;
let Inst{4-0} = rd;
}

class MovtcF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins GPRC:$rt, imm2:$imm), "movtc\t$imm,\t$rd,\t$rt", [], IIAlu> {
bits<5> rd;
bits<5> rt;
bits<2> imm;
let Inst{16-12} = rd;
let Inst{11-10} = imm;
let Inst{9-5} = rt;
}

class ExbF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins GPRC:$rt, imm2:$imm), "exb\t$rd,\t$rt,\t$imm", [], IIAlu> {
bits<5> rd;
bits<5> rt;
bits<2> imm;

let Inst{11-10} = imm;
let Inst{9-5} = rt;
let Inst{4-0} = rd;
}

class ExhF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins GPRC:$rt, imm1:$imm), "exh\t$rd,\t$rt,\t$imm", [], IIAlu> {
bits<5> rd;
bits<5> rt;
bits<1> imm;

let Inst{10} = imm;
let Inst{9-5} = rt;
let Inst{4-0} = rd;
}


class SextiF<bits<6> opcode>: F<opcode, (outs GPRC:$rd), (ins GPRC:$rt, imm5:$imm), "sexti\t$rd,\t$rt,\t$imm", [], IIAlu> {
bits<5> rd;
bits<5> rt;
bits<5> imm;

let Inst{14-10} = imm;
let Inst{9-5} = rt;
let Inst{4-0} = rd;
}

// Jump and Link (Call)
let isCall=1, hasDelaySlot=1, Defs = [LR] in {
  class JumpLink<bits<6> op, string instr_asm>:
    FJ<op, (outs), (ins calltarget:$addr),
       !strconcat(instr_asm, "\t$addr"), [(CoffeeJmpLink imm:$addr)],
       IIBranch>;

  class JumpLinkReg<bits<6> op, string instr_asm,
                    RegisterClass RC>:
    F2R<op, (outs), (ins RC:$rs),
       !strconcat(instr_asm, "\t$rs"), [(CoffeeJmpLink RC:$rs)], IIBranch> {
    let Inst{4-0} = 0b11111;
  }
}

////////////////////////
// Instruction        //
////////////////////////


// Arithmetic Instruction (imm)
def ADDiu   : ArithLogicI<0b101000, "addiu", add, uimm15, immZExt15, GPRC>;
def ADDi    : ArithOverflowI<0b101101, "addi", add, simm15, immSExt15, GPRC>;
def ORi     : ArithLogicI<0b101010, "ori", or, uimm15, immZExt15, GPRC>;
def ANDi    : ArithOverflowI<0b101001, "andi", and, simm15, immSExt15, GPRC>;

def LUi     : LoadImm<0b111111, "lui", GPRC, uimm16_load>;
//LUri is used together with LLiu to handle 32 bit imm or address
def LUri    : LoadUpper<0b111111, "lui", GPRC, uimm16_load>;
def LLi     : LoadImm<0b111110, "lli", GPRC, simm16_load>;
def LLiu    : LoadImm<0b111110, "lli", GPRC, uimm16_load>;



// Arithmetic Instruction (reg)
def ADDu    : ArithLogicR<0b000000, "addu", add, IIAlu, GPRC>;
def SUBu    : ArithLogicR<0b010001, "subu", sub, IIAlu, GPRC>;
def ADD     : ArithOverflowR<0b000001, "add", add, IIAlu, GPRC>;
def SUB     : ArithOverflowR<0b010000, "sub", sub, IIAlu, GPRC>;

def AND     : ArithLogicR<0b000010, "and", and, IIAlu, GPRC>;
def OR      : ArithLogicR<0b001011, "or",  or, IIAlu, GPRC>;
def XOR     : ArithLogicR<0b010010, "xor", xor, IIAlu, GPRC>;

defm CMP :  I_cmp <BinOpFrag<(COFFEEcmp node:$LHS, node:$RHS)>>;

//shift instruction
def SLL     : shift_imm32<0b001101, "slli", shl>;
def SRL     : shift_imm32<0b001111, "srli", srl>;
def SRA     : shift_imm32<0b001110, "srai", sra>;
def SLLR    : shift_reg<0b001101, "sll", shl, GPRC>;
def SRLR    : shift_reg<0b001111, "srl", srl, GPRC>;
def SRAR    : shift_reg<0b001110, "sra", sra, GPRC>;


def MULR   : ArithLogicR<0b000101, "muls", mul, IIImul, GPRC>;
def MULI   : ArithLogicI<0b101110, "muli", mul, simm16, immSExt16, GPRC>;

def MULTR   : ArithLogicR<0b000101, "muls", MUL, IIImul, GPRC>;
def MULTI   : ArithLogicI<0b101110, "muli", MUL, simm16, immSExt16, GPRC>;


/// Load and Store Instructions
///  aligned

// we may enable LB, LH, SB, SH when hardware supports it
// for now, truncate store and extend load are handled as pattern below
//def LB      : LoadM<0b110000, "ldb",  sextloadi8, GPRC, mem_load, 0>;
//def LBu     : LoadM<0b110000, "ldb", zextloadi8,  GPRC, mem_load, 0>;
//def LH      : LoadM<0b110001, "ldh",  sextloadi16,  GPRC, mem_load, 0>;
//def LHu     : LoadM<0b110001, "ldh", zextloadi16,  GPRC, mem_load, 0>;
def LW      : LoadM<0b110010, "ldw",  load,  GPRC, mem_load, 0>;
//def SB      : StoreM<0b000011, "stb", truncstorei8,  GPRC, mem_store, 0>;
//def SH      : StoreM<0b000100, "sth", truncstorei16,  GPRC, mem_store, 0>;
def SW      : StoreM<0b110100, "stw", store,  GPRC, mem_store, 0>;

// condition
def JMP       : JumpFJ<0b111000, "jmp">;
def JMPR      : IndirectBranch<0b011011, GPRC>;
def JAL       : JumpLink<0b111001, "jal">;
def JALR      : JumpLinkReg<0b110101, "jalr", GPRC>;

def BEGT    :  CBranch<0b100001, "begt", setge>;
def BELT    :  CBranch<0b100010, "belt", setle>;
def BEQ     :  CBranch<0b100011, "beq", seteq>;
def BGT     :  CBranch<0b100100, "bgt", setgt>;
def BLT     :  CBranch<0b100101, "blt", setlt>;
def BNE     :  CBranch<0b100110, "bne", setne>;

def RET     :  RetBase<0b011011, GPRC>;


//def NOP     :  SI<0b111010, "nop">;

//def CMPi    : SetCC_I<0b000000, "cmpi", simm16, GPRC>;
//def CMPr     : SetCC_R<0b000000, "cmp", GPRC>;


//Instruction without pattern

def Mov     : MovF2R<0b010011>;
def Not     : NotF2R<0b010100>;
def NOP     : NopF<0b111010>;
def MULHI   : MulHiF<0b011101>;

def RETI     : RetiF<0b010111>;

def RETU     : RetuF<0b011111>;
def SCALL    : ScallF<0b111011>;

def DI     : DiF<0b010101>;
def EI    : EiF<0b010110>;

def RCON   : RconF<0b011110>;
def SCON   : SconF<0b011100>;

def SWM    : SwmF<0b101111>;
def TRAP   : TrapF<0b011000>;
def CHRS   : ChrsF<0b110011>;
def CONH   : ConhF<0b000100>;

def MOVTC   : MovtcF<0b110110>;

def EXB    : ExbF<0b110000>;
def EXH    : ExhF<0b110001>;

def SEXTI  : SextiF<0b101011>;


// pseudo instruction definition

// Return LR.
let isReturn=1, isTerminator=1, hasDelaySlot=1, isBarrier=1, hasCtrlDep=1 in
def RetLR : CoffeePseudo<(outs), (ins), "", [(COFFEEret)]>;

let Defs = [SP], Uses = [SP], hasSideEffects = 1 in {
def ADJCALLSTACKDOWN : CoffeePseudo<(outs), (ins i32imm:$amt),
                               "!ADJCALLSTACKDOWN $amt",
                               [(callseq_start timm:$amt)]>;
def ADJCALLSTACKUP : CoffeePseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "!ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;
}


// FrameIndexes are legalized when they are operands from load/store
// instructions. The same not happens for stack address copies, so an
// add op with mem ComplexPattern is used and the stack address copy
// can be matched. It's similar to Sparc LEA_ADDRi
def LEA_ADDiu : EffectiveAddress<"addiu\t$rt,\t$addr", GPRC, MEMri> {
  let isCodeGenOnly = 1;
}

// DynAlloc node points to dynamically allocated stack space.
// $sp is added to the list of implicitly used registers to prevent dead code
// elimination from removing instructions that modify $sp.
let Uses = [SP] in
def DynAlloc : EffectiveAddress<"addiu\t$rt,\t$addr", GPRC, MEMri> {
  let isCodeGenOnly = 1;
}




//===----------------------------------------------------------------------===//
// Pseudo Instruction definition
//===----------------------------------------------------------------------===//


class LoadImm32< string instr_asm, Operand Od, RegisterClass RC> :
  CoffeeAsmPseudo<(outs RC:$rt), (ins Od:$imm32),
                     !strconcat(instr_asm, "\t$rt,\t$imm32"), []> ;

def LoadImm32Reg : LoadImm32<"ldri", shamt, GPRC>;

class LoadAddress<string instr_asm, Operand MemOpnd, RegisterClass RC> :
  CoffeeAsmPseudo<(outs RC:$rt), (ins MemOpnd:$addr),
                     !strconcat(instr_asm, "\t$rt,\t$addr"), []> ;
def LoadAddr32Reg : LoadAddress<"ldra", address, GPRC>;

//class LoadAddressImm<string instr_asm, Operand Od, RegisterClass RC> :
//  CoffeeAsmPseudo<(outs RC:$rt), (ins Od:$imm32),
                     //!strconcat(instr_asm, "\t$rt,\t$imm32"), []> ;
//def LoadAddr32Imm : LoadAddressImm<"ldra", shamt, GPRC>;


def Pushr : CoffeeAsmPseudo<(outs), (ins GPRC:$rs), "push\t$rs", []> ;
def Popr : CoffeeAsmPseudo<(outs), (ins GPRC:$rs), "pop\t$rs", []> ;
def St : CoffeeAsmPseudo<(outs GPRC:$rd), (ins GPRC:$rt, simm15:$imm15), "st\t$rd\t$rt\t$imm15", []> ;
def Ld : CoffeeAsmPseudo<(outs GPRC:$rd), (ins GPRC:$rt, simm15:$imm15), "ld\t$rd\t$rt\t$imm15", []> ;



//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

def : Pat<(i32 immSExt16:$val),
          (LLi (LO16 imm:$val))>;
def : Pat<(i32 immZExt16:$val),
          (LLiu (LO16 imm:$val))>;
// big immediates
def : Pat<(i32 imm:$val),
          (LUri (LLiu (LO16 imm:$val)), (HI16 imm:$val))>;

def : Pat<(i32 imm:$val),
          (LUri (LLiu (LO16 imm:$val)), (HI16 imm:$val))>;

//load & store
def : Pat<(i32 (extloadi1  addr:$src)), (LW addr:$src)>;
def : Pat<(i32 (extloadi8  addr:$src)), (LW addr:$src)>;
def : Pat<(i32 (extloadi16 addr:$src)), (LW addr:$src)>;

def : Pat<(i32 (zextloadi1  addr:$src)), (LW addr:$src)>;
def : Pat<(i32 (zextloadi8  addr:$src)), (LW addr:$src)>;
def : Pat<(i32 (zextloadi16 addr:$src)), (LW addr:$src)>;

def : Pat<(i32 (sextloadi1  addr:$src)), (SEXTI (LW addr:$src), 7)>;
def : Pat<(i32 (sextloadi8  addr:$src)), (SEXTI (LW addr:$src), 7)>;
def : Pat<(i32 (sextloadi16 addr:$src)), (SEXTI (LW addr:$src), 15)>;

def : Pat<(truncstorei8 GPRC:$rt, addr:$src), (SW (EXB GPRC:$rt, 0), addr:$src)>;
def : Pat<(truncstorei16 GPRC:$rt, addr:$src), (SW (EXH GPRC:$rt, 0), addr:$src)>;

// select COFFEEDynAlloc
def : Pat<(COFFEEDynAlloc addr:$f), (DynAlloc addr:$f)>;

// immediates definition

// global address
def : Pat<(i32 tglobaladdr:$val),
          (LUri (LLiu (LO16 imm:$val)), (HI16 imm:$val))>;

//hi/lo relocs
def : Pat<(COFFEELo tglobaladdr:$in), (LLiu tglobaladdr:$in)>;
def : Pat<(COFFEELo tblockaddress:$in), (LLiu tblockaddress:$in)>;
def : Pat<(COFFEELo tjumptable:$in), (LLiu tjumptable:$in)>;
def : Pat<(COFFEELo tconstpool:$in), (LLiu tconstpool:$in)>;

def : Pat<(COFFEEHi GPRC:$src, tglobaladdr:$in), (LUri GPRC:$src, tglobaladdr:$in)>;
def : Pat<(COFFEEHi GPRC:$src, tblockaddress:$in), (LUri GPRC:$src, tblockaddress:$in)>;
def : Pat<(COFFEEHi GPRC:$src, tjumptable:$in), (LUri GPRC:$src, tjumptable:$in)>;
def : Pat<(COFFEEHi GPRC:$src, tconstpool:$in), (LUri GPRC:$src, tconstpool:$in)>;


// Call
def : Pat<(CoffeeJmpLink (i32 tglobaladdr:$dst)),
              (JAL tglobaladdr:$dst)>;
def : Pat<(CoffeeJmpLink (i32 texternalsym:$dst)),
              (JAL texternalsym:$dst)>;

// brcond patterns

// != imm
def : Pat<(brcond (i32 (setne GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BNE (CMPri GPRC:$lhs, immSExt16:$rhs), bb:$dst)>;

// == imm
def : Pat<(brcond (i32 (seteq GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BEQ (CMPri GPRC:$lhs, immSExt16:$rhs), bb:$dst)>;

// != reg
def : Pat<(brcond (i32 (setne GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BNE (CMPrr GPRC:$lhs, GPRC:$rhs), bb:$dst)>;
// == reg
def : Pat<(brcond (i32 (seteq GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BEQ (CMPrr GPRC:$lhs, GPRC:$rhs), bb:$dst)>;

// >= reg
def : Pat<(brcond (i32 (setge GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BGT (CMPrr GPRC:$lhs, GPRC:$rhs), bb:$dst)>;

// >= imm
def : Pat<(brcond (i32 (setge GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BGT (CMPri GPRC:$lhs, immSExt16:$rhs), bb:$dst)>;

// <= reg
def : Pat<(brcond (i32 (setle GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BELT (CMPrr GPRC:$rhs, GPRC:$lhs), bb:$dst)>;
// <= imm
def : Pat<(brcond (i32 (setle GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BELT (CMPri GPRC:$rhs, immSExt16:$lhs), bb:$dst)>;

// > reg
def : Pat<(brcond (i32 (setge GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BGT (CMPrr GPRC:$lhs, GPRC:$rhs), bb:$dst)>;

// > imm
def : Pat<(brcond (i32 (setge GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BGT (CMPri GPRC:$lhs, immSExt16:$rhs), bb:$dst)>;

// < reg
def : Pat<(brcond (i32 (setlt GPRC:$lhs, GPRC:$rhs)), bb:$dst),
          (BLT (CMPrr GPRC:$rhs, GPRC:$lhs), bb:$dst)>;
// < imm
def : Pat<(brcond (i32 (setlt GPRC:$lhs, immSExt16:$rhs)), bb:$dst),
          (BLT (CMPri GPRC:$rhs, immSExt16:$lhs), bb:$dst)>;


def : Pat<(brcond GPRC:$cond, bb:$dst),
          (BNE (CMPri GPRC:$cond, 0), bb:$dst)>;

//===----------------------------------------------------------------------===//
// Floating Point Support
//===----------------------------------------------------------------------

include "CoffeeInstrFPU.td"


