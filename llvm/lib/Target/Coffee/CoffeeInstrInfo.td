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

include "CoffeeInstrFormats.td"


// Node definition
def SDT_COFFEECallSeqStart : SDCallSeqStart<[ SDTCisVT<0, i32> ]>;
def SDT_COFFEECallSeqEnd   : SDCallSeqEnd<[ SDTCisVT<0, i32>, SDTCisVT<1, i32> ]>;
def SDT_COFFEECmp     : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
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

def SDT_COFFEEcall    : SDTypeProfile<0, -1, [SDTCisPtrTy<0>]>;


def COFFEEcall          : SDNode<"COFFEEISD::CALL", SDT_COFFEEcall,
                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                               SDNPVariadic]>;

def COFFEEret         : SDNode<"COFFEEISD::RET", SDT_COFFEERet,
                              [SDNPHasChain, SDNPOptInGlue]>;

//guoqing: this might be needed when we need to sup
def MUL              : SDNode<"COFFEEISD::MUL", SDT_MUL, [SDNPOutGlue]>;
def MUL_64           : SDNode<"COFFEEISD::MUL_64", SDT_MUL_64, [SDNPInGlue]>;

//def COFFEEbrcond        : SDNode<"COFFEEISD::BRCOND", SDT_COFFEEBrcond,
//                              [SDNPHasChain, SDNPInGlue]>;

def COFFEEcmp           : SDNode<"COFFEEISD::CMP", SDT_COFFEECmp,
                              [SDNPOutGlue]>;

def COFFEEHi    : SDNode<"COFFEEISD::Hi", SDTIntUnaryOp>;
def COFFEELo    : SDNode<"COFFEEISD::Lo", SDTIntBinOp>;

// Pointer to dynamically allocated stack area.
def COFFEEDynAlloc  : SDNode<"COFFEEISD::DynAlloc", SDT_COFFEEDynAlloc,
                           [SDNPHasChain, SDNPInGlue]>;


def immSExt16  : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;

def jmptarget   : Operand<OtherVT> {
 // let EncoderMethod = "getJumpTargetOpValue";
}

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


// Operand definition
def bl_target : Operand<i32> {
  let EncoderMethod = "getCoffeeBLTargetOpValue";
  let OperandType = "OPERAND_PCREL";
}

def brtarget    : Operand<OtherVT> {
  let EncoderMethod = "getBranchTargetOpValue";
  let OperandType = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranchTarget";
}


def simm16      : Operand<i32> {
  //let DecoderMethod= "DecodeSimm16";
}

def uimm16      : Operand<i32> {
}

//guoqing: we use i32 for now, this needs to recheck later
// to use the right type
// Address operands
def MEMri : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops GPRC, simm16);
}

/*def mem_ea : Operand<i32> {
  let PrintMethod = "printMemOperandEA";
  let MIOperandInfo = (ops GPRC, i32imm);
  let EncoderMethod = "getMemEncoding";
}*/


// class definition

class FMem<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
          InstrItinClass itin>: FFI<op, outs, ins, asmstr, pattern> {

}

// Instruction definition




// arithmetic instructions

def ADDrr : InstCoffee_0<0b000001,
                       (outs GPRC:$Rd),
                       (ins GPRC:$R1, GPRC:$R2),
                       "add\t$Rd,\t$R1,\t$R2",
                       [(set GPRC:$Rd, (add GPRC:$R1, GPRC:$R2))] > {
                       bits<5> Rd;
                       bits<5> R1;
                       bits<5> R2;
                       let Inst{14-10} = R2;
                       let Inst{9-5} = R1;
                       let Inst{4-0} = Rd;
                       }

def ADDri : InstCoffee_0<0b101101,
                       (outs GPRC:$Rd),
                       (ins GPRC:$R1, i32imm:$R2), //we have 15bits
                       "addi\t$Rd,\t$R1,\t$R2",
                       [(set GPRC:$Rd, (add GPRC:$R1, imm:$R2))] > {
                       bits<15> R2;
                       bits<5> R1;
                       bits<5> Rd;
                       let Inst{24-10} = R2;
                       let Inst{9-5} = R1;
                       let Inst{4-0} = Rd;
                       }


/*def ORrr : InstCoffee_0<0b001011, (outs GPRC:$dst), (ins GPRC:$r1, GPRC:$r2),
                       "or\t$dst,\t$r1,\t$r2",
                       [(set GPRC:$dst, (or GPRC:$r1, GPRC:$r2))]> {
                       bits<5> r2;
                       bits<5> r1;
                       bits<5> dst;
                       let Inst{14-10} = r2;
                       let Inst{9-5} = r1;
                       let Inst{4-0} = dst;
                       }*/


def ORri : InstCoffee_0<0b101010, (outs GPRC:$dst), (ins GPRC:$r1, i32imm:$r2),
                        "ori\t$dst,\t$r1,\t$r2",
                        [(set GPRC:$dst, (or GPRC:$r1, imm:$r2))]> {
                        bits<15> r2; //imm
                        bits<5> r1;
                        bits<5> dst;
                        let Inst{24-10} = r2;
                        let Inst{9-5} = r1;
                        let Inst{4-0} = dst;
                        }


//  Control Flow Instructions.


let isReturn=1, isTerminator=1, hasDelaySlot=1, isCodeGenOnly=1,
    isBarrier=1 in
  def RET : FR <0b011011, (outs), (ins GPRC:$target),
                "jmpr\t$target", [(COFFEEret GPRC:$target)], IIBranch>;

/*let isReturn = 1, isTerminator = 1, isBarrier = 1 in {
  def JMPR_RET : InstCoffee_0<0b011011,
                       (outs),
                       (ins),
                       "jmpr\tlr", [(COFFEEretflag)]> {
                       let Inst{9-5} = 0b11111;
                       }
}*/

let isCall = 1, Defs = [LR], hasDelaySlot=1 in {
  def JALR_CALL : InstCoffee_0<0b011011,
                  (outs),
                  (ins GPRC:$func, variable_ops),
                  "jalr\t$func", [(COFFEEcall GPRC:$func)]> {
                  bits<5> func;
                  let Inst{9-5} = func;
                  let Inst{4-0} = 0b11111;
                  }



  def JAL_CALL : InstCoffee_0<0b011011,
                  (outs),
                  (ins bl_target:$func, variable_ops),
                  "jal\t$func", [(COFFEEcall tglobaladdr:$func)]> {
                  bits<5> func;
                  let Inst{9-5} = func;
                  let Inst{4-0} = 0b11111;
                  }
}

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

/*class SI<bits<6> op, string instr_asms>:
    FJ<op, (outs), (ins),
   instr_asms, [], IIAlu> {}*/

def NOP   : InstCoffee<(outs), (ins), "nop", [], IIAlu, FrmJ> {
  let Inst{31-26} = 0b111010;
  let Inst{15-10} = 0b111010;
  }



class EffectiveAddress<string instr_asm, RegisterClass RC, Operand Mem> :
  FMem<0b000000, (outs RC:$rt), (ins Mem:$addr),
     instr_asm, [(set RC:$rt, addr:$addr)], IIAlu>;




class JumpFR<bits<6> op, string instr_asm, RegisterClass RC>:
  FR<op, (outs), (ins RC:$rs),
     !strconcat(instr_asm, "\t$rs"), [(brind RC:$rs)], IIBranch> {
  let isBranch=1;
  let isTerminator = 1;
  let isBarrier = 1;
  let hasDelaySlot = 1;
  let isIndirectBranch = 1;
}



// SetCC
class SetCC_R<bits<6> op, string instr_asm, RegisterClass RC>:
  FR<op, (outs CRRC:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd, $rs, $rt"),
     [/*(set CRRC:$rd, (COFFEEcmp RC:$rs, RC:$rt))*/], IIAlu> {}

class SetCC_I<bits<6> op, string instr_asm, Operand Od, RegisterClass RC>:
  FI<op, (outs CRRC:$rt), (ins RC:$rs, Od:$imm16),
     !strconcat(instr_asm, "\t$rt, $rs, $imm16"),
     [/*(set CRRC:$rd, (COFFEEcmp RC:$rs, imm:$rt))*/], IIAlu> {}


class CBranch<bits<6> op, string instr_asm, PatFrag cond_op>:
  BranchBase<op, (outs), (ins CRRC:$creg, brtarget:$imm16),
             !strconcat(instr_asm, "\t$creg, $imm16"),
             [], IIBranch> {
  let isBranch = 1;
  let isTerminator = 1;
  let hasDelaySlot = 1;
}

def JMP       : JumpFJ<0b000000, "jmp">;
def JMPR      : JumpFR<0b000000, "jmpr", GPRC>;
//def JAL       : JumpLink<0b000000, "jal">;
//def JALR      : JumpLinkReg<0b000000, "jalr", GPRC>;

def BEGT    :  CBranch<0b000000, "begt", setge>;
def BELT    :  CBranch<0b000000, "belt", setle>;
def BEQ     :  CBranch<0b000000, "beq", seteq>;
def BGT     :  CBranch<0b000000, "bgt", setgt>;
def BLT     :  CBranch<0b000000, "blt", setlt>;
def BNE     :  CBranch<0b000000, "bne", setne>;

//def NOP     :  SI<0b111010, "nop">;

//def CMPi    : SetCC_I<0b000000, "cmpi", simm16, GPRC>;
//def CMPr     : SetCC_R<0b000000, "cmp", GPRC>;


let isCompare = 1, Defs = [CR0] in {
multiclass I_cmp<PatFrag opnode> {
  def ri : InstCoffee<(outs), (ins GPRC:$Rn, simm16:$imm),
               "cmpi\t$Rn,\t$imm", [(opnode GPRC:$Rn, imm:$imm)], IIAlu, FrmI> {}
  def rr : InstCoffee<(outs), (ins GPRC:$Rn, GPRC:$Rm),
               "cmp\t$Rn,\t$Rm", [(opnode GPRC:$Rn, GPRC:$Rm)], IIAlu, FrmI> {}
}
}

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

defm CMP :  I_cmp <BinOpFrag<(COFFEEcmp node:$LHS, node:$RHS)>>;


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
  let isPseudo = Pseudo;
}

class StoreM<bits<6> op, string instr_asm, PatFrag OpNode, RegisterClass RC,
             Operand MemOpnd, bit Pseudo>:
  FMem<op, (outs), (ins RC:$rt, MemOpnd:$addr),
     !strconcat(instr_asm, "\t$rt,\t$addr"),
     [(OpNode RC:$rt, addr:$addr)], IIStore> {
  let isPseudo = Pseudo;
}


// Arithmetic and logical instructions with 2 register operands.
class ArithLogicI<bits<6> op, string instr_asm, SDNode OpNode,
                  Operand Od, PatLeaf imm_type, RegisterClass RC> :
  FI<op, (outs RC:$rt), (ins RC:$rs, Od:$imm16),
     !strconcat(instr_asm, "\t$rt,\t$rs,\t$imm16"),
     [(set RC:$rt, (OpNode RC:$rs, imm_type:$imm16))], IIAlu> {
  let isReMaterializable = 1;
}

class ArithOverflowI<bits<6> op, string instr_asm, SDNode OpNode,
                     Operand Od, PatLeaf imm_type, RegisterClass RC> :
  FI<op, (outs RC:$rt), (ins RC:$rs, Od:$imm16),
     !strconcat(instr_asm, "\t$rt, $rs, $imm16"), [], IIAlu>;


class ArithLogicR<bits<6> op, string instr_asm, SDNode OpNode,
                  InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd,\t$rs,\t$rt"),
     [(set RC:$rd, (OpNode RC:$rs, RC:$rt))], itin> {
  /*let shamt = 0;
  let isCommutable = isComm;
  let isReMaterializable = 1;*/
}

class ArithOverflowR<bits<6> op, string instr_asm,
                    InstrItinClass itin, RegisterClass RC, bit isComm = 0>:
  FR<op, (outs RC:$rd), (ins RC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd,\t$rs,\t$rt"), [], itin> {
 /* let shamt = 0;
  let isCommutable = isComm;*/
}


// Load Upper Imediate
class LoadUpper<bits<6> op, string instr_asm, RegisterClass RC, Operand Imm>:
  FI<op, (outs RC:$rt), (ins Imm:$imm16),
     !strconcat(instr_asm, "\t$rt,\t$imm16"), [], IIAlu> {

  let neverHasSideEffects = 1;
  let isReMaterializable = 1;
}


def immZExt16  : PatLeaf<(imm), [{return (uint32_t)N->getZExtValue() == (unsigned short)N->getZExtValue();}], LO16>;

// Arithmetic Instruction (imm)
def ADDiu   : ArithLogicI<0b000000, "addiu", add, simm16, immSExt16, GPRC>;
def ADDi    : ArithOverflowI<0b000000, "addi", add, simm16, immSExt16, GPRC>;
def ORi     : ArithLogicI<0b000000, "ori", or, uimm16, immZExt16, GPRC>;
def LUi     : LoadUpper<0b000000, "lui", GPRC, uimm16>;

// Arithmetic Instruction (reg)
def ADDu    : ArithLogicR<0b000000, "addu", add, IIAlu, GPRC>;
def SUBu    : ArithLogicR<0b000000, "subu", sub, IIAlu, GPRC>;
def ADD     : ArithOverflowR<0b000000, "add", IIAlu, GPRC>;
def SUB     : ArithOverflowR<0b000000, "sub", IIAlu, GPRC>;

def AND     : ArithLogicR<0b000000, "and", and, IIAlu, GPRC>;
def OR      : ArithLogicR<0b000000, "or",  or, IIAlu, GPRC>;
def XOR     : ArithLogicR<0b000000, "xor", xor, IIAlu, GPRC>;

/// Load and Store Instructions
///  aligned

// these are pesudo instruction
def LB      : LoadM<0x00, "lb",  sextloadi8, GPRC, MEMri, 1>;
def LBu     : LoadM<0x00, "lbu", zextloadi8,  GPRC, MEMri, 1>;
def LH      : LoadM<0x00, "lh",  sextloadi16_a,  GPRC, MEMri, 1>;
def LHu     : LoadM<0x00, "lhu", zextloadi16_a,  GPRC, MEMri, 1>;
def LW      : LoadM<0x00, "lw",  load_a,  GPRC, MEMri, 1>;
def SB      : StoreM<0x00, "sb", truncstorei8,  GPRC, MEMri, 1>;
def SH      : StoreM<0x00, "sh", truncstorei16_a,  GPRC, MEMri, 1>;
def SW      : StoreM<0x00, "sw", store_a,  GPRC, MEMri, 1>;



//load lower half, no pattern, use for store constant to a register
def Loadlinp : InstCoffee<(outs GPRC:$dst),
                         (ins i32imm:$src),
                         "lli\t$dst,\t$src",
                         [], IILoad, FrmFI> {
                         bits<15> imm;
                         bits<5> dst;
                         bits<1> msb;
                         let Inst{24-10} = imm;
                         let Inst{9} = msb;
                         let Inst{4-0} = dst;
                         }


// load upper half, no patern, use for store constant to a register
def Loaduinp : InstCoffee<(outs GPRC:$dst),
                         (ins i32imm:$src),
                         "lui\t$dst,\t$src",
                         [], IIStore, FrmFI> {
                         bits<15> imm;
                         bits<5> dst;
                         bits<1> msb;
                         let Inst{24-10} = imm;
                         let Inst{9} = msb;
                         let Inst{4-0} = dst;
                         }

//store from register to memory
/*def STORErm : InstCoffee_0<0b110100,
                 (outs), (ins GPRC:$src, MEMri:$addr),
                 "st\t$src,\t$addr",
                 [(store GPRC:$src, addr:$addr)]> {
                 // dummy
                 }

// load from memory to register
//def LOADmr  : InstCoffee_0<0b110010,
//                 (outs GPRC:$dst), (ins MEMri:$addr),
//                 "ld\t$dst,\t$addr",
//                 [(set GPRC:$dst, (load addr:$addr))]> {

                 //dummy
                 }*/

// load no pattern, used directly from emitPopInstr in CoffeeFrameLowering.cpp
//def LOADmrnp  : InstCoffee_0<0b110010,
//                 (outs), (ins GPRC:$dst, MEMri:$addr),
//                 "ld\t$dst,\t$addr",
//                 []> {

                 //dummy
//                 }


// pseudo instruction definition

def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                               "!ADJCALLSTACKDOWN $amt",
                               [(callseq_start timm:$amt)]>;
def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "!ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;



// immediates definition

def : Pat<(i32 immSExt16:$val),
          (Loadlinp (LO16 imm:$val))>;

// big immediates
def : Pat<(i32 imm:$val),
          (ORri (Loaduinp (HI16 imm:$val)), (LO16 imm:$val))>;

// global address
def : Pat<(i32 tglobaladdr:$val),
          (ORri (Loaduinp (HI16 imm:$val)), (LO16 imm:$val))>;


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

//hi/lo relocs

def : Pat<(COFFEEHi tglobaladdr:$in), (LUi tglobaladdr:$in)>;


def : Pat<(COFFEELo GPRC:$src, tglobaladdr:$in), (ORi GPRC:$src, tglobaladdr:$in)>;



/// Shift Instructions
def immZExt6 : ImmLeaf<i32, [{return Imm == (Imm & 0x3f);}]>;
def shamt       : Operand<i32>;

class shift_imm<bits<6> opcode, string instr_asm,
                       SDNode OpNode, PatFrag PF, Operand ImmOpnd,
                       RegisterClass RC>:
  FR<opcode, (outs RC:$rd), (ins RC:$rt, ImmOpnd:$shamt),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$shamt"),
     [(set RC:$rd, (OpNode RC:$rt, PF:$shamt))], IIAlu> {

}

// 32-bit shift instructions.
class shift_imm32<bits<6> opcode, string instr_asm,
                         SDNode OpNode>:
  shift_imm<opcode, instr_asm, OpNode, immZExt6, shamt, GPRC>;

class shift_reg<bits<6> opcode, string instr_asm,
                       SDNode OpNode, RegisterClass RC>:
  FR<opcode, (outs RC:$rd), (ins GPRC:$rs, RC:$rt),
     !strconcat(instr_asm, "\t$rd,\t$rt,\t$rs"),
     [(set RC:$rd, (OpNode RC:$rt, GPRC:$rs))], IIAlu> {

}


def SLLI     : shift_imm32<0b000000, "slli", shl>;
def SRLI     : shift_imm32<0b000000, "srli", srl>;
def SRAI     : shift_imm32<0b000000, "srai", sra>;
def SLLR    : shift_reg<0b000000, "sll", shl, GPRC>;
def SRLR    : shift_reg<0b000000, "srl", srl, GPRC>;
def SRAR    : shift_reg<0b000000, "sra", sra, GPRC>;


// select COFFEEDynAlloc
def : Pat<(COFFEEDynAlloc addr:$f), (DynAlloc addr:$f)>;

def MULR   : ArithLogicR<0b000000, "muls", mul, IIImul, GPRC>;
def MULI   : ArithLogicI<0b000000, "muli", mul, simm16, immSExt16, GPRC>;

def MULTR   : ArithLogicR<0b000000, "muls", MUL, IIImul, GPRC>;
def MULTI   : ArithLogicI<0b000000, "muli", MUL, simm16, immSExt16, GPRC>;

// no pattern

/*class HI<string instr_asm, RegisterClass RC,
                   list<Register> UseRegs>:
  FR<0b000000, func, (outs RC:$rd), (ins),
     !strconcat(instr_asm, "\t$rd"), [], IIHiLo> {
  let rs = 0;
  let rt = 0;
  let shamt = 0;
  let Uses = UseRegs;
  let neverHasSideEffects = 1;
}*/

def MULHI   : InstCoffee<(outs GPRC:$rd), (ins), "mulhi\t$rd", [], IIAlu, FrmJ> {
  let Inst{31-26} = 0b111010;
  let Inst{15-10} = 0b111010;
  }
