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

def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_COFFEECallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END",   SDT_COFFEECallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;
def SDT_COFFEEcall    : SDTypeProfile<0, -1, [SDTCisPtrTy<0>]>;

def COFFEEcall          : SDNode<"COFFEEISD::CALL", SDT_COFFEEcall,
                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                               SDNPVariadic]>;
def COFFEEretflag       : SDNode<"COFFEEISD::RET_FLAG", SDTNone,
                              [SDNPHasChain, SDNPOptInGlue]>;



def LO16 : SDNodeXForm<imm, [{
  // Transformation function: get the low 16 bits.
  return getI32Imm((unsigned short)N->getZExtValue());  // unsigned short cast to 16 bit
}]>;

def HI16 : SDNodeXForm<imm, [{
  // Transformation function: shift the immediate value down into the low bits.
  return getI32Imm((unsigned)N->getZExtValue() >> 16);
}]>;


// Addressing modes.
def ADDRrr : ComplexPattern<i32, 2, "SelectADDRrr", [], []>;
def ADDRri : ComplexPattern<i32, 2, "SelectADDRri", [frameindex], []>;

// Operand definition
def bl_target : Operand<i32> {
  let EncoderMethod = "getCoffeeBLTargetOpValue";
  let OperandType = "OPERAND_PCREL";
}

// Address operands
def MEMri : Operand<i32> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops GPRC, i32imm);
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
                       "addi\t$Rd,\t$R1,\t,$R2",
                       [(set GPRC:$Rd, (add GPRC:$R1, imm:$R2))] > {
                       bits<15> R2;
                       bits<5> R1;
                       bits<5> Rd;
                       let Inst{24-10} = R2;
                       let Inst{9-5} = R1;
                       let Inst{4-0} = Rd;
                       }


def ORrr : InstCoffee_0<0b001011, (outs GPRC:$dst), (ins GPRC:$r1, GPRC:$r2),
                       "or\t$dst,\t$r1,\t$r2",
                       [(set GPRC:$dst, (or GPRC:$r1, GPRC:$r2))]> {
                       bits<5> r2;
                       bits<5> r1;
                       bits<5> dst;
                       let Inst{14-10} = r2;
                       let Inst{9-5} = r1;
                       let Inst{4-0} = dst;
                       }


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

let isReturn = 1, isTerminator = 1, isBarrier = 1 in {
  def JMPR_RET : InstCoffee_0<0b011011,
                       (outs),
                       (ins),
                       "jmpr\tlr", [(COFFEEretflag)]> {
                       let Inst{9-5} = 0b11111;
                       }
}

let isCall = 1, Defs = [LR] in {
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

//load & store instruction

//load lower half, no pattern, use for store constant to a register
def Loadlinp : InstCoffee_0<0b111110,
                         (outs GPRC:$dst),
                         (ins i32imm:$src),
                         "lli\t$dst,\t$src",
                         []> {
                         bits<15> imm;
                         bits<5> dst;
                         bits<1> msb;
                         let Inst{24-10} = imm;
                         let Inst{9} = msb;
                         let Inst{4-0} = dst;
                         }





// load upper half, no patern, use for store constant to a register
def Loaduinp : InstCoffee_0<0b111111,
                         (outs GPRC:$dst),
                         (ins i32imm:$src),
                         "lui\t$dst,\t$src",
                         []> {
                         bits<15> imm;
                         bits<5> dst;
                         bits<1> msb;
                         let Inst{24-10} = imm;
                         let Inst{9} = msb;
                         let Inst{4-0} = dst;
                         }


def STORErr  : InstCoffee_0<0b110100,
                 (outs), (ins MEMri:$addr, GPRC:$src),
                 "st\t$src,\t$addr",
                 [(store GPRC:$src, ADDRrr:$addr)]> {
                 bits<5> R1;
                 bits<5> R2;
                 let Inst{24-15} = 0b0000000000;
                 let Inst{4-0} = 0b00000;
                 let Inst{14-10} = R2;
                 let Inst{9-5} = R1;
                 }


//def LOADrr  : InstCoffee_0<0b110100,
//                 (outs), (ins MEMri:$addr, GPRC:$src),
//                 "ld\t$src,\t$addr",
//                 [(load GPRC:$src, ADDRrr:$addr)]> {
//                 bits<5> R1;
//                 bits<5> R2;
//                 let Inst{24-15} = 0b0000000000;
//                 let Inst{4-0} = 0b00000;
//                 let Inst{14-10} = R2;
//                 let Inst{9-5} = R1;
//                 }


//store from register to memory
def STORErm : InstCoffee_0<0b110100,
                 (outs), (ins GPRC:$src, MEMri:$addr),
                 "st\t$src,\t$addr",
                 [(store GPRC:$src, ADDRri:$addr)]> {
                 // dummy
                 }

// load from memory to register
def LOADmr  : InstCoffee_0<0b110010,
                 (outs GPRC:$dst), (ins MEMri:$addr),
                 "ld\t$dst,\t$addr",
                 [(set GPRC:$dst, (load ADDRri:$addr))]> {

                 //dummy
                 }

// load no pattern, used directly from emitPopInstr in CoffeeFrameLowering.cpp
def LOADmrnp  : InstCoffee_0<0b110010,
                 (outs), (ins GPRC:$dst, MEMri:$addr),
                 "ld\t$dst,\t$addr",
                 []> {

                 //dummy
                 }


// pseudo instruction definition

def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                               "!ADJCALLSTACKDOWN $amt",
                               [(callseq_start timm:$amt)]>;
def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "!ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;



// immediates definition

def simm16  : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;
def : Pat<(i32 simm16:$val),
          (Loadlinp (LO16 imm:$val))>;

// big immediates
def : Pat<(i32 imm:$val),
          (ORri (Loaduinp (HI16 imm:$val)), (LO16 imm:$val))>;




