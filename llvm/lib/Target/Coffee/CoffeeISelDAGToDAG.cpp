//===-- CoffeeISelDAGToDAG.cpp - Coffee --pattern matching inst selector --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines a pattern matching instruction selector for Coffee,
// converting from a legalized dag to a Coffee dag.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Coffee-codegen"
#include "Coffee.h"
#include "CoffeeTargetMachine.h"
//#include "MCTargetDesc/CoffeePredicates.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Constants.h"
#include "llvm/Function.h"
#include "llvm/GlobalValue.h"
#include "llvm/Intrinsics.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

namespace {
//===--------------------------------------------------------------------===//
/// CoffeeDAGToDAGISel - Coffee specific code to select Coffee machine
/// instructions for SelectionDAG operations.
///
class CoffeeDAGToDAGISel : public SelectionDAGISel {
    const CoffeeTargetMachine &TM;
    const CoffeeTargetLowering &CoffeeLowering;
    unsigned GlobalBaseReg;

    const CoffeeSubtarget &Subtarget; // this subtarget is used in td files

public:
    explicit CoffeeDAGToDAGISel(CoffeeTargetMachine &tm)
        : SelectionDAGISel(tm), TM(tm),
          CoffeeLowering(*TM.getTargetLowering()),
          Subtarget(tm.getSubtarget<CoffeeSubtarget>()){}
    virtual SDNode *Select(SDNode *N);

    //Complex Pattern selectors
    bool SelectAddr(SDNode *Parent, SDValue N, SDValue &Base, SDValue &Offset);

    SDNode* SelectMULT(SDNode *N, DebugLoc dl);

    /// getI32Imm - Return a target constant of type i32 with the specified
    /// value.
    inline SDValue getI32Imm(unsigned Imm) {
      return CurDAG->getTargetConstant(Imm, MVT::i32);
    }
#include "CoffeeGenDAGISel.inc"
};
}


#include "CoffeeGenRegisterInfo.inc"
bool CoffeeDAGToDAGISel::SelectAddr(SDNode *Parent, SDValue Addr, SDValue &Base, SDValue &Offset) {
    EVT ValTy = Addr.getValueType();

    // If Parent is an unaligned f32 load or store, select a (base + index)
    // floating point load/store instruction (luxc1 or suxc1).
    const LSBaseSDNode* LS = 0;

    if (Parent && (LS = dyn_cast<LSBaseSDNode>(Parent))) {
      EVT VT = LS->getMemoryVT();

      if (VT.getSizeInBits() / 8 > LS->getAlignment()) {
        assert(TLI.allowsUnalignedMemoryAccesses(VT) &&
               "Unaligned loads/stores not supported for this type.");
        if (VT == MVT::f32)
          return false;
      }
    }

    // if Address is FI, get the TargetFrameIndex.
    if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
      Base   = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
      Offset = CurDAG->getTargetConstant(0, ValTy);
      return true;
    }




      if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
          Addr.getOpcode() == ISD::TargetGlobalAddress))
        return false;


    // Addresses of the form FI+const or FI|const
    if (CurDAG->isBaseWithConstantOffset(Addr)) {
      ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
      if (isInt<16>(CN->getSExtValue())) {

        // If the first operand is a FI, get the TargetFI Node
        if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>
                                    (Addr.getOperand(0)))
          Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
        else
          Base = Addr.getOperand(0);

        Offset = CurDAG->getTargetConstant(CN->getZExtValue(), ValTy);
        return true;
      }
    }

    // Operand is a result from an ADD.
    if (Addr.getOpcode() == ISD::ADD) {

        //guoqing: no need to do anything here,
        // it was just a pesudo instr for mips
        // we don't need it in coffee

        //llvm_unreachable("coffee: we need to recheck");
      // When loading from constant pools, load the lower address part in
      // the instruction itself. Example, instead of:
      //  lui $2, %hi($CPI1_0)
      //  addiu $2, $2, %lo($CPI1_0)
      //  lwc1 $f0, 0($2)
      // Generate:
      //  lui $2, %hi($CPI1_0)
      //  lwc1 $f0, %lo($CPI1_0)($2)
     /* if (Addr.getOperand(1).getOpcode() == MipsISD::Lo) {
        SDValue LoVal = Addr.getOperand(1);
        if (isa<ConstantPoolSDNode>(LoVal.getOperand(0)) ||
            isa<GlobalAddressSDNode>(LoVal.getOperand(0))) {
          Base = Addr.getOperand(0);
          Offset = LoVal.getOperand(0);
          return true;
        }
      }

      // If an indexed floating point load/store can be emitted, return false.
      if (LS && (LS->getMemoryVT() == MVT::f32 || LS->getMemoryVT() == MVT::f64) &&
          Subtarget.hasMips32r2Or64())
        return false;*/
    }

    Base   = Addr;
    Offset = CurDAG->getTargetConstant(0, ValTy);
    return true;
  }

SDNode* CoffeeDAGToDAGISel::Select(SDNode *N) {

    DebugLoc dl = N->getDebugLoc();

    if (N->isMachineOpcode())
        return NULL;   // Already selected.

    EVT NodeTy = N->getValueType(0);

    switch (N->getOpcode()) {
    default: break;

    case COFFEEISD::BRCOND: {
        // Pattern: (ARMbrcond:void (bb:Other):$dst, (imm:i32):$cc)
        // Emits: (Bcc:void (bb:Other):$dst, (imm:i32):$cc)
        // Pattern complexity = 6  cost = 1  size = 0

        // Pattern: (ARMbrcond:void (bb:Other):$dst, (imm:i32):$cc)
        // Emits: (tBcc:void (bb:Other):$dst, (imm:i32):$cc)
        // Pattern complexity = 6  cost = 1  size = 0

        // Pattern: (ARMbrcond:void (bb:Other):$dst, (imm:i32):$cc)
        // Emits: (t2Bcc:void (bb:Other):$dst, (imm:i32):$cc)
        // Pattern complexity = 6  cost = 1  size = 0

        SDValue Chain = N->getOperand(0);
        SDValue N1 = N->getOperand(1);
        SDValue N2 = N->getOperand(2);
        SDValue N3 = N->getOperand(3);
        SDValue InFlag = N->getOperand(4);
        assert(N1.getOpcode() == ISD::BasicBlock);
        assert(N2.getOpcode() == ISD::Constant);
        //assert(N3.getOpcode() == ISD::Register);

        ISD::CondCode cc = (ISD::CondCode)cast<ConstantSDNode>(N2)->getZExtValue();

        unsigned Opc = 0;
        switch (cc) {
        default:
            llvm_unreachable("coffee:: unexpected condition code");
            break;
        case ISD::SETEQ:
        case ISD::SETUEQ:
        case ISD::SETOEQ:
            Opc = Coffee::BEQ;
            break;
        case ISD::SETGT:
        case ISD::SETUGT:
        case ISD::SETOGT:
            Opc = Coffee::BGT;
            break;
        case ISD::SETGE:
        case ISD::SETUGE:
        case ISD::SETOGE:
            Opc = Coffee::BEGT;
            break;
        case ISD::SETLT:
        case ISD::SETULT:
        case ISD::SETOLT:
            Opc = Coffee::BLT;
            break;
        case ISD::SETLE:
        case ISD::SETULE:
        case ISD::SETOLE:
            Opc = Coffee::BELT;
            break;
        case ISD::SETNE:
        case ISD::SETUNE:
        case ISD::SETONE:
            Opc = Coffee::BNE;
            break;

        }

        SDValue Ops[] = { N3, N1, Chain, InFlag };
        SDNode *ResNode = CurDAG->getMachineNode(Opc, dl, MVT::Other,
                                                 MVT::Glue, Ops, 4);
        Chain = SDValue(ResNode, 0);
        if (N->getNumValues() == 2) {
            int test = 1;
        }
        ReplaceUses(SDValue(N, 0),
                    SDValue(Chain.getNode(), Chain.getResNo()));
        return NULL;

    }
        break;
        // guoqing: we don't do anything to ISD::MUL as the muls muli instruction
        // will put the lo to destination register automatically

        // guoqing: these are meant for handling hi part
    case ISD::MULHS:
    case ISD::MULHU: {
        if (NodeTy == MVT::i32)
            return SelectMULT(N, dl);
        else
            llvm_unreachable("coffee: nodetype for mulhs is not i32");


    }
        break;
    }

    return SelectCode(N);
}

/// Select multiply instructions.
SDNode*
CoffeeDAGToDAGISel::SelectMULT(SDNode *N, DebugLoc dl) {

    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N->getOperand(1));

    unsigned Opc = 0;
    if (CN) {
    // signed 15 bit imm

        bool test = isInt<15>(CN->getSExtValue());
        bool test1 = isInt<32>(CN->getSExtValue());

        if (isInt<15>(CN->getSExtValue()))
            Opc = Coffee::MULTI;
        else
           Opc = Coffee::MULTR;
    } else {
      // register
        Opc = Coffee::MULTR;
    }

    if (Opc == 0 ) llvm_unreachable("coffee: selectMULT");

  SDNode *Mul = CurDAG->getMachineNode(Opc, dl, MVT::i32, MVT::Glue, N->getOperand(0),
    N->getOperand(1));

  // take the second output which is glue
  SDValue glue = SDValue(Mul, 1);


  /* def MULHI   : InstCoffee<(outs GPRC:$rd), (ins), "mulhi\t$rd", [], IIAlu, FrmJ> {
    }*/
  // this MULHI instruction is meant for telling which register is used to save the upper half of the
  // mulitplication result so no inputs are needed here.
  // but we need to take the glue output from MULTI/MULTR so that it will guarantee the MULHI will appear
  // right after them.

  return CurDAG->getMachineNode(Coffee::MULHI, dl,
                                MVT::i32, glue);


}

/// createCoffeeISelDag - This pass converts a legalized DAG into a
/// Coffee-specific DAG, ready for instruction scheduling.
///
FunctionPass *llvm::createCoffeeISelDag(CoffeeTargetMachine &TM) {
    return new CoffeeDAGToDAGISel(TM);
}

