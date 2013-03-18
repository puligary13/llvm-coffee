//===-- CoffeeCLISelDAGToDAG.cpp - CoffeeCL --pattern matching inst selector --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines a pattern matching instruction selector for CoffeeCL,
// converting from a legalized dag to a CoffeeCL dag.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "CoffeeCL-codegen"
#include "CoffeeCL.h"
#include "CoffeeCLTargetMachine.h"
//#include "MCTargetDesc/CoffeeCLPredicates.h"
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
/// CoffeeCLDAGToDAGISel - CoffeeCL specific code to select CoffeeCL machine
/// instructions for SelectionDAG operations.
///
class CoffeeCLDAGToDAGISel : public SelectionDAGISel {
    const CoffeeCLTargetMachine &TM;
    const CoffeeCLTargetLowering &CoffeeCLLowering;
    unsigned GlobalBaseReg;

    const CoffeeCLSubtarget &Subtarget; // this subtarget is used in td files

public:
    explicit CoffeeCLDAGToDAGISel(CoffeeCLTargetMachine &tm)
        : SelectionDAGISel(tm), TM(tm),
          CoffeeCLLowering(*TM.getTargetLowering()),
          Subtarget(tm.getSubtarget<CoffeeCLSubtarget>()){}
    virtual SDNode *Select(SDNode *N);

    //Complex Pattern selectors
    bool SelectAddr(SDNode *Parent, SDValue N, SDValue &Base, SDValue &Offset);

    SDNode* SelectCMOVOp(SDNode *N);

    SDNode *SelectBRCONDOp(SDNode *N);

    SDNode* SelectMULT(SDNode *N, DebugLoc dl);

    /// getI32Imm - Return a target constant of type i32 with the specified
    /// value.
    inline SDValue getI32Imm(unsigned Imm) {
      return CurDAG->getTargetConstant(Imm, MVT::i32);
    }
#include "CoffeeCLGenDAGISel.inc"
};
}


#include "CoffeeCLGenRegisterInfo.inc"
bool CoffeeCLDAGToDAGISel::SelectAddr(SDNode *Parent, SDValue Addr, SDValue &Base, SDValue &Offset) {
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
        // we don't need it in coffeecl

        //llvm_unreachable("coffeecl: we need to recheck");
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

SDNode* CoffeeCLDAGToDAGISel::Select(SDNode *N) {

    DebugLoc dl = N->getDebugLoc();

    if (N->isMachineOpcode())
        return NULL;   // Already selected.

    EVT NodeTy = N->getValueType(0);

    switch (N->getOpcode()) {
    default: break;

    case COFFEEISD::BRCOND:
        return SelectBRCONDOp(N);

    case COFFEEISD::CondMov:
         return SelectCMOVOp(N);

    case ISD::MULHS:
    case ISD::MULHU: {
        if (NodeTy == MVT::i32)
            return SelectMULT(N, dl);
        else
            llvm_unreachable("coffeecl: nodetype for mulhs is not i32");
    }
        break;
    }

    return SelectCode(N);
}


SDNode *CoffeeCLDAGToDAGISel::SelectBRCONDOp(SDNode *N) {

    SDValue Chain = N->getOperand(0); // Chain
    SDValue N1 = N->getOperand(1); // Destination
    SDValue N2 = N->getOperand(2); // CondCode as constant
    SDValue N3 = N->getOperand(3); // CC regsiter to check --> CR0
    SDValue InFlag = N->getOperand(4); // glue
    assert(N1.getOpcode() == ISD::BasicBlock);
    assert(N2.getOpcode() == ISD::Constant);
    assert(N3.getOpcode() == ISD::Register);

    DebugLoc dl = N->getDebugLoc();

    ISD::CondCode cc = (ISD::CondCode)cast<ConstantSDNode>(N2)->getZExtValue();

    // The main logic here follows ARM implementation but unlike ARM,
    // CoffeeCL has dedicated branch instructions for each situations while
    // ARM has one instruction which take CondCode as operands.
    unsigned Opc = 0;
    switch (cc) {
    default:
        llvm_unreachable("coffeecl:: unexpected condition code");
        break;
    case ISD::SETEQ:
    case ISD::SETUEQ:
    case ISD::SETOEQ:
        Opc = CoffeeCL::BEQ;
        break;
    case ISD::SETGT:
    case ISD::SETUGT:
    case ISD::SETOGT:
        Opc = CoffeeCL::BGT;
        break;
    case ISD::SETGE:
    case ISD::SETUGE:
    case ISD::SETOGE:
        Opc = CoffeeCL::BEGT;
        break;
    case ISD::SETLT:
    case ISD::SETULT:
    case ISD::SETOLT:
        Opc = CoffeeCL::BLT;
        break;
    case ISD::SETLE:
    case ISD::SETULE:
    case ISD::SETOLE:
        Opc = CoffeeCL::BELT;
        break;
    case ISD::SETNE:
    case ISD::SETUNE:
    case ISD::SETONE:
        Opc = CoffeeCL::BNE;
        break;
    }

    SDValue Ops[] = { N3, N1, Chain, InFlag };

    // TODO: do we need output glue here ?
    SDNode *ResNode = CurDAG->getMachineNode(Opc, dl, MVT::Other,
                                             MVT::Glue, Ops, 4);

    /***NOTE**/
    // Is replaceUses function meant for Chain Node ?

    Chain = SDValue(ResNode, 0);
    ReplaceUses(SDValue(N, 0),
                SDValue(Chain.getNode(), Chain.getResNo()));
    return NULL;

}


SDNode *CoffeeCLDAGToDAGISel::SelectCMOVOp(SDNode *N) {
    EVT VT = N->getValueType(0);
    SDValue FalseVal = N->getOperand(0);
    SDValue TrueVal  = N->getOperand(1);
    SDValue CC = N->getOperand(2);     // CondCode as constant
    SDValue CCR = N->getOperand(3);    // CR0 register
    SDValue InFlag = N->getOperand(4); //glue

    DebugLoc dl = N->getDebugLoc();

    assert(CC.getOpcode() == ISD::Constant);
    assert(CCR.getOpcode() == ISD::Register);

    ISD::CondCode CCVal =
      (ISD::CondCode)cast<ConstantSDNode>(CC)->getZExtValue();

    unsigned Opc = 0;
    switch (CCVal) {
    default:
        llvm_unreachable("coffeecl:: unexpected condition code");
        break;
    case ISD::SETEQ:
    case ISD::SETUEQ:
    case ISD::SETOEQ:
        Opc = (VT == MVT::i32) ? CoffeeCL::CMov_EQ : CoffeeCL::FPCMov_EQ;
        break;
    case ISD::SETGT:
    case ISD::SETUGT:
    case ISD::SETOGT:
       Opc = (VT == MVT::i32) ? CoffeeCL::CMov_GT : CoffeeCL::FPCMov_GT;
        break;
    case ISD::SETGE:
    case ISD::SETUGE:
    case ISD::SETOGE:
        Opc = (VT == MVT::i32) ? CoffeeCL::CMov_EGT : CoffeeCL::FPCMov_EGT;
        break;
    case ISD::SETLT:
    case ISD::SETULT:
    case ISD::SETOLT:
        Opc = (VT == MVT::i32) ? CoffeeCL::CMov_LT : CoffeeCL::FPCMov_LT;
        break;
    case ISD::SETLE:
    case ISD::SETULE:
    case ISD::SETOLE:
        Opc = (VT == MVT::i32) ? CoffeeCL::CMov_ELT : CoffeeCL::FPCMov_ELT;
        break;
    case ISD::SETNE:
    case ISD::SETUNE:
    case ISD::SETONE:
        Opc = (VT == MVT::i32) ? CoffeeCL::CMov_NE : CoffeeCL::FPCMov_NE;
        break;
    }

    /****NOTE***/
    // Unlike BRCOND, we don't have chain here. This is according to
    // the definition of ISD::SELECT_CC that we customized to
    // COFFEEISD::CondMov

    SDValue Ops[] = { TrueVal, FalseVal, CCR, InFlag };
    return CurDAG->SelectNodeTo(N, Opc, (VT == MVT::i32) ? MVT::i32 : MVT::f32, Ops, 4);

  }

/// Select multiply instructions.
SDNode*
CoffeeCLDAGToDAGISel::SelectMULT(SDNode *N, DebugLoc dl) {

    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N->getOperand(1));

    unsigned Opc = 0;
    if (CN) {
    // signed 15 bit imm

        bool test = isInt<15>(CN->getSExtValue());
        bool test1 = isInt<32>(CN->getSExtValue());

        if (isInt<15>(CN->getSExtValue()))
            Opc = CoffeeCL::MULTI;
        else
           Opc = CoffeeCL::MULTR;
    } else {
      // register
        Opc = CoffeeCL::MULTR;
    }

    if (Opc == 0 ) llvm_unreachable("coffeecl: selectMULT");

  SDNode *Mul = CurDAG->getMachineNode(Opc, dl, MVT::i32, MVT::Glue, N->getOperand(0),
    N->getOperand(1));

  // take the second output which is glue
  SDValue glue = SDValue(Mul, 1);


  /* def MULHI   : InstCoffeeCL<(outs GPRC:$rd), (ins), "mulhi\t$rd", [], IIAlu, FrmJ> {
    }*/
  // this MULHI instruction is meant for telling which register is used to save the upper half of the
  // mulitplication result so no inputs are needed here.
  // but we need to take the glue output from MULTI/MULTR so that it will guarantee the MULHI will appear
  // right after them.

  return CurDAG->getMachineNode(CoffeeCL::MULHI, dl,
                                MVT::i32, glue);


}

/// createCoffeeCLISelDag - This pass converts a legalized DAG into a
/// CoffeeCL-specific DAG, ready for instruction scheduling.
///
FunctionPass *llvm::createCoffeeCLISelDag(CoffeeCLTargetMachine &TM) {
    return new CoffeeCLDAGToDAGISel(TM);
}

