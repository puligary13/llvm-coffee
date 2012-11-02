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
#include "MCTargetDesc/CoffeePredicates.h"
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
public:
    explicit CoffeeDAGToDAGISel(CoffeeTargetMachine &tm)
        : SelectionDAGISel(tm), TM(tm),
          CoffeeLowering(*TM.getTargetLowering()) {}
    virtual SDNode *Select(SDNode *N);

    //Complex Pattern selectors
    bool SelectADDRri(SDValue N, SDValue &Base, SDValue &Offset);
    bool SelectADDRrr(SDValue N, SDValue &R1, SDValue &R2);

    /// getI32Imm - Return a target constant of type i32 with the specified
    /// value.
    inline SDValue getI32Imm(unsigned Imm) {
      return CurDAG->getTargetConstant(Imm, MVT::i32);
    }
#include "CoffeeGenDAGISel.inc"
};
}


#include "CoffeeGenRegisterInfo.inc"
bool CoffeeDAGToDAGISel::SelectADDRri(SDValue Addr, SDValue &Base, SDValue &Offset) {
    if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
      Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), MVT::i32);
      Offset = CurDAG->getTargetConstant(0, MVT::i32);
      return true;
    }
    if (Addr.getOpcode() == ISD::TargetExternalSymbol ||
        Addr.getOpcode() == ISD::TargetGlobalAddress)
      return false;  // direct calls.

    if (Addr.getOpcode() == ISD::ADD)
        llvm_unreachable("coffee: select ADDRri");
    Base = Addr;
    Offset = CurDAG->getTargetConstant(0, MVT::i32);
    return true;
  }

bool CoffeeDAGToDAGISel::SelectADDRrr(SDValue Addr, SDValue &R1, SDValue &R2) {
    if (Addr.getOpcode() == ISD::FrameIndex) return false;
    if (Addr.getOpcode() == ISD::TargetExternalSymbol ||
        Addr.getOpcode() == ISD::TargetGlobalAddress)
      return false;  // direct calls.

    if (Addr.getOpcode() == ISD::ADD)
        llvm_unreachable("coffee: select ADDRrr");

    R1 = Addr;
    R2 = CurDAG->getRegister(Coffee::T0, MVT::i32);
    return true;
}

SDNode* CoffeeDAGToDAGISel::Select(SDNode *N) {
    return SelectCode(N);
}
/// createCoffeeISelDag - This pass converts a legalized DAG into a
/// Coffee-specific DAG, ready for instruction scheduling.
///
FunctionPass *llvm::createCoffeeISelDag(CoffeeTargetMachine &TM) {
    return new CoffeeDAGToDAGISel(TM);
}

