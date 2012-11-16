//===-- CoffeeISelLowering.h - Coffee32 DAG Lowering Interface --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Coffee uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_Coffee_Coffee32ISELLOWERING_H
#define LLVM_TARGET_Coffee_Coffee32ISELLOWERING_H

#include "Coffee.h"

#include "llvm/Target/TargetLowering.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/CallingConvLower.h"

namespace llvm {
  /// Define some predicates that are used for node matching.


namespace COFFEEISD {
// ARM Specific DAG Nodes
enum NodeType {
    // Start the numbering where the builtin ops and target ops leave off.
    FIRST_NUMBER = ISD::BUILTIN_OP_END,
    RET,     // Return with a flag operand
    CALL,
    BRCOND,
    CMP,
    Hi,
    Lo,
    DynAlloc,
    MUL,
    MUL_64
};
}


class CoffeeTargetLowering : public TargetLowering {

public:
    explicit CoffeeTargetLowering(CoffeeTargetMachine &TM);

    virtual SDValue
    LowerFormalArguments(SDValue Chain,
                         CallingConv::ID CallConv,
                         bool isVarArg,
                         const SmallVectorImpl<ISD::InputArg> &Ins,
                         DebugLoc dl, SelectionDAG &DAG,
                         SmallVectorImpl<SDValue> &InVals) const;

    virtual SDValue
    LowerReturn(SDValue Chain,
                CallingConv::ID CallConv, bool isVarArg,
                const SmallVectorImpl<ISD::OutputArg> &Outs,
                const SmallVectorImpl<SDValue> &OutVals,
                DebugLoc dl, SelectionDAG &DAG) const;

    virtual const char *getTargetNodeName(unsigned Opcode) const;

    virtual SDValue
    LowerCall(SDValue Chain, SDValue Callee,
              CallingConv::ID CallConv, bool isVarArg,
              bool doesNotRet, bool &isTailCall,
              const SmallVectorImpl<ISD::OutputArg> &Outs,
              const SmallVectorImpl<SDValue> &OutVals,
              const SmallVectorImpl<ISD::InputArg> &Ins,
              DebugLoc dl, SelectionDAG &DAG,
              SmallVectorImpl<SDValue> &InVals) const;

    CCAssignFn *CCAssignFnForNode(CallingConv::ID CC,
                                  bool Return,
                                  bool isVarArg) const;

    SDValue LowerMemOpCallTo(SDValue Chain,
                     SDValue StackPtr, SDValue Arg,
                     DebugLoc dl, SelectionDAG &DAG,
                     const CCValAssign &VA,
                     ISD::ArgFlagsTy Flags) const;


    SDValue
    LowerCallResult(SDValue Chain, SDValue InFlag,
                                       CallingConv::ID CallConv, bool isVarArg,
                                       const SmallVectorImpl<ISD::InputArg> &Ins,
                                       DebugLoc dl, SelectionDAG &DAG,
                                       SmallVectorImpl<SDValue> &InVals) const;

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerBR_CC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerGlobalAddress(SDValue Op,SelectionDAG &DAG) const;

    typedef SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPassVector;

    SDValue getCoffeeCmp(SDValue LHS, SDValue RHS, SelectionDAG &DAG,
                                 DebugLoc dl) const;

    bool isLegalICmpImmediate(int64_t Imm) const;

    virtual SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const;

    SDValue LowerDYNAMIC_STACKALLOC(SDValue Op, SelectionDAG &DAG) const;



    SDValue LowerJumpTable(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;
};
}

#endif   // LLVM_TARGET_Coffee_Coffee32ISELLOWERING_H
