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
    BRCOND,
    CMP,
    FPCMP,
    Hi,
    Lo,
    DynAlloc,
    MUL,
    MUL_64,
    JmpLink, //CALL
    CondMov,
    EXB,
    EXH,
    EXBF,
    AddCarry,
    ADDC,
    SubCarry,
    SUBC
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
      LowerCall(TargetLowering::CallLoweringInfo &CLI,
                SmallVectorImpl<SDValue> &InVals) const;

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const;

    typedef SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPassVector;
    bool isLegalICmpImmediate(int64_t Imm) const;
    virtual SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const;

    EVT getOptimalMemOpType(uint64_t Size,
                            unsigned DstAlign, unsigned SrcAlign,
                            bool IsZeroVal,
                            bool MemcpyStrSrc,
                            MachineFunction &MF) const;


private:

    /// ByValArgInfo - Byval argument information.
    struct ByValArgInfo {
      unsigned FirstIdx; // Index of the first register used.
      unsigned NumRegs;  // Number of registers used for this argument.
      unsigned Address;  // Offset of the stack area used to pass this argument.

      ByValArgInfo() : FirstIdx(0), NumRegs(0), Address(0) {}
    };

    /// MipsCC - This class provides methods used to analyze formal and call
    /// arguments and inquire about calling convention information.
    class CoffeeCC {
    public:
      CoffeeCC(CallingConv::ID CallConv, bool IsVarArg,
             CCState &Info);

      void analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Outs);
      void analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Ins);
      void handleByValArg(unsigned ValNo, MVT ValVT, MVT LocVT,
                          CCValAssign::LocInfo LocInfo,
                          ISD::ArgFlagsTy ArgFlags);

      const CCState &getCCInfo() const { return CCInfo; }

      /// hasByValArg - Returns true if function has byval arguments.
      bool hasByValArg() const { return !ByValArgs.empty(); }

      /// useRegsForByval - Returns true if the calling convention allows the
      /// use of registers to pass byval arguments.
      bool useRegsForByval() const { return UseRegsForByval; }

      /// regSize - Size (in number of bits) of integer registers.
      unsigned regSize() const { return RegSize; }

      /// numIntArgRegs - Number of integer registers available for calls.
      unsigned numIntArgRegs() const { return NumIntArgRegs; }

      /// reservedArgArea - The size of the area the caller reserves for
      /// register arguments. This is 16-byte if ABI is O32.
      unsigned reservedArgArea() const { return ReservedArgArea; }

      /// intArgRegs - Pointer to array of integer registers.
      const uint16_t *intArgRegs() const { return IntArgRegs; }

      typedef SmallVector<ByValArgInfo, 2>::const_iterator byval_iterator;
      byval_iterator byval_begin() const { return ByValArgs.begin(); }
      byval_iterator byval_end() const { return ByValArgs.end(); }

    private:
      void allocateRegs(ByValArgInfo &ByVal, unsigned ByValSize,
                        unsigned Align);

      CCState &CCInfo;
      bool UseRegsForByval;
      unsigned RegSize;
      unsigned NumIntArgRegs;
      unsigned ReservedArgArea;
      const uint16_t *IntArgRegs, *ShadowRegs;
      SmallVector<ByValArgInfo, 2> ByValArgs;
      llvm::CCAssignFn *FixedFn, *VarFn;
    };

    SDValue LowerSTORE(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerLOAD(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerADDE(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerADDC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSUBE(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSUBC(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerConstantPool(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerBITCAST(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerADD(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerDYNAMIC_STACKALLOC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerJumpTable(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerBR_CC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSETCC(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerSELECT(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerGlobalAddress(SDValue Op,SelectionDAG &DAG) const;
    SDValue getCoffeeCmp(SDValue LHS, SDValue RHS, SelectionDAG &DAG,
                                 DebugLoc dl) const;
    SDValue LowerRETURNADDR(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerVASTART(SDValue Op, SelectionDAG &DAG) const;

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

    SDValue
    passArgOnStack(SDValue StackPtr, unsigned Offset,
                                       SDValue Chain, SDValue Arg, DebugLoc DL,
                                       bool IsTailCall, SelectionDAG &DAG) const;

    /// copyByValArg - Copy argument registers which were used to pass a byval
    /// argument to the stack. Create a stack frame object for the byval
    /// argument.
    void copyByValRegs(SDValue Chain, DebugLoc DL,
                       std::vector<SDValue> &OutChains, SelectionDAG &DAG,
                       const ISD::ArgFlagsTy &Flags,
                       SmallVectorImpl<SDValue> &InVals,
                       const Argument *FuncArg,
                       const CoffeeCC &CC, const ByValArgInfo &ByVal) const;

    /// passByValArg - Pass a byval argument in registers or on stack.
    void passByValArg(SDValue Chain, DebugLoc DL,
                      SmallVector<std::pair<unsigned, SDValue>, 16> &RegsToPass,
                      SmallVector<SDValue, 8> &MemOpChains, SDValue StackPtr,
                      MachineFrameInfo *MFI, SelectionDAG &DAG, SDValue Arg,
                      const CoffeeCC &CC, const ByValArgInfo &ByVal,
                      const ISD::ArgFlagsTy &Flags, bool isLittle) const;

    /// writeVarArgRegs - Write variable function arguments passed in registers
    /// to the stack. Also create a stack frame object for the first variable
    /// argument.
    void writeVarArgRegs(std::vector<SDValue> &OutChains, const CoffeeCC &CC,
                         SDValue Chain, DebugLoc DL, SelectionDAG &DAG) const;

};
}

#endif   // LLVM_TARGET_Coffee_Coffee32ISELLOWERING_H
