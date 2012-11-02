//===-- CoffeeInstrInfo.h - Coffee Instruction Information --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Coffee implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef Coffee_INSTRUCTIONINFO_H
#define Coffee_INSTRUCTIONINFO_H

#include "Coffee.h"
#include "CoffeeRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "CoffeeGenInstrInfo.inc"

namespace llvm {

class CoffeeInstrInfo : public CoffeeGenInstrInfo {
  CoffeeTargetMachine &TM;
  const CoffeeRegisterInfo RI;

public:
  explicit CoffeeInstrInfo(CoffeeTargetMachine &TM);

  virtual const CoffeeRegisterInfo &getRegisterInfo() const { return RI;}

  virtual MachineInstr *commuteInstruction(MachineInstr *MI, bool NewMI) const;

  virtual void insertNoop(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI) const;


  virtual bool AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                             MachineBasicBlock *&FBB,
                             SmallVectorImpl<MachineOperand> &Cond,
                             bool AllowModify) const;
  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  virtual void copyPhysReg(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator I, DebugLoc DL,
                           unsigned DestReg, unsigned SrcReg,
                           bool KillSrc) const;

  virtual void storeRegToStackSlot(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MBBI,
                                   unsigned SrcReg, bool isKill, int FrameIndex,
                                   const TargetRegisterClass *RC,
                                   const TargetRegisterInfo *TRI) const;

  virtual void loadRegFromStackSlot(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MBBI,
                                    unsigned DestReg, int FrameIndex,
                                    const TargetRegisterClass *RC,
                                    const TargetRegisterInfo *TRI) const;

  virtual MachineInstr *emitFrameIndexDebugValue(MachineFunction &MF,
                                                 int FrameIx,
                                                 uint64_t Offset,
                                                 const MDNode *MDPtr,
                                                 DebugLoc DL) const;

  virtual
  bool ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const;

  virtual unsigned GetInstSizeInBytes(const MachineInstr *MI) const;
};


void emitCoffeeRegPlusImmediate(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator &MBBI, DebugLoc dl,
                             unsigned DestReg, unsigned BaseReg, int NumBytes,
                             const TargetInstrInfo &TII, unsigned MIFlags = 0);

}

#endif
