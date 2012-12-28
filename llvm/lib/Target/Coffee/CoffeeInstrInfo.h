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
  unsigned UncondBrOpc;

public:
  explicit CoffeeInstrInfo(CoffeeTargetMachine &TM);

  virtual const CoffeeRegisterInfo &getRegisterInfo() const { return RI;}

  virtual void insertNoop(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI) const;

  unsigned GetAnalyzableBrOpc(unsigned Opc) const;

  void AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                    MachineBasicBlock *&BB,
                                    SmallVectorImpl<MachineOperand> &Cond) const;

  void BuildCondBr(MachineBasicBlock &MBB,
                                  MachineBasicBlock *TBB, DebugLoc DL,
                                  const SmallVectorImpl<MachineOperand>& Cond) const;

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

  unsigned GetOppositeBranchOpc(unsigned Opc) const;

  unsigned loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator II, DebugLoc DL,
                                 unsigned *NewImm) const;

  void adjustStackPtr(unsigned SP, int64_t Amount,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator I) const;

  bool expandPostRAPseudo(MachineBasicBlock::iterator MI) const;

  void ExpandRetLR(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  unsigned Opc) const;
};

}

#endif
