//===-- CoffeeRegisterInfo.h - Coffee Register Information Impl ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Coffee implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef Coffee32_REGISTERINFO_H
#define Coffee32_REGISTERINFO_H

#include "Coffee.h"
#include "llvm/Target/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "CoffeeGenRegisterInfo.inc"

namespace llvm {
class TargetInstrInfo;
class Type;

class CoffeeRegisterInfo : public CoffeeGenRegisterInfo {
  const TargetInstrInfo &TII;
  unsigned FramePtr;
  /// BasePtr - ARM physical register used as a base ptr in complex stack
  /// frames. I.e., when we need a 3rd base, not just SP and FP, due to
  /// variable size stack objects.
  unsigned BasePtr;

public:
  CoffeeRegisterInfo(const TargetInstrInfo &tii);

  /// Code Generation virtual methods...
  const uint16_t *getCalleeSavedRegs(const MachineFunction* MF = 0) const;

  const uint32_t* getCallPreservedMask(CallingConv::ID) const;

  BitVector getReservedRegs(const MachineFunction &MF) const;

  void eliminateCallFramePseudoInstr(MachineFunction &MF,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const;

  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, RegScavenger *RS = NULL) const;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF) const;

  unsigned getFrameRegister(const MachineFunction &MF) const;

  bool canRealignStack(const MachineFunction &MF) const;

  bool cannotEliminateFrame(const MachineFunction &MF) const;

  bool hasBasePointer(const MachineFunction &MF) const;

  unsigned getBaseRegister() const { return BasePtr; }
};


} // end namespace llvm

#endif
