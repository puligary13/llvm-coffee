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
#include "CoffeeSubtarget.h"
#include "CoffeeRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "CoffeeGenRegisterInfo.inc"

namespace llvm {
class CoffeeInstrInfo;
class Type;

class CoffeeRegisterInfo : public CoffeeGenRegisterInfo {
  const CoffeeInstrInfo &TII;
  const CoffeeSubtarget &Subtarget;

public:
  CoffeeRegisterInfo(const CoffeeInstrInfo &tii, const CoffeeSubtarget &Subtarget);

  /// getRegisterNumbering - Given the enum value for some register, e.g.
  /// Coffee::LR, return the number that it corresponds to (e.g. 31).
  static unsigned getRegisterNumbering(unsigned RegEnum);

  /// Get PIC indirect call register
  static unsigned getPICCallReg();

  /// Adjust the Coffee stack frame.
  void adjustCoffeeStackFrame(MachineFunction &MF) const;

  /// Code Generation virtual methods...
  const uint16_t *getCalleeSavedRegs(const MachineFunction *MF = 0) const;
  const uint32_t *getCallPreservedMask(CallingConv::ID) const;

  BitVector getReservedRegs(const MachineFunction &MF) const;

  const TargetRegisterClass *getCrossCopyRegClass(const TargetRegisterClass *RC) const;

  virtual bool requiresRegisterScavenging(const MachineFunction &MF) const;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const;

  virtual bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const;

  /// Stack Frame Processing Methods
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, RegScavenger *RS = NULL) const;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF) const;

  /// Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const;

  /// Exception handling queries.
  unsigned getEHExceptionRegister() const;
  unsigned getEHHandlerRegister() const;

  void eliminateCallFramePseudoInstr(MachineFunction &MF,
                                                         MachineBasicBlock &MBB,
                                                         MachineBasicBlock::iterator I) const;

private:
  void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                           int FrameIndex, uint64_t StackSize,
                           int64_t SPOffset) const;
};


} // end namespace llvm

#endif
