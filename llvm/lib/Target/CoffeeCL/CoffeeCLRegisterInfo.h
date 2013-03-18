//===-- CoffeeCLRegisterInfo.h - CoffeeCL Register Information Impl ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the CoffeeCL implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeCL32_REGISTERINFO_H
#define CoffeeCL32_REGISTERINFO_H

#include "CoffeeCL.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "CoffeeCLSubtarget.h"
#include "CoffeeCLRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "CoffeeCLGenRegisterInfo.inc"

namespace llvm {
class CoffeeCLInstrInfo;
class Type;

class CoffeeCLRegisterInfo : public CoffeeCLGenRegisterInfo {
  const CoffeeCLInstrInfo &TII;
  const CoffeeCLSubtarget &Subtarget;

public:
  CoffeeCLRegisterInfo(const CoffeeCLInstrInfo &tii, const CoffeeCLSubtarget &Subtarget);

  /// getRegisterNumbering - Given the enum value for some register, e.g.
  /// CoffeeCL::LR, return the number that it corresponds to (e.g. 31).
  static unsigned getRegisterNumbering(unsigned RegEnum);

  /// Get PIC indirect call register
  static unsigned getPICCallReg();

  /// Adjust the CoffeeCL stack frame.
  void adjustCoffeeCLStackFrame(MachineFunction &MF) const;

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
