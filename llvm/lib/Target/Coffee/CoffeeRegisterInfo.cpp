//===-- CoffeeRegisterInfo.cpp - Coffee Register Information ----------------===//
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

#define DEBUG_TYPE "reginfo"
#include "CoffeeRegisterInfo.h"
#include "Coffee.h"
#include "CoffeeInstrBuilder.h"
#include "CoffeeInstrInfo.h"
#include "CoffeeMachineFunctionInfo.h"
#include "CoffeeFrameLowering.h"
#include "CoffeeSubtarget.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/Function.h"
#include "llvm/Type.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include <cstdlib>

#define GET_REGINFO_TARGET_DESC
#include "CoffeeGenRegisterInfo.inc"


using namespace llvm;


CoffeeRegisterInfo::CoffeeRegisterInfo(const CoffeeInstrInfo &tii, const CoffeeSubtarget &ST)
    : CoffeeGenRegisterInfo(Coffee::LR), Subtarget(ST), TII(tii)
     {}

unsigned CoffeeRegisterInfo::getPICCallReg() { return Coffee::T9; }

//Coffee callee saved registers
const uint16_t*
CoffeeRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_COFFEE_SaveList;
}

const uint32_t*
CoffeeRegisterInfo::getCallPreservedMask(CallingConv::ID) const {
  return CSR_COFFEE_RegMask;
}


BitVector CoffeeRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
    static const uint16_t ReservedCPURegs[] = {
      Coffee::SP
    };

    BitVector Reserved(getNumRegs());
    typedef TargetRegisterClass::const_iterator RegIter;

    for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)
      Reserved.set(ReservedCPURegs[I]);

    // Reserve FP if this function should have a dedicated frame pointer register.
    if (MF.getTarget().getFrameLowering()->hasFP(MF)) {
        Reserved.set(Coffee::FP);
    }

    // Reserve GP if small section is used.
    if (Subtarget.useSmallSection()) {
      Reserved.set(Coffee::GP);
    }

    return Reserved;
  }

bool
CoffeeRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool CoffeeRegisterInfo::
requiresFrameIndexScavenging(const MachineFunction &MF) const {
  return true;
}

bool
CoffeeRegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

void CoffeeRegisterInfo::eliminateCallFramePseudoInstr(MachineFunction &MF,
                                                       MachineBasicBlock &MBB,
                                                       MachineBasicBlock::iterator I) const {
    const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

    if (!TFI->hasReservedCallFrame(MF)) {
      int64_t Amount = I->getOperand(0).getImm();

      if (I->getOpcode() == Coffee::ADJCALLSTACKDOWN)
        Amount = -Amount;

      const CoffeeInstrInfo *II = static_cast<const CoffeeInstrInfo*>(&TII);
      unsigned SP = Coffee::SP;

      II->adjustStackPtr(SP, Amount, MBB, I);
    }

    MBB.erase(I);
}

void CoffeeRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                             int SPAdj, RegScavenger *RS) const {
    MachineInstr &MI = *II;
    MachineFunction &MF = *MI.getParent()->getParent();

    unsigned i = 0;
    while (!MI.getOperand(i).isFI()) {
      ++i;
      assert(i < MI.getNumOperands() &&
             "Instr doesn't have FrameIndex operand!");
    }

    DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
          errs() << "<--------->\n" << MI);

    int FrameIndex = MI.getOperand(i).getIndex();
    uint64_t stackSize = MF.getFrameInfo()->getStackSize();
    int64_t spOffset = MF.getFrameInfo()->getObjectOffset(FrameIndex);

    DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
                 << "spOffset   : " << spOffset << "\n"
                 << "stackSize  : " << stackSize << "\n");

    eliminateFI(MI, i, FrameIndex, stackSize, spOffset);
  }

void CoffeeRegisterInfo::eliminateFI(MachineBasicBlock::iterator II,
                                     unsigned OpNo, int FrameIndex,
                                     uint64_t StackSize,
                                     int64_t SPOffset) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo *MFI = MF.getFrameInfo();

  const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  // The following stack frame objects are always referenced relative to $sp:
  //  1. Outgoing arguments.
  //  2. Pointer to dynamically allocated stack space.
  //  3. Locations for callee-saved registers.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;

  if (FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI)
    FrameReg = Coffee::SP;
  else
    FrameReg = getFrameRegister(MF);

  // Calculate final offset.
  // - There is no need to change the offset if the frame object is one of the
  //   following: an outgoing argument, pointer to a dynamically allocated
  //   stack space or a $gp restore location,
  // - If the frame object is any of the following, its offset must be adjusted
  //   by adding the size of the stack:
  //   incoming argument, callee-saved register location or local variable.
  bool IsKill = false;
  int64_t Offset;

  Offset = SPOffset + (int64_t)StackSize;
  Offset += MI.getOperand(OpNo + 1).getImm();

  DEBUG(errs() << "Offset     : " << Offset << "\n" << "<--------->\n");

  // If MI is not a debug value, make sure Offset fits in the 16-bit immediate
  // field.
  if (!MI.isDebugValue() && !isInt<16>(Offset)) {
    MachineBasicBlock &MBB = *MI.getParent();
    DebugLoc DL = II->getDebugLoc();
    unsigned NewImm;

    // if the offset is too big (SP + offset), we change it to (RE + NewImm) where RE2 = SP + RE1

    unsigned Reg = TII.loadImmediate(Offset, MBB, II, DL, &NewImm);

    BuildMI(MBB, II, DL, TII.get(Coffee::ADDu), Reg).addReg(FrameReg)
      .addReg(Reg, RegState::Kill);

    FrameReg = Reg;
    Offset = SignExtend32<16>(NewImm);
    IsKill = true;
  }

  MI.getOperand(OpNo).ChangeToRegister(FrameReg, false, false, IsKill);
  MI.getOperand(OpNo + 1).ChangeToImmediate(Offset);
}


unsigned CoffeeRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
    //we have dedicated frame register
    const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

    return TFI->hasFP(MF) ? Coffee::FP : Coffee::SP;
}


unsigned CoffeeRegisterInfo::
getEHExceptionRegister() const {
  llvm_unreachable("What is the exception register");
}

unsigned CoffeeRegisterInfo::
getEHHandlerRegister() const {
  llvm_unreachable("What is the exception handler register");
}
