//===-- CoffeeCLFrameLowering.cpp - CoffeeCL Frame Information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the CoffeeCL implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLFrameLowering.h"
#include "CoffeeCLInstrInfo.h"
#include "CoffeeCLMachineFunctionInfo.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;


#include "CoffeeCLGenInstrInfo.inc"

/// hasFP - Return true if the specified function should have a dedicated frame
/// pointer register.  This is true if the function has variable sized allocas
/// or if frame pointer elimination is disabled.
bool CoffeeCLFrameLowering::hasFP(const MachineFunction &MF) const {
    const MachineFrameInfo *MFI = MF.getFrameInfo();
    return MF.getTarget().Options.DisableFramePointerElim(MF) ||
        MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();

}

static void expandLargeImm(unsigned Reg, int64_t Imm,
                           const CoffeeCLInstrInfo &TII, MachineBasicBlock& MBB,
                           MachineBasicBlock::iterator II, DebugLoc DL) {
 /* unsigned LUi = IsN64 ? Mips::LUi64 : Mips::LUi;
  unsigned ADDu = IsN64 ? Mips::DADDu : Mips::ADDu;
  unsigned ZEROReg = IsN64 ? Mips::ZERO_64 : Mips::ZERO;
  unsigned ATReg = IsN64 ? Mips::AT_64 : Mips::AT;
  MipsAnalyzeImmediate AnalyzeImm;
  const MipsAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, IsN64 ? 64 : 32, false  LastInstrIsADDiu );
  MipsAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, TII.get(LUi), ATReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, TII.get(Inst->Opc), ATReg).addReg(ZEROReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end(); ++Inst)
    BuildMI(MBB, II, DL, TII.get(Inst->Opc), ATReg).addReg(ATReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  BuildMI(MBB, II, DL, TII.get(ADDu), Reg).addReg(Reg).addReg(ATReg);
  */

    llvm_unreachable("coffeecl: we need to support large imm value");
}


void CoffeeCLFrameLowering::emitPrologue(MachineFunction &MF) const {
    MachineBasicBlock &MBB = MF.front();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    const CoffeeCLRegisterInfo *RegInfo =
            static_cast<const CoffeeCLRegisterInfo*>(MF.getTarget().getRegisterInfo());
    const CoffeeCLInstrInfo &TII =
            *static_cast<const CoffeeCLInstrInfo*>(MF.getTarget().getInstrInfo());
    MachineBasicBlock::iterator MBBI = MBB.begin();
    DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();


    // First, compute final stack size.

    uint64_t StackSize = MFI->getStackSize();

    if (StackSize == 0 && !MFI->adjustsStack()) return;


    TII.adjustStackPtr(CoffeeCL::SP, -StackSize, MBB, MBBI);

    // if frame pointer enabled, set it to point to the stack pointers
    if (hasFP(MF)) {
        //move to the last spill instruction
        MachineBasicBlock::iterator I = MBBI;
        for (unsigned i = 0; i < MFI->getCalleeSavedInfo().size(); ++i)
          ++I;

        // we copy SP to FP register, processFunctionBeforeCalleeSavedScan() has add FP as used so
        // it will be spilled on stack before this is called to save the original value in FP
        BuildMI(MBB, I, dl, TII.get(CoffeeCL::Mov), CoffeeCL::FP).addReg(CoffeeCL::SP);
    }


}

void CoffeeCLFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {

    MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    const CoffeeCLInstrInfo &TII =
      *static_cast<const CoffeeCLInstrInfo*>(MF.getTarget().getInstrInfo());
     DebugLoc dl = MBBI->getDebugLoc();


    if (hasFP(MF)) {
        // Find the first instruction that restores a callee-saved register.
        MachineBasicBlock::iterator I = MBBI;

        for (unsigned i = 0; i < MFI->getCalleeSavedInfo().size(); ++i)
          --I;

        // Insert instruction "move $sp, $fp" at this location.
        BuildMI(MBB, I, dl, TII.get(CoffeeCL::Mov), CoffeeCL::SP).addReg(CoffeeCL::FP);
    }


    // Get the number of bytes from FrameInfo
    uint64_t StackSize = MFI->getStackSize();

    if (!StackSize)
      return;

    // restore the SP after function returns
    TII.adjustStackPtr(CoffeeCL::SP, StackSize, MBB, MBBI);
  }

uint64_t CoffeeCLFrameLowering::estimateStackSize(const MachineFunction &MF) const {
    const MachineFrameInfo *MFI = MF.getFrameInfo();
    const TargetRegisterInfo &TRI = *MF.getTarget().getRegisterInfo();

    int64_t Offset = 0;

    // Iterate over fixed sized objects.
    for (int I = MFI->getObjectIndexBegin(); I != 0; ++I)
      Offset = std::max(Offset, -MFI->getObjectOffset(I));

    // Conservatively assume all callee-saved registers will be saved.
    for (const uint16_t *R = TRI.getCalleeSavedRegs(&MF); *R; ++R) {
      unsigned Size = TRI.getMinimalPhysRegClass(*R)->getSize();
      Offset = RoundUpToAlignment(Offset + Size, Size);
    }

    unsigned MaxAlign = MFI->getMaxAlignment();

    // Check that MaxAlign is not zero if there is a stack object that is not a
    // callee-saved spill.
    assert(!MFI->getObjectIndexEnd() || MaxAlign);

    // Iterate over other objects.
    for (unsigned I = 0, E = MFI->getObjectIndexEnd(); I != E; ++I)
      Offset = RoundUpToAlignment(Offset + MFI->getObjectSize(I), MaxAlign);

    // Call frame.
    if (MFI->adjustsStack() && hasReservedCallFrame(MF))
      Offset = RoundUpToAlignment(Offset + MFI->getMaxCallFrameSize(),
                                  std::max(MaxAlign, getStackAlignment()));

    return RoundUpToAlignment(Offset, getStackAlignment());
}

bool
CoffeeCLFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();

  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  return isInt<16>(MFI->getMaxCallFrameSize()) && !MFI->hasVarSizedObjects();
}


void
CoffeeCLFrameLowering::processFunctionBeforeCalleeSavedScan(MachineFunction &MF,
                                                          RegScavenger *RS) const {

    MachineRegisterInfo& MRI = MF.getRegInfo();
    unsigned FP = CoffeeCL::FP;

    // Mark $fp and $ra as used or unused.
    if (hasFP(MF))
      MRI.setPhysRegUsed(FP);  //guoqing: this will spill FP on stack

    // Set scavenging frame index if necessary.
    uint64_t MaxSPOffset = MF.getInfo<CoffeeCLFunctionInfo>()->getIncomingArgSize() +
      estimateStackSize(MF);

    if (isInt<16>(MaxSPOffset))
      return;

    const TargetRegisterClass *RC = &CoffeeCL::GPRCRegClass;
    int FI = MF.getFrameInfo()->CreateStackObject(RC->getSize(),
                                                  RC->getAlignment(), false);
    RS->setScavengingFrameIndex(FI);
  }


bool CoffeeCLFrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const {
    MachineFunction *MF = MBB.getParent();
    MachineBasicBlock *EntryBlock = MF->begin();
    const TargetInstrInfo &TII = *MF->getTarget().getInstrInfo();

    for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
      // Add the callee-saved register as live-in. Do not add if the register is
      // LR and return address is taken, because it has already been added in
      // method CoffeeCLTargetLowering::LowerRETURNADDR.
      // It's killed at the spill, unless the register is LR and return address
      // is taken.
      unsigned Reg = CSI[i].getReg();
      bool IsLRAndRetAddrIsTaken = (Reg == CoffeeCL::LR) && MF->getFrameInfo()->isReturnAddressTaken();
      if (!IsLRAndRetAddrIsTaken)
        EntryBlock->addLiveIn(Reg);

      // Insert the spill to the stack frame.
      bool IsKill = !IsLRAndRetAddrIsTaken;
      const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
      TII.storeRegToStackSlot(*EntryBlock, MI, Reg, IsKill,
                              CSI[i].getFrameIdx(), RC, TRI);
    }

    return true;
}

/*bool CoffeeCLFrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo *MFI = MF.getFrameInfo();


  unsigned NumBytes = MFI->getStackSize();
  CoffeeCLFunctionInfo *AFI = MF.getInfo<CoffeeCLFunctionInfo>();
  bool isVarArg = AFI->getVarArgsRegSaveSize() > 0;
  unsigned NumAlignedDPRCS2Regs = AFI->getNumAlignedDPRCS2Regs();

  // The emitPopInst calls below do not insert reloads for the aligned DPRCS2
  // registers. Do that here instead.
  if (NumAlignedDPRCS2Regs)
      llvm_unreachable("coffeecl: frame lowering, unreachable");
    //emitAlignedDPRCS2Restores(MBB, MI, NumAlignedDPRCS2Regs, CSI, TRI);


  unsigned LdrOpc = CoffeeCL::LOADmrnp;


  emitPopInst(MBB, MI, CSI, LdrOpc, NumBytes,isVarArg, 0);

  return true;
}*/
