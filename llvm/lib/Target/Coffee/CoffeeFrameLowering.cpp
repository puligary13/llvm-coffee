//===-- CoffeeFrameLowering.cpp - Coffee Frame Information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Coffee implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "CoffeeFrameLowering.h"
#include "CoffeeInstrInfo.h"
#include "CoffeeMachineFunctionInfo.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;


#include "CoffeeGenInstrInfo.inc"

/// hasFP - Return true if the specified function should have a dedicated frame
/// pointer register.  This is true if the function has variable sized allocas
/// or if frame pointer elimination is disabled.
bool CoffeeFrameLowering::hasFP(const MachineFunction &MF) const {
    // guoqing: we should need stack realignment which needs FP also. Ignore it for now.

    const MachineFrameInfo *MFI = MF.getFrameInfo();
    return (MF.getTarget().Options.DisableFramePointerElim(MF) &&
             MFI->hasCalls()) || MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();

}

static void
emitSPUpdate(MachineBasicBlock &MBB, MachineBasicBlock::iterator &MBBI,
             DebugLoc dl, const TargetInstrInfo &TII,
             int NumBytes, unsigned MIFlags = MachineInstr::NoFlags) {
    emitCoffeeRegPlusImmediate(MBB, MBBI, dl, Coffee::SP, Coffee::SP, NumBytes,
                               TII, MIFlags);
}


static void expandLargeImm(unsigned Reg, int64_t Imm,
                           const CoffeeInstrInfo &TII, MachineBasicBlock& MBB,
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

    llvm_unreachable("coffee: we need to support large imm value");
}


void CoffeeFrameLowering::emitPrologue(MachineFunction &MF) const {
    MachineBasicBlock &MBB = MF.front();
    MachineBasicBlock::iterator MBBI = MBB.begin();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();
    const CoffeeRegisterInfo *RegInfo =
            static_cast<const CoffeeRegisterInfo*>(MF.getTarget().getRegisterInfo());
    const CoffeeInstrInfo &TII =
            *static_cast<const CoffeeInstrInfo*>(MF.getTarget().getInstrInfo());


    //guoqing: currently we assume we don't need PIC support,
    // if needed, we need to reimplement this function to arrange the stack
     bool isPIC = (MF.getTarget().getRelocationModel() == Reloc::PIC_);
     if (isPIC)
         llvm_unreachable("coffee: emitPrologue, PIC");

    // First, compute final stack size.
    unsigned RegSize = 4;
    unsigned StackAlign = getStackAlignment();
    unsigned LocalVarAreaOffset = CoffeeFI->getMaxCallFrameSize();
    uint64_t StackSize =  RoundUpToAlignment(LocalVarAreaOffset, StackAlign) +
       RoundUpToAlignment(MFI->getStackSize(), StackAlign);

    // Update stack size after round up to alignment
    MFI->setStackSize(StackSize);


    const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();
    DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
    unsigned FramePtr = RegInfo->getFrameRegister(MF);


    if (StackSize == 0 && !MFI->adjustsStack()) return;


    // guoqing: Mips checks the stacksize and treat large imm differetly
    // in coffee, the large imm should be handled in td file, no need do anything

    BuildMI(MBB, MBBI, dl, TII.get(Coffee::ADDri), Coffee::SP).addReg(Coffee::SP).addImm(-StackSize);

    // if frame pointer enabled, set it to point to the stack pointers
    if (hasFP(MF)) {
        // we copy SP to FP register, processFunctionBeforeCalleeSavedScan() has add FP as used so
        // it will be spilled on stack before this is called to save the original value in FP
        BuildMI(MBB, MBBI, dl, TII.get(Coffee::ADDri), Coffee::FP).addReg(Coffee::SP).addImm(0);
    }


}

void CoffeeFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {

    MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
    assert(MBBI->isReturn() && "Can only insert epilog into returning blocks");

    DebugLoc dl = MBBI->getDebugLoc();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();
    const TargetRegisterInfo *RegInfo = MF.getTarget().getRegisterInfo();
    const CoffeeInstrInfo &TII =
      *static_cast<const CoffeeInstrInfo*>(MF.getTarget().getInstrInfo());
    const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();

    if (hasFP(MF)) {
        // Find the first instruction that restores a callee-saved register.
        MachineBasicBlock::iterator I = MBBI;

        for (unsigned i = 0; i < MFI->getCalleeSavedInfo().size(); ++i)
          --I;

        // Insert instruction "move $sp, $fp" at this location.
        BuildMI(MBB, I, dl, TII.get(Coffee::ADDri), Coffee::SP).addReg(Coffee::FP).addImm(0);
    }


    int StackSize = (int)MFI->getStackSize();


    if (StackSize == 0) return;

    // restore the SP after function returns
    BuildMI(MBB, MBBI, dl, TII.get(Coffee::ADDri), Coffee::SP).addReg(Coffee::SP).addImm(StackSize);
  }

static unsigned estimateStackSize(MachineFunction &MF) {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();
  const TargetRegisterInfo *RegInfo = MF.getTarget().getRegisterInfo();
  unsigned MaxAlign = MFI->getMaxAlignment();
  int Offset = 0;

  // This code is very, very similar to PEI::calculateFrameObjectOffsets().
  // It really should be refactored to share code. Until then, changes
  // should keep in mind that there's tight coupling between the two.

  for (int i = MFI->getObjectIndexBegin(); i != 0; ++i) {
    int FixedOff = -MFI->getObjectOffset(i);
    if (FixedOff > Offset) Offset = FixedOff;
  }
  for (unsigned i = 0, e = MFI->getObjectIndexEnd(); i != e; ++i) {
    if (MFI->isDeadObjectIndex(i))
      continue;
    Offset += MFI->getObjectSize(i);
    unsigned Align = MFI->getObjectAlignment(i);
    // Adjust to alignment boundary
    Offset = (Offset+Align-1)/Align*Align;

    MaxAlign = std::max(Align, MaxAlign);
  }

  if (MFI->adjustsStack() && TFI->hasReservedCallFrame(MF))
    Offset += MFI->getMaxCallFrameSize();

  // Round up the size to a multiple of the alignment.  If the function has
  // any calls or alloca's, align to the target's StackAlignment value to
  // ensure that the callee's frame or the alloca data is suitably aligned;
  // otherwise, for leaf functions, align to the TransientStackAlignment
  // value.
  unsigned StackAlign;
  if (MFI->adjustsStack() || MFI->hasVarSizedObjects() ||
      (MFI->getObjectIndexEnd() != 0))
    StackAlign = TFI->getStackAlignment();
  else
    StackAlign = TFI->getTransientStackAlignment();

  // If the frame pointer is eliminated, all frame offsets will be relative to
  // SP not FP. Align to MaxAlign so this works.
  StackAlign = std::max(StackAlign, MaxAlign);
  unsigned AlignMask = StackAlign - 1;
  Offset = (Offset + AlignMask) & ~uint64_t(AlignMask);

  return (unsigned)Offset;
}

/// estimateRSStackSizeLimit - Look at each instruction that references stack
/// frames and return the stack size limit beyond which some of these
/// instructions will require a scratch register during their expansion later.
// FIXME: Move to TII?
static unsigned estimateRSStackSizeLimit(MachineFunction &MF,
                                         const TargetFrameLowering *TFI) {
  const CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
  unsigned Limit = (1 << 12) - 1;
  for (MachineFunction::iterator BB = MF.begin(),E = MF.end(); BB != E; ++BB) {
    for (MachineBasicBlock::iterator I = BB->begin(), E = BB->end();
         I != E; ++I) {
      for (unsigned i = 0, e = I->getNumOperands(); i != e; ++i) {
        if (!I->getOperand(i).isFI()) continue;

        // When using ADDri to get the address of a stack object, 2^15 is the
        // largest offset guaranteed to fit in the immediate offset.
        if (I->getOpcode() == Coffee::ADDri) {
          Limit = std::min(Limit, (1U << 15) - 1);
          break;
        }

        // Otherwise check the addressing mode.
       /* switch (I->getDesc().TSFlags & ARMII::AddrModeMask) {
        case ARMII::AddrMode3:
        case ARMII::AddrModeT2_i8:
          Limit = std::min(Limit, (1U << 8) - 1);
          break;
        case ARMII::AddrMode5:
        case ARMII::AddrModeT2_i8s4:
          Limit = std::min(Limit, ((1U << 8) - 1) * 4);
          break;
        case ARMII::AddrModeT2_i12:
          // i12 supports only positive offset so these will be converted to
          // i8 opcodes. See llvm::rewriteT2FrameIndex.
          if (TFI->hasFP(MF) && AFI->hasStackFrame())
            Limit = std::min(Limit, (1U << 8) - 1);
          break;
        case ARMII::AddrMode4:
        case ARMII::AddrMode6:
          // Addressing modes 4 & 6 (load/store) instructions can't encode an
          // immediate offset for stack references.
          return 0;
        default:
          break;
        }*/
        break; // At most one FI per instruction
      }
    }
  }

  return Limit;
}



void
CoffeeFrameLowering::processFunctionBeforeCalleeSavedScan(MachineFunction &MF,
                                                          RegScavenger *RS) const {

    MachineRegisterInfo& MRI = MF.getRegInfo();
    unsigned FP = Coffee::FP;

    // FIXME: remove this code if register allocator can correctly mark
    //        $fp and $ra used or unused.

    // Mark $fp and $ra as used or unused.
    if (hasFP(MF))
      MRI.setPhysRegUsed(FP);  //guoqing: this will spill FP on stack

    // The register allocator might determine $ra is used after seeing
    // instruction "jr $ra", but we do not want PrologEpilogInserter to insert
    // instructions to save/restore $ra unless there is a function call.
    // To correct this, $ra is explicitly marked unused if there is no
    // function call.
    if (MF.getFrameInfo()->hasCalls())
      MRI.setPhysRegUsed(Coffee::LR);
    else {
      MRI.setPhysRegUnused(Coffee::LR);
    }

  }

void CoffeeFrameLowering::processFunctionBeforeFrameFinalized(MachineFunction &MF)
const {
}



int CoffeeFrameLowering::ResolveFrameIndexReference(const MachineFunction &MF,
                                             int FI, unsigned &FrameReg,
                                             int SPAdj) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  const CoffeeRegisterInfo *RegInfo =
    static_cast<const CoffeeRegisterInfo*>(MF.getTarget().getRegisterInfo());
  const CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
  int Offset = MFI->getObjectOffset(FI) + MFI->getStackSize();
  int FPOffset = Offset - AFI->getFramePtrSpillOffset();
  bool isFixed = MFI->isFixedObjectIndex(FI);

  FrameReg = Coffee::SP;
  Offset += SPAdj;
  if (AFI->isGPRCalleeSavedArea1Frame(FI))
    return Offset - AFI->getGPRCalleeSavedArea1Offset();
  else if (AFI->isGPRCalleeSavedArea2Frame(FI))
    return Offset - AFI->getGPRCalleeSavedArea2Offset();
  else if (AFI->isDPRCalleeSavedAreaFrame(FI))
    return Offset - AFI->getDPRCalleeSavedAreaOffset();

  // SP can move around if there are allocas.  We may also lose track of SP
  // when emergency spilling inside a non-reserved call frame setup.
  bool hasMovingSP = !hasReservedCallFrame(MF);

  // When dynamically realigning the stack, use the frame pointer for
  // parameters, and the stack/base pointer for locals.

  // If there is a frame pointer, use it when we can.
  if (hasFP(MF) && AFI->hasStackFrame()) {
    // Use frame pointer to reference fixed objects. Use it for locals if
    // there are VLAs (and thus the SP isn't reliable as a base).
    if (isFixed || (hasMovingSP && !RegInfo->hasBasePointer(MF))) {
      FrameReg = RegInfo->getFrameRegister(MF);
      return FPOffset;
    } else if (hasMovingSP) {
      assert(RegInfo->hasBasePointer(MF) && "missing base pointer!");
      llvm_unreachable("coffee:ResolveFrameIndexReference hasmovingsp");
    }  else if (Offset > (FPOffset < 0 ? -FPOffset : FPOffset)) {
      // Otherwise, use SP or FP, whichever is closer to the stack slot.
      FrameReg = RegInfo->getFrameRegister(MF);
      return FPOffset;
    }
  }
  // Use the base pointer if we have one.
  if (RegInfo->hasBasePointer(MF))
    FrameReg = RegInfo->getBaseRegister();
  return Offset;
}


void CoffeeFrameLowering::emitPushInst(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MI,
                                    const std::vector<CalleeSavedInfo> &CSI,
                                    unsigned StrOpc,
                                    unsigned StackSize,
                                    unsigned MIFlags) const {
  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  SmallVector<unsigned, 4> Regs;
  unsigned i = CSI.size();
  while (i != 0) {
    unsigned LastReg = 0;
    for (; i != 0; --i) {
      unsigned Reg = CSI[i-1].getReg();

      // Add the callee-saved register as live-in unless it's LR and
      // @llvm.returnaddress is called. If LR is returned for
      // @llvm.returnaddress then it's already added to the function and
      // entry block live-in sets.
      bool isKill = true;
      if (Reg == Coffee::LR) {
        if (MF.getFrameInfo()->isReturnAddressTaken() &&
            MF.getRegInfo().isLiveIn(Reg))
          isKill = false;
      }

      if (isKill)
        MBB.addLiveIn(Reg);

      LastReg = Reg;
      Regs.push_back(Reg);
    }

    if (Regs.empty())
      continue;

    if (Regs.size() > 0) {
        for (unsigned i = 0, e = Regs.size(); i < e; ++i) {
            // Guoqing: This should be implemented as pseudo instruction
            // so it is easier for assembly reader to understand

            // this can also be optimized to one instruction, but we leave it
            // as this for simplicity

            //coffee currently doesn't support multi store/load
            // the basic store/load doesn't update base pointer either


            // add the following instruction:
            // st reg, sp, 0
            // addi sp, sp, -4
            BuildMI(MBB, MI, DL, TII.get(StrOpc))
                    .addReg(Regs[e-i-1])
                    .addReg(Coffee::SP)
                    .addImm(0);


            BuildMI(MBB, MI, DL, TII.get(Coffee::ADDri), Coffee::SP)
                    .addReg(Coffee::SP).addImm(-4);
        }
    }
    Regs.clear();
  }
}


/*bool CoffeeFrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  MachineFunction &MF = *MBB.getParent();
  CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();

  MachineFrameInfo *MFI = MF.getFrameInfo();


  unsigned NumBytes = MFI->getStackSize();

  unsigned StrOpc = Coffee::STORErm;

  emitPushInst(MBB, MI, CSI, StrOpc, NumBytes, MachineInstr::FrameSetup);

  return true;
}

bool CoffeeFrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo *MFI = MF.getFrameInfo();


  unsigned NumBytes = MFI->getStackSize();
  CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
  bool isVarArg = AFI->getVarArgsRegSaveSize() > 0;
  unsigned NumAlignedDPRCS2Regs = AFI->getNumAlignedDPRCS2Regs();

  // The emitPopInst calls below do not insert reloads for the aligned DPRCS2
  // registers. Do that here instead.
  if (NumAlignedDPRCS2Regs)
      llvm_unreachable("coffee: frame lowering, unreachable");
    //emitAlignedDPRCS2Restores(MBB, MI, NumAlignedDPRCS2Regs, CSI, TRI);


  unsigned LdrOpc = Coffee::LOADmrnp;


  emitPopInst(MBB, MI, CSI, LdrOpc, NumBytes,isVarArg, 0);

  return true;
}*/



void CoffeeFrameLowering::emitPopInst(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MI,
                                   const std::vector<CalleeSavedInfo> &CSI,
                                   unsigned LdrOpc,
                                   unsigned StackSize,
                                   bool isVarArg,
                                   unsigned NumAlignedDPRCS2Regs) const {
  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();
  CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
  DebugLoc DL = MI->getDebugLoc();
  unsigned RetOpcode = MI->getOpcode();

  SmallVector<unsigned, 4> Regs;
  unsigned i = CSI.size();
  while (i != 0) {
    unsigned LastReg = 0;
    bool DeleteRet = false;
    for (; i != 0; --i) {
      unsigned Reg = CSI[i-1].getReg();
      LastReg = Reg;
      Regs.push_back(Reg);
    }

    if (Regs.empty())
      continue;

   /* if (Regs.size() > 1 || LdrOpc == 0) {
      MachineInstrBuilder MIB =
        AddDefaultPred(BuildMI(MBB, MI, DL, TII.get(LdmOpc), ARM::SP)
                       .addReg(ARM::SP));
      for (unsigned i = 0, e = Regs.size(); i < e; ++i)
        MIB.addReg(Regs[i], getDefRegState(true));
      if (DeleteRet) {
        MIB->copyImplicitOps(&*MI);
        MI->eraseFromParent();
      }
      MI = MIB;
    } else*/


    if (Regs.size() > 0) {
        // If we adjusted the reg to PC from LR above, switch it back here. We
        // only do that for LDM.

        for (unsigned i = 0, e = Regs.size(); i < e; ++i)

            BuildMI(MBB, MI, DL, TII.get(LdrOpc))
                    .addReg(Regs[i])
                    .addReg(Coffee::SP)
                    .addImm(0);
        BuildMI(MBB, MI, DL, TII.get(Coffee::ADDri), Coffee::SP)
                .addReg(Coffee::SP).addImm(4);


    }
    Regs.clear();
  }
}


