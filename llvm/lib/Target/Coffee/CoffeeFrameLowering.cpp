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

    const TargetRegisterInfo *RegInfo = MF.getTarget().getRegisterInfo();
    const MachineFrameInfo *MFI = MF.getFrameInfo();
    // Always eliminate non-leaf frame pointers.
    return ((MF.getTarget().Options.DisableFramePointerElim(MF) &&
             MFI->hasCalls()) ||
            RegInfo->needsStackRealignment(MF) ||
            MFI->hasVarSizedObjects() ||
            MFI->isFrameAddressTaken());

}

static void
emitSPUpdate(MachineBasicBlock &MBB, MachineBasicBlock::iterator &MBBI,
             DebugLoc dl, const TargetInstrInfo &TII,
             int NumBytes, unsigned MIFlags = MachineInstr::NoFlags) {
    emitCoffeeRegPlusImmediate(MBB, MBBI, dl, Coffee::SP, Coffee::SP, NumBytes,
                               TII, MIFlags);
}

void CoffeeFrameLowering::emitPrologue(MachineFunction &MF) const {
    MachineBasicBlock &MBB = MF.front();
    MachineBasicBlock::iterator MBBI = MBB.begin();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
    const CoffeeRegisterInfo *RegInfo =
            static_cast<const CoffeeRegisterInfo*>(MF.getTarget().getRegisterInfo());
    const CoffeeInstrInfo &TII =
            *static_cast<const CoffeeInstrInfo*>(MF.getTarget().getInstrInfo());

    unsigned VARegSaveSize = AFI->getVarArgsRegSaveSize();

     bool isPIC = (MF.getTarget().getRelocationModel() == Reloc::PIC_);
     if (isPIC)
         llvm_unreachable("coffee: emitPrologue, PIC");

    if(VARegSaveSize)
        llvm_unreachable("coffee: Size of the register save area for vararg functions is not zero");


    const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();
    DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
    unsigned FramePtr = RegInfo->getFrameRegister(MF);

    // Determine the sizes of each callee-save LR areas and record which frame
    // belongs to which callee-save spill areas.
    unsigned GPRCS1Size = 0, GPRCS2Size = 0, DPRCSSize = 0;
    int FramePtrSpillFI = 0;
    int D8SpillFI = 0;

    // Allocate the vararg register save area. This is not counted in NumBytes.
    if (VARegSaveSize)
        emitSPUpdate(MBB, MBBI, dl, TII, -VARegSaveSize,
                     MachineInstr::FrameSetup);

    unsigned StackSize = MFI->getStackSize();


    if (StackSize == 0) return;

   // for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
        // check spillCalleeSavedRegisters for instruction details
   //     GPRCS1Size += 4;
   // }

    emitSPUpdate(MBB, MBBI, dl, TII, -StackSize,
                      MachineInstr::FrameSetup);

}

void CoffeeFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
    MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
    assert(MBBI->isReturn() && "Can only insert epilog into returning blocks");
    unsigned RetOpcode = MBBI->getOpcode();
    DebugLoc dl = MBBI->getDebugLoc();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
    const TargetRegisterInfo *RegInfo = MF.getTarget().getRegisterInfo();
    const CoffeeInstrInfo &TII =
      *static_cast<const CoffeeInstrInfo*>(MF.getTarget().getInstrInfo());
 const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();

 unsigned GPRCS1Size = 0;
    unsigned VARegSaveSize = 0; // guoqing: don't support VA reg
    int StackSize = (int)MFI->getStackSize();
    unsigned FramePtr = RegInfo->getFrameRegister(MF);

   if (StackSize == 0) return;


   if (!AFI->hasStackFrame()) {
     if (StackSize != 0)
       emitSPUpdate(MBB, MBBI, dl, TII, StackSize);
   } else {
     // Unwind MBBI to point to first LDR / VLDRD.
     //  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
     //      GPRCS1Size += 4;
     //  }
       emitSPUpdate(MBB, MBBI, dl, TII, StackSize,
                         MachineInstr::FrameSetup);
   }



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
      (RegInfo->needsStackRealignment(MF) && MFI->getObjectIndexEnd() != 0))
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
    // This tells PEI to spill the FP as if it is any other callee-save register
    // to take advantage the eliminateFrameIndex machinery. This also ensures it
    // is spilled in the order specified by getCalleeSavedRegs() to make it easier
    // to combine multiple loads / stores.
    bool CanEliminateFrame = true;
    bool CS1Spilled = false;
    bool LRSpilled = false;
    unsigned NumGPRSpills = 0;
    SmallVector<unsigned, 4> UnspilledCS1GPRs;
    SmallVector<unsigned, 4> UnspilledCS2GPRs;
    const CoffeeRegisterInfo *RegInfo =
      static_cast<const CoffeeRegisterInfo*>(MF.getTarget().getRegisterInfo());
    const CoffeeInstrInfo &TII =
      *static_cast<const CoffeeInstrInfo*>(MF.getTarget().getInstrInfo());
    CoffeeFunctionInfo *AFI = MF.getInfo<CoffeeFunctionInfo>();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    unsigned FramePtr = RegInfo->getFrameRegister(MF);

    // See if we can spill vector registers to aligned stack.
    //checkNumAlignedDPRCS2Regs(MF);

    // Spill the BasePtr if it's used.
    if (RegInfo->hasBasePointer(MF))
      MF.getRegInfo().setPhysRegUsed(RegInfo->getBaseRegister());

    // Don't spill FP if the frame can be eliminated. This is determined
    // by scanning the callee-save registers to see if any is used.
    const uint16_t *CSRegs = RegInfo->getCalleeSavedRegs();
    for (unsigned i = 0; CSRegs[i]; ++i) {
      unsigned Reg = CSRegs[i];
      bool Spilled = false;
      if (MF.getRegInfo().isPhysRegOrOverlapUsed(Reg)) {
        Spilled = true;
        CanEliminateFrame = false;
      }

      if (!Coffee::GPRCRegisterClass->contains(Reg))
        continue;

      if (Spilled) {
          NumGPRSpills++;

          if (Reg == Coffee::LR)
              LRSpilled = true;
          CS1Spilled = true;

      } else {

          UnspilledCS1GPRs.push_back(Reg);
          continue;

      }
    }

    bool ForceLRSpill = false;


    // If any of the stack slot references may be out of range of an immediate
    // offset, make sure a register (or a spill slot) is available for the
    // register scavenger. Note that if we're indexing off the frame pointer, the
    // effective stack size is 4 bytes larger since the FP points to the stack
    // slot of the previous FP. Also, if we have variable sized objects in the
    // function, stack slot references will often be negative, and some of
    // our instructions are positive-offset only, so conservatively consider
    // that case to want a spill slot (or register) as well. Similarly, if
    // the function adjusts the stack pointer during execution and the
    // adjustments aren't already part of our stack size estimate, our offset
    // calculations may be off, so be conservative.
    // FIXME: We could add logic to be more precise about negative offsets
    //        and which instructions will need a scratch register for them. Is it
    //        worth the effort and added fragility?
    bool BigStack =
      (RS &&
       (estimateStackSize(MF) + ((hasFP(MF) && AFI->hasStackFrame()) ? 4:0) >=
        estimateRSStackSizeLimit(MF, this)))
      || MFI->hasVarSizedObjects()
      || (MFI->adjustsStack() && !canSimplifyCallFramePseudos(MF));

    if (BigStack)
        llvm_unreachable("coffee: the stack size is too big to handle the by addri instruction");

    bool ExtraCSSpill = false;
    if (BigStack || !CanEliminateFrame || RegInfo->cannotEliminateFrame(MF)) {
        AFI->setHasStackFrame(true);
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
  if (RegInfo->needsStackRealignment(MF))
      llvm_unreachable("coffee:ResolveFrameIndexReference no stack realignment support ");

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


