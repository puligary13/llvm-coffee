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
#include "CoffeeMachineFunctionInfo.h"
#include "CoffeeFrameLowering.h"
//#include "CoffeeSubtarget.h"
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

/*

getCalleeSavedRegs — Returns a list of callee-saved registers in the order of the desired callee-save stack frame offset.
getReservedRegs — Returns a bitset indexed by physical register numbers, indicating if a particular register is unavailable.
hasFP — Return a Boolean indicating if a function should have a dedicated frame pointer register.
eliminateCallFramePseudoInstr — If call frame setup or destroy pseudo instructions are used, this can be called to eliminate them.
eliminateFrameIndex — Eliminate abstract frame indices from instructions that may use them.
emitPrologue — Insert prologue code into the function.
emitEpilogue — Insert epilogue code into the function.

*/

using namespace llvm;


static cl::opt<bool>
EnableBasePointer("coffee-use-base-pointer", cl::Hidden, cl::init(true),
          cl::desc("Enable use of a base pointer for complex stack frames"));

CoffeeRegisterInfo::CoffeeRegisterInfo(const TargetInstrInfo &tii)
    : CoffeeGenRegisterInfo(/*Coffee::LR*/ Coffee::LR,0,0), // the number is drawf related
      TII(tii), FramePtr(Coffee::FP), BasePtr(Coffee::GP) // no particular reason, follow ARM for base register
     {}


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
      Coffee::SP, Coffee::FP, Coffee::LR
    };

    BitVector Reserved(getNumRegs());
    typedef TargetRegisterClass::iterator RegIter;

    for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)
      Reserved.set(ReservedCPURegs[I]);

    // If GP is dedicated as a global base register, reserve it.
    if (MF.getInfo<CoffeeFunctionInfo>()->globalBaseRegFixed()) {
      Reserved.set(Coffee::GP);
    }

    return Reserved;
  }

void CoffeeRegisterInfo::eliminateCallFramePseudoInstr(MachineFunction &MF,
                                                       MachineBasicBlock &MBB,
                                                       MachineBasicBlock::iterator I) const {
    const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();
    if (!TFI->hasReservedCallFrame(MF)) {
        llvm_unreachable("coffee: eliminate call frame guoqing");
    /*  // If we have alloca, convert as follows:
      // ADJCALLSTACKDOWN -> sub, sp, sp, amount
      // ADJCALLSTACKUP   -> add, sp, sp, amount
      MachineInstr *Old = I;
      DebugLoc dl = Old->getDebugLoc();
      unsigned Amount = Old->getOperand(0).getImm();
      if (Amount != 0) {
        // We need to keep the stack aligned properly.  To do this, we round the
        // amount of space needed for the outgoing arguments up to the next
        // alignment boundary.
        unsigned Align = TFI->getStackAlignment();
        Amount = (Amount+Align-1)/Align*Align;

        ARMFunctionInfo *AFI = MF.getInfo<ARMFunctionInfo>();
        assert(!AFI->isThumb1OnlyFunction() &&
               "This eliminateCallFramePseudoInstr does not support Thumb1!");
        bool isARM = !AFI->isThumbFunction();

        // Replace the pseudo instruction with a new instruction...
        unsigned Opc = Old->getOpcode();
        int PIdx = Old->findFirstPredOperandIdx();
        ARMCC::CondCodes Pred = (PIdx == -1)
          ? ARMCC::AL : (ARMCC::CondCodes)Old->getOperand(PIdx).getImm();
        if (Opc == ARM::ADJCALLSTACKDOWN || Opc == ARM::tADJCALLSTACKDOWN) {
          // Note: PredReg is operand 2 for ADJCALLSTACKDOWN.
          unsigned PredReg = Old->getOperand(2).getReg();
          emitSPUpdate(isARM, MBB, I, dl, TII, -Amount, Pred, PredReg);
        } else {
          // Note: PredReg is operand 3 for ADJCALLSTACKUP.
          unsigned PredReg = Old->getOperand(3).getReg();
          assert(Opc == ARM::ADJCALLSTACKUP || Opc == ARM::tADJCALLSTACKUP);
          emitSPUpdate(isARM, MBB, I, dl, TII, Amount, Pred, PredReg);
        }
      }*/
    }
    MBB.erase(I);
}

void CoffeeRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                             int SPAdj, RegScavenger *RS) const {

    MachineInstr &MI = *II;
    MachineFunction &MF = *MI.getParent()->getParent();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();

    unsigned i = 0;
    while (!MI.getOperand(i).isFI()) {
      ++i;
      assert(i < MI.getNumOperands() &&
             "Instr doesn't have FrameIndex operand!");
    }

    int FrameIndex = MI.getOperand(i).getIndex();
    uint64_t stackSize = MF.getFrameInfo()->getStackSize();
    int64_t spOffset = MF.getFrameInfo()->getObjectOffset(FrameIndex);


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

    if (CoffeeFI->isOutArgFI(FrameIndex) || CoffeeFI->isDynAllocFI(FrameIndex) ||
        (FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI))
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
    int64_t Offset;

    if (CoffeeFI->isOutArgFI(FrameIndex) ||
        CoffeeFI->isDynAllocFI(FrameIndex))
      Offset = spOffset;
    else
      Offset = spOffset + (int64_t)stackSize;

    Offset += MI.getOperand(i+1).getImm();


    // If MI is not a debug value, make sure Offset fits in the 16-bit immediate
    // field.
    if (!MI.isDebugValue() && !isInt<15>(Offset)) {
        llvm_unreachable("coffee: imm is too big");
     /* MachineBasicBlock &MBB = *MI.getParent();
      DebugLoc DL = II->getDebugLoc();
      MipsAnalyzeImmediate AnalyzeImm;
      unsigned Size = Subtarget.isABI_N64() ? 64 : 32;
      unsigned LUi = Subtarget.isABI_N64() ? Mips::LUi64 : Mips::LUi;
      unsigned ADDu = Subtarget.isABI_N64() ? Mips::DADDu : Mips::ADDu;
      unsigned ZEROReg = Subtarget.isABI_N64() ? Mips::ZERO_64 : Mips::ZERO;
      unsigned ATReg = Subtarget.isABI_N64() ? Mips::AT_64 : Mips::AT;
      const MipsAnalyzeImmediate::InstSeq &Seq =
        AnalyzeImm.Analyze(Offset, Size, true );
      MipsAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

      CoffeeFI->setEmitNOAT();

      // The first instruction can be a LUi, which is different from other
      // instructions (ADDiu, ORI and SLL) in that it does not have a register
      // operand.
      if (Inst->Opc == LUi)
        BuildMI(MBB, II, DL, TII.get(LUi), ATReg)
          .addImm(SignExtend64<16>(Inst->ImmOpnd));
      else
        BuildMI(MBB, II, DL, TII.get(Inst->Opc), ATReg).addReg(ZEROReg)
          .addImm(SignExtend64<16>(Inst->ImmOpnd));

      // Build the remaining instructions in Seq except for the last one.
      for (++Inst; Inst != Seq.end() - 1; ++Inst)
        BuildMI(MBB, II, DL, TII.get(Inst->Opc), ATReg).addReg(ATReg)
          .addImm(SignExtend64<16>(Inst->ImmOpnd));

      BuildMI(MBB, II, DL, TII.get(ADDu), ATReg).addReg(FrameReg).addReg(ATReg);

      FrameReg = ATReg;
      Offset = SignExtend64<16>(Inst->ImmOpnd);*/
    }

    MI.getOperand(i).ChangeToRegister(FrameReg, false);
    MI.getOperand(i+1).ChangeToImmediate(Offset);
  }


void CoffeeRegisterInfo::processFunctionBeforeFrameFinalized(MachineFunction &MF) const {
    llvm_unreachable("coffee: process function before frame Finalized");
}

unsigned CoffeeRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
    //we have dedicated frame register
    const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();
    if (TFI->hasFP(MF))
        return FramePtr;
    return Coffee::SP;
}



bool CoffeeRegisterInfo::hasBasePointer(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  //guqoing: should double check the usage of base pointer, when it is needed ?
  if (!EnableBasePointer)
    return false;

  if (!TFI->hasReservedCallFrame(MF))
    return true;

  return false;
}

bool CoffeeRegisterInfo::canRealignStack(const MachineFunction &MF) const {
  //guoqing: we don't allow dynamic realign stack
    return false;
}


bool CoffeeRegisterInfo::
cannotEliminateFrame(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  if (MF.getTarget().Options.DisableFramePointerElim(MF) && MFI->adjustsStack())
    return true;
  return MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();
}
