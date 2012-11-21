//===-- CoffeeInstrInfo.cpp - Coffee Instruction Information ----------------===//
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

#include "CoffeeInstrInfo.h"
#include "Coffee.h"
#include "CoffeeInstrBuilder.h"
#include "CoffeeMachineFunctionInfo.h"
#include "CoffeeTargetMachine.h"
#include "CoffeeHazardRecognizers.h"
#include "MCTargetDesc/CoffeePredicates.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/STLExtras.h"

#define GET_INSTRINFO_CTOR
#include "CoffeeGenInstrInfo.inc"

using namespace llvm;

CoffeeInstrInfo::CoffeeInstrInfo(CoffeeTargetMachine &tm)
  : CoffeeGenInstrInfo(Coffee::ADJCALLSTACKDOWN, Coffee::ADJCALLSTACKUP),
    TM(tm), RI(*this) {
}

MachineInstr *
CoffeeInstrInfo::commuteInstruction(MachineInstr *MI, bool NewMI) const {
}

void CoffeeInstrInfo::insertNoop(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI) const {
    DebugLoc DL;
    BuildMI(MBB, MI, DL, get(Coffee::NOP));
}

bool CoffeeInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB,MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
}

unsigned CoffeeInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
}

unsigned
CoffeeInstrInfo::InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                           MachineBasicBlock *FBB,
                           const SmallVectorImpl<MachineOperand> &Cond,
                           DebugLoc DL) const {

}

void CoffeeInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
    unsigned Opc = 0;

    if (Coffee::GPRCRegClass.contains(DestReg)) { // Copy to CPU Reg.
      if (Coffee::GPRCRegClass.contains(SrcReg))
        Opc = Coffee::Mov;
    }

    assert(Opc && "Cannot copy registers");

    MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

    if (DestReg)
      MIB.addReg(DestReg, RegState::Define);

    if (SrcReg)
      MIB.addReg(SrcReg, getKillRegState(KillSrc));
  }


static MachineMemOperand* GetMemOperand(MachineBasicBlock &MBB, int FI,
                                        unsigned Flag) {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = *MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(FI), Flag,
                              MFI.getObjectSize(FI), Align);
}

void
CoffeeInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 unsigned SrcReg, bool isKill, int FI,
                                 const TargetRegisterClass *RC,
                                 const TargetRegisterInfo *TRI) const {
    DebugLoc DL;
    if (I != MBB.end()) DL = I->getDebugLoc();
    MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

    unsigned Opc = 0;

    /*if (RC == Mips::CPURegsRegisterClass)
      Opc = IsN64 ? Mips::SW_P8 : Mips::SW;
    else if (RC == Mips::CPU64RegsRegisterClass)
      Opc = IsN64 ? Mips::SD_P8 : Mips::SD;
    else if (RC == Mips::FGR32RegisterClass)
      Opc = IsN64 ? Mips::SWC1_P8 : Mips::SWC1;
    else if (RC == Mips::AFGR64RegisterClass)
      Opc = Mips::SDC1;
    else if (RC == Mips::FGR64RegisterClass)
      Opc = IsN64 ? Mips::SDC164_P8 : Mips::SDC164;*/
    Opc = Coffee::SW;

    assert(Opc && "Register class not handled!");
    BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
      .addFrameIndex(FI).addImm(0).addMemOperand(MMO);


  /*  BuildMI(MBB, MI, DL, TII.get(StrOpc))
            .addReg(Regs[e-i-1])
            .addReg(Coffee::SP)
            .addImm(0);


    BuildMI(MBB, MI, DL, TII.get(Coffee::ADDri), Coffee::SP)
            .addReg(Coffee::SP).addImm(-4);*/

}



void
CoffeeInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator I,
                                   unsigned DestReg, int FI,
                                   const TargetRegisterClass *RC,
                                   const TargetRegisterInfo *TRI) const {
    DebugLoc DL;
    if (I != MBB.end()) DL = I->getDebugLoc();
    MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
    unsigned Opc = Coffee::LW;

    /*if (RC == Mips::CPURegsRegisterClass)
      Opc = IsN64 ? Mips::LW_P8 : Mips::LW;
    else if (RC == Mips::CPU64RegsRegisterClass)
      Opc = IsN64 ? Mips::LD_P8 : Mips::LD;
    else if (RC == Mips::FGR32RegisterClass)
      Opc = IsN64 ? Mips::LWC1_P8 : Mips::LWC1;
    else if (RC == Mips::AFGR64RegisterClass)
      Opc = Mips::LDC1;
    else if (RC == Mips::FGR64RegisterClass)
      Opc = IsN64 ? Mips::LDC164_P8 : Mips::LDC164;*/

    assert(Opc && "Register class not handled!");
    BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
}

MachineInstr*
CoffeeInstrInfo::emitFrameIndexDebugValue(MachineFunction &MF,
                                       int FrameIx, uint64_t Offset,
                                       const MDNode *MDPtr,
                                       DebugLoc DL) const {
}

bool CoffeeInstrInfo::
ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
}

unsigned CoffeeInstrInfo::GetInstSizeInBytes(const MachineInstr *MI) const {
}


void llvm::emitCoffeeRegPlusImmediate(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator &MBBI, DebugLoc dl,
                               unsigned DestReg, unsigned BaseReg, int NumBytes,
                               const TargetInstrInfo &TII, unsigned MIFlags) {
    if (NumBytes) {
        //might need do more if the num of bytes is too big
        unsigned Opc = Coffee::ADDri;
        BuildMI(MBB, MBBI, dl, TII.get(Opc), DestReg)
                .addReg(BaseReg, RegState::Kill).addImm(NumBytes)
                .setMIFlags(MIFlags);
        BaseReg = DestReg;
    }
}
