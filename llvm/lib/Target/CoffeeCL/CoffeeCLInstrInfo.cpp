//===-- CoffeeCLInstrInfo.cpp - CoffeeCL Instruction Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the CoffeeCL implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLInstrInfo.h"
#include "CoffeeCL.h"
#include "CoffeeCLInstrBuilder.h"
#include "CoffeeCLMachineFunctionInfo.h"
#include "CoffeeCLTargetMachine.h"
#include "CoffeeCLAnalyzeImmediate.h"
#include "CoffeeCLHazardRecognizers.h"
//#include "MCTargetDesc/CoffeeCLPredicates.h"
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
#include "CoffeeCLGenInstrInfo.inc"

using namespace llvm;

CoffeeCLInstrInfo::CoffeeCLInstrInfo(CoffeeCLTargetMachine &tm)
  : CoffeeCLGenInstrInfo(CoffeeCL::ADJCALLSTACKDOWN, CoffeeCL::ADJCALLSTACKUP),
    TM(tm), RI(*this, *tm.getSubtargetImpl()), UncondBrOpc(CoffeeCL::JMP) {
}


void CoffeeCLInstrInfo::insertNoop(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI) const {
    DebugLoc DL;
    BuildMI(MBB, MI, DL, get(CoffeeCL::NOP));
}

MachineInstr*
CoffeeCLInstrInfo::emitFrameIndexDebugValue(MachineFunction &MF,
                                       int FrameIx, uint64_t Offset,
                                       const MDNode *MDPtr,
                                       DebugLoc DL) const {
    MachineInstrBuilder MIB = BuildMI(MF, DL, get(CoffeeCL::DBG_VALUE))
      .addFrameIndex(FrameIx).addImm(0).addImm(Offset).addMetadata(MDPtr);
    return &*MIB;
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//
void CoffeeCLInstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                  MachineBasicBlock *&BB,
                                  SmallVectorImpl<MachineOperand> &Cond) const {
  assert(GetAnalyzableBrOpc(Opc) && "Not an analyzable branch");
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i=0; i<NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}


unsigned CoffeeCLInstrInfo::GetAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == CoffeeCL::BEQ    || Opc == CoffeeCL::BNE    || Opc == CoffeeCL::BGT   ||
          Opc == CoffeeCL::BELT   || Opc == CoffeeCL::BLT  || Opc == CoffeeCL::BEGT   ||
          Opc == CoffeeCL::JMP) ?
         Opc : 0;
}

bool CoffeeCLInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB,MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {

    MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();

    // Skip all the debug instructions.
    while (I != REnd && I->isDebugValue())
      ++I;

    if (I == REnd || !isUnpredicatedTerminator(&*I)) {
      // If this block ends with no branches (it just falls through to its succ)
      // just return false, leaving TBB/FBB null.
      TBB = FBB = NULL;
      return false;
    }

    MachineInstr *LastInst = &*I;
    unsigned LastOpc = LastInst->getOpcode();

    // Not an analyzable branch (must be an indirect jump).
    if (!GetAnalyzableBrOpc(LastOpc))
      return true;

    // Get the second to last instruction in the block.
    unsigned SecondLastOpc = 0;
    MachineInstr *SecondLastInst = NULL;

    if (++I != REnd) {
      SecondLastInst = &*I;
      SecondLastOpc = GetAnalyzableBrOpc(SecondLastInst->getOpcode());

      // Not an analyzable branch (must be an indirect jump).
      if (isUnpredicatedTerminator(SecondLastInst) && !SecondLastOpc)
        return true;
    }

    // If there is only one terminator instruction, process it.
    if (!SecondLastOpc) {
      // Unconditional branch
      if (LastOpc == UncondBrOpc) {
        TBB = LastInst->getOperand(0).getMBB();
        return false;
      }

      // Conditional branch
      AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
      return false;
    }

    // If we reached here, there are two branches.
    // If there are three terminators, we don't know what sort of block this is.
    if (++I != REnd && isUnpredicatedTerminator(&*I))
      return true;

    // If second to last instruction is an unconditional branch,
    // analyze it and remove the last instruction.
    if (SecondLastOpc == UncondBrOpc) {
      // Return if the last instruction cannot be removed.
      if (!AllowModify)
        return true;

      TBB = SecondLastInst->getOperand(0).getMBB();
      LastInst->eraseFromParent();
      return false;
    }

    // Conditional branch followed by an unconditional branch.
    // The last one must be unconditional.
    if (LastOpc != UncondBrOpc)
      return true;

    AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
    FBB = LastInst->getOperand(0).getMBB();

    return false;
  }

unsigned CoffeeCLInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
    MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
    MachineBasicBlock::reverse_iterator FirstBr;
    unsigned removed;

    // Skip all the debug instructions.
    while (I != REnd && I->isDebugValue())
      ++I;

    FirstBr = I;

    // Up to 2 branches are removed.
    // Note that indirect branches are not removed.
    for(removed = 0; I != REnd && removed < 2; ++I, ++removed)
      if (!GetAnalyzableBrOpc(I->getOpcode()))
        break;

    MBB.erase(I.base(), FirstBr.base());

    return removed;
}

unsigned
CoffeeCLInstrInfo::InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                           MachineBasicBlock *FBB,
                           const SmallVectorImpl<MachineOperand> &Cond,
                           DebugLoc DL) const {
    // Shouldn't be a fall through.
    assert(TBB && "InsertBranch must not be told to insert a fallthrough");

    // # of condition operands:
    //  Unconditional branches: 0
    //  Floating point branches: 1 (opc)
    //  Int BranchZero: 2 (opc, reg)
    //  Int Branch: 3 (opc, reg0, reg1)
    assert((Cond.size() <= 3) &&
           "# of Mips branch conditions must be <= 3!");

    // Two-way Conditional branch.
    if (FBB) {
      BuildCondBr(MBB, TBB, DL, Cond);
      BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
      return 2;
    }

    // One way branch.
    // Unconditional branch.
    if (Cond.empty())
      BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(TBB);
    else // Conditional branch.
      BuildCondBr(MBB, TBB, DL, Cond);
    return 1;

}

void CoffeeCLInstrInfo::BuildCondBr(MachineBasicBlock &MBB,
                                MachineBasicBlock *TBB, DebugLoc DL,
                                const SmallVectorImpl<MachineOperand>& Cond)
  const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned i = 1; i < Cond.size(); ++i) {
    if (Cond[i].isReg())
      MIB.addReg(Cond[i].getReg());
    else if (Cond[i].isImm())
      MIB.addImm(Cond[i].getImm());
    else
       assert(true && "Cannot copy operand");
  }
  MIB.addMBB(TBB);
}

void CoffeeCLInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                                  bool KillSrc) const {
    unsigned Opc = 0;

    if (CoffeeCL::GPRCRegClass.contains(DestReg)) { // Copy to CPU Reg.
        if (CoffeeCL::GPRCRegClass.contains(SrcReg))
            Opc = CoffeeCL::Mov;
    }

    if (CoffeeCL::FPRCRegClass.contains(DestReg)) { // Copy to CPU Reg.
        if (CoffeeCL::FPRCRegClass.contains(SrcReg))
            Opc = CoffeeCL::Mov;
    }
    Opc = CoffeeCL::Mov;
    // TODO: Is this correct

    //if(CoffeeCL::CRRCRegClass.contains(SrcReg))
    //    return;


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
CoffeeCLInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
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
    Opc = CoffeeCL::SW;

    assert(Opc && "Register class not handled!");
    BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
      .addFrameIndex(FI).addImm(0).addMemOperand(MMO);
}



void
CoffeeCLInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator I,
                                   unsigned DestReg, int FI,
                                   const TargetRegisterClass *RC,
                                   const TargetRegisterInfo *TRI) const {
    DebugLoc DL;
    if (I != MBB.end()) DL = I->getDebugLoc();
    MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
    unsigned Opc = CoffeeCL::LW;

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


bool CoffeeCLInstrInfo::
ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
    assert( (Cond.size() && Cond.size() <= 3) &&
            "Invalid Mips branch condition!");
    Cond[0].setImm(GetOppositeBranchOpc(Cond[0].getImm()));
    return false;
}

unsigned CoffeeCLInstrInfo::GetInstSizeInBytes(const MachineInstr *MI) const {
    switch (MI->getOpcode()) {
    default:
      return MI->getDesc().getSize();
    case  TargetOpcode::INLINEASM: {       // Inline Asm: Variable size.
      const MachineFunction *MF = MI->getParent()->getParent();
      const char *AsmStr = MI->getOperand(0).getSymbolName();
      return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
    }
    }
}

/// GetOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned CoffeeCLInstrInfo::GetOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:           llvm_unreachable("Illegal opcode!");
  case CoffeeCL::BEQ:    return CoffeeCL::BNE;
  case CoffeeCL::BNE:    return CoffeeCL::BEQ;
  case CoffeeCL::BEGT:   return CoffeeCL::BLT;
  case CoffeeCL::BLT:   return CoffeeCL::BEGT;

  case CoffeeCL::BELT:   return CoffeeCL::BGT;
  case CoffeeCL::BGT:   return CoffeeCL::BELT;
  }
}

void CoffeeCLInstrInfo::ExpandRetLR(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I,
                                unsigned Opc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Opc)).addReg(CoffeeCL::LR);
}


bool CoffeeCLInstrInfo::expandPostRAPseudo(MachineBasicBlock::iterator MI) const {
  MachineBasicBlock &MBB = *MI->getParent();

  switch(MI->getDesc().getOpcode()) {
  default:
    return false;
  case CoffeeCL::RetLR:
    ExpandRetLR(MBB, MI, CoffeeCL::RET);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// Adjust SP by Amount bytes.
void CoffeeCLInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();

  if (isInt<16>(Amount))// addi sp, sp, amount
    BuildMI(MBB, I, DL, get(CoffeeCL::ADDi), CoffeeCL::SP).addReg(CoffeeCL::SP).addImm(Amount);
  else { // Expand immediate that doesn't fit in 16-bit.
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, 0);
    BuildMI(MBB, I, DL, get(CoffeeCL::ADDu), CoffeeCL::SP).addReg(CoffeeCL::SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned
CoffeeCLInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator II, DebugLoc DL,
                               unsigned *NewImm) const {
  CoffeeCLAnalyzeImmediate AnalyzeImm;

  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  unsigned Size = 32;
  unsigned LUi = CoffeeCL::LUi;

  const TargetRegisterClass *RC = &CoffeeCL::GPRCRegClass;
  bool LastInstrIsADDiu = NewImm;

  const CoffeeCLAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);
  CoffeeCLAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  unsigned Reg = RegInfo.createVirtualRegister(RC);

  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, get(LUi), Reg).addImm(SignExtend64<16>(Inst->ImmOpnd));


  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg, RegState::Kill)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}
