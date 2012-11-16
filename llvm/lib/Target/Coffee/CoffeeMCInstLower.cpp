//===-- CoffeeMCInstLower.cpp - Convert Coffee MachineInstr to MCInst ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Coffee MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "CoffeeMCInstLower.h"
#include "CoffeeAsmPrinter.h"
#include "CoffeeInstrInfo.h"
#include "MCTargetDesc/CoffeeBaseInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Target/Mangler.h"

using namespace llvm;

CoffeeMCInstLower::CoffeeMCInstLower(CoffeeAsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void CoffeeMCInstLower::Initialize(Mangler *M, MCContext* C) {
  Mang = M;
  Ctx = C;
}

MCOperand CoffeeMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              unsigned Offset) const {
  MCSymbolRefExpr::VariantKind Kind;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
  default:                   llvm_unreachable("Invalid target flag!");
  case CoffeeII::MO_NO_FLAG:   Kind = MCSymbolRefExpr::VK_None; break;
  case CoffeeII::MO_ABS_HI:    Kind = MCSymbolRefExpr::VK_Coffee_ABS_HI; break;
  case CoffeeII::MO_ABS_LO:    Kind = MCSymbolRefExpr::VK_Coffee_ABS_LO; break;

  }

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = Mang->getSymbol(MO.getGlobal());
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AsmPrinter.GetCPISymbol(MO.getIndex());
    if (MO.getOffset())
      Offset += MO.getOffset();
    break;

  default:
    llvm_unreachable("<unknown operand type>");
  }

  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::Create(Symbol, Kind, *Ctx);

  if (!Offset)
    return MCOperand::CreateExpr(MCSym);

  // Assume offset is never negative.
  assert(Offset > 0);

  const MCConstantExpr *OffsetExpr =  MCConstantExpr::Create(Offset, *Ctx);
  const MCBinaryExpr *AddExpr = MCBinaryExpr::CreateAdd(MCSym, OffsetExpr, *Ctx);
  return MCOperand::CreateExpr(AddExpr);
}

static void CreateMCInst(MCInst& Inst, unsigned Opc, const MCOperand& Opnd0,
                         const MCOperand& Opnd1,
                         const MCOperand& Opnd2 = MCOperand()) {
  Inst.setOpcode(Opc);
  Inst.addOperand(Opnd0);
  Inst.addOperand(Opnd1);
  if (Opnd2.isValid())
    Inst.addOperand(Opnd2);
}

// Lower ".cpload $reg" to
//  "lui   $gp, %hi(_gp_disp)"
//  "addiu $gp, $gp, %lo(_gp_disp)"
//  "addu  $gp, $gp, $t9"
void CoffeeMCInstLower::LowerCPLOAD(SmallVector<MCInst, 4>& MCInsts) {
 /* MCOperand GPReg = MCOperand::CreateReg(Coffee::GP);
  MCOperand T9Reg = MCOperand::CreateReg(Coffee::T9);
  StringRef SymName("_gp_disp");
  const MCSymbol *Sym = Ctx->GetOrCreateSymbol(SymName);
  const MCSymbolRefExpr *MCSym;

  MCSym = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_Coffee_ABS_HI, *Ctx);
  MCOperand SymHi = MCOperand::CreateExpr(MCSym);
  MCSym = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_Coffee_ABS_LO, *Ctx);
  MCOperand SymLo = MCOperand::CreateExpr(MCSym);

  MCInsts.resize(3);

  CreateMCInst(MCInsts[0], Coffee::LUi, GPReg, SymHi);
  CreateMCInst(MCInsts[1], Coffee::ADDiu, GPReg, GPReg, SymLo);
  CreateMCInst(MCInsts[2], Coffee::ADDu, GPReg, GPReg, T9Reg);*/

    llvm_unreachable("coffeeMCInstLower lowercpload");
}

// Lower ".cprestore offset" to "sw $gp, offset($sp)".
void CoffeeMCInstLower::LowerCPRESTORE(int64_t Offset,
                                     SmallVector<MCInst, 4>& MCInsts) {
 /* assert(isInt<32>(Offset) && (Offset >= 0) &&
         "Imm operand of .cprestore must be a non-negative 32-bit value.");

  MCOperand SPReg = MCOperand::CreateReg(Coffee::SP), BaseReg = SPReg;
  MCOperand GPReg = MCOperand::CreateReg(Coffee::GP);

  if (!isInt<16>(Offset)) {
    unsigned Hi = ((Offset + 0x8000) >> 16) & 0xffff;
    Offset &= 0xffff;
    MCOperand ATReg = MCOperand::CreateReg(Coffee::AT);
    BaseReg = ATReg;

    // lui   at,hi
    // addu  at,at,sp
    MCInsts.resize(2);
    CreateMCInst(MCInsts[0], Coffee::LUi, ATReg, MCOperand::CreateImm(Hi));
    CreateMCInst(MCInsts[1], Coffee::ADDu, ATReg, ATReg, SPReg);
  }

  MCInst Sw;
  CreateMCInst(Sw, Coffee::SW, GPReg, BaseReg, MCOperand::CreateImm(Offset));
  MCInsts.push_back(Sw);*/

    llvm_unreachable("coffee: lowercprestore");
}

MCOperand CoffeeMCInstLower::LowerOperand(const MachineOperand& MO,
                                        unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default: llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) break;
    return MCOperand::CreateReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::CreateImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
 }

  return MCOperand();
}

void CoffeeMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}

void CoffeeMCInstLower::LowerUnalignedLoadStore(const MachineInstr *MI,
                                              SmallVector<MCInst,
                                              4>& MCInsts) {
 /* unsigned Opc = MI->getOpcode();
  MCInst Instr1, Instr2, Instr3, Move;

  bool TwoInstructions = false;

  assert(MI->getNumOperands() == 3);
  assert(MI->getOperand(0).isReg());
  assert(MI->getOperand(1).isReg());

  MCOperand Target = LowerOperand(MI->getOperand(0));
  MCOperand Base = LowerOperand(MI->getOperand(1));
  MCOperand ATReg = MCOperand::CreateReg(Coffee::AT);
  MCOperand ZeroReg = MCOperand::CreateReg(Coffee::ZERO);

  MachineOperand UnLoweredName = MI->getOperand(2);
  MCOperand Name = LowerOperand(UnLoweredName);

  Move.setOpcode(Coffee::ADDu);
  Move.addOperand(Target);
  Move.addOperand(ATReg);
  Move.addOperand(ZeroReg);

  switch (Opc) {
  case Coffee::ULW: {
    // FIXME: only works for little endian right now
    MCOperand AdjName = LowerOperand(UnLoweredName, 3);
    if (Base.getReg() == (Target.getReg())) {
      Instr1.setOpcode(Coffee::LWL);
      Instr1.addOperand(ATReg);
      Instr1.addOperand(Base);
      Instr1.addOperand(AdjName);
      Instr2.setOpcode(Coffee::LWR);
      Instr2.addOperand(ATReg);
      Instr2.addOperand(Base);
      Instr2.addOperand(Name);
      Instr3 = Move;
    } else {
      TwoInstructions = true;
      Instr1.setOpcode(Coffee::LWL);
      Instr1.addOperand(Target);
      Instr1.addOperand(Base);
      Instr1.addOperand(AdjName);
      Instr2.setOpcode(Coffee::LWR);
      Instr2.addOperand(Target);
      Instr2.addOperand(Base);
      Instr2.addOperand(Name);
    }
    break;
  }
  case Coffee::ULHu: {
    // FIXME: only works for little endian right now
    MCOperand AdjName = LowerOperand(UnLoweredName, 1);
    Instr1.setOpcode(Coffee::LBu);
    Instr1.addOperand(ATReg);
    Instr1.addOperand(Base);
    Instr1.addOperand(AdjName);
    Instr2.setOpcode(Coffee::LBu);
    Instr2.addOperand(Target);
    Instr2.addOperand(Base);
    Instr2.addOperand(Name);
    Instr3.setOpcode(Coffee::INS);
    Instr3.addOperand(Target);
    Instr3.addOperand(ATReg);
    Instr3.addOperand(MCOperand::CreateImm(0x8));
    Instr3.addOperand(MCOperand::CreateImm(0x18));
    break;
  }

  case Coffee::USW: {
    // FIXME: only works for little endian right now
    assert (Base.getReg() != Target.getReg());
    TwoInstructions = true;
    MCOperand AdjName = LowerOperand(UnLoweredName, 3);
    Instr1.setOpcode(Coffee::SWL);
    Instr1.addOperand(Target);
    Instr1.addOperand(Base);
    Instr1.addOperand(AdjName);
    Instr2.setOpcode(Coffee::SWR);
    Instr2.addOperand(Target);
    Instr2.addOperand(Base);
    Instr2.addOperand(Name);
    break;
  }
  case Coffee::USH: {
    MCOperand AdjName = LowerOperand(UnLoweredName, 1);
    Instr1.setOpcode(Coffee::SB);
    Instr1.addOperand(Target);
    Instr1.addOperand(Base);
    Instr1.addOperand(Name);
    Instr2.setOpcode(Coffee::SRL);
    Instr2.addOperand(ATReg);
    Instr2.addOperand(Target);
    Instr2.addOperand(MCOperand::CreateImm(8));
    Instr3.setOpcode(Coffee::SB);
    Instr3.addOperand(ATReg);
    Instr3.addOperand(Base);
    Instr3.addOperand(AdjName);
    break;
  }
  default:
    // FIXME: need to add others
    llvm_unreachable("unaligned instruction not processed");
  }

  MCInsts.push_back(Instr1);
  MCInsts.push_back(Instr2);
  if (!TwoInstructions) MCInsts.push_back(Instr3);*/

    llvm_unreachable("coffee lower unaligned load/store");
}

// Convert
//  "setgp01 $reg"
// to
//  "lui   $reg, %hi(_gp_disp)"
//  "addiu $reg, $reg, %lo(_gp_disp)"
void CoffeeMCInstLower::LowerSETGP01(const MachineInstr *MI,
                                   SmallVector<MCInst, 4>& MCInsts) {
  /*const MachineOperand &MO = MI->getOperand(0);
  assert(MO.isReg());
  MCOperand RegOpnd = MCOperand::CreateReg(MO.getReg());
  StringRef SymName("_gp_disp");
  const MCSymbol *Sym = Ctx->GetOrCreateSymbol(SymName);
  const MCSymbolRefExpr *MCSym;

  MCSym = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_Coffee_ABS_HI, *Ctx);
  MCOperand SymHi = MCOperand::CreateExpr(MCSym);
  MCSym = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_Coffee_ABS_LO, *Ctx);
  MCOperand SymLo = MCOperand::CreateExpr(MCSym);

  MCInsts.resize(2);

  CreateMCInst(MCInsts[0], Coffee::LUi, RegOpnd, SymHi);
  CreateMCInst(MCInsts[1], Coffee::ADDiu, RegOpnd, RegOpnd, SymLo);*/
    llvm_unreachable("coffee lower setgp01");

}


