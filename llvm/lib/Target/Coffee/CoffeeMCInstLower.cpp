//===-- CoffeeMCInstLower.cpp - Convert Coffee MachineInstr to an MCInst --------===//
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

#include "Coffee.h"
#include "CoffeeAsmPrinter.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Target/Mangler.h"
#include "llvm/ADT/SmallString.h"
using namespace llvm;

static MachineModuleInfoMachO &getMachOMMI(AsmPrinter &AP) {
  return AP.MMI->getObjFileInfo<MachineModuleInfoMachO>();
}



MCOperand CoffeeAsmPrinter::GetSymbolRef(const MachineOperand &MO,
                                      const MCSymbol *Symbol) {
  const MCExpr *Expr;
  switch (MO.getTargetFlags()) {
  default: {
    Expr = MCSymbolRefExpr::Create(Symbol, MCSymbolRefExpr::VK_None,
                                   OutContext);
    switch (MO.getTargetFlags()) {
    default: llvm_unreachable("Unknown target flag on symbol operand");
    case 0:
      break;
    }
    break;
  }
  }

  if (!MO.isJTI() && MO.getOffset())
    Expr = MCBinaryExpr::CreateAdd(Expr,
                                   MCConstantExpr::Create(MO.getOffset(),
                                                          OutContext),
                                   OutContext);
  return MCOperand::CreateExpr(Expr);
}


bool CoffeeAsmPrinter::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
    switch (MO.getType()) {
    default:
      llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      assert(!MO.getSubReg() && "Subregs should be eliminated!");
      MCOp = MCOperand::CreateReg(MO.getReg());
      break;
    case MachineOperand::MO_Immediate:
      MCOp = MCOperand::CreateImm(MO.getImm());
      break;
    case MachineOperand::MO_MachineBasicBlock:
      MCOp = MCOperand::CreateExpr(MCSymbolRefExpr::Create(
                                      MO.getMBB()->getSymbol(), OutContext));
      break;
    case MachineOperand::MO_GlobalAddress:
      MCOp = GetSymbolRef(MO, Mang->getSymbol(MO.getGlobal()));
      break;
    case MachineOperand::MO_ExternalSymbol:
     MCOp = GetSymbolRef(MO,
                          GetExternalSymbolSymbol(MO.getSymbolName()));
      break;
    case MachineOperand::MO_JumpTableIndex:
      MCOp = GetSymbolRef(MO, GetJTISymbol(MO.getIndex()));
      break;
    case MachineOperand::MO_ConstantPoolIndex:
      MCOp = GetSymbolRef(MO, GetCPISymbol(MO.getIndex()));
      break;
    case MachineOperand::MO_BlockAddress:
      MCOp = GetSymbolRef(MO,GetBlockAddressSymbol(MO.getBlockAddress()));
      break;
    case MachineOperand::MO_RegisterMask:
      return false;
    }
  return true;
}

void llvm::LowerCoffeeMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                        CoffeeAsmPrinter &AP) {
  OutMI.setOpcode(MI->getOpcode());
  
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    
    MCOperand MCOp;

    if (AP.lowerOperand(MO, MCOp))
      OutMI.addOperand(MCOp);
  }
}

