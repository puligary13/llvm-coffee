//===-- CoffeeInstPrinter.cpp - Convert Coffee MCInst to assembly syntax --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints an Coffee MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "asm-printer"
#include "CoffeeInstPrinter.h"
#include "MCTargetDesc/CoffeeBaseInfo.h"
#include "MCTargetDesc/CoffeePredicates.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#include "CoffeeGenAsmWriter.inc"

void CoffeeInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  OS << getRegisterName(RegNo);
}

void CoffeeInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                               StringRef Annot) {
  // Check for slwi/srwi mnemonics.
  /*if (MI->getOpcode() == Coffee::RLWINM) {
    unsigned char SH = MI->getOperand(2).getImm();
    unsigned char MB = MI->getOperand(3).getImm();
    unsigned char ME = MI->getOperand(4).getImm();
    bool useSubstituteMnemonic = false;
    if (SH <= 31 && MB == 0 && ME == (31-SH)) {
      O << "\tslwi "; useSubstituteMnemonic = true;
    }
    if (SH <= 31 && MB == (32-SH) && ME == 31) {
      O << "\tsrwi "; useSubstituteMnemonic = true;
      SH = 32-SH;
    }
    if (useSubstituteMnemonic) {
      printOperand(MI, 0, O);
      O << ", ";
      printOperand(MI, 1, O);
      O << ", " << (unsigned int)SH;

      printAnnotation(O, Annot);
      return;
    }
  }

  if ((MI->getOpcode() == Coffee::OR || MI->getOpcode() == Coffee::OR8) &&
      MI->getOperand(1).getReg() == MI->getOperand(2).getReg()) {
    O << "\tmr ";
    printOperand(MI, 0, O);
    O << ", ";
    printOperand(MI, 1, O);
    printAnnotation(O, Annot);
    return;
  }

  if (MI->getOpcode() == Coffee::RLDICR) {
    unsigned char SH = MI->getOperand(2).getImm();
    unsigned char ME = MI->getOperand(3).getImm();
    // rldicr RA, RS, SH, 63-SH == sldi RA, RS, SH
    if (63-SH == ME) {
      O << "\tsldi ";
      printOperand(MI, 0, O);
      O << ", ";
      printOperand(MI, 1, O);
      O << ", " << (unsigned int)SH;
      printAnnotation(O, Annot);
      return;
    }
  }*/

  printInstruction(MI, O);
  printAnnotation(O, Annot);
}


void CoffeeInstPrinter::printPredicateOperand(const MCInst *MI, unsigned OpNo,
                                           raw_ostream &O,
                                           const char *Modifier) {
  assert(Modifier && "Must specify 'cc' or 'reg' as predicate op modifier!");
  unsigned Code = MI->getOperand(OpNo).getImm();
  if (StringRef(Modifier) == "cc") {
    switch ((Coffee::Predicate)Code) {
    case Coffee::PRED_ALWAYS: return; // Don't print anything for always.
    case Coffee::PRED_LT: O << "lt"; return;
    case Coffee::PRED_LE: O << "le"; return;
    case Coffee::PRED_EQ: O << "eq"; return;
    case Coffee::PRED_GE: O << "ge"; return;
    case Coffee::PRED_GT: O << "gt"; return;
    case Coffee::PRED_NE: O << "ne"; return;
    case Coffee::PRED_UN: O << "un"; return;
    case Coffee::PRED_NU: O << "nu"; return;
    }
  }

  assert(StringRef(Modifier) == "reg" &&
         "Need to specify 'cc' or 'reg' as predicate op modifier!");
  // Don't print the register for 'always'.
  if (Code == Coffee::PRED_ALWAYS) return;
  printOperand(MI, OpNo+1, O);
}

void CoffeeInstPrinter::printS5ImmOperand(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  char Value = MI->getOperand(OpNo).getImm();
  Value = (Value << (32-5)) >> (32-5);
  O << (int)Value;
}

void CoffeeInstPrinter::printU5ImmOperand(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  unsigned char Value = MI->getOperand(OpNo).getImm();
  assert(Value <= 31 && "Invalid u5imm argument!");
  O << (unsigned int)Value;
}

void CoffeeInstPrinter::printU6ImmOperand(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  unsigned char Value = MI->getOperand(OpNo).getImm();
  assert(Value <= 63 && "Invalid u6imm argument!");
  O << (unsigned int)Value;
}

void CoffeeInstPrinter::printS16ImmOperand(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  O << (short)MI->getOperand(OpNo).getImm();
}

void CoffeeInstPrinter::printU16ImmOperand(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  O << (unsigned short)MI->getOperand(OpNo).getImm();
}

void CoffeeInstPrinter::printS16X4ImmOperand(const MCInst *MI, unsigned OpNo,
                                          raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm())
    O << (short)(MI->getOperand(OpNo).getImm()*4);
  else
    printOperand(MI, OpNo, O);
}

void CoffeeInstPrinter::printBranchOperand(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  if (!MI->getOperand(OpNo).isImm())
    return printOperand(MI, OpNo, O);

  // Branches can take an immediate operand.  This is used by the branch
  // selection pass to print $+8, an eight byte displacement from the PC.
  O << "$+";
  printAbsAddrOperand(MI, OpNo, O);
}

void CoffeeInstPrinter::printAbsAddrOperand(const MCInst *MI, unsigned OpNo,
                                         raw_ostream &O) {
  O << (int)MI->getOperand(OpNo).getImm()*4;
}


void CoffeeInstPrinter::printcrbitm(const MCInst *MI, unsigned OpNo,
                                 raw_ostream &O) {
  unsigned CCReg = MI->getOperand(OpNo).getReg();
  unsigned RegNo;
  switch (CCReg) {
  default: llvm_unreachable("Unknown CR register");
  case Coffee::CR0: RegNo = 0; break;
  case Coffee::CR1: RegNo = 1; break;
  case Coffee::CR2: RegNo = 2; break;
  case Coffee::CR3: RegNo = 3; break;
  case Coffee::CR4: RegNo = 4; break;
  case Coffee::CR5: RegNo = 5; break;
  case Coffee::CR6: RegNo = 6; break;
  case Coffee::CR7: RegNo = 7; break;
  }
  O << (0x80 >> RegNo);
}

void CoffeeInstPrinter::printMemRegImm(const MCInst *MI, unsigned OpNo,
                                    raw_ostream &O) {
  printSymbolLo(MI, OpNo, O);
  O << '(';
  if (MI->getOperand(OpNo+1).getReg() == Coffee::T0)
    O << "0";
  else
    printOperand(MI, OpNo+1, O);
  O << ')';
}

void CoffeeInstPrinter::printMemRegImmShifted(const MCInst *MI, unsigned OpNo,
                                           raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm())
    printS16X4ImmOperand(MI, OpNo, O);
  else
    printSymbolLo(MI, OpNo, O);
  O << '(';

  if (MI->getOperand(OpNo+1).getReg() == Coffee::T0)
    O << "0";
  else
    printOperand(MI, OpNo+1, O);
  O << ')';
}


void CoffeeInstPrinter::printMemRegReg(const MCInst *MI, unsigned OpNo,
                                    raw_ostream &O) {
  // When used as the base register, r0 reads constant zero rather than
  // the value contained in the register.  For this reason, the darwin
  // assembler requires that we print r0 as 0 (no r) when used as the base.
  if (MI->getOperand(OpNo).getReg() == Coffee::T0)
    O << "0";
  else
    printOperand(MI, OpNo, O);
  O << ", ";
  printOperand(MI, OpNo+1, O);
}



/// stripRegisterPrefix - This method strips the character prefix from a
/// register name so that only the number is left.  Used by for linux asm.
static const char *stripRegisterPrefix(const char *RegName) {
  switch (RegName[0]) {
  case 'r':
  case 'f':
  case 'v': return RegName + 1;
  case 'c': if (RegName[1] == 'r') return RegName + 2;
  }

  return RegName;
}

void CoffeeInstPrinter::printMemOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O) {

   /* const MCOperand &Op = MI->getOperand(OpNo);
    const MCOperand &Op1 = MI->getOperand(OpNo+1);

    const char *RegName = getRegisterName(Op.getReg());

    O << RegName;
    O <<",\t";
    O <<(int)Op1.getImm();  // cast to int in order to show negative numbers*/

    printOperand(MI, OpNo, O);
    O <<",\t";
    printOperand(MI, OpNo+1, O);

}

void CoffeeInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    const char *RegName = getRegisterName(Op.getReg());

    O << RegName;
    return;
  }

  if (Op.isImm()) {
    O << Op.getImm();
    return;
  }

  assert(Op.isExpr() && "unknown operand kind in printOperand");
  O << *Op.getExpr();
}

void CoffeeInstPrinter::printSymbolLo(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm())
    return printS16ImmOperand(MI, OpNo, O);

  // FIXME: This is a terrible hack because we can't encode lo16() as an operand
  // flag of a subtraction.  See the FIXME in GetSymbolRef in CoffeeMCInstLower.
  if (MI->getOperand(OpNo).isExpr() &&
      isa<MCBinaryExpr>(MI->getOperand(OpNo).getExpr())) {
    O << "lo16(";
    printOperand(MI, OpNo, O);
    O << ')';
  } else {
    printOperand(MI, OpNo, O);
  }
}

void CoffeeInstPrinter::printSymbolHi(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm())
    return printS16ImmOperand(MI, OpNo, O);

  // FIXME: This is a terrible hack because we can't encode lo16() as an operand
  // flag of a subtraction.  See the FIXME in GetSymbolRef in CoffeeMCInstLower.
  if (MI->getOperand(OpNo).isExpr() &&
      isa<MCBinaryExpr>(MI->getOperand(OpNo).getExpr())) {
    O << "ha16(";
    printOperand(MI, OpNo, O);
    O << ')';
  } else {
    printOperand(MI, OpNo, O);
  }
}


