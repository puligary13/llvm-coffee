//===-- CoffeeAsmPrinter.cpp - Print machine instrs to Coffee assembly ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to Coffee assembly language. This printer is
// the output mechanism used by `llc'.
//
// Documentation at http://developer.apple.com/documentation/DeveloperTools/
// Reference/Assembler/ASMIntroduction/chapter_1_section_1.html
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "asmprinter"
#include "Coffee.h"
#include "CoffeeTargetMachine.h"
#include "InstPrinter/CoffeeInstPrinter.h"
#include "MCTargetDesc/CoffeePredicates.h"
#include "llvm/Analysis/DebugInfo.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Assembly/Writer.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSectionMachO.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ELF.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"


namespace llvm{
  class CoffeeAsmPrinter : public AsmPrinter {
  protected:
    DenseMap<MCSymbol*, MCSymbol*> TOC;
    uint64_t TOCLabelID;
  public:
    explicit CoffeeAsmPrinter(TargetMachine &TM, MCStreamer &Streamer)
      : AsmPrinter(TM, Streamer), TOCLabelID(0) {}

    virtual const char *getPassName() const {
      return "Coffee Assembly Printer";
    }

    virtual void EmitInstruction(const MachineInstr *MI);

    void printOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O);

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         unsigned AsmVariant, const char *ExtraCode,
                         raw_ostream &O);
    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               unsigned AsmVariant, const char *ExtraCode,
                               raw_ostream &O);

    bool doFinalization(Module &M);

    virtual void EmitFunctionEntryLabel();

    // lowerOperand - Convert a MachineOperand into the equivalent MCOperand.
    bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

    MachineLocation getDebugValueLocation(const MachineInstr *MI) const {
      MachineLocation Location;
      assert(MI->getNumOperands() == 4 && "Invalid no. of machine operands!");
      // Frame address.  Currently handles register +- offset only.
      if (MI->getOperand(0).isReg() && MI->getOperand(2).isImm())
        Location.set(MI->getOperand(0).getReg(), MI->getOperand(2).getImm());
      else {
        DEBUG(dbgs() << "DBG_VALUE instruction ignored! " << *MI << "\n");
      }
      return Location;
    }

    MCOperand GetSymbolRef(const MachineOperand &MO,
                                          const MCSymbol *Symbol);
  };

} // end of anonymous namespace


