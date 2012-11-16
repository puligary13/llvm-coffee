//===-- CoffeeAsmPrinter.cpp - Coffee LLVM Assembly Printer -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format Coffee assembly language.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Coffee-asm-printer"
#include "CoffeeAsmPrinter.h"
#include "Coffee.h"
#include "CoffeeInstrInfo.h"
#include "InstPrinter/CoffeeInstPrinter.h"
#include "MCTargetDesc/CoffeeBaseInfo.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Analysis/DebugInfo.h"
#include "llvm/BasicBlock.h"
#include "llvm/Instructions.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/Instructions.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

void CoffeeAsmPrinter::EmitInstrWithMacroNoAT(const MachineInstr *MI) {
  MCInst TmpInst;

  MCInstLowering.Lower(MI, TmpInst);
  OutStreamer.EmitRawText(StringRef("\t.set\tmacro"));
 /* if (CoffeeFI->getEmitNOAT())
    OutStreamer.EmitRawText(StringRef("\t.set\tat"));
  OutStreamer.EmitInstruction(TmpInst);
  if (CoffeeFI->getEmitNOAT())
    OutStreamer.EmitRawText(StringRef("\t.set\tnoat"));*/
  OutStreamer.EmitRawText(StringRef("\t.set\tnomacro"));
}

bool CoffeeAsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();
  AsmPrinter::runOnMachineFunction(MF);
  return true;
}

void CoffeeAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  if (MI->isDebugValue()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);

    PrintDebugValueComment(MI, OS);
    return;
  }

  unsigned Opc = MI->getOpcode();
  MCInst TmpInst0;
  SmallVector<MCInst, 4> MCInsts;

  switch (Opc) {

  default:
    break;
  }

  MCInstLowering.Lower(MI, TmpInst0);
  OutStreamer.EmitInstruction(TmpInst0);
}

//===----------------------------------------------------------------------===//
//
//  Coffee Asm Directives
//
//  -- Frame directive "frame Stackpointer, Stacksize, RARegister"
//  Describe the stack frame.
//
//  -- Mask directives "(f)mask  bitmask, offset"
//  Tells the assembler which registers are saved and where.
//  bitmask - contain a little endian bitset indicating which registers are
//            saved on function prologue (e.g. with a 0x80000000 mask, the
//            assembler knows the register 31 (RA) is saved at prologue.
//  offset  - the position before stack pointer subtraction indicating where
//            the first saved register on prologue is located. (e.g. with a
//
//  Consider the following function prologue:
//
//    .frame  $fp,48,$ra
//    .mask   0xc0000000,-8
//       addiu $sp, $sp, -48
//       sw $ra, 40($sp)
//       sw $fp, 36($sp)
//
//    With a 0xc0000000 mask, the assembler knows the register 31 (RA) and
//    30 (FP) are saved at prologue. As the save order on prologue is from
//    left to right, RA is saved first. A -8 offset means that after the
//    stack pointer subtration, the first register in the mask (RA) will be
//    saved at address 48-8=40.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Mask directives
//===----------------------------------------------------------------------===//

// Create a bitmask with all callee saved registers for CPU or Floating Point
// registers. For CPU registers consider RA, GP and FP for saving if necessary.
void CoffeeAsmPrinter::printSavedRegsBitmask(raw_ostream &O) {


  // CPU and FPU Saved Registers Bitmasks
  unsigned GPRCBitmask = 0;
  int GPRCTopSavedRegOff;

  // Set the CPU and FPU Bitmasks
  const MachineFrameInfo *MFI = MF->getFrameInfo();
  const std::vector<CalleeSavedInfo> &CSI = MFI->getCalleeSavedInfo();
  // size of stack area to which FP callee-saved regs are saved.
  unsigned GPRCRegSize = Coffee::GPRCRegisterClass->getSize();

  unsigned i, e = CSI.size();

  // Set CPU Bitmask.
  for (i=0; i != e; ++i) {
    unsigned Reg = CSI[i].getReg();
    unsigned RegNum = getCoffeeRegisterNumbering(Reg);
    GPRCBitmask |= (1 << RegNum);
  }


  // CPU Regs are saved below FP Regs.
  GPRCTopSavedRegOff = GPRCBitmask ? - GPRCRegSize : 0;

  // Print CPUBitmask
  O << "\t.mask \t"; printHex32(GPRCBitmask, O);
  O << ',' << GPRCTopSavedRegOff << '\n';


}

// Print a 32 bit hex number with all numbers.
void CoffeeAsmPrinter::printHex32(unsigned Value, raw_ostream &O) {
  O << "0x";
  for (int i = 7; i >= 0; i--)
    O.write_hex((Value & (0xF << (i*4))) >> (i*4));
}

//===----------------------------------------------------------------------===//
// Frame and Set directives
//===----------------------------------------------------------------------===//

/// Frame Directive
void CoffeeAsmPrinter::emitFrameDirective() {
  const TargetRegisterInfo &RI = *TM.getRegisterInfo();

  unsigned stackReg  = RI.getFrameRegister(*MF);
  unsigned returnReg = RI.getRARegister();
  unsigned stackSize = MF->getFrameInfo()->getStackSize();

  if (OutStreamer.hasRawTextSupport())
    OutStreamer.EmitRawText("\t.frame\t$" +
           StringRef(CoffeeInstPrinter::getRegisterName(stackReg)).lower() +
           "," + Twine(stackSize) + ",$" +
           StringRef(CoffeeInstPrinter::getRegisterName(returnReg)).lower());
}

/// Emit Set directives.
const char *CoffeeAsmPrinter::getCurrentABIString() const {
    return "abi32";
}

void CoffeeAsmPrinter::EmitFunctionEntryLabel() {
  if (OutStreamer.hasRawTextSupport())
    OutStreamer.EmitRawText("\t.ent\t" + Twine(CurrentFnSym->getName()));
  OutStreamer.EmitLabel(CurrentFnSym);
}

/// EmitFunctionBodyStart - Targets can override this to emit stuff before
/// the first basic block in the function.
void CoffeeAsmPrinter::EmitFunctionBodyStart() {
  MCInstLowering.Initialize(Mang, &MF->getContext());

  emitFrameDirective();

  bool EmitCPLoad = false;

  if (OutStreamer.hasRawTextSupport()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);
    printSavedRegsBitmask(OS);
    OutStreamer.EmitRawText(OS.str());

    OutStreamer.EmitRawText(StringRef("\t.set\tnoreorder"));

    // Emit .cpload directive if needed.
    if (EmitCPLoad)
      OutStreamer.EmitRawText(StringRef("\t.cpload\t$25"));

    OutStreamer.EmitRawText(StringRef("\t.set\tnomacro"));
  //  if (CoffeeFI->getEmitNOAT())
  //    OutStreamer.EmitRawText(StringRef("\t.set\tnoat"));
  } else if (EmitCPLoad) {
    SmallVector<MCInst, 4> MCInsts;
    MCInstLowering.LowerCPLOAD(MCInsts);
    for (SmallVector<MCInst, 4>::iterator I = MCInsts.begin();
         I != MCInsts.end(); ++I)
      OutStreamer.EmitInstruction(*I);
  }
}

/// EmitFunctionBodyEnd - Targets can override this to emit stuff after
/// the last basic block in the function.
void CoffeeAsmPrinter::EmitFunctionBodyEnd() {
  // There are instruction for this macros, but they must
  // always be at the function end, and we can't emit and
  // break with BB logic.
  if (OutStreamer.hasRawTextSupport()) {
   // if (CoffeeFI->getEmitNOAT())
   //   OutStreamer.EmitRawText(StringRef("\t.set\tat"));

    OutStreamer.EmitRawText(StringRef("\t.set\tmacro"));
    OutStreamer.EmitRawText(StringRef("\t.set\treorder"));
    OutStreamer.EmitRawText("\t.end\t" + Twine(CurrentFnSym->getName()));
  }
}

/// isBlockOnlyReachableByFallthough - Return true if the basic block has
/// exactly one predecessor and the control transfer mechanism between
/// the predecessor and this block is a fall-through.
bool CoffeeAsmPrinter::isBlockOnlyReachableByFallthrough(const MachineBasicBlock*
                                                       MBB) const {
  // The predecessor has to be immediately before this block.
  const MachineBasicBlock *Pred = *MBB->pred_begin();

  // If the predecessor is a switch statement, assume a jump table
  // implementation, so it is not a fall through.
  if (const BasicBlock *bb = Pred->getBasicBlock())
    if (isa<SwitchInst>(bb->getTerminator()))
      return false;

  // If this is a landing pad, it isn't a fall through.  If it has no preds,
  // then nothing falls through to it.
  if (MBB->isLandingPad() || MBB->pred_empty())
    return false;

  // If there isn't exactly one predecessor, it can't be a fall through.
  MachineBasicBlock::const_pred_iterator PI = MBB->pred_begin(), PI2 = PI;
  ++PI2;

  if (PI2 != MBB->pred_end())
    return false;

  // The predecessor has to be immediately before this block.
  if (!Pred->isLayoutSuccessor(MBB))
    return false;

  // If the block is completely empty, then it definitely does fall through.
  if (Pred->empty())
    return true;

  // Otherwise, check the last instruction.
  // Check if the last terminator is an unconditional branch.
  MachineBasicBlock::const_iterator I = Pred->end();
  while (I != Pred->begin() && !(--I)->isTerminator()) ;

  return !I->isBarrier();
}

// Print out an operand for an inline asm expression.
bool CoffeeAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                     unsigned AsmVariant,const char *ExtraCode,
                                     raw_ostream &O) {
  // Does this asm operand have a single letter operand modifier?
  if (ExtraCode && ExtraCode[0])
    return true; // Unknown modifier.

  printOperand(MI, OpNo, O);
  return false;
}

bool CoffeeAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                           unsigned OpNum, unsigned AsmVariant,
                                           const char *ExtraCode,
                                           raw_ostream &O) {
  if (ExtraCode && ExtraCode[0])
     return true; // Unknown modifier.

  const MachineOperand &MO = MI->getOperand(OpNum);
  assert(MO.isReg() && "unexpected inline asm memory operand");
  O << "0($" << CoffeeInstPrinter::getRegisterName(MO.getReg()) << ")";
  return false;
}

void CoffeeAsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                  raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(opNum);
  bool closeP = false;

  if (MO.getTargetFlags())
    closeP = true;

  switch(MO.getTargetFlags()) {

  case CoffeeII::MO_ABS_HI:   O << "%hi(";     break;
  case CoffeeII::MO_ABS_LO:   O << "%lo(";     break;

  }

  switch (MO.getType()) {
    case MachineOperand::MO_Register:
      O << '$'
        << StringRef(CoffeeInstPrinter::getRegisterName(MO.getReg())).lower();
      break;

    case MachineOperand::MO_Immediate:
      O << MO.getImm();
      break;

    case MachineOperand::MO_MachineBasicBlock:
      O << *MO.getMBB()->getSymbol();
      return;

    case MachineOperand::MO_GlobalAddress:
      O << *Mang->getSymbol(MO.getGlobal());
      break;

    case MachineOperand::MO_BlockAddress: {
      MCSymbol* BA = GetBlockAddressSymbol(MO.getBlockAddress());
      O << BA->getName();
      break;
    }

    case MachineOperand::MO_ExternalSymbol:
      O << *GetExternalSymbolSymbol(MO.getSymbolName());
      break;

    case MachineOperand::MO_JumpTableIndex:
      O << MAI->getPrivateGlobalPrefix() << "JTI" << getFunctionNumber()
        << '_' << MO.getIndex();
      break;

    case MachineOperand::MO_ConstantPoolIndex:
      O << MAI->getPrivateGlobalPrefix() << "CPI"
        << getFunctionNumber() << "_" << MO.getIndex();
      if (MO.getOffset())
        O << "+" << MO.getOffset();
      break;

    default:
      llvm_unreachable("<unknown operand type>");
  }

  if (closeP) O << ")";
}

void CoffeeAsmPrinter::printUnsignedImm(const MachineInstr *MI, int opNum,
                                      raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(opNum);
  if (MO.isImm())
    O << (unsigned short int)MO.getImm();
  else
    printOperand(MI, opNum, O);
}

void CoffeeAsmPrinter::
printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // Load/Store memory operands -- imm($reg)
  // If PIC target the target is loaded as the
  // pattern lw $25,%call16($28)
  printOperand(MI, opNum+1, O);
  O << "(";
  printOperand(MI, opNum, O);
  O << ")";
}

void CoffeeAsmPrinter::
printMemOperandEA(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // when using stack locations for not load/store instructions
  // print the same way as all normal 3 operand instructions.
  printOperand(MI, opNum, O);
  O << ", ";
  printOperand(MI, opNum+1, O);
  return;
}

void CoffeeAsmPrinter::
printFCCOperand(const MachineInstr *MI, int opNum, raw_ostream &O,
                const char *Modifier) {
    llvm_unreachable("coffee: printFCCOperand");
    //const MachineOperand& MO = MI->getOperand(opNum);
  //O << Coffee::CoffeeFCCToString((Coffee::CondCode)MO.getImm());
}

void CoffeeAsmPrinter::EmitStartOfAsmFile(Module &M) {
  // FIXME: Use SwitchSection.

  // Tell the assembler which ABI we are using
  if (OutStreamer.hasRawTextSupport())
    OutStreamer.EmitRawText("\t.section .mdebug." +
                            Twine(getCurrentABIString()));

  // TODO: handle O64 ABI
  if (OutStreamer.hasRawTextSupport()) {


        OutStreamer.EmitRawText(StringRef("\t.section .gcc_compiled_long32"));


  }

  // return to previous section
  if (OutStreamer.hasRawTextSupport())
    OutStreamer.EmitRawText(StringRef("\t.previous"));
}

MachineLocation
CoffeeAsmPrinter::getDebugValueLocation(const MachineInstr *MI) const {
  // Handles frame addresses emitted in CoffeeInstrInfo::emitFrameIndexDebugValue.
  assert(MI->getNumOperands() == 4 && "Invalid no. of machine operands!");
  assert(MI->getOperand(0).isReg() && MI->getOperand(1).isImm() &&
         "Unexpected MachineOperand types");
  return MachineLocation(MI->getOperand(0).getReg(),
                         MI->getOperand(1).getImm());
}

void CoffeeAsmPrinter::PrintDebugValueComment(const MachineInstr *MI,
                                           raw_ostream &OS) {
  // TODO: implement
}

// Force static initialization.
extern "C" void LLVMInitializeCoffeeAsmPrinter() {
  RegisterAsmPrinter<CoffeeAsmPrinter> X(TheCoffeeTarget);

}
