//===-- Coffee.h - Top-level interface for Coffee Target ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Coffee back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_Coffee_H
#define LLVM_TARGET_Coffee_H

#include "MCTargetDesc/CoffeeBaseInfo.h"
#include "MCTargetDesc/CoffeeMCTargetDesc.h"
#include <string>

// GCC #defines Coffee on Linux but we use it as our namespace name
#undef Coffee

namespace llvm {
  class CoffeeTargetMachine;
  class CoffeeAsmPrinter;
  class FunctionPass;
  class JITCodeEmitter;
  class MachineInstr;
  class AsmPrinter;
  class MCInst;

  FunctionPass *createCoffeeBranchSelectionPass();
  FunctionPass *createCoffeeISelDag(CoffeeTargetMachine &TM);
  FunctionPass *createCoffeeDelaySlotFillerPass(CoffeeTargetMachine &TM);
  FunctionPass *createCoffeeJITCodeEmitterPass(CoffeeTargetMachine &TM,
                                            JITCodeEmitter &MCE);
  void LowerCoffeeMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    CoffeeAsmPrinter &AP);
  
 /* namespace CoffeeII {
    
  /// Target Operand Flag enum.
  enum TOF {
    //===------------------------------------------------------------------===//
    // Coffee Specific MachineOperand flags.
    MO_NO_FLAG,
    MO_ABS_HI,

    MO_ABS_LO,
    
    /// MO_DARWIN_STUB - On a symbol operand "FOO", this indicates that the
    /// reference is actually to the "FOO$stub" symbol.  This is used for calls
    /// and jumps to external functions on Tiger and earlier.
    //MO_DARWIN_STUB = 1,
    
    /// MO_LO16, MO_HA16 - lo16(symbol) and ha16(symbol)
    MO_LO16 = 4, MO_HA16 = 8,

    /// MO_PIC_FLAG - If this bit is set, the symbol reference is relative to
    /// the function's picbase, e.g. lo16(symbol-picbase).
    MO_PIC_FLAG = 16,

    /// MO_NLP_FLAG - If this bit is set, the symbol reference is actually to
    /// the non_lazy_ptr for the global, e.g. lo16(symbol$non_lazy_ptr-picbase).
    MO_NLP_FLAG = 32,
    
    /// MO_NLP_HIDDEN_FLAG - If this bit is set, the symbol reference is to a
    /// symbol with hidden visibility.  This causes a different kind of
    /// non-lazy-pointer to be generated.
    MO_NLP_HIDDEN_FLAG = 64
  };
  } // end namespace CoffeeII*/
  
} // end namespace llvm;

#endif
