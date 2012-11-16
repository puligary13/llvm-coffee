//===-- CoffeeMCInstLower.h - Lower MachineInstr to MCInst -------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeMCINSTLOWER_H
#define CoffeeMCINSTLOWER_H
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/Support/Compiler.h"

namespace llvm {
  class MCContext;
  class MCInst;
  class MCOperand;
  class MachineInstr;
  class MachineFunction;
  class Mangler;
  class CoffeeAsmPrinter;

/// CoffeeMCInstLower - This class is used to lower an MachineInstr into an
//                    MCInst.
class LLVM_LIBRARY_VISIBILITY CoffeeMCInstLower {
  typedef MachineOperand::MachineOperandType MachineOperandType;
  MCContext *Ctx;
  Mangler *Mang;
  CoffeeAsmPrinter &AsmPrinter;
public:
  CoffeeMCInstLower(CoffeeAsmPrinter &asmprinter);
  void Initialize(Mangler *mang, MCContext* C);
  void Lower(const MachineInstr *MI, MCInst &OutMI) const;
  void LowerCPLOAD(SmallVector<MCInst, 4>& MCInsts);
  void LowerCPRESTORE(int64_t Offset, SmallVector<MCInst, 4>& MCInsts);
  void LowerUnalignedLoadStore(const MachineInstr *MI,
                               SmallVector<MCInst, 4>& MCInsts);
  void LowerSETGP01(const MachineInstr *MI, SmallVector<MCInst, 4>& MCInsts);
private:
  MCOperand LowerSymbolOperand(const MachineOperand &MO,
                               MachineOperandType MOTy, unsigned Offset) const;
  MCOperand LowerOperand(const MachineOperand& MO, unsigned offset = 0) const;
};
}

#endif
