//===-- CoffeeMachineFunctionInfo.h - Private data used for Coffee ----*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Coffee specific subclass of MachineFunctionInfo.
//
//===----------------------------------------------------------------------===//

#ifndef Coffee_MACHINE_FUNCTION_INFO_H
#define Coffee_MACHINE_FUNCTION_INFO_H

#include "CoffeeSubtarget.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include <utility>

namespace llvm {

/// CoffeeFunctionInfo - This class is derived from MachineFunction private
/// Coffee target-specific information for each MachineFunction.
class CoffeeFunctionInfo : public MachineFunctionInfo {
  virtual void anchor();

  MachineFunction& MF;
  /// SRetReturnReg - Some subtargets require that sret lowering includes
  /// returning the value of the returned struct in a register. This field
  /// holds the virtual register into which the sret argument is passed.
  unsigned SRetReturnReg;

  /// GlobalBaseReg - keeps track of the virtual register initialized for
  /// use as the global base register. This is used for PIC in some PIC
  /// relocation models.
  unsigned GlobalBaseReg;



  /// VarArgsFrameIndex - FrameIndex for start of varargs area.
  int VarArgsFrameIndex;

  /// True if function has a byval argument.
  bool HasByvalArg;

  /// Size of incoming argument area.
  unsigned IncomingArgSize;

public:
  CoffeeFunctionInfo(MachineFunction& MF)
   : MF(MF), SRetReturnReg(0), GlobalBaseReg(0),
     VarArgsFrameIndex(0)
  {}

  unsigned getSRetReturnReg() const { return SRetReturnReg; }
  void setSRetReturnReg(unsigned Reg) { SRetReturnReg = Reg; }

  bool globalBaseRegSet() const;
  unsigned getGlobalBaseReg();



  int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
  void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

  bool hasByvalArg() const { return HasByvalArg; }
  void setFormalArgInfo(unsigned Size, bool HasByval) {
    IncomingArgSize = Size;
    HasByvalArg = HasByval;
  }

  unsigned getIncomingArgSize() const { return IncomingArgSize; }
};

} // end of namespace llvm

#endif // Coffee_MACHINE_FUNCTION_INFO_H
