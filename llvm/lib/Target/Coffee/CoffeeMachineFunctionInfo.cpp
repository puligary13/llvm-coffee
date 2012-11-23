//===-- CoffeeMachineFunctionInfo.cpp - Private data used for Coffee --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "CoffeeMachineFunctionInfo.h"
#include "CoffeeInstrInfo.h"

#include "llvm/Function.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

void CoffeeFunctionInfo::anchor() { }

static cl::opt<bool>
FixGlobalBaseReg("coffee-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));


bool CoffeeFunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

unsigned CoffeeFunctionInfo::getGlobalBaseReg() {
  // Return if it has already been initialized.
  if (GlobalBaseReg)
    return GlobalBaseReg;

  const TargetRegisterClass *RC = &Coffee::GPRCRegClass;
  return GlobalBaseReg = MF.getRegInfo().createVirtualRegister(RC);
}
