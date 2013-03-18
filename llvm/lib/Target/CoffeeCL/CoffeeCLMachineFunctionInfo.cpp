//===-- CoffeeCLMachineFunctionInfo.cpp - Private data used for CoffeeCL --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLMachineFunctionInfo.h"
#include "CoffeeCLInstrInfo.h"

#include "llvm/Function.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

void CoffeeCLFunctionInfo::anchor() { }

static cl::opt<bool>
FixGlobalBaseReg("coffeecl-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));


bool CoffeeCLFunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

unsigned CoffeeCLFunctionInfo::getGlobalBaseReg() {
  // Return if it has already been initialized.
  if (GlobalBaseReg)
    return GlobalBaseReg;

  const TargetRegisterClass *RC = &CoffeeCL::GPRCRegClass;
  return GlobalBaseReg = MF.getRegInfo().createVirtualRegister(RC);
}
