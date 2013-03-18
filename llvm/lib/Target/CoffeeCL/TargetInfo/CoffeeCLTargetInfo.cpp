//===-- CoffeeCLTargetInfo.cpp - CoffeeCL Target Implementation -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCL.h"
#include "llvm/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheCoffeeCLTarget;

extern "C" void LLVMInitializeCoffeeCLTargetInfo() {
  RegisterTarget<Triple::coffeecl, /*HasJIT=*/false>
    X(TheCoffeeCLTarget, "CoffeeCL", "CoffeeCL");
}
