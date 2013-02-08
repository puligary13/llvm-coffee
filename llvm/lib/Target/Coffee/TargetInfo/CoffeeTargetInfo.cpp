//===-- CoffeeTargetInfo.cpp - Coffee Target Implementation -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Coffee.h"
#include "llvm/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheCoffeeTarget;

extern "C" void LLVMInitializeCoffeeTargetInfo() {
  RegisterTarget<Triple::coffee, /*HasJIT=*/false>
    X(TheCoffeeTarget, "Coffee", "Coffee");
  RegisterTarget<Triple::coffeecl, /*HasJIT=*/false>
    Y(TheCoffeeTarget, "Coffee", "Coffee");
}
