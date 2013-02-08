//===-- CoffeeTargetMachine.cpp - Define TargetMachine for Coffee -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Top-level implementation for the Coffee target.
//
//===----------------------------------------------------------------------===//

#include "CoffeeTargetMachine.h"
#include "Coffee.h"
#include "llvm/PassManager.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializeCoffeeTarget() {
  // Register the targets
  RegisterTargetMachine<CoffeeTargetMachine> A(TheCoffeeTarget);
}

CoffeeTargetMachine::CoffeeTargetMachine(const Target &T, StringRef TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Reloc::Model RM, CodeModel::Model CM,
                                   CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
    Subtarget(TT, CPU, FS, false, RM),  //is Big Endian
    //DL("E-p:32:32:32-i1:32:32-i8:32:32-i16:32:32-i32:32:32-f32:32:32-n32-S32"),
    DL("E-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-"
       "f32:32:32-f64:64:64-v16:16:16-v32:32:32-v64:64:64-v128:128:128-"
       "n16:32:64"),
    InstrInfo(*this),
    FrameLowering(),
    TLInfo(*this),
    TSInfo(*this) {

}

void CoffeeTargetMachine::anchor() {
}


//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
class CoffeePassConfig : public TargetPassConfig {
public:
    CoffeePassConfig(CoffeeTargetMachine *TM, PassManagerBase &PM)
        : TargetPassConfig(TM, PM) {}

    CoffeeTargetMachine &getCoffeeTargetMachine() const {
        return getTM<CoffeeTargetMachine>();
    }

    virtual bool addInstSelector();

    virtual bool addPreEmitPass();
};
} // namespace

TargetPassConfig *CoffeeTargetMachine::createPassConfig(PassManagerBase &PM) {
    return new CoffeePassConfig(this, PM);
}

bool CoffeePassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createCoffeeISelDag(getCoffeeTargetMachine()));
  return false;
}

bool CoffeePassConfig::addPreEmitPass() {
  addPass(createCoffeeDelaySlotFillerPass(getCoffeeTargetMachine()));
  return true;
}

//end of file
