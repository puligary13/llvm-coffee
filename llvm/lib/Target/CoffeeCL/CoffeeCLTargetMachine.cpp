//===-- CoffeeCLTargetMachine.cpp - Define TargetMachine for CoffeeCL -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Top-level implementation for the CoffeeCL target.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLTargetMachine.h"
#include "CoffeeCLPass.h"
#include "CoffeeCL.h"
#include "llvm/PassManager.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializeCoffeeCLTarget() {
  // Register the targets
  RegisterTargetMachine<CoffeeCLTargetMachine> A(TheCoffeeCLTarget);
}

CoffeeCLTargetMachine::CoffeeCLTargetMachine(const Target &T, StringRef TT,
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

void CoffeeCLTargetMachine::anchor() {
}


//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
class CoffeeCLPassConfig : public TargetPassConfig {
public:
    CoffeeCLPassConfig(CoffeeCLTargetMachine *TM, PassManagerBase &PM)
        : TargetPassConfig(TM, PM) {}

    CoffeeCLTargetMachine &getCoffeeCLTargetMachine() const {
        return getTM<CoffeeCLTargetMachine>();
    }

    virtual bool addInstSelector();

    virtual bool addPreEmitPass();
};
} // namespace

TargetPassConfig *CoffeeCLTargetMachine::createPassConfig(PassManagerBase &PM) {
    return new CoffeeCLPassConfig(this, PM);
}

bool CoffeeCLPassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createCoffeeCLPass());
  addPass(createCoffeeCLISelDag(getCoffeeCLTargetMachine()));
  return false;
}

bool CoffeeCLPassConfig::addPreEmitPass() {
  addPass(createCoffeeCLDelaySlotFillerPass(getCoffeeCLTargetMachine()));
  return true;
}

//end of file
