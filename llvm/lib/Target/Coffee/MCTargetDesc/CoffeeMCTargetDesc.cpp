//===-- CoffeeMCTargetDesc.cpp - Coffee Target Descriptions -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Coffee specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "CoffeeMCTargetDesc.h"
#include "CoffeeMCAsmInfo.h"
#include "InstPrinter/CoffeeInstPrinter.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "CoffeeGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "CoffeeGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createCoffeeMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitCoffeeMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createCoffeeMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitCoffeeMCRegisterInfo(X, Coffee::LR);
  return X;
}


static MCAsmInfo *createCoffeeMCAsmInfo(const Target &T, StringRef TT) {

  MCAsmInfo *MAI = new CoffeeLinuxMCAsmInfo();

  // Initial state of the frame pointer is R1.
  MachineLocation Dst(MachineLocation::VirtualFP);
  MachineLocation Src(Coffee::SP, 0);
  MAI->addInitialFrameState(0, Dst, Src);

  return MAI;
}

static MCCodeGenInfo *createCoffeeMCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                             CodeModel::Model CM,
                                             CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();

  if (RM == Reloc::Default) {
     RM = Reloc::Static;
  }
  X->InitMCCodeGenInfo(RM, CM, OL);
  return X;
}

static MCInstPrinter *createCoffeeMCInstPrinter(const Target &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI,
                                             const MCSubtargetInfo &STI) {
  return new CoffeeInstPrinter(MAI, MII, MRI);
}


static MCStreamer *createMCStreamer(const Target &T, StringRef TT,
                                    MCContext &Ctx, MCAsmBackend &MAB,
                                    raw_ostream &OS,
                                    MCCodeEmitter *Emitter,
                                    bool RelaxAll,
                                    bool NoExecStack) {
  return createELFStreamer(Ctx, MAB, OS, Emitter, RelaxAll, NoExecStack);
}


extern "C" void LLVMInitializeCoffeeTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn C(TheCoffeeTarget, createCoffeeMCAsmInfo);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheCoffeeTarget, createCoffeeMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheCoffeeTarget, createCoffeeMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheCoffeeTarget, createCoffeeMCRegisterInfo);

  // Register the MC subtarget info.
  // no subtarget

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(TheCoffeeTarget, createCoffeeMCCodeEmitter);
  
    // Register the asm backend.
  TargetRegistry::RegisterMCAsmBackend(TheCoffeeTarget, createCoffeeAsmBackend);
  
  // Register the object streamer.
  TargetRegistry::RegisterMCObjectStreamer(TheCoffeeTarget, createMCStreamer);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(TheCoffeeTarget, createCoffeeMCInstPrinter);
}
