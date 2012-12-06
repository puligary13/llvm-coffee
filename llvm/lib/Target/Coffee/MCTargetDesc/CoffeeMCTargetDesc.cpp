//===-- CoffeeMCTargetDesc.cpp - Coffee Target Descriptions -------------------===//
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

#include "CoffeeMCAsmInfo.h"
#include "CoffeeMCTargetDesc.h"
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

#define GET_SUBTARGETINFO_MC_DESC
#include "CoffeeGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "CoffeeGenRegisterInfo.inc"

using namespace llvm;

static std::string ParseCoffeeTriple(StringRef TT, StringRef CPU) {
  std::string CoffeeArchFeature;
  size_t DashPosition = 0;
  StringRef TheTriple;

  // Let's see if there is a dash, like Coffee-unknown-linux.
  DashPosition = TT.find('-');

  if (DashPosition == StringRef::npos) {
    // No dash, we check the string size.
    TheTriple = TT.substr(0);
  } else {
    // We are only interested in substring before dash.
    TheTriple = TT.substr(0,DashPosition);
  }

  if (TheTriple == "Coffee") {
    if (CPU.empty() || CPU == "Coffee32") {
      CoffeeArchFeature = "+Coffee32";
    }
  }
  return CoffeeArchFeature;
}

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

static MCSubtargetInfo *createCoffeeMCSubtargetInfo(StringRef TT, StringRef CPU,
                                                  StringRef FS) {
  std::string ArchFS = ParseCoffeeTriple(TT,CPU);
  if (!FS.empty()) {
    if (!ArchFS.empty())
      ArchFS = ArchFS + "," + FS.str();
    else
      ArchFS = FS;
  }
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitCoffeeMCSubtargetInfo(X, TT, CPU, ArchFS);
  return X;
}

static MCAsmInfo *createCoffeeMCAsmInfo(const Target &T, StringRef TT) {
  MCAsmInfo *MAI = new CoffeeMCAsmInfo(T, TT);

  MachineLocation Dst(MachineLocation::VirtualFP);
  MachineLocation Src(Coffee::SP, 0);
  MAI->addInitialFrameState(0, Dst, Src);

  return MAI;
}

static MCCodeGenInfo *createCoffeeMCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                              CodeModel::Model CM,
                                              CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();
  if (CM == CodeModel::JITDefault)
    RM = Reloc::Static;
  else if (RM == Reloc::Default)
   // RM = Reloc::PIC_;
      RM = Reloc::Static; // guoqing: modified for trial
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
                                    raw_ostream &_OS,
                                    MCCodeEmitter *_Emitter,
                                    bool RelaxAll,
                                    bool NoExecStack) {
  Triple TheTriple(TT);

  return createELFStreamer(Ctx, MAB, _OS, _Emitter, RelaxAll, NoExecStack);
}

extern "C" void LLVMInitializeCoffeeTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn X(TheCoffeeTarget, createCoffeeMCAsmInfo);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheCoffeeTarget,
                                        createCoffeeMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheCoffeeTarget, createCoffeeMCInstrInfo);


  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheCoffeeTarget, createCoffeeMCRegisterInfo);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(TheCoffeeTarget,
                                        createCoffeeMCCodeEmitter);

  // Register the object streamer.
  TargetRegistry::RegisterMCObjectStreamer(TheCoffeeTarget, createMCStreamer);;

  // Register the asm backend.
  TargetRegistry::RegisterMCAsmBackend(TheCoffeeTarget,
                                       createCoffeeAsmBackend);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheCoffeeTarget,
                                          createCoffeeMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(TheCoffeeTarget,
                                        createCoffeeMCInstPrinter);
}
