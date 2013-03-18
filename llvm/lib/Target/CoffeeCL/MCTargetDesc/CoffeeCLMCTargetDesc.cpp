//===-- CoffeeCLMCTargetDesc.cpp - CoffeeCL Target Descriptions -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides CoffeeCL specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLMCAsmInfo.h"
#include "CoffeeCLMCTargetDesc.h"
#include "InstPrinter/CoffeeCLInstPrinter.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "CoffeeCLGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "CoffeeCLGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "CoffeeCLGenRegisterInfo.inc"

using namespace llvm;

static std::string ParseCoffeeCLTriple(StringRef TT, StringRef CPU) {
  std::string CoffeeCLArchFeature;
  size_t DashPosition = 0;
  StringRef TheTriple;

  // Let's see if there is a dash, like CoffeeCL-unknown-linux.
  DashPosition = TT.find('-');

  if (DashPosition == StringRef::npos) {
    // No dash, we check the string size.
    TheTriple = TT.substr(0);
  } else {
    // We are only interested in substring before dash.
    TheTriple = TT.substr(0,DashPosition);
  }

  if (TheTriple == "CoffeeCL") {
    if (CPU.empty() || CPU == "CoffeeCL32") {
      CoffeeCLArchFeature = "+CoffeeCL32";
    }
  }
  return CoffeeCLArchFeature;
}

static MCInstrInfo *createCoffeeCLMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitCoffeeCLMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createCoffeeCLMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitCoffeeCLMCRegisterInfo(X, CoffeeCL::LR);
  return X;
}

static MCSubtargetInfo *createCoffeeCLMCSubtargetInfo(StringRef TT, StringRef CPU,
                                                  StringRef FS) {
  std::string ArchFS = ParseCoffeeCLTriple(TT,CPU);
  if (!FS.empty()) {
    if (!ArchFS.empty())
      ArchFS = ArchFS + "," + FS.str();
    else
      ArchFS = FS;
  }
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitCoffeeCLMCSubtargetInfo(X, TT, CPU, ArchFS);
  return X;
}

static MCAsmInfo *createCoffeeCLMCAsmInfo(const Target &T, StringRef TT) {
  MCAsmInfo *MAI = new CoffeeCLMCAsmInfo(T, TT);

  MachineLocation Dst(MachineLocation::VirtualFP);
  MachineLocation Src(CoffeeCL::SP, 0);
  MAI->addInitialFrameState(0, Dst, Src);

  return MAI;
}

static MCCodeGenInfo *createCoffeeCLMCCodeGenInfo(StringRef TT, Reloc::Model RM,
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

static MCInstPrinter *createCoffeeCLMCInstPrinter(const Target &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI,
                                              const MCSubtargetInfo &STI) {
  return new CoffeeCLInstPrinter(MAI, MII, MRI);
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

extern "C" void LLVMInitializeCoffeeCLTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn X(TheCoffeeCLTarget, createCoffeeCLMCAsmInfo);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheCoffeeCLTarget,
                                        createCoffeeCLMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheCoffeeCLTarget, createCoffeeCLMCInstrInfo);


  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheCoffeeCLTarget, createCoffeeCLMCRegisterInfo);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(TheCoffeeCLTarget,
                                        createCoffeeCLMCCodeEmitter);

  // Register the object streamer.
  TargetRegistry::RegisterMCObjectStreamer(TheCoffeeCLTarget, createMCStreamer);;

  // Register the asm backend.
  TargetRegistry::RegisterMCAsmBackend(TheCoffeeCLTarget,
                                       createCoffeeCLAsmBackend);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheCoffeeCLTarget,
                                          createCoffeeCLMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(TheCoffeeCLTarget,
                                        createCoffeeCLMCInstPrinter);
}
