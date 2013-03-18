//===-- CoffeeCLSubtarget.cpp - CoffeeCL Subtarget Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeCL specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLSubtarget.h"
#include "CoffeeCL.h"
#include "CoffeeCLRegisterInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "CoffeeCLGenSubtargetInfo.inc"

using namespace llvm;

void CoffeeCLSubtarget::anchor() { }

CoffeeCLSubtarget::CoffeeCLSubtarget(const std::string &TT, const std::string &CPU,
                             const std::string &FS, bool little,
                             Reloc::Model RM) :
  CoffeeCLGenSubtargetInfo(TT, CPU, FS),
  CoffeeCLArchVersion(CoffeeCL32), CoffeeCLABI(UnknownABI), IsLittle(little),
  IsSingleFloat(false), IsFP64bit(false), IsGP64bit(false), HasVFPU(false),
  IsLinux(true), HasSEInReg(false), HasCondMov(false), HasMulDivAdd(false),
  HasMinMax(false), HasSwap(false), HasBitCount(false), HasFPIdx(false),
  InCoffeeCL16Mode(false), HasDSP(false), HasDSPR2(false), IsAndroid(false),
  IsCoProcessor0(false), IsCoProcessor1(true), IsCoProcessor2(false),
  IsCoProcessor3(false)
{
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "coffeecl32";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);

  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  // Set CoffeeCLABI if it hasn't been set yet.
  if (CoffeeCLABI == UnknownABI)
    CoffeeCLABI = ABI32;

  // Check if Architecture and ABI are compatible.


  // Is the target system Linux ?
  if (TT.find("linux") == std::string::npos && TT.find("coffeeclcl") == std::string::npos) {
      llvm_unreachable("coffeecl: we only support linux or opencl");
  }

  // Set UseSmallSection.
  UseSmallSection = !IsLinux && (RM == Reloc::Static);
}

bool
CoffeeCLSubtarget::enablePostRAScheduler(CodeGenOpt::Level OptLevel,
                                    TargetSubtargetInfo::AntiDepBreakMode &Mode,
                                     RegClassVector &CriticalPathRCs) const {
  Mode = TargetSubtargetInfo::ANTIDEP_NONE;
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(&CoffeeCL::GPRCRegClass);
  return OptLevel >= CodeGenOpt::Aggressive;
}
