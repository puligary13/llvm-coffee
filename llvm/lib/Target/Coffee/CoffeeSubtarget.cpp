//===-- CoffeeSubtarget.cpp - Coffee Subtarget Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Coffee specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "CoffeeSubtarget.h"
#include "Coffee.h"
#include "CoffeeRegisterInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "CoffeeGenSubtargetInfo.inc"

using namespace llvm;

void CoffeeSubtarget::anchor() { }

CoffeeSubtarget::CoffeeSubtarget(const std::string &TT, const std::string &CPU,
                             const std::string &FS, bool little,
                             Reloc::Model RM) :
  CoffeeGenSubtargetInfo(TT, CPU, FS),
  CoffeeArchVersion(Coffee32), CoffeeABI(UnknownABI), IsLittle(little),
  IsSingleFloat(false), IsFP64bit(false), IsGP64bit(false), HasVFPU(false),
  IsLinux(true), HasSEInReg(false), HasCondMov(false), HasMulDivAdd(false),
  HasMinMax(false), HasSwap(false), HasBitCount(false), HasFPIdx(false),
  InCoffee16Mode(false), HasDSP(false), HasDSPR2(false), IsAndroid(false),
  IsCoProcessor0(false), IsCoProcessor1(true), IsCoProcessor2(false),
  IsCoProcessor3(false)
{
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "coffee32";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);

  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  // Set CoffeeABI if it hasn't been set yet.
  if (CoffeeABI == UnknownABI)
    CoffeeABI = ABI32;

  // Check if Architecture and ABI are compatible.


  // Is the target system Linux ?
  if (TT.find("linux") == std::string::npos) {
      llvm_unreachable("coffee: we only support linux");
  }

  // Set UseSmallSection.
  UseSmallSection = !IsLinux && (RM == Reloc::Static);
}

bool
CoffeeSubtarget::enablePostRAScheduler(CodeGenOpt::Level OptLevel,
                                    TargetSubtargetInfo::AntiDepBreakMode &Mode,
                                     RegClassVector &CriticalPathRCs) const {
  Mode = TargetSubtargetInfo::ANTIDEP_NONE;
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(&Coffee::GPRCRegClass);
  return OptLevel >= CodeGenOpt::Aggressive;
}
