//===-- CoffeeCLMCAsmInfo.cpp - CoffeeCL Asm Properties ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the CoffeeCLMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "CoffeeCLMCAsmInfo.h"
#include "llvm/ADT/Triple.h"

using namespace llvm;

void CoffeeCLMCAsmInfo::anchor() { }

CoffeeCLMCAsmInfo::CoffeeCLMCAsmInfo(const Target &T, StringRef TT) {
  Triple TheTriple(TT);
  if ((TheTriple.getArch() == Triple::coffeecl))
    IsLittleEndian = false;

  AlignmentIsInBytes          = false;
  Data16bitsDirective         = "\t.2byte\t";
  Data32bitsDirective         = "\t.4byte\t";
  Data64bitsDirective         = "\t.8byte\t";
  PrivateGlobalPrefix         = "$";
  CommentString               = ";";
  ZeroDirective               = "\t.space\t";
  GPRel32Directive            = "\t.gpword\t";
  GPRel64Directive            = "\t.gpdword\t";
  WeakRefDirective            = "\t.weak\t";

  SupportsDebugInformation = true;
  ExceptionsType = ExceptionHandling::DwarfCFI;
  HasLEB128 = true;
  DwarfRegNumForCFI = true;
}
