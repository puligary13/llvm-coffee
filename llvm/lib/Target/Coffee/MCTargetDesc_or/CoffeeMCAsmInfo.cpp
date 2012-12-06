//===-- CoffeeMCAsmInfo.cpp - Coffee asm properties -----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MCAsmInfoDarwin properties.
//
//===----------------------------------------------------------------------===//

#include "CoffeeMCAsmInfo.h"
using namespace llvm;

void CoffeeLinuxMCAsmInfo::anchor() { }

CoffeeLinuxMCAsmInfo::CoffeeLinuxMCAsmInfo() {
    IsLittleEndian = false;

    AlignmentIsInBytes          = false;
    Data16bitsDirective         = "\t.2byte\t";
    Data32bitsDirective         = "\t.4byte\t";
    Data64bitsDirective         = "\t.8byte\t";
    PrivateGlobalPrefix         = "L";  //todo: change this to # or $ adds " or ( around the label, need to check why in future when needed
    CommentString               = "#";
    ZeroDirective               = "\t.space\t";
    GPRel32Directive            = "\t.gpword\t";
    GPRel64Directive            = "\t.gpdword\t";
    WeakRefDirective            = "\t.weak\t";

    SupportsDebugInformation = true;
    ExceptionsType = ExceptionHandling::DwarfCFI;
    HasLEB128 = true;
    DwarfRegNumForCFI = true;
}

