//===-- CoffeeCLMCTargetDesc.h - CoffeeCL Target Descriptions -----------*- C++ -*-===//
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

#ifndef CoffeeCLMCTARGETDESC_H
#define CoffeeCLMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class StringRef;
class Target;
class raw_ostream;

extern Target TheCoffeeCLTarget;

MCCodeEmitter *createCoffeeCLMCCodeEmitter(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         const MCSubtargetInfo &STI,
                                         MCContext &Ctx);

MCAsmBackend *createCoffeeCLAsmBackend(const Target &T, StringRef TT,
                                       StringRef CPU);


MCObjectWriter *createCoffeeCLELFObjectWriter(raw_ostream &OS,
                                          uint8_t OSABI,
                                          bool IsLittleEndian,
                                          bool Is64Bit);
} // End llvm namespace

// Defines symbolic names for CoffeeCL registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "CoffeeCLGenRegisterInfo.inc"

// Defines symbolic names for the CoffeeCL instructions.
#define GET_INSTRINFO_ENUM
#include "CoffeeCLGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "CoffeeCLGenSubtargetInfo.inc"

#endif
