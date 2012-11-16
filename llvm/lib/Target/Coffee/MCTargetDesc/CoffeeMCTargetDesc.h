//===-- CoffeeMCTargetDesc.h - Coffee Target Descriptions ---------*- C++ -*-===//
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

#ifndef CoffeeMCTARGETDESC_H
#define CoffeeMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCSubtargetInfo;
class MCRegisterInfo;
class Target;
class StringRef;
class raw_ostream;

extern Target TheCoffeeTarget;
  
MCCodeEmitter *createCoffeeMCCodeEmitter(const MCInstrInfo &MCII,
                                          const MCRegisterInfo &MRI,
                                      const MCSubtargetInfo &STI,
                                      MCContext &Ctx);

MCAsmBackend *createCoffeeAsmBackend(const Target &T, StringRef TT, StringRef CPU);

/// createCoffeeELFObjectWriter - Construct an Coffee ELF object writer.
MCObjectWriter *createCoffeeELFObjectWriter(raw_ostream &OS,
                                         uint8_t OSABI);
} // End llvm namespace

// Defines symbolic names for Coffee registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "CoffeeGenRegisterInfo.inc"

// Defines symbolic names for the Coffee instructions.
//
#define GET_INSTRINFO_ENUM
#include "CoffeeGenInstrInfo.inc"

#endif
