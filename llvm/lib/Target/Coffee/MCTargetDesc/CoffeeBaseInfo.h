//===-- CoffeeBaseInfo.h - Top level definitions for Coffee MC ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the Coffee target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef CoffeeBASEINFO_H
#define CoffeeBASEINFO_H

#include "CoffeeFixupKinds.h"
#include "CoffeeMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// CoffeeII - This namespace holds all of the target specific flags that
/// instruction info tracks.
///
namespace CoffeeII {
  /// Target Operand Flag enum.
  enum TOF {
    //===------------------------------------------------------------------===//
    // Coffee Specific MachineOperand flags.

    MO_NO_FLAG,

    /// MO_GOT16 - Represents the offset into the global offset table at which
    /// the address the relocation entry symbol resides during execution.
    MO_GOT16,
    MO_GOT,

    /// MO_GOT_CALL - Represents the offset into the global offset table at
    /// which the address of a call site relocation entry symbol resides
    /// during execution. This is different from the above since this flag
    /// can only be present in call instructions.
    MO_GOT_CALL,

    /// MO_GPREL - Represents the offset from the current gp value to be used
    /// for the relocatable object file being produced.
    MO_GPREL,

    /// MO_ABS_HI/LO - Represents the hi or low part of an absolute symbol
    /// address.
    MO_ABS_HI,
    MO_ABS_LO,

    /// MO_TLSGD - Represents the offset into the global offset table at which
    // the module ID and TSL block offset reside during execution (General
    // Dynamic TLS).
    MO_TLSGD,

    /// MO_TLSLDM - Represents the offset into the global offset table at which
    // the module ID and TSL block offset reside during execution (Local
    // Dynamic TLS).
    MO_TLSLDM,
    MO_DTPREL_HI,
    MO_DTPREL_LO,

    /// MO_GOTTPREL - Represents the offset from the thread pointer (Initial
    // Exec TLS).
    MO_GOTTPREL,

    /// MO_TPREL_HI/LO - Represents the hi and low part of the offset from
    // the thread pointer (Local Exec TLS).
    MO_TPREL_HI,
    MO_TPREL_LO,

    // N32/64 Flags.
    MO_GPOFF_HI,
    MO_GPOFF_LO,
    MO_GOT_DISP,
    MO_GOT_PAGE,
    MO_GOT_OFST,

    /// MO_HIGHER/HIGHEST - Represents the highest or higher half word of a
    /// 64-bit symbol address.
    MO_HIGHER,
    MO_HIGHEST
  };

  enum {
    //===------------------------------------------------------------------===//
    // Instruction encodings.  These are the standard/most common forms for
    // Coffee instructions.
    //

    // Pseudo - This represents an instruction that is a pseudo instruction
    // or one that has not been implemented yet.  It is illegal to code generate
    // it, but tolerated for intermediate implementation stages.
    Pseudo   = 0,

    /// FrmR - This form is for instructions of the format R.
    FrmR  = 1,
    /// FrmI - This form is for instructions of the format I.
    FrmI  = 2,
    /// FrmJ - This form is for instructions of the format J.
    FrmJ  = 3,
    /// FrmFR - This form is for instructions of the format FR.
    FrmFR = 4,
    /// FrmFI - This form is for instructions of the format FI.
    FrmFI = 5,
    /// FrmOther - This form is for instructions that have no specific format.
    FrmOther = 6,

    FormMask = 15
  };
}


/// getCoffeeRegisterNumbering - Given the enum value for some register,
/// return the number that it corresponds to.
inline static unsigned getCoffeeRegisterNumbering(unsigned RegEnum)
{
  switch (RegEnum) {
  case Coffee::T0: case Coffee::CR0:
    return 0;
  case Coffee::T1: case Coffee::CR1:
    return 1;
  case Coffee::T2: case Coffee::CR2:
    return 2;
  case Coffee::T3: case Coffee::CR3:
    return 3;
  case Coffee::T4: case Coffee::CR4:
    return 4;
  case Coffee::T5: case Coffee::CR5:
    return 5;
  case Coffee::T6: case Coffee::CR6:
    return 6;
  case Coffee::T7: case Coffee::CR7:
    return 7;
  case Coffee::T8:
    return 8;
  case Coffee::T9:
    return 9;
  case Coffee::S0:
    return 10;
  case Coffee::S1:
    return 11;
  case Coffee::S2:
    return 12;
  case Coffee::S3:
    return 13;
  case Coffee::S4:
    return 14;
  case Coffee::S5:
    return 15;
  case Coffee::S6:
    return 16;
  case Coffee::S7:
    return 17;
  case Coffee::S8:
    return 18;
  case Coffee::S9:
    return 19;
  case Coffee::S10:
    return 20;
  case Coffee::S11:
    return 21;
  case Coffee::A0:
    return 22;
  case Coffee::A1:
    return 23;
  case Coffee::A2:
    return 24;
  case Coffee::A3:
    return 25;
  case Coffee::V0:
    return 26;
  case Coffee::V1:
    return 27;
  case Coffee::GP:
    return 28;
  case Coffee::SP:
    return 29;
  case Coffee::FP:
    return 30;
  case Coffee::LR:
    return 31;
  default: llvm_unreachable("Unknown register number!");
  }
}

inline static std::pair<const MCSymbolRefExpr*, int64_t>
CoffeeGetSymAndOffset(const MCFixup &Fixup) {
  MCFixupKind FixupKind = Fixup.getKind();

  if ((FixupKind < FirstTargetFixupKind) ||
      (FixupKind >= MCFixupKind(Coffee::LastTargetFixupKind)))
    return std::make_pair((const MCSymbolRefExpr*)0, (int64_t)0);

  const MCExpr *Expr = Fixup.getValue();
  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    const MCBinaryExpr *BE = static_cast<const MCBinaryExpr*>(Expr);
    const MCExpr *LHS = BE->getLHS();
    const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(BE->getRHS());

    if ((LHS->getKind() != MCExpr::SymbolRef) || !CE)
      return std::make_pair((const MCSymbolRefExpr*)0, (int64_t)0);

    return std::make_pair(cast<MCSymbolRefExpr>(LHS), CE->getValue());
  }

  if (Kind != MCExpr::SymbolRef)
    return std::make_pair((const MCSymbolRefExpr*)0, (int64_t)0);

  return std::make_pair(cast<MCSymbolRefExpr>(Expr), 0);
}
}

#endif
