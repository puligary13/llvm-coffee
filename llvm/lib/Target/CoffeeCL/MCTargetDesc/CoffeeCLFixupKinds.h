//===-- MipsFixupKinds.h - Mips Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CoffeeCL_MIPSFIXUPKINDS_H
#define LLVM_CoffeeCL_MIPSFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace CoffeeCL {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the save order of
  // MCFixupKindInfo Infos[Mips::NumTargetFixupKinds]
  // in MipsAsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_CoffeeCL_16.
    fixup_CoffeeCL_16 = FirstTargetFixupKind,

    // Pure 32 bit data fixup resulting in - R_CoffeeCL_32.
    fixup_CoffeeCL_32,

    // Full 32 bit data relative data fixup resulting in - R_CoffeeCL_REL32.
    fixup_CoffeeCL_REL32,

    // Jump 26 bit fixup resulting in - R_CoffeeCL_26.
    fixup_CoffeeCL_26,
    // Jump 25 bit fixup
    fixup_CoffeeCL_25,

    // branch 22
    fixup_CoffeeCL_22,

    // Pure upper 16 bit fixup resulting in - R_CoffeeCL_HI16.
    fixup_CoffeeCL_HI16,

    // Pure lower 16 bit fixup resulting in - R_CoffeeCL_LO16.
    fixup_CoffeeCL_LO16,

    // 16 bit fixup for GP offest resulting in - R_CoffeeCL_GPREL16.
    fixup_CoffeeCL_GPREL16,

    // 16 bit literal fixup resulting in - R_CoffeeCL_LITERAL.
    fixup_CoffeeCL_LITERAL,

    // Global symbol fixup resulting in - R_CoffeeCL_GOT16.
    fixup_CoffeeCL_GOT_Global,

    // Local symbol fixup resulting in - R_CoffeeCL_GOT16.
    fixup_CoffeeCL_GOT_Local,

    // PC relative branch fixup resulting in - R_CoffeeCL_PC16.
    fixup_CoffeeCL_PC16,

    // resulting in - R_CoffeeCL_CALL16.
    fixup_CoffeeCL_CALL16,

    // resulting in - R_CoffeeCL_GPREL32.
    fixup_CoffeeCL_GPREL32,

    // resulting in - R_CoffeeCL_SHIFT5.
    fixup_CoffeeCL_SHIFT5,

    // resulting in - R_CoffeeCL_SHIFT6.
    fixup_CoffeeCL_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_CoffeeCL_64.
    fixup_CoffeeCL_64,

    // resulting in - R_CoffeeCL_TLS_GD.
    fixup_CoffeeCL_TLSGD,

    // resulting in - R_CoffeeCL_TLS_GOTTPREL.
    fixup_CoffeeCL_GOTTPREL,

    // resulting in - R_CoffeeCL_TLS_TPREL_HI16.
    fixup_CoffeeCL_TPREL_HI,

    // resulting in - R_CoffeeCL_TLS_TPREL_LO16.
    fixup_CoffeeCL_TPREL_LO,

    // resulting in - R_CoffeeCL_TLS_LDM.
    fixup_CoffeeCL_TLSLDM,

    // resulting in - R_CoffeeCL_TLS_DTPREL_HI16.
    fixup_CoffeeCL_DTPREL_HI,

    // resulting in - R_CoffeeCL_TLS_DTPREL_LO16.
    fixup_CoffeeCL_DTPREL_LO,

    // PC relative branch fixup resulting in - R_CoffeeCL_PC16
    fixup_CoffeeCL_Branch_PCRel,

    // resulting in - R_CoffeeCL_GPREL16/R_CoffeeCL_SUB/R_CoffeeCL_HI16
    fixup_CoffeeCL_GPOFF_HI,

    // resulting in - R_CoffeeCL_GPREL16/R_CoffeeCL_SUB/R_CoffeeCL_LO16
    fixup_CoffeeCL_GPOFF_LO,

    // resulting in - R_CoffeeCL_PAGE
    fixup_CoffeeCL_GOT_PAGE,

    // resulting in - R_CoffeeCL_GOT_OFST
    fixup_CoffeeCL_GOT_OFST,

    // resulting in - R_CoffeeCL_GOT_DISP
    fixup_CoffeeCL_GOT_DISP,

    // resulting in - R_CoffeeCL_GOT_HIGHER
    fixup_CoffeeCL_HIGHER,

    // resulting in - R_CoffeeCL_HIGHEST
    fixup_CoffeeCL_HIGHEST,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace Mips
} // namespace llvm


#endif // LLVM_CoffeeCL_MIPSFIXUPKINDS_H
