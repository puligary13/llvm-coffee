//===-- MipsFixupKinds.h - Mips Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Coffee_MIPSFIXUPKINDS_H
#define LLVM_Coffee_MIPSFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace Coffee {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the save order of
  // MCFixupKindInfo Infos[Mips::NumTargetFixupKinds]
  // in MipsAsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_Coffee_16.
    fixup_Coffee_16 = FirstTargetFixupKind,

    // Pure 32 bit data fixup resulting in - R_Coffee_32.
    fixup_Coffee_32,

    // Full 32 bit data relative data fixup resulting in - R_Coffee_REL32.
    fixup_Coffee_REL32,

    // Jump 26 bit fixup resulting in - R_Coffee_26.
    fixup_Coffee_26,
    // Jump 25 bit fixup
    fixup_Coffee_25,

    // Pure upper 16 bit fixup resulting in - R_Coffee_HI16.
    fixup_Coffee_HI16,

    // Pure lower 16 bit fixup resulting in - R_Coffee_LO16.
    fixup_Coffee_LO16,

    // 16 bit fixup for GP offest resulting in - R_Coffee_GPREL16.
    fixup_Coffee_GPREL16,

    // 16 bit literal fixup resulting in - R_Coffee_LITERAL.
    fixup_Coffee_LITERAL,

    // Global symbol fixup resulting in - R_Coffee_GOT16.
    fixup_Coffee_GOT_Global,

    // Local symbol fixup resulting in - R_Coffee_GOT16.
    fixup_Coffee_GOT_Local,

    // PC relative branch fixup resulting in - R_Coffee_PC16.
    fixup_Coffee_PC16,

    // resulting in - R_Coffee_CALL16.
    fixup_Coffee_CALL16,

    // resulting in - R_Coffee_GPREL32.
    fixup_Coffee_GPREL32,

    // resulting in - R_Coffee_SHIFT5.
    fixup_Coffee_SHIFT5,

    // resulting in - R_Coffee_SHIFT6.
    fixup_Coffee_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_Coffee_64.
    fixup_Coffee_64,

    // resulting in - R_Coffee_TLS_GD.
    fixup_Coffee_TLSGD,

    // resulting in - R_Coffee_TLS_GOTTPREL.
    fixup_Coffee_GOTTPREL,

    // resulting in - R_Coffee_TLS_TPREL_HI16.
    fixup_Coffee_TPREL_HI,

    // resulting in - R_Coffee_TLS_TPREL_LO16.
    fixup_Coffee_TPREL_LO,

    // resulting in - R_Coffee_TLS_LDM.
    fixup_Coffee_TLSLDM,

    // resulting in - R_Coffee_TLS_DTPREL_HI16.
    fixup_Coffee_DTPREL_HI,

    // resulting in - R_Coffee_TLS_DTPREL_LO16.
    fixup_Coffee_DTPREL_LO,

    // PC relative branch fixup resulting in - R_Coffee_PC16
    fixup_Coffee_Branch_PCRel,

    // resulting in - R_Coffee_GPREL16/R_Coffee_SUB/R_Coffee_HI16
    fixup_Coffee_GPOFF_HI,

    // resulting in - R_Coffee_GPREL16/R_Coffee_SUB/R_Coffee_LO16
    fixup_Coffee_GPOFF_LO,

    // resulting in - R_Coffee_PAGE
    fixup_Coffee_GOT_PAGE,

    // resulting in - R_Coffee_GOT_OFST
    fixup_Coffee_GOT_OFST,

    // resulting in - R_Coffee_GOT_DISP
    fixup_Coffee_GOT_DISP,

    // resulting in - R_Coffee_GOT_HIGHER
    fixup_Coffee_HIGHER,

    // resulting in - R_Coffee_HIGHEST
    fixup_Coffee_HIGHEST,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace Mips
} // namespace llvm


#endif // LLVM_Coffee_MIPSFIXUPKINDS_H
