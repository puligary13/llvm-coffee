//===-- CoffeeELFObjectWriter.cpp - Coffee ELF Writer ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CoffeeFixupKinds.h"
#include "MCTargetDesc/CoffeeMCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
  class CoffeeELFObjectWriter : public MCELFObjectTargetWriter {
  public:
    CoffeeELFObjectWriter(uint8_t OSABI);

    virtual ~CoffeeELFObjectWriter();
  protected:
    virtual unsigned GetRelocType(const MCValue &Target, const MCFixup &Fixup,
                                  bool IsPCRel, bool IsRelocWithSymbol,
                                  int64_t Addend) const;
    virtual void adjustFixupOffset(const MCFixup &Fixup, uint64_t &RelocOffset);
  };
}

CoffeeELFObjectWriter::CoffeeELFObjectWriter(uint8_t OSABI)
  : MCELFObjectTargetWriter(false, OSABI,
                            ELF::EM_COFFEE,
                            /*HasRelocationAddend*/ true) {}

CoffeeELFObjectWriter::~CoffeeELFObjectWriter() {
}

unsigned CoffeeELFObjectWriter::GetRelocType(const MCValue &Target,
                                             const MCFixup &Fixup,
                                             bool IsPCRel,
                                             bool IsRelocWithSymbol,
                                             int64_t Addend) const {
  // determine the type of the relocation
  unsigned Type;
  if (IsPCRel) {
    switch ((unsigned)Fixup.getKind()) {
    default:
      llvm_unreachable("Unimplemented");
    case Coffee::fixup_Coffee_br24:
      Type = ELF::R_Coffee_REL24;
      break;
    case FK_PCRel_4:
      Type = ELF::R_Coffee_REL32;
      break;
    }
  } else {
    switch ((unsigned)Fixup.getKind()) {
      default: llvm_unreachable("invalid fixup kind!");
    case Coffee::fixup_Coffee_br24:
      Type = ELF::R_Coffee_ADDR24;
      break;
    case Coffee::fixup_Coffee_brcond14:
      Type = ELF::R_Coffee_ADDR14_BRTAKEN; // XXX: or BRNTAKEN?_
      break;
    case Coffee::fixup_Coffee_ha16:
      Type = ELF::R_Coffee_ADDR16_HA;
      break;
    case Coffee::fixup_Coffee_lo16:
      Type = ELF::R_Coffee_ADDR16_LO;
      break;
    case Coffee::fixup_Coffee_lo14:
      Type = ELF::R_Coffee_ADDR14;
      break;
    case FK_Data_4:
      Type = ELF::R_Coffee_ADDR32;
      break;
    case FK_Data_2:
      Type = ELF::R_Coffee_ADDR16;
      break;
    }
  }
  return Type;
}

void CoffeeELFObjectWriter::
adjustFixupOffset(const MCFixup &Fixup, uint64_t &RelocOffset) {
  switch ((unsigned)Fixup.getKind()) {
    case Coffee::fixup_Coffee_ha16:
    case Coffee::fixup_Coffee_lo16:
      RelocOffset += 2;
      break;
    default:
      break;
  }
}

MCObjectWriter *llvm::createCoffeeELFObjectWriter(raw_ostream &OS,
                                               uint8_t OSABI) {
  MCELFObjectTargetWriter *MOTW = new CoffeeELFObjectWriter(OSABI);
  return createELFObjectWriter(MOTW, OS,  /*IsLittleEndian=*/false);
}
