//===-- CoffeeAsmBackend.cpp - Coffee Assembler Backend -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CoffeeMCTargetDesc.h"
#include "MCTargetDesc/CoffeeFixupKinds.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCMachObjectWriter.h"
#include "llvm/MC/MCSectionMachO.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Object/MachOFormat.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

static unsigned adjustFixupValue(unsigned Kind, uint64_t Value) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
    return Value;
  case Coffee::fixup_Coffee_brcond14:
    return Value & 0x3ffc;
  case Coffee::fixup_Coffee_br24:
    return Value & 0x3fffffc;
#if 0
  case Coffee::fixup_Coffee_hi16:
    return (Value >> 16) & 0xffff;
#endif
  case Coffee::fixup_Coffee_ha16:
    return ((Value >> 16) + ((Value & 0x8000) ? 1 : 0)) & 0xffff;
  case Coffee::fixup_Coffee_lo16:
    return Value & 0xffff;
  }
}

namespace {
class CoffeeMachObjectWriter : public MCMachObjectTargetWriter {
public:
  CoffeeMachObjectWriter(bool Is64Bit, uint32_t CPUType,
                      uint32_t CPUSubtype)
    : MCMachObjectTargetWriter(Is64Bit, CPUType, CPUSubtype) {}

  void RecordRelocation(MachObjectWriter *Writer,
                        const MCAssembler &Asm, const MCAsmLayout &Layout,
                        const MCFragment *Fragment, const MCFixup &Fixup,
                        MCValue Target, uint64_t &FixedValue) {}
};

class CoffeeAsmBackend : public MCAsmBackend {
const Target &TheTarget;
public:
  CoffeeAsmBackend(const Target &T) : MCAsmBackend(), TheTarget(T) {}

  unsigned getNumFixupKinds() const { return Coffee::NumTargetFixupKinds; }

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const {
    const static MCFixupKindInfo Infos[Coffee::NumTargetFixupKinds] = {
      // name                    offset  bits  flags
      { "fixup_Coffee_br24",        6,      24,   MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_Coffee_brcond14",    16,     14,   MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_Coffee_lo16",        16,     16,   0 },
      { "fixup_Coffee_ha16",        16,     16,   0 },
      { "fixup_Coffee_lo14",        16,     14,   0 }
    };

    if (Kind < FirstTargetFixupKind)
      return MCAsmBackend::getFixupKindInfo(Kind);

    assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
           "Invalid kind!");
    return Infos[Kind - FirstTargetFixupKind];
  }

  bool mayNeedRelaxation(const MCInst &Inst) const {
    // FIXME.
    return false;
  }

  bool fixupNeedsRelaxation(const MCFixup &Fixup,
                            uint64_t Value,
                            const MCInstFragment *DF,
                            const MCAsmLayout &Layout) const {
    // FIXME.
    llvm_unreachable("relaxInstruction() unimplemented");
  }


  void relaxInstruction(const MCInst &Inst, MCInst &Res) const {
    // FIXME.
    llvm_unreachable("relaxInstruction() unimplemented");
  }

  bool writeNopData(uint64_t Count, MCObjectWriter *OW) const {
    // FIXME: Zero fill for now. That's not right, but at least will get the
    // section size right.
    for (uint64_t i = 0; i != Count; ++i)
      OW->Write8(0);
    return true;
  }

  unsigned getPointerSize() const {
    StringRef Name = TheTarget.getName();
    if (Name == "Coffee64") return 8;
    assert(Name == "Coffee32" && "Unknown target name!");
    return 4;
  }
};
} // end anonymous namespace


// FIXME: This should be in a separate file.
namespace {

  class ELFCoffeeAsmBackend : public CoffeeAsmBackend {
    uint8_t OSABI;
  public:
    ELFCoffeeAsmBackend(const Target &T, uint8_t OSABI) :
      CoffeeAsmBackend(T), OSABI(OSABI) { }

    void applyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                    uint64_t Value) const {
      Value = adjustFixupValue(Fixup.getKind(), Value);
      if (!Value) return;           // Doesn't change encoding.

      unsigned Offset = Fixup.getOffset();

      // For each byte of the fragment that the fixup touches, mask in the bits from
      // the fixup value. The Value has been "split up" into the appropriate
      // bitfields above.
      for (unsigned i = 0; i != 4; ++i)
        Data[Offset + i] |= uint8_t((Value >> ((4 - i - 1)*8)) & 0xff);
    }

    MCObjectWriter *createObjectWriter(raw_ostream &OS) const {
      return createCoffeeELFObjectWriter(OS, OSABI);
    }

    virtual bool doesSectionRequireSymbols(const MCSection &Section) const {
      return false;
    }
  };

} // end anonymous namespace




MCAsmBackend *llvm::createCoffeeAsmBackend(const Target &T, StringRef TT) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(Triple(TT).getOS());
  return new ELFCoffeeAsmBackend(T, OSABI);
}
