//===-- CoffeeCLASMBackend.cpp - CoffeeCL Asm Backend  ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeCLAsmBackend and CoffeeCLELFObjectWriter classes.
//
//===----------------------------------------------------------------------===//
//

#include "CoffeeCLFixupKinds.h"
#include "MCTargetDesc/CoffeeCLMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

// Prepare value for the target space for it
static unsigned adjustFixupValue(unsigned Kind, uint64_t Value) {

  // Add/subtract and shift
  switch (Kind) {
  default:
    return 0;
  case FK_GPRel_4:
  case FK_Data_4:
  case FK_Data_8:
  case CoffeeCL::fixup_CoffeeCL_LO16:
  case CoffeeCL::fixup_CoffeeCL_GPOFF_HI:
  case CoffeeCL::fixup_CoffeeCL_GPOFF_LO:
  case CoffeeCL::fixup_CoffeeCL_GOT_PAGE:
  case CoffeeCL::fixup_CoffeeCL_GOT_OFST:
  case CoffeeCL::fixup_CoffeeCL_GOT_DISP:
    break;
  case CoffeeCL::fixup_CoffeeCL_PC16:
    // So far we are only using this type for branches.
    // For branches we start 1 instruction after the branch
    // so the displacement will be one instruction size less.
    Value -= 4;
    // The displacement is then divided by 4 to give us an 18 bit
    // address range.
    Value >>= 2;
    break;
  case CoffeeCL::fixup_CoffeeCL_26:
    // So far we are only using this type for jumps.
    // The displacement is then divided by 4 to give us an 28 bit
    // address range.
    Value >>= 2;
    break;
  case CoffeeCL::fixup_CoffeeCL_25:
  case CoffeeCL::fixup_CoffeeCL_22:
        DEBUG(dbgs() << "coffeecl25before" << Value<<"\n");
        if (Value) {
            //TODO: should recheck this. if value is 0, what to do ?
            Value -= 4;
            Value >>= 1;
        }
        DEBUG(dbgs() << "coffeecl25after" << Value<<"\n");
    break;
  case CoffeeCL::fixup_CoffeeCL_HI16:
  case CoffeeCL::fixup_CoffeeCL_GOT_Local:
    // Get the 2nd 16-bits. Also add 1 if bit 15 is 1.
    Value = ((Value + 0x8000) >> 16) & 0xffff;
    break;
  case CoffeeCL::fixup_CoffeeCL_HIGHER:
    // Get the 3rd 16-bits.
    Value = ((Value + 0x80008000LL) >> 32) & 0xffff;
    break;
  case CoffeeCL::fixup_CoffeeCL_HIGHEST:
    // Get the 4th 16-bits.
    Value = ((Value + 0x800080008000LL) >> 48) & 0xffff;
    break;
  }

  return Value;
}

namespace {
class CoffeeCLAsmBackend : public MCAsmBackend {
  Triple::OSType OSType;
  bool IsLittle; // Big or little endian
  bool Is64Bit;  // 32 or 64 bit words

public:
  CoffeeCLAsmBackend(const Target &T,  Triple::OSType _OSType,
                 bool _isLittle, bool _is64Bit)
    :MCAsmBackend(), OSType(_OSType), IsLittle(_isLittle), Is64Bit(_is64Bit) {}

  MCObjectWriter *createObjectWriter(raw_ostream &OS) const {
    return createCoffeeCLELFObjectWriter(OS,
      MCELFObjectTargetWriter::getOSABI(OSType), IsLittle, Is64Bit);
  }

  /// ApplyFixup - Apply the \p Value for given \p Fixup into the provided
  /// data fragment, at the offset specified by the fixup and following the
  /// fixup kind as appropriate.
  void applyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                  uint64_t Value) const {
    MCFixupKind Kind = Fixup.getKind();
    Value = adjustFixupValue((unsigned)Kind, Value);

    if (!Value)
      return; // Doesn't change encoding.

    // Where do we start in the object
    unsigned Offset = Fixup.getOffset();
    // Number of bytes we need to fixup
    unsigned NumBytes = (getFixupKindInfo(Kind).TargetSize + 7) / 8;
    // Used to point to big endian bytes
    unsigned FullSize;

    switch ((unsigned)Kind) {
    case CoffeeCL::fixup_CoffeeCL_16:
      FullSize = 2;
      break;
    case CoffeeCL::fixup_CoffeeCL_64:
      FullSize = 8;
      break;
    default:
      FullSize = 4;
      break;
    }

    // Grab current value, if any, from bits.
    uint64_t CurVal = 0;

    for (unsigned i = 0; i != NumBytes; ++i) {
      unsigned Idx = IsLittle ? i : (FullSize - 1 - i);
      CurVal |= (uint64_t)((uint8_t)Data[Offset + Idx]) << (i*8);
    }

    uint64_t Mask = ((uint64_t)(-1) >>
                     (64 - getFixupKindInfo(Kind).TargetSize));
    CurVal |= Value & Mask;

    // Write out the fixed up bytes back to the code/data bits.
    for (unsigned i = 0; i != NumBytes; ++i) {
      unsigned Idx = IsLittle ? i : (FullSize - 1 - i);
      Data[Offset + Idx] = (uint8_t)((CurVal >> (i*8)) & 0xff);
    }
  }

  unsigned getNumFixupKinds() const { return CoffeeCL::NumTargetFixupKinds; }

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const {
    const static MCFixupKindInfo Infos[CoffeeCL::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // CoffeeCLFixupKinds.h.
      //
      // name                    offset  bits  flags
      { "fixup_CoffeeCL_16",           0,     16,   0 },
      { "fixup_CoffeeCL_32",           0,     32,   0 },
      { "fixup_CoffeeCL_REL32",        0,     32,   0 },
      { "fixup_CoffeeCL_26",           0,     26,   0 },
      { "fixup_CoffeeCL_25",           0,     25,   MCFixupKindInfo::FKF_IsPCRel }, //for jmp in coffeecl
      { "fixup_CoffeeCL_22",           0,     22,   MCFixupKindInfo::FKF_IsPCRel }, //for branch in coffeecl
      { "fixup_CoffeeCL_HI16",         0,     16,   0 },
      { "fixup_CoffeeCL_LO16",         0,     16,   0 },
      { "fixup_CoffeeCL_GPREL16",      0,     16,   0 },
      { "fixup_CoffeeCL_LITERAL",      0,     16,   0 },
      { "fixup_CoffeeCL_GOT_Global",   0,     16,   0 },
      { "fixup_CoffeeCL_GOT_Local",    0,     16,   0 },
      { "fixup_CoffeeCL_PC16",         0,     16,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_CoffeeCL_CALL16",       0,     16,   0 },
      { "fixup_CoffeeCL_GPREL32",      0,     32,   0 },
      { "fixup_CoffeeCL_SHIFT5",       6,      5,   0 },
      { "fixup_CoffeeCL_SHIFT6",       6,      5,   0 },
      { "fixup_CoffeeCL_64",           0,     64,   0 },
      { "fixup_CoffeeCL_TLSGD",        0,     16,   0 },
      { "fixup_CoffeeCL_GOTTPREL",     0,     16,   0 },
      { "fixup_CoffeeCL_TPREL_HI",     0,     16,   0 },
      { "fixup_CoffeeCL_TPREL_LO",     0,     16,   0 },
      { "fixup_CoffeeCL_TLSLDM",       0,     16,   0 },
      { "fixup_CoffeeCL_DTPREL_HI",    0,     16,   0 },
      { "fixup_CoffeeCL_DTPREL_LO",    0,     16,   0 },
      { "fixup_CoffeeCL_Branch_PCRel", 0,     16,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_CoffeeCL_GPOFF_HI",     0,     16,   0 },
      { "fixup_CoffeeCL_GPOFF_LO",     0,     16,   0 },
      { "fixup_CoffeeCL_GOT_PAGE",     0,     16,   0 },
      { "fixup_CoffeeCL_GOT_OFST",     0,     16,   0 },
      { "fixup_CoffeeCL_GOT_DISP",     0,     16,   0 },
      { "fixup_CoffeeCL_HIGHER",       0,     16,   0 },
      { "fixup_CoffeeCL_HIGHEST",      0,     16,   0 }
    };

    if (Kind < FirstTargetFixupKind)
      return MCAsmBackend::getFixupKindInfo(Kind);

    assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
           "Invalid kind!");
    return Infos[Kind - FirstTargetFixupKind];
  }

  /// @name Target Relaxation Interfaces
  /// @{

  /// MayNeedRelaxation - Check whether the given instruction may need
  /// relaxation.
  ///
  /// \param Inst - The instruction to test.
  bool mayNeedRelaxation(const MCInst &Inst) const {
    return false;
  }

  /// fixupNeedsRelaxation - Target specific predicate for whether a given
  /// fixup requires the associated instruction to be relaxed.
  bool fixupNeedsRelaxation(const MCFixup &Fixup,
                            uint64_t Value,
                            const MCInstFragment *DF,
                            const MCAsmLayout &Layout) const {
    // FIXME.
    assert(0 && "RelaxInstruction() unimplemented");
    return false;
  }

  /// RelaxInstruction - Relax the instruction in the given fragment
  /// to the next wider instruction.
  ///
  /// \param Inst - The instruction to relax, which may be the same
  /// as the output.
  /// \param [out] Res On return, the relaxed instruction.
  void relaxInstruction(const MCInst &Inst, MCInst &Res) const {
  }

  /// @}

  /// WriteNopData - Write an (optimal) nop sequence of Count bytes
  /// to the given output. If the target cannot generate such a sequence,
  /// it should return an error.
  ///
  /// \return - True on success.
  bool writeNopData(uint64_t Count, MCObjectWriter *OW) const {
    // Check for a less than instruction size number of bytes
    // FIXME: 16 bit instructions are not handled yet here.
    // We shouldn't be using a hard coded number for instruction size.
    if (Count % 4) return false;

    uint64_t NumNops = Count / 4;
    for (uint64_t i = 0; i != NumNops; ++i)
      OW->Write32(0);
    return true;
  }
}; // class CoffeeCLAsmBackend

} // namespace

// MCAsmBackend
MCAsmBackend *llvm::createCoffeeCLAsmBackend(const Target &T, StringRef TT,
                                             StringRef CPU) {
  return new CoffeeCLAsmBackend(T, Triple(TT).getOS(),
                            /*IsLittle*/false, /*Is64Bit*/false);
}

