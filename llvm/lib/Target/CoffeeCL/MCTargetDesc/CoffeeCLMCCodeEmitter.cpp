//===-- CoffeeCLMCCodeEmitter.cpp - Convert CoffeeCL Code to Machine Code ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeCLMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//
//
#define DEBUG_TYPE "mccodeemitter"
#include "MCTargetDesc/CoffeeCLBaseInfo.h"
#include "MCTargetDesc/CoffeeCLDirectObjLower.h"
#include "MCTargetDesc/CoffeeCLFixupKinds.h"
#include "MCTargetDesc/CoffeeCLMCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
class CoffeeCLMCCodeEmitter : public MCCodeEmitter {
  CoffeeCLMCCodeEmitter(const CoffeeCLMCCodeEmitter &) LLVM_DELETED_FUNCTION;
  void operator=(const CoffeeCLMCCodeEmitter &) LLVM_DELETED_FUNCTION;
  const MCInstrInfo &MCII;
  bool IsLittleEndian;

public:
  CoffeeCLMCCodeEmitter(const MCInstrInfo &mcii, bool IsLittle) :
            MCII(mcii), IsLittleEndian(IsLittle) {}

  ~CoffeeCLMCCodeEmitter() {}

  void EmitByte(unsigned char C, raw_ostream &OS) const {
    OS << (char)C;
  }

  void EmitInstruction(uint64_t Val, unsigned Size, raw_ostream &OS) const {
    // Output the instruction encoding in little endian byte order.
    for (unsigned i = 0; i < Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }

  void EncodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups) const;

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups) const;

  // getBranchJumpOpValue - Return binary encoding of the jump
  // target operand. If the machine operand requires relocation,
  // record the relocation and return zero.
   unsigned getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                                 SmallVectorImpl<MCFixup> &Fixups) const;

   // getBranchTargetOpValue - Return binary encoding of the branch
   // target operand. If the machine operand requires relocation,
   // record the relocation and return zero.
  unsigned getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups) const;

   // getMachineOpValue - Return binary encoding of operand. If the machin
   // operand requires relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI,const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups) const;

  unsigned getMemEncoding(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups) const;

  unsigned getMemLoadEncoding(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups) const;

  unsigned getMemStoreEncoding(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups) const;

  unsigned getSizeExtEncoding(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups) const;
  unsigned getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups) const;




}; // class CoffeeCLMCCodeEmitter
}  // namespace


MCCodeEmitter *llvm::createCoffeeCLMCCodeEmitter(const MCInstrInfo &MCII,
                                               const MCRegisterInfo &MRI,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx)
{
  return new CoffeeCLMCCodeEmitter(MCII, false); //big endian
}

/// EncodeInstruction - Emit the instruction.
/// Size the instruction (currently only 4 bytes
void CoffeeCLMCCodeEmitter::
EncodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups) const
{

  // Non-pseudo instructions that get changed for direct object
  // only based on operand values.
  // If this list of instructions get much longer we will move
  // the check to a function call. Until then, this is more efficient.
  MCInst TmpInst = MI;
 /* switch (MI.getOpcode()) {
  // If shift amount is >= 32 it the inst needs to be lowered further
  case CoffeeCL::DSLL:
  case CoffeeCL::DSRL:
  case CoffeeCL::DSRA:
    CoffeeCL::LowerLargeShift(TmpInst);
    break;
    // Double extract instruction is chosen by pos and size operands
  case CoffeeCL::DEXT:
  case CoffeeCL::DINS:
    CoffeeCL::LowerDextDins(TmpInst);
  }*/

  /**********************************************/
  //print encoding instruction

  DEBUG(dbgs() << "--------------------------------\n");
  DEBUG(dbgs() << "opcode" << MI.getOpcode()<<"\n");
  for (int i = 0; i< MI.getNumOperands(); i++) {
      DEBUG(dbgs() << "oprand " <<i<< MI.getOperand(i)<<"\n");
  }


  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups);
  DEBUG(dbgs() << "----------Encoding-----------\n");


  int bits = 32;
  char str[32] = {0};

  // type punning because signed shift is implementation-defined
  unsigned u = *(unsigned *)&Binary;
  for(; bits--; u <<= 1)
      str[bits] = u & 0x80000000 ? '1' : '0';

  DEBUG(dbgs() <<str<<"\n" );

  /************************************************/

  // Check for unimplemented opcodes.
  // Unfortunately in CoffeeCL both NOP and SLL will come in with Binary == 0
  // so we have to special check for them.
  unsigned Opcode = TmpInst.getOpcode();
  if ((Opcode != CoffeeCL::NOP) && (Opcode != CoffeeCL::SLL) && !Binary)
    llvm_unreachable("unimplemented opcode in EncodeInstruction()");

  const MCInstrDesc &Desc = MCII.get(TmpInst.getOpcode());
  uint64_t TSFlags = Desc.TSFlags;

  // Pseudo instructions don't get encoded and shouldn't be here
  // in the first place!
  if ((TSFlags & CoffeeCLII::FormMask) == CoffeeCLII::Pseudo)
    llvm_unreachable("Pseudo opcode found in EncodeInstruction()");

  // Get byte count of instruction
  unsigned Size = Desc.getSize();
  if (!Size)
    llvm_unreachable("Desc.getSize() returns 0");

  EmitInstruction(Binary, Size, OS);
}

/// getBranchTargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned CoffeeCLMCCodeEmitter::
getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups) const {

  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::Create(0, Expr,
                                   MCFixupKind(CoffeeCL::fixup_CoffeeCL_22)));
  return 0;
}

/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned CoffeeCLMCCodeEmitter::
getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups) const {

  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() &&
         "getJumpTargetOpValue expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::Create(0, Expr,
                                   MCFixupKind(CoffeeCL::fixup_CoffeeCL_25)));
  return 0;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned CoffeeCLMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    unsigned RegNo = getCoffeeCLRegisterNumbering(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } else if (MO.isFPImm()) {
    return static_cast<unsigned>(APFloat(MO.getFPImm())
        .bitcastToAPInt().getHiBits(32).getLimitedValue());
  }

  // MO must be an Expr.
  assert(MO.isExpr());

  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr*>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  assert (Kind == MCExpr::SymbolRef);
  CoffeeCL::Fixups FixupKind = CoffeeCL::Fixups(0);

  switch(cast<MCSymbolRefExpr>(Expr)->getKind()) {
  default: llvm_unreachable("Unknown fixup kind!");
    break;
  /*case MCSymbolRefExpr::VK_CoffeeCL_GPOFF_HI :
    FixupKind = CoffeeCL::fixup_CoffeeCL_GPOFF_HI;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GPOFF_LO :
    FixupKind = CoffeeCL::fixup_CoffeeCL_GPOFF_LO;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT_PAGE :
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOT_PAGE;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT_OFST :
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOT_OFST;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT_DISP :
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOT_DISP;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GPREL:
    FixupKind = CoffeeCL::fixup_CoffeeCL_GPREL16;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT_CALL:
    FixupKind = CoffeeCL::fixup_CoffeeCL_CALL16;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT16:
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOT_Global;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOT:
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOT_Local;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_ABS_HI:
    FixupKind = CoffeeCL::fixup_CoffeeCL_HI16;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_ABS_LO:
    FixupKind = CoffeeCL::fixup_CoffeeCL_LO16;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_TLSGD:
    FixupKind = CoffeeCL::fixup_CoffeeCL_TLSGD;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_TLSLDM:
    FixupKind = CoffeeCL::fixup_CoffeeCL_TLSLDM;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_DTPREL_HI:
    FixupKind = CoffeeCL::fixup_CoffeeCL_DTPREL_HI;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_DTPREL_LO:
    FixupKind = CoffeeCL::fixup_CoffeeCL_DTPREL_LO;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_GOTTPREL:
    FixupKind = CoffeeCL::fixup_CoffeeCL_GOTTPREL;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_TPREL_HI:
    FixupKind = CoffeeCL::fixup_CoffeeCL_TPREL_HI;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_TPREL_LO:
    FixupKind = CoffeeCL::fixup_CoffeeCL_TPREL_LO;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_HIGHER:
    FixupKind = CoffeeCL::fixup_CoffeeCL_HIGHER;
    break;
  case MCSymbolRefExpr::VK_CoffeeCL_HIGHEST:
    FixupKind = CoffeeCL::fixup_CoffeeCL_HIGHEST;
    break;*/
  } // switch

  Fixups.push_back(MCFixup::Create(0, MO.getExpr(), MCFixupKind(FixupKind)));

  // All of the information is in the fixup.
  return 0;
}

/// getMemEncoding - Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
unsigned
CoffeeCLMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups) const {
  assert(MI.getOperand(OpNo).isReg());
  // register 0-4
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups);
  // imm 5-19
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups)<<5;

  return (OffBits & 0xFFFE0) | (RegBits & 0x1F);
}

unsigned
CoffeeCLMCCodeEmitter::getMemLoadEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups) const {

  assert(MI.getOperand(OpNo).isReg());
  // register 0-4
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups);
  // imm 5-19
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups)<<5;



  return (OffBits & 0xFFFE0) | (RegBits & 0x1F);
}


unsigned
CoffeeCLMCCodeEmitter::getMemStoreEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups) const {

  assert(MI.getOperand(OpNo).isReg());
  // register 5-9
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups) << 5;
  // imm1 0-4
  unsigned OffBits1 = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups);
  // imm2 15-24

  // imm is 15 bit value
  // we need to assign offbits2 the upper 10 bits imm{14-5} and put it to addr {19-10}
  unsigned OffBits2 = (getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups) & 0x7FE0) << 5;
  // addr    {19-10}                   {9-5}              {4-0}
  return (OffBits2 & 0xFFC00) | (RegBits & 0x3E0) | (OffBits1 & 0x1F);
}


unsigned
CoffeeCLMCCodeEmitter::getSizeExtEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups) const {
  assert(MI.getOperand(OpNo).isImm());
  unsigned SizeEncoding = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups);
  return SizeEncoding - 1;
}

// FIXME: should be called getMSBEncoding
//
unsigned
CoffeeCLMCCodeEmitter::getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups) const {
  assert(MI.getOperand(OpNo-1).isImm());
  assert(MI.getOperand(OpNo).isImm());
  unsigned Position = getMachineOpValue(MI, MI.getOperand(OpNo-1), Fixups);
  unsigned Size = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups);

  return Position + Size - 1;
}


#include "CoffeeCLGenMCCodeEmitter.inc"

