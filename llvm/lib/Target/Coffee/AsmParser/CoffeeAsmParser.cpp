//===-- CoffeeAsmParser.cpp - Parse Coffee assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CoffeeMCTargetDesc.h"
#include "CoffeeRegisterInfo.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCTargetAsmParser.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

namespace {
class CoffeeAssemblerOptions {
public:
  CoffeeAssemblerOptions():
    aTReg(1), reorder(true), macro(true) {
  }

  unsigned getATRegNum() {return aTReg;}
  bool setATReg(unsigned Reg);

  bool isReorder() {return reorder;}
  void setReorder() {reorder = true;}
  void setNoreorder() {reorder = false;}

  bool isMacro() {return macro;}
  void setMacro() {macro = true;}
  void setNomacro() {macro = false;}

private:
  unsigned aTReg;
  bool reorder;
  bool macro;
};
}

namespace {
class CoffeeAsmParser : public MCTargetAsmParser {

  enum FpFormatTy {
    FP_FORMAT_NONE = -1,
    FP_FORMAT_S,
    FP_FORMAT_D,
    FP_FORMAT_L,
    FP_FORMAT_W
  } FpFormat;

  MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  CoffeeAssemblerOptions Options;


#define GET_ASSEMBLER_HEADER
#include "CoffeeGenAsmMatcher.inc"

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                               MCStreamer &Out, unsigned &ErrorInfo,
                               bool MatchingInlineAsm);

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc);

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc,
                        SmallVectorImpl<MCParsedAsmOperand*> &Operands);

  bool parseMathOperation(StringRef Name, SMLoc NameLoc,
                        SmallVectorImpl<MCParsedAsmOperand*> &Operands);

  bool ParseDirective(AsmToken DirectiveID);

  CoffeeAsmParser::OperandMatchResultTy
  parseMemOperand(SmallVectorImpl<MCParsedAsmOperand*>&);

  CoffeeAsmParser::OperandMatchResultTy
  parseAddrOperand(SmallVectorImpl<MCParsedAsmOperand*>&);

  bool ParseOperand(SmallVectorImpl<MCParsedAsmOperand*> &,
                    StringRef Mnemonic);

  int tryParseRegister(StringRef Mnemonic);

  bool tryParseRegisterOperand(SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                               StringRef Mnemonic);

  bool needsExpansion(MCInst &Inst);

  void expandInstruction(MCInst &Inst, SMLoc IDLoc,
                         SmallVectorImpl<MCInst> &Instructions);
  void expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                     SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);


  void expandPushr(MCInst &Inst, SMLoc IDLoc,
                   SmallVectorImpl<MCInst> &Instructions);

  void expandPopr(MCInst &Inst, SMLoc IDLoc,
                  SmallVectorImpl<MCInst> &Instructions);

  void expandSt(MCInst &Inst, SMLoc IDLoc,
                SmallVectorImpl<MCInst> &Instructions);

  void expandLd(MCInst &Inst, SMLoc IDLoc,
                SmallVectorImpl<MCInst> &Instructions);


  bool reportParseError(StringRef ErrorMsg);

  bool parseMemOffset(const MCExpr *&Res);
  bool parseRelocOperand(const MCExpr *&Res);

  bool parseDirectiveWord(unsigned Size, SMLoc L);

  bool parseDirectiveSet();

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();

  MCSymbolRefExpr::VariantKind getVariantKind(StringRef Symbol);

  bool isCoffee64() const {
    return false;
  }

  bool isFP64() const {
    return false;
  }

  int matchRegisterName(StringRef Symbol);

  int matchRegisterByNumber(unsigned RegNum, StringRef Mnemonic);

  void setFpFormat(FpFormatTy Format) {
    FpFormat = Format;
  }

  void setDefaultFpFormat();

  void setFpFormat(StringRef Format);

  FpFormatTy getFpFormat() {return FpFormat;}

  bool requestsDoubleOperand(StringRef Mnemonic);

  unsigned getReg(int RC,int RegNo);

  unsigned getATReg();
public:
  CoffeeAsmParser(MCSubtargetInfo &sti, MCAsmParser &parser)
    : MCTargetAsmParser(), STI(sti), Parser(parser) {
    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }

};
}

namespace {

/// CoffeeOperand - Instances of this class represent a parsed Coffee machine
/// instruction.
class CoffeeOperand : public MCParsedAsmOperand {

  enum KindTy {
    k_CondCode,
    k_CoprocNum,
    k_Immediate,
    k_Memory,
    k_PostIndexRegister,
    k_Register,
    k_Token,
    k_Expr,
    k_Address
  } Kind;

  CoffeeOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

  union {
    struct {
      const char *Data;
      unsigned Length;
    } Tok;

    struct {
      unsigned RegNum;
    } Reg;

    struct {
      const MCExpr *Val;
    } Imm;

    struct {
      const MCExpr* ExprVal;
    } Expr;

    struct {
      const MCExpr* AddrVal;
    } Addr;

    struct {
      unsigned Base;
      const MCExpr *Off;
    } Mem;
  };

  SMLoc StartLoc, EndLoc;

public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::CreateReg(getReg()));
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const{
    // Add as immediate when possible.  Null MCExpr = 0.
    if (Expr == 0)
      Inst.addOperand(MCOperand::CreateImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::CreateImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::CreateExpr(Expr));
  }

  void addExprOperands(MCInst &Inst, unsigned N) const{
      assert(N == 1 && "Invalid number of operands!");
      const MCExpr *Expr = getExpr();
      addExpr(Inst,Expr);
  }

  void addAddrOperands(MCInst &Inst, unsigned N) const{
      assert(N == 1 && "Invalid number of operands!");
      const MCExpr *Expr = getAddr();
      addExpr(Inst,Expr);
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst,Expr);
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::CreateReg(getMemBase()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst,Expr);
  }

  bool isReg() const { return Kind == k_Register; }
  bool isImm() const { return Kind == k_Immediate; }
  bool isToken() const { return Kind == k_Token; }
  bool isMem() const { return Kind == k_Memory; }
  bool isExpr() const { return Kind == k_Expr; }
  bool isAddr() const { return Kind == k_Address; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  unsigned getReg() const {
    assert((Kind == k_Register) && "Invalid access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  unsigned getMemBase() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr *getMemOff() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Off;
  }

  const MCExpr *getExpr() const {
      assert((Kind == k_Expr) && "Invalid access!");
      return Expr.ExprVal;
  }

  const MCExpr *getAddr() const {
      assert((Kind == k_Address) && "Invalid access!");
      return Addr.AddrVal;
  }

  static CoffeeOperand *CreateExpr(const MCExpr *Val) {
      CoffeeOperand *Op = new CoffeeOperand(k_Expr);
      Op->Expr.ExprVal = Val;
      return Op;
  }

  static CoffeeOperand *CreateAddr(const MCExpr *Val) {
      CoffeeOperand *Op = new CoffeeOperand(k_Address);
      Op->Addr.AddrVal = Val;
      return Op;
  }

  static CoffeeOperand *CreateToken(StringRef Str, SMLoc S) {
    CoffeeOperand *Op = new CoffeeOperand(k_Token);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  static CoffeeOperand *CreateReg(unsigned RegNum, SMLoc S, SMLoc E) {
    CoffeeOperand *Op = new CoffeeOperand(k_Register);
    Op->Reg.RegNum = RegNum;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static CoffeeOperand *CreateImm(const MCExpr *Val, SMLoc S, SMLoc E) {
    CoffeeOperand *Op = new CoffeeOperand(k_Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static CoffeeOperand *CreateMem(unsigned Base, const MCExpr *Off,
                                 SMLoc S, SMLoc E) {
    CoffeeOperand *Op = new CoffeeOperand(k_Memory);
    Op->Mem.Base = Base;
    Op->Mem.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const { return EndLoc; }

  virtual void print(raw_ostream &OS) const {
    llvm_unreachable("unimplemented!");
  }
};
}

bool CoffeeAsmParser::needsExpansion(MCInst &Inst) {
  switch(Inst.getOpcode()) {
    case Coffee::LoadImm32Reg:
    //case Coffee::LoadAddr32Imm:
    case Coffee::Pushr:
    case Coffee::Popr:
    case Coffee::St:
    case Coffee::Ld:
    case Coffee::LoadAddr32Reg:
      return true;
    default:
      return false;
  }
}

void CoffeeAsmParser::expandInstruction(MCInst &Inst, SMLoc IDLoc,
                        SmallVectorImpl<MCInst> &Instructions){

     switch(Inst.getOpcode()) {
    case Coffee::LoadImm32Reg:
      return expandLoadImm(Inst, IDLoc, Instructions);
    //case Coffee::LoadAddr32Imm:
    //  return expandLoadAddressImm(Inst,IDLoc,Instructions);

     case Coffee::Pushr:
         return expandPushr(Inst, IDLoc, Instructions);
     case Coffee::Popr:
         return expandPopr(Inst, IDLoc, Instructions);
     case Coffee::St:
         return expandSt(Inst, IDLoc, Instructions);
     case Coffee::Ld:
         return expandLd(Inst, IDLoc, Instructions);
    case Coffee::LoadAddr32Reg:
      return expandLoadAddressReg(Inst, IDLoc, Instructions);
    }
}

void CoffeeAsmParser::expandPushr(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){

    MCInst tmpInst;
    const MCOperand &RegOp = Inst.getOperand(0);
    assert(RegOp.isReg() && "expected register operand kind");


      tmpInst.setOpcode(Coffee::SW);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateImm(0));
      Instructions.push_back(tmpInst);
      tmpInst.clear();
      tmpInst.setOpcode(Coffee::ADDi);
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateImm(4));
      Instructions.push_back(tmpInst);

}

void CoffeeAsmParser::expandPopr(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){

    MCInst tmpInst;
    const MCOperand &RegOp = Inst.getOperand(0);
    assert(RegOp.isReg() && "expected register operand kind");


      tmpInst.setOpcode(Coffee::LW);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateImm(0));
      Instructions.push_back(tmpInst);
      tmpInst.clear();
      tmpInst.setOpcode(Coffee::ADDi);
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateReg(Coffee::SP));
      tmpInst.addOperand(MCOperand::CreateImm(-4));
      Instructions.push_back(tmpInst);

}

void CoffeeAsmParser::expandSt(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){

    MCInst tmpInst;

    const MCOperand &ImmOp = Inst.getOperand(2);
    assert(ImmOp.isImm() && "expected immediate operand kind");
    const MCOperand &Reg1Op = Inst.getOperand(1);
    assert(Reg1Op.isReg() && "expected register operand kind");
    const MCOperand &Reg0Op = Inst.getOperand(0);
    assert(Reg0Op.isReg() && "expected register operand kind");


      tmpInst.setOpcode(Coffee::SW);
      tmpInst.addOperand(MCOperand::CreateReg(Reg0Op.getReg()));
      tmpInst.addOperand(MCOperand::CreateReg(Reg1Op.getReg()));
      tmpInst.addOperand(MCOperand::CreateImm(ImmOp.getImm()));
      Instructions.push_back(tmpInst);

}

void CoffeeAsmParser::expandLd(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){

    MCInst tmpInst;

    const MCOperand &ImmOp = Inst.getOperand(2);
    assert(ImmOp.isImm() && "expected immediate operand kind");
    const MCOperand &Reg1Op = Inst.getOperand(1);
    assert(Reg1Op.isReg() && "expected register operand kind");
    const MCOperand &Reg0Op = Inst.getOperand(0);
    assert(Reg0Op.isReg() && "expected register operand kind");


      tmpInst.setOpcode(Coffee::LW);
      tmpInst.addOperand(MCOperand::CreateReg(Reg0Op.getReg()));
      tmpInst.addOperand(MCOperand::CreateReg(Reg1Op.getReg()));
      tmpInst.addOperand(MCOperand::CreateImm(ImmOp.getImm()));
      Instructions.push_back(tmpInst);

}


void CoffeeAsmParser::expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                                  SmallVectorImpl<MCInst> &Instructions){

  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &RegOp = Inst.getOperand(0);
  assert(RegOp.isReg() && "expected register operand kind");

  int ImmValue = ImmOp.getImm();
  tmpInst.setLoc(IDLoc);
  // 16 bit unsigned
  if ( 0 <= ImmValue && ImmValue <= 65535) {
    tmpInst.setOpcode(Coffee::LLi);
    tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::CreateImm(ImmValue));
    Instructions.push_back(tmpInst);
  } /*else if ( ImmValue < 0 && ImmValue >= -32768) {
    tmpInst.setOpcode(Coffee::LLi); //TODO:no ADDiu64 in td files?
    tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::CreateImm(ImmValue));
    Instructions.push_back(tmpInst);
  } */else {
    // for any other value of j that is representable as a 32-bit integer.

    tmpInst.setOpcode(Coffee::LLi);
    tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::CreateImm(ImmValue & 0xffff));
    Instructions.push_back(tmpInst);
    tmpInst.clear();
    tmpInst.setOpcode(Coffee::LUi);
    tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::CreateImm((ImmValue & 0xffff0000) >> 16));
    tmpInst.setLoc(IDLoc);
    Instructions.push_back(tmpInst);
  }
}

void CoffeeAsmParser::expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){
    MCInst tmpInst;
     MCOperand &ExprOp = Inst.getOperand(1);
    assert(ExprOp.isExpr() && "expected immediate operand kind");

     MCOperand &RegOp = Inst.getOperand(0);
    assert(RegOp.isReg() && "expected register operand kind");


      const MCSymbolRefExpr* SymbolExpr = static_cast<const MCSymbolRefExpr*>(ExprOp.getExpr());

      const MCSymbolRefExpr* HIExpr = MCSymbolRefExpr::Create(&SymbolExpr->getSymbol(), MCSymbolRefExpr::VK_Coffee_ABS_HI, getContext());

      const MCSymbolRefExpr* LoExpr = MCSymbolRefExpr::Create(&SymbolExpr->getSymbol(), MCSymbolRefExpr::VK_Coffee_ABS_LO, getContext());

     tmpInst.setLoc(IDLoc);
      tmpInst.setOpcode(Coffee::LLi);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateExpr(LoExpr));
      Instructions.push_back(tmpInst);
      tmpInst.clear();

      tmpInst.setOpcode(Coffee::LUi);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateExpr(HIExpr));
      tmpInst.setLoc(IDLoc);
      Instructions.push_back(tmpInst);

}

void CoffeeAsmParser::expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){
  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &RegOp = Inst.getOperand(0);
  assert(RegOp.isReg() && "expected register operand kind");
  int ImmValue = ImmOp.getImm();
  // upper 16 bits of the integer in this range are already zero so
  // we can lli for that.
  tmpInst.setLoc(IDLoc);
  if ( 0 <= ImmValue && ImmValue <= 65535) {

    tmpInst.setOpcode(Coffee::LLi);
    tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::CreateImm(ImmValue));
    Instructions.push_back(tmpInst);
  } else {

      tmpInst.setOpcode(Coffee::LLi);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateImm(ImmValue & 0xffff));
      Instructions.push_back(tmpInst);

      tmpInst.clear();

      tmpInst.setOpcode(Coffee::LUi);
      tmpInst.addOperand(MCOperand::CreateReg(RegOp.getReg()));
      tmpInst.addOperand(MCOperand::CreateImm((ImmValue & 0xffff0000) >> 16));
      tmpInst.setLoc(IDLoc);
      Instructions.push_back(tmpInst);
  }
}

bool CoffeeAsmParser::
MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                        SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                        MCStreamer &Out, unsigned &ErrorInfo,
                        bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                              MatchingInlineAsm);

  switch (MatchResult) {
  default: break;
  case Match_Success: {
    if (needsExpansion(Inst)) {
      SmallVector<MCInst, 4> Instructions;
      expandInstruction(Inst, IDLoc, Instructions);
      for(unsigned i =0; i < Instructions.size(); i++){
        Out.EmitInstruction(Instructions[i]);
      }
    } else {
        Inst.setLoc(IDLoc);
        Out.EmitInstruction(Inst);
      }
    return false;
  }
  case Match_MissingFeature:
    Error(IDLoc, "instruction requires a CPU feature not currently enabled");
    return true;
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = ((CoffeeOperand*)Operands[ErrorInfo])->getStartLoc();
      if (ErrorLoc == SMLoc()) ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction");
  }
  return true;
}

int CoffeeAsmParser::matchRegisterName(StringRef Name) {

    int CC;

    CC = StringSwitch<unsigned>(Name)
            .Case("t0",  Coffee::T0)
            .Case("t1",  Coffee::T1)
            .Case("t2",  Coffee::T2)
            .Case("t3",  Coffee::T3)
            .Case("t4",  Coffee::T4)
            .Case("t5",  Coffee::T5)
            .Case("t6",  Coffee::T6)
            .Case("t7",  Coffee::T7)
            .Case("t8",  Coffee::T8)
            .Case("t9",  Coffee::T9)
            .Case("s0",  Coffee::S0)
            .Case("s1",  Coffee::S1)
            .Case("s2",  Coffee::S2)
            .Case("s3",  Coffee::S3)
            .Case("s4",  Coffee::S4)
            .Case("s5",  Coffee::S5)
            .Case("s6",  Coffee::S6)
            .Case("s7",  Coffee::S7)
            .Case("s8",  Coffee::S8)
            .Case("s9",  Coffee::S9)
            .Case("s10",  Coffee::S10)
            .Case("a0",  Coffee::A0)
            .Case("a1",  Coffee::A1)
            .Case("a2",  Coffee::A2)
            .Case("a3",  Coffee::A3)
            .Case("v0",  Coffee::V0)
            .Case("v1",  Coffee::V1)
            .Case("sp",  Coffee::SP)
            .Case("fp",  Coffee::FP)
            .Case("psr",  Coffee::PSR)
            .Case("spsr",  Coffee::SPSR)
            .Case("lr",  Coffee::LR)
            // register alias
            .Case("r0",  Coffee::T0)
            .Case("r1",  Coffee::T1)
            .Case("r2",  Coffee::T2)
            .Case("r3",  Coffee::T3)
            .Case("r4",  Coffee::T4)
            .Case("r5",  Coffee::T5)
            .Case("r6",  Coffee::T6)
            .Case("r7",  Coffee::T7)
            .Case("r8",  Coffee::T8)
            .Case("r9",  Coffee::T9)
            .Case("r10",  Coffee::S0)
            .Case("r11",  Coffee::S1)
            .Case("r12",  Coffee::S2)
            .Case("r13",  Coffee::S3)
            .Case("r14",  Coffee::S4)
            .Case("r15",  Coffee::S5)
            .Case("r16",  Coffee::S6)
            .Case("r17",  Coffee::S7)
            .Case("r18",  Coffee::S8)
            .Case("r19",  Coffee::S9)
            .Case("r20",  Coffee::S10)
            .Case("r21",  Coffee::A0)
            .Case("r22",  Coffee::A1)
            .Case("r23",  Coffee::A2)
            .Case("r24",  Coffee::A3)
            .Case("r25",  Coffee::V0)
            .Case("r26",  Coffee::V1)
            .Case("r27",  Coffee::SP)
            .Case("r28",  Coffee::FP)
            .Case("r29",  Coffee::PSR)
            .Case("r30",  Coffee::SPSR)
            .Case("r31",  Coffee::LR)


            .Case("c0",  Coffee::CR0)
            .Case("c1",  Coffee::CR1)
            .Case("c2",  Coffee::CR2)
            .Case("c3",  Coffee::CR3)
            .Case("c4",  Coffee::CR4)
            .Case("c5",  Coffee::CR5)
            .Case("c6",  Coffee::CR6)
            .Case("c7",  Coffee::CR7)

            .Default(-1);


    if (CC != -1)
        return CC;

    return -1;
}
void CoffeeAsmParser::setDefaultFpFormat() {

  if (isCoffee64() || isFP64())
    FpFormat = FP_FORMAT_D;
  else
    FpFormat = FP_FORMAT_S;
}

bool CoffeeAsmParser::requestsDoubleOperand(StringRef Mnemonic){

  bool IsDouble = StringSwitch<bool>(Mnemonic.lower())
    .Case("ldxc1", true)
    .Case("ldc1",  true)
    .Case("sdxc1", true)
    .Case("sdc1",  true)
    .Default(false);

  return IsDouble;
}
void CoffeeAsmParser::setFpFormat(StringRef Format) {

  FpFormat = StringSwitch<FpFormatTy>(Format.lower())
    .Case(".s",  FP_FORMAT_S)
    .Case(".d",  FP_FORMAT_D)
    .Case(".l",  FP_FORMAT_L)
    .Case(".w",  FP_FORMAT_W)
    .Default(FP_FORMAT_NONE);
}

bool CoffeeAssemblerOptions::setATReg(unsigned Reg) {
  if (Reg > 31)
    return false;

  aTReg = Reg;
  return true;
}

unsigned CoffeeAsmParser::getATReg() {
  unsigned Reg = Options.getATRegNum();
  
  return getReg(Coffee::GPRCRegClassID,Reg);
}

unsigned CoffeeAsmParser::getReg(int RC,int RegNo) {
  return *(getContext().getRegisterInfo().getRegClass(RC).begin() + RegNo);
}

int CoffeeAsmParser::matchRegisterByNumber(unsigned RegNum, StringRef Mnemonic) {

  if (Mnemonic.lower() == "rdhwr") {
    // at the moment only hwreg29 is supported
   /* if (RegNum != 29)
      return -1;
    return Coffee::HWR29;*/
  }

  if (RegNum > 31)
    return -1;

  // Coffee64 registers are numbered 1 after the 32-bit equivalents
  return getReg(Coffee::GPRCRegClassID, RegNum);
}

int CoffeeAsmParser::tryParseRegister(StringRef Mnemonic) {
  const AsmToken &Tok = Parser.getTok();
  int RegNum = -1;

  if (Tok.is(AsmToken::Identifier)) {
    std::string lowerCase = Tok.getString().lower();
    RegNum = matchRegisterName(lowerCase);
  } else if (Tok.is(AsmToken::Integer))
    RegNum = matchRegisterByNumber(static_cast<unsigned>(Tok.getIntVal()),
                                   Mnemonic.lower());
    else
      return RegNum;  //error
  // 64 bit div operations require Coffee::ZERO instead of Coffee::ZERO_64
 /* if (isCoffee64() && RegNum == Coffee::ZERO_64) {
    if (Mnemonic.find("ddiv") != StringRef::npos)
      RegNum = Coffee::ZERO;
  }*/
  return RegNum;
}

bool CoffeeAsmParser::
  tryParseRegisterOperand(SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                          StringRef Mnemonic){

  SMLoc S = Parser.getTok().getLoc();
  int RegNo = -1;

  // FIXME: we should make a more generic method for CCR
  if ((Mnemonic == "cfc1" || Mnemonic == "ctc1")
      && Operands.size() == 2 && Parser.getTok().is(AsmToken::Integer)){
    RegNo = Parser.getTok().getIntVal();  // get the int value
    // at the moment only fcc0 is supported
   // if (RegNo ==  0)
      //RegNo = Coffee::FCC0;
  } else
    RegNo = tryParseRegister(Mnemonic);
  if (RegNo == -1)
    return true;

  Operands.push_back(CoffeeOperand::CreateReg(RegNo, S,
      Parser.getTok().getLoc()));
  Parser.Lex(); // Eat register token.
  return false;
}

bool CoffeeAsmParser::ParseOperand(SmallVectorImpl<MCParsedAsmOperand*>&Operands,
                                 StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;
  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_ParseFail)
    return true;

  switch (getLexer().getKind()) {
  default:
    Error(Parser.getTok().getLoc(), "unexpected token in operand");
    return true;

  case AsmToken::Dollar: {
    // parse register
    SMLoc S = Parser.getTok().getLoc();
    Parser.Lex(); // Eat dollar token.
    // parse register operand
    if (!tryParseRegisterOperand(Operands, Mnemonic)) {
      if (getLexer().is(AsmToken::LParen)) {
        // check if it is indexed addressing operand
        Operands.push_back(CoffeeOperand::CreateToken("(", S));
        Parser.Lex(); // eat parenthesis
        if (getLexer().isNot(AsmToken::Dollar))
          return true;

        Parser.Lex(); // eat dollar
        if (tryParseRegisterOperand(Operands, Mnemonic))
          return true;

        if (!getLexer().is(AsmToken::RParen))
          return true;

        S = Parser.getTok().getLoc();
        Operands.push_back(CoffeeOperand::CreateToken(")", S));
        Parser.Lex();
      }
      return false;
    }
    // maybe it is a symbol reference
    StringRef Identifier;
    if (Parser.ParseIdentifier(Identifier))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    MCSymbol *Sym = getContext().GetOrCreateSymbol("$" + Identifier);

    // Otherwise create a symbol ref.
    const MCExpr *Res = MCSymbolRefExpr::Create(Sym, MCSymbolRefExpr::VK_None,
                                                getContext());

    Operands.push_back(CoffeeOperand::CreateImm(Res, S, E));
    return false;
  }
  case AsmToken::Identifier:
  {
      // quoted label names
     const MCExpr *IdVal;
     SMLoc S = Parser.getTok().getLoc();
     if (getParser().ParseExpression(IdVal))
       return true;

     if (IdVal->getKind() == MCExpr::Constant) {
     SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
     Operands.push_back(CoffeeOperand::CreateImm(IdVal, S, E));
     return false;
     } else if (IdVal->getKind() == MCExpr::SymbolRef) {

     // get the value this symbol is assigned , if it is register,

         if (static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().isVariable()) {
             const MCExpr* val = static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().getVariableValue();

             StringRef regstr = static_cast<const MCSymbolRefExpr*>(val)->getSymbol().getName();
             int RegNum = matchRegisterName(regstr.lower());
             if (RegNum != -1) {
                 SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
                 Operands.push_back(CoffeeOperand::CreateReg(RegNum, S, E));
             }
         } else if (static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().isDefined()) {

            /* if (Expr == 0)
               Inst.addOperand(MCOperand::CreateImm(0));
             else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
               Inst.addOperand(MCOperand::CreateImm(CE->getValue()));
             else
               Inst.addOperand(MCOperand::CreateExpr(Expr));*/

             Operands.push_back(CoffeeOperand::CreateExpr(IdVal));


         } else if (static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().isUndefined()) {
             StringRef Name = static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().getName();

             int RegNum = matchRegisterName(Name.lower());
             if (RegNum != -1) {
                 SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
                 Operands.push_back(CoffeeOperand::CreateReg(RegNum, S, E));
             } else {
                 MCSymbol *Sym = getContext().LookupSymbol(Name);
                 bool tes = Sym->isDefined();
                 if (Sym) {
                     Sym->setUsed(true);
                     Operands.push_back(CoffeeOperand::CreateExpr(IdVal));
                 }
             }

         } else {
             int k = 12;
         }

         return false;

     } else {
     return false;
     }
  }
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Integer:
  case AsmToken::String: {
     // quoted label names
    const MCExpr *IdVal;
    SMLoc S = Parser.getTok().getLoc();
    if (getParser().ParseExpression(IdVal))
      return true;
    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Operands.push_back(CoffeeOperand::CreateImm(IdVal, S, E));
    return false;
  }
  case AsmToken::Percent: {
    // it is a symbol reference or constant expression
    const MCExpr *IdVal;
    SMLoc S = Parser.getTok().getLoc(); // start location of the operand
    if (parseRelocOperand(IdVal))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    Operands.push_back(CoffeeOperand::CreateImm(IdVal, S, E));
    return false;
  } // case AsmToken::Percent
  } // switch(getLexer().getKind())
  return true;
}

bool CoffeeAsmParser::parseRelocOperand(const MCExpr *&Res) {

  Parser.Lex(); // eat % token
  const AsmToken &Tok = Parser.getTok(); // get next token, operation
  if (Tok.isNot(AsmToken::Identifier))
    return true;

  std::string Str = Tok.getIdentifier().str();

  Parser.Lex(); // eat identifier
  // now make expression from the rest of the operand
  const MCExpr *IdVal;
  SMLoc EndLoc;

  if (getLexer().getKind() == AsmToken::LParen) {
    while (1) {
      Parser.Lex(); // eat '(' token
      if (getLexer().getKind() == AsmToken::Percent) {
        Parser.Lex(); // eat % token
        const AsmToken &nextTok = Parser.getTok();
        if (nextTok.isNot(AsmToken::Identifier))
          return true;
        Str += "(%";
        Str += nextTok.getIdentifier();
        Parser.Lex(); // eat identifier
        if (getLexer().getKind() != AsmToken::LParen)
          return true;
      } else
        break;
    }
    if (getParser().ParseParenExpression(IdVal,EndLoc))
      return true;

    while (getLexer().getKind() == AsmToken::RParen)
      Parser.Lex(); // eat ')' token

  } else
    return true; // parenthesis must follow reloc operand

  // Check the type of the expression
  if (const MCConstantExpr *MCE = dyn_cast<MCConstantExpr>(IdVal)) {
    // it's a constant, evaluate lo or hi value
    int Val = MCE->getValue();
    if (Str == "lo") {
      Val = Val & 0xffff;
    } else if (Str == "hi") {
      Val = (Val & 0xffff0000) >> 16;
    }
    Res = MCConstantExpr::Create(Val, getContext());
    return false;
  }

  if (const MCSymbolRefExpr *MSRE = dyn_cast<MCSymbolRefExpr>(IdVal)) {
    // it's a symbol, create symbolic expression from symbol
    StringRef Symbol = MSRE->getSymbol().getName();
    MCSymbolRefExpr::VariantKind VK = getVariantKind(Str);
    Res = MCSymbolRefExpr::Create(Symbol,VK,getContext());
    return false;
  }
  return true;
}

bool CoffeeAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                  SMLoc &EndLoc) {

  StartLoc = Parser.getTok().getLoc();
  RegNo = tryParseRegister("");
  EndLoc = Parser.getTok().getLoc();
  return (RegNo == (unsigned)-1);
}

bool CoffeeAsmParser::parseMemOffset(const MCExpr *&Res) {

  SMLoc S;

  switch(getLexer().getKind()) {
  default:
    return true;
  case AsmToken::Integer:
  case AsmToken::Minus:
  case AsmToken::Plus:
    return (getParser().ParseExpression(Res));
  case AsmToken::Percent:
    return parseRelocOperand(Res);
  case AsmToken::LParen:
    return false;  // it's probably assuming 0
  }
  return true;
}

CoffeeAsmParser::OperandMatchResultTy CoffeeAsmParser::parseMemOperand(
               SmallVectorImpl<MCParsedAsmOperand*>&Operands) {

  const MCExpr *IdVal = 0;
  SMLoc S;
  // first operand is the offset
  S = Parser.getTok().getLoc();

  if (parseMemOffset(IdVal))
    return MatchOperand_ParseFail;

  const AsmToken &Tok = Parser.getTok(); // get next token
  if (Tok.isNot(AsmToken::LParen)) {
    CoffeeOperand *Mnemonic = static_cast<CoffeeOperand*>(Operands[0]);
    if (Mnemonic->getToken() == "la") {
      SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer()-1);
      Operands.push_back(CoffeeOperand::CreateImm(IdVal, S, E));
      return MatchOperand_Success;
    }
    Error(Parser.getTok().getLoc(), "'(' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat '(' token.

  const AsmToken &Tok1 = Parser.getTok(); // get next token
  if (Tok1.is(AsmToken::Dollar)) {
    Parser.Lex(); // Eat '$' token.
    if (tryParseRegisterOperand(Operands,"")) {
      Error(Parser.getTok().getLoc(), "unexpected token in operand");
      return MatchOperand_ParseFail;
    }

  } else {
    Error(Parser.getTok().getLoc(), "unexpected token in operand");
    return MatchOperand_ParseFail;
  }

  const AsmToken &Tok2 = Parser.getTok(); // get next token
  if (Tok2.isNot(AsmToken::RParen)) {
    Error(Parser.getTok().getLoc(), "')' expected");
    return MatchOperand_ParseFail;
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  Parser.Lex(); // Eat ')' token.

  if (IdVal == 0)
    IdVal = MCConstantExpr::Create(0, getContext());

  // now replace register operand with the mem operand
  CoffeeOperand* op = static_cast<CoffeeOperand*>(Operands.back());
  int RegNo = op->getReg();
  // remove register from operands
  Operands.pop_back();
  // and add memory operand
  Operands.push_back(CoffeeOperand::CreateMem(RegNo, IdVal, S, E));
  delete op;
  return MatchOperand_Success;
}


CoffeeAsmParser::OperandMatchResultTy CoffeeAsmParser::parseAddrOperand(
        SmallVectorImpl<MCParsedAsmOperand*>&Operands) {

    const MCExpr *IdVal;
    SMLoc S = Parser.getTok().getLoc();
    if (getParser().ParseExpression(IdVal))
      return MatchOperand_ParseFail;

    if (IdVal->getKind() == MCExpr::SymbolRef) {

        CoffeeOperand *Mnemonic = static_cast<CoffeeOperand*>(Operands[0]);
        bool Defined = static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().isDefined();

        if (Mnemonic->getToken() == "ldra" )
            if (Defined) {
                Operands.push_back(CoffeeOperand::CreateAddr(IdVal));
                return MatchOperand_Success;
            } else {
                StringRef Name = static_cast<const MCSymbolRefExpr*>(IdVal)->getSymbol().getName();
                MCSymbol *Sym = getContext().LookupSymbol(Name);
                if (Sym) {
                    Sym->setUsed(true);
                    Operands.push_back(CoffeeOperand::CreateAddr(IdVal));
                    return MatchOperand_Success;
                } else {
                    SMLoc Loc = getLexer().getLoc();
                    Parser.EatToEndOfStatement();
                    Error(Loc, "unexpected token in argument list");
                    return MatchOperand_ParseFail;
                }
            }
    }

    return MatchOperand_ParseFail;
}


MCSymbolRefExpr::VariantKind CoffeeAsmParser::getVariantKind(StringRef Symbol) {

  MCSymbolRefExpr::VariantKind VK
                   = StringSwitch<MCSymbolRefExpr::VariantKind>(Symbol)
    .Case("hi",          MCSymbolRefExpr::VK_Coffee_ABS_HI)
    .Case("lo",          MCSymbolRefExpr::VK_Coffee_ABS_LO)
    .Case("gp_rel",      MCSymbolRefExpr::VK_Coffee_GPREL)
    .Case("call16",      MCSymbolRefExpr::VK_Coffee_GOT_CALL)
    .Case("got",         MCSymbolRefExpr::VK_Coffee_GOT)
    .Case("tlsgd",       MCSymbolRefExpr::VK_Coffee_TLSGD)
    .Case("tlsldm",      MCSymbolRefExpr::VK_Coffee_TLSLDM)
    .Case("dtprel_hi",   MCSymbolRefExpr::VK_Coffee_DTPREL_HI)
    .Case("dtprel_lo",   MCSymbolRefExpr::VK_Coffee_DTPREL_LO)
    .Case("gottprel",    MCSymbolRefExpr::VK_Coffee_GOTTPREL)
    .Case("tprel_hi",    MCSymbolRefExpr::VK_Coffee_TPREL_HI)
    .Case("tprel_lo",    MCSymbolRefExpr::VK_Coffee_TPREL_LO)
    .Case("got_disp",    MCSymbolRefExpr::VK_Coffee_GOT_DISP)
    .Case("got_page",    MCSymbolRefExpr::VK_Coffee_GOT_PAGE)
    .Case("got_ofst",    MCSymbolRefExpr::VK_Coffee_GOT_OFST)
    .Case("hi(%neg(%gp_rel",    MCSymbolRefExpr::VK_Coffee_GPOFF_HI)
    .Case("lo(%neg(%gp_rel",    MCSymbolRefExpr::VK_Coffee_GPOFF_LO)
    .Default(MCSymbolRefExpr::VK_None);

  return VK;
}

static int ConvertCcString(StringRef CondString) {
  int CC = StringSwitch<unsigned>(CondString)
      .Case(".f",    0)
      .Case(".un",   1)
      .Case(".eq",   2)
      .Case(".ueq",  3)
      .Case(".olt",  4)
      .Case(".ult",  5)
      .Case(".ole",  6)
      .Case(".ule",  7)
      .Case(".sf",   8)
      .Case(".ngle", 9)
      .Case(".seq",  10)
      .Case(".ngl",  11)
      .Case(".lt",   12)
      .Case(".nge",  13)
      .Case(".le",   14)
      .Case(".ngt",  15)
      .Default(-1);

  return CC;
}

bool CoffeeAsmParser::
parseMathOperation(StringRef Name, SMLoc NameLoc,
                   SmallVectorImpl<MCParsedAsmOperand*> &Operands) {
  // split the format
  size_t Start = Name.find('.'), Next = Name.rfind('.');
  StringRef Format1 = Name.slice(Start, Next);
  // and add the first format to the operands
  Operands.push_back(CoffeeOperand::CreateToken(Format1, NameLoc));
  // now for the second format
  StringRef Format2 = Name.slice(Next, StringRef::npos);
  Operands.push_back(CoffeeOperand::CreateToken(Format2, NameLoc));

  // set the format for the first register
  setFpFormat(Format1);

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.EatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    if (getLexer().isNot(AsmToken::Comma)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.EatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");

    }
    Parser.Lex();  // Eat the comma.

    //set the format for the first register
    setFpFormat(Format2);

    // Parse and remember the operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.EatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.EatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool CoffeeAsmParser::
ParseInstruction(ParseInstructionInfo &Info, StringRef Name, SMLoc NameLoc,
                 SmallVectorImpl<MCParsedAsmOperand*> &Operands) {
  // floating point instructions: should register be treated as double?
  if (requestsDoubleOperand(Name)) {
    setFpFormat(FP_FORMAT_D);
  Operands.push_back(CoffeeOperand::CreateToken(Name, NameLoc));
  }
  else {
    setDefaultFpFormat();
    // Create the leading tokens for the mnemonic, split by '.' characters.
    size_t Start = 0, Next = Name.find('.');
    StringRef Mnemonic = Name.slice(Start, Next);

    Operands.push_back(CoffeeOperand::CreateToken(Mnemonic, NameLoc));

    if (Next != StringRef::npos) {
      // there is a format token in mnemonic
      // StringRef Rest = Name.slice(Next, StringRef::npos);
      size_t Dot = Name.find('.', Next+1);
      StringRef Format = Name.slice(Next, Dot);
      if (Dot == StringRef::npos) //only one '.' in a string, it's a format
        Operands.push_back(CoffeeOperand::CreateToken(Format, NameLoc));
      else {
        if (Name.startswith("c.")){
          // floating point compare, add '.' and immediate represent for cc
          Operands.push_back(CoffeeOperand::CreateToken(".", NameLoc));
          int Cc = ConvertCcString(Format);
          if (Cc == -1) {
            return Error(NameLoc, "Invalid conditional code");
          }
          SMLoc E = SMLoc::getFromPointer(
              Parser.getTok().getLoc().getPointer() -1 );
          Operands.push_back(CoffeeOperand::CreateImm(
              MCConstantExpr::Create(Cc, getContext()), NameLoc, E));
        } else {
          // trunc, ceil, floor ...
          return parseMathOperation(Name, NameLoc, Operands);
        }

        // the rest is a format
        Format = Name.slice(Dot, StringRef::npos);
        Operands.push_back(CoffeeOperand::CreateToken(Format, NameLoc));
      }

      setFpFormat(Format);
    }
  }

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.EatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    while (getLexer().is(AsmToken::Comma) ) {
      Parser.Lex();  // Eat the comma.

      // Parse and remember the operand.
      if (ParseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        Parser.EatToEndOfStatement();
        return Error(Loc, "unexpected token in argument list");
      }
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.EatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool CoffeeAsmParser::reportParseError(StringRef ErrorMsg) {
   SMLoc Loc = getLexer().getLoc();
   Parser.EatToEndOfStatement();
   return Error(Loc, ErrorMsg);
}

bool CoffeeAsmParser::parseSetNoAtDirective() {
  // line should look like:
  //  .set noat
  // set at reg to 0
  Options.setATReg(0);
  // eat noat
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}
bool CoffeeAsmParser::parseSetAtDirective() {
  // line can be
  //  .set at - defaults to $1
  // or .set at=$reg
  getParser().Lex();
  if (getLexer().is(AsmToken::EndOfStatement)) {
    Options.setATReg(1);
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  } else if (getLexer().is(AsmToken::Equal)) {
    getParser().Lex(); //eat '='
    if (getLexer().isNot(AsmToken::Dollar)) {
      reportParseError("unexpected token in statement");
      return false;
    }
    Parser.Lex(); // eat '$'
    if (getLexer().isNot(AsmToken::Integer)) {
      reportParseError("unexpected token in statement");
      return false;
    }
    const AsmToken &Reg = Parser.getTok();
    if (!Options.setATReg(Reg.getIntVal())) {
      reportParseError("unexpected token in statement");
      return false;
    }
    getParser().Lex(); //eat reg

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token in statement");
      return false;
     }
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  } else {
    reportParseError("unexpected token in statement");
    return false;
  }
}

bool CoffeeAsmParser::parseSetReorderDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setReorder();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool CoffeeAsmParser::parseSetNoReorderDirective() {
    Parser.Lex();
    // if this is not the end of the statement, report error
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token in statement");
      return false;
    }
    Options.setNoreorder();
    Parser.Lex(); // Consume the EndOfStatement
    return false;
}

bool CoffeeAsmParser::parseSetMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setMacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool CoffeeAsmParser::parseSetNoMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  if (Options.isReorder()) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  Options.setNomacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}
bool CoffeeAsmParser::parseDirectiveSet() {

  // get next token
  const AsmToken &Tok = Parser.getTok();

  if (Tok.getString() == "noat") {
    return parseSetNoAtDirective();
  } else if (Tok.getString() == "at") {
    return parseSetAtDirective();
  } else if (Tok.getString() == "reorder") {
    return parseSetReorderDirective();
  } else if (Tok.getString() == "noreorder") {
    return parseSetNoReorderDirective();
  } else if (Tok.getString() == "macro") {
    return parseSetMacroDirective();
  } else if (Tok.getString() == "nomacro") {
    return parseSetNoMacroDirective();
  } else if (Tok.getString() == "noCoffee16") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  } else if (Tok.getString() == "nomicroCoffee") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }
  return true;
}

bool CoffeeAsmParser::parseDirectiveWord(unsigned Size, SMLoc L) {
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    for (;;) {
      const MCExpr *Value;
      if (getParser().ParseExpression(Value))
        return true;

      getParser().getStreamer().EmitValue(Value, Size, 0/*addrspace*/);

      if (getLexer().is(AsmToken::EndOfStatement))
        break;

      // FIXME: Improve diagnostic.
      if (getLexer().isNot(AsmToken::Comma))
        return Error(L, "unexpected token in directive");
      Parser.Lex();
    }
  }

  Parser.Lex();
  return false;
}

bool CoffeeAsmParser::ParseDirective(AsmToken DirectiveID) {

  if (DirectiveID.getString() == ".ent") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".word") {
     return parseDirectiveWord(4, DirectiveID.getLoc());;
  }

  if (DirectiveID.getString() == ".proc") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".endproc") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".end") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".frame") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".code") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".code32") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".set") {
    return parseDirectiveSet();
  }

  if (DirectiveID.getString() == ".fmask") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".mask") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".gpword") {
    // ignore this directive for now
    Parser.EatToEndOfStatement();
    return false;
  }

  return true;
}

extern "C" void LLVMInitializeCoffeeAsmLexer();

extern "C" void LLVMInitializeCoffeeAsmParser() {
  RegisterMCAsmParser<CoffeeAsmParser> X(TheCoffeeTarget);
  LLVMInitializeCoffeeAsmLexer();
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "CoffeeGenAsmMatcher.inc"
