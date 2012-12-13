//===-- CoffeeAsmLexer.cpp - Tokenize Coffee assembly to AsmTokens --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CoffeeBaseInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCTargetAsmLexer.h"

#include "llvm/Support/TargetRegistry.h"

#include "llvm/ADT/StringSwitch.h"

#include <string>
#include <map>

using namespace llvm;

namespace {

class CoffeeAsmLexer : public MCTargetAsmLexer {
  const MCAsmInfo &AsmInfo;

  const AsmToken &lexDefinite() {
    return getLexer()->Lex();
  }

  AsmToken LexTokenUAL();
protected:
  typedef std::map <std::string, unsigned> rmap_ty;

  rmap_ty RegisterMap;

  void InitRegisterMap(const MCRegisterInfo *info) {
    unsigned numRegs = info->getNumRegs();

    for (unsigned i = 0; i < numRegs; ++i) {
      const char *regName = info->getName(i);
      if (regName)
        RegisterMap[regName] = i;
    }
  }

  unsigned MatchRegisterName(StringRef Name) {
    rmap_ty::iterator iter = RegisterMap.find(Name.str());
    if (iter != RegisterMap.end())
      return iter->second;
    else
      return 0;
  }

  AsmToken LexToken() {
    if (!Lexer) {
      SetError(SMLoc(), "No MCAsmLexer installed");
      return AsmToken(AsmToken::Error, "", 0);
    }

    switch (AsmInfo.getAssemblerDialect()) {
    default:
      SetError(SMLoc(), "Unhandled dialect");
      return AsmToken(AsmToken::Error, "", 0);
    case 0:
      return LexTokenUAL();
    }
  }
public:
  CoffeeAsmLexer(const Target &T, const MCRegisterInfo &MRI, const MCAsmInfo &MAI)
    : MCTargetAsmLexer(T), AsmInfo(MAI) {
       InitRegisterMap(&MRI);
  }
};


} // end anonymous namespace

AsmToken CoffeeAsmLexer::LexTokenUAL() {
  const AsmToken &lexedToken = lexDefinite();

  switch (lexedToken.getKind()) {
  default: break;
  case AsmToken::Error:
    SetError(Lexer->getErrLoc(), Lexer->getErr());
    break;
  case AsmToken::Identifier: {
    std::string lowerCase = lexedToken.getString().lower();

    unsigned regID = MatchRegisterName(lowerCase);
    // Check for register aliases.
    //   r13 -> sp
    //   r14 -> lr
    //   r15 -> pc
    //   ip  -> r12
    //   FIXME: Some assemblers support lots of others. Do we want them all?
    if (!regID) {
      regID = StringSwitch<unsigned>(lowerCase)
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
              .Case("s11",  Coffee::S11)
              .Case("a0",  Coffee::A0)
              .Case("a1",  Coffee::A1)
              .Case("a2",  Coffee::A2)
              .Case("a3",  Coffee::A3)
              .Case("v0",  Coffee::V0)
              .Case("v1",  Coffee::V1)
              .Case("gp",  Coffee::GP)
              .Case("sp",  Coffee::SP)
              .Case("fp",  Coffee::FP)
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
              .Case("r21",  Coffee::S11)
              .Case("r22",  Coffee::A0)
              .Case("r23",  Coffee::A1)
              .Case("r24",  Coffee::A2)
              .Case("r25",  Coffee::A3)
              .Case("r26",  Coffee::V0)
              .Case("r27",  Coffee::V1)
              .Case("r28",  Coffee::GP)
              .Case("r29",  Coffee::SP)
              .Case("r30",  Coffee::FP)
              .Case("r31",  Coffee::LR)

              .Case("cr0",  Coffee::CR0)
              .Case("c0",  Coffee::CR0)
              .Case("cr1",  Coffee::CR1)
              .Case("c1",  Coffee::CR1)
              .Case("cr2",  Coffee::CR2)
              .Case("c2",  Coffee::CR2)
              .Case("cr3",  Coffee::CR3)
              .Case("c3",  Coffee::CR3)

              .Case("cr4",  Coffee::CR4)
              .Case("c4",  Coffee::CR4)
              .Case("cr5",  Coffee::CR5)
              .Case("c5",  Coffee::CR5)
              .Case("cr6",  Coffee::CR6)
              .Case("c6",  Coffee::CR6)
              .Case("cr7",  Coffee::CR7)
              .Case("c7",  Coffee::CR7)
        .Default(0);
    }

    if (regID)
      return AsmToken(AsmToken::Register,
                      lexedToken.getString(),
                      static_cast<int64_t>(regID));
  }
  }

  return AsmToken(lexedToken);
}

extern "C" void LLVMInitializeCoffeeAsmLexer() {
  RegisterMCAsmLexer<CoffeeAsmLexer> X(TheCoffeeTarget);
}

