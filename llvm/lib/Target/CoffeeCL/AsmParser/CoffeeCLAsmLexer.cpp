//===-- CoffeeCLAsmLexer.cpp - Tokenize CoffeeCL assembly to AsmTokens --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CoffeeCLBaseInfo.h"

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

class CoffeeCLAsmLexer : public MCTargetAsmLexer {
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
  CoffeeCLAsmLexer(const Target &T, const MCRegisterInfo &MRI, const MCAsmInfo &MAI)
    : MCTargetAsmLexer(T), AsmInfo(MAI) {
       InitRegisterMap(&MRI);
  }
};


} // end anonymous namespace

AsmToken CoffeeCLAsmLexer::LexTokenUAL() {
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
              .Case("t0",  CoffeeCL::T0)
              .Case("t1",  CoffeeCL::T1)
              .Case("t2",  CoffeeCL::T2)
              .Case("t3",  CoffeeCL::T3)
              .Case("t4",  CoffeeCL::T4)
              .Case("t5",  CoffeeCL::T5)
              .Case("t6",  CoffeeCL::T6)
              .Case("t7",  CoffeeCL::T7)
              .Case("t8",  CoffeeCL::T8)
              .Case("t9",  CoffeeCL::T9)
              .Case("s0",  CoffeeCL::S0)
              .Case("s1",  CoffeeCL::S1)
              .Case("s2",  CoffeeCL::S2)
              .Case("s3",  CoffeeCL::S3)
              .Case("s4",  CoffeeCL::S4)
              .Case("s5",  CoffeeCL::S5)
              .Case("s6",  CoffeeCL::S6)
              .Case("s7",  CoffeeCL::S7)
              .Case("s8",  CoffeeCL::S8)
              .Case("s9",  CoffeeCL::S9)

              // register alias
              .Case("r0",  CoffeeCL::T0)
              .Case("r1",  CoffeeCL::T1)
              .Case("r2",  CoffeeCL::T2)
              .Case("r3",  CoffeeCL::T3)
              .Case("r4",  CoffeeCL::T4)
              .Case("r5",  CoffeeCL::T5)
              .Case("r6",  CoffeeCL::T6)
              .Case("r7",  CoffeeCL::T7)
              .Case("r8",  CoffeeCL::T8)
              .Case("r9",  CoffeeCL::T9)
              .Case("r10",  CoffeeCL::S0)
              .Case("r11",  CoffeeCL::S1)
              .Case("r12",  CoffeeCL::S2)
              .Case("r13",  CoffeeCL::S3)
              .Case("r14",  CoffeeCL::S4)
              .Case("r15",  CoffeeCL::S5)
              .Case("r16",  CoffeeCL::S6)
              .Case("r17",  CoffeeCL::S7)
              .Case("r18",  CoffeeCL::S8)
              .Case("r19",  CoffeeCL::S9)
              .Case("r20",  CoffeeCL::S10)


              .Case("c0",  CoffeeCL::CR0)
              .Case("c1",  CoffeeCL::CR1)
              .Case("c2",  CoffeeCL::CR2)
              .Case("c3",  CoffeeCL::CR3)
              .Case("c4",  CoffeeCL::CR4)
              .Case("c5",  CoffeeCL::CR5)
              .Case("c6",  CoffeeCL::CR6)
              .Case("c7",  CoffeeCL::CR7)
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

extern "C" void LLVMInitializeCoffeeCLAsmLexer() {
  RegisterMCAsmLexer<CoffeeCLAsmLexer> X(TheCoffeeCLTarget);
}

