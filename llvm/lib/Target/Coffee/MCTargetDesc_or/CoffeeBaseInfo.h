//===-- CoffeeBaseInfo.h - Top level definitions for Coffee -----------*- C++ -*-===//
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
// As such, it deliberately does not include references to LLVM core
// code gen types, passes, etc..
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeBASEINFO_H
#define CoffeeBASEINFO_H

#include "CoffeeMCTargetDesc.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// getCoffeeRegisterNumbering - Given the enum value for some register, e.g.
/// Coffee::F14, return the number that it corresponds to (e.g. 14).
inline static unsigned getCoffeeRegisterNumbering(unsigned RegEnum) {
  using namespace Coffee;
  switch (RegEnum) {
  case 0: return 0;
  case A0 : return  0;
  case A1 : return  1;
  case A2 : return  2;
  case A3 : return  3;
  case V0 : return  4;
  case V1 : return  5;
  case T0 : return  6;
  case T1 : return  7;
  case T2 : return  8;
  case T3 : return  9;
  case T4: return 10;
  case T5: return 11;
  case T6: return 12;
  case T7: return 13;
  case T8: return 14;
  case T9: return 15;
  case S0: return 16;
  case S1: return 17;
  case S2: return 18;
  case S3: return 19;
  case S4: return 20;
  case S5: return 21;
  case S6: return 22;
  case S7: return 23;
  case S8: return 24;
  case S9: return 25;
  case S10: return 26;
  case S11: return 27;
  case GP: return 28;
  case SP: return 29;
  case FP: return 30;
  case LR: return 31;
  default:
    llvm_unreachable("Unhandled reg in CoffeeRegisterInfo::getRegisterNumbering!");
  }
}

} // end namespace llvm;

#endif
