//===-- CoffeePredicates.cpp - Coffee Branch Predicate Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Coffee branch predicates.
//
//===----------------------------------------------------------------------===//

#include "CoffeePredicates.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
using namespace llvm;

Coffee::Predicate Coffee::InvertPredicate(Coffee::Predicate Opcode) {
  switch (Opcode) {
  default: llvm_unreachable("Unknown Coffee branch opcode!");
  case Coffee::PRED_EQ: return Coffee::PRED_NE;
  case Coffee::PRED_NE: return Coffee::PRED_EQ;
  case Coffee::PRED_LT: return Coffee::PRED_GE;
  case Coffee::PRED_GE: return Coffee::PRED_LT;
  case Coffee::PRED_GT: return Coffee::PRED_LE;
  case Coffee::PRED_LE: return Coffee::PRED_GT;
  case Coffee::PRED_NU: return Coffee::PRED_UN;
  case Coffee::PRED_UN: return Coffee::PRED_NU;
  }
}
