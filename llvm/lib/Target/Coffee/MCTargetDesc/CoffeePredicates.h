//===-- CoffeePredicates.h - Coffee Branch Predicate Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the Coffee branch predicates.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_Coffee_CoffeePREDICATES_H
#define LLVM_TARGET_Coffee_CoffeePREDICATES_H

namespace llvm {
namespace Coffee {
  /// Predicate - These are "(BI << 5) | BO"  for various predicates.
  enum Predicate {
    PRED_ALWAYS = (0 << 5) | 20,
    PRED_LT     = (0 << 5) | 12,
    PRED_LE     = (1 << 5) |  4,
    PRED_EQ     = (2 << 5) | 12,
    PRED_GE     = (0 << 5) |  4,
    PRED_GT     = (1 << 5) | 12,
    PRED_NE     = (2 << 5) |  4,
    PRED_UN     = (3 << 5) | 12,
    PRED_NU     = (3 << 5) |  4
  };
  
  /// Invert the specified predicate.  != -> ==, < -> >=.
  Predicate InvertPredicate(Predicate Opcode);
}
}

#endif
