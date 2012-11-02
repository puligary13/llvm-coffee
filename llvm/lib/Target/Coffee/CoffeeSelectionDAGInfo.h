//===-- CoffeeSelectionDAGInfo.h - Coffee SelectionDAG Info -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the Coffee subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeCSELECTIONDAGINFO_H
#define CoffeeCSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class CoffeeTargetMachine;

class CoffeeSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit CoffeeSelectionDAGInfo(const CoffeeTargetMachine &TM);
  ~CoffeeSelectionDAGInfo();
};

}

#endif
