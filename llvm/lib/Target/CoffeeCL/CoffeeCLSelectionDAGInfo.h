//===-- CoffeeCLSelectionDAGInfo.h - CoffeeCL SelectionDAG Info -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the CoffeeCL subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeCLCSELECTIONDAGINFO_H
#define CoffeeCLCSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class CoffeeCLTargetMachine;

class CoffeeCLSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit CoffeeCLSelectionDAGInfo(const CoffeeCLTargetMachine &TM);
  ~CoffeeCLSelectionDAGInfo();
};

}

#endif
