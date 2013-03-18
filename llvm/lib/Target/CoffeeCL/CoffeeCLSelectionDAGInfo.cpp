//===-- CoffeeCLSelectionDAGInfo.cpp - CoffeeCL SelectionDAG Info ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeCLSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "CoffeeCL-selectiondag-info"
#include "CoffeeCLTargetMachine.h"
using namespace llvm;

CoffeeCLSelectionDAGInfo::CoffeeCLSelectionDAGInfo(const CoffeeCLTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

CoffeeCLSelectionDAGInfo::~CoffeeCLSelectionDAGInfo() {
}
