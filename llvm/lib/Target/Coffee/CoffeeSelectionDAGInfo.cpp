//===-- CoffeeSelectionDAGInfo.cpp - Coffee SelectionDAG Info ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Coffee-selectiondag-info"
#include "CoffeeTargetMachine.h"
using namespace llvm;

CoffeeSelectionDAGInfo::CoffeeSelectionDAGInfo(const CoffeeTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

CoffeeSelectionDAGInfo::~CoffeeSelectionDAGInfo() {
}
