//===- CoffeeCallingConv.td - Calling Conventions for Coffee -*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the Coffee 32- and 64-bit
// architectures.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Return-value convention for Coffee
// the calling convention is defnined in coffeeisellowering.cpp
def RetCC_Coffee : CallingConv<[
  CCIfType<[i32], CCAssignToReg<[V0, V1, A0, A1]>>
]>;



//def CSR_COFFEE : CalleeSavedRegs<(add LR, R11, R10, R9, R8, R7, R6, R5, R4)>;

def CSR_COFFEE : CalleeSavedRegs<(add LR, FP,
                                  (sequence "S%u", 11, 0))>;
