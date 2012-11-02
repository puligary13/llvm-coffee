//===-- Coffee.td - Describe the Coffee Target Machine -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is the top level entry point for the Coffee target.
//
//===----------------------------------------------------------------------===//

// Get the target-independent interfaces which we are implementing.
//
include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Register File Description
//===----------------------------------------------------------------------===//

include "CoffeeRegisterInfo.td"
include "CoffeeSchedule.td"
include "CoffeeInstrInfo.td"

//===----------------------------------------------------------------------===//
// Calling Conventions
//===----------------------------------------------------------------------===//

include "CoffeeCallingConv.td"

def CoffeeInstrInfo : InstrInfo {
  let isLittleEndianEncoding = 1;
}

def CoffeeAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  bit isMCAsmWriter = 1;
}

def Coffee : Target {
  // Information about the instructions.
  let InstructionSet = CoffeeInstrInfo;
  
  let AssemblyWriters = [CoffeeAsmWriter];
}
