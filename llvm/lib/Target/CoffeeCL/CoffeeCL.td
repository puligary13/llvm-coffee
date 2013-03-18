//===-- CoffeeCL.td - Describe the CoffeeCL Target Machine -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is the top level entry point for the CoffeeCL target.
//
//===----------------------------------------------------------------------===//

// Get the target-independent interfaces which we are implementing.
//
include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Register File Description
//===----------------------------------------------------------------------===//

include "CoffeeCLRegisterInfo.td"
include "CoffeeCLSchedule.td"
include "CoffeeCLInstrInfo.td"

//===----------------------------------------------------------------------===//
// Calling Conventions
//===----------------------------------------------------------------------===//


def Feature32         : SubtargetFeature<"coffeecl32", "CoffeeCLABI", "ABI32",
                                "Enable CoffeeCL ABI32">;

def COP0 : SubtargetFeature<"fpu0", "IsCoProcessor0", "true",
                                   "Enable FPU 0">;

def COP1 : SubtargetFeature<"fpu1", "IsCoProcessor1", "true",
                                   "Enable FPU 1">;


def COP2 : SubtargetFeature<"fpu2", "IsCoProcessor2", "true",
                                   "Enable FPU 2">;

def COP3 : SubtargetFeature<"fpu3", "IsCoProcessor3", "true",
                                   "Enable FPU 3">;

include "CoffeeCLCallingConv.td"

def CoffeeCLInstrInfo : InstrInfo {
}

//===----------------------------------------------------------------------===//
// CoffeeCL processors supported.
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, CoffeeCLGenericItineraries, Features>;

def : Proc<"coffeecl32", [Feature32]>;


def CoffeeCLAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  bit isMCAsmWriter = 1;
}

def CoffeeCLAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def CoffeeCLAsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def CoffeeCL : Target {
  // Information about the instructions.
  let InstructionSet = CoffeeCLInstrInfo;
  
  let AssemblyWriters = [CoffeeCLAsmWriter];
  let AssemblyParsers = [CoffeeCLAsmParser];
  let AssemblyParserVariants = [CoffeeCLAsmParserVariant];
}
