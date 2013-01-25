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


def Feature32         : SubtargetFeature<"coffee32", "CoffeeABI", "ABI32",
                                "Enable Coffee ABI32">;

def COP0 : SubtargetFeature<"fpu0", "IsCoProcessor0", "true",
                                   "Enable FPU 0">;

def COP1 : SubtargetFeature<"fpu1", "IsCoProcessor1", "true",
                                   "Enable FPU 1">;


def COP2 : SubtargetFeature<"fpu2", "IsCoProcessor2", "true",
                                   "Enable FPU 2">;

def COP3 : SubtargetFeature<"fpu3", "IsCoProcessor3", "true",
                                   "Enable FPU 3">;

include "CoffeeCallingConv.td"

def CoffeeInstrInfo : InstrInfo {
}

//===----------------------------------------------------------------------===//
// Coffee processors supported.
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, CoffeeGenericItineraries, Features>;

def : Proc<"coffee32", [Feature32]>;


def CoffeeAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  bit isMCAsmWriter = 1;
}

def CoffeeAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def CoffeeAsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def Coffee : Target {
  // Information about the instructions.
  let InstructionSet = CoffeeInstrInfo;
  
  let AssemblyWriters = [CoffeeAsmWriter];
  let AssemblyParsers = [CoffeeAsmParser];
  let AssemblyParserVariants = [CoffeeAsmParserVariant];
}
