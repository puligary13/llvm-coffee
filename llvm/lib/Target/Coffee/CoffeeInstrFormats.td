//===- CoffeeInstrFormats.td - Coffee Instruction Formats --*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//
// Coffee instruction formats


class InstCoffee<bits<6> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction {
  field bits<32> Inst;

  let Namespace = "Coffee";
  let Inst{31-26} = opcode;
  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;
}

// cex is set to 1
class InstCoffee_1<bits<6> opcode, bits<3> creg, bits<3> cond, dag outs, dag ins, string asmstr, list<dag> pattern = []>
        : InstCoffee<opcode, outs, ins, asmstr, pattern> {

  let Inst{25} = 1;
  let Inst{24-22} = creg;
  let Inst{21-19} = cond;
}

// cex is set to 0
class InstCoffee_0<bits<6> opcode, dag outs, dag ins, string asmstr, list<dag> pattern = []>
        : InstCoffee<opcode, outs, ins, asmstr, pattern> {
  let Inst{25} = 0;
}

// for pseudo instruction
class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
   : InstCoffee_0<0b000000,outs, ins, asmstr, pattern>;
