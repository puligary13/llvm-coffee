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

// Format specifies the encoding used by the instruction.  This is part of the
// ad-hoc solution used to emit machine instruction encodings by our machine
// code emitter.
class Format<bits<4> val> {
  bits<4> Value = val;
}


def Pseudo    : Format<0>;
def FrmR      : Format<1>;
def FrmI      : Format<2>;
def FrmJ      : Format<3>;
def FrmFR     : Format<4>;
def FrmFI     : Format<5>;
def FrmOther  : Format<6>; // Instruction w/ a custom format


class InstCoffee<dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin, Format f> : Instruction {
  field bits<32> Inst;

  let Namespace = "Coffee";

  let Size = 4;

  bits<6> Opcode = 0;

  let Inst{31-26} = Opcode;

  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;
  let Itinerary  = itin;
}


class FFI<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern>:
          InstCoffee<outs, ins, asmstr, pattern, NoItinerary, FrmFI>
{
}


class FR<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmR>
{
// to be defined
}

class FI<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
// to be defined
}




class CoffeePseudo<dag outs, dag ins, string asmstr, list<dag> pattern>:
      InstCoffee<outs, ins, asmstr, pattern, IIPseudo, Pseudo> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}

// defintion of instruction format in coffee

class BranchBase<bits<6> op, dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin>:
  InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<3>  creg;
  bits<22>  imm;

  let Opcode = op;

  let Inst{24-22} = creg;
  let Inst{21-0} = imm;

}


class FJ<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmJ>
{
  bits<25> addr;

  let Opcode = op;

  let Inst{24-0} = addr;
}


// cex is set to 1
//class InstCoffee_1<bits<6> opcode, bits<3> creg, bits<3> cond, dag outs, dag ins, string asmstr, list<dag> pattern = []>
//        : InstCoffee<opcode, outs, ins, asmstr, pattern> {

//  let Inst{25} = 1;
//  let Inst{24-22} = creg;
//  let Inst{21-19} = cond;
//}

// cex is set to 0
class InstCoffee_0<bits<6> opcode, dag outs, dag ins, string asmstr, list<dag> pattern = []>
        : InstCoffee<outs, ins, asmstr, pattern, IIAlu, FrmI> {
  let Opcode = opcode;
  let Inst{25} = 0;
}

