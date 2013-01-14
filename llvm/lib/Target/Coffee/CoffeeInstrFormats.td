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

  Format Form = f;

  let Namespace = "Coffee";

  let Size = 4;

  bits<6> Opcode = 0;

  let Inst{31-26} = Opcode;

  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;
  let Itinerary  = itin;

  //
  // Attributes specific to Mips instructions...
  //
  bits<4> FormBits = Form.Value;

  // TSFlags layout should be kept in sync with MipsInstrInfo.h.
  let TSFlags{3-0}   = FormBits;
}

// Instruction without operands
class F<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmOther>
{
  let Opcode = op;
  let Inst{25} = 0b0; // disable condition
}


// register instruction format with 2 operands
class F2R<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rt;
  bits<5> rs;

  let Opcode = op;

  let Inst{25} = 0b0; // disable condition
  let Inst{9-5} = rs;
  let Inst{4-0}  = rt;
}

// register instruction format with 3 operands
class F3R<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rd;
  bits<5> rt;
  bits<5> rs;

  let Opcode = op;

  let Inst{25} = 0b0; // disable condition
  let Inst{4-0}  = rd;
  let Inst{9-5} = rt;
  let Inst{14-10} = rs;
}


class FI<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<15>  imm15;
  bits<5> rs;
  bits<5> rt;

  let Opcode = op;

  let Inst{25} = 0b0; // disable condition
  let Inst{24-10} = imm15;
  let Inst{9-5} = rs;
  let Inst{4-0}  = rt;
}

class CMPI<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<3> rd;
  bits<5> rt;
  bits<17> imm17;
  let Opcode = op;
  let Inst{24-22} = rd;
  let Inst{9-5} = rt;
  let Inst{21-10} = imm17{11-0};
  let Inst{4-0} = imm17{16-12};
}

class CMPR<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<3> rd;
  bits<5> rt;
  bits<5> rs;
  let Opcode = op;
  let Inst{24-22} = rd;
  let Inst{9-5} = rt;
  let Inst{14-10} = rs;
}

class BranchBase<bits<6> op, dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin>:
  InstCoffee<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<22>  imm22;
  bits<3> creg;

  let Opcode = op;

  let Inst{25} = 0b1;
  let Inst{24-22} = creg;
  let Inst{21-0} = imm22;

}


class FJ<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: InstCoffee<outs, ins, asmstr, pattern, itin, FrmJ>
{
  bits<25> addr;

  let Opcode = op;

  let Inst{25} = 0;
  let Inst{24-0} = addr;
}

class CoffeePseudo<dag outs, dag ins, string asmstr, list<dag> pattern>:
      InstCoffee<outs, ins, asmstr, pattern, IIPseudo, Pseudo> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}

class CoffeeAsmPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>:
      InstCoffee<outs, ins, asmstr, pattern, IIPseudo, Pseudo> {
  let isPseudo = 1;
}

//////////////////////////////
// floating point format    //
//////////////////////////////

class FFI<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern> :
  InstCoffee<outs, ins, asmstr, pattern, NoItinerary, FrmFI>
{
}


// floating point instruction with register
class FFR_C<bits<6> op, dag outs, dag ins, string asmstr, list<dag> pattern> :
  InstCoffee<outs, ins, asmstr, pattern, NoItinerary, FrmFR>
{
  // opcode for coffee cop instruction
  let Opcode = 0b111100;
  let Inst{21} = 0b0; //single precision only
  let Inst{5-0}   = op; //opcode for milk coprocessor
}

// FP unary instruction without patterns
class FFR1_C<bits<6> op, string opstr, bits<2> co, string copro,
           RegisterClass DstRC, RegisterClass SrcRC> :
  FFR_C<op, (outs DstRC:$fd), (ins SrcRC:$fs),
      !strconcat(opstr, copro, "\t$fd,\t$fs"), []> {
      bits<5> fd;
      bits<5> fs;

      let Inst{25-24} = co;
      let Inst{15-11} = fs;
      let Inst{10-6} = fd;
}

// FP unary instruction with patterns
class FFR1P_C<bits<6> op, string opstr, bits<2> co,string copro,
            RegisterClass DstRC, RegisterClass SrcRC, SDNode OpNode> :
  FFR_C<op, (outs DstRC:$fd), (ins SrcRC:$fs),
      !strconcat(opstr, copro, "\t$fd,\t$fs"),
      [(set DstRC:$fd, (OpNode SrcRC:$fs))]> {
      bits<5> fd;
      bits<5> fs;

      let Inst{25-24} = co;
      let Inst{15-11} = fs;
      let Inst{10-6} = fd;
}

class FFR2P_C<bits<6> op, string opstr, bits<2> co,
            string copro, RegisterClass RC, SDNode OpNode> :
  FFR_C<op, (outs RC:$fd), (ins RC:$fs, RC:$ft),
      !strconcat(opstr, copro, "\t$fd,\t$fs,\t$ft"),
      [(set RC:$fd, (OpNode RC:$fs, RC:$ft))]> {
      bits<5> fd;
      bits<5> ft;
      bits<5> fs;

      let Inst{25-24} = co;
      let Inst{20-16} = ft;
      let Inst{15-11} = fs;
      let Inst{10-6} = fd;
}

class FFR2P_CMP_C<bits<6> op, string opstr, bits<2> co,
            string copro, RegisterClass RC, SDNode OpNode> :
  FFR_C<op, (outs CRRC:$fd), (ins RC:$fs, RC:$ft),
      !strconcat(opstr, copro, "\t$fd,\t$fs,\t$ft"),
      [(set CRRC:$fd, (OpNode RC:$fs, RC:$ft))]> {

      bits<5> fd;
      bits<5> ft;
      bits<5> fs;

      let Inst{25-24} = co;
      let Inst{20-16} = ft;
      let Inst{15-11} = fs;
      let Inst{10-6} = fd;

      let hasDelaySlot = 1;
}


////////////////////////////////////////////////////////////////////////////
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

