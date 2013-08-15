//===-- CoffeeCondMov.td - Describe Mips Conditional Moves --*- tablegen -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is the Conditional Moves implementation.
//
//===----------------------------------------------------------------------===//

class CondF2R<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rt;
  bits<5> rs;
  bits<3> creg;
  bits<3> cond;

  let Opcode = op;

  let Inst{25} = 0b1; // enable condition
  let Inst{24-22} = creg;
  let Inst{21-19} = cond;
  let Inst{9-5} = rs;
  let Inst{4-0}  = rt;
}

//---------------------------------------------------------------------------------//


class CondF3R<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      InstCoffee<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rd;
  bits<5> rt;
  bits<5> rs;
  bits<3> creg;
  bits<3> cond;

  let Opcode = op;

  let Inst{25} = 0b1; // enable condition
  let Inst{24-22} = creg;
  let Inst{21-19} = cond;
  let Inst{14-10} = rs;
  let Inst{9-5} = rt;
  let Inst{4-0}  = rd;
}

class ArithmeticCondF3R<bits<6> op, string cond, string operation, bits<3>condbit>:
    CondF3R<op, (outs GPRC:$rd), (ins GPRC:$rt, GPRC:$rs, CRRC:$creg), !strconcat(operation,".",cond,"\t$creg,\t$rt,\t$rs"), [], IIAlu> {
    let cond = condbit;
    let Constraints = "$rs = $rd";
}


class MovCondF2R<string cond, bits<3>condbit>:
    CondF2R<0b010011, (outs GPRC:$rt), (ins GPRC:$rs, GPRC:$F, CRRC:$creg), !strconcat("mov.",cond,"\t$creg,\t$rt,\t$rs"), [], IIAlu> {
    let cond = condbit;
    let Constraints = "$F = $rt";
}

class FPMovCondF2R<string cond, bits<3>condbit>:
    CondF2R<0b010011, (outs FPRC:$rt), (ins FPRC:$rs, FPRC:$F, CRRC:$creg), !strconcat("mov.",cond,"\t$creg,\t$rt,\t$rs"), [], IIAlu> {
    let cond = condbit;
    let Constraints = "$F = $rt";
}


def CMov_EGT : MovCondF2R<"egt", 0b001>;
def CMov_ELT : MovCondF2R<"elt", 0b010>;
def CMov_EQ  : MovCondF2R<"eq", 0b011>;
def CMov_GT  : MovCondF2R<"gt", 0b100>;
def CMov_LT  : MovCondF2R<"lt", 0b101>;
def CMov_NE  : MovCondF2R<"ne", 0b110>;


def FPCMov_EGT : FPMovCondF2R<"egt", 0b001>;
def FPCMov_ELT : FPMovCondF2R<"elt", 0b010>;
def FPCMov_EQ  : FPMovCondF2R<"eq", 0b011>;
def FPCMov_GT  : FPMovCondF2R<"gt", 0b100>;
def FPCMov_LT  : FPMovCondF2R<"lt", 0b101>;
def FPCMov_NE  : FPMovCondF2R<"ne", 0b110>;


def ConSub_EGT : ArithmeticCondF3R<0b010000, "egt", "sub", 0b001>;
def ConSub_ELT : ArithmeticCondF3R<0b010000, "elt", "sub", 0b010>;
def ConSub_EQ  : ArithmeticCondF3R<0b010000, "eq", "sub", 0b011>;
def ConSub_GT  : ArithmeticCondF3R<0b010000, "gt", "sub", 0b100>;
def ConSub_LT : ArithmeticCondF3R<0b010000, "lt", "sub", 0b101>;
def ConSub_NE : ArithmeticCondF3R<0b010000, "ne", "sub", 0b110>;

// integer select patterns

multiclass CMovIntPat<RegisterClass CR, Instruction EGT, Instruction ELT,
                      Instruction EQ, Instruction GT, Instruction LT,
                      Instruction NE> {
//greater or equal
def : Pat<(select (i32 (setge GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (EGT CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;

//less or equal
def : Pat<(select (i32 (setle GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (ELT CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;

//equal
def : Pat<(select (i32 (seteq GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (EQ CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;

//greater
def : Pat<(select (i32 (setgt GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (GT CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;

//less
def : Pat<(select (i32 (setlt GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (LT CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;

//not equal
def : Pat<(select (i32 (setne GPRC:$lhs, GPRC:$rhs)), CR:$T, CR:$F),
          (NE CR:$T, CR:$F, (CMPrr GPRC:$lhs, GPRC:$rhs))>;
}

defm : CMovIntPat<GPRC, CMov_EGT, CMov_ELT, CMov_EQ, CMov_GT, CMov_LT, CMov_NE>;
defm : CMovIntPat<FPRC, FPCMov_EGT, FPCMov_ELT, FPCMov_EQ, FPCMov_GT, FPCMov_LT, FPCMov_NE>;

//float point select patterns

multiclass CMovFPPat<RegisterClass CR, Instruction EGT, Instruction ELT,
                     Instruction EQ, Instruction GT, Instruction LT,
                     Instruction NE> {
//greater or equal
def : Pat<(select (i32 (setge FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EGT CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (setge FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EGT CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (setge FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EGT CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (setge FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EGT CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;

//less or equal
def : Pat<(select (i32 (setle FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (ELT CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (setle FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (ELT CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (setle FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (ELT CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (setle FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (ELT CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;

//equal
def : Pat<(select (i32 (seteq FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EQ CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (seteq FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EQ CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (seteq FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EQ CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (seteq FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (EQ CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;

//greater
def : Pat<(select (i32 (setgt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (GT CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (setgt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (GT CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (setgt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (GT CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (setgt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (GT CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;

//less
def : Pat<(select (i32 (setlt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (LT CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (setlt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (LT CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (setlt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (LT CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (setlt FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (LT CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;

//not equal
def : Pat<(select (i32 (setne FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (NE CR:$T, CR:$F, (FP_CMP_0 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP0]>;
def : Pat<(select (i32 (setne FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (NE CR:$T, CR:$F, (FP_CMP_1 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP1]>;
def : Pat<(select (i32 (setne FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (NE CR:$T, CR:$F, (FP_CMP_2 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP2]>;
def : Pat<(select (i32 (setne FPRC:$lhs, FPRC:$rhs)), CR:$T, CR:$F),
          (NE CR:$T, CR:$F, (FP_CMP_3 FPRC:$lhs, FPRC:$rhs))>,
          Requires<[IsCOP3]>;
}


defm : CMovFPPat<GPRC, CMov_EGT, CMov_ELT, CMov_EQ, CMov_GT, CMov_LT, CMov_NE>;
defm : CMovFPPat<FPRC, FPCMov_EGT, FPCMov_ELT, FPCMov_EQ, FPCMov_GT, FPCMov_LT, FPCMov_NE>;
