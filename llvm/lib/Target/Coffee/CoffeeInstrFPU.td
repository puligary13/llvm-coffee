//===-- CoffeeInstrFPU.td - Coffee FPU Instruction Information -*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the Coffee FPU instruction set.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Floating Point Instructions
// ------------------------
// * 64bit fp:
//    - 32 64-bit registers (default mode)
//    - 16 even 32-bit registers (32-bit compatible mode) for
//      single and double access.
// * 32bit fp:
//    - 16 even 32-bit registers - single and double (aliased)
//    - 32 32-bit registers (within single-only mode)
//===----------------------------------------------------------------------===//

// Floating Point Compare and Branch


//===----------------------------------------------------------------------===//
// Feature predicates.
//===----------------------------------------------------------------------===//

def IsCOP0        : Predicate<"Subtarget.isCOP0()">;
def IsCOP1        : Predicate<"Subtarget.isCOP1()">;
def IsCOP2        : Predicate<"Subtarget.isCOP2()">;
def IsCOP3        : Predicate<"Subtarget.isCOP3()">;

//def SDT_COFFEEfpCmp     : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>]>;
def SDT_COFFEEfpCmp     : SDTypeProfile<1, 2, []>;

def COFFEEfpcmp           : SDNode<"COFFEEISD::FPCMP", SDT_COFFEEfpCmp,
                              [SDNPOutGlue]>;

//===----------------------------------------------------------------------===//
// Instruction Class Templates
//
// A set of multiclasses is used to address the register usage.
//
// S32 - single precision in 16 32bit even fp registers
//       single precision in 32 32bit fp registers in SingleOnly mode
// S64 - single precision in 32 64bit fp registers (In64BitMode)
// D32 - double precision in 16 32bit even fp registers
// D64 - double precision in 32 64bit fp registers (In64BitMode)
//
// Only S32 and D32 are supported right now.
//===----------------------------------------------------------------------===//

multiclass FFR1_CW_M<bits<6> op, string opstr> {
  def _0 : FFR1_C<op, opstr, 0b00, ".w0", GPRC, FPRC>,
           Requires<[IsCOP0]>;
  def _1 : FFR1_C<op, opstr, 0b01, ".w1", GPRC, FPRC>,
           Requires<[IsCOP1]>;
  def _2 : FFR1_C<op, opstr, 0b10, ".w2", GPRC, FPRC>,
           Requires<[IsCOP2]>;
  def _3 : FFR1_C<op, opstr, 0b11, ".w3", GPRC, FPRC>,
           Requires<[IsCOP3]>;
}

multiclass FFR1_CS_M<bits<6> op, string opstr> {
  def _0 : FFR1_C<op, opstr, 0b00, ".s0", FPRC, GPRC>,
           Requires<[IsCOP0]>;
  def _1 : FFR1_C<op, opstr, 0b01, ".s1", FPRC, GPRC>,
           Requires<[IsCOP1]>;
  def _2 : FFR1_C<op, opstr, 0b10, ".s2", FPRC, GPRC>,
           Requires<[IsCOP2]>;
  def _3 : FFR1_C<op, opstr, 0b11, ".s3", FPRC, GPRC>,
           Requires<[IsCOP3]>;
}

multiclass FFR1_CM<bits<6> op, string opstr> {
  def _0 : FFR1_C<op, opstr, 0b00, "0", FPRC, FPRC>,
           Requires<[IsCOP0]>;
  def _1 : FFR1_C<op, opstr, 0b01, "1", FPRC, FPRC>,
           Requires<[IsCOP1]>;
  def _2 : FFR1_C<op, opstr, 0b10, "2", FPRC, FPRC>,
           Requires<[IsCOP2]>;
  def _3 : FFR1_C<op, opstr, 0b11, "3", FPRC, FPRC>,
           Requires<[IsCOP3]>;
}


multiclass FFR1P_CM<bits<6> op, string opstr, SDNode OpNode> {
  def _0 : FFR1P_C<op, opstr, 0b00, "0", FPRC, FPRC, OpNode>,
           Requires<[IsCOP0]>;
  def _1 : FFR1P_C<op, opstr, 0b01, "1", FPRC, FPRC, OpNode>,
           Requires<[IsCOP1]>;
  def _2 : FFR1P_C<op, opstr, 0b10, "2", FPRC, FPRC, OpNode>,
           Requires<[IsCOP2]>;
  def _3 : FFR1P_C<op, opstr, 0b11, "3", FPRC, FPRC, OpNode>,
           Requires<[IsCOP3]>;
}

multiclass FFR2P_CM<bits<6> op, string opstr, SDNode OpNode, bit isComm = 0> {
  let isCommutable = isComm in {
  def _0 : FFR2P_C<op, opstr, 0b00, "0", FPRC, OpNode>,
           Requires<[IsCOP0]>;
  def _1 : FFR2P_C<op, opstr, 0b01, "1", FPRC, OpNode>,
           Requires<[IsCOP1]>;
  def _2 : FFR2P_C<op, opstr, 0b10, "2", FPRC, OpNode>,
           Requires<[IsCOP2]>;
  def _3 : FFR2P_C<op, opstr, 0b11, "3", FPRC, OpNode>,
           Requires<[IsCOP3]>;
}
}

multiclass FP_CMPM<bits<6> op, string opstr, SDNode OpNode> {
  def _0 : FFR2P_CMP_C<op, opstr, 0b00, "0", FPRC, OpNode>,
         Requires<[IsCOP0]>;
  def _1 : FFR2P_CMP_C<op, opstr, 0b01, "1", FPRC, OpNode>,
         Requires<[IsCOP1]>;
  def _2 : FFR2P_CMP_C<op, opstr, 0b10, "2", FPRC, OpNode>,
         Requires<[IsCOP2]>;
  def _3 : FFR2P_CMP_C<op, opstr, 0b11, "3", FPRC, OpNode>,
         Requires<[IsCOP3]>;
}

class MTCF<bits<6> op>:
    F2R<op, (outs FPRC:$rt), (ins GPRC:$rs), "mov\t$rt,\t$rs",
    [(set FPRC:$rt, (bitconvert GPRC:$rs))], IIAlu> {
let Constraints = "$rt = $rs";
}

class MFCF<bits<6> op>:
    F2R<op, (outs GPRC:$rt), (ins FPRC:$rs), "mov\t$rt,\t$rs",
    [(set GPRC:$rt, (bitconvert FPRC:$rs))], IIAlu> {
let Constraints = "$rt = $rs";
}


//===----------------------------------------------------------------------===//
// Floating Point Instructions
//===----------------------------------------------------------------------===//

//load & store

def FLOAD      : LoadM<0b110010, "ldw",  load,  FPRC, mem_load, 0>;
def FSTORE      : StoreM<0b110100, "stw", store,  FPRC, mem_store, 0>;

// convert 32-bit integer to single-floating point
defm CVT_W_S : FFR1_CW_M<0b100100, "fcvt">, NeverHasSideEffects;
defm CVT_S_W : FFR1_CS_M<0b100000, "fcvt">, NeverHasSideEffects;

defm FABS : FFR1P_CM<0b000101, "fabs",  fabs>;
defm FNEG : FFR1P_CM<0b000111, "fneg",  fneg>;
defm FSQRT : FFR1P_CM<0b000100, "fsqrt", fsqrt>;

defm FMOV : FFR1_CM<0b000110, "fmov">;

/// Floating-point Aritmetic
defm FADD : FFR2P_CM<0b000000, "fadd", fadd, 1>;
defm FDIV : FFR2P_CM<0b000011, "fdiv", fdiv>;
defm FMUL : FFR2P_CM<0b000010, "fmul", fmul, 1>;
defm FSUB : FFR2P_CM<0b000001, "fsub", fsub>;
// FP Compare
defm FP_CMP :  FP_CMPM<0b110110, "fc.ole", COFFEEfpcmp>;

// bitcast

def MTC : MTCF<0b010011>;
def MFC : MFCF<0b010011>;


//-----------------
// Pattern
//-----------------

// FP immediate patterns.
def fpimm0 : PatLeaf<(fpimm), [{
  return N->isExactlyValue(+0.0);
}]>;

def fpimm0neg : PatLeaf<(fpimm), [{
  return N->isExactlyValue(-0.0);
}]>;

let Predicates = [IsCOP0] in {
def : Pat<(f32 (sint_to_fp GPRC:$src)), (CVT_S_W_0 GPRC:$src)>;
def : Pat<(i32 (fp_to_sint FPRC:$src)), (CVT_W_S_0 FPRC:$src)>;
//def : Pat<(f32 (uint_to_fp GPRC:$src)), (CVT_S_W_0 GPRC:$src)>;
//def : Pat<(i32 (fp_to_uint FPRC:$src)), (CVT_W_S_0 FPRC:$src)>;
}

let Predicates = [IsCOP1] in {
def : Pat<(f32 (sint_to_fp GPRC:$src)), (CVT_S_W_1 GPRC:$src)>;
def : Pat<(i32 (fp_to_sint FPRC:$src)), (CVT_W_S_1 FPRC:$src)>;
//def : Pat<(f32 (uint_to_fp GPRC:$src)), (CVT_S_W_1 GPRC:$src)>;
//def : Pat<(i32 (fp_to_uint FPRC:$src)), (CVT_W_S_1 FPRC:$src)>;
}

let Predicates = [IsCOP2] in {
def : Pat<(f32 (sint_to_fp GPRC:$src)), (CVT_S_W_2 GPRC:$src)>;
def : Pat<(i32 (fp_to_sint FPRC:$src)), (CVT_W_S_2 FPRC:$src)>;
//def : Pat<(f32 (uint_to_fp GPRC:$src)), (CVT_S_W_2 GPRC:$src)>;
//def : Pat<(i32 (fp_to_uint FPRC:$src)), (CVT_W_S_2 FPRC:$src)>;
}

let Predicates = [IsCOP3] in {
def : Pat<(f32 (sint_to_fp GPRC:$src)), (CVT_S_W_3 GPRC:$src)>;
def : Pat<(i32 (fp_to_sint FPRC:$src)), (CVT_W_S_3 FPRC:$src)>;
//def : Pat<(f32 (uint_to_fp GPRC:$src)), (CVT_S_W_3 GPRC:$src)>;
//def : Pat<(i32 (fp_to_uint FPRC:$src)), (CVT_W_S_3 FPRC:$src)>;
}

//branch pattern
//greater or equal
def : Pat<(brcond (i32 (setge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setuge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setuge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setuge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setuge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setoge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setoge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setoge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setoge FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

//less or equal
def : Pat<(brcond (i32 (setle FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setle FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setle FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setle FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setule FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setule FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setule FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setule FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setole FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setole FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setole FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setole FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BELT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

//equal
def : Pat<(brcond (i32 (seteq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (seteq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (seteq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (seteq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setueq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setueq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setueq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setueq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setoeq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setoeq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setoeq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setoeq FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BEQ (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

//greater
def : Pat<(brcond (i32 (setgt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setgt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setgt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setgt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setugt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setugt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setugt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setugt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setogt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setogt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setogt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setogt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BGT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

//less
def : Pat<(brcond (i32 (setlt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setlt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setlt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setlt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setult FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setult FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setult FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setult FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setolt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setolt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setolt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setolt FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BLT (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

//not equal
def : Pat<(brcond (i32 (setne FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setne FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setne FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setne FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

def : Pat<(brcond (i32 (setune FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setune FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setune FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setune FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;


def : Pat<(brcond (i32 (setone FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_0 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP0]>;
def : Pat<(brcond (i32 (setone FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_1 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP1]>;
def : Pat<(brcond (i32 (setone FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_2 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP2]>;
def : Pat<(brcond (i32 (setone FPRC:$lhs, FPRC:$rhs)), bb:$dst),
          (BNE (FP_CMP_3 FPRC:$lhs, FPRC:$rhs), bb:$dst)>,
          Requires<[IsCOP3]>;

// setcc pattern
//greater or equal
def : Pat<(setge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setuge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setuge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setuge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setuge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setoge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setoge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setoge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setoge FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

//less or equal
def : Pat<(setle FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setle FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setle FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setle FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setule FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setule FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setule FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setule FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setole FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setole FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setole FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setole FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

//equal
def : Pat<(seteq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(seteq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(seteq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(seteq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setueq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setueq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setueq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setueq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setoeq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setoeq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setoeq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setoeq FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

//greater
def : Pat<(setgt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setgt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setgt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setgt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setugt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setugt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setugt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setugt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setogt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setogt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setogt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setogt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

//less
def : Pat<(setlt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setlt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setlt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setlt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setult FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setult FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setult FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setult FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setolt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setolt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setolt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setolt FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

//not equal
def : Pat<(setne FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setne FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setne FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setne FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;

def : Pat<(setune FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setune FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setune FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setune FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;


def : Pat<(setone FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_0 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP0]>;
def : Pat<(setone FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_1 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP1]>;
def : Pat<(setone FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_2 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP2]>;
def : Pat<(setone FPRC:$lhs, FPRC:$rhs),
          (FP_CMP_3 FPRC:$lhs, FPRC:$rhs)>,
          Requires<[IsCOP3]>;
