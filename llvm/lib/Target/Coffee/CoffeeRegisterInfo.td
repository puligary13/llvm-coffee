//===-- CoffeeRegisterInfo.td - The Coffee Register File -----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

class CoffeeReg<string n> : Register<n> {
  let Namespace = "Coffee";
}

// GPR - One of the 32-bit general-purpose registers
class GPR<bits<5> num, string n> : CoffeeReg<n> {
  field bits<5> Num = num;
}

// CR - One of the 8 3-bit condition registers
class CR<bits<3> num, string n> : CoffeeReg<n> {
  field bits<3> Num = num;
}


// General-purpose registers
def T0  : GPR<0,  "t0">, DwarfRegNum<[0]>;
def T1  : GPR<1,  "t1">, DwarfRegNum<[1]>;
def T2  : GPR<2,  "t2">, DwarfRegNum<[2]>;
def T3  : GPR<3,  "t3">, DwarfRegNum<[3]>;
def T4 : GPR<4, "t4">, DwarfRegNum<[4]>;
def T5 : GPR<5, "t5">, DwarfRegNum<[5]>;
def T6 : GPR<6, "t6">, DwarfRegNum<[6]>;
def T7 : GPR<7, "t7">, DwarfRegNum<[7]>;
def T8 : GPR<8, "t8">, DwarfRegNum<[8]>;
def T9 : GPR<9, "t9">, DwarfRegNum<[9]>;
def S0 : GPR<10, "s0">, DwarfRegNum<[10]>;
def S1 : GPR<11, "s1">, DwarfRegNum<[11]>;
def S2 : GPR<12, "s2">, DwarfRegNum<[12]>;
def S3 : GPR<13, "s3">, DwarfRegNum<[13]>;
def S4 : GPR<14, "s4">, DwarfRegNum<[14]>;
def S5 : GPR<15, "s5">, DwarfRegNum<[15]>;
def S6 : GPR<16, "s6">, DwarfRegNum<[16]>;
def S7 : GPR<17, "s7">, DwarfRegNum<[17]>;
def S8 : GPR<18, "s8">, DwarfRegNum<[18]>;
def S9 : GPR<19, "s9">, DwarfRegNum<[19]>;
def S10 : GPR<20, "s10">, DwarfRegNum<[20]>;
def A0 : GPR<21, "a0">, DwarfRegNum<[21]>;
def A1  : GPR< 22,  "a1">, DwarfRegNum<[22]>;
def A2  : GPR< 23,  "a2">, DwarfRegNum<[23]>;
def A3  : GPR< 24,  "a3">, DwarfRegNum<[24]>;
def V0  : GPR< 25,  "v0">, DwarfRegNum<[25]>;
def V1  : GPR< 26,  "v1">, DwarfRegNum<[26]>;
def SP  : GPR< 27,  "sp">, DwarfRegNum<[27]>;
def FP : GPR<28, "fp">, DwarfRegNum<[28]>;
def PSR : GPR<29, "psr">, DwarfRegNum<[29]>;
def SPSR : GPR<30, "SPSR">, DwarfRegNum<[30]>;
def LR : GPR<31, "lr">, DwarfRegNum<[31]>;

// Condition registers
def CR0 : CR<0, "c0">, DwarfRegNum<[32]>;
def CR1 : CR<1, "c1">, DwarfRegNum<[33]>;
def CR2 : CR<2, "c2">, DwarfRegNum<[34]>;
def CR3 : CR<3, "c3">, DwarfRegNum<[35]>;
def CR4 : CR<4, "c4">, DwarfRegNum<[36]>;
def CR5 : CR<5, "c5">, DwarfRegNum<[37]>;
def CR6 : CR<6, "c6">, DwarfRegNum<[38]>;
def CR7 : CR<7, "c7">, DwarfRegNum<[39]>;


// general purpose register class
def GPRC : RegisterClass<"Coffee", [i32], 32, (add
//Not preserved
T0, T1, T2, T3, T4, T5, T6, T7, T8, T9,
//Callee save
S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10,
//Arguments and Return
A0, A1, A2, A3, V0, V1,
//Reserved
SP, FP, PSR, SPSR, LR
)>;

// FP register class
def FPRC : RegisterClass<"Coffee", [f32], 32, (add
//Not preserved
T0, T1, T2, T3, T4, T5, T6, T7, T8, T9,
//Callee save
S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10,
//Arguments and Return
A0, A1, A2, A3, V0, V1,
//Reserved
SP, FP, PSR, SPSR, LR
)>;


def CRRC : RegisterClass<"Coffee", [i32], 32, (add CR0, CR1, CR2, CR3,
                                                CR4, CR5, CR6, CR7)>;
