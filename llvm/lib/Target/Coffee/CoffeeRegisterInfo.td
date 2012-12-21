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

let Namespace = "Coffee" in {
def sub_zero : SubRegIndex;
def sub_negative : SubRegIndex;
def sub_carry : SubRegIndex;
}


class CoffeeReg<string n> : Register<n> {
  let Namespace = "Coffee";
}

// We identify all our registers with a 5-bit ID, for consistency's sake.

// GPR - One of the 32-bit general-purpose registers
class GPR<bits<5> num, string n> : CoffeeReg<n> {
  field bits<5> Num = num;
}

// PGPR - One of the 32-bit priority general-purpose registers
class PGPR<bits<5> num, string n> : CoffeeReg<n> {
  field bits<5> Num = num;
}

// PSR - the 8-bit special-purpose register
//class PSR<string n> : CoffeeReg<n> {
//}

// SPSR - the 32-bit special-purpose register
//class SPSR<string n> : CoffeeReg<n> {
//}

// CR - One of the 8 3-bit condition registers
class CR<bits<3> num, string n, list<Register> subregs> : CoffeeReg<n> {
  field bits<3> Num = num;
  let SubRegs = subregs;
}

// CRBIT - One of the 1-bit condition register fields
class CRBIT<bits<5> num, string n> : CoffeeReg<n> {
  field bits<5> Num = num;
}

class Rf<bits<3> num, string n> : CoffeeReg<n> {
   field bits<3> Num = num;
}

// General-purpose registers
/*def T0  : GPR<0,  "t0">, DwarfRegNum<[0]>;
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
def S11 : GPR<21, "s11">, DwarfRegNum<[21]>;
def A0  : GPR< 22,  "a0">, DwarfRegNum<[22]>;
def A1  : GPR< 23,  "a1">, DwarfRegNum<[23]>;
def A2  : GPR< 24,  "a2">, DwarfRegNum<[24]>;
def A3  : GPR< 25,  "a3">, DwarfRegNum<[25]>;
def V0  : GPR< 26,  "v0">, DwarfRegNum<[26]>;
def V1  : GPR< 27,  "v1">, DwarfRegNum<[27]>;
def GP : GPR<28, "gp">, DwarfRegNum<[28]>;
def SP : GPR<29, "sp">, DwarfRegNum<[29]>;
def FP : GPR<30, "fp">, DwarfRegNum<[30]>;
def LR : GPR<31, "lr">, DwarfRegNum<[31]>; *///R31

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

// priority General-purpose registers
def PR0  : PGPR< 0,  "pr0">, DwarfRegNum<[32]>;
def PR1  : PGPR< 1,  "pr1">, DwarfRegNum<[33]>;
def PR2  : PGPR< 2,  "pr2">, DwarfRegNum<[34]>;
def PR3  : PGPR< 3,  "pr3">, DwarfRegNum<[35]>;
def PR4  : PGPR< 4,  "pr4">, DwarfRegNum<[36]>;
def PR5  : PGPR< 5,  "pr5">, DwarfRegNum<[37]>;
def PR6  : PGPR< 6,  "pr6">, DwarfRegNum<[38]>;
def PR7  : PGPR< 7,  "pr7">, DwarfRegNum<[39]>;
def PR8  : PGPR< 8,  "pr8">, DwarfRegNum<[40]>;
def PR9  : PGPR< 9,  "pr9">, DwarfRegNum<[41]>;
def PR10 : PGPR<10, "pr10">, DwarfRegNum<[42]>;
def PR11 : PGPR<11, "pr11">, DwarfRegNum<[43]>;
def PR12 : PGPR<12, "pr12">, DwarfRegNum<[44]>;
def PR13 : PGPR<13, "pr13">, DwarfRegNum<[45]>;
def PR14 : PGPR<14, "pr14">, DwarfRegNum<[46]>;
def PR15 : PGPR<15, "pr15">, DwarfRegNum<[47]>;
def PR16 : PGPR<16, "pr16">, DwarfRegNum<[48]>;
def PR17 : PGPR<17, "pr17">, DwarfRegNum<[49]>;
def PR18 : PGPR<18, "pr18">, DwarfRegNum<[50]>;
def PR19 : PGPR<19, "pr19">, DwarfRegNum<[51]>;
def PR20 : PGPR<20, "pr20">, DwarfRegNum<[52]>;
def PR21 : PGPR<21, "pr21">, DwarfRegNum<[53]>;
def PR22 : PGPR<22, "pr22">, DwarfRegNum<[54]>;
def PR23 : PGPR<23, "pr23">, DwarfRegNum<[55]>;
def PR24 : PGPR<24, "pr24">, DwarfRegNum<[56]>;
def PR25 : PGPR<25, "pr25">, DwarfRegNum<[57]>;
def PR26 : PGPR<26, "pr26">, DwarfRegNum<[58]>;
def PR27 : PGPR<27, "pr27">, DwarfRegNum<[59]>;
def PR28 : PGPR<28, "pr28">, DwarfRegNum<[60]>;
//def PSR : PGPR<29, "psr">, DwarfRegNum<[61]>; //pr29
//def SPSR : PGPR<30, "spsr">, DwarfRegNum<[62]>; //pr30
def PLR : PGPR<31, "plr">, DwarfRegNum<[63]>; //pr31

// Condition register bits

def CR0ZE : CRBIT< 0, "0">;
def CR0NE : CRBIT< 1, "1">;
def CR0CA : CRBIT< 2, "2">;

def CR1ZE : CRBIT< 3, "3">;
def CR1NE : CRBIT< 4, "4">;
def CR1CA : CRBIT< 5, "5">;

def CR2ZE : CRBIT< 6, "6">;
def CR2NE : CRBIT< 7, "7">;
def CR2CA : CRBIT< 8, "8">;

def CR3ZE : CRBIT< 9, "9">;
def CR3NE : CRBIT< 10, "10">;
def CR3CA : CRBIT< 11, "11">;

def CR4ZE : CRBIT< 12, "12">;
def CR4NE : CRBIT< 13, "13">;
def CR4CA : CRBIT< 14, "14">;

def CR5ZE : CRBIT< 15, "15">;
def CR5NE : CRBIT< 16, "16">;
def CR5CA : CRBIT< 17, "17">;

def CR6ZE : CRBIT< 18, "18">;
def CR6NE : CRBIT< 19, "19">;
def CR6CA : CRBIT< 20, "20">;

def CR7ZE : CRBIT< 21, "21">;
def CR7NE : CRBIT< 22, "22">;
def CR7CA : CRBIT< 23, "23">;


// Condition registers
let SubRegIndices = [sub_zero, sub_negative, sub_carry] in {
def CR0 : CR<0, "cr0", [CR0ZE, CR0NE, CR0CA]>, DwarfRegNum<[64]>;
def CR1 : CR<1, "cr1", [CR1ZE, CR1NE, CR1CA]>, DwarfRegNum<[65]>;
def CR2 : CR<2, "cr2", [CR2ZE, CR2NE, CR2CA]>, DwarfRegNum<[66]>;
def CR3 : CR<3, "cr3", [CR3ZE, CR3NE, CR3CA]>, DwarfRegNum<[67]>;
def CR4 : CR<4, "cr4", [CR4ZE, CR4NE, CR4CA]>, DwarfRegNum<[68]>;
def CR5 : CR<5, "cr5", [CR5ZE, CR5NE, CR5CA]>, DwarfRegNum<[69]>;
def CR6 : CR<6, "cr6", [CR6ZE, CR6NE, CR6CA]>, DwarfRegNum<[70]>;
def CR7 : CR<7, "cr7", [CR7ZE, CR7NE, CR7CA]>, DwarfRegNum<[71]>;
}


//def S0  : Rf< 0, "s0">;
//def S1  : Rf< 1, "s1">;
//def S2  : Rf< 2, "s2">;
//def S3  : Rf< 3, "s3">;
//def S4  : Rf< 4, "s4">;
//def S5  : Rf< 5, "s5">;
//def S6  : Rf< 6, "s6">;
//def S7  : Rf< 7, "s7">;

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



def CRBITRC : RegisterClass<"Coffee", [i32], 32,
  (add CR0ZE, CR0NE, CR0CA,
       CR1ZE, CR1NE, CR1CA,
       CR2ZE, CR2NE, CR2CA,
       CR3ZE, CR3NE, CR3CA,
       CR4ZE, CR4NE, CR4CA,
       CR5ZE, CR5NE, CR5CA,
       CR6ZE, CR6NE, CR6CA,
       CR7ZE, CR7NE, CR7CA)>
{
  let CopyCost = -1;
}

def CRRC : RegisterClass<"Coffee", [i32], 32, (add CR0, CR1, CR2, CR3,
                                                CR4, CR5, CR6, CR7)> {
}


// priority general purpose register class
def PGPRC : RegisterClass<"Coffee", [i32], 32, (add (sequence "PR%u", 0, 28), PSR, SPSR, PLR)>;

// float point register
def FPRC : RegisterClass <"Coffee", [f32], 32, (add (sequence "S%u", 0, 7))>;
