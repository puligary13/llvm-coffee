//===- IntrinsicsCoffee.td - Defines Coffee intrinsics ---*- tablegen -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file defines all of the Coffee-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Definitions for all Coffee intrinsics.
//

// Non-altivec intrinsics.
let TargetPrefix = "Coffee" in {  // All intrinsics start with "llvm.Coffee.".
  // dcba/dcbf/dcbi/dcbst/dcbt/dcbz/dcbzl(Coffee970) instructions.
  def int_Coffee_dcba  : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbf  : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbi  : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbst : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbt  : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbtst: Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbz  : Intrinsic<[], [llvm_ptr_ty], []>;
  def int_Coffee_dcbzl : Intrinsic<[], [llvm_ptr_ty], []>;

  // sync instruction
  def int_Coffee_sync : Intrinsic<[], [], []>;
}


let TargetPrefix = "Coffee" in {  // All Coffee intrinsics start with "llvm.Coffee.".
  /// Coffee_Vec_Intrinsic - Base class for all altivec intrinsics.
  class Coffee_Vec_Intrinsic<string GCCIntSuffix, list<LLVMType> ret_types,
                              list<LLVMType> param_types,
                              list<IntrinsicProperty> properties>
    : GCCBuiltin<!strconcat("__builtin_altivec_", GCCIntSuffix)>,
      Intrinsic<ret_types, param_types, properties>;
}

//===----------------------------------------------------------------------===//
// Coffee Altivec Intrinsic Class Definitions.
//

/// Coffee_Vec_FF_Intrinsic - A Coffee intrinsic that takes one v4f32
/// vector and returns one.  These intrinsics have no side effects.
class Coffee_Vec_FF_Intrinsic<string GCCIntSuffix>
  : Coffee_Vec_Intrinsic<GCCIntSuffix,
                          [llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;

/// Coffee_Vec_FFF_Intrinsic - A Coffee intrinsic that takes two v4f32
/// vectors and returns one.  These intrinsics have no side effects.
class Coffee_Vec_FFF_Intrinsic<string GCCIntSuffix>
  : Coffee_Vec_Intrinsic<GCCIntSuffix,
                          [llvm_v4f32_ty], [llvm_v4f32_ty, llvm_v4f32_ty],
                          [IntrNoMem]>;

/// Coffee_Vec_BBB_Intrinsic - A Coffee intrinsic that takes two v16f8
/// vectors and returns one.  These intrinsics have no side effects.
class Coffee_Vec_BBB_Intrinsic<string GCCIntSuffix>
  : Coffee_Vec_Intrinsic<GCCIntSuffix,
                          [llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                          [IntrNoMem]>;

/// Coffee_Vec_HHH_Intrinsic - A Coffee intrinsic that takes two v8i16
/// vectors and returns one.  These intrinsics have no side effects.
class Coffee_Vec_HHH_Intrinsic<string GCCIntSuffix>
  : Coffee_Vec_Intrinsic<GCCIntSuffix,
                          [llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                          [IntrNoMem]>;

/// Coffee_Vec_WWW_Intrinsic - A Coffee intrinsic that takes two v4i32
/// vectors and returns one.  These intrinsics have no side effects.
class Coffee_Vec_WWW_Intrinsic<string GCCIntSuffix>
  : Coffee_Vec_Intrinsic<GCCIntSuffix,
                          [llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                          [IntrNoMem]>;


//===----------------------------------------------------------------------===//
// Coffee Altivec Intrinsic Definitions.

let TargetPrefix = "Coffee" in {  // All intrinsics start with "llvm.Coffee.".
  // Data Stream Control.
  def int_Coffee_altivec_dss : GCCBuiltin<"__builtin_altivec_dss">,
              Intrinsic<[], [llvm_i32_ty], []>;
  def int_Coffee_altivec_dssall : GCCBuiltin<"__builtin_altivec_dssall">,
              Intrinsic<[], [], []>;
  def int_Coffee_altivec_dst : GCCBuiltin<"__builtin_altivec_dst">,
              Intrinsic<[],
                        [llvm_ptr_ty, llvm_i32_ty, llvm_i32_ty],
                        []>;
  def int_Coffee_altivec_dstt : GCCBuiltin<"__builtin_altivec_dstt">,
              Intrinsic<[],
                        [llvm_ptr_ty, llvm_i32_ty, llvm_i32_ty],
                        []>;
  def int_Coffee_altivec_dstst : GCCBuiltin<"__builtin_altivec_dstst">,
              Intrinsic<[],
                        [llvm_ptr_ty, llvm_i32_ty, llvm_i32_ty],
                        []>;
  def int_Coffee_altivec_dststt : GCCBuiltin<"__builtin_altivec_dststt">,
              Intrinsic<[],
                        [llvm_ptr_ty, llvm_i32_ty, llvm_i32_ty],
                        []>;

  // VSCR access.
  def int_Coffee_altivec_mfvscr : GCCBuiltin<"__builtin_altivec_mfvscr">,
              Intrinsic<[llvm_v8i16_ty], [], [IntrReadMem]>;
  def int_Coffee_altivec_mtvscr : GCCBuiltin<"__builtin_altivec_mtvscr">,
              Intrinsic<[], [llvm_v4i32_ty], []>;


  // Loads.  These don't map directly to GCC builtins because they represent the
  // source address with a single pointer.
  def int_Coffee_altivec_lvx :
              Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty], [IntrReadMem]>;
  def int_Coffee_altivec_lvxl :
              Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty], [IntrReadMem]>;
  def int_Coffee_altivec_lvebx :
              Intrinsic<[llvm_v16i8_ty], [llvm_ptr_ty], [IntrReadMem]>;
  def int_Coffee_altivec_lvehx :
              Intrinsic<[llvm_v8i16_ty], [llvm_ptr_ty], [IntrReadMem]>;
  def int_Coffee_altivec_lvewx :
              Intrinsic<[llvm_v4i32_ty], [llvm_ptr_ty], [IntrReadMem]>;

  // Stores.  These don't map directly to GCC builtins because they represent the
  // source address with a single pointer.
  def int_Coffee_altivec_stvx :
              Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty], []>;
  def int_Coffee_altivec_stvxl :
              Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty], []>;
  def int_Coffee_altivec_stvebx :
              Intrinsic<[], [llvm_v16i8_ty, llvm_ptr_ty], []>;
  def int_Coffee_altivec_stvehx :
              Intrinsic<[], [llvm_v8i16_ty, llvm_ptr_ty], []>;
  def int_Coffee_altivec_stvewx :
              Intrinsic<[], [llvm_v4i32_ty, llvm_ptr_ty], []>;

  // Comparisons setting a vector.
  def int_Coffee_altivec_vcmpbfp : GCCBuiltin<"__builtin_altivec_vcmpbfp">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpeqfp : GCCBuiltin<"__builtin_altivec_vcmpeqfp">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgefp : GCCBuiltin<"__builtin_altivec_vcmpgefp">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtfp : GCCBuiltin<"__builtin_altivec_vcmpgtfp">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_v4f32_ty],
                        [IntrNoMem]>;
                        
  def int_Coffee_altivec_vcmpequw : GCCBuiltin<"__builtin_altivec_vcmpequw">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsw : GCCBuiltin<"__builtin_altivec_vcmpgtsw">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtuw : GCCBuiltin<"__builtin_altivec_vcmpgtuw">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                        [IntrNoMem]>;
                        
  def int_Coffee_altivec_vcmpequh : GCCBuiltin<"__builtin_altivec_vcmpequh">,
              Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsh : GCCBuiltin<"__builtin_altivec_vcmpgtsh">,
              Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtuh : GCCBuiltin<"__builtin_altivec_vcmpgtuh">,
              Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                        [IntrNoMem]>;

  def int_Coffee_altivec_vcmpequb : GCCBuiltin<"__builtin_altivec_vcmpequb">,
              Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsb : GCCBuiltin<"__builtin_altivec_vcmpgtsb">,
              Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtub : GCCBuiltin<"__builtin_altivec_vcmpgtub">,
              Intrinsic<[llvm_v16i8_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                        [IntrNoMem]>;

  // Predicate Comparisons.  The first operand specifies interpretation of CR6.
  def int_Coffee_altivec_vcmpbfp_p : GCCBuiltin<"__builtin_altivec_vcmpbfp_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4f32_ty,llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpeqfp_p : GCCBuiltin<"__builtin_altivec_vcmpeqfp_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4f32_ty,llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgefp_p : GCCBuiltin<"__builtin_altivec_vcmpgefp_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4f32_ty,llvm_v4f32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtfp_p : GCCBuiltin<"__builtin_altivec_vcmpgtfp_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4f32_ty,llvm_v4f32_ty],
                        [IntrNoMem]>;
                        
  def int_Coffee_altivec_vcmpequw_p : GCCBuiltin<"__builtin_altivec_vcmpequw_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4i32_ty,llvm_v4i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsw_p : GCCBuiltin<"__builtin_altivec_vcmpgtsw_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4i32_ty,llvm_v4i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtuw_p : GCCBuiltin<"__builtin_altivec_vcmpgtuw_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v4i32_ty,llvm_v4i32_ty],
                        [IntrNoMem]>;
                        
  def int_Coffee_altivec_vcmpequh_p : GCCBuiltin<"__builtin_altivec_vcmpequh_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v8i16_ty,llvm_v8i16_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsh_p : GCCBuiltin<"__builtin_altivec_vcmpgtsh_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v8i16_ty,llvm_v8i16_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtuh_p : GCCBuiltin<"__builtin_altivec_vcmpgtuh_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v8i16_ty,llvm_v8i16_ty],
                        [IntrNoMem]>;

  def int_Coffee_altivec_vcmpequb_p : GCCBuiltin<"__builtin_altivec_vcmpequb_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v16i8_ty,llvm_v16i8_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtsb_p : GCCBuiltin<"__builtin_altivec_vcmpgtsb_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v16i8_ty,llvm_v16i8_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcmpgtub_p : GCCBuiltin<"__builtin_altivec_vcmpgtub_p">,
              Intrinsic<[llvm_i32_ty],[llvm_i32_ty,llvm_v16i8_ty,llvm_v16i8_ty],
                        [IntrNoMem]>;
}

// Vector average.
def int_Coffee_altivec_vavgsb : Coffee_Vec_BBB_Intrinsic<"vavgsb">;
def int_Coffee_altivec_vavgsh : Coffee_Vec_HHH_Intrinsic<"vavgsh">;
def int_Coffee_altivec_vavgsw : Coffee_Vec_WWW_Intrinsic<"vavgsw">;
def int_Coffee_altivec_vavgub : Coffee_Vec_BBB_Intrinsic<"vavgub">;
def int_Coffee_altivec_vavguh : Coffee_Vec_HHH_Intrinsic<"vavguh">;
def int_Coffee_altivec_vavguw : Coffee_Vec_WWW_Intrinsic<"vavguw">;

// Vector maximum.
def int_Coffee_altivec_vmaxfp : Coffee_Vec_FFF_Intrinsic<"vmaxfp">;
def int_Coffee_altivec_vmaxsb : Coffee_Vec_BBB_Intrinsic<"vmaxsb">;
def int_Coffee_altivec_vmaxsh : Coffee_Vec_HHH_Intrinsic<"vmaxsh">;
def int_Coffee_altivec_vmaxsw : Coffee_Vec_WWW_Intrinsic<"vmaxsw">;
def int_Coffee_altivec_vmaxub : Coffee_Vec_BBB_Intrinsic<"vmaxub">;
def int_Coffee_altivec_vmaxuh : Coffee_Vec_HHH_Intrinsic<"vmaxuh">;
def int_Coffee_altivec_vmaxuw : Coffee_Vec_WWW_Intrinsic<"vmaxuw">;

// Vector minimum.
def int_Coffee_altivec_vminfp : Coffee_Vec_FFF_Intrinsic<"vminfp">;
def int_Coffee_altivec_vminsb : Coffee_Vec_BBB_Intrinsic<"vminsb">;
def int_Coffee_altivec_vminsh : Coffee_Vec_HHH_Intrinsic<"vminsh">;
def int_Coffee_altivec_vminsw : Coffee_Vec_WWW_Intrinsic<"vminsw">;
def int_Coffee_altivec_vminub : Coffee_Vec_BBB_Intrinsic<"vminub">;
def int_Coffee_altivec_vminuh : Coffee_Vec_HHH_Intrinsic<"vminuh">;
def int_Coffee_altivec_vminuw : Coffee_Vec_WWW_Intrinsic<"vminuw">;

// Saturating adds.
def int_Coffee_altivec_vaddubs : Coffee_Vec_BBB_Intrinsic<"vaddubs">;
def int_Coffee_altivec_vaddsbs : Coffee_Vec_BBB_Intrinsic<"vaddsbs">;
def int_Coffee_altivec_vadduhs : Coffee_Vec_HHH_Intrinsic<"vadduhs">;
def int_Coffee_altivec_vaddshs : Coffee_Vec_HHH_Intrinsic<"vaddshs">;
def int_Coffee_altivec_vadduws : Coffee_Vec_WWW_Intrinsic<"vadduws">;
def int_Coffee_altivec_vaddsws : Coffee_Vec_WWW_Intrinsic<"vaddsws">;
def int_Coffee_altivec_vaddcuw : Coffee_Vec_WWW_Intrinsic<"vaddcuw">;

// Saturating subs.
def int_Coffee_altivec_vsububs : Coffee_Vec_BBB_Intrinsic<"vsububs">;
def int_Coffee_altivec_vsubsbs : Coffee_Vec_BBB_Intrinsic<"vsubsbs">;
def int_Coffee_altivec_vsubuhs : Coffee_Vec_HHH_Intrinsic<"vsubuhs">;
def int_Coffee_altivec_vsubshs : Coffee_Vec_HHH_Intrinsic<"vsubshs">;
def int_Coffee_altivec_vsubuws : Coffee_Vec_WWW_Intrinsic<"vsubuws">;
def int_Coffee_altivec_vsubsws : Coffee_Vec_WWW_Intrinsic<"vsubsws">;
def int_Coffee_altivec_vsubcuw : Coffee_Vec_WWW_Intrinsic<"vsubcuw">;

let TargetPrefix = "Coffee" in {  // All Coffee intrinsics start with "llvm.Coffee.".
  // Saturating multiply-adds.
  def int_Coffee_altivec_vmhaddshs : GCCBuiltin<"__builtin_altivec_vmhaddshs">,
              Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty,
                         llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmhraddshs : GCCBuiltin<"__builtin_altivec_vmhraddshs">,
              Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty,
                         llvm_v8i16_ty, llvm_v8i16_ty], [IntrNoMem]>;

  def int_Coffee_altivec_vmaddfp : GCCBuiltin<"__builtin_altivec_vmaddfp">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty,
                         llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vnmsubfp : GCCBuiltin<"__builtin_altivec_vnmsubfp">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty,
                         llvm_v4f32_ty, llvm_v4f32_ty], [IntrNoMem]>;

  // Vector Multiply Sum Intructions.
  def int_Coffee_altivec_vmsummbm : GCCBuiltin<"__builtin_altivec_vmsummbm">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v16i8_ty, llvm_v16i8_ty,
                       llvm_v4i32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmsumshm : GCCBuiltin<"__builtin_altivec_vmsumshm">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty,
                       llvm_v4i32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmsumshs : GCCBuiltin<"__builtin_altivec_vmsumshs">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty, 
                       llvm_v4i32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmsumubm : GCCBuiltin<"__builtin_altivec_vmsumubm">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v16i8_ty, llvm_v16i8_ty, 
                       llvm_v4i32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmsumuhm : GCCBuiltin<"__builtin_altivec_vmsumuhm">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty,
                       llvm_v4i32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vmsumuhs : GCCBuiltin<"__builtin_altivec_vmsumuhs">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty,
                       llvm_v4i32_ty], [IntrNoMem]>;

  // Vector Multiply Intructions.
  def int_Coffee_altivec_vmulesb : GCCBuiltin<"__builtin_altivec_vmulesb">,
          Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmulesh : GCCBuiltin<"__builtin_altivec_vmulesh">,
          Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmuleub : GCCBuiltin<"__builtin_altivec_vmuleub">,
          Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmuleuh : GCCBuiltin<"__builtin_altivec_vmuleuh">,
          Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                    [IntrNoMem]>;

  def int_Coffee_altivec_vmulosb : GCCBuiltin<"__builtin_altivec_vmulosb">,
          Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmulosh : GCCBuiltin<"__builtin_altivec_vmulosh">,
          Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmuloub : GCCBuiltin<"__builtin_altivec_vmuloub">,
          Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty, llvm_v16i8_ty],
                    [IntrNoMem]>;
  def int_Coffee_altivec_vmulouh : GCCBuiltin<"__builtin_altivec_vmulouh">,
          Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                    [IntrNoMem]>;

  // Vector Sum Intructions.
  def int_Coffee_altivec_vsumsws : GCCBuiltin<"__builtin_altivec_vsumsws">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vsum2sws : GCCBuiltin<"__builtin_altivec_vsum2sws">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vsum4sbs : GCCBuiltin<"__builtin_altivec_vsum4sbs">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v16i8_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vsum4shs : GCCBuiltin<"__builtin_altivec_vsum4shs">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vsum4ubs : GCCBuiltin<"__builtin_altivec_vsum4ubs">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v16i8_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;

  // Other multiplies.
  def int_Coffee_altivec_vmladduhm : GCCBuiltin<"__builtin_altivec_vmladduhm">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v8i16_ty, llvm_v8i16_ty, 
                       llvm_v8i16_ty], [IntrNoMem]>;

  // Packs.
  def int_Coffee_altivec_vpkpx : GCCBuiltin<"__builtin_altivec_vpkpx">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vpkshss : GCCBuiltin<"__builtin_altivec_vpkshss">,
            Intrinsic<[llvm_v16i8_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vpkshus : GCCBuiltin<"__builtin_altivec_vpkshus">,
            Intrinsic<[llvm_v16i8_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vpkswss : GCCBuiltin<"__builtin_altivec_vpkswss">,
            Intrinsic<[llvm_v16i8_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  def int_Coffee_altivec_vpkswus : GCCBuiltin<"__builtin_altivec_vpkswus">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;
  // vpkuhum is lowered to a shuffle.
  def int_Coffee_altivec_vpkuhus : GCCBuiltin<"__builtin_altivec_vpkuhus">,
            Intrinsic<[llvm_v16i8_ty], [llvm_v8i16_ty, llvm_v8i16_ty],
                      [IntrNoMem]>;
  // vpkuwum is lowered to a shuffle.
  def int_Coffee_altivec_vpkuwus : GCCBuiltin<"__builtin_altivec_vpkuwus">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v4i32_ty, llvm_v4i32_ty],
                      [IntrNoMem]>;

  // Unpacks.
  def int_Coffee_altivec_vupkhpx : GCCBuiltin<"__builtin_altivec_vupkhpx">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vupkhsb : GCCBuiltin<"__builtin_altivec_vupkhsb">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vupkhsh : GCCBuiltin<"__builtin_altivec_vupkhsh">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vupklpx : GCCBuiltin<"__builtin_altivec_vupklpx">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vupklsb : GCCBuiltin<"__builtin_altivec_vupklsb">,
            Intrinsic<[llvm_v8i16_ty], [llvm_v16i8_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vupklsh : GCCBuiltin<"__builtin_altivec_vupklsh">,
            Intrinsic<[llvm_v4i32_ty], [llvm_v8i16_ty], [IntrNoMem]>;


  // FP <-> integer conversion.
  def int_Coffee_altivec_vcfsx : GCCBuiltin<"__builtin_altivec_vcfsx">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty, llvm_i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vcfux : GCCBuiltin<"__builtin_altivec_vcfux">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4i32_ty, llvm_i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vctsxs : GCCBuiltin<"__builtin_altivec_vctsxs">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_i32_ty],
                        [IntrNoMem]>;
  def int_Coffee_altivec_vctuxs : GCCBuiltin<"__builtin_altivec_vctuxs">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4f32_ty, llvm_i32_ty],
                        [IntrNoMem]>;

  def int_Coffee_altivec_vrfim : GCCBuiltin<"__builtin_altivec_vrfim">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vrfin : GCCBuiltin<"__builtin_altivec_vrfin">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vrfip : GCCBuiltin<"__builtin_altivec_vrfip">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vrfiz : GCCBuiltin<"__builtin_altivec_vrfiz">,
              Intrinsic<[llvm_v4f32_ty], [llvm_v4f32_ty], [IntrNoMem]>;
}

def int_Coffee_altivec_vsl   : Coffee_Vec_WWW_Intrinsic<"vsl">;
def int_Coffee_altivec_vslo  : Coffee_Vec_WWW_Intrinsic<"vslo">;

def int_Coffee_altivec_vslb  : Coffee_Vec_BBB_Intrinsic<"vslb">;
def int_Coffee_altivec_vslh  : Coffee_Vec_HHH_Intrinsic<"vslh">;
def int_Coffee_altivec_vslw  : Coffee_Vec_WWW_Intrinsic<"vslw">;

// Right Shifts.
def int_Coffee_altivec_vsr   : Coffee_Vec_WWW_Intrinsic<"vsr">;
def int_Coffee_altivec_vsro  : Coffee_Vec_WWW_Intrinsic<"vsro">;
  
def int_Coffee_altivec_vsrb  : Coffee_Vec_BBB_Intrinsic<"vsrb">;
def int_Coffee_altivec_vsrh  : Coffee_Vec_HHH_Intrinsic<"vsrh">;
def int_Coffee_altivec_vsrw  : Coffee_Vec_WWW_Intrinsic<"vsrw">;
def int_Coffee_altivec_vsrab : Coffee_Vec_BBB_Intrinsic<"vsrab">;
def int_Coffee_altivec_vsrah : Coffee_Vec_HHH_Intrinsic<"vsrah">;
def int_Coffee_altivec_vsraw : Coffee_Vec_WWW_Intrinsic<"vsraw">;

// Rotates.
def int_Coffee_altivec_vrlb  : Coffee_Vec_BBB_Intrinsic<"vrlb">;
def int_Coffee_altivec_vrlh  : Coffee_Vec_HHH_Intrinsic<"vrlh">;
def int_Coffee_altivec_vrlw  : Coffee_Vec_WWW_Intrinsic<"vrlw">;

let TargetPrefix = "Coffee" in {  // All Coffee intrinsics start with "llvm.Coffee.".
  // Miscellaneous.
  def int_Coffee_altivec_lvsl :
              Intrinsic<[llvm_v16i8_ty], [llvm_ptr_ty], [IntrNoMem]>;
  def int_Coffee_altivec_lvsr :
              Intrinsic<[llvm_v16i8_ty], [llvm_ptr_ty], [IntrNoMem]>;

  def int_Coffee_altivec_vperm : GCCBuiltin<"__builtin_altivec_vperm_4si">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, 
                         llvm_v4i32_ty, llvm_v16i8_ty], [IntrNoMem]>;
  def int_Coffee_altivec_vsel : GCCBuiltin<"__builtin_altivec_vsel_4si">,
              Intrinsic<[llvm_v4i32_ty], [llvm_v4i32_ty, 
                         llvm_v4i32_ty, llvm_v4i32_ty], [IntrNoMem]>;
}

def int_Coffee_altivec_vexptefp  : Coffee_Vec_FF_Intrinsic<"vexptefp">;
def int_Coffee_altivec_vlogefp   : Coffee_Vec_FF_Intrinsic<"vlogefp">;
def int_Coffee_altivec_vrefp     : Coffee_Vec_FF_Intrinsic<"vrefp">;
def int_Coffee_altivec_vrsqrtefp : Coffee_Vec_FF_Intrinsic<"vrsqrtefp">;
