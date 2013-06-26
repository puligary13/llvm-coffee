//=========== InstrinsicsCoffeecl.td===================//
// The implementation of this file is corresponding to //
// the definition in BuiltinsCOFFEECL.def and this is  //
// used by target specific intrinsic implemenation in  //
// backend CoffeeCLIntrinsics.td                       //
//=====================================================//


multiclass COFFEECLReadSpecialRegisterIntrinsic_v4i32<string prefix> {
// FIXME: Do we need the 128-bit integer type version?
//    def _r64   : Intrinsic<[llvm_i128_ty],   [], [IntrNoMem]>;

// FIXME: Enable this once v4i32 support is enabled in back-end.
//    def _v4i16 : Intrinsic<[llvm_v4i32_ty], [], [IntrNoMem]>;

  def _x     : Intrinsic<[llvm_i32_ty], [], [IntrNoMem]>,
               GCCBuiltin<!strconcat(prefix, "_x")>;
  def _y     : Intrinsic<[llvm_i32_ty], [], [IntrNoMem]>,
               GCCBuiltin<!strconcat(prefix, "_y")>;
  def _z     : Intrinsic<[llvm_i32_ty], [], [IntrNoMem]>,
               GCCBuiltin<!strconcat(prefix, "_z")>;
  def _w     : Intrinsic<[llvm_i32_ty], [], [IntrNoMem]>,
               GCCBuiltin<!strconcat(prefix, "_w")>;
}

class COFFEECLReadSpecialRegisterIntrinsic_r32<string name>
  : Intrinsic<[llvm_i32_ty], [], [IntrNoMem]>,
    GCCBuiltin<name>;

class COFFEECLReadSpecialRegisterIntrinsic_r64<string name>
  : Intrinsic<[llvm_i64_ty], [], [IntrNoMem]>,
    GCCBuiltin<name>;

defm int_coffeecl_read_tid        : COFFEECLReadSpecialRegisterIntrinsic_v4i32
                               <"__builtin_coffeecl_read_tid">;
defm int_coffeecl_read_ntid       : COFFEECLReadSpecialRegisterIntrinsic_v4i32
                               <"__builtin_coffeecl_read_ntid">;

def int_coffeecl_read_laneid      : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_laneid">;
def int_coffeecl_read_warpid      : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_warpid">;
def int_coffeecl_read_nwarpid     : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_nwarpid">;

defm int_coffeecl_read_ctaid      : COFFEECLReadSpecialRegisterIntrinsic_v4i32
                               <"__builtin_coffeecl_read_ctaid">;
defm int_coffeecl_read_nctaid     : COFFEECLReadSpecialRegisterIntrinsic_v4i32
                               <"__builtin_coffeecl_read_nctaid">;

def int_coffeecl_read_smid        : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_smid">;
def int_coffeecl_read_nsmid       : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_nsmid">;
def int_coffeecl_read_gridid      : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_gridid">;

def int_coffeecl_read_lanemask_eq : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_lanemask_eq">;
def int_coffeecl_read_lanemask_le : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_lanemask_le">;
def int_coffeecl_read_lanemask_lt : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_lanemask_lt">;
def int_coffeecl_read_lanemask_ge : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_lanemask_ge">;
def int_coffeecl_read_lanemask_gt : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_lanemask_gt">;

def int_coffeecl_read_clock       : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_clock">;
def int_coffeecl_read_clock64     : COFFEECLReadSpecialRegisterIntrinsic_r64
                               <"__builtin_coffeecl_read_clock64">;

def int_coffeecl_read_pm0         : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_pm0">;
def int_coffeecl_read_pm1         : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_pm1">;
def int_coffeecl_read_pm2         : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_pm2">;
def int_coffeecl_read_pm3         : COFFEECLReadSpecialRegisterIntrinsic_r32
                               <"__builtin_coffeecl_read_pm3">;

def int_coffeecl_bar_sync : Intrinsic<[], [llvm_i32_ty], []>,
                       GCCBuiltin<"__builtin_coffeecl_bar_sync">;



def int_coffeecl_abs_i : Intrinsic<[llvm_i32_ty], [llvm_i32_ty]>,
                       GCCBuiltin<"__coffeecl_abs_i">;

def int_coffeecl_sin_approx_f : GCCBuiltin<"__coffeecl_sin_approx_f">,
    Intrinsic<[llvm_float_ty], [llvm_float_ty], [IntrNoMem]>;

def int_coffeecl_sin : GCCBuiltin<"__coffeecl_sin">,
    Intrinsic<[llvm_float_ty], [llvm_float_ty], [IntrNoMem]>;

def int_coffeecl_get_global_id : GCCBuiltin<"__coffeecl_get_global_id">,
    Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;


def int_coffeecl_load : GCCBuiltin<"__coffeecl_load">,
    Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_coffeecl_getelementptr : GCCBuiltin<"__coffeecl_getelementptr">,
    Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;
