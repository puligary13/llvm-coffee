//=========== CoffeeCLIntrinsic.td  ===================//
// Backend definition of Instrinsic defined for Coffee //
// CL. The intrinsic pattern is defined in front end   //
//=====================================================//

class COFFEECL_READ_SPECIAL_REGISTER_R64<string regname, Intrinsic intop>;


class COFFEECL_READ_SPECIAL_REGISTER_R32<string regname, Intrinsic intop>;

class F_MATH_1<string OpcStr, Intrinsic IntOP>;


// TODO Add read vector-version of special registers

def COFFEECL_READ_TID_X   : COFFEECL_READ_SPECIAL_REGISTER_R32<"tid.x",
                                                     int_coffeecl_read_tid_x>;
def COFFEECL_READ_TID_Y   : COFFEECL_READ_SPECIAL_REGISTER_R32<"tid.y",
                                                     int_coffeecl_read_tid_y>;
def COFFEECL_READ_TID_Z   : COFFEECL_READ_SPECIAL_REGISTER_R32<"tid.z",
                                                     int_coffeecl_read_tid_z>;
def COFFEECL_READ_TID_W   : COFFEECL_READ_SPECIAL_REGISTER_R32<"tid.w",
                                                     int_coffeecl_read_tid_w>;

def COFFEECL_READ_NTID_X   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ntid.x",
                                                      int_coffeecl_read_ntid_x>;
def COFFEECL_READ_NTID_Y   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ntid.y",
                                                      int_coffeecl_read_ntid_y>;
def COFFEECL_READ_NTID_Z   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ntid.z",
                                                      int_coffeecl_read_ntid_z>;
def COFFEECL_READ_NTID_W   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ntid.w",
                                                      int_coffeecl_read_ntid_w>;

def COFFEECL_READ_LANEID  : COFFEECL_READ_SPECIAL_REGISTER_R32<"laneid",
                                                     int_coffeecl_read_laneid>;
def COFFEECL_READ_WARPID  : COFFEECL_READ_SPECIAL_REGISTER_R32<"warpid",
                                                     int_coffeecl_read_warpid>;
def COFFEECL_READ_NWARPID : COFFEECL_READ_SPECIAL_REGISTER_R32<"nwarpid",
                                                     int_coffeecl_read_nwarpid>;

def COFFEECL_READ_CTAID_X   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ctaid.x",
                                                       int_coffeecl_read_ctaid_x>;
def COFFEECL_READ_CTAID_Y   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ctaid.y",
                                                       int_coffeecl_read_ctaid_y>;
def COFFEECL_READ_CTAID_Z   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ctaid.z",
                                                       int_coffeecl_read_ctaid_z>;
def COFFEECL_READ_CTAID_W   : COFFEECL_READ_SPECIAL_REGISTER_R32<"ctaid.w",
                                                       int_coffeecl_read_ctaid_w>;

def COFFEECL_READ_NCTAID_X   : COFFEECL_READ_SPECIAL_REGISTER_R32<"nctaid.x",
                                                        int_coffeecl_read_nctaid_x>;
def COFFEECL_READ_NCTAID_Y   : COFFEECL_READ_SPECIAL_REGISTER_R32<"nctaid.y",
                                                        int_coffeecl_read_nctaid_y>;
def COFFEECL_READ_NCTAID_Z   : COFFEECL_READ_SPECIAL_REGISTER_R32<"nctaid.z",
                                                        int_coffeecl_read_nctaid_z>;
def COFFEECL_READ_NCTAID_W   : COFFEECL_READ_SPECIAL_REGISTER_R32<"nctaid.w",
                                                        int_coffeecl_read_nctaid_w>;

def COFFEECL_READ_SMID  : COFFEECL_READ_SPECIAL_REGISTER_R32<"smid",
                                                   int_coffeecl_read_smid>;
def COFFEECL_READ_NSMID  : COFFEECL_READ_SPECIAL_REGISTER_R32<"nsmid",
                                                    int_coffeecl_read_nsmid>;
def COFFEECL_READ_GRIDID  : COFFEECL_READ_SPECIAL_REGISTER_R32<"gridid",
                                                     int_coffeecl_read_gridid>;

def COFFEECL_READ_LANEMASK_EQ
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"lanemask_eq", int_coffeecl_read_lanemask_eq>;
def COFFEECL_READ_LANEMASK_LE
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"lanemask_le", int_coffeecl_read_lanemask_le>;
def COFFEECL_READ_LANEMASK_LT
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"lanemask_lt", int_coffeecl_read_lanemask_lt>;
def COFFEECL_READ_LANEMASK_GE
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"lanemask_ge", int_coffeecl_read_lanemask_ge>;
def COFFEECL_READ_LANEMASK_GT
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"lanemask_gt", int_coffeecl_read_lanemask_gt>;

def COFFEECL_READ_CLOCK
  : COFFEECL_READ_SPECIAL_REGISTER_R32<"clock", int_coffeecl_read_clock>;
def COFFEECL_READ_CLOCK64
  : COFFEECL_READ_SPECIAL_REGISTER_R64<"clock64", int_coffeecl_read_clock64>;

def COFFEECL_READ_PM0 : COFFEECL_READ_SPECIAL_REGISTER_R32<"pm0", int_coffeecl_read_pm0>;
def COFFEECL_READ_PM1 : COFFEECL_READ_SPECIAL_REGISTER_R32<"pm1", int_coffeecl_read_pm1>;
def COFFEECL_READ_PM2 : COFFEECL_READ_SPECIAL_REGISTER_R32<"pm2", int_coffeecl_read_pm2>;
def COFFEECL_READ_PM3 : COFFEECL_READ_SPECIAL_REGISTER_R32<"pm3", int_coffeecl_read_pm3>;

// PTX Parallel Synchronization and Communication Intrinsics

def COFFEECL_BAR_SYNC : InstCoffeeCL<(outs), (ins i32imm:$i), "bar.sync $i",
                             [(int_coffeecl_bar_sync imm:$i)], NoItinerary, FrmFI>;



def INT_COFFEECL_ABS_I : F_MATH_1<"crema_abs_configuration", int_coffeecl_abs_i>;

def INT_COFFEECL_SIN_APPROX_F : F_MATH_1<"crema_sin_approx_f_configuration", int_coffeecl_sin_approx_f>;

def INT_COFFEECL_SIN : F_MATH_1<"crema_sin_configuration", int_coffeecl_sin>;

def INT_COFFEECL_GET_GLOBAL_ID : F_MATH_1<"crema_get_global_id", int_coffeecl_get_global_id>;

def INT_COFFEECL_LOAD : F_MATH_1<"crema_load", int_coffeecl_load>;

def INT_COFFEECL_GETELEMENTPTR : F_MATH_1<"crema_getelementptr", int_coffeecl_getelementptr>;

// Guoqing: continue from here, add sin function as intrinsic and see how the function call
// behave in llvm backends.

