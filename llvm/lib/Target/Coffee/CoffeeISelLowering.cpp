//===-- CoffeeISelLowering.cpp - Coffee DAG Lowering Implementation -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeISelLowering class.
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "coffee-lower"
#include "CoffeeISelLowering.h"
#include "CoffeeMachineFunctionInfo.h"
#include "CoffeePerfectShuffle.h"
#include "CoffeeTargetMachine.h"
#include "MCTargetDesc/CoffeePredicates.h"
#include "CoffeeTargetObjectFile.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Function.h"
#include "llvm/Intrinsics.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
using namespace llvm;


static cl::opt<bool> EnableCoffeePreinc("enable-Coffee-preinc",
cl::desc("enable preincrement load/store generation on Coffee (experimental)"),
                                     cl::Hidden);


static const uint16_t IntRegs[4] = {
  Coffee::A0, Coffee::A1, Coffee::A2, Coffee::A3
};


static TargetLoweringObjectFile *CreateTLOF(const CoffeeTargetMachine &TM) {
  return new TargetLoweringObjectFileELF();
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// TODO: Implement a generic logic using tblgen that can support this.
// Coffee ABI rules:
// ---
// i32 - Passed in A0, A1, A2, A3 and stack
// f32 - Only passed in f32 registers if no int reg has been used yet to hold
//       an argument. Otherwise, passed in A1, A2, A3 and stack.
// f64 - Only passed in two aliased f32 registers if no int reg has been used
//       yet to hold an argument. Otherwise, use A2, A3 and stack. If A1 is
//       not used, it must be shadowed. If only A3 is avaiable, shadow it and
//       go to stack.
//
//  For vararg functions, all arguments are passed in A0, A1, A2, A3 and stack.
//===----------------------------------------------------------------------===//

static bool CC_Coffee(unsigned ValNo, MVT ValVT,
                       MVT LocVT, CCValAssign::LocInfo LocInfo,
                       ISD::ArgFlagsTy ArgFlags, CCState &State) {

  static const unsigned IntRegsSize=4, FloatRegsSize=2;

  static const uint16_t IntRegs[] = {
      Coffee::A0, Coffee::A1, Coffee::A2, Coffee::A3
  };
  /*static const uint16_t F32Regs[] = {
      Coffee::F12, Coffee::F14
  };
  static const uint16_t F64Regs[] = {
      Coffee::D6, Coffee::D7
  };*/

  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;

  // Promote i8 and i16
  if (LocVT == MVT::i8 || LocVT == MVT::i16) {
    LocVT = MVT::i32;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }

  unsigned Reg;

  // f32 and f64 are allocated in A0, A1, A2, A3 when either of the following
  // is true: function is vararg, argument is 3rd or higher, there is previous
  // argument which is not f32 or f64.
 // bool AllocateFloatsInIntReg = State.isVarArg() || ValNo > 1
 //     || State.getFirstUnallocated(F32Regs, FloatRegsSize) != ValNo;
  unsigned OrigAlign = ArgFlags.getOrigAlign();
  // bool isI64 = (ValVT == MVT::i32 && OrigAlign == 8);

  if (ValVT == MVT::i32 /* || (ValVT == MVT::f32 && AllocateFloatsInIntReg)*/) {
    Reg = State.AllocateReg(IntRegs, IntRegsSize);
    // If this is the first part of an i64 arg,
    // the allocated register must be either A0 or A2.
    //if (isI64 && (Reg == Mips::A1 || Reg == Mips::A3))
    //  Reg = State.AllocateReg(IntRegs, IntRegsSize);
    LocVT = MVT::i32;
  } /*else if (ValVT == MVT::f64 && AllocateFloatsInIntReg) {
    // Allocate int register and shadow next int register. If first
    // available register is Mips::A1 or Mips::A3, shadow it too.
    Reg = State.AllocateReg(IntRegs, IntRegsSize);
    if (Reg == Mips::A1 || Reg == Mips::A3)
      Reg = State.AllocateReg(IntRegs, IntRegsSize);
    State.AllocateReg(IntRegs, IntRegsSize);
    LocVT = MVT::i32;
  } else if (ValVT.isFloatingPoint() && !AllocateFloatsInIntReg) {
    // we are guaranteed to find an available float register
    if (ValVT == MVT::f32) {
      Reg = State.AllocateReg(F32Regs, FloatRegsSize);
      // Shadow int register
      State.AllocateReg(IntRegs, IntRegsSize);
    } else {
      Reg = State.AllocateReg(F64Regs, FloatRegsSize);
      // Shadow int registers
      unsigned Reg2 = State.AllocateReg(IntRegs, IntRegsSize);
      if (Reg2 == Mips::A1 || Reg2 == Mips::A3)
        State.AllocateReg(IntRegs, IntRegsSize);
      State.AllocateReg(IntRegs, IntRegsSize);
    }
  }*/ else
    llvm_unreachable("Cannot handle this ValVT.");

  if (!Reg) {
    unsigned Offset = State.AllocateStack(ValVT.getSizeInBits() >> 3,
                                          OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));

  return false;
}

#include "CoffeeGenCallingConv.inc"

CoffeeTargetLowering::CoffeeTargetLowering(CoffeeTargetMachine &TM)
    : TargetLowering(TM, new TargetLoweringObjectFileELF()) {

    // Set up the register classes.
    setBooleanContents(ZeroOrOneBooleanContent);
    setBooleanVectorContents(ZeroOrOneBooleanContent); // FIXME: Is this correct?

    addRegisterClass(MVT::i32, &Coffee::GPRCRegClass);
    addRegisterClass(MVT::f32, &Coffee::FPRCRegClass);



    ///////

    // Load extented operations for i1 types must be promoted
    setLoadExtAction(ISD::EXTLOAD,  MVT::i1,  Promote);
    setLoadExtAction(ISD::ZEXTLOAD, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, MVT::i1,  Promote);


    setLoadExtAction(ISD::EXTLOAD, MVT::f32, Expand);
    setTruncStoreAction(MVT::f64, MVT::f32, Expand);

    AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i32);



    setOperationAction(ISD::SETCC,     MVT::i32, Expand);
    setOperationAction(ISD::SETCC,     MVT::f32, Expand);
    setOperationAction(ISD::SETCC,     MVT::f64, Expand);
    setOperationAction(ISD::SELECT,    MVT::i32, Custom);
    setOperationAction(ISD::SELECT,    MVT::f32, Custom);
    setOperationAction(ISD::SELECT,    MVT::f64, Custom);
    setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);
    setOperationAction(ISD::SELECT_CC, MVT::f32, Custom);
    setOperationAction(ISD::SELECT_CC, MVT::f64, Custom);

    setOperationAction(ISD::BRCOND,    MVT::Other, Expand);
    setOperationAction(ISD::BR_CC,     MVT::i32,   Custom);
    setOperationAction(ISD::BR_CC,     MVT::f32,   Custom);
    setOperationAction(ISD::BR_CC,     MVT::f64,   Custom);
    setOperationAction(ISD::BR_CC,     MVT::Other, Custom);

    setOperationAction(ISD::BR_JT,     MVT::Other, Custom);

    // Coffee Custom Operations
    setOperationAction(ISD::GlobalAddress,      MVT::i32,   Custom);
    setOperationAction(ISD::BlockAddress,       MVT::i32,   Custom);
    setOperationAction(ISD::GlobalTLSAddress,   MVT::i32,   Custom);
    setOperationAction(ISD::JumpTable,          MVT::i32,   Custom);
    setOperationAction(ISD::ConstantPool,       MVT::i32,   Custom);
    setOperationAction(ISD::SELECT,             MVT::f32,   Custom);
    setOperationAction(ISD::SELECT,             MVT::f64,   Custom);
    setOperationAction(ISD::SELECT,             MVT::i32,   Custom);




    setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i32,   Custom);
    setOperationAction(ISD::VASTART,            MVT::Other, Custom);
    setOperationAction(ISD::FCOPYSIGN,          MVT::f32,   Custom);
    setOperationAction(ISD::FCOPYSIGN,          MVT::f64,   Custom);
    setOperationAction(ISD::MEMBARRIER,         MVT::Other, Custom);
    setOperationAction(ISD::ATOMIC_FENCE,       MVT::Other, Custom);

    if (!TM.Options.NoNaNsFPMath) {
      setOperationAction(ISD::FABS,             MVT::f32,   Custom);
      setOperationAction(ISD::FABS,             MVT::f64,   Custom);
    }

    setOperationAction(ISD::ADD,                MVT::i32,   Custom);

    setOperationAction(ISD::SDIV, MVT::i32, Expand);
    setOperationAction(ISD::SREM, MVT::i32, Expand);
    setOperationAction(ISD::UDIV, MVT::i32, Expand);
    setOperationAction(ISD::UREM, MVT::i32, Expand);
    setOperationAction(ISD::SDIV, MVT::i64, Expand);
    setOperationAction(ISD::SREM, MVT::i64, Expand);
    setOperationAction(ISD::UDIV, MVT::i64, Expand);
    setOperationAction(ISD::UREM, MVT::i64, Expand);

    // Operations not directly supported by coffee.
    setOperationAction(ISD::BR_JT,             MVT::Other, Expand);

    setOperationAction(ISD::SELECT_CC,         MVT::Other, Expand);
    setOperationAction(ISD::UINT_TO_FP,        MVT::i32,   Expand);
    setOperationAction(ISD::UINT_TO_FP,        MVT::i64,   Expand);
    setOperationAction(ISD::FP_TO_UINT,        MVT::i32,   Expand);
    setOperationAction(ISD::FP_TO_UINT,        MVT::i64,   Expand);
    setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1,    Expand);
    setOperationAction(ISD::CTPOP,             MVT::i32,   Expand);
    setOperationAction(ISD::CTPOP,             MVT::i64,   Expand);
    setOperationAction(ISD::CTTZ,              MVT::i32,   Expand);
    setOperationAction(ISD::CTTZ,              MVT::i64,   Expand);
    setOperationAction(ISD::CTTZ_ZERO_UNDEF,   MVT::i32,   Expand);
    setOperationAction(ISD::CTTZ_ZERO_UNDEF,   MVT::i64,   Expand);
    setOperationAction(ISD::CTLZ_ZERO_UNDEF,   MVT::i32,   Expand);
    setOperationAction(ISD::CTLZ_ZERO_UNDEF,   MVT::i64,   Expand);
    setOperationAction(ISD::ROTL,              MVT::i32,   Expand);
    setOperationAction(ISD::ROTL,              MVT::i64,   Expand);
    setOperationAction(ISD::ROTR, MVT::i32,   Expand);
    setOperationAction(ISD::SHL_PARTS,         MVT::i32,   Expand);
    setOperationAction(ISD::SRA_PARTS,         MVT::i32,   Expand);
    setOperationAction(ISD::SRL_PARTS,         MVT::i32,   Expand);
    setOperationAction(ISD::FSIN,              MVT::f32,   Expand);
    setOperationAction(ISD::FSIN,              MVT::f64,   Expand);
    setOperationAction(ISD::FCOS,              MVT::f32,   Expand);
    setOperationAction(ISD::FCOS,              MVT::f64,   Expand);
    setOperationAction(ISD::FPOWI,             MVT::f32,   Expand);
    setOperationAction(ISD::FPOW,              MVT::f32,   Expand);
    setOperationAction(ISD::FPOW,              MVT::f64,   Expand);
    setOperationAction(ISD::FLOG,              MVT::f32,   Expand);
    setOperationAction(ISD::FLOG2,             MVT::f32,   Expand);
    setOperationAction(ISD::FLOG10,            MVT::f32,   Expand);
    setOperationAction(ISD::FEXP,              MVT::f32,   Expand);
    setOperationAction(ISD::FMA,               MVT::f32,   Expand);
    setOperationAction(ISD::FMA,               MVT::f64,   Expand);
    setOperationAction(ISD::FREM,              MVT::f32,   Expand);
    setOperationAction(ISD::FREM,              MVT::f64,   Expand);

    if (!TM.Options.NoNaNsFPMath) {
      setOperationAction(ISD::FNEG,             MVT::f32,   Expand);
      setOperationAction(ISD::FNEG,             MVT::f64,   Expand);
    }

    setOperationAction(ISD::EXCEPTIONADDR,     MVT::i32, Expand);
    setOperationAction(ISD::EXCEPTIONADDR,     MVT::i64, Expand);
    setOperationAction(ISD::EHSELECTION,       MVT::i32, Expand);
    setOperationAction(ISD::EHSELECTION,       MVT::i64, Expand);

    setOperationAction(ISD::VAARG,             MVT::Other, Expand);
    setOperationAction(ISD::VACOPY,            MVT::Other, Expand);
    setOperationAction(ISD::VAEND,             MVT::Other, Expand);

    // Use the default for now
    setOperationAction(ISD::STACKSAVE,         MVT::Other, Expand);
    setOperationAction(ISD::STACKRESTORE,      MVT::Other, Expand);

    setOperationAction(ISD::ATOMIC_LOAD,       MVT::i32,    Expand);
    setOperationAction(ISD::ATOMIC_LOAD,       MVT::i64,    Expand);
    setOperationAction(ISD::ATOMIC_STORE,      MVT::i32,    Expand);
    setOperationAction(ISD::ATOMIC_STORE,      MVT::i64,    Expand);

    //////


    setTargetDAGCombine(ISD::ADDE);
    setTargetDAGCombine(ISD::SUBE);
    setTargetDAGCombine(ISD::SDIVREM);
    setTargetDAGCombine(ISD::UDIVREM);
    setTargetDAGCombine(ISD::SELECT);
    setTargetDAGCombine(ISD::AND);
    setTargetDAGCombine(ISD::OR);


    setMinFunctionAlignment(2);

    setInsertFencesForAtomic(true);

    setSchedulingPreference(Sched::Source);

    computeRegisterProperties();
}

// AddLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value.  It also creates a corresponding
// virtual register for it.
static unsigned
AddLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  assert(RC->contains(PReg) && "Not the correct regclass!");
  unsigned VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}

#include "CoffeeGenRegisterInfo.inc"

SDValue
CoffeeTargetLowering::LowerFormalArguments(SDValue Chain,
                                          CallingConv::ID CallConv, bool isVarArg,
                                          const SmallVectorImpl<ISD::InputArg>
                                            &Ins,
                                          DebugLoc dl, SelectionDAG &DAG,
                                          SmallVectorImpl<SDValue> &InVals)
                                            const {
    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();

    CoffeeFI->setVarArgsFrameIndex(0);

    // Used with vargs to acumulate store chains.
    std::vector<SDValue> OutChains;

    // Assign locations to all of the incoming arguments.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   getTargetMachine(), ArgLocs, *DAG.getContext());


    CCInfo.AnalyzeFormalArguments(Ins, CC_Coffee);

    Function::const_arg_iterator FuncArg =
      DAG.getMachineFunction().getFunction()->arg_begin();
    int LastFI = 0;// CoffeeFI->LastInArgFI is 0 at the entry of this function.

    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i, ++FuncArg) {
      CCValAssign &VA = ArgLocs[i];
      EVT ValVT = VA.getValVT();
      ISD::ArgFlagsTy Flags = Ins[i].Flags;
      bool IsRegLoc = VA.isRegLoc();

      if (Flags.isByVal()) {
          llvm_unreachable("coffee: don't support isByVal type for now");
      }

      // Arguments stored on registers
      if (IsRegLoc) {
        EVT RegVT = VA.getLocVT();
        unsigned ArgReg = VA.getLocReg();
        const TargetRegisterClass *RC;

        if (RegVT == MVT::i32)
          RC = &Coffee::GPRCRegClass;
        else
          llvm_unreachable("Coffee: RegVT not supported by FormalArguments Lowering");

        // Transform the arguments stored on
        // physical registers into virtual ones
        unsigned Reg = AddLiveIn(DAG.getMachineFunction(), ArgReg, RC);
        SDValue ArgValue = DAG.getCopyFromReg(Chain, dl, Reg, RegVT);

        // If this is an 8 or 16-bit value, it has been passed promoted
        // to 32 bits.  Insert an assert[sz]ext to capture this, then
        // truncate to the right size.
        if (VA.getLocInfo() != CCValAssign::Full) {
          unsigned Opcode = 0;
          if (VA.getLocInfo() == CCValAssign::SExt)
            Opcode = ISD::AssertSext;
          else if (VA.getLocInfo() == CCValAssign::ZExt)
            Opcode = ISD::AssertZext;
          if (Opcode)
            ArgValue = DAG.getNode(Opcode, dl, RegVT, ArgValue,
                                   DAG.getValueType(ValVT));
          ArgValue = DAG.getNode(ISD::TRUNCATE, dl, ValVT, ArgValue);
        }

        // Handle floating point arguments passed in integer registers.
        if ((RegVT == MVT::i32 && ValVT == MVT::f32) ||
            (RegVT == MVT::i64 && ValVT == MVT::f64))
          ArgValue = DAG.getNode(ISD::BITCAST, dl, ValVT, ArgValue);
        else if (RegVT == MVT::i32 && ValVT == MVT::f64) {
            llvm_unreachable("coffee: we don't support i32 to f64");
        }

        InVals.push_back(ArgValue);
      } else { // VA.isRegLoc()

        // sanity check
        assert(VA.isMemLoc());

        // The stack pointer offset is relative to the caller stack frame.
        LastFI = MFI->CreateFixedObject(ValVT.getSizeInBits()/8,
                                        VA.getLocMemOffset(), true);

        // Create load nodes to retrieve arguments from the stack
        SDValue FIN = DAG.getFrameIndex(LastFI, getPointerTy());
        InVals.push_back(DAG.getLoad(ValVT, dl, Chain, FIN,
                                     MachinePointerInfo::getFixedStack(LastFI),
                                     false, false, false, 0));
      }
    }

    // The coffee ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (DAG.getMachineFunction().getFunction()->hasStructRetAttr()) {
        unsigned Reg = CoffeeFI->getSRetReturnReg();
        if (!Reg) {
          Reg = MF.getRegInfo().createVirtualRegister(getRegClassFor(MVT::i32));
          CoffeeFI->setSRetReturnReg(Reg);
        }
        //guoqing: when a function has return value hasStructRetAttr() is true
        SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), dl, Reg, InVals[0]);
        Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other, Copy, Chain);
    }

    if (isVarArg)
        llvm_unreachable("coffee: is var arg");

    CoffeeFI->setLastInArgFI(LastFI);

    // All stores are grouped in one node to allow the matching between
    // the size of Ins and InVals. This only happens when on varg functions
    if (!OutChains.empty()) {
      OutChains.push_back(Chain);
      Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other,
                          &OutChains[0], OutChains.size());
    }

    return Chain;
  }


SDValue  CoffeeTargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI)
  const {
  SelectionDAG &DAG = DCI.DAG;
  unsigned opc = N->getOpcode();

  switch (opc) {
  default: break;
  case ISD::ADDE:
  case ISD::SUBE:
  case ISD::SDIVREM:
  case ISD::UDIVREM:
  case ISD::SELECT:
  case ISD::AND:

  case ISD::OR:

      break;
  }

  return SDValue();
}





SDValue
CoffeeTargetLowering::LowerReturn(SDValue Chain,
            CallingConv::ID CallConv, bool isVarArg,
            const SmallVectorImpl<ISD::OutputArg> &Outs,
            const SmallVectorImpl<SDValue> &OutVals,
            DebugLoc dl, SelectionDAG &DAG) const {

    SmallVector<CCValAssign, 16> RVLocs;

    // CCState - Info about the registers and stack slots.
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                      DAG.getTarget(), RVLocs, *DAG.getContext());

    // Analyze outgoing return values.
    CCInfo.AnalyzeReturn(Outs, RetCC_Coffee);

    // If this is the first return lowered for this function, add
    // the regs to the liveout set for the function.
    if (DAG.getMachineFunction().getRegInfo().liveout_empty()) {
      for (unsigned i = 0; i != RVLocs.size(); ++i)
        if (RVLocs[i].isRegLoc())
          DAG.getMachineFunction().getRegInfo().addLiveOut(RVLocs[i].getLocReg());
    }

    SDValue Flag;

    // Copy the result values into the output registers.
    for (unsigned i = 0; i != RVLocs.size(); ++i) {

      CCValAssign &VA = RVLocs[i];
      assert(VA.isRegLoc() && "Can only return in registers!");

      Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(),
                               OutVals[i], Flag);

      // Guarantee that all emitted copies are stuck together with flags.
      Flag = Chain.getValue(1);
    }


    if (DAG.getMachineFunction().getFunction()->hasStructRetAttr()) {
      MachineFunction &MF      = DAG.getMachineFunction();
      CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();
      unsigned Reg = CoffeeFI->getSRetReturnReg();

      if (!Reg)
        llvm_unreachable("sret virtual register not created in the entry block");

      SDValue Val = DAG.getCopyFromReg(Chain, dl, Reg, getPointerTy());

      Chain = DAG.getCopyToReg(Chain, dl, Coffee::V0, Val, Flag);


      Flag = Chain.getValue(1);
    }

    SDValue result;
    if (Flag.getNode()) {
        result = DAG.getNode(COFFEEISD::RET, dl, MVT::Other, Chain, DAG.getRegister(Coffee::LR, MVT::i32), Flag);

    } else // Return Void
      result = DAG.getNode(COFFEEISD::RET, dl, MVT::Other, Chain, DAG.getRegister(Coffee::LR, MVT::i32));
    return result;
  }

const char * CoffeeTargetLowering::getTargetNodeName(unsigned Opcode) const {
    switch (Opcode) {
    default: return 0;
    case COFFEEISD::RET:       return "COFFEEISD::RET";
    case COFFEEISD::CALL:       return "COFFEEISD::CALL";
    case COFFEEISD::BRCOND:     return   "COFFEEISD::BRCOND";
    case COFFEEISD::CMP:        return "COFFEEISD::CMP";
    case COFFEEISD::Hi:         return "COFFEEISD::Hi";
    case COFFEEISD::Lo:         return "COFFEEISD::Lo";

    }
}

/// LowerMemOpCallTo - Store the argument to the stack.
SDValue
CoffeeTargetLowering::LowerMemOpCallTo(SDValue Chain,
                                    SDValue StackPtr, SDValue Arg,
                                    DebugLoc dl, SelectionDAG &DAG,
                                    const CCValAssign &VA,
                                    ISD::ArgFlagsTy Flags) const {
  unsigned LocMemOffset = VA.getLocMemOffset();
  SDValue PtrOff = DAG.getIntPtrConstant(LocMemOffset);
  PtrOff = DAG.getNode(ISD::ADD, dl, getPointerTy(), StackPtr, PtrOff);
  return DAG.getStore(Chain, dl, Arg, PtrOff,
                      MachinePointerInfo::getStack(LocMemOffset),
                      false, false, 0);
}


SDValue
CoffeeTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                   SDValue Chain, SDValue Arg, DebugLoc DL,
                                   bool IsTailCall, SelectionDAG &DAG) const {
  if (!IsTailCall) {
    SDValue PtrOff = DAG.getNode(ISD::ADD, DL, getPointerTy(), StackPtr,
                                 DAG.getIntPtrConstant(Offset));
    return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo(), false,
                        false, 0);
  }

  llvm_unreachable("coffee:: no tail call support");

}

/// LowerCall - Lowering a call into a callseq_start <-
/// COFFEEISD:CALL <- callseq_end chain. Also add input and output parameter
/// nodes.
SDValue
CoffeeTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                              SmallVectorImpl<SDValue> &InVals) const {
    SelectionDAG &DAG                     = CLI.DAG;
    DebugLoc &dl                          = CLI.DL;
    SmallVector<ISD::OutputArg, 32> &Outs = CLI.Outs;
    SmallVector<SDValue, 32> &OutVals     = CLI.OutVals;
    SmallVector<ISD::InputArg, 32> &Ins   = CLI.Ins;
    SDValue Chain                         = CLI.Chain;
    SDValue Callee                        = CLI.Callee;
    bool &isTailCall                      = CLI.IsTailCall;
    CallingConv::ID CallConv              = CLI.CallConv;
    bool isVarArg                         = CLI.IsVarArg;

    //guoqing: we don't support tail call optimization
    isTailCall = false;

    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    const TargetFrameLowering *TFL = MF.getTarget().getFrameLowering();
    bool IsPIC = getTargetMachine().getRelocationModel() == Reloc::PIC_;

    if (IsPIC)
        llvm_unreachable("coffee: only static reloc for now");

    // Analyze operands of the call, assigning locations to each operand.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   getTargetMachine(), ArgLocs, *DAG.getContext());

    CoffeeCC CoffeeCCInfo(CallConv, isVarArg, CCInfo);

    CoffeeCCInfo.analyzeCallOperands(Outs);

    // Get a count of how many bytes are to be pushed on the stack.
    unsigned NextStackOffset = CCInfo.getNextStackOffset();

    // Check if it's really possible to do a tail call.
   /* if (isTailCall)
      isTailCall =
        IsEligibleForTailCallOptimization(CoffeeCCInfo, NextStackOffset,
                                          *MF.getInfo<CoffeeFunctionInfo>());

    if (isTailCall)
      ++NumTailCalls;*/

    // Chain is the output chain of the last Load/Store or CopyToReg node.
    // ByValChain is the output chain of the last Memcpy node created for copying
    // byval arguments to the stack.
    unsigned StackAlignment = TFL->getStackAlignment();
    NextStackOffset = RoundUpToAlignment(NextStackOffset, StackAlignment);
    SDValue NextStackOffsetVal = DAG.getIntPtrConstant(NextStackOffset, true);

    //if (!isTailCall)
      Chain = DAG.getCALLSEQ_START(Chain, NextStackOffsetVal);

    SDValue StackPtr = DAG.getCopyFromReg(Chain, dl,
                                          Coffee::SP,
                                          getPointerTy());

    // With EABI is it possible to have 16 args on registers.
    SmallVector<std::pair<unsigned, SDValue>, 16> RegsToPass;
    SmallVector<SDValue, 8> MemOpChains;
    CoffeeCC::byval_iterator ByValArg = CoffeeCCInfo.byval_begin();

    // Walk the register/memloc assignments, inserting copies/loads.
    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
      SDValue Arg = OutVals[i];
      CCValAssign &VA = ArgLocs[i];
      MVT ValVT = VA.getValVT(), LocVT = VA.getLocVT();
      ISD::ArgFlagsTy Flags = Outs[i].Flags;

      // ByVal Arg.
      if (Flags.isByVal()) {

          llvm_unreachable("coffee: struct is passed by value");
        /*assert(Flags.getByValSize() &&
               "ByVal args of size 0 should have been ignored by front-end.");
        assert(ByValArg != CoffeeCCInfo.byval_end());
        assert(!isTailCall &&
               "Do not tail-call optimize if there is a byval argument.");
        passByValArg(Chain, dl, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                     CoffeeCCInfo, *ByValArg, Flags, Subtarget->isLittle());
        ++ByValArg;
        continue;*/
      }

      // Promote the value if needed.
      switch (VA.getLocInfo()) {
      default: llvm_unreachable("Unknown loc info!");
      case CCValAssign::Full:
        /*if (VA.isRegLoc()) {
          if ((ValVT == MVT::f32 && LocVT == MVT::i32) ||
              (ValVT == MVT::f64 && LocVT == MVT::i64))
            Arg = DAG.getNode(ISD::BITCAST, dl, LocVT, Arg);
          else if (ValVT == MVT::f64 && LocVT == MVT::i32) {
            SDValue Lo = DAG.getNode(CoffeeISD::ExtractElementF64, dl, MVT::i32,
                                     Arg, DAG.getConstant(0, MVT::i32));
            SDValue Hi = DAG.getNode(CoffeeISD::ExtractElementF64, dl, MVT::i32,
                                     Arg, DAG.getConstant(1, MVT::i32));
            if (!Subtarget->isLittle())
              std::swap(Lo, Hi);
            unsigned LocRegLo = VA.getLocReg();
            unsigned LocRegHigh = getNextIntArgReg(LocRegLo);
            RegsToPass.push_back(std::make_pair(LocRegLo, Lo));
            RegsToPass.push_back(std::make_pair(LocRegHigh, Hi));
            continue;
          }
        }*/
        break;
      case CCValAssign::SExt:
        Arg = DAG.getNode(ISD::SIGN_EXTEND, dl, LocVT, Arg);
        break;
      case CCValAssign::ZExt:
        Arg = DAG.getNode(ISD::ZERO_EXTEND, dl, LocVT, Arg);
        break;
      case CCValAssign::AExt:
        Arg = DAG.getNode(ISD::ANY_EXTEND, dl, LocVT, Arg);
        break;
      }

      // Arguments that can be passed on register must be kept at
      // RegsToPass vector
      if (VA.isRegLoc()) {
        RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
        continue;
      }

      // Register can't get to this point...
      assert(VA.isMemLoc());

      // emit ISD::STORE whichs stores the
      // parameter value to a stack Location
      MemOpChains.push_back(passArgOnStack(StackPtr, VA.getLocMemOffset(),
                                           Chain, Arg, dl, isTailCall, DAG));
    }

    // Transform all store nodes into one single node because all store
    // nodes are independent of each other.
    if (!MemOpChains.empty())
      Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other,
                          &MemOpChains[0], MemOpChains.size());

    // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
    // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
    // node so that legalize doesn't hack it.
    unsigned char OpFlag;
    //bool IsPICCall = (IsN64 || IsPIC); // true if calls are translated to jalr $25
    bool GlobalOrExternal = false;
    SDValue CalleeLo;

    if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
     /* if (IsPICCall && G->getGlobal()->hasInternalLinkage()) {
        OpFlag = IsO32 ? CoffeeII::MO_GOT : CoffeeII::MO_GOT_PAGE;
        unsigned char LoFlag = IsO32 ? CoffeeII::MO_ABS_LO : CoffeeII::MO_GOT_OFST;
        Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl, getPointerTy(), 0,
                                            OpFlag);
        CalleeLo = DAG.getTargetGlobalAddress(G->getGlobal(), dl, getPointerTy(),
                                              0, LoFlag);
      } else {*/
        OpFlag = /*IsPICCall ? CoffeeII::MO_GOT_CALL : */ CoffeeII::MO_NO_FLAG;
        Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl,
                                            getPointerTy(), 0, OpFlag);
     // }

      GlobalOrExternal = true;
    }
    else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    /*  if (IsN64 || (!IsO32 && IsPIC))
        OpFlag = CoffeeII::MO_GOT_DISP;
      else if (!IsPIC) // !N64 && static*/
        OpFlag = CoffeeII::MO_NO_FLAG;
     /* else // O32 & PIC
        OpFlag = CoffeeII::MO_GOT_CALL;*/
      Callee = DAG.getTargetExternalSymbol(S->getSymbol(), getPointerTy(),
                                           OpFlag);
      GlobalOrExternal = true;
    }

    SDValue InFlag;

    // Create nodes that load address of callee and copy it to T9
   /* if (IsPICCall) {
      if (GlobalOrExternal) {
        // Load callee address
        Callee = DAG.getNode(CoffeeISD::Wrapper, dl, getPointerTy(),
                             GetGlobalReg(DAG, getPointerTy()), Callee);
        SDValue LoadValue = DAG.getLoad(getPointerTy(), dl, DAG.getEntryNode(),
                                        Callee, MachinePointerInfo::getGOT(),
                                        false, false, false, 0);

        // Use GOT+LO if callee has internal linkage.
        if (CalleeLo.getNode()) {
          SDValue Lo = DAG.getNode(CoffeeISD::Lo, dl, getPointerTy(), CalleeLo);
          Callee = DAG.getNode(ISD::ADD, dl, getPointerTy(), LoadValue, Lo);
        } else
          Callee = LoadValue;
      }
    }*/

    // T9 register operand.
    SDValue T9;

    // T9 should contain the address of the callee function if
    // -reloction-model=pic or it is an indirect call.
    if (/*IsPICCall ||*/ !GlobalOrExternal) {
      // copy to T9
      unsigned T9Reg = /*IsN64 ? Coffee::T9_64 : */Coffee::T9;
      Chain = DAG.getCopyToReg(Chain, dl, T9Reg, Callee, SDValue(0, 0));
      InFlag = Chain.getValue(1);

      //if (Subtarget->inCoffee16Mode())
      //  T9 = DAG.getRegister(T9Reg, getPointerTy());
      //else
        Callee = DAG.getRegister(T9Reg, getPointerTy());
    }

    // Insert node "GP copy globalreg" before call to function.
    // Lazy-binding stubs require GP to point to the GOT.
    /*if (IsPICCall) {
      unsigned GPReg = IsN64 ? Coffee::GP_64 : Coffee::GP;
      EVT Ty = IsN64 ? MVT::i64 : MVT::i32;
      RegsToPass.push_back(std::make_pair(GPReg, GetGlobalReg(DAG, Ty)));
    }*/

    // Build a sequence of copy-to-reg nodes chained together with token
    // chain and flag operands which copy the outgoing args into registers.
    // The InFlag in necessary since all emitted instructions must be
    // stuck together.
    for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
      Chain = DAG.getCopyToReg(Chain, dl, RegsToPass[i].first,
                               RegsToPass[i].second, InFlag);
      InFlag = Chain.getValue(1);
    }

    // CoffeeJmpLink = #chain, #target_address, #opt_in_flags...
    //             = Chain, Callee, Reg#1, Reg#2, ...
    //
    // Returns a chain & a flag for retval copy to use.
    SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
    SmallVector<SDValue, 8> Ops;
    Ops.push_back(Chain);
    Ops.push_back(Callee);

    // Add argument registers to the end of the list so that they are
    // known live into the call.
    for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
      Ops.push_back(DAG.getRegister(RegsToPass[i].first,
                                    RegsToPass[i].second.getValueType()));

    // Add T9 register operand.
    if (T9.getNode())
      Ops.push_back(T9);

    // Add a register mask operand representing the call-preserved registers.
    const TargetRegisterInfo *TRI = getTargetMachine().getRegisterInfo();
    const uint32_t *Mask = TRI->getCallPreservedMask(CallConv);
    assert(Mask && "Missing call preserved mask for calling convention");
    Ops.push_back(DAG.getRegisterMask(Mask));

    if (InFlag.getNode())
      Ops.push_back(InFlag);

    //if (isTailCall)
     // return DAG.getNode(CoffeeISD::TailCall, dl, MVT::Other, &Ops[0], Ops.size());

    Chain  = DAG.getNode(COFFEEISD::CALL, dl, NodeTys, &Ops[0], Ops.size());
    InFlag = Chain.getValue(1);

    // Create the CALLSEQ_END node.
    Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                               DAG.getIntPtrConstant(0, true), InFlag);
    InFlag = Chain.getValue(1);

    // Handle result values, copying them out of physregs into vregs that we
    // return.
    return LowerCallResult(Chain, InFlag, CallConv, isVarArg,
                           Ins, dl, DAG, InVals);
  }


/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
SDValue
CoffeeTargetLowering::LowerCallResult(SDValue Chain, SDValue InFlag,
                                   CallingConv::ID CallConv, bool isVarArg,
                                   const SmallVectorImpl<ISD::InputArg> &Ins,
                                   DebugLoc dl, SelectionDAG &DAG,
                                   SmallVectorImpl<SDValue> &InVals) const {

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                    DAG.getTarget(), RVLocs, *DAG.getContext());


  CCInfo.AnalyzeCallResult(Ins, RetCC_Coffee);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    Chain = DAG.getCopyFromReg(Chain, dl, RVLocs[i].getLocReg(),
                               RVLocs[i].getValVT(), InFlag).getValue(1);
    InFlag = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}



SDValue CoffeeTargetLowering::LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
  MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
  MFI->setFrameAddressIsTaken(true);

  EVT VT = Op.getValueType();
  DebugLoc dl = Op.getDebugLoc();  // FIXME probably not meaningful
  unsigned Depth = cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue();
  unsigned FrameReg = Coffee::FP;
  SDValue FrameAddr = DAG.getCopyFromReg(DAG.getEntryNode(), dl, FrameReg, VT);
  while (Depth--)
    FrameAddr = DAG.getLoad(VT, dl, DAG.getEntryNode(), FrameAddr,
                            MachinePointerInfo(),
                            false, false, false, 0);
  return FrameAddr;
}

SDValue CoffeeTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {

    //need to recheck this part when we start to implement the float point support
  switch (Op.getOpcode()) {
  default: llvm_unreachable("Don't know how to custom lower this!");
  case ISD::FRAMEADDR:
  case ISD::BRCOND:
  case ISD::ConstantPool:



  case ISD::SELECT:
  case ISD::SETCC:
  case ISD::VASTART:
  case ISD::FCOPYSIGN:
  case ISD::FABS:
  case ISD::MEMBARRIER:
  case ISD::ATOMIC_FENCE:
      return Op;

  case ISD::GlobalTLSAddress: llvm_unreachable("coffee: tls address");
  case ISD::BlockAddress: return LowerBlockAddress(Op, DAG);
  case ISD::JumpTable: return     LowerJumpTable(Op, DAG);
  case ISD::GlobalAddress: return LowerGlobalAddress(Op, DAG);
  case ISD::DYNAMIC_STACKALLOC: return LowerDYNAMIC_STACKALLOC(Op, DAG);
  case ISD::BR_CC: return LowerBR_CC(Op, DAG);
  case ISD::ADD: return LowerADD(Op, DAG);

  }
}

SDValue CoffeeTargetLowering::LowerADD(SDValue Op, SelectionDAG &DAG) const {
  if (Op->getOperand(0).getOpcode() != ISD::FRAMEADDR
      || cast<ConstantSDNode>
        (Op->getOperand(0).getOperand(0))->getZExtValue() != 0
      || Op->getOperand(1).getOpcode() != ISD::FRAME_TO_ARGS_OFFSET)
    return SDValue();

  // The pattern
  //   (add (frameaddr 0), (frame_to_args_offset))
  // results from lowering llvm.eh.dwarf.cfa intrinsic. Transform it to
  //   (add FrameObject, 0)
  // where FrameObject is a fixed StackObject with offset 0 which points to
  // the old stack pointer.
  MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
  EVT ValTy = Op->getValueType(0);
  int FI = MFI->CreateFixedObject(Op.getValueSizeInBits() / 8, 0, false);
  SDValue InArgsAddr = DAG.getFrameIndex(FI, ValTy);
  return DAG.getNode(ISD::ADD, Op->getDebugLoc(), ValTy, InArgsAddr,
                     DAG.getConstant(0, ValTy));
}

SDValue CoffeeTargetLowering::LowerBlockAddress(SDValue Op,
                                              SelectionDAG &DAG) const {
  const BlockAddress *BA = cast<BlockAddressSDNode>(Op)->getBlockAddress();
  // FIXME there isn't actually debug info here
  DebugLoc dl = Op.getDebugLoc();


    // %hi/%lo relocation
    SDValue BAHi = DAG.getBlockAddress(BA, MVT::i32, true, CoffeeII::MO_ABS_HI);
    SDValue BALo = DAG.getBlockAddress(BA, MVT::i32, true, CoffeeII::MO_ABS_LO);
    SDValue Hi = DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, BAHi);

    return DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, BALo, Hi);
}


SDValue CoffeeTargetLowering::
LowerJumpTable(SDValue Op, SelectionDAG &DAG) const
{
  SDValue HiPart, JTI, JTILo;
  // FIXME there isn't actually debug info here
  DebugLoc dl = Op.getDebugLoc();
  bool IsPIC = getTargetMachine().getRelocationModel() == Reloc::PIC_;
  EVT PtrVT = Op.getValueType();
  JumpTableSDNode *JT = cast<JumpTableSDNode>(Op);


    JTI = DAG.getTargetJumpTable(JT->getIndex(), PtrVT, CoffeeII::MO_ABS_HI);
    HiPart = DAG.getNode(COFFEEISD::Hi, dl, PtrVT, JTI);


    JTILo = DAG.getTargetJumpTable(JT->getIndex(), PtrVT, CoffeeII::MO_ABS_LO);

   return DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, HiPart, JTILo);


}

SDValue CoffeeTargetLowering::
LowerDYNAMIC_STACKALLOC(SDValue Op, SelectionDAG &DAG) const
{
  MachineFunction &MF = DAG.getMachineFunction();
  CoffeeFunctionInfo *CoffeeFI = MF.getInfo<CoffeeFunctionInfo>();
  unsigned SP = Coffee::SP;

  assert(getTargetMachine().getFrameLowering()->getStackAlignment() >=
         cast<ConstantSDNode>(Op.getOperand(2).getNode())->getZExtValue() &&
         "Cannot lower if the alignment of the allocated space is larger than \
          that of the stack.");

  SDValue Chain = Op.getOperand(0);
  SDValue Size = Op.getOperand(1);
  DebugLoc dl = Op.getDebugLoc();

  // Get a reference from Coffee stack pointer
  SDValue StackPointer = DAG.getCopyFromReg(Chain, dl, SP, getPointerTy());

  // Subtract the dynamic size from the actual stack size to
  // obtain the new stack size.
  SDValue Sub = DAG.getNode(ISD::SUB, dl, getPointerTy(), StackPointer, Size);

  // The Sub result contains the new stack start address, so it
  // must be placed in the stack pointer register.
  Chain = DAG.getCopyToReg(StackPointer.getValue(1), dl, SP, Sub, SDValue());

  // This node always has two return values: a new stack pointer
  // value and a chain
  SDVTList VTLs = DAG.getVTList(getPointerTy(), MVT::Other);
  SDValue Ptr = DAG.getFrameIndex(CoffeeFI->getDynAllocFI(), getPointerTy());
  SDValue Ops[] = { Chain, Ptr, Chain.getValue(1) };

  return DAG.getNode(COFFEEISD::DynAlloc, dl, VTLs, Ops, 3);
}



SDValue CoffeeTargetLowering::LowerGlobalAddress(SDValue Op,
                                                 SelectionDAG &DAG) const {
    // FIXME there isn't actually debug info here
    DebugLoc dl = Op.getDebugLoc();
    const GlobalValue *GV = cast<GlobalAddressSDNode>(Op)->getGlobal();


   // SDVTList VTs = DAG.getVTList(MVT::i32);

    CoffeeTargetObjectFile &TLOF = (CoffeeTargetObjectFile&)getObjFileLowering();

    //
    if (TLOF.IsGlobalInSmallSection(GV, getTargetMachine())) {
        llvm_unreachable("coffee: gobal in small section");
    }
    // %hi/%lo relocation
    SDValue GAHi = DAG.getTargetGlobalAddress(GV, dl, MVT::i32, 0,
                                              CoffeeII::MO_ABS_HI);
    SDValue GALo = DAG.getTargetGlobalAddress(GV, dl, MVT::i32, 0,
                                              CoffeeII::MO_ABS_LO);
    SDValue HiPart = DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, GAHi);

    return DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, HiPart, GALo);


}


SDValue CoffeeTargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);
  ISD::CondCode CondCode = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue LHS = Op.getOperand(2);
  SDValue RHS = Op.getOperand(3);
  SDValue Dest = Op.getOperand(4);
  DebugLoc dl = Op.getDebugLoc();

  if (LHS.getValueType() == MVT::i32) {
    SDValue CC = DAG.getConstant(CondCode, MVT::i32);
    SDValue cmp = getCoffeeCmp(LHS, RHS, DAG, dl); // condition register

    SDValue glue = cmp.getValue(1);
    SDValue ccreg = cmp.getValue(0);

   // SDValue CCR = DAG.getRegister(Coffee::CR0, MVT::i32);

    return DAG.getNode(COFFEEISD::BRCOND, dl, MVT::Other,
                       Chain, Dest, CC, ccreg, glue);
  }

}


bool CoffeeTargetLowering::isLegalICmpImmediate(int64_t Imm) const {
  // we have 16 bits for immediate
  return Imm >= 0 && Imm <= 65535;
}


SDValue
CoffeeTargetLowering::getCoffeeCmp(SDValue LHS, SDValue RHS,
                             SelectionDAG &DAG,
                             DebugLoc dl) const {
  if (ConstantSDNode *RHSC = dyn_cast<ConstantSDNode>(RHS.getNode())) {
    unsigned C = RHSC->getZExtValue();
    if (!isLegalICmpImmediate(C))
        llvm_unreachable("coffee: cmp imm doesn't fit");
  }

  SDVTList VTLs = DAG.getVTList(MVT::i32, MVT::Glue);

  return DAG.getNode(COFFEEISD::CMP, dl, VTLs, LHS, RHS);
}

//CoffeeCC

CoffeeTargetLowering::CoffeeCC::CoffeeCC(CallingConv::ID CallConv, bool IsVarArg,
                                    CCState &Info) : CCInfo(Info) {
  UseRegsForByval = true;

  if (CallConv != CallingConv::C)
      llvm_unreachable("Coffee: the call convention is not C");

    RegSize = 4;
    NumIntArgRegs = array_lengthof(IntRegs);
    ReservedArgArea = 16;
    IntArgRegs = ShadowRegs = IntRegs;
    FixedFn = VarFn = CC_Coffee;

  // Pre-allocate reserved argument area.
  CCInfo.AllocateStack(ReservedArgArea, 1);
}



void CoffeeTargetLowering::CoffeeCC::
analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Args) {
  unsigned NumOpnds = Args.size();

  for (unsigned I = 0; I != NumOpnds; ++I) {
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;
    bool R;

    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }

    if (Args[I].IsFixed)
      R = FixedFn(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags, CCInfo);
    else
      R = VarFn(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags, CCInfo);

    if (R) {
#ifndef NDEBUG
      dbgs() << "Call operand #" << I << " has unhandled type "
             << EVT(ArgVT).getEVTString();
#endif
      llvm_unreachable(0);
    }
  }
}

void CoffeeTargetLowering::CoffeeCC::
analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Args) {
  unsigned NumArgs = Args.size();

  for (unsigned I = 0; I != NumArgs; ++I) {
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;

    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }

    if (!FixedFn(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags, CCInfo))
      continue;

#ifndef NDEBUG
    dbgs() << "Formal Arg #" << I << " has unhandled type "
           << EVT(ArgVT).getEVTString();
#endif
    llvm_unreachable(0);
  }
}

void
CoffeeTargetLowering::CoffeeCC::handleByValArg(unsigned ValNo, MVT ValVT,
                                           MVT LocVT,
                                           CCValAssign::LocInfo LocInfo,
                                           ISD::ArgFlagsTy ArgFlags) {
  assert(ArgFlags.getByValSize() && "Byval argument's size shouldn't be 0.");

  struct ByValArgInfo ByVal;
  unsigned ByValSize = RoundUpToAlignment(ArgFlags.getByValSize(), RegSize);
  unsigned Align = std::min(std::max(ArgFlags.getByValAlign(), RegSize),
                            RegSize * 2);

  if (UseRegsForByval)
    allocateRegs(ByVal, ByValSize, Align);

  // Allocate space on caller's stack.
  ByVal.Address = CCInfo.AllocateStack(ByValSize - RegSize * ByVal.NumRegs,
                                       Align);
  CCInfo.addLoc(CCValAssign::getMem(ValNo, ValVT, ByVal.Address, LocVT,
                                    LocInfo));
  ByValArgs.push_back(ByVal);
}

void CoffeeTargetLowering::CoffeeCC::allocateRegs(ByValArgInfo &ByVal,
                                              unsigned ByValSize,
                                              unsigned Align) {
  assert(!(ByValSize % RegSize) && !(Align % RegSize) &&
         "Byval argument's size and alignment should be a multiple of"
         "RegSize.");

  ByVal.FirstIdx = CCInfo.getFirstUnallocated(IntArgRegs, NumIntArgRegs);

  // If Align > RegSize, the first arg register must be even.
  if ((Align > RegSize) && (ByVal.FirstIdx % 2)) {
    CCInfo.AllocateReg(IntArgRegs[ByVal.FirstIdx], ShadowRegs[ByVal.FirstIdx]);
    ++ByVal.FirstIdx;
  }

  // Mark the registers allocated.
  for (unsigned I = ByVal.FirstIdx; ByValSize && (I < NumIntArgRegs);
       ByValSize -= RegSize, ++I, ++ByVal.NumRegs)
    CCInfo.AllocateReg(IntArgRegs[I], ShadowRegs[I]);
}



