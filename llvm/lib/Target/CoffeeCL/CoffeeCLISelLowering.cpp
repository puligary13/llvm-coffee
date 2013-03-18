//===-- CoffeeCLISelLowering.cpp - CoffeeCL DAG Lowering Implementation -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the CoffeeCLISelLowering class.
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "coffeecl-lower"
#include "CoffeeCLISelLowering.h"
#include "CoffeeCLMachineFunctionInfo.h"
#include "CoffeeCLPerfectShuffle.h"
#include "CoffeeCLTargetMachine.h"
//#include "MCTargetDesc/CoffeeCLPredicates.h"
#include "CoffeeCLTargetObjectFile.h"
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


static cl::opt<bool> EnableCoffeeCLPreinc("enable-CoffeeCL-preinc",
                                        cl::desc("enable preincrement load/store generation on CoffeeCL (experimental)"),
                                        cl::Hidden);


static const uint16_t IntRegs[4] = {
    CoffeeCL::A0, CoffeeCL::A1, CoffeeCL::A2, CoffeeCL::A3
};


static TargetLoweringObjectFile *CreateTLOF(const CoffeeCLTargetMachine &TM) {
    return new TargetLoweringObjectFileELF();
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// TODO: Implement a generic logic using tblgen that can support this.
// CoffeeCL ABI rules:
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

static bool CC_CoffeeCL(unsigned ValNo, MVT ValVT,
                      MVT LocVT, CCValAssign::LocInfo LocInfo,
                      ISD::ArgFlagsTy ArgFlags, CCState &State) {

    // In coffeecl, float point and integer share the same register bank
    static const unsigned RegsSize=4;

    static const uint16_t Regs[] = {
        CoffeeCL::A0, CoffeeCL::A1, CoffeeCL::A2, CoffeeCL::A3
    };

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

    unsigned OrigAlign = ArgFlags.getOrigAlign();


    if (ValVT == MVT::i32 || ValVT == MVT::f32) {
        Reg = State.AllocateReg(Regs, RegsSize);
        LocVT = MVT::i32;
    } else {
        llvm_unreachable("CoffeeCL: Cannot handle this ValVT.");
    }

    if (!Reg) {
        unsigned Offset = State.AllocateStack(ValVT.getSizeInBits() >> 3,
                                              OrigAlign);
        State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
    } else
        State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));

    return false;
}

#include "CoffeeCLGenCallingConv.inc"

CoffeeCLTargetLowering::CoffeeCLTargetLowering(CoffeeCLTargetMachine &TM)
    : TargetLowering(TM, new TargetLoweringObjectFileELF()) {

    // Set up the register classes.
    setBooleanContents(ZeroOrOneBooleanContent);


    addRegisterClass(MVT::i32, &CoffeeCL::GPRCRegClass);
    addRegisterClass(MVT::f32, &CoffeeCL::FPRCRegClass);


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

#include "CoffeeCLGenRegisterInfo.inc"

SDValue
CoffeeCLTargetLowering::LowerFormalArguments(SDValue Chain,
                                           CallingConv::ID CallConv, bool isVarArg,
                                           const SmallVectorImpl<ISD::InputArg>
                                           &Ins,
                                           DebugLoc dl, SelectionDAG &DAG,
                                           SmallVectorImpl<SDValue> &InVals)
const {
    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeCLFunctionInfo *CoffeeCLFI = MF.getInfo<CoffeeCLFunctionInfo>();

    CoffeeCLFI->setVarArgsFrameIndex(0);

    // Used with vargs to acumulate store chains.
    std::vector<SDValue> OutChains;

    // Assign locations to all of the incoming arguments.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   getTargetMachine(), ArgLocs, *DAG.getContext());
    CoffeeCLCC CoffeeCLCCInfo(CallConv, isVarArg, CCInfo);

    CoffeeCLCCInfo.analyzeFormalArguments(Ins);

    CoffeeCLFI->setFormalArgInfo(CCInfo.getNextStackOffset(),
                               CoffeeCLCCInfo.hasByValArg());

    Function::const_arg_iterator FuncArg =
            DAG.getMachineFunction().getFunction()->arg_begin();
    unsigned CurArgIdx = 0;
    CoffeeCLCC::byval_iterator ByValArg = CoffeeCLCCInfo.byval_begin();

    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
        CCValAssign &VA = ArgLocs[i];
        std::advance(FuncArg, Ins[i].OrigArgIndex - CurArgIdx);
        CurArgIdx = Ins[i].OrigArgIndex;
        EVT ValVT = VA.getValVT();
        ISD::ArgFlagsTy Flags = Ins[i].Flags;
        bool IsRegLoc = VA.isRegLoc();

        if (Flags.isByVal()) {
            assert(Flags.getByValSize() &&
                   "ByVal args of size 0 should have been ignored by front-end.");
            assert(ByValArg != CoffeeCLCCInfo.byval_end());
            copyByValRegs(Chain, dl, OutChains, DAG, Flags, InVals, &*FuncArg,
                          CoffeeCLCCInfo, *ByValArg);
            ++ByValArg;
            continue;
        }

        // Arguments stored on registers
        if (IsRegLoc) {
            EVT RegVT = VA.getLocVT();
            unsigned ArgReg = VA.getLocReg();
            const TargetRegisterClass *RC;

            // In CoffeeCL, integer and float point both use GPR registers
            // CC_CoffeeCL convert f32 to i32 so we should get only i32 here
            if (RegVT == MVT::i32)
                RC = &CoffeeCL::GPRCRegClass;
            else
                llvm_unreachable("CoffeeCL: RegVT not supported by FormalArguments Lowering");

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
            if (RegVT == MVT::i32 && ValVT == MVT::f32) {
                // FP and integer share the same register in COFFEE
                // BITCAST map to nothing here
                ArgValue = DAG.getNode(ISD::BITCAST, dl, ValVT, ArgValue);
            }
            else if (RegVT == MVT::i32 && ValVT == MVT::f64) {
                llvm_unreachable("coffeecl: we don't support i32 to f64");
            }

            InVals.push_back(ArgValue);
        } else { // VA.isRegLoc()

            // sanity check
            assert(VA.isMemLoc());

            // The stack pointer offset is relative to the caller stack frame.
            int FI = MFI->CreateFixedObject(ValVT.getSizeInBits()/8,
                                            VA.getLocMemOffset(), true);

            // Create load nodes to retrieve arguments from the stack
            SDValue FIN = DAG.getFrameIndex(FI, getPointerTy());
            InVals.push_back(DAG.getLoad(ValVT, dl, Chain, FIN,
                                         MachinePointerInfo::getFixedStack(FI),
                                         false, false, false, 0));
        }
    }

    // The coffeecl ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (DAG.getMachineFunction().getFunction()->hasStructRetAttr()) {
        unsigned Reg = CoffeeCLFI->getSRetReturnReg();
        if (!Reg) {
            Reg = MF.getRegInfo().createVirtualRegister(getRegClassFor(MVT::i32));
            CoffeeCLFI->setSRetReturnReg(Reg);
        }
        //guoqing: when a function has return value hasStructRetAttr() is true
        SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), dl, Reg, InVals[0]);
        Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other, Copy, Chain);
    }

    if (isVarArg)
        writeVarArgRegs(OutChains, CoffeeCLCCInfo, Chain, dl, DAG);

    // All stores are grouped in one node to allow the matching between
    // the size of Ins and InVals. This only happens when on varg functions
    if (!OutChains.empty()) {
        OutChains.push_back(Chain);
        Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other,
                            &OutChains[0], OutChains.size());
    }

    return Chain;
}


static SDValue PerformSELECTCombine(SDNode *N, SelectionDAG &DAG,
                                    TargetLowering::DAGCombinerInfo &DCI
                                    /*const CoffeeCLSubtarget *Subtarget*/) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

  SDValue SetCC = N->getOperand(0);

  if ((SetCC.getOpcode() != ISD::SETCC) ||
      !SetCC.getOperand(0).getValueType().isInteger())
    return SDValue();

  SDValue False = N->getOperand(2);
  EVT FalseTy = False.getValueType();

  if (FalseTy.isInteger()) {
      ConstantSDNode *CN = dyn_cast<ConstantSDNode>(False);
      if (!CN || CN->getZExtValue())
          return SDValue();
  } else {
      ConstantFPSDNode *FPCN = dyn_cast<ConstantFPSDNode>(False);
      if (!FPCN)
          return SDValue();
  }

  const DebugLoc DL = N->getDebugLoc();
  ISD::CondCode CC = cast<CondCodeSDNode>(SetCC.getOperand(2))->get();
  SDValue True = N->getOperand(1);

  SetCC = DAG.getSetCC(DL, SetCC.getValueType(), SetCC.getOperand(0),
                       SetCC.getOperand(1), ISD::getSetCCInverse(CC, true));

  return DAG.getNode(ISD::SELECT, DL, FalseTy, SetCC, False, True);
}


SDValue  CoffeeCLTargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI)
const {
    SelectionDAG &DAG = DCI.DAG;
    unsigned opc = N->getOpcode();

    switch (opc) {
    default: break;
    case ISD::ADDE:
    case ISD::SUBE:
    case ISD::SDIVREM:
    case ISD::UDIVREM:

    case ISD::AND:
    case ISD::OR:
        llvm_unreachable("coffeeclisellowering.cpp::performDAGCombine");
        break;
    // TODO: is this needed ?
    // case ISD::SELECT:
        //return PerformSELECTCombine(N, DAG, DCI);
    }

    return SDValue();
}

SDValue
CoffeeCLTargetLowering::LowerReturn(SDValue Chain,
                                  CallingConv::ID CallConv, bool isVarArg,
                                  const SmallVectorImpl<ISD::OutputArg> &Outs,
                                  const SmallVectorImpl<SDValue> &OutVals,
                                  DebugLoc dl, SelectionDAG &DAG) const {

    SmallVector<CCValAssign, 16> RVLocs;

    // CCState - Info about the registers and stack slots.
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   DAG.getTarget(), RVLocs, *DAG.getContext());

    // Analyze outgoing return values.
    CCInfo.AnalyzeReturn(Outs, RetCC_CoffeeCL);

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
        CoffeeCLFunctionInfo *CoffeeCLFI = MF.getInfo<CoffeeCLFunctionInfo>();
        unsigned Reg = CoffeeCLFI->getSRetReturnReg();

        if (!Reg)
            llvm_unreachable("sret virtual register not created in the entry block");

        SDValue Val = DAG.getCopyFromReg(Chain, dl, Reg, getPointerTy());

        Chain = DAG.getCopyToReg(Chain, dl, CoffeeCL::V0, Val, Flag);

        Flag = Chain.getValue(1);
        MF.getRegInfo().addLiveOut(CoffeeCL::V0);
    }


    if (Flag.getNode())
        return DAG.getNode(COFFEEISD::RET, dl, MVT::Other, Chain, Flag);


    return DAG.getNode(COFFEEISD::RET, dl, MVT::Other, Chain);

}

const char * CoffeeCLTargetLowering::getTargetNodeName(unsigned Opcode) const {
    switch (Opcode) {
    default: return 0;
    case COFFEEISD::RET:       return "COFFEEISD::RET";
    case COFFEEISD::JmpLink:       return "COFFEEISD::JmpLink";
    case COFFEEISD::BRCOND:     return   "COFFEEISD::BRCOND";
    case COFFEEISD::CMP:        return "COFFEEISD::CMP";
    case COFFEEISD::FPCMP:        return "COFFEEISD::FPCMP";
    case COFFEEISD::Hi:         return "COFFEEISD::Hi";
    case COFFEEISD::Lo:         return "COFFEEISD::Lo";
    }
}

/// LowerMemOpCallTo - Store the argument to the stack.
SDValue
CoffeeCLTargetLowering::LowerMemOpCallTo(SDValue Chain,
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
CoffeeCLTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                     SDValue Chain, SDValue Arg, DebugLoc DL,
                                     bool IsTailCall, SelectionDAG &DAG) const {
    if (!IsTailCall) {
        SDValue PtrOff = DAG.getNode(ISD::ADD, DL, getPointerTy(), StackPtr,
                                     DAG.getIntPtrConstant(Offset));
        return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo(), false,
                            false, 0);
    }

    llvm_unreachable("coffeecl:: no tail call support");

}

/// LowerCall - Lowering a call into a callseq_start <-
/// COFFEEISD:CALL <- callseq_end chain. Also add input and output parameter
/// nodes.
SDValue
CoffeeCLTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
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
        llvm_unreachable("coffeecl: only static reloc for now");

    // Analyze operands of the call, assigning locations to each operand.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   getTargetMachine(), ArgLocs, *DAG.getContext());

    CoffeeCLCC CoffeeCLCCInfo(CallConv, isVarArg, CCInfo);

    CoffeeCLCCInfo.analyzeCallOperands(Outs);

    // Get a count of how many bytes are to be pushed on the stack.
    unsigned NextStackOffset = CCInfo.getNextStackOffset();

    // Check if it's really possible to do a tail call.
    /* if (isTailCall)
      isTailCall =
        IsEligibleForTailCallOptimization(CoffeeCLCCInfo, NextStackOffset,
                                          *MF.getInfo<CoffeeCLFunctionInfo>());

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
                                          CoffeeCL::SP,
                                          getPointerTy());

    // With EABI is it possible to have 16 args on registers.
    SmallVector<std::pair<unsigned, SDValue>, 16> RegsToPass;
    SmallVector<SDValue, 8> MemOpChains;
    CoffeeCLCC::byval_iterator ByValArg = CoffeeCLCCInfo.byval_begin();

    // Walk the register/memloc assignments, inserting copies/loads.
    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
        SDValue Arg = OutVals[i];
        CCValAssign &VA = ArgLocs[i];
        MVT ValVT = VA.getValVT(), LocVT = VA.getLocVT();
        ISD::ArgFlagsTy Flags = Outs[i].Flags;

        // ByVal Arg.
        if (Flags.isByVal()) {


            assert(Flags.getByValSize() &&
                   "ByVal args of size 0 should have been ignored by front-end.");
            assert(ByValArg != CoffeeCLCCInfo.byval_end());
            assert(!isTailCall &&
                   "Do not tail-call optimize if there is a byval argument.");
            passByValArg(Chain, dl, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                         CoffeeCLCCInfo, *ByValArg, Flags, true); // little endian
            ++ByValArg;
            continue;
        }

        // Promote the value if needed.
        switch (VA.getLocInfo()) {
        default: llvm_unreachable("Unknown loc info!");
        case CCValAssign::Full:
            if (VA.isRegLoc()) {
                if ((ValVT == MVT::f32 && LocVT == MVT::i32)) {
                    // FP and integer share the same register in COFFEE
                    // no need for BITCAST as MIPS
                   Arg = DAG.getNode(ISD::BITCAST, dl, LocVT, Arg);

                } else if (ValVT == MVT::f64 && LocVT == MVT::i32) {
                    llvm_unreachable("coffeecl: full unsupported type");
                }
            }
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
        OpFlag = IsO32 ? CoffeeCLII::MO_GOT : CoffeeCLII::MO_GOT_PAGE;
        unsigned char LoFlag = IsO32 ? CoffeeCLII::MO_ABS_LO : CoffeeCLII::MO_GOT_OFST;
        Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl, getPointerTy(), 0,
                                            OpFlag);
        CalleeLo = DAG.getTargetGlobalAddress(G->getGlobal(), dl, getPointerTy(),
                                              0, LoFlag);
      } else {*/
        OpFlag = /*IsPICCall ? CoffeeCLII::MO_GOT_CALL : */ CoffeeCLII::MO_NO_FLAG;
        Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl,
                                            getPointerTy(), 0, OpFlag);
        // }

        GlobalOrExternal = true;
    }
    else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
        /*  if (IsN64 || (!IsO32 && IsPIC))
        OpFlag = CoffeeCLII::MO_GOT_DISP;
      else if (!IsPIC) // !N64 && static*/
        OpFlag = CoffeeCLII::MO_NO_FLAG;
        /* else // O32 & PIC
        OpFlag = CoffeeCLII::MO_GOT_CALL;*/
        Callee = DAG.getTargetExternalSymbol(S->getSymbol(), getPointerTy(),
                                             OpFlag);
        GlobalOrExternal = true;
    }

    SDValue InFlag;

    // Create nodes that load address of callee and copy it to T9
    /* if (IsPICCall) {
      if (GlobalOrExternal) {
        // Load callee address
        Callee = DAG.getNode(CoffeeCLISD::Wrapper, dl, getPointerTy(),
                             GetGlobalReg(DAG, getPointerTy()), Callee);
        SDValue LoadValue = DAG.getLoad(getPointerTy(), dl, DAG.getEntryNode(),
                                        Callee, MachinePointerInfo::getGOT(),
                                        false, false, false, 0);

        // Use GOT+LO if callee has internal linkage.
        if (CalleeLo.getNode()) {
          SDValue Lo = DAG.getNode(CoffeeCLISD::Lo, dl, getPointerTy(), CalleeLo);
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
        unsigned T9Reg = /*IsN64 ? CoffeeCL::T9_64 : */CoffeeCL::T9;
        Chain = DAG.getCopyToReg(Chain, dl, T9Reg, Callee, SDValue(0, 0));
        InFlag = Chain.getValue(1);

        //if (Subtarget->inCoffeeCL16Mode())
        //  T9 = DAG.getRegister(T9Reg, getPointerTy());
        //else
        Callee = DAG.getRegister(T9Reg, getPointerTy());
    }

    // Insert node "GP copy globalreg" before call to function.
    // Lazy-binding stubs require GP to point to the GOT.
    /*if (IsPICCall) {
      unsigned GPReg = IsN64 ? CoffeeCL::GP_64 : CoffeeCL::GP;
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

    // CoffeeCLJmpLink = #chain, #target_address, #opt_in_flags...
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
    // return DAG.getNode(CoffeeCLISD::TailCall, dl, MVT::Other, &Ops[0], Ops.size());

    Chain  = DAG.getNode(COFFEEISD::JmpLink, dl, NodeTys, &Ops[0], Ops.size());
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
CoffeeCLTargetLowering::LowerCallResult(SDValue Chain, SDValue InFlag,
                                      CallingConv::ID CallConv, bool isVarArg,
                                      const SmallVectorImpl<ISD::InputArg> &Ins,
                                      DebugLoc dl, SelectionDAG &DAG,
                                      SmallVectorImpl<SDValue> &InVals) const {

    // Assign locations to each value returned by this call.
    SmallVector<CCValAssign, 16> RVLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                   DAG.getTarget(), RVLocs, *DAG.getContext());


    CCInfo.AnalyzeCallResult(Ins, RetCC_CoffeeCL);

    // Copy all of the result registers out of their specified physreg.
    for (unsigned i = 0; i != RVLocs.size(); ++i) {
        Chain = DAG.getCopyFromReg(Chain, dl, RVLocs[i].getLocReg(),
                                   RVLocs[i].getValVT(), InFlag).getValue(1);
        InFlag = Chain.getValue(2);
        InVals.push_back(Chain.getValue(0));
    }

    return Chain;
}


SDValue CoffeeCLTargetLowering::LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
    // check the depth
    assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
           "Frame address can only be determined for current frame.");

    MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
    MFI->setFrameAddressIsTaken(true);
    EVT VT = Op.getValueType();
    DebugLoc dl = Op.getDebugLoc();
    SDValue FrameAddr = DAG.getCopyFromReg(DAG.getEntryNode(), dl,
                                           CoffeeCL::FP, VT);
    return FrameAddr;
}

SDValue CoffeeCLTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {

    //need to recheck this part when we start to implement the float point support
    switch (Op.getOpcode()) {
    default: llvm_unreachable("Don't know how to custom lower this!");

    case ISD::GlobalTLSAddress: llvm_unreachable("coffeecl: GlobalTLSAddress");
    case ISD::BlockAddress: return LowerBlockAddress(Op, DAG);
    case ISD::JumpTable: return     LowerJumpTable(Op, DAG);
    case ISD::GlobalAddress: return LowerGlobalAddress(Op, DAG);
    case ISD::BR_CC: return LowerBR_CC(Op, DAG);
    case ISD::ADD: return LowerADD(Op, DAG);
    // TODO: I am not sure, comment out for now
    //case ISD::BITCAST: return LowerBITCAST(Op, DAG);
    case ISD::RETURNADDR: return LowerRETURNADDR(Op, DAG);
    case ISD::ConstantPool: return LowerConstantPool(Op, DAG);
    case ISD::FRAMEADDR: return LowerFRAMEADDR(Op, DAG);

    //case ISD::SETCC: return Op; //LowerSETCC(Op, DAG);
    //case ISD::SELECT: return Op; //LowerSELECT(Op, DAG);
    case ISD::SELECT_CC: return LowerSELECT_CC(Op, DAG);
    case ISD::VASTART: return LowerVASTART(Op, DAG);

    }
}


SDValue CoffeeCLTargetLowering::LowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  CoffeeCLFunctionInfo *FuncInfo = MF.getInfo<CoffeeCLFunctionInfo>();

  DebugLoc dl = Op.getDebugLoc();
  SDValue FI = DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(),
                                 getPointerTy());

  // vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), dl, FI, Op.getOperand(1),
                      MachinePointerInfo(SV), false, false, 0);
}

SDValue CoffeeCLTargetLowering::
LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const {
    EVT VT = Op.getValueType();
    SDValue LHS = Op.getOperand(0);
    SDValue RHS = Op.getOperand(1);
    ISD::CondCode CondCode = cast<CondCodeSDNode>(Op.getOperand(4))->get();
    SDValue TrueVal = Op.getOperand(2);
    SDValue FalseVal = Op.getOperand(3);
    DebugLoc dl = Op.getDebugLoc();

    if (LHS.getValueType() == MVT::i32 || LHS.getValueType() == MVT::f32) {
        SDValue CC = DAG.getConstant(CondCode, MVT::i32);
        SDValue Cmp = getCoffeeCLCmp(LHS, RHS, DAG, dl); // condition register

        SDValue glue = Cmp.getValue(0);
        // We decidedd to use only one CC registier which is CR0
        SDValue ccreg = DAG.getRegister(CoffeeCL::CR0, MVT::i32);

        return DAG.getNode(COFFEEISD::CondMov, dl, VT, FalseVal, TrueVal, CC, ccreg, glue);
    } else {
        llvm_unreachable("coffeecl: LowerSELECT_CC unsupported type");
    }
}


static SDValue CreateConMov(SelectionDAG &DAG, SDValue Cond, SDValue True,
                            SDValue False, DebugLoc DL, SDValue CCReg, SDValue Glue) {

 // return DAG.getNode((invert ? MipsISD::CMovFP_F : MipsISD::CMovFP_T), DL,
 //                    True.getValueType(), True, False, Cond);
}


SDValue CoffeeCLTargetLowering::
LowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
  // Select(COND, TRUEVAL, FALSEVAL)

 /* SDValue Cond = Op.getOperand(0);

  SDValue truevalue = Op.getValue(1);
  SDValue falsevalue = Op.getValue(2);

  return CreateConMov(DAG, Cond, Op.getOperand(1), Op.getOperand(2),
                      Op.getDebugLoc(), ccreg, glue);*/
}

SDValue CoffeeCLTargetLowering::LowerSETCC(SDValue Op, SelectionDAG &DAG) const {
    // set conditional flag
  /*  SDValue LHS = Op.getOperand(0);
    SDValue RHS = Op.getOperand(1);
    DebugLoc dl = Op.getDebugLoc();
    return getCoffeeCLCmp(LHS, RHS, DAG, dl);*/
    //in coffeecl, the CC is set by comparison instruction
    // for both integer and float point, output values are CC and glue
}


SDValue CoffeeCLTargetLowering::
LowerConstantPool(SDValue Op, SelectionDAG &DAG) const
{
    SDValue ResNode;
    ConstantPoolSDNode *N = cast<ConstantPoolSDNode>(Op);
    const Constant *C = N->getConstVal();
    // FIXME there isn't actually debug info here
    DebugLoc dl = Op.getDebugLoc();

    // gp_rel relocation
    // FIXME: we should reference the constant pool using small data sections,
    // but the asm printer currently doesn't support this feature without
    // hacking it. This feature should come soon so we can uncomment the
    // stuff below.
    //if (IsInSmallSection(C->getType())) {
    //  SDValue GPRelNode = DAG.getNode(MipsISD::GPRel, MVT::i32, CP);
    //  SDValue GOT = DAG.getGLOBAL_OFFSET_TABLE(MVT::i32);
    //  ResNode = DAG.getNode(ISD::ADD, MVT::i32, GOT, GPRelNode);

    if (getTargetMachine().getRelocationModel() != Reloc::PIC_) {

        SDValue CPHi = DAG.getTargetConstantPool(C, MVT::i32, N->getAlignment(),
                                                 N->getOffset(), CoffeeCLII::MO_ABS_HI);
        SDValue CPLo = DAG.getTargetConstantPool(C, MVT::i32, N->getAlignment(),
                                                 N->getOffset(), CoffeeCLII::MO_ABS_LO);

        SDValue LoPart = DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, CPLo);
        ResNode = DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, LoPart, CPHi);

    } else {
        llvm_unreachable("coffeecl LowerConstantPoolreloc PIC_");
        /*EVT ValTy = Op.getValueType();
    unsigned GOTFlag = HasMips64 ? MipsII::MO_GOT_PAGE : MipsII::MO_GOT;
    unsigned OFSTFlag = HasMips64 ? MipsII::MO_GOT_OFST : MipsII::MO_ABS_LO;
    SDValue CP = DAG.getTargetConstantPool(C, ValTy, N->getAlignment(),
                                           N->getOffset(), GOTFlag);
    CP = DAG.getNode(MipsISD::Wrapper, dl, ValTy, GetGlobalReg(DAG, ValTy), CP);
    SDValue Load = DAG.getLoad(ValTy, dl, DAG.getEntryNode(), CP,
                               MachinePointerInfo::getConstantPool(), false,
                               false, false, 0);
    SDValue CPLo = DAG.getTargetConstantPool(C, ValTy, N->getAlignment(),
                                             N->getOffset(), OFSTFlag);
    SDValue Lo = DAG.getNode(MipsISD::Lo, dl, ValTy, CPLo);
    ResNode = DAG.getNode(ISD::ADD, dl, ValTy, Load, Lo);*/
    }

    return ResNode;
}

SDValue CoffeeCLTargetLowering::LowerBITCAST(SDValue Op,
                                              SelectionDAG &DAG) const {
    return SDValue();
}


SDValue CoffeeCLTargetLowering::LowerRETURNADDR(SDValue Op,
                                              SelectionDAG &DAG) const {
    // check the depth
    assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
           "Return address can be determined only for current frame.");

    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    EVT VT = Op.getValueType();
    unsigned LR = CoffeeCL::LR;
    MFI->setReturnAddressIsTaken(true);

    // Return LR, which contains the return address. Mark it an implicit live-in.
    unsigned Reg = MF.addLiveIn(LR, getRegClassFor(VT));
    return DAG.getCopyFromReg(DAG.getEntryNode(), Op.getDebugLoc(), Reg, VT);
}


SDValue CoffeeCLTargetLowering::LowerADD(SDValue Op, SelectionDAG &DAG) const {
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
    llvm_unreachable("coffeeclisellowering.cpp:lower ADD");

    MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
    EVT ValTy = Op->getValueType(0);
    int FI = MFI->CreateFixedObject(Op.getValueSizeInBits() / 8, 0, false);
    SDValue InArgsAddr = DAG.getFrameIndex(FI, ValTy);
    return DAG.getNode(ISD::ADD, Op->getDebugLoc(), ValTy, InArgsAddr,
                       DAG.getConstant(0, ValTy));
}

SDValue CoffeeCLTargetLowering::LowerBlockAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
    const BlockAddress *BA = cast<BlockAddressSDNode>(Op)->getBlockAddress();
    // FIXME there isn't actually debug info here
    DebugLoc dl = Op.getDebugLoc();


    // %hi/%lo relocation
    SDValue BAHi = DAG.getBlockAddress(BA, MVT::i32, true, CoffeeCLII::MO_ABS_HI);
    SDValue BALo = DAG.getBlockAddress(BA, MVT::i32, true, CoffeeCLII::MO_ABS_LO);


    SDValue Lo = DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, BALo);

    return DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, BAHi, Lo);
}


SDValue CoffeeCLTargetLowering::
LowerJumpTable(SDValue Op, SelectionDAG &DAG) const
{
    SDValue LoPart, JTIHi, JTILo;
    // FIXME there isn't actually debug info here
    DebugLoc dl = Op.getDebugLoc();
    bool IsPIC = getTargetMachine().getRelocationModel() == Reloc::PIC_;
    EVT PtrVT = Op.getValueType();
    JumpTableSDNode *JT = cast<JumpTableSDNode>(Op);


    JTIHi = DAG.getTargetJumpTable(JT->getIndex(), PtrVT, CoffeeCLII::MO_ABS_HI);
    JTILo = DAG.getTargetJumpTable(JT->getIndex(), PtrVT, CoffeeCLII::MO_ABS_LO);

    LoPart = DAG.getNode(COFFEEISD::Lo, dl, PtrVT, JTILo);

    return DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, LoPart, JTIHi);


}


SDValue CoffeeCLTargetLowering::LowerGlobalAddress(SDValue Op,
                                                 SelectionDAG &DAG) const {
    // FIXME there isn't actually debug info here
    DebugLoc dl = Op.getDebugLoc();
    const GlobalValue *GV = cast<GlobalAddressSDNode>(Op)->getGlobal();


    // SDVTList VTs = DAG.getVTList(MVT::i32);

    CoffeeCLTargetObjectFile &TLOF = (CoffeeCLTargetObjectFile&)getObjFileLowering();

    //
    if (TLOF.IsGlobalInSmallSection(GV, getTargetMachine())) {
        llvm_unreachable("coffeecl: gobal in small section");
    }
    // %hi/%lo relocation
    SDValue GAHi = DAG.getTargetGlobalAddress(GV, dl, MVT::i32, 0,
                                              CoffeeCLII::MO_ABS_HI);
    SDValue GALo = DAG.getTargetGlobalAddress(GV, dl, MVT::i32, 0,
                                              CoffeeCLII::MO_ABS_LO);

    SDValue LoPart = DAG.getNode(COFFEEISD::Lo, dl, MVT::i32, GALo);

    return DAG.getNode(COFFEEISD::Hi, dl, MVT::i32, LoPart, GAHi);


}


SDValue CoffeeCLTargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
    SDValue Chain = Op.getOperand(0);
    ISD::CondCode CondCode = cast<CondCodeSDNode>(Op.getOperand(1))->get();
    SDValue LHS = Op.getOperand(2);
    SDValue RHS = Op.getOperand(3);
    SDValue Dest = Op.getOperand(4);
    DebugLoc dl = Op.getDebugLoc();

    if (LHS.getValueType() == MVT::i32 || LHS.getValueType() == MVT::f32) {
        SDValue CC = DAG.getConstant(CondCode, MVT::i32);
        SDValue cmp = getCoffeeCLCmp(LHS, RHS, DAG, dl); // condition register

        SDValue glue = cmp.getValue(0);
        // We decidedd to use only one CC registier which is CR0
        SDValue ccreg = DAG.getRegister(CoffeeCL::CR0, MVT::i32);

        return DAG.getNode(COFFEEISD::BRCOND, dl, MVT::Other,
                           Chain, Dest, CC, ccreg, glue);
    } else {
        llvm_unreachable("coffeecl: lower BR_CC unsupported type");
    }
}


bool CoffeeCLTargetLowering::isLegalICmpImmediate(int64_t Imm) const {
    // we have 16 bits for immediate
    return Imm >= 0 && Imm <= 65535;
}


SDValue
CoffeeCLTargetLowering::getCoffeeCLCmp(SDValue LHS, SDValue RHS,
                                   SelectionDAG &DAG,
                                   DebugLoc dl) const {
    if (ConstantSDNode *RHSC = dyn_cast<ConstantSDNode>(RHS.getNode())) {

        // The following definition in CoffeeCLInstrInfo.td should make the imm fits

        /* multiclass I_cmp<PatFrag opnode> {
  def ri : CMPI<0b110111, (outs CRRC:$rd ), (ins GPRC:$rt, simm17_cmp:$imm17),
               "cmpi\t$rd\t$rt,\t$imm17", [(set CRRC:$rd, (opnode GPRC:$rt, immSExt17:$imm17))], IIAlu>;

  def riu : CMPI<0b110111, (outs CRRC:$rd ), (ins GPRC:$rt, uimm17_cmp:$imm17),
             "cmpi\t$rd\t$rt,\t$imm17", [(set CRRC:$rd, (opnode GPRC:$rt, immZExt17:$imm17))], IIAlu>;

  def rr : CMPR<0b011001, (outs CRRC:$rd), (ins GPRC:$rt, GPRC:$rs),
               "cmp\t$rd\t$rt,\t$rs", [(set CRRC:$rd, (opnode GPRC:$rt, GPRC:$rs))], IIAlu>;
}*/

        /*unsigned C = RHSC->getZExtValue();
    if (!isLegalICmpImmediate(C))
        llvm_unreachable("coffeecl: cmp imm doesn't fit");*/
    }

    SDVTList VTLs;
    unsigned Opcode;
    if (LHS.getValueType() == MVT::i32) {
        VTLs = DAG.getVTList(MVT::Glue); //glue
        Opcode = COFFEEISD::CMP;
    } else if (LHS.getValueType() == MVT::f32) {
        VTLs = DAG.getVTList(MVT::Glue); // glue
        Opcode = COFFEEISD::FPCMP;
    } else {
        llvm_unreachable("CoffeeCL: CoffeeCL cmp unsupported type");
    }

    return DAG.getNode(Opcode, dl, VTLs, LHS, RHS);
}

//CoffeeCLCC

CoffeeCLTargetLowering::CoffeeCLCC::CoffeeCLCC(CallingConv::ID CallConv, bool IsVarArg,
                                         CCState &Info) : CCInfo(Info) {
    UseRegsForByval = true;

    if (CallConv != CallingConv::C && CallConv != CallingConv::Fast && CallConv != CallingConv::COFFEECL_Device && CallConv != CallingConv::COFFEECL_Kernel)
        llvm_unreachable("CoffeeCL: the call convention is not C or COFFEECL device");

    RegSize = 4;
    NumIntArgRegs = array_lengthof(IntRegs);
    ReservedArgArea = 16;
    IntArgRegs = ShadowRegs = IntRegs;
    FixedFn = VarFn = CC_CoffeeCL;

    // Pre-allocate reserved argument area.
    CCInfo.AllocateStack(ReservedArgArea, 1);
}



void CoffeeCLTargetLowering::CoffeeCLCC::
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

void CoffeeCLTargetLowering::CoffeeCLCC::
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
CoffeeCLTargetLowering::CoffeeCLCC::handleByValArg(unsigned ValNo, MVT ValVT,
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

void CoffeeCLTargetLowering::CoffeeCLCC::allocateRegs(ByValArgInfo &ByVal,
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

void CoffeeCLTargetLowering::
copyByValRegs(SDValue Chain, DebugLoc DL, std::vector<SDValue> &OutChains,
              SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
              SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
              const CoffeeCLCC &CC, const ByValArgInfo &ByVal) const {
    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    unsigned RegAreaSize = ByVal.NumRegs * CC.regSize();
    unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
    int FrameObjOffset;

    if (RegAreaSize)
        FrameObjOffset = (int)CC.reservedArgArea() -
                (int)((CC.numIntArgRegs() - ByVal.FirstIdx) * CC.regSize());
    else
        FrameObjOffset = ByVal.Address;

    // Create frame object.
    EVT PtrTy = getPointerTy();
    int FI = MFI->CreateFixedObject(FrameObjSize, FrameObjOffset, true);
    SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
    InVals.push_back(FIN);

    if (!ByVal.NumRegs)
        return;

    // Copy arg registers.
    EVT RegTy = MVT::getIntegerVT(CC.regSize() * 8);
    const TargetRegisterClass *RC = getRegClassFor(RegTy);

    for (unsigned I = 0; I < ByVal.NumRegs; ++I) {
        unsigned ArgReg = CC.intArgRegs()[ByVal.FirstIdx + I];
        unsigned VReg = AddLiveIn(MF, ArgReg, RC);
        unsigned Offset = I * CC.regSize();
        SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                       DAG.getConstant(Offset, PtrTy));
        SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                     StorePtr, MachinePointerInfo(FuncArg, Offset),
                                     false, false, 0);
        OutChains.push_back(Store);
    }
}

// Copy byVal arg to registers and stack.
void CoffeeCLTargetLowering::
passByValArg(SDValue Chain, DebugLoc DL,
             SmallVector<std::pair<unsigned, SDValue>, 16> &RegsToPass,
             SmallVector<SDValue, 8> &MemOpChains, SDValue StackPtr,
             MachineFrameInfo *MFI, SelectionDAG &DAG, SDValue Arg,
             const CoffeeCLCC &CC, const ByValArgInfo &ByVal,
             const ISD::ArgFlagsTy &Flags, bool isLittle) const {
    unsigned ByValSize = Flags.getByValSize();
    unsigned Offset = 0; // Offset in # of bytes from the beginning of struct.
    unsigned RegSize = CC.regSize();
    unsigned Alignment = std::min(Flags.getByValAlign(), RegSize);
    EVT PtrTy = getPointerTy(), RegTy = MVT::getIntegerVT(RegSize * 8);

    if (ByVal.NumRegs) {
        const uint16_t *ArgRegs = CC.intArgRegs();
        bool LeftoverBytes = (ByVal.NumRegs * RegSize > ByValSize);
        unsigned I = 0;

        // Copy words to registers.
        for (; I < ByVal.NumRegs - LeftoverBytes; ++I, Offset += RegSize) {
            SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                          DAG.getConstant(Offset, PtrTy));
            SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                          MachinePointerInfo(), false, false, false,
                                          Alignment);
            MemOpChains.push_back(LoadVal.getValue(1));
            unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
            RegsToPass.push_back(std::make_pair(ArgReg, LoadVal));
        }

        // Return if the struct has been fully copied.
        if (ByValSize == Offset)
            return;

        // Copy the remainder of the byval argument with sub-word loads and shifts.
        if (LeftoverBytes) {
            assert((ByValSize > Offset) && (ByValSize < Offset + RegSize) &&
                   "Size of the remainder should be smaller than RegSize.");
            SDValue Val;

            for (unsigned LoadSize = RegSize / 2, TotalSizeLoaded = 0;
                 Offset < ByValSize; LoadSize /= 2) {
                unsigned RemSize = ByValSize - Offset;

                if (RemSize < LoadSize)
                    continue;

                // Load subword.
                SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                              DAG.getConstant(Offset, PtrTy));
                SDValue LoadVal =
                        DAG.getExtLoad(ISD::ZEXTLOAD, DL, RegTy, Chain, LoadPtr,
                                       MachinePointerInfo(), MVT::getIntegerVT(LoadSize * 8),
                                       false, false, Alignment);
                MemOpChains.push_back(LoadVal.getValue(1));

                // Shift the loaded value.
                unsigned Shamt;

                if (isLittle)
                    Shamt = TotalSizeLoaded;
                else
                    Shamt = (RegSize - (TotalSizeLoaded + LoadSize)) * 8;

                SDValue Shift = DAG.getNode(ISD::SHL, DL, RegTy, LoadVal,
                                            DAG.getConstant(Shamt, MVT::i32));

                if (Val.getNode())
                    Val = DAG.getNode(ISD::OR, DL, RegTy, Val, Shift);
                else
                    Val = Shift;

                Offset += LoadSize;
                TotalSizeLoaded += LoadSize;
                Alignment = std::min(Alignment, LoadSize);
            }

            unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
            RegsToPass.push_back(std::make_pair(ArgReg, Val));
            return;
        }
    }

    // Copy remainder of byval arg to it with memcpy.
    unsigned MemCpySize = ByValSize - Offset;
    SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                              DAG.getConstant(Offset, PtrTy));
    SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                              DAG.getIntPtrConstant(ByVal.Address));
    Chain = DAG.getMemcpy(Chain, DL, Dst, Src,
                          DAG.getConstant(MemCpySize, PtrTy), Alignment,
                          /*isVolatile=*/false, /*AlwaysInline=*/false,
                          MachinePointerInfo(0), MachinePointerInfo(0));
    MemOpChains.push_back(Chain);
}

void
CoffeeCLTargetLowering::writeVarArgRegs(std::vector<SDValue> &OutChains,
                                      const CoffeeCLCC &CC, SDValue Chain,
                                      DebugLoc DL, SelectionDAG &DAG) const {
    unsigned NumRegs = CC.numIntArgRegs();
    const uint16_t *ArgRegs = CC.intArgRegs();
    const CCState &CCInfo = CC.getCCInfo();
    unsigned Idx = CCInfo.getFirstUnallocated(ArgRegs, NumRegs);
    unsigned RegSize = CC.regSize();
    EVT RegTy = MVT::getIntegerVT(RegSize * 8);
    const TargetRegisterClass *RC = getRegClassFor(RegTy);
    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo *MFI = MF.getFrameInfo();
    CoffeeCLFunctionInfo *CoffeeCLFI = MF.getInfo<CoffeeCLFunctionInfo>();

    // Offset of the first variable argument from stack pointer.
    int VaArgOffset;

    if (NumRegs == Idx)
        VaArgOffset = RoundUpToAlignment(CCInfo.getNextStackOffset(), RegSize);
    else
        VaArgOffset =
                (int)CC.reservedArgArea() - (int)(RegSize * (NumRegs - Idx));

    // Record the frame index of the first variable argument
    // which is a value necessary to VASTART.
    int FI = MFI->CreateFixedObject(RegSize, VaArgOffset, true);
    CoffeeCLFI->setVarArgsFrameIndex(FI);

    // Copy the integer registers that have not been used for argument passing
    // to the argument register save area. For O32, the save area is allocated
    // in the caller's stack frame, while for N32/64, it is allocated in the
    // callee's stack frame.
    for (unsigned I = Idx; I < NumRegs; ++I, VaArgOffset += RegSize) {
        unsigned Reg = AddLiveIn(MF, ArgRegs[I], RC);
        SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegTy);
        FI = MFI->CreateFixedObject(RegSize, VaArgOffset, true);
        SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy());
        SDValue Store = DAG.getStore(Chain, DL, ArgValue, PtrOff,
                                     MachinePointerInfo(), false, false, 0);
        cast<StoreSDNode>(Store.getNode())->getMemOperand()->setValue(0);
        OutChains.push_back(Store);
    }
}


