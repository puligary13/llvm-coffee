//===-- CoffeeHazardRecognizers.cpp - Coffee Hazard Recognizer Impls --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements hazard recognizers for scheduling on Coffee processors.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "pre-RA-sched"
#include "CoffeeHazardRecognizers.h"
#include "Coffee.h"
#include "CoffeeInstrInfo.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

//===----------------------------------------------------------------------===//
// Coffee Scoreboard Hazard Recognizer
void CoffeeScoreboardHazardRecognizer::EmitInstruction(SUnit *SU) {
  const MCInstrDesc *MCID = DAG->getInstrDesc(SU);
  if (!MCID)
    // This is a Coffee pseudo-instruction.
    return;

  ScoreboardHazardRecognizer::EmitInstruction(SU);
}

ScheduleHazardRecognizer::HazardType
CoffeeScoreboardHazardRecognizer::getHazardType(SUnit *SU, int Stalls) {
  return ScoreboardHazardRecognizer::getHazardType(SU, Stalls);
}

void CoffeeScoreboardHazardRecognizer::AdvanceCycle() {
  ScoreboardHazardRecognizer::AdvanceCycle();
}

void CoffeeScoreboardHazardRecognizer::Reset() {
  ScoreboardHazardRecognizer::Reset();
}

//===----------------------------------------------------------------------===//
// Coffee 970 Hazard Recognizer
//
// This models the dispatch group formation of the Coffee970 processor.  Dispatch
// groups are bundles of up to five instructions that can contain various mixes
// of instructions.  The Coffee970 can dispatch a peak of 4 non-branch and one
// branch instruction per-cycle.
//
// There are a number of restrictions to dispatch group formation: some
// instructions can only be issued in the first slot of a dispatch group, & some
// instructions fill an entire dispatch group.  Additionally, only branches can
// issue in the 5th (last) slot.
//
// Finally, there are a number of "structural" hazards on the Coffee970.  These
// conditions cause large performance penalties due to misprediction, recovery,
// and replay logic that has to happen.  These cases include setting a CTR and
// branching through it in the same dispatch group, and storing to an address,
// then loading from the same address within a dispatch group.  To avoid these
// conditions, we insert no-op instructions when appropriate.
//
// FIXME: This is missing some significant cases:
//   1. Modeling of microcoded instructions.
//   2. Handling of serialized operations.
//   3. Handling of the esoteric cases in "Resource-based Instruction Grouping".
//

CoffeeHazardRecognizer970::CoffeeHazardRecognizer970(const TargetInstrInfo &tii)
  : TII(tii) {
  EndDispatchGroup();
}

void CoffeeHazardRecognizer970::EndDispatchGroup() {
  DEBUG(errs() << "=== Start of dispatch group\n");
  NumIssued = 0;

  // Structural hazard info.
  HasCTRSet = false;
  NumStores = 0;
}


/*CoffeeII::Coffee970_Unit
CoffeeHazardRecognizer970::GetInstrType(unsigned Opcode,
                                     bool &isFirst, bool &isSingle,
                                     bool &isCracked,
                                     bool &isLoad, bool &isStore) {
  const MCInstrDesc &MCID = TII.get(Opcode);

  isLoad  = MCID.mayLoad();
  isStore = MCID.mayStore();

  uint64_t TSFlags = MCID.TSFlags;

  isFirst   = TSFlags & CoffeeII::Coffee970_First;
  isSingle  = TSFlags & CoffeeII::Coffee970_Single;
  isCracked = TSFlags & CoffeeII::Coffee970_Cracked;
  return (CoffeeII::Coffee970_Unit)(TSFlags & CoffeeII::Coffee970_Mask);
}*/

/// isLoadOfStoredAddress - If we have a load from the previously stored pointer
/// as indicated by StorePtr1/StorePtr2/StoreSize, return true.
bool CoffeeHazardRecognizer970::
isLoadOfStoredAddress(uint64_t LoadSize, int64_t LoadOffset,
  const Value *LoadValue) const {
  for (unsigned i = 0, e = NumStores; i != e; ++i) {
    // Handle exact and commuted addresses.
    if (LoadValue == StoreValue[i] && LoadOffset == StoreOffset[i])
      return true;

    // Okay, we don't have an exact match, if this is an indexed offset, see if
    // we have overlap (which happens during fp->int conversion for example).
    if (StoreValue[i] == LoadValue) {
      // Okay the base pointers match, so we have [c1+r] vs [c2+r].  Check
      // to see if the load and store actually overlap.
      if (StoreOffset[i] < LoadOffset) {
        if (int64_t(StoreOffset[i]+StoreSize[i]) > LoadOffset) return true;
      } else {
        if (int64_t(LoadOffset+LoadSize) > StoreOffset[i]) return true;
      }
    }
  }
  return false;
}

/// getHazardType - We return hazard for any non-branch instruction that would
/// terminate the dispatch group.  We turn NoopHazard for any
/// instructions that wouldn't terminate the dispatch group that would cause a
/// pipeline flush.
ScheduleHazardRecognizer::HazardType CoffeeHazardRecognizer970::
getHazardType(SUnit *SU, int Stalls) {
  assert(Stalls == 0 && "Coffee hazards don't support scoreboard lookahead");

  MachineInstr *MI = SU->getInstr();

  if (MI->isDebugValue())
    return NoHazard;

  unsigned Opcode = MI->getOpcode();
  bool isFirst, isSingle, isCracked, isLoad, isStore;
  /*CoffeeII::Coffee970_Unit InstrType =
    GetInstrType(Opcode, isFirst, isSingle, isCracked,
                 isLoad, isStore);*/

  //if (InstrType == CoffeeII::Coffee970_Pseudo) return NoHazard;

  // We can only issue a Coffee970_First/Coffee970_Single instruction (such as
  // crand/mtspr/etc) if this is the first cycle of the dispatch group.
  if (NumIssued != 0 && (isFirst || isSingle))
    return Hazard;

  // If this instruction is cracked into two ops by the decoder, we know that
  // it is not a branch and that it cannot issue if 3 other instructions are
  // already in the dispatch group.
  if (isCracked && NumIssued > 2)
    return Hazard;

  /*switch (InstrType) {
  default: llvm_unreachable("Unknown instruction type!");
  case CoffeeII::Coffee970_FXU:
  case CoffeeII::Coffee970_LSU:
  case CoffeeII::Coffee970_FPU:
  case CoffeeII::Coffee970_VALU:
  case CoffeeII::Coffee970_VPERM:
    // We can only issue a branch as the last instruction in a group.
    if (NumIssued == 4) return Hazard;
    break;
  case CoffeeII::Coffee970_CRU:
    // We can only issue a CR instruction in the first two slots.
    if (NumIssued >= 2) return Hazard;
    break;
  case CoffeeII::Coffee970_BRU:
    break;
    }*/

  // Do not allow MTCTR and BCTRL to be in the same dispatch group.
  /*if (HasCTRSet && (Opcode == Coffee::BCTRL_Darwin || Opcode == Coffee::BCTRL_SVR4))
    return NoopHazard;*/

  // If this is a load following a store, make sure it's not to the same or
  // overlapping address.
  if (isLoad && NumStores && !MI->memoperands_empty()) {
    MachineMemOperand *MO = *MI->memoperands_begin();
    if (isLoadOfStoredAddress(MO->getSize(),
                              MO->getOffset(), MO->getValue()))
      return NoopHazard;
  }

  return NoHazard;
}

void CoffeeHazardRecognizer970::EmitInstruction(SUnit *SU) {
  MachineInstr *MI = SU->getInstr();

  if (MI->isDebugValue())
    return;

  unsigned Opcode = MI->getOpcode();
  bool isFirst, isSingle, isCracked, isLoad, isStore;
 /* CoffeeII::Coffee970_Unit InstrType =
    GetInstrType(Opcode, isFirst, isSingle, isCracked,
                 isLoad, isStore);
  if (InstrType == CoffeeII::Coffee970_Pseudo) return;*/

  // Update structural hazard information.
  //if (Opcode == Coffee::MTCTR || Opcode == Coffee::MTCTR8) HasCTRSet = true;

  // Track the address stored to.
  if (isStore && NumStores < 4 && !MI->memoperands_empty()) {
    MachineMemOperand *MO = *MI->memoperands_begin();
    StoreSize[NumStores] = MO->getSize();
    StoreOffset[NumStores] = MO->getOffset();
    StoreValue[NumStores] = MO->getValue();
    ++NumStores;
  }

  /*if (InstrType == CoffeeII::Coffee970_BRU || isSingle)
    NumIssued = 4;  */// Terminate a d-group.
  ++NumIssued;

  // If this instruction is cracked into two ops by the decoder, remember that
  // we issued two pieces.
  if (isCracked)
    ++NumIssued;

  if (NumIssued == 5)
    EndDispatchGroup();
}

void CoffeeHazardRecognizer970::AdvanceCycle() {
  assert(NumIssued < 5 && "Illegal dispatch group!");
  ++NumIssued;
  if (NumIssued == 5)
    EndDispatchGroup();
}

void CoffeeHazardRecognizer970::Reset() {
  EndDispatchGroup();
}

