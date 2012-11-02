//===-- CoffeeHazardRecognizers.h - Coffee Hazard Recognizers -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines hazard recognizers for scheduling on Coffee processors.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeHAZRECS_H
#define CoffeeHAZRECS_H

#include "CoffeeInstrInfo.h"
#include "llvm/CodeGen/ScheduleHazardRecognizer.h"
#include "llvm/CodeGen/ScoreboardHazardRecognizer.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"

namespace llvm {

/// CoffeeScoreboardHazardRecognizer - This class implements a scoreboard-based
/// hazard recognizer for generic Coffee processors.
class CoffeeScoreboardHazardRecognizer : public ScoreboardHazardRecognizer {
  const ScheduleDAG *DAG;
public:
  CoffeeScoreboardHazardRecognizer(const InstrItineraryData *ItinData,
                         const ScheduleDAG *DAG_) :
    ScoreboardHazardRecognizer(ItinData, DAG_), DAG(DAG_) {}

  virtual HazardType getHazardType(SUnit *SU, int Stalls);
  virtual void EmitInstruction(SUnit *SU);
  virtual void AdvanceCycle();
  virtual void Reset();
};

/// CoffeeHazardRecognizer970 - This class defines a finite state automata that
/// models the dispatch logic on the Coffee 970 (aka G5) processor.  This
/// promotes good dispatch group formation and implements noop insertion to
/// avoid structural hazards that cause significant performance penalties (e.g.
/// setting the CTR register then branching through it within a dispatch group),
/// or storing then loading from the same address within a dispatch group.
class CoffeeHazardRecognizer970 : public ScheduleHazardRecognizer {
  const TargetInstrInfo &TII;

  unsigned NumIssued;  // Number of insts issued, including advanced cycles.

  // Various things that can cause a structural hazard.

  // HasCTRSet - If the CTR register is set in this group, disallow BCTRL.
  bool HasCTRSet;

  // StoredPtr - Keep track of the address of any store.  If we see a load from
  // the same address (or one that aliases it), disallow the store.  We can have
  // up to four stores in one dispatch group, hence we track up to 4.
  //
  // This is null if we haven't seen a store yet.  We keep track of both
  // operands of the store here, since we support [r+r] and [r+i] addressing.
  const Value *StoreValue[4];
  int64_t StoreOffset[4];
  uint64_t StoreSize[4];
  unsigned NumStores;

public:
  CoffeeHazardRecognizer970(const TargetInstrInfo &TII);
  virtual HazardType getHazardType(SUnit *SU, int Stalls);
  virtual void EmitInstruction(SUnit *SU);
  virtual void AdvanceCycle();
  virtual void Reset();

private:
  /// EndDispatchGroup - Called when we are finishing a new dispatch group.
  ///
  void EndDispatchGroup();

  /// GetInstrType - Classify the specified Coffee opcode according to its
  /// pipeline.
  /*CoffeeII::Coffee970_Unit GetInstrType(unsigned Opcode,
                                  bool &isFirst, bool &isSingle,bool &isCracked,
                                  bool &isLoad, bool &isStore);*/

  bool isLoadOfStoredAddress(uint64_t LoadSize, int64_t LoadOffset,
                             const Value *LoadValue) const;
};

} // end namespace llvm

#endif

