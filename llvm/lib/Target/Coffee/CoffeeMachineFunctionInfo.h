//===-- CoffeeMachineFunctionInfo.h - Private data used for Coffee --*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Coffee specific subclass of MachineFunctionInfo.
//
//===----------------------------------------------------------------------===//

#ifndef Coffee_MACHINE_FUNCTION_INFO_H
#define Coffee_MACHINE_FUNCTION_INFO_H

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"

namespace llvm {

class CoffeeFunctionInfo : public MachineFunctionInfo {
    virtual void anchor();

    /// VarArgsRegSaveSize - Size of the register save area for vararg functions.
    ///
    unsigned VarArgsRegSaveSize;

    /// HasStackFrame - True if this function has a stack frame. Set by
    /// processFunctionBeforeCalleeSavedScan().
    bool HasStackFrame;

    /// RestoreSPFromFP - True if epilogue should restore SP from FP. Set by
    /// emitPrologue.
    bool RestoreSPFromFP;

    /// LRSpilledForFarJump - True if the LR register has been for spilled to
    /// enable far jump.
    bool LRSpilledForFarJump;

    /// FramePtrSpillOffset - If HasStackFrame, this records the frame pointer
    /// spill stack offset.
    unsigned FramePtrSpillOffset;

    /// GPRCS1Offset, GPRCS2Offset, DPRCSOffset - Starting offset of callee saved
    /// register spills areas. For Mac OS X:
    ///
    /// GPR callee-saved (1) : r4, r5, r6, r7, lr
    /// --------------------------------------------
    /// GPR callee-saved (2) : r8, r10, r11
    /// --------------------------------------------
    /// DPR callee-saved : d8 - d15
    ///
    /// Also see AlignedDPRCSRegs below. Not all D-regs need to go in area 3.
    /// Some may be spilled after the stack has been realigned.
    unsigned GPRCS1Offset;
    unsigned GPRCS2Offset;
    unsigned DPRCSOffset;

    /// GPRCS1Size, GPRCS2Size, DPRCSSize - Sizes of callee saved register spills
    /// areas.
    unsigned GPRCS1Size;
    unsigned GPRCS2Size;
    unsigned DPRCSSize;

    /// GPRCS1Frames, GPRCS2Frames, DPRCSFrames - Keeps track of frame indices
    /// which belong to these spill areas.
    BitVector GPRCS1Frames;
    BitVector GPRCS2Frames;
    BitVector DPRCSFrames;

    /// NumAlignedDPRCS2Regs - The number of callee-saved DPRs that are saved in
    /// the aligned portion of the stack frame.  This is always a contiguous
    /// sequence of D-registers starting from d8.
    ///
    /// We do not keep track of the frame indices used for these registers - they
    /// behave like any other frame index in the aligned stack frame.  These
    /// registers also aren't included in DPRCSSize above.
    unsigned NumAlignedDPRCS2Regs;

    /// JumpTableUId - Unique id for jumptables.
    ///
    unsigned JumpTableUId;

    unsigned PICLabelUId;

    /// VarArgsFrameIndex - FrameIndex for start of varargs area.
    int VarArgsFrameIndex;

    /// HasITBlocks - True if IT blocks have been inserted.
    bool HasITBlocks;

    /// CPEClones - Track constant pool entries clones created by Constant Island
    /// pass.
    DenseMap<unsigned, unsigned> CPEClones;

    // Range of frame object indices.
    // InArgFIRange: Range of indices of all frame objects created during call to
    //               LowerFormalArguments.
    // OutArgFIRange: Range of indices of all frame objects created during call to
    //                LowerCall except for the frame object for restoring $gp.
    std::pair<int, int> InArgFIRange, OutArgFIRange;


    mutable int DynAllocFI;
    unsigned MaxCallFrameSize;
    MachineFunction& MF;
    unsigned SRetReturnReg;
  public:
    CoffeeFunctionInfo(MachineFunction& MF) :
      MF(MF),
      VarArgsRegSaveSize(0), HasStackFrame(false), RestoreSPFromFP(false),
      LRSpilledForFarJump(false),
      FramePtrSpillOffset(0), GPRCS1Offset(0), GPRCS2Offset(0), DPRCSOffset(0),
      GPRCS1Size(0), GPRCS2Size(0), DPRCSSize(0),
      GPRCS1Frames(0), GPRCS2Frames(0), DPRCSFrames(0),
      NumAlignedDPRCS2Regs(0),
      JumpTableUId(0), PICLabelUId(0),
      VarArgsFrameIndex(0), HasITBlocks(false), InArgFIRange(std::make_pair(-1, 0)),
        OutArgFIRange(std::make_pair(-1, 0)), DynAllocFI(0), MaxCallFrameSize(0), SRetReturnReg(0) {}

   /* explicit CoffeeFunctionInfo(MachineFunction &MF) :
         MF(MF),
      VarArgsRegSaveSize(0), HasStackFrame(false), RestoreSPFromFP(false),
      LRSpilledForFarJump(false),
      FramePtrSpillOffset(0), GPRCS1Offset(0), GPRCS2Offset(0), DPRCSOffset(0),
      GPRCS1Size(0), GPRCS2Size(0), DPRCSSize(0),
      GPRCS1Frames(32), GPRCS2Frames(32), DPRCSFrames(32),
      JumpTableUId(0), PICLabelUId(0),
      VarArgsFrameIndex(0), HasITBlocks(false), InArgFIRange(std::make_pair(-1, 0)),
        OutArgFIRange(std::make_pair(-1, 0)), DynAllocFI(0), MaxCallFrameSize(0) {}*/



    bool isInArgFI(int FI) const {
      return FI <= InArgFIRange.first && FI >= InArgFIRange.second;
    }
    void setLastInArgFI(int FI) { InArgFIRange.second = FI; }

    bool isOutArgFI(int FI) const {
      return FI <= OutArgFIRange.first && FI >= OutArgFIRange.second;
    }
    void extendOutArgFIRange(int FirstFI, int LastFI) {
      if (!OutArgFIRange.second)
        // this must be the first time this function was called.
        OutArgFIRange.first = FirstFI;
      OutArgFIRange.second = LastFI;
    }


    // The first call to this function creates a frame object for dynamically
    // allocated stack area.
    int getDynAllocFI() const {
      if (!DynAllocFI)
        DynAllocFI = MF.getFrameInfo()->CreateFixedObject(4, 0, true);

      return DynAllocFI;
    }

    bool isDynAllocFI(int FI) const { return DynAllocFI && DynAllocFI == FI; }


    unsigned getVarArgsRegSaveSize() const { return VarArgsRegSaveSize; }
    void setVarArgsRegSaveSize(unsigned s) { VarArgsRegSaveSize = s; }

    bool hasStackFrame() const { return HasStackFrame; }
    void setHasStackFrame(bool s) { HasStackFrame = s; }

    bool shouldRestoreSPFromFP() const { return RestoreSPFromFP; }
    void setShouldRestoreSPFromFP(bool s) { RestoreSPFromFP = s; }

    bool isLRSpilledForFarJump() const { return LRSpilledForFarJump; }
    void setLRIsSpilledForFarJump(bool s) { LRSpilledForFarJump = s; }

    unsigned getFramePtrSpillOffset() const { return FramePtrSpillOffset; }
    void setFramePtrSpillOffset(unsigned o) { FramePtrSpillOffset = o; }

    unsigned getNumAlignedDPRCS2Regs() const { return NumAlignedDPRCS2Regs; }
    void setNumAlignedDPRCS2Regs(unsigned n) { NumAlignedDPRCS2Regs = n; }

    unsigned getGPRCalleeSavedArea1Offset() const { return GPRCS1Offset; }
    unsigned getGPRCalleeSavedArea2Offset() const { return GPRCS2Offset; }
    unsigned getDPRCalleeSavedAreaOffset()  const { return DPRCSOffset; }

    void setGPRCalleeSavedArea1Offset(unsigned o) { GPRCS1Offset = o; }
    void setGPRCalleeSavedArea2Offset(unsigned o) { GPRCS2Offset = o; }
    void setDPRCalleeSavedAreaOffset(unsigned o)  { DPRCSOffset = o; }

    unsigned getGPRCalleeSavedArea1Size() const { return GPRCS1Size; }
    unsigned getGPRCalleeSavedArea2Size() const { return GPRCS2Size; }
    unsigned getDPRCalleeSavedAreaSize()  const { return DPRCSSize; }

    void setGPRCalleeSavedArea1Size(unsigned s) { GPRCS1Size = s; }
    void setGPRCalleeSavedArea2Size(unsigned s) { GPRCS2Size = s; }
    void setDPRCalleeSavedAreaSize(unsigned s)  { DPRCSSize = s; }

    bool isGPRCalleeSavedArea1Frame(int fi) const {
      if (fi < 0 || fi >= (int)GPRCS1Frames.size())
        return false;
      return GPRCS1Frames[fi];
    }
    bool isGPRCalleeSavedArea2Frame(int fi) const {
      if (fi < 0 || fi >= (int)GPRCS2Frames.size())
        return false;
      return GPRCS2Frames[fi];
    }
    bool isDPRCalleeSavedAreaFrame(int fi) const {
      if (fi < 0 || fi >= (int)DPRCSFrames.size())
        return false;
      return DPRCSFrames[fi];
    }

    void addGPRCalleeSavedArea1Frame(int fi) {
      if (fi >= 0) {
        int Size = GPRCS1Frames.size();
        if (fi >= Size) {
          Size *= 2;
          if (fi >= Size)
            Size = fi+1;
          GPRCS1Frames.resize(Size);
        }
        GPRCS1Frames[fi] = true;
      }
    }
    void addGPRCalleeSavedArea2Frame(int fi) {
      if (fi >= 0) {
        int Size = GPRCS2Frames.size();
        if (fi >= Size) {
          Size *= 2;
          if (fi >= Size)
            Size = fi+1;
          GPRCS2Frames.resize(Size);
        }
        GPRCS2Frames[fi] = true;
      }
    }
    void addDPRCalleeSavedAreaFrame(int fi) {
      if (fi >= 0) {
        int Size = DPRCSFrames.size();
        if (fi >= Size) {
          Size *= 2;
          if (fi >= Size)
            Size = fi+1;
          DPRCSFrames.resize(Size);
        }
        DPRCSFrames[fi] = true;
      }
    }

    unsigned createJumpTableUId() {
      return JumpTableUId++;
    }

    unsigned getNumJumpTables() const {
      return JumpTableUId;
    }

    void initPICLabelUId(unsigned UId) {
      PICLabelUId = UId;
    }

    unsigned getNumPICLabels() const {
      return PICLabelUId;
    }

    unsigned createPICLabelUId() {
      return PICLabelUId++;
    }

    int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
    void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }


    bool globalBaseRegFixed() const;

    bool hasITBlocks() const { return HasITBlocks; }
    void setHasITBlocks(bool h) { HasITBlocks = h; }

    void recordCPEClone(unsigned CPIdx, unsigned CPCloneIdx) {
      if (!CPEClones.insert(std::make_pair(CPCloneIdx, CPIdx)).second)
        assert(0 && "Duplicate entries!");
    }

    unsigned getOriginalCPIdx(unsigned CloneIdx) const {
      DenseMap<unsigned, unsigned>::const_iterator I = CPEClones.find(CloneIdx);
      if (I != CPEClones.end())
        return I->second;
      else
        return -1U;
    }


    unsigned getMaxCallFrameSize() const { return MaxCallFrameSize; }
    void setMaxCallFrameSize(unsigned S) { MaxCallFrameSize = S; }


    unsigned getSRetReturnReg() const { return SRetReturnReg; }
    void setSRetReturnReg(unsigned Reg) { SRetReturnReg = Reg; }
  };

} // end of namespace llvm


#endif
