//===-- CoffeeFrameLowering.h - Define frame lowering for Coffee --*- C++ -*-===//
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

#ifndef Coffee_FRAMEINFO_H
#define Coffee_FRAMEINFO_H

#include "Coffee.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/ADT/STLExtras.h"

namespace llvm {

class CoffeeFrameLowering: public TargetFrameLowering {

public:
    CoffeeFrameLowering()

        : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 8, 0)
    {
        // guoqing: let's use 8 bytes for stack alignment for now, same as mips
        // need to double check this
    }


    int ResolveFrameIndexReference(const MachineFunction &MF,
                                   int FI, unsigned &FrameReg,
                                   int SPAdj) const;

    void emitPrologue(MachineFunction &MF) const;
    void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

    bool hasFP(const MachineFunction &MF) const;



    void processFunctionBeforeCalleeSavedScan(MachineFunction &MF,
                                              RegScavenger *RS = NULL) const;
    void processFunctionBeforeFrameFinalized(MachineFunction &MF) const;

  /*  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MI,
                                   const std::vector<CalleeSavedInfo> &CSI,
                                   const TargetRegisterInfo *TRI) const;

    bool restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator MI,
                                     const std::vector<CalleeSavedInfo> &CSI,
                                     const TargetRegisterInfo *TRI) const;*/

    void emitPopInst(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       const std::vector<CalleeSavedInfo> &CSI,
                                       unsigned LdrOpc,
                                       unsigned StackSize,
                                       bool isVarArg,
                                       unsigned NumAlignedDPRCS2Regs) const;

    void emitPushInst(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        unsigned StrOpc,
                                        unsigned StackSize,
                                        unsigned MIFlags) const;
};

} // End llvm namespace

#endif
