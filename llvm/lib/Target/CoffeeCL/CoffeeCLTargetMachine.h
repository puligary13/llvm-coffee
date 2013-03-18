//===-- CoffeeCLTargetMachine.h - Define TargetMachine for CoffeeCL ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the CoffeeCL specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef CoffeeCL_TARGETMACHINE_H
#define CoffeeCL_TARGETMACHINE_H

#include "CoffeeCLFrameLowering.h"
#include "CoffeeCLSubtarget.h"
#include "CoffeeCLInstrInfo.h"
#include "CoffeeCLISelLowering.h"
#include "CoffeeCLSelectionDAGInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/DataLayout.h"

namespace llvm {

/// CoffeeCLTargetMachine - 32 bit
///
class CoffeeCLTargetMachine : public LLVMTargetMachine {
    virtual void anchor();
    CoffeeCLSubtarget       Subtarget;
    const DataLayout       DL;       // Calculates type size & alignment
    CoffeeCLInstrInfo        InstrInfo;
    CoffeeCLFrameLowering    FrameLowering;
    CoffeeCLTargetLowering   TLInfo;
    CoffeeCLSelectionDAGInfo TSInfo;

public:
    CoffeeCLTargetMachine(const Target &T, StringRef TT,
                        StringRef CPU, StringRef FS, const TargetOptions &Options,
                        Reloc::Model RM, CodeModel::Model CM,
                        CodeGenOpt::Level OL);

    virtual const CoffeeCLInstrInfo      *getInstrInfo() const { return &InstrInfo; }
    virtual const CoffeeCLFrameLowering  *getFrameLowering() const {
        return &FrameLowering;
    }

    virtual const CoffeeCLTargetLowering *getTargetLowering() const {
        return &TLInfo;
    }
    virtual const CoffeeCLSelectionDAGInfo* getSelectionDAGInfo() const {
        return &TSInfo;
    }
    virtual const CoffeeCLRegisterInfo   *getRegisterInfo() const {
        return &InstrInfo.getRegisterInfo();
    }

    virtual const DataLayout *getDataLayout()    const    { return &DL; }

    virtual const CoffeeCLSubtarget *getSubtargetImpl() const
    { return &Subtarget; }

    // Pass Pipeline Configuration
    virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);
    virtual bool addCodeEmitter(PassManagerBase &PM,
                                JITCodeEmitter &JCE) {}
};

} // end namespace llvm

#endif
