//===-- CoffeeTargetMachine.h - Define TargetMachine for Coffee ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Coffee specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef Coffee_TARGETMACHINE_H
#define Coffee_TARGETMACHINE_H

#include "CoffeeFrameLowering.h"
#include "CoffeeSubtarget.h"
#include "CoffeeInstrInfo.h"
#include "CoffeeISelLowering.h"
#include "CoffeeSelectionDAGInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/DataLayout.h"

namespace llvm {

/// CoffeeTargetMachine - 32 bit
///
class CoffeeTargetMachine : public LLVMTargetMachine {
    virtual void anchor();
    CoffeeSubtarget       Subtarget;
    const DataLayout       DL;       // Calculates type size & alignment
    CoffeeInstrInfo        InstrInfo;
    CoffeeFrameLowering    FrameLowering;
    CoffeeTargetLowering   TLInfo;
    CoffeeSelectionDAGInfo TSInfo;

public:
    CoffeeTargetMachine(const Target &T, StringRef TT,
                        StringRef CPU, StringRef FS, const TargetOptions &Options,
                        Reloc::Model RM, CodeModel::Model CM,
                        CodeGenOpt::Level OL);

    virtual const CoffeeInstrInfo      *getInstrInfo() const { return &InstrInfo; }
    virtual const CoffeeFrameLowering  *getFrameLowering() const {
        return &FrameLowering;
    }

    virtual const CoffeeTargetLowering *getTargetLowering() const {
        return &TLInfo;
    }
    virtual const CoffeeSelectionDAGInfo* getSelectionDAGInfo() const {
        return &TSInfo;
    }
    virtual const CoffeeRegisterInfo   *getRegisterInfo() const {
        return &InstrInfo.getRegisterInfo();
    }

    virtual const DataLayout *getDataLayout()    const    { return &DL; }

    virtual const CoffeeSubtarget *getSubtargetImpl() const
    { return &Subtarget; }

    // Pass Pipeline Configuration
    virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);
    virtual bool addCodeEmitter(PassManagerBase &PM,
                                JITCodeEmitter &JCE) {}
};

} // end namespace llvm

#endif
