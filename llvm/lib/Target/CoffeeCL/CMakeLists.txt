set(LLVM_TARGET_DEFINITIONS CoffeeCL.td)

tablegen(LLVM CoffeeCLGenAsmWriter.inc -gen-asm-writer)
#tablegen(LLVM CoffeeCLGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM CoffeeCLGenMCCodeEmitter.inc -gen-emitter -mc-emitter)
tablegen(LLVM CoffeeCLGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM CoffeeCLGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM CoffeeCLGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM CoffeeCLGenCallingConv.inc -gen-callingconv)
tablegen(LLVM CoffeeCLGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM CoffeeCLGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM CoffeeCLGenMCPseudoLowering.inc -gen-pseudo-lowering)

add_public_tablegen_target(CoffeeCLCommonTableGen)

add_llvm_target(CoffeeCLCodeGen
  CoffeeCLAnalyzeImmediate.cpp
  CoffeeCLAsmPrinter.cpp
  CoffeeCLDelaySlotFiller.cpp
  CoffeeCLInstrInfo.cpp
  CoffeeCLISelDAGToDAG.cpp
  CoffeeCLISelLowering.cpp
  CoffeeCLFrameLowering.cpp
  CoffeeCLMCInstLower.cpp
  CoffeeCLMachineFunctionInfo.cpp
  CoffeeCLRegisterInfo.cpp
  CoffeeCLSubtarget.cpp
  CoffeeCLTargetMachine.cpp
  CoffeeCLTargetObjectFile.cpp
  CoffeeCLSelectionDAGInfo.cpp
  CoffeeCLCodeEmitter.cpp
  CoffeeCLBranchSelector.cpp
  CoffeeCLHazardRecognizers.cpp

  )

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)


