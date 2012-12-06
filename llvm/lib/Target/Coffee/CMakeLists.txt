set(LLVM_TARGET_DEFINITIONS Coffee.td)

tablegen(LLVM CoffeeGenAsmWriter.inc -gen-asm-writer)
#tablegen(LLVM CoffeeGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM CoffeeGenMCCodeEmitter.inc -gen-emitter -mc-emitter)
tablegen(LLVM CoffeeGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM CoffeeGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM CoffeeGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM CoffeeGenCallingConv.inc -gen-callingconv)
tablegen(LLVM CoffeeGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM CoffeeGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM CoffeeGenMCPseudoLowering.inc -gen-pseudo-lowering)

add_public_tablegen_target(CoffeeCommonTableGen)

add_llvm_target(CoffeeCodeGen
  CoffeeAnalyzeImmediate.cpp
  CoffeeAsmPrinter.cpp
  CoffeeDelaySlotFiller.cpp
  CoffeeInstrInfo.cpp
  CoffeeISelDAGToDAG.cpp
  CoffeeISelLowering.cpp
  CoffeeFrameLowering.cpp
  CoffeeMCInstLower.cpp
  CoffeeMachineFunctionInfo.cpp
  CoffeeRegisterInfo.cpp
  CoffeeSubtarget.cpp
  CoffeeTargetMachine.cpp
  CoffeeTargetObjectFile.cpp
  CoffeeSelectionDAGInfo.cpp
  CoffeeCodeEmitter.cpp
  CoffeeBranchSelector.cpp
  CoffeeHazardRecognizers.cpp

  )

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)


