set(LLVM_TARGET_DEFINITIONS Coffee.td)

tablegen(LLVM CoffeeGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM CoffeeGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM CoffeeGenMCCodeEmitter.inc -gen-emitter -mc-emitter)
tablegen(LLVM CoffeeGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM CoffeeGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM CoffeeGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM CoffeeGenCallingConv.inc -gen-callingconv)
tablegen(LLVM CoffeeGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(CoffeeCommonTableGen)

add_llvm_target(CoffeeCodeGen
  CoffeeAsmPrinter.cpp
  CoffeeBranchSelector.cpp
  CoffeeCodeEmitter.cpp
  CoffeeHazardRecognizers.cpp
  CoffeeInstrInfo.cpp
  CoffeeISelDAGToDAG.cpp
  CoffeeISelLowering.cpp
  CoffeeFrameLowering.cpp
  CoffeeMCInstLower.cpp
  CoffeeMachineFunctionInfo.cpp
  CoffeeRegisterInfo.cpp
  CoffeeTargetMachine.cpp
  CoffeeSelectionDAGInfo.cpp
  CoffeeDelaySlotFiller.cpp
  CoffeeTargetObjectFile.cpp
  CoffeeSubtarget.cpp
  )

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
