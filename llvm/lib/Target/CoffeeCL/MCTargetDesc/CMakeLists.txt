add_llvm_library(LLVMCoffeeCLDesc
  CoffeeCLAsmBackend.cpp
  CoffeeCLDirectObjLower.cpp
  CoffeeCLMCAsmInfo.cpp
  CoffeeCLMCCodeEmitter.cpp
  CoffeeCLMCTargetDesc.cpp
  CoffeeCLELFObjectWriter.cpp
  )

add_dependencies(LLVMCoffeeCLDesc CoffeeCLCommonTableGen)
