add_llvm_library(LLVMCoffeeDesc
  CoffeeAsmBackend.cpp
  CoffeeMCTargetDesc.cpp
  CoffeeMCAsmInfo.cpp
  CoffeeMCCodeEmitter.cpp
  CoffeePredicates.cpp
  CoffeeELFObjectWriter.cpp
  )

add_dependencies(LLVMCoffeeDesc CoffeeCommonTableGen)
