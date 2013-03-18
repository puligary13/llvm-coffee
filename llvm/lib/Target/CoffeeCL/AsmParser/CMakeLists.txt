include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/.. )
add_llvm_library(LLVMCoffeeCLAsmParser
  CoffeeCLAsmParser.cpp
  CoffeeCLAsmLexer.cpp
  )

add_dependencies(LLVMCoffeeCLAsmParser MipsCommonTableGen)
