include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/.. )
add_llvm_library(LLVMCoffeeAsmParser
  CoffeeAsmParser.cpp
  CoffeeAsmLexer.cpp
  )

add_dependencies(LLVMCoffeeAsmParser MipsCommonTableGen)
