##===- lib/Target/PowerPC/Makefile -------------------------*- Makefile -*-===##
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
##===----------------------------------------------------------------------===##

LEVEL = ../../..
LIBRARYNAME = LLVMCoffeeCodeGen
TARGET = Coffee

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = CoffeeGenRegisterInfo.inc \
                CoffeeGenAsmWriter.inc  CoffeeGenCodeEmitter.inc \
                CoffeeGenInstrInfo.inc CoffeeGenDAGISel.inc \
                CoffeeGenSubtargetInfo.inc CoffeeGenCallingConv.inc \
                CoffeeGenMCCodeEmitter.inc

DIRS = InstPrinter TargetInfo MCTargetDesc

include $(LEVEL)/Makefile.common
