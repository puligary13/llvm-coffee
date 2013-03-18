##===- lib/Target/PowerPC/Makefile -------------------------*- Makefile -*-===##
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
##===----------------------------------------------------------------------===##

LEVEL = ../../..
LIBRARYNAME = LLVMCoffeeCLCodeGen
TARGET = CoffeeCL

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = CoffeeCLGenRegisterInfo.inc \
                CoffeeCLGenAsmWriter.inc  CoffeeCLGenCodeEmitter.inc \
                CoffeeCLGenInstrInfo.inc CoffeeCLGenDAGISel.inc \
                CoffeeCLGenSubtargetInfo.inc CoffeeCLGenCallingConv.inc \
                CoffeeCLGenMCCodeEmitter.inc

DIRS = InstPrinter TargetInfo MCTargetDesc

include $(LEVEL)/Makefile.common
