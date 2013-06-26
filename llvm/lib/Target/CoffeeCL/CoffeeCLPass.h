#ifndef COFFEECL_PASS_H
#define COFFEECL_PASS_H

#include "llvm/Pass.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/DataLayout.h"

namespace llvm {

class FunctionPass;
class Function;

class CoffeeCLPass : public FunctionPass {
public:
  static char ID; // Pass ID

  CoffeeCLPass() : FunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<DataLayout>();
    AU.addPreserved<MachineFunctionAnalysis>();
  }

  virtual const char *getPassName() const {
    return "Coffee CL pass";
  }

  virtual bool runOnFunction(Function &F);
};

extern FunctionPass *createCoffeeCLPass();

} // end namespace llvm

#endif
