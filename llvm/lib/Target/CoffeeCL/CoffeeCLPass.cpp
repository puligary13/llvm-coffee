#include "CoffeeCLPass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/Support/Debug.h"

namespace llvm {
FunctionPass *createCoffeeCLPass() {
    return new CoffeeCLPass();
}

char CoffeeCLPass::ID = 0;


bool CoffeeCLPass::runOnFunction(Function &function) {
    bool               functionModified    = false;

    Function::iterator I = function.begin();


    SmallVector<Instruction *, 4> instructionToRemove;

    for (BasicBlock::iterator II = I->begin(), IE = I->end(); II != IE;
         ++II) {
        DEBUG(dbgs() << "start ");



        Instruction *Inst = II;

 DEBUG(dbgs() << "start 1");

        if (Inst) {
            DEBUG(dbgs() << "coffee cl pass this opcode " << Inst->getOpcodeName()<<"\n");


            if (Inst->getOpcode() == Instruction::GetElementPtr ) {

               /*   Instruction *NextInst = dyn_cast<Instruction>(++II);
                  DEBUG(dbgs() << "coffee cl pass this  next opcode " << NextInst->getOpcodeName()<<"\n");

                  if( !NextInst || NextInst->getOpcode() != Instruction::Load)
                     return false;


                SmallVector<Value*, 8> Args(Inst->op_begin(), Inst->op_end());*/

                SmallVector<Value*, 8> Args(Inst->op_begin(), Inst->op_end());

                std::vector<Type*> ParamTypeList;

                for (int i = 0; i != Args.size(); ++i)
                    ParamTypeList.push_back(Args[i]->getType());

                Type* RetType = Inst->getType(); //IntegerType::getInt32Ty(Inst->getContext());

                FunctionType *FT =
                  FunctionType::get(RetType, ParamTypeList, /*isVarArg*/ false);
                Function* Fn = Function::Create(FT, GlobalValue::ExternalLinkage, "llvm.coffeecl.getelementptr");

                CallInst *NewCall = CallInst::Create(Fn, Args,"llvm.coffeecl.getelementptr", Inst );

                 NewCall->setCallingConv(CallingConv::COFFEECL_Device);

               // Instruction *NextInst = dyn_cast<Instruction>(++II);

               Inst->replaceAllUsesWith(NewCall);

               instructionToRemove.push_back(Inst);



               // --II;


               // Type* elementType = IntegerType::getInt32Ty(Inst->getContext());

               // Type* ArgType = PointerType::get(elementType, 1);

                 functionModified = true;
            }

            if (Inst->getOpcode() == Instruction::Load) {

                SmallVector<Value*, 8> Args(Inst->op_begin(), Inst->op_end());
                std::vector<Type*> ParamTypeList;

                for (int i = 0; i != Args.size(); ++i)
                    ParamTypeList.push_back(Args[i]->getType());

                Type* RetType = Inst->getType(); //IntegerType::getInt32Ty(Inst->getContext());

                FunctionType *FT =
                  FunctionType::get(RetType, ParamTypeList, /*isVarArg*/ false);
                Function* Fn = Function::Create(FT, GlobalValue::ExternalLinkage, "llvm.coffeecl.load");

                CallInst *NewCall = CallInst::Create(Fn, Args,"llvm.coffeecl.load",Inst );

                 NewCall->setCallingConv(CallingConv::COFFEECL_Device);

                 Inst->replaceAllUsesWith(NewCall);

                  instructionToRemove.push_back(Inst);

              //   Inst->replaceAllUsesWith(NewCall);


              //  Inst->removeFromParent();
                // Inst->eraseFromParent();

               //  if (II == IE)
               //      break;

               //  II++;

                 // Instruction *TestInst = dyn_cast<Instruction>(II);

                  // DEBUG(dbgs() << "coffee cl pass this  next of next opcode " << TestInst->getOpcodeName()<<"\n");

                // isGetElementPtr = false;
                 functionModified = true;

                // NextInst->replaceAllUsesWith(NewCall);

               //
               //  NextInst->removeFromParent();





                 //NewCall->insertBefore(Inst);
               //





              /*  I = CallInst::Create(Callee, Args);
                InstructionList.push_back(I);
                cast<CallInst>(I)->setCallingConv(
                  static_cast<CallingConv::ID>(CCInfo>>1));
                cast<CallInst>(I)->setTailCall(CCInfo & 1);
                cast<CallInst>(I)->setAttributes(PAL);


                callInst->insertBefore(II);*/


            }





         /*   for (Instruction::op_iterator OI = Inst->op_begin(), OE = Inst->op_end();
                 OI != OE; ++OI) {

                  Value *val = dyn_cast<Value>(OI);
                 // DEBUG(dbgs() << "coffee cl pass oprand " << val->getName()<<"\n");
            }*/

            }

       // --II;
    }

    for (unsigned i = 0, e = instructionToRemove.size(); i != e; ++i) {
     Instruction *Inst = instructionToRemove[i];

      Inst->eraseFromParent();
    }



        return functionModified;
    }

} // llvm namespace end
