#ifndef UpdateExpressionDecrementPrefixNode_h
#define UpdateExpressionDecrementPrefixNode_h

#include "ExpressionNode.h"

namespace escargot {

class UpdateExpressionDecrementPrefixNode : public ExpressionNode {
public:
    friend class ESScriptParser;

    UpdateExpressionDecrementPrefixNode(Node *argument)
            : ExpressionNode(NodeType::UpdateExpressionDecrementPrefix)
    {
        m_argument = (ExpressionNode*)argument;
    }

    ESValue executeExpression(ESVMInstance* instance)
    {
        ESSlotAccessor slot = m_argument->executeForWrite(instance);
        ESValue argval = slot.value();
        ESValue ret(ESValue::ESForceUninitialized);

        if (LIKELY(argval.isInt32())) {
            //FIXME check overflow
            argval = ESValue(argval.asInt32() - 1);
        } else {
            double argnum = argval.toNumber();
            argval = ESValue(argnum - 1);
        }

        slot.setValue(argval);

        ret = argval;
        return ret;
    }

    virtual void generateExpressionByteCode(CodeBlock* codeBlock, ByteCodeGenereateContext& context)
    {
        m_argument->generateByteCodeWriteCase(codeBlock, context);
        m_argument->generateExpressionByteCode(codeBlock, context);
        codeBlock->pushCode(Push(ESValue(-1)), this);
        codeBlock->pushCode(Plus(), this);
        codeBlock->pushCode(Put(), this);
    }
protected:
    ExpressionNode* m_argument;
};

}

#endif
