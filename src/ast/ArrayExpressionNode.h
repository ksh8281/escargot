#ifndef ArrayExpressionNode_h
#define ArrayExpressionNode_h

#include "ExpressionNode.h"

namespace escargot {

class ArrayExpressionNode : public ExpressionNode {
public:
    friend class ScriptParser;
    ArrayExpressionNode(ExpressionNodeVector&& elements)
        : ExpressionNode(NodeType::ArrayExpression)
    {
        m_elements = elements;
    }

    virtual NodeType type() { return NodeType::ArrayExpression; }

    virtual void generateExpressionByteCode(CodeBlock* codeBlock, ByteCodeGenerateContext& context)
    {
        unsigned len = m_elements.size();
        codeBlock->pushCode(CreateArray(len), context, this);
        for (unsigned i = 0; i < len; i++) {
            codeBlock->pushCode(Push(ESValue(i)), context, this);
            if (m_elements[i]) {
                m_elements[i]->generateExpressionByteCode(codeBlock, context);
            } else {
                codeBlock->pushCode(Push(ESValue(ESValue::ESEmptyValue)), context, this);
            }
            codeBlock->pushCode(InitObject(), context, this);
        }
    }

    virtual void computeRoughCodeBlockSizeInWordSize(size_t& result)
    {
        result += m_elements.size() * 2;
        unsigned len = m_elements.size();
        for (unsigned i = 0; i < len; i++) {
            if (m_elements[i]) {
                m_elements[i]->computeRoughCodeBlockSizeInWordSize(result);
            }
        }
    }
protected:
    ExpressionNodeVector m_elements;
};

}

#endif
