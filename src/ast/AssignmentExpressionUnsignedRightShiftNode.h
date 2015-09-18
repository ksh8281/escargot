#ifndef AssignmentExpressionUnsignedRightShiftNode_h
#define AssignmentExpressionUnsignedRightShiftNode_h

#include "ExpressionNode.h"
#include "PatternNode.h"
#include "IdentifierNode.h"

namespace escargot {

//An assignment operator expression.
class AssignmentExpressionUnsignedShiftNode : public ExpressionNode {
public:
    friend class ESScriptParser;

    AssignmentExpressionUnsignedShiftNode(Node* left, Node* right)
            : ExpressionNode(NodeType::AssignmentExpressionUnsignedRightShift)
    {
        m_left = left;
        m_right = right;
    }

    ESValue executeExpression(ESVMInstance* instance)
    {
        ESSlotAccessor slot = m_left->executeForWrite(instance);
        int32_t lnum = slot.value().toInt32();
        int32_t rnum = m_right->executeExpression(instance).toInt32();
        unsigned int shiftCount = ((unsigned int)rnum) & 0x1F;
        lnum = ((unsigned int)lnum) >> shiftCount;
        ESValue rvalue(lnum);
        slot.setValue(rvalue);
        return rvalue;
    }

    virtual void generateExpressionByteCode(CodeBlock* codeBlock, ByteCodeGenereateContext& context)
    {
        m_left->generateByteCodeWriteCase(codeBlock, context);
        codeBlock->pushCode(ReferenceTopValueWithPeeking(), this);
        m_right->generateExpressionByteCode(codeBlock, context);
        codeBlock->pushCode(UnsignedRightShift(), this);
        codeBlock->pushCode(Put(), this);
    }

protected:
    Node* m_left; //left: Pattern;
    Node* m_right; //right: Expression;
};

}

#endif
