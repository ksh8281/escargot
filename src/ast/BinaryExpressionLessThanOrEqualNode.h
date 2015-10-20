#ifndef BinaryExpressionLessThanOrEqualNode_h
#define BinaryExpressionLessThanOrEqualNode_h

#include "ExpressionNode.h"

namespace escargot {

class BinaryExpressionLessThanOrEqualNode : public ExpressionNode {
public:
    friend class ScriptParser;

    BinaryExpressionLessThanOrEqualNode(Node *left, Node* right)
            : ExpressionNode(NodeType::BinaryExpressionLessThanOrEqual)
    {
        m_left = (ExpressionNode*)left;
        m_right = (ExpressionNode*)right;
    }

    virtual void generateExpressionByteCode(CodeBlock* codeBlock, ByteCodeGenerateContext& context)
    {
        m_left->generateExpressionByteCode(codeBlock, context);
        m_right->generateExpressionByteCode(codeBlock, context);
        updateNodeIndex(context);
        codeBlock->pushCode(LessThanOrEqual(), context, this);
        WRITE_LAST_INDEX(m_nodeIndex, m_left->nodeIndex(), m_right->nodeIndex());
    }

protected:
    ExpressionNode* m_left;
    ExpressionNode* m_right;
};

}

#endif
