#ifndef BinaryExpressionSignedRightShiftNode_h
#define BinaryExpressionSignedRightShiftNode_h

#include "ExpressionNode.h"

namespace escargot {

class BinaryExpressionSignedRightShiftNode : public ExpressionNode {
public:
    BinaryExpressionSignedRightShiftNode(Node *left, Node* right)
            : ExpressionNode(NodeType::BinaryExpressionSignedRightShift)
    {
        m_left = (ExpressionNode*)left;
        m_right = (ExpressionNode*)right;
    }

    ESValue execute(ESVMInstance* instance)
    {
        int32_t lnum = m_left->execute(instance).toInt32();
        int32_t rnum = m_right->execute(instance).toInt32();
        lnum >>= ((unsigned int)rnum) & 0x1F;
        return ESValue(lnum);
    }
protected:
    ExpressionNode* m_left;
    ExpressionNode* m_right;
};

}

#endif
