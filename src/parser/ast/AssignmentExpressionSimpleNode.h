/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef AssignmentExpressionSimpleNode_h
#define AssignmentExpressionSimpleNode_h

#include "ExpressionNode.h"
#include "IdentifierNode.h"
#include "PatternNode.h"
// #include "MemberExpressionNode.h"

namespace Escargot {

// An assignment operator expression.
class AssignmentExpressionSimpleNode : public ExpressionNode {
public:
    friend class ScriptParser;

    AssignmentExpressionSimpleNode(Node* left, Node* right)
        : ExpressionNode()
    {
        m_left = left;
        m_right = right;
    }

    Node* left()
    {
        return m_left;
    }

    virtual ASTNodeType type() { return ASTNodeType::AssignmentExpressionSimple; }
    virtual void generateExpressionByteCode(ByteCodeBlock* codeBlock, ByteCodeGenerateContext* context)
    {
        m_right->generateExpressionByteCode(codeBlock, context);
        m_left->generateStoreByteCode(codeBlock, context);
    }

protected:
    Node* m_left; // left: Pattern;
    Node* m_right; // right: Expression;
};
}

#endif