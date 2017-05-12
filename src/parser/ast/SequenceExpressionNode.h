/*
 * Copyright (c) 2016-present Samsung Electronics Co., Ltd
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

#ifndef SequenceExpressionNode_h
#define SequenceExpressionNode_h

#include "ExpressionNode.h"

namespace Escargot {

// An sequence expression, i.e., a statement consisting of vector of expressions.
class SequenceExpressionNode : public ExpressionNode {
public:
    friend class ScriptParser;
    SequenceExpressionNode(ExpressionNodeVector&& expressions)
        : ExpressionNode()
    {
        m_expressions = expressions;
    }

    virtual ~SequenceExpressionNode()
    {
        for (size_t i = 0; i < m_expressions.size(); i++) {
            delete m_expressions[i];
        }
    }

    virtual void generateExpressionByteCode(ByteCodeBlock* codeBlock, ByteCodeGenerateContext* context, ByteCodeRegisterIndex dstRegister)
    {
        ByteCodeRegisterIndex r = 0;
        for (size_t i = 0; i < m_expressions.size(); i++) {
            r = m_expressions[i]->getRegister(codeBlock, context);
            m_expressions[i]->generateExpressionByteCode(codeBlock, context, r);
            context->giveUpRegister();
        }
        if (r != dstRegister) {
            codeBlock->pushCode(Move(ByteCodeLOC(m_loc.index), r, dstRegister), context, this);
        }
    }

    virtual void iterateChildrenIdentifier(const std::function<void(AtomicString name, bool isAssignment)>& fn)
    {
        for (size_t i = 0; i < m_expressions.size(); i++) {
            m_expressions[i]->iterateChildrenIdentifier(fn);
        }
    }

    virtual ASTNodeType type() { return ASTNodeType::SequenceExpression; }
    const ExpressionNodeVector& expressions() { return m_expressions; }
protected:
    ExpressionNodeVector m_expressions; // expression: Expression;
};
}

#endif
