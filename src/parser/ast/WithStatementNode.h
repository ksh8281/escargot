/*
 * Copyright (c) 2016-present Samsung Electronics Co., Ltd
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 */

#ifndef WithStatementNode_h
#define WithStatementNode_h

#include "StatementNode.h"

namespace Escargot {

class WithStatementNode : public StatementNode {
public:
    friend class ScriptParser;
    WithStatementNode(RefPtr<Node> object, RefPtr<Node> body)
        : StatementNode()
        , m_object(object)
        , m_body(body)
    {
    }

    virtual ASTNodeType type() override { return ASTNodeType::WithStatement; }
    virtual void generateStatementByteCode(ByteCodeBlock* codeBlock, ByteCodeGenerateContext* context) override
    {
        size_t start = codeBlock->currentCodeSize();
        auto r = m_object->getRegister(codeBlock, context);
        m_object->generateExpressionByteCode(codeBlock, context, r);
        size_t withPos = codeBlock->currentCodeSize();
        context->m_recursiveStatementStack.push_back(std::make_pair(ByteCodeGenerateContext::With, withPos));
        codeBlock->pushCode(WithOperation(ByteCodeLOC(m_loc.index), r), context, this);
        context->giveUpRegister();

        bool isWithScopeBefore = context->m_isWithScope;
        context->m_isWithScope = true;
        m_body->generateStatementByteCode(codeBlock, context);
        context->registerJumpPositionsToComplexCase(start);

        codeBlock->pushCode(TryCatchFinallyWithBlockBodyEnd(ByteCodeLOC(m_loc.index)), context, this);
        codeBlock->peekCode<WithOperation>(withPos)->m_withEndPostion = codeBlock->currentCodeSize();
        context->m_isWithScope = isWithScopeBefore;

        context->m_recursiveStatementStack.pop_back();
    }

    virtual void iterateChildren(const std::function<void(Node* node)>& fn) override
    {
        fn(this);

        m_object->iterateChildren(fn);
        m_body->iterateChildren(fn);
    }

private:
    RefPtr<Node> m_object;
    RefPtr<Node> m_body; // body: [ Statement ];
};
}

#endif
