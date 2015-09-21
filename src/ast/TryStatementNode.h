#ifndef TryStatementNode_h
#define TryStatementNode_h

#include "StatementNode.h"
#include "runtime/ExecutionContext.h"
#include "CatchClauseNode.h"

namespace escargot {

class TryStatementNode : public StatementNode {
public:
    friend class ESScriptParser;
    TryStatementNode(Node *block, Node *handler, CatchClauseNodeVector&& guardedHandlers,  Node *finalizer)
        : StatementNode(NodeType::TryStatement)
      {
          m_block = (BlockStatementNode*) block;
          m_handler = (CatchClauseNode*) handler;
          m_guardedHandlers = guardedHandlers;
          m_finalizer = (BlockStatementNode*) finalizer;
      }

    virtual void generateStatementByteCode(CodeBlock* codeBlock, ByteCodeGenerateContext& context)
    {
        ASSERT(!m_finalizer); //TODO
        codeBlock->pushCode(Try(), this);
        size_t pos = codeBlock->lastCodePosition<Try>();
        m_block->generateStatementByteCode(codeBlock, context);
        codeBlock->pushCode(TryCatchBodyEnd(), this);
        size_t catchPos = codeBlock->currentCodeSize();
        m_handler->generateStatementByteCode(codeBlock, context);
        codeBlock->pushCode(TryCatchBodyEnd(), this);
        size_t endPos = codeBlock->currentCodeSize();
        codeBlock->peekCode<Try>(pos)->m_catchPosition = catchPos;
        codeBlock->peekCode<Try>(pos)->m_statementEndPosition = endPos;
        codeBlock->peekCode<Try>(pos)->m_name = m_handler->param()->name();
        codeBlock->peekCode<Try>(pos)->m_nonAtomicName = m_handler->param()->nonAtomicName();
    }

protected:
    BlockStatementNode *m_block;
    CatchClauseNode *m_handler;
    CatchClauseNodeVector m_guardedHandlers;
    BlockStatementNode *m_finalizer;
};

}

#endif
