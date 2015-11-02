#ifndef FunctionDeclarationNode_h
#define FunctionDeclarationNode_h

#include "FunctionNode.h"

namespace escargot {

class FunctionDeclarationNode : public FunctionNode {
public:
    friend class ScriptParser;
    FunctionDeclarationNode(const InternalAtomicString& id, InternalAtomicStringVector&& params, Node* body, bool isGenerator, bool isExpression, bool isStrict)
        : FunctionNode(NodeType::FunctionDeclaration, id, std::move(params), body, isGenerator, isExpression, isStrict)
    {
        m_isExpression = false;
    }

    FunctionDeclarationNode(ESString* id, InternalAtomicStringVector&& params, Node* body, bool isGenerator, bool isExpression, bool isStrict)
        : FunctionNode(NodeType::FunctionDeclaration, id->data(), std::move(params), body, isGenerator, isExpression, isStrict)
    {
        m_isExpression = false;
    }

    virtual void generateStatementByteCode(CodeBlock* codeBlock, ByteCodeGenerateContext& context)
    {
        CodeBlock* cb = CodeBlock::create();
        cb->m_innerIdentifiers = std::move(m_innerIdentifiers);
        cb->m_needsActivation = m_needsActivation;
        cb->m_params = std::move(m_params);
        cb->m_isStrict = m_isStrict;

        ByteCodeGenerateContext newContext;
        m_body->generateStatementByteCode(cb, newContext);
#ifdef ENABLE_ESJIT
        cb->m_tempRegisterSize = newContext.m_currentSSARegisterCount;
#endif
        cb->pushCode(ReturnFunction(), newContext, this);

#ifndef NDEBUG
        cb->m_id = m_id;
        cb->m_nonAtomicId = m_nonAtomicId;
        if(ESVMInstance::currentInstance()->m_dumpByteCode) {
            char* code = cb->m_code.data();
            ByteCode* currentCode = (ByteCode *)(&code[0]);
            if(currentCode->m_orgOpcode != ExecuteNativeFunctionOpcode) {
                cb->m_nonAtomicId = m_nonAtomicId;
                dumpBytecode(cb);
            }
        }
        if (ESVMInstance::currentInstance()->m_reportUnsupportedOpcode) {
            char* code = cb->m_code.data();
            ByteCode* currentCode = (ByteCode *)(&code[0]);
            if(currentCode->m_orgOpcode != ExecuteNativeFunctionOpcode) {
                dumpUnsupported(cb);
            }
        }
#endif
        codeBlock->pushCode(CreateFunction(m_id, m_nonAtomicId, cb, true), context, this);
#ifdef ENABLE_ESJIT
        newContext.cleanupSSARegisterCount();
#endif
    }

protected:
};

}

#endif


