#ifndef FunctionNode_h
#define FunctionNode_h

#include "Node.h"
#include "PatternNode.h"
#include "ExpressionNode.h"


namespace escargot {

class FunctionNode : public Node {
public:
    FunctionNode(NodeType type ,const InternalAtomicString& id, InternalAtomicStringVector&& params, Node* body,bool isGenerator, bool isExpression)
            : Node(type)
    {
        m_id = id;
        m_nonAtomicId = id.data();
        m_params = params;
        for(unsigned i = 0 ; i < m_params.size() ; i ++) {
            m_nonAtomicParams.push_back(m_params[i].data());
        }
        m_body = body;
        m_isGenerator = isGenerator;
        m_isExpression = isExpression;
        m_needsActivation = false;
        m_outerFunctionNode = NULL;
        m_needsArgumentsObject = false;
    }

    ALWAYS_INLINE const InternalAtomicStringVector& params() { return m_params; }
    ALWAYS_INLINE const InternalStringVector& nonAtomicParams() { return m_nonAtomicParams; }
    ALWAYS_INLINE Node* body() { return m_body; }
    ALWAYS_INLINE const InternalAtomicString& id() { return m_id; }
    ALWAYS_INLINE const InternalString& nonAtomicId() { return m_nonAtomicId; }

    ALWAYS_INLINE bool needsActivation() { return m_needsActivation; } //child & parent AST has eval, with, catch
    ALWAYS_INLINE void setNeedsActivation(bool b) { m_needsActivation = b; }
    ALWAYS_INLINE bool needsArgumentsObject() { return m_needsArgumentsObject; }
    ALWAYS_INLINE void markNeedsArgumentsObject() { m_needsArgumentsObject = true; }

    void setInnerIdentifiers(InternalAtomicStringVector&& vec)
    {
        m_innerIdentifiers = vec;
    }

    InternalAtomicStringVector& innerIdentifiers() { return m_innerIdentifiers; }

    void setOuterFunctionNode(FunctionNode* o) { m_outerFunctionNode = o; }
    FunctionNode* outerFunctionNode() { return m_outerFunctionNode; }

protected:
    InternalAtomicString m_id; //id: Identifier;
    InternalString m_nonAtomicId; //id: Identifier;
    InternalAtomicStringVector m_params; //params: [ Pattern ];
    InternalStringVector m_nonAtomicParams;
    InternalAtomicStringVector m_innerIdentifiers;
    //defaults: [ Expression ];
    //rest: Identifier | null;
    Node* m_body; //body: BlockStatement | Expression;
    bool m_isGenerator; //generator: boolean;
    bool m_isExpression; //expression: boolean;

    bool m_needsActivation;
    bool m_needsArgumentsObject;
    FunctionNode* m_outerFunctionNode;
};

}

#endif
