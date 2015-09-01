#ifndef ProgramNode_h
#define ProgramNode_h

#include "Node.h"
#include "StatementNode.h"

namespace escargot {

class ProgramNode : public Node {
public:
    friend class ESScriptParser;
    ProgramNode(StatementNodeVector&& body)
            : Node(NodeType::Program)
    {
        m_body = body;
        m_bodySize = m_body.size();
        m_rootedBody = m_body.data();
    }

    ESValue execute(ESVMInstance* instance)
    {
        ESValue ret;
        for(unsigned i = 0; i < m_bodySize ; i ++) {
            ESValue s = m_rootedBody[i]->execute(instance);
            if (!s.isEmpty())
                ret = s;
        }
        return ret;
    }
protected:
    StatementNodeVector m_body; //body: [ Statement ];
    Node** m_rootedBody;
    size_t m_bodySize;
};

}

#endif
