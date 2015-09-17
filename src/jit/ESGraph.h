#ifndef ESGraph_h
#define ESGraph_h

#include <vector>
#include <iostream>

namespace nanojit {

class LIns;

}

namespace escargot {

class FunctionNode;

namespace ESJIT {

class ESIR;
class ESGraph;

class ESBasicBlock : public gc {
    friend class NativeGenerator;
public:
    static ESBasicBlock* create(ESGraph* graph, ESBasicBlock* parentBlock = nullptr)
    {
        // FIXME no bdwgc
        return new ESBasicBlock(graph, parentBlock);
    }

    void push(ESIR* ir) { m_instructions.push_back(ir); }
    size_t instructionSize() { return m_instructions.size(); }
    ESIR* instruction(size_t index) { return m_instructions[index]; }

    void addParent(ESBasicBlock* parent) { m_parents.push_back(parent); }
    void addChild(ESBasicBlock* child) { m_children.push_back(child); }

    size_t index() { return m_index; }
    bool endsWithJumpOrBranch();

    void setLabel(nanojit::LIns* label) { m_label = label; }
    nanojit::LIns* getLabel() { return m_label; }

    void addJumpOrBranchSource(nanojit::LIns* source) { 
        m_jumpOrBranchSources.push_back(source);
    }

#ifndef NDEBUG
    void dump(std::ostream& out);
#endif

private:
    ESBasicBlock(ESGraph* graph, ESBasicBlock* parentBlock);

    ESGraph* m_graph;
    std::vector<ESBasicBlock*> m_parents;
    std::vector<ESBasicBlock*> m_children;
    std::vector<ESIR*> m_instructions;
    size_t m_index;
    nanojit::LIns* m_label;
    std::vector<nanojit::LIns*> m_jumpOrBranchSources;
};

class ESGraph : public gc {
    friend class NativeGenerator;
public:
    static ESGraph* create(CodeBlock* codeBlock) 
    {
        ASSERT(codeBlock);
        // FIXME no bdwgc
        return new ESGraph(codeBlock);
    }
    size_t basicBlockSize() { return m_basicBlocks.size(); }
    ESBasicBlock* basicBlock(size_t index) { return m_basicBlocks[index]; }
    void push(ESBasicBlock* bb) { m_basicBlocks.push_back(bb); }

    int tempRegisterSize();

#ifndef NDEBUG
    void dump(std::ostream& out);
#endif

private:
    ESGraph(CodeBlock* codeBlock) : m_codeBlock(codeBlock) { }

    std::vector<ESBasicBlock*> m_basicBlocks;
    CodeBlock* m_codeBlock;
};

}}
#endif
