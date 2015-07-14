#ifndef __ESScriptParser__
#define __ESScriptParser__

#include "ast/AST.h"

namespace escargot {

class ESScriptParser {
public:

    static AST* parseScript(const std::string& cs);
    //TODO
    //static AST* parseScript(const wchar_t* str);
};

}

#endif
