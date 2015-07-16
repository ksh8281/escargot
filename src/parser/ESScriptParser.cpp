#include "Escargot.h"
#include "ESScriptParser.h"

namespace escargot {

Node* ESScriptParser::parseScript(const std::string& source)
{
    std::string sc;
    for(unsigned i = 0 ; i < source.length() ; i ++) {
        char c = source[i];

        if(c == '\n') {
            sc.push_back('\\');
            c = '\n';
        } else if(c == '/') {
            if(i + 1 < source.length() && source[i + 1] == '/') {
                while(source[i] != '\n' && i < source.length()) {
                    i ++;
                }
                continue;
            }
            else if(i + 1 < source.length() && source[i + 1] == '*') {
                while(i < source.length()) {
                    if(source[i] == '*') {
                        if(i + 1 < source.length()) {
                            if(source[i + 1] == '/') {
                                i++;
                                break;
                            }
                        }
                    }
                    i ++;
                }
                continue;
            }

        }

        sc.push_back(c);
    }

    std::string sourceString = std::string("print(JSON.stringify(Reflect.parse('") + sc + "')))";

    FILE *fp;

    fp = fopen("/tmp/input.js", "w");
    fputs(sourceString.c_str(), fp);
    fflush(fp);
    fclose(fp);

    char path[1035];

    fp = popen("./mozjs /tmp/input.js", "r");
    if (fp == NULL) {
        printf("Failed to run command\n" );
        exit(1);
    }

    std::string outputString;
    while (fgets(path, sizeof(path)-1, fp) != NULL) {
        outputString += path;
    }

    pclose(fp);

    ESString output = outputString.data();
    //output.show();

    rapidjson::GenericDocument<rapidjson::UTF16<>> jsonDocument;
    rapidjson::GenericStringStream<rapidjson::UTF16<>> stringStream(output.data());
    jsonDocument.ParseStream(stringStream);

    //READ SAMPLE
    //std::wstring type = jsonDocument[L"type"].GetString();
    //wprintf(L"%ls\n",type.data());

    //TODO move these strings into elsewhere
    ESString astTypeProgram(L"Program");
    ESString astTypeVariableDeclaration(L"VariableDeclaration");
    ESString astTypeExpressionStatement(L"ExpressionStatement");
    ESString astTypeVariableDeclarator(L"VariableDeclarator");
    ESString astTypeIdentifier(L"Identifier");
    ESString astAssignmentExpression(L"AssignmentExpression");
    ESString astLiteral(L"Literal");

    StatementNodeVector body;
    std::function<Node *(rapidjson::GenericValue<rapidjson::UTF16<>>& value)> fn;
    fn = [&](rapidjson::GenericValue<rapidjson::UTF16<>>& value) -> Node* {
        Node* parsedNode = NULL;
        ESString type(value[L"type"].GetString());
        if(type == astTypeProgram) {
            rapidjson::GenericValue<rapidjson::UTF16<>>& children = value[L"body"];
            for (rapidjson::SizeType i = 0; i < children.Size(); i++) {
                Node* n = fn(children[i]);
                if (n != NULL) {
                	body.push_back(n);
                  }
            }
            parsedNode = new ProgramNode(std::move(body));
        } else if(type == astTypeVariableDeclaration) {
            rapidjson::GenericValue<rapidjson::UTF16<>>& children = value[L"declarations"];
            VariableDeclaratorVector decl;
            ExpressionNodeVector assi;
            for (rapidjson::SizeType i = 0; i < children.Size(); i++) {
                decl.push_back(fn(children[i]));
                if (children[i][L"init"].GetType() != rapidjson::Type::kNullType) {
                		assi.push_back(new AssignmentExpressionNode(fn(children[i][L"id"]), fn(children[i][L"init"]), AssignmentExpressionNode::AssignmentOperator::Equal));
                  }
             }

            body.insert(body.begin(), new VariableDeclarationNode(std::move(decl)));

            if (assi.size() > 1) {
            		parsedNode = new ExpressionStatementNode(new SequenceExpressionNode(std::move(assi)));
            } else if (assi.size() == 1) {
            		parsedNode = new ExpressionStatementNode(assi[0]);
            } else {
            		return NULL;
              }
        } else if(type == astTypeVariableDeclarator) {
            parsedNode = new VariableDeclaratorNode(fn(value[L"id"]));
        } else if(type == astTypeIdentifier) {
            parsedNode = new IdentifierNode(value[L"name"].GetString());
        } else if(type == astTypeExpressionStatement) {
            Node* node = fn(value[L"expression"]);
            parsedNode = new ExpressionStatementNode(node);
        } else if(type == astAssignmentExpression) {
            parsedNode = new AssignmentExpressionNode(fn(value[L"left"]), fn(value[L"right"]), AssignmentExpressionNode::AssignmentOperator::Equal);
        } else if(type == astLiteral) {
            //TODO parse esvalue better
            if(value[L"value"].IsInt()) {
                parsedNode = new LiteralNode(Smi::fromInt(value[L"value"].GetInt()));
            } else {
                RELEASE_ASSERT_NOT_REACHED();
            }

        }
#ifndef NDEBUG
        if(!parsedNode) {
            type.show();
        }
#endif
        RELEASE_ASSERT(parsedNode);
        return parsedNode;
    };

    return fn(jsonDocument);
}

}
