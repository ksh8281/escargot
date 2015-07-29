#include "Escargot.h"
#include "MemberExpressionNode.h"

#include "IdentifierNode.h"
#include "vm/ESVMInstance.h"
#include "runtime/ExecutionContext.h"
#include "runtime/Environment.h"


namespace escargot {

ESValue* MemberExpressionNode::execute(ESVMInstance* instance)
{
    ESValue* value = m_object->execute(instance)->ensureValue();
    //TODO string,number-> stringObject, numberObject;
    if(value->isHeapObject() && value->toHeapObject()->isPString()) {
        JSString* stringObject = JSString::create(value->toHeapObject()->toPString()->string());
        stringObject->set__proto__(instance->globalObject()->stringPrototype());
        stringObject->setConstructor(instance->globalObject()->string());
        value = stringObject;
    }

    if(value->isHeapObject() && value->toHeapObject()->isJSObject()) {
        JSObject* obj = value->toHeapObject()->toJSObject();
        ESAtomicString propertyName;
        ESValue* propertyVal = NULL;
        if(!m_computed && m_property->type() == NodeType::Identifier) {
            propertyName = ((IdentifierNode*)m_property)->name();
        } else {
            ESValue* tmpVal = m_property->execute(instance)->ensureValue();
            if(m_computed && obj->toHeapObject()->isJSArray() && tmpVal->isSmi())
                propertyVal = tmpVal;
            propertyName = ESAtomicString(tmpVal->toESString().data());
        }

        instance->currentExecutionContext()->setLastJSObjectMetInMemberExpressionNode(obj->toHeapObject()->toJSObject(),
                propertyName, propertyVal);

        JSSlot* slot;
        if (obj->isJSArray() && propertyVal != NULL)
            slot = obj->toJSArray()->find(propertyVal);
        else
            slot = obj->find(propertyName);

        if(slot) {
            return slot;
        } else {
            ESValue* prototype = obj->__proto__();
            while(prototype && prototype->isHeapObject() && prototype->toHeapObject()->isJSObject()) {
                ::escargot::JSObject* obj = prototype->toHeapObject()->toJSObject();
                JSSlot* s = obj->find(propertyName);
                if(s)
                    return s;
                prototype = obj->__proto__();
            }
        }
        return esUndefined;
    } else {
        throw TypeError();
    }
    return esUndefined;
}
}
