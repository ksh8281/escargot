#include "Escargot.h"
#include "Environment.h"
#include "GlobalObject.h"
#include "ESValue.h"
#include "ExecutionContext.h"
#include "ast/FunctionNode.h"
#include "vm/ESVMInstance.h"

namespace escargot {

//http://www.ecma-international.org/ecma-262/6.0/index.html#sec-newfunctionenvironment
//$8.1.2.4
LexicalEnvironment* LexicalEnvironment::newFunctionEnvironment(ESFunctionObject* function, const ESValue& newTarget)
{
    ASSERT(newTarget.isUndefined() || (newTarget.isESPointer() && newTarget.asESPointer()->isESObject()));
    FunctionEnvironmentRecord* envRec = new FunctionEnvironmentRecord();

    envRec->m_functionObject = function;
    envRec->m_newTarget = newTarget;

    LexicalEnvironment* env = new LexicalEnvironment(envRec, function->outerEnvironment());
    //TODO
    //If F’s [[ThisMode]] internal slot is lexical, set envRec.[[thisBindingStatus]] to "lexical".
    //[[ThisMode]] internal slot is lexical, set envRec.[[thisBindingStatus]] to "lexical".
    //Let home be the value of F’s [[HomeObject]] internal slot.
    //Set envRec.[[HomeObject]] to home.
    //Set envRec.[[NewTarget]] to newTarget.

    return env;
}

void EnvironmentRecord::createMutableBindingForAST(const InternalAtomicString& atomicName,const InternalString& name,bool canDelete)
{
    if(UNLIKELY(isGlobalEnvironmentRecord())) {
        toGlobalEnvironmentRecord()->createGlobalVarBinding(atomicName, name, canDelete);
        ESVMInstance::currentInstance()->invalidateIdentifierCacheCheckCount();
    } else {
        createMutableBinding(atomicName, name, canDelete);
    }
}

//$8.1.1.4.12
bool GlobalEnvironmentRecord::hasVarDeclaration(const InternalAtomicString& name)
{
    if( std::find(m_varNames.begin(), m_varNames.end(), name) != m_varNames.end() )
        return true;
    return false;
}

//$8.1.1.4.15
bool GlobalEnvironmentRecord::canDeclareGlobalVar(const InternalAtomicString& name)
{
    RELEASE_ASSERT_NOT_REACHED();
    /*
    ESObject* globalObj = m_objectRecord->bindingObject();
    bool hasProperty = globalObj->hasOwnProperty(name);
    if (hasProperty)
        return true;
    else
        return globalObj->isExtensible();
        */
}

//$8.1.1.4.16
bool GlobalEnvironmentRecord::canDeclareGlobalFunction(const InternalAtomicString& name)
{
    RELEASE_ASSERT_NOT_REACHED();
    /*
    ESObject* globalObj = m_objectRecord->bindingObject();
    ESSlot* pd = globalObj->find(name);
    if(pd == NULL)
        return globalObj->isExtensible();

    if (pd->isConfigurable() == true)
        return true;

    // IsDataDescriptor && ..
     */
    
    return false;
}

//$8.1.1.4.17
void GlobalEnvironmentRecord::createGlobalVarBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, bool canDelete)
{
    ESObject* globalObj = m_objectRecord->bindingObject();
    bool hasProperty = globalObj->hasOwnProperty(nonAtomicName);
    bool extensible = globalObj->isExtensible();
    if (!hasProperty && extensible) {
        m_objectRecord->createMutableBinding(name, nonAtomicName, canDelete);
        m_objectRecord->initializeBinding(name, nonAtomicName, ESValue());
    }
    if( std::find(m_varNames.begin(), m_varNames.end(), name) == m_varNames.end() )
        m_varNames.push_back(name);
}

//$8.1.1.4.18
void GlobalEnvironmentRecord::createGlobalFunctionBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, const ESValue& V, bool canDelete) {
    ESObject* globalObj = m_objectRecord->bindingObject();
    globalObj->definePropertyOrThrow(nonAtomicName, true, true, canDelete);
    globalObj->set(nonAtomicName, V, false);
    if( std::find(m_varNames.begin(), m_varNames.end(), name) == m_varNames.end() )
        m_varNames.push_back(name);
}

//$8.1.1.4.11
ESObject* GlobalEnvironmentRecord::getThisBinding() {
    return m_objectRecord->bindingObject();
}

//$8.1.1.4.1
ESSlot* GlobalEnvironmentRecord::hasBinding(const InternalAtomicString& atomicName, const InternalString& name) {
    ESSlot* ret = m_declarativeRecord->hasBinding(atomicName, name);
    if(ret)
        return ret;
    return m_objectRecord->hasBinding(atomicName, name);
}

//$8.1.1.4.2
void GlobalEnvironmentRecord::createMutableBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, bool canDelete) {
    if( m_declarativeRecord->hasBinding(name, nonAtomicName) )
        throw "TypeError";
    m_declarativeRecord->createMutableBinding(name, nonAtomicName, canDelete);
}

//$8.1.1.4.4
void GlobalEnvironmentRecord::initializeBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, const ESValue& V) {
    //FIXME
    if( m_declarativeRecord->hasBinding(name, nonAtomicName) )
        m_declarativeRecord->initializeBinding(name, nonAtomicName, V);
    else {
        ASSERT(m_objectRecord->hasBinding(name, nonAtomicName));
        m_objectRecord->initializeBinding(name, nonAtomicName, V);
    }
}

//$8.1.1.4.6
/*
ESValue GlobalEnvironmentRecord::getBindingValue(const InternalAtomicString& name, bool ignoreReferenceErrorException) {
    //FIXME
    //if( m_declarativeRecord->hasBinding(name) )
    //    return m_declarativeRecord->getBindingValue(name, ignoreReferenceErrorException);
    //else {
    //    return m_objectRecord->getBindingValue(name, ignoreReferenceErrorException);
    //}
    return m_objectRecord->getBindingValue(name, ignoreReferenceErrorException);
}*/

//$8.1.1.4.5
//http://www.ecma-international.org/ecma-262/6.0/index.html#sec-global-environment-records-setmutablebinding-n-v-s
void GlobalEnvironmentRecord::setMutableBinding(const InternalAtomicString& name, const InternalString& nonAtomicName, const ESValue& V, bool S) {
    if( m_declarativeRecord->hasBinding(name, nonAtomicName)) {
        m_declarativeRecord->setMutableBinding(name, nonAtomicName, V, S);
    } else {
        m_objectRecord->setMutableBinding(name, nonAtomicName, V, S);
    }
}

//$8.1.1.2.2
void ObjectEnvironmentRecord::createMutableBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, bool canDelete)
{
    m_bindingObject->definePropertyOrThrow(nonAtomicName, true, true, canDelete);
}

//$8.1.1.2.4
void ObjectEnvironmentRecord::initializeBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, const ESValue& V)
{
    return setMutableBinding(name, nonAtomicName, V, false);
}

//$8.1.1.2.5
void ObjectEnvironmentRecord::setMutableBinding(const InternalAtomicString& name,const InternalString& nonAtomicName, const ESValue& V, bool S)
{
    m_bindingObject->set(name.data(), V, S);
}

//http://www.ecma-international.org/ecma-262/6.0/index.html#sec-bindthisvalue
void FunctionEnvironmentRecord::bindThisValue(const ESValue& V)
{
    ASSERT(m_thisBindingStatus != Initialized);
    if(m_thisBindingStatus == Lexical)
        throw ReferenceError(L"");
    m_thisValue = V;
    m_thisBindingStatus = Initialized;
}

ESObject* FunctionEnvironmentRecord::getThisBinding()
{
    ASSERT(m_thisBindingStatus != Lexical);
    if(m_thisBindingStatus == Uninitialized)
        throw ReferenceError(L"");

    return m_thisValue.asESPointer()->asESObject();
}

}
