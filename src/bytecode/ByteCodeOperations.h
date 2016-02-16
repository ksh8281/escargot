#ifndef ByteCodeOperations_h
#define ByteCodeOperations_h

#include "runtime/ESValue.h"

namespace escargot {

NEVER_INLINE void throwObjectWriteError();

ALWAYS_INLINE ESValue* getByIdOperation(ESVMInstance* instance, ExecutionContext* ec, GetById* code)
{
    if (LIKELY(code->m_identifierCacheInvalidationCheckCount == instance->identifierCacheInvalidationCheckCount())) {
        ASSERT(ec->resolveBinding(code->m_name) == code->m_cachedSlot);
#ifdef ENABLE_ESJIT
        code->m_profile.addProfile(*code->m_cachedSlot);
#endif
        return code->m_cachedSlot;
    } else {
        ESValue* slot;
        if (code->m_onlySearchGlobal)
            slot = instance->globalObject()->addressOfProperty(code->m_name.string());
        else
            slot = ec->resolveBinding(code->m_name);
        if (LIKELY(slot != NULL)) {
            code->m_cachedSlot = slot;
            code->m_identifierCacheInvalidationCheckCount = instance->identifierCacheInvalidationCheckCount();
#ifdef ENABLE_ESJIT
            code->m_profile.addProfile(*code->m_cachedSlot);
#endif
            return code->m_cachedSlot;
        } else {
            ReferenceError* receiver = ReferenceError::create();
            std::vector<ESValue, gc_allocator<ESValue> > arguments;
            UTF16String err_msg;
            err_msg.append(code->m_name.string()->toNullableUTF16String().m_buffer);
            err_msg.append(u" is not defined");

            // TODO call constructor
            // ESFunctionObject::call(fn, receiver, &arguments[0], arguments.size(), instance);
            receiver->set(strings->message.string(), ESString::create(std::move(err_msg)));
            instance->throwError(receiver);
            RELEASE_ASSERT_NOT_REACHED();
        }
    }
}

NEVER_INLINE void setByIdSlowCase(ESVMInstance* instance, GlobalObject* globalObject, SetById* code, ESValue* value);
NEVER_INLINE ESValue getByGlobalIndexOperationSlowCase(GlobalObject* globalObject, GetByGlobalIndex* code);
ALWAYS_INLINE ESValue getByGlobalIndexOperation(GlobalObject* globalObject, GetByGlobalIndex* code)
{
    ASSERT(code->m_orgOpcode == GetByGlobalIndexOpcode);
    ASSERT(globalObject->hiddenClass()->findProperty(code->m_name) == code->m_index);
    if (UNLIKELY(code->m_index == SIZE_MAX)) {
        getByGlobalIndexOperationSlowCase(globalObject, code);
    }
    return globalObject->hiddenClass()->read(globalObject, globalObject, code->m_name, code->m_index);
}

NEVER_INLINE void setByGlobalIndexOperationSlowCase(GlobalObject* globalObject, SetByGlobalIndex* code, const ESValue& value);
ALWAYS_INLINE void setByGlobalIndexOperation(GlobalObject* globalObject, SetByGlobalIndex* code, const ESValue& value)
{
    ASSERT(globalObject->hiddenClass()->findPropertyCheckDeleted(code->m_name) == code->m_index);
    if (UNLIKELY(code->m_index == SIZE_MAX)) {
        setByGlobalIndexOperationSlowCase(globalObject, code, value);
    } else {
        if (!globalObject->hiddenClass()->write(globalObject, globalObject, code->m_name, code->m_index, value))
            throwObjectWriteError();
    }
}

NEVER_INLINE ESValue getByGlobalIndexOperationWithNoInline(GlobalObject* globalObject, GetByGlobalIndex* code);
NEVER_INLINE void setByGlobalIndexOperationWithNoInline(GlobalObject* globalObject, SetByGlobalIndex* code, const ESValue& value);

NEVER_INLINE ESValue getByIdOperationWithNoInline(ESVMInstance* instance, ExecutionContext* ec, GetById* code);

NEVER_INLINE ESValue plusOperationSlowCase(ESValue* left, ESValue* right);
ALWAYS_INLINE ESValue plusOperation(ESValue* left, ESValue* right)
{
    ESValue ret(ESValue::ESForceUninitialized);
    if (left->isInt32() && right->isInt32()) {
        int32_t a = left->asInt32();
        int32_t b = right->asInt32();
        int32_t c = right->asInt32();
        bool result = ArithmeticOperations<int32_t, int32_t, int32_t>::add(a, b, c);
        if (LIKELY(result)) {
            ret = ESValue(c);
        } else {
            ret = ESValue(ESValue::EncodeAsDouble, (double)a + (double)b);
        }
        return ret;
    } else if (left->isNumber() && right->isNumber()) {
        ret = ESValue(left->asNumber() + right->asNumber());
        return ret;
    } else {
        return plusOperationSlowCase(left, right);
    }

}

ALWAYS_INLINE ESValue minusOperation(ESValue* left, ESValue* right)
{
    // http://www.ecma-international.org/ecma-262/5.1/#sec-11.6.2
    ESValue ret(ESValue::ESForceUninitialized);
    if (left->isInt32() && right->isInt32()) {
        int32_t a = left->asInt32();
        int32_t b = right->asInt32();
        int32_t c = right->asInt32();
        bool result = ArithmeticOperations<int32_t, int32_t, int32_t>::sub(a, b, c);
        if (LIKELY(result)) {
            ret = ESValue(c);
        } else {
            ret = ESValue(ESValue::EncodeAsDouble, (double)a - (double)b);
        }
        return ret;
    } else {
        ret = ESValue(left->toNumber() - right->toNumber());
    }
    return ret;
}

NEVER_INLINE ESValue modOperation(ESValue* left, ESValue* right);

// http://www.ecma-international.org/ecma-262/5.1/#sec-11.8.5
NEVER_INLINE bool abstractRelationalComparisonSlowCase(ESValue* left, ESValue* right, bool leftFirst);
ALWAYS_INLINE bool abstractRelationalComparison(ESValue* left, ESValue* right, bool leftFirst)
{
    // consume very fast case
    if (LIKELY(left->isInt32() && right->isInt32())) {
        return left->asInt32() < right->asInt32();
    }

    if (LIKELY(left->isNumber() && right->isNumber())) {
        return left->asNumber() < right->asNumber();
    }

    return abstractRelationalComparisonSlowCase(left, right, leftFirst);
}

NEVER_INLINE bool abstractRelationalComparisonOrEqualSlowCase(ESValue* left, ESValue* right, bool leftFirst);
ALWAYS_INLINE bool abstractRelationalComparisonOrEqual(ESValue* left, ESValue* right, bool leftFirst)
{
    // consume very fast case
    if (LIKELY(left->isInt32() && right->isInt32())) {
        return left->asInt32() <= right->asInt32();
    }

    if (LIKELY(left->isNumber() && right->isNumber())) {
        return left->asNumber() <= right->asNumber();
    }

    return abstractRelationalComparisonOrEqualSlowCase(left, right, leftFirst);
}

// d = {}. d[0]
NEVER_INLINE ESValue getObjectOperationSlowCase(ESValue* willBeObject, ESValue* property, GlobalObject* globalObject);
ALWAYS_INLINE ESValue getObjectOperation(ESValue* willBeObject, ESValue* property, GlobalObject* globalObject)
{
    ASSERT(!ESVMInstance::currentInstance()->globalObject()->didSomePrototypeObjectDefineIndexedProperty());
    if (LIKELY(willBeObject->isESPointer())) {
        if (LIKELY(willBeObject->asESPointer()->isESArrayObject())) {
            ESArrayObject* arr = willBeObject->asESPointer()->asESArrayObject();
            if (LIKELY(arr->isFastmode())) {
                uint32_t idx = property->toIndex();
                if (LIKELY(idx < arr->length())) {
                    ASSERT(idx != ESValue::ESInvalidIndexValue);
                    const ESValue& v = arr->data()[idx];
                    if (LIKELY(!v.isEmpty())) {
                        return v;
                    } else {
                        return ESValue();
                    }
                }
            }
        } else if (willBeObject->asESPointer()->isESTypedArrayObject()) {
            uint32_t idx = property->toIndex();
            ESTypedArrayObjectWrapper* arr = willBeObject->asESPointer()->asESTypedArrayObjectWrapper();
            return arr->get(idx);
        } else if (willBeObject->asESPointer()->isESString()) {
            uint32_t idx = property->toIndex();
            if (LIKELY(idx < willBeObject->asESString()->length())) {
                char16_t c = willBeObject->asESString()->charAt(idx);
                if (LIKELY(c < ESCARGOT_ASCII_TABLE_MAX)) {
                    return strings->asciiTable[c].string();
                } else {
                    return ESString::create(c);
                }
            }
        }
    }
    return getObjectOperationSlowCase(willBeObject, property, globalObject);
}

// d = {}. d.foo
ALWAYS_INLINE ESValue getObjectPreComputedCaseOperation(ESValue* willBeObject, ESString* keyString, GlobalObject* globalObject
    , ESHiddenClassInlineCache* inlineCache)
{
    ASSERT(!ESVMInstance::currentInstance()->globalObject()->didSomePrototypeObjectDefineIndexedProperty());
    ESObject* obj;
    ESObject* targetObj;
    if (LIKELY(willBeObject->isESPointer())) {
        if (LIKELY(willBeObject->asESPointer()->isESObject())) {
            targetObj = obj = willBeObject->asESPointer()->asESObject();
GetObjectPreComputedCaseInlineCacheOperation:
            unsigned currentCacheIndex = 0;
            const size_t cacheFillCount = inlineCache->m_cache.size();
            for (;currentCacheIndex < cacheFillCount ; currentCacheIndex++) {
                const ESHiddenClassInlineCacheData& data = inlineCache->m_cache[currentCacheIndex];
                const ESHiddenClassChain * const cachedHiddenClassChain = &data.m_cachedhiddenClassChain;
                const size_t& cachedIndex = data.m_cachedIndex;
                const size_t cSiz = cachedHiddenClassChain->size() - 1;
                for (size_t i = 0; i < cSiz; i ++) {
                    if (UNLIKELY((*cachedHiddenClassChain)[i] != obj->hiddenClass())) {
                        goto GetObjecPreComputedCacheMiss;
                    }
                    const ESValue& proto = obj->__proto__();
                    if (LIKELY(proto.isObject())) {
                        obj = proto.asESPointer()->asESObject();
                    } else {
                        goto GetObjecPreComputedCacheMiss;
                    }
                }
                if (LIKELY((*cachedHiddenClassChain)[cSiz] == obj->hiddenClass())) {
                    if (cachedIndex != SIZE_MAX) {
                        return obj->hiddenClass()->read(obj, targetObj, keyString, cachedIndex);
                    } else {
                        return ESValue();
                    }
                } GetObjecPreComputedCacheMiss: { }
            }

            // cache miss.
            inlineCache->m_executeCount++;
            if (inlineCache->m_cache.size() > 3 || inlineCache->m_executeCount <= 3) {
                return willBeObject->toObject()->get(keyString);
            }

            obj = targetObj;
            inlineCache->m_cache.push_back(ESHiddenClassInlineCacheData());
            ASSERT(&inlineCache->m_cache.back() == &inlineCache->m_cache[currentCacheIndex]);
            ESHiddenClassChain* cachedHiddenClassChain = &inlineCache->m_cache[currentCacheIndex].m_cachedhiddenClassChain;
            size_t* cachedHiddenClassIndex = &inlineCache->m_cache[currentCacheIndex].m_cachedIndex;
            while (true) {
                cachedHiddenClassChain->push_back(obj->hiddenClass());
                size_t idx = obj->hiddenClass()->findProperty(keyString);
                if (idx != SIZE_MAX) {
                    *cachedHiddenClassIndex = idx;
                    break;
                }
                const ESValue& proto = obj->__proto__();
                if (proto.isObject()) {
                    obj = proto.asESPointer()->asESObject();
                } else
                    break;
            }

            if (*cachedHiddenClassIndex != SIZE_MAX) {
                return obj->hiddenClass()->read(obj, targetObj, keyString, *cachedHiddenClassIndex);
            } else {
                return ESValue();
            }
        } else {
            ASSERT(willBeObject->asESPointer()->isESString());
            if (*keyString == *strings->length.string()) {
                return ESValue(willBeObject->asESString()->length());
            }
            globalObject->stringObjectProxy()->setStringData(willBeObject->asESString());
            targetObj = obj = globalObject->stringObjectProxy();
            goto GetObjectPreComputedCaseInlineCacheOperation;
        }
    } else {
        // number
        if (willBeObject->isNumber()) {
            globalObject->numberObjectProxy()->setNumberData(willBeObject->asNumber());
            targetObj = obj = globalObject->numberObjectProxy();
            goto GetObjectPreComputedCaseInlineCacheOperation;
        }
        return willBeObject->toObject()->get(keyString);
    }
}

NEVER_INLINE ESValue getObjectPreComputedCaseOperationWithNeverInline(ESValue* willBeObject, ESString* property, GlobalObject* globalObject
    , ESHiddenClassInlineCache* inlineCache);

// d = {}. d[0] = 1
NEVER_INLINE void setObjectOperationSlowCase(ESValue* willBeObject, ESValue* property, const ESValue& value);
NEVER_INLINE void setObjectOperationExpandLengthCase(ESArrayObject* arr, uint32_t idx, const ESValue& value);
ALWAYS_INLINE void setObjectOperation(ESValue* willBeObject, ESValue* property, const ESValue& value)
{
    ASSERT(!ESVMInstance::currentInstance()->globalObject()->didSomePrototypeObjectDefineIndexedProperty());
    if (LIKELY(willBeObject->isESPointer())) {
        if (LIKELY(willBeObject->asESPointer()->isESArrayObject())) {
            ESArrayObject* arr = willBeObject->asESPointer()->asESArrayObject();
            if (LIKELY(arr->isFastmode())) {
                uint32_t idx = property->toIndex();
                if (LIKELY(idx < arr->length())) {
                    ASSERT(idx != ESValue::ESInvalidIndexValue);
                    arr->data()[idx] = value;
                    return;
                } else {
                    if (UNLIKELY(!arr->isExtensible()))
                        return;
                    if (idx != ESValue::ESInvalidIndexValue) {
                        setObjectOperationExpandLengthCase(arr, idx, value);
                        return;
                    }
                }
            }
        } else if (willBeObject->asESPointer()->isESTypedArrayObject()) {
            uint32_t idx = property->toIndex();
            willBeObject->asESPointer()->asESTypedArrayObjectWrapper()->set(idx, value);
            return;
        }
    }
    setObjectOperationSlowCase(willBeObject, property, value);
}

// d = {}. d.foo = 1
NEVER_INLINE void setObjectPreComputedCaseOperationSlowCase(ESValue* willBeObject, ESString* keyString, const ESValue& value);
NEVER_INLINE void setObjectPreComputedCaseOperationWithNeverInline(ESValue* willBeObject, ESString* keyString, const ESValue& value
    , ESHiddenClassChain* cachedHiddenClassChain, size_t* cachedHiddenClassIndex, ESHiddenClass** hiddenClassWillBe);
ALWAYS_INLINE void setObjectPreComputedCaseOperation(ESValue* willBeObject, ESString* keyString, const ESValue& value
    , ESHiddenClassChain* cachedHiddenClassChain, size_t* cachedHiddenClassIndex, ESHiddenClass** hiddenClassWillBe)
{
    ASSERT(!ESVMInstance::currentInstance()->globalObject()->didSomePrototypeObjectDefineIndexedProperty());

    if (LIKELY(willBeObject->isESPointer())) {
        if (LIKELY(willBeObject->asESPointer()->isESObject())) {
            ESObject* obj = willBeObject->asESPointer()->asESObject();
            if (*cachedHiddenClassIndex != SIZE_MAX && (*cachedHiddenClassChain)[0] == obj->hiddenClass()) {
                ASSERT((*cachedHiddenClassChain).size() == 1);
                // cache hit!
                if (!obj->hiddenClass()->write(obj, obj, keyString, *cachedHiddenClassIndex, value)) {
                    throwObjectWriteError();
                }
                return;
            } else if (*hiddenClassWillBe) {
                int cSiz = cachedHiddenClassChain->size();
                bool miss = false;
                for (int i = 0; i < cSiz - 1; i ++) {
                    if ((*cachedHiddenClassChain)[i] != obj->hiddenClass()) {
                        miss = true;
                        break;
                    } else {
                        ESValue o = obj->__proto__();
                        if (!o.isObject()) {
                            miss = true;
                            break;
                        }
                        obj = o.asESPointer()->asESObject();
                    }
                }
                if (!miss) {
                    if ((*cachedHiddenClassChain)[cSiz - 1] == obj->hiddenClass()) {
                        // cache hit!
                        obj = willBeObject->asESPointer()->asESObject();
                        obj->m_hiddenClassData.push_back(value);
                        obj->m_hiddenClass = *hiddenClassWillBe;
                        return;
                    }
                }
            }

            // cache miss
            *cachedHiddenClassIndex = SIZE_MAX;
            *hiddenClassWillBe = NULL;
            cachedHiddenClassChain->clear();

            obj = willBeObject->asESPointer()->asESObject();
            size_t idx = obj->hiddenClass()->findProperty(keyString);
            if (idx != SIZE_MAX) {
                // own property
                *cachedHiddenClassIndex = idx;
                cachedHiddenClassChain->push_back(obj->hiddenClass());

                if (!obj->hiddenClass()->write(obj, obj, keyString, idx, value))
                    throwObjectWriteError();
            } else {
                cachedHiddenClassChain->push_back(obj->hiddenClass());
                ESValue proto = obj->__proto__();
                while (proto.isObject()) {
                    obj = proto.asESPointer()->asESObject();
                    cachedHiddenClassChain->push_back(obj->hiddenClass());

                    size_t idx = obj->hiddenClass()->findProperty(keyString);
                    if (idx != SIZE_MAX) {
                        // http://www.ecma-international.org/ecma-262/5.1/#sec-8.12.5
                        // If IsAccessorDescriptor(desc) is true, then
                        // Let setter be desc.[[Set]] which cannot be undefined.
                        // Call the [[Call]] internal method of setter providing O as the this value and providing V as the sole argument.
                        if (!obj->hiddenClass()->propertyInfo(idx).m_flags.m_isDataProperty) {
                            ESPropertyAccessorData* data = obj->accessorData(idx);
                            if (data->isAccessorDescriptor()) {
                                *cachedHiddenClassIndex = SIZE_MAX;
                                *hiddenClassWillBe = NULL;
                                cachedHiddenClassChain->clear();
                                if (data->getJSSetter()) {
                                    ESValue args[] = {value};
                                    ESFunctionObject::call(ESVMInstance::currentInstance(), data->getJSSetter(), willBeObject->asESPointer()->asESObject(), args, 1, false);
                                    return;
                                } else {
                                    throwObjectWriteError();
                                    return;
                                }
                            } else if (data->getNativeSetter()) {
                                *cachedHiddenClassIndex = SIZE_MAX;
                                *hiddenClassWillBe = NULL;
                                cachedHiddenClassChain->clear();
                                data->setValue(obj, willBeObject->asESPointer()->asESObject(), keyString, value);
                                return;
                            }
                        }

                        if (!obj->hiddenClass()->propertyInfo(idx).m_flags.m_isWritable) {
                            *cachedHiddenClassIndex = SIZE_MAX;
                            *hiddenClassWillBe = NULL;
                            cachedHiddenClassChain->clear();
                            throwObjectWriteError();
                            return;
                        }
                    }
                    proto = obj->__proto__();
                }

                ASSERT(!willBeObject->asESPointer()->asESObject()->hasOwnProperty(keyString));
                bool res = willBeObject->asESPointer()->asESObject()->defineDataProperty(keyString, true, true, true, value);
                if (!res)
                    throwObjectWriteError();

                // only cache vector mode object.
                if (res && willBeObject->asESPointer()->asESObject()->hiddenClass()->isVectorMode()) {
                    *hiddenClassWillBe = willBeObject->asESPointer()->asESObject()->hiddenClass();
                } else {
                    cachedHiddenClassChain->clear();
                    *hiddenClassWillBe = NULL;
                }
            }
            return;
        }
    }
    setObjectPreComputedCaseOperationSlowCase(willBeObject, keyString, value);
}

NEVER_INLINE ESValue getObjectOperationSlowMode(ESValue* willBeObject, ESValue* property, GlobalObject* globalObject);
NEVER_INLINE void setObjectOperationSlowMode(ESValue* willBeObject, ESValue* property, const ESValue& value);

NEVER_INLINE bool instanceOfOperation(ESValue* lval, ESValue* rval);
NEVER_INLINE ESValue typeOfOperation(ESValue* v);
NEVER_INLINE ESValue newOperation(ESVMInstance* instance, GlobalObject* globalObject, ESValue fn, ESValue* arguments, size_t argc);
NEVER_INLINE bool inOperation(ESValue* obj, ESValue* key);
NEVER_INLINE void tryOperation(ESVMInstance* instance, CodeBlock* codeBlock, char* codeBuffer, ExecutionContext* ec, size_t programCounter, Try* code, ESValue* stackStorage, ESValueVector* heapStorage);
NEVER_INLINE void tryOperationThrowCase(const ESValue& err, LexicalEnvironment* oldEnv, ExecutionContext* backupedEC, ESVMInstance* instance, CodeBlock* codeBlock, char* codeBuffer, ExecutionContext* ec, size_t programCounter, Try* code, ESValue* stackStorage, ESValueVector* heapStorage);
NEVER_INLINE EnumerateObjectData* executeEnumerateObject(ESObject* obj);

}
#endif
