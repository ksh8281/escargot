#ifndef ESJITOperations_h
#define ESJITOperations_h

#include "runtime/ESValue.h"
#include "runtime/Operations.h"

namespace escargot {

inline ESValueInDouble plusOp(ESValueInDouble left, ESValueInDouble right)
{
    ESValue leftVal = ESValue::fromRawDouble(left);
    ESValue rightVal = ESValue::fromRawDouble(right);
    ESValueInDouble ret = ESValue::toRawDouble(plusOperation(leftVal, rightVal));
    //printf("plusop %lx = %lx + %lx\n", bitwise_cast<uint64_t>(ret), bitwise_cast<uint64_t>(left), bitwise_cast<uint64_t>(right));
    return ret;
}

inline ESValueInDouble minusOp(ESValueInDouble left, ESValueInDouble right)
{
    ESValue leftVal = ESValue::fromRawDouble(left);
    ESValue rightVal = ESValue::fromRawDouble(right);
    ESValueInDouble ret = ESValue::toRawDouble(minusOperation(leftVal, rightVal));
    //printf("plusop %lx = %lx + %lx\n", bitwise_cast<uint64_t>(ret), bitwise_cast<uint64_t>(left), bitwise_cast<uint64_t>(right));
    return ret;
}

inline ESValueInDouble ESObjectSetOp(ESValueInDouble obj, ESValueInDouble property, ESValueInDouble source)
{
    ESValue objVal = ESValue::fromRawDouble(obj);
    if (objVal.isESPointer()) {
        ESPointer* objP = objVal.asESPointer();
        if (objP->isESArrayObject()) {
            ESArrayObject* arrObj = objP->asESArrayObject();

            ESValue propVal = ESValue::fromRawDouble(property);
            ESValue srcVal = ESValue::fromRawDouble(source);
            arrObj->set(propVal.asInt32(), srcVal);
        }
    }
    return source;
}

inline ESValue* contextResolveBinding(ExecutionContext* context, InternalAtomicString* atomicName, ESString* name)
{
    return context->resolveBinding(*atomicName);
}

inline void objectDefineDataProperty(ESObject* object, ESString* key,
        /*bool isWritable, bool isEnumarable, bool isConfigurable,*/
        ESValueInDouble initial)
{
    ESValue initialVal = ESValue::fromRawDouble(initial);
    object->defineDataProperty(key, /*isWritable, isEnumarable, isConfigurable,*/
            true, true, true, initialVal);
}

inline ESValueInDouble esFunctionObjectCall(ESVMInstance* instance,
        ESValueInDouble callee, ESValueInDouble receiverInput,
        ESValue* arguments, size_t argumentCount, int isNewExpression)
{
    ESValue calleeVal = ESValue::fromRawDouble(callee);
    ESValue receiverInputVal = ESValue::fromRawDouble(receiverInput);
    ESValue ret = ESFunctionObject::call(instance, calleeVal,
            receiverInputVal, arguments, argumentCount, isNewExpression);
    return ESValue::toRawDouble(ret);
}

#if 0
ALWAYS_INLINE ESValueInDouble resolveNonDataProperty(ESObject* object, ESPointer* hiddenClassIdxData)
{
    // printf("[resolveNonDataProperty] (void*)object : %p\n", (void*)object);
    // printf("[resolveNonDataProperty] (void*)hiddenClassIdxData : %p\n", (void*)hiddenClassIdxData);
    return ESValue::toRawDouble(((ESAccessorData *)hiddenClassIdxData)->value(object));
}
#else
ALWAYS_INLINE ESValueInDouble resolveNonDataProperty(ESObject* object, size_t idx)
{
    // printf("[resolveNonDataProperty] (void*)object : %p\n", (void*)object);
#ifdef EJJEONG_MERGING
    return ESValue::toRawDouble(object->readHiddenClass(idx));
#else
    return 0;
#endif
}
#endif

#ifndef NDEBUG
inline void jitLogIntOperation(int arg, const char* msg)
{
    printf("[JIT_LOG] %s : int 0x%x\n", msg, bitwise_cast<unsigned>(arg));
}
inline void jitLogDoubleOperation(ESValueInDouble arg, const char* msg)
{
    printf("[JIT_LOG] %s : double 0x%lx\n", msg, bitwise_cast<uint64_t>(arg));
}
inline void jitLogPointerOperation(void* arg, const char* msg)
{
    printf("[JIT_LOG] %s : pointer 0x%lx\n", msg, bitwise_cast<uint64_t>(arg));
}
inline void jitLogStringOperation(const char* arg, const char* msg)
{
    printf("[JIT_LOG] %s : string %s\n", msg, arg);
}
#endif

}

#endif
