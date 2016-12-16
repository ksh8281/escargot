#include "Escargot.h"
#include "ByteCode.h"
#include "ByteCodeInterpreter.h"
#include "runtime/Environment.h"
#include "runtime/EnvironmentRecord.h"
#include "runtime/FunctionObject.h"
#include "runtime/Context.h"
#include "runtime/SandBox.h"
#include "runtime/GlobalObject.h"
#include "runtime/ErrorObject.h"
#include "runtime/ArrayObject.h"
#include "parser/ScriptParser.h"
#include "../third_party/checked_arithmetic/CheckedArithmetic.h"

namespace Escargot {

NEVER_INLINE void registerOpcode(Opcode opcode, void* opcodeAddress)
{
    static std::unordered_set<void*> labelAddressChecker;
    if (labelAddressChecker.find(opcodeAddress) != labelAddressChecker.end()) {
        ESCARGOT_LOG_ERROR("%d\n", opcode);
        RELEASE_ASSERT_NOT_REACHED();
    }

    static int cnt = 0;
    g_opcodeTable.m_table[opcode] = opcodeAddress;
    g_opcodeTable.m_reverseTable[labelAddressChecker.size()] = std::make_pair(opcodeAddress, opcode);
    labelAddressChecker.insert(opcodeAddress);

    if (opcode == EndOpcode) {
        labelAddressChecker.clear();
    }
}

template <typename CodeType>
ALWAYS_INLINE void executeNextCode(size_t& programCounter)
{
    programCounter += sizeof(CodeType);
}

ALWAYS_INLINE size_t jumpTo(char* codeBuffer, const size_t& jumpPosition)
{
    return (size_t)&codeBuffer[jumpPosition];
}

ALWAYS_INLINE size_t resolveProgramCounter(char* codeBuffer, const size_t& programCounter)
{
    return programCounter - (size_t)codeBuffer;
}

class ECResetter {
public:
    ECResetter(size_t** addr)
    {
        m_addr = addr;
    }
    ~ECResetter()
    {
        m_addr = nullptr;
    }

protected:
    size_t** m_addr;
};

void ByteCodeInterpreter::interpret(ExecutionState& state, CodeBlock* codeBlock, size_t programCounter)
{
    if (UNLIKELY(codeBlock == nullptr)) {
        goto FillOpcodeTable;
    }
    {
        ASSERT(codeBlock->byteCodeBlock() != nullptr);
        ByteCodeBlock* byteCodeBlock = codeBlock->byteCodeBlock();
        ExecutionContext* ec = state.executionContext();
        LexicalEnvironment* env = ec->lexicalEnvironment();
        EnvironmentRecord* record = env->record();
        Value thisValue(Value::EmptyValue);
        Value* registerFile = ALLOCA(byteCodeBlock->m_requiredRegisterFileSizeInValueSize * sizeof(Value), Value, state);
        Value* stackStorage = ec->stackStorage();
        char* codeBuffer = byteCodeBlock->m_code.data();
        ec->m_programCounter = &programCounter;
        ECResetter resetPC(&ec->m_programCounter);
        programCounter = (size_t)(&codeBuffer[programCounter]);
        ByteCode* currentCode;

#define NEXT_INSTRUCTION() \
    goto NextInstrucution;
    /*
        currentCode = (ByteCode *)programCounter; \
        ASSERT(((size_t)currentCode % sizeof(size_t)) == 0); \
        goto *(currentCode->m_opcodeInAddress);
    */
    NextInstrucution:
        currentCode = (ByteCode*)programCounter;
        ASSERT(((size_t)currentCode % sizeof(size_t)) == 0);
        goto*(currentCode->m_opcodeInAddress);

    LoadLiteralOpcodeLbl : {
        LoadLiteral* code = (LoadLiteral*)currentCode;
        registerFile[code->m_registerIndex] = code->m_value;
        executeNextCode<LoadLiteral>(programCounter);
        NEXT_INSTRUCTION();
    }

    MoveOpcodeLbl : {
        Move* code = (Move*)currentCode;
        registerFile[code->m_registerIndex1] = registerFile[code->m_registerIndex0];
        executeNextCode<Move>(programCounter);
        NEXT_INSTRUCTION();
    }

    LoadByStackIndexOpcodeLbl : {
        LoadByStackIndex* code = (LoadByStackIndex*)currentCode;
        registerFile[code->m_registerIndex] = stackStorage[code->m_index];
        executeNextCode<LoadByStackIndex>(programCounter);
        NEXT_INSTRUCTION();
    }

    StoreByStackIndexOpcodeLbl : {
        StoreByStackIndex* code = (StoreByStackIndex*)currentCode;
        stackStorage[code->m_index] = registerFile[code->m_registerIndex];
        executeNextCode<StoreByStackIndex>(programCounter);
        NEXT_INSTRUCTION();
    }

    LoadByHeapIndexOpcodeLbl : {
        LoadByHeapIndex* code = (LoadByHeapIndex*)currentCode;
        LexicalEnvironment* upperEnv = env;
        for (size_t i = 0; i < code->m_upperIndex; i++) {
            upperEnv = upperEnv->outerEnvironment();
        }
        FunctionEnvironmentRecord* record = upperEnv->record()->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord();
        ASSERT(record->isFunctionEnvironmentRecordOnHeap());
        registerFile[code->m_registerIndex] = ((FunctionEnvironmentRecordOnHeap*)record)->m_heapStorage[code->m_index];
        executeNextCode<LoadByHeapIndex>(programCounter);
        NEXT_INSTRUCTION();
    }

    StoreByHeapIndexOpcodeLbl : {
        StoreByHeapIndex* code = (StoreByHeapIndex*)currentCode;
        LexicalEnvironment* upperEnv = env;
        for (size_t i = 0; i < code->m_upperIndex; i++) {
            upperEnv = upperEnv->outerEnvironment();
        }
        FunctionEnvironmentRecord* record = upperEnv->record()->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord();
        ASSERT(record->isFunctionEnvironmentRecordOnHeap());
        ((FunctionEnvironmentRecordOnHeap*)record)->m_heapStorage[code->m_index] = registerFile[code->m_registerIndex];
        executeNextCode<StoreByHeapIndex>(programCounter);
        NEXT_INSTRUCTION();
    }

    LoadByGlobalNameOpcodeLbl : {
        LoadByGlobalName* code = (LoadByGlobalName*)currentCode;
        GlobalObject* g = state.context()->globalObject();
        // check cache
        if (UNLIKELY(!g->hasPropertyOnIndex(state, code->m_name, code->m_cacheIndex))) {
            // fill cache
            code->m_cacheIndex = g->findPropertyIndex(state, code->m_name);
            ASSERT(code->m_cacheIndex != SIZE_MAX);
        }
        registerFile[code->m_registerIndex] = g->getPropertyOnIndex(state, code->m_cacheIndex);
        executeNextCode<LoadByGlobalName>(programCounter);
        NEXT_INSTRUCTION();
    }

    StoreByGlobalNameOpcodeLbl : {
        StoreByGlobalName* code = (StoreByGlobalName*)currentCode;
        GlobalObject* g = state.context()->globalObject();
        // check cache
        if (UNLIKELY(!g->hasPropertyOnIndex(state, code->m_name, code->m_cacheIndex))) {
            // fill cache
            code->m_cacheIndex = g->findPropertyIndex(state, code->m_name);
            ASSERT(code->m_cacheIndex != SIZE_MAX);
        }
        if (UNLIKELY(!g->setPropertyOnIndex(state, code->m_cacheIndex, registerFile[code->m_registerIndex]))) {
            if (state.inStrictMode()) {
                // TODO throw execption
                RELEASE_ASSERT_NOT_REACHED();
            }
        }
        executeNextCode<StoreByGlobalName>(programCounter);
        NEXT_INSTRUCTION();
    }

    LoadByNameOpcodeLbl : {
        LoadByName* code = (LoadByName*)currentCode;
        registerFile[code->m_registerIndex] = loadByName(state, env, code->m_name);
        executeNextCode<LoadByName>(programCounter);
        NEXT_INSTRUCTION();
    }

    StoreByNameOpcodeLbl : {
        StoreByName* code = (StoreByName*)currentCode;
        storeByName(state, env, code->m_name, registerFile[code->m_registerIndex]);
        executeNextCode<StoreByName>(programCounter);
        NEXT_INSTRUCTION();
    }

    DeclareVarVariableOpcodeLbl : {
        DeclareVarVariable* code = (DeclareVarVariable*)currentCode;
        record->createMutableBinding(state, code->m_name, false);
        executeNextCode<DeclareVarVariable>(programCounter);
        NEXT_INSTRUCTION();
    }

    DeclareFunctionExpressionOpcodeLbl : {
        DeclareFunctionExpression* code = (DeclareFunctionExpression*)currentCode;
        registerFile[code->m_registerIndex] = new FunctionObject(state, code->m_codeBlock, env);
        executeNextCode<DeclareFunctionExpression>(programCounter);
        NEXT_INSTRUCTION();
    }

    GetThisOpcodeLbl : {
        GetThis* code = (GetThis*)currentCode;
        if (UNLIKELY(thisValue.isEmpty())) {
            thisValue = env->getThisBinding();
        }
        registerFile[code->m_registerIndex] = thisValue;
        executeNextCode<GetThis>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryPlusOpcodeLbl : {
        BinaryPlus* code = (BinaryPlus*)currentCode;
        const Value& v0 = registerFile[code->m_srcIndex0];
        const Value& v1 = registerFile[code->m_srcIndex1];
        Value ret(Value::ForceUninitialized);
        if (v0.isInt32() && v1.isInt32()) {
            int32_t a = v0.asInt32();
            int32_t b = v1.asInt32();
            int32_t c;
            bool result = ArithmeticOperations<int32_t, int32_t, int32_t>::add(a, b, c);
            if (LIKELY(result)) {
                ret = Value(c);
            } else {
                ret = Value(Value::EncodeAsDouble, (double)a + (double)b);
            }
        } else if (v0.isNumber() && v1.isNumber()) {
            ret = Value(v0.asNumber() + v1.asNumber());
        } else {
            ret = plusSlowCase(state, v0, v1);
        }
        registerFile[code->m_srcIndex0] = ret;
        executeNextCode<BinaryPlus>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryMinusOpcodeLbl : {
        BinaryMinus* code = (BinaryMinus*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        Value ret(Value::ForceUninitialized);
        if (left.isInt32() && right.isInt32()) {
            int32_t a = left.asInt32();
            int32_t b = right.asInt32();
            int32_t c;
            bool result = ArithmeticOperations<int32_t, int32_t, int32_t>::sub(a, b, c);
            if (LIKELY(result)) {
                ret = Value(c);
            } else {
                ret = Value(Value::EncodeAsDouble, (double)a - (double)b);
            }
        } else {
            ret = Value(left.toNumber(state) - right.toNumber(state));
        }
        registerFile[code->m_srcIndex0] = ret;
        executeNextCode<BinaryMinus>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryMultiplyOpcodeLbl : {
        BinaryMultiply* code = (BinaryMultiply*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        Value ret(Value::ForceUninitialized);
        if (left.isInt32() && right.isInt32()) {
            int32_t a = left.asInt32();
            int32_t b = right.asInt32();
            if ((!a || !b) && (a >> 31 || b >> 31)) { // -1 * 0 should be treated as -0, not +0
                ret = Value(left.toNumber(state) * right.toNumber(state));
            } else {
                int32_t c = right.asInt32();
                bool result = ArithmeticOperations<int32_t, int32_t, int32_t>::multiply(a, b, c);
                if (LIKELY(result)) {
                    ret = Value(c);
                } else {
                    ret = Value(Value::EncodeAsDouble, left.toNumber(state) * right.toNumber(state));
                }
            }
        } else {
            ret = Value(Value::EncodeAsDouble, left.toNumber(state) * right.toNumber(state));
        }
        registerFile[code->m_srcIndex0] = ret;
        executeNextCode<BinaryMultiply>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryDivisionOpcodeLbl : {
        BinaryDivision* code = (BinaryDivision*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.toNumber(state) / right.toNumber(state));
        executeNextCode<BinaryDivision>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryModOpcodeLbl : {
        BinaryMod* code = (BinaryMod*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = modOperation(state, left, right);
        executeNextCode<BinaryMod>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryEqualOpcodeLbl : {
        BinaryEqual* code = (BinaryEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.abstractEqualsTo(state, right));
        executeNextCode<BinaryEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryNotEqualOpcodeLbl : {
        BinaryNotEqual* code = (BinaryNotEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(!left.abstractEqualsTo(state, right));
        executeNextCode<BinaryNotEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryStrictEqualOpcodeLbl : {
        BinaryStrictEqual* code = (BinaryStrictEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.equalsTo(state, right));
        executeNextCode<BinaryStrictEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryNotStrictEqualOpcodeLbl : {
        BinaryNotStrictEqual* code = (BinaryNotStrictEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(!left.equalsTo(state, right));
        executeNextCode<BinaryNotStrictEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryLessThanOpcodeLbl : {
        BinaryLessThan* code = (BinaryLessThan*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(abstractRelationalComparison(state, left, right, true));
        executeNextCode<BinaryLessThan>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryLessThanOrEqualOpcodeLbl : {
        BinaryLessThanOrEqual* code = (BinaryLessThanOrEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(abstractRelationalComparisonOrEqual(state, left, right, true));
        executeNextCode<BinaryLessThanOrEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryGreaterThanOpcodeLbl : {
        BinaryGreaterThan* code = (BinaryGreaterThan*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(abstractRelationalComparison(state, right, left, false));
        executeNextCode<BinaryGreaterThan>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryGreaterThanOrEqualOpcodeLbl : {
        BinaryGreaterThanOrEqual* code = (BinaryGreaterThanOrEqual*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(abstractRelationalComparisonOrEqual(state, right, left, false));
        executeNextCode<BinaryGreaterThanOrEqual>(programCounter);
        NEXT_INSTRUCTION();
    }

    IncrementOpcodeLbl : {
        Increment* code = (Increment*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        Value ret(Value::ForceUninitialized);
        if (LIKELY(val.isInt32())) {
            int32_t a = val.asInt32();
            if (UNLIKELY(a == std::numeric_limits<int32_t>::max()))
                ret = Value(Value::EncodeAsDouble, ((double)a) + 1);
            else
                ret = Value(a + 1);
        } else {
            ret = Value(val.toNumber(state) + 1);
        }
        registerFile[code->m_registerIndex] = ret;
        executeNextCode<Increment>(programCounter);
        NEXT_INSTRUCTION();
    }

    DecrementOpcodeLbl : {
        Decrement* code = (Decrement*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        Value ret(Value::ForceUninitialized);
        if (LIKELY(val.isInt32())) {
            int32_t a = val.asInt32();
            if (UNLIKELY(a == std::numeric_limits<int32_t>::min()))
                ret = Value(Value::EncodeAsDouble, ((double)a) - 1);
            else
                ret = Value(a - 1);
        } else {
            ret = Value(val.toNumber(state) - 1);
        }
        registerFile[code->m_registerIndex] = ret;
        executeNextCode<Decrement>(programCounter);
        NEXT_INSTRUCTION();
    }

    UnaryMinusOpcodeLbl : {
        UnaryMinus* code = (UnaryMinus*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        registerFile[code->m_registerIndex] = Value(-val.toNumber(state));
        executeNextCode<UnaryMinus>(programCounter);
        NEXT_INSTRUCTION();
    }

    UnaryNotOpcodeLbl : {
        UnaryNot* code = (UnaryNot*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        registerFile[code->m_registerIndex] = Value(!val.toBoolean(state));
        executeNextCode<UnaryNot>(programCounter);
        NEXT_INSTRUCTION();
    }

    GetObjectOpcodeLbl : {
        GetObject* code = (GetObject*)currentCode;
        const Value& willBeObject = registerFile[code->m_objectRegisterIndex];
        const Value& property = registerFile[code->m_objectRegisterIndex + 1];
        if (LIKELY(willBeObject.isPointerValue() && willBeObject.asPointerValue()->isArrayObject())) {
            ArrayObject* arr = willBeObject.asObject()->asArrayObject();
            auto result = arr->getFastModeValue(state, ObjectPropertyName(state, property));
            if (LIKELY(result.hasValue())) {
                registerFile[code->m_objectRegisterIndex] = result.value();
                executeNextCode<GetObject>(programCounter);
                NEXT_INSTRUCTION();
            }
        }
        registerFile[code->m_objectRegisterIndex] = willBeObject.toObject(state)->get(state, ObjectPropertyName(state, property)).value();
        executeNextCode<GetObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    SetObjectOpcodeLbl : {
        SetObject* code = (SetObject*)currentCode;
        const Value& willBeObject = registerFile[code->m_objectRegisterIndex];
        const Value& property = registerFile[code->m_propertyRegisterIndex];
        if (LIKELY(willBeObject.isPointerValue() && willBeObject.asPointerValue()->isArrayObject())) {
            ArrayObject* arr = willBeObject.asObject()->asArrayObject();
            if (LIKELY(arr->setFastModeValue(state, ObjectPropertyName(state, property), ObjectPropertyDescriptorForDefineOwnProperty(registerFile[code->m_loadRegisterIndex], ObjectPropertyDescriptorForDefineOwnProperty::AllPresent)))) {
                executeNextCode<SetObject>(programCounter);
                NEXT_INSTRUCTION();
            }
        }
        Object* obj = willBeObject.toObject(state);
        obj->setThrowsExceptionWhenStrictMode(state, ObjectPropertyName(state, property), registerFile[code->m_loadRegisterIndex], obj);
        executeNextCode<SetObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    GetObjectPreComputedCaseOpcodeLbl : {
        GetObjectPreComputedCase* code = (GetObjectPreComputedCase*)currentCode;
        const Value& willBeObject = registerFile[code->m_objectRegisterIndex];
        registerFile[code->m_objectRegisterIndex] = getObjectPrecomputedCaseOperation(state, willBeObject.toObject(state), code->m_propertyName, code->m_inlineCache).second;
        executeNextCode<GetObjectPreComputedCase>(programCounter);
        NEXT_INSTRUCTION();
    }

    SetObjectPreComputedCaseOpcodeLbl : {
        SetObjectPreComputedCase* code = (SetObjectPreComputedCase*)currentCode;
        const Value& willBeObject = registerFile[code->m_objectRegisterIndex];
        setObjectPreComputedCaseOperation(state, willBeObject.toObject(state), code->m_propertyName, registerFile[code->m_loadRegisterIndex], code->m_inlineCache);
        executeNextCode<SetObjectPreComputedCase>(programCounter);
        NEXT_INSTRUCTION();
    }

    GetGlobalObjectOpcodeLbl : {
        GetGlobalObject* code = (GetGlobalObject*)currentCode;
        auto result = getObjectPrecomputedCaseOperation(state, state.context()->globalObject(), code->m_propertyName, code->m_inlineCache);
        if (UNLIKELY(!result.first)) {
            ErrorObject::throwBuiltinError(state, ErrorObject::ReferenceError, code->m_propertyName.string(), false, String::emptyString, errorMessage_IsNotDefined);
        }
        registerFile[code->m_registerIndex] = result.second;
        executeNextCode<GetGlobalObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    SetGlobalObjectOpcodeLbl : {
        SetGlobalObject* code = (SetGlobalObject*)currentCode;
        setObjectPreComputedCaseOperation(state, state.context()->globalObject(), code->m_propertyName, registerFile[code->m_registerIndex], code->m_inlineCache);
        executeNextCode<SetGlobalObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    JumpOpcodeLbl : {
        Jump* code = (Jump*)currentCode;
        ASSERT(code->m_jumpPosition != SIZE_MAX);
        programCounter = jumpTo(codeBuffer, code->m_jumpPosition);
        NEXT_INSTRUCTION();
    }

    JumpIfTrueOpcodeLbl : {
        JumpIfTrue* code = (JumpIfTrue*)currentCode;
        ASSERT(code->m_jumpPosition != SIZE_MAX);
        if (registerFile[code->m_registerIndex].toBoolean(state)) {
            programCounter = jumpTo(codeBuffer, code->m_jumpPosition);
        } else {
            executeNextCode<JumpIfTrue>(programCounter);
        }
        NEXT_INSTRUCTION();
    }

    JumpIfFalseOpcodeLbl : {
        JumpIfFalse* code = (JumpIfFalse*)currentCode;
        ASSERT(code->m_jumpPosition != SIZE_MAX);
        if (!registerFile[code->m_registerIndex].toBoolean(state)) {
            programCounter = jumpTo(codeBuffer, code->m_jumpPosition);
        } else {
            executeNextCode<JumpIfFalse>(programCounter);
        }
        NEXT_INSTRUCTION();
    }

    CallFunctionOpcodeLbl : {
        CallFunction* code = (CallFunction*)currentCode;
        const Value& receiver = registerFile[code->m_registerIndex];
        const Value& callee = registerFile[code->m_registerIndex + 1];
        registerFile[code->m_registerIndex] = FunctionObject::call(callee, state, receiver, code->m_argumentCount, &registerFile[code->m_registerIndex + 2]);
        executeNextCode<CallFunction>(programCounter);
        NEXT_INSTRUCTION();
    }

    DeclareFunctionDeclarationOpcodeLbl : {
        DeclareFunctionDeclaration* code = (DeclareFunctionDeclaration*)currentCode;
        registerFile[0] = new FunctionObject(state, code->m_codeBlock, env);
        executeNextCode<DeclareFunctionDeclaration>(programCounter);
        NEXT_INSTRUCTION();
    }

    ReturnFunctionOpcodeLbl : {
        ReturnFunction* code = (ReturnFunction*)currentCode;
        if (code->m_registerIndex != SIZE_MAX)
            *state.exeuctionResult() = registerFile[code->m_registerIndex];
        else
            *state.exeuctionResult() = Value();
        return;
    }

    BinaryBitwiseAndOpcodeLbl : {
        BinaryBitwiseAnd* code = (BinaryBitwiseAnd*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.toInt32(state) & right.toInt32(state));
        executeNextCode<BinaryBitwiseAnd>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryBitwiseOrOpcodeLbl : {
        BinaryBitwiseOr* code = (BinaryBitwiseOr*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.toInt32(state) | right.toInt32(state));
        executeNextCode<BinaryBitwiseOr>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryBitwiseXorOpcodeLbl : {
        BinaryBitwiseXor* code = (BinaryBitwiseXor*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        registerFile[code->m_srcIndex0] = Value(left.toInt32(state) ^ right.toInt32(state));
        executeNextCode<BinaryBitwiseXor>(programCounter);
        NEXT_INSTRUCTION();
    }

    BinaryLeftShiftOpcodeLbl : {
        BinaryLeftShift* code = (BinaryLeftShift*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        int32_t lnum = left.toInt32(state);
        int32_t rnum = right.toInt32(state);
        lnum <<= ((unsigned int)rnum) & 0x1F;
        registerFile[code->m_srcIndex0] = Value(lnum);
        executeNextCode<BinaryLeftShift>(programCounter);
        NEXT_INSTRUCTION();
    }


    BinarySignedRightShiftOpcodeLbl : {
        BinarySignedRightShift* code = (BinarySignedRightShift*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        int32_t lnum = left.toInt32(state);
        int32_t rnum = right.toInt32(state);
        lnum >>= ((unsigned int)rnum) & 0x1F;
        registerFile[code->m_srcIndex0] = Value(lnum);
        executeNextCode<BinarySignedRightShift>(programCounter);
        NEXT_INSTRUCTION();
    }


    BinaryUnsignedRightShiftOpcodeLbl : {
        BinaryUnsignedRightShift* code = (BinaryUnsignedRightShift*)currentCode;
        const Value& left = registerFile[code->m_srcIndex0];
        const Value& right = registerFile[code->m_srcIndex1];
        uint32_t lnum = left.toUint32(state);
        uint32_t rnum = right.toUint32(state);
        lnum = (lnum) >> ((rnum)&0x1F);
        registerFile[code->m_srcIndex0] = Value(lnum);
        executeNextCode<BinaryUnsignedRightShift>(programCounter);
        NEXT_INSTRUCTION();
    }

    CreateObjectOpcodeLbl : {
        CreateObject* code = (CreateObject*)currentCode;
        registerFile[code->m_registerIndex] = new Object(state);
        executeNextCode<CreateObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    CreateArrayOpcodeLbl : {
        CreateArray* code = (CreateArray*)currentCode;
        registerFile[code->m_registerIndex] = new ArrayObject(state);
        executeNextCode<CreateArray>(programCounter);
        NEXT_INSTRUCTION();
    }

    NewOperationOpcodeLbl : {
        NewOperation* code = (NewOperation*)currentCode;
        registerFile[code->m_registerIndex] = newOperation(state, registerFile[code->m_registerIndex], code->m_argumentCount, &registerFile[code->m_registerIndex + 1]);
        executeNextCode<NewOperation>(programCounter);
        NEXT_INSTRUCTION();
    }

    EndOpcodeLbl : {
        *state.exeuctionResult() = registerFile[0];
        return;
    }

    CallNativeFunctionOpcodeLbl : {
        CallNativeFunction* code = (CallNativeFunction*)currentCode;
        Value* argv = record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->argv();
        size_t argc = record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->argc();
        if (argc < record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->functionObject()->codeBlock()->parametersInfomation().size()) {
            size_t len = record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->functionObject()->codeBlock()->parametersInfomation().size();
            Value* newArgv = ALLOCA(sizeof(Value) * len, Value, state);
            for (size_t i = 0; i < argc; i++) {
                newArgv[i] = argv[i];
            }
            for (size_t i = argc; i < len; i++) {
                newArgv[i] = Value();
            }
            argv = newArgv;
            // argc = record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->functionObject()->codeBlock()->parametersInfomation().size();
        }
        *state.exeuctionResult() = code->m_fn(state, env->getThisBinding(), argc, argv, record->asDeclarativeEnvironmentRecord()->asFunctionEnvironmentRecord()->isNewExpression());
        return;
    }

    CallEvalFunctionOpcodeLbl : {
        CallEvalFunction* code = (CallEvalFunction*)currentCode;
        Value eval = loadByName(state, env, state.context()->staticStrings().eval);
        if (eval.equalsTo(state, state.context()->globalObject()->eval())) {
            // do eval
            Value arg;
            if (code->m_argumentCount) {
                arg = registerFile[code->m_registerIndex];
            }
            registerFile[code->m_registerIndex] = state.context()->globalObject()->eval(state, arg, codeBlock);
        } else {
            registerFile[code->m_registerIndex] = FunctionObject::call(eval, state, Value(), code->m_argumentCount, &registerFile[code->m_registerIndex]);
        }
        executeNextCode<CallEvalFunction>(programCounter);
        NEXT_INSTRUCTION();
    }

    TryOperationOpcodeLbl : {
        TryOperation* code = (TryOperation*)currentCode;
        try {
            size_t newPc = programCounter + sizeof(TryOperation);
            interpret(state, codeBlock, resolveProgramCounter(codeBuffer, newPc));
            ec->m_programCounter = &programCounter;
            programCounter = jumpTo(codeBuffer, code->m_tryCatchEndPosition);
        } catch (const Value& val) {
            state.context()->m_sandBoxStack.back()->m_stackTraceData.clear();
            if (code->m_hasCatch == false) {
                throw val;
            } else {
                // setup new env
                EnvironmentRecord* newRecord = new DeclarativeEnvironmentRecordNotIndexded(state);
                newRecord->createMutableBinding(state, code->m_catchVariableName);
                newRecord->setMutableBinding(state, code->m_catchVariableName, val);
                LexicalEnvironment* newEnv = new LexicalEnvironment(newRecord, env);
                ExecutionContext* newEc = new ExecutionContext(state.context(), state.executionContext(), newEnv, state.inStrictMode());
                newEc->giveStackStorage(stackStorage);
                ExecutionState newState(state.context(), newEc, state.exeuctionResult());
                interpret(newState, codeBlock, code->m_catchPosition);
                programCounter = jumpTo(codeBuffer, code->m_tryCatchEndPosition);
            }
        }
        NEXT_INSTRUCTION();
    }

    TryCatchBodyEndOpcodeLbl : {
        return;
    }

    FinallyEndOpcodeLbl : {
        FinallyEnd* code = (FinallyEnd*)currentCode;
        executeNextCode<FinallyEnd>(programCounter);
        NEXT_INSTRUCTION();
    }

    ThrowOperationOpcodeLbl : {
        ThrowOperation* code = (ThrowOperation*)currentCode;
        state.context()->throwException(state, registerFile[code->m_registerIndex]);
    }

    UnaryPlusOpcodeLbl : {
        UnaryPlus* code = (UnaryPlus*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        registerFile[code->m_registerIndex] = Value(val.toNumber(state));
        executeNextCode<UnaryPlus>(programCounter);
        NEXT_INSTRUCTION();
    }

    UnaryBitwiseNotOpcodeLbl : {
        UnaryBitwiseNot* code = (UnaryBitwiseNot*)currentCode;
        const Value& val = registerFile[code->m_registerIndex];
        registerFile[code->m_registerIndex] = Value(~val.toInt32(state));
        executeNextCode<UnaryBitwiseNot>(programCounter);
        NEXT_INSTRUCTION();
    }

    UnaryTypeofOpcodeLbl : {
        UnaryTypeof* code = (UnaryTypeof*)currentCode;
        Value val;
        if (code->m_id.string()->length()) {
            val = loadByName(state, env, code->m_id, false);
        } else {
            val = registerFile[code->m_registerIndex];
        }
        if (val.isUndefined())
            val = state.context()->staticStrings().undefined.string();
        else if (val.isNull())
            val = state.context()->staticStrings().object.string();
        else if (val.isBoolean())
            val = state.context()->staticStrings().boolean.string();
        else if (val.isNumber())
            val = state.context()->staticStrings().number.string();
        else if (val.isString())
            val = state.context()->staticStrings().string.string();
        else {
            ASSERT(val.isPointerValue());
            PointerValue* p = val.asPointerValue();
            if (p->isFunctionObject()) {
                val = state.context()->staticStrings().function.string();
            } else {
                val = state.context()->staticStrings().object.string();
            }
        }

        registerFile[code->m_registerIndex] = val;
        executeNextCode<UnaryTypeof>(programCounter);
        NEXT_INSTRUCTION();
    }

    JumpComplexCaseOpcodeLbl : {
        JumpComplexCase* code = (JumpComplexCase*)currentCode;
        ASSERT(code->m_jumpPosition != SIZE_MAX);
        programCounter = jumpTo(codeBuffer, code->m_jumpPosition);
        NEXT_INSTRUCTION();
    }

    EnumerateObjectOpcodeLbl : {
        EnumerateObject* code = (EnumerateObject*)currentCode;
        auto data = executeEnumerateObject(state, registerFile[code->m_objectRegisterIndex].toObject(state));
        registerFile[code->m_dataRegisterIndex] = Value((PointerValue*)data);
        executeNextCode<EnumerateObject>(programCounter);
        NEXT_INSTRUCTION();
    }

    CheckIfKeyIsLastOpcodeLbl : {
        CheckIfKeyIsLast* code = (CheckIfKeyIsLast*)currentCode;
        EnumerateObjectData* data = (EnumerateObjectData*)registerFile[code->m_registerIndex].asPointerValue();
        bool shouldUpdateEnumerateObjectData = false;
        Object* obj = data->m_object;
        for (size_t i = 0; i < data->m_hiddenClassChain.size(); i++) {
            ObjectStructure* hc = data->m_hiddenClassChain[i];
            if (hc != obj->structure()) {
                shouldUpdateEnumerateObjectData = true;
                break;
            }
            Value val = obj->getPrototype(state);
            if (val.isObject()) {
                obj = val.asObject();
            } else {
                break;
            }
        }

        if (shouldUpdateEnumerateObjectData) {
            registerFile[code->m_registerIndex] = Value((PointerValue*)updateEnumerateObjectData(state, data));
        }

        if (data->m_keys.size() == data->m_idx) {
            programCounter = jumpTo(codeBuffer, code->m_forInEndPosition);
        } else {
            executeNextCode<CheckIfKeyIsLast>(programCounter);
        }
        NEXT_INSTRUCTION();
    }

    EnumerateObjectKeyOpcodeLbl : {
        EnumerateObjectKey* code = (EnumerateObjectKey*)currentCode;
        EnumerateObjectData* data = (EnumerateObjectData*)registerFile[code->m_dataRegisterIndex].asPointerValue();
        data->m_idx++;
        registerFile[code->m_registerIndex] = data->m_keys[data->m_idx - 1];
        executeNextCode<EnumerateObjectKey>(programCounter);
        NEXT_INSTRUCTION();
    }

    LoadRegexpOpcodeLbl : {
        LoadRegexp* code = (LoadRegexp*)currentCode;
        auto reg = new RegExpObject(state, code->m_body, code->m_option);
        registerFile[code->m_registerIndex] = reg;
        executeNextCode<LoadRegexp>(programCounter);
        NEXT_INSTRUCTION();
    }
    }
FillOpcodeTable : {
#define REGISTER_TABLE(opcode, pushCount, popCount) \
    registerOpcode(opcode##Opcode, &&opcode##OpcodeLbl);
    FOR_EACH_BYTECODE_OP(REGISTER_TABLE);

#undef REGISTER_TABLE
}
}

Value ByteCodeInterpreter::loadByName(ExecutionState& state, LexicalEnvironment* env, const AtomicString& name, bool throwException)
{
    while (env) {
        EnvironmentRecord::GetBindingValueResult result = env->record()->getBindingValue(state, name);
        if (result.m_hasBindingValue) {
            return result.m_value;
        }
        env = env->outerEnvironment();
    }

    if (throwException)
        ErrorObject::throwBuiltinError(state, ErrorObject::ReferenceError, name.string(), false, String::emptyString, errorMessage_IsNotDefined);

    return Value();
}

void ByteCodeInterpreter::storeByName(ExecutionState& state, LexicalEnvironment* env, const AtomicString& name, const Value& value)
{
    while (env) {
        auto result = env->record()->hasBinding(state, name);
        if (result.m_index != SIZE_MAX) {
            return env->record()->asDeclarativeEnvironmentRecord()->setMutableBindingByIndex(result.m_index, value);
        }
        env = env->outerEnvironment();
    }
    if (state.inStrictMode()) {
        ErrorObject::throwBuiltinError(state, ErrorObject::Code::ReferenceError, name.string(), false, String::emptyString, errorMessage_IsNotDefined);
    }
    GlobalObject* o = state.context()->globalObject();
    o->setThrowsExceptionWhenStrictMode(state, name, value, o);
}

Value ByteCodeInterpreter::plusSlowCase(ExecutionState& state, const Value& left, const Value& right)
{
    Value ret(Value::ForceUninitialized);
    Value lval(Value::ForceUninitialized);
    Value rval(Value::ForceUninitialized);

    // http://www.ecma-international.org/ecma-262/5.1/#sec-8.12.8
    // No hint is provided in the calls to ToPrimitive in steps 5 and 6.
    // All native ECMAScript objects except Date objects handle the absence of a hint as if the hint Number were given;
    // Date objects handle the absence of a hint as if the hint String were given.
    // Host objects may handle the absence of a hint in some other manner.
    if (UNLIKELY(left.isPointerValue() && left.asPointerValue()->isDateObject())) {
        lval = left.toPrimitive(state, Value::PreferString);
    } else {
        lval = left.toPrimitive(state);
    }

    if (UNLIKELY(right.isPointerValue() && right.asPointerValue()->isDateObject())) {
        rval = right.toPrimitive(state, Value::PreferString);
    } else {
        rval = right.toPrimitive(state);
    }
    if (lval.isString() || rval.isString()) {
        ret = RopeString::createRopeString(lval.toString(state), rval.toString(state));
    } else {
        ret = Value(lval.toNumber(state) + rval.toNumber(state));
    }

    return ret;
}

Value ByteCodeInterpreter::modOperation(ExecutionState& state, const Value& left, const Value& right)
{
    Value ret(Value::ForceUninitialized);

    int32_t intLeft;
    int32_t intRight;
    if (left.isInt32() && ((intLeft = left.asInt32()) > 0) && right.isInt32() && (intRight = right.asInt32())) {
        ret = Value(intLeft % intRight);
    } else {
        double lvalue = left.toNumber(state);
        double rvalue = right.toNumber(state);
        // http://www.ecma-international.org/ecma-262/5.1/#sec-11.5.3
        if (std::isnan(lvalue) || std::isnan(rvalue))
            ret = Value(std::numeric_limits<double>::quiet_NaN());
        else if (std::isinf(lvalue) || rvalue == 0 || rvalue == -0.0)
            ret = Value(std::numeric_limits<double>::quiet_NaN());
        else if (std::isinf(rvalue))
            ret = Value(lvalue);
        else if (lvalue == 0.0) {
            if (std::signbit(lvalue))
                ret = Value(Value::EncodeAsDouble, -0.0);
            else
                ret = Value(0);
        } else {
            bool isLNeg = lvalue < 0.0;
            lvalue = std::abs(lvalue);
            rvalue = std::abs(rvalue);
            double r = fmod(lvalue, rvalue);
            if (isLNeg)
                r = -r;
            ret = Value(r);
        }
    }

    return ret;
}

Value ByteCodeInterpreter::newOperation(ExecutionState& state, const Value& callee, size_t argc, Value* argv)
{
    if (!callee.isFunction()) {
        ErrorObject::throwBuiltinError(state, ErrorObject::TypeError, errorMessage_Call_NotFunction);
    }
    FunctionObject* function = callee.asFunction();
    if (!function->isConstructor()) {
        ErrorObject::throwBuiltinError(state, ErrorObject::TypeError, function->codeBlock()->functionName().string(), false, String::emptyString, errorMessage_New_NotConstructor);
    }
    Object* receiver;
    CodeBlock* cb = function->codeBlock();
    if (cb->isNativeFunction()) {
        receiver = cb->nativeFunctionConstructor()(state, argc, argv);
    } else {
        receiver = new Object(state);
    }

    if (function->getFunctionPrototype(state).isObject())
        receiver->setPrototype(state, function->getFunctionPrototype(state));
    else
        receiver->setPrototype(state, new Object(state));

    Value res = function->call(state, receiver, argc, argv, true);
    if (res.isObject())
        return res;
    else
        return receiver;
}

inline bool ByteCodeInterpreter::abstractRelationalComparison(ExecutionState& state, const Value& left, const Value& right, bool leftFirst)
{
    // consume very fast case
    if (LIKELY(left.isInt32() && right.isInt32())) {
        return left.asInt32() < right.asInt32();
    }

    if (LIKELY(left.isNumber() && right.isNumber())) {
        return left.asNumber() < right.asNumber();
    }

    return abstractRelationalComparisonSlowCase(state, left, right, leftFirst);
}

inline bool ByteCodeInterpreter::abstractRelationalComparisonOrEqual(ExecutionState& state, const Value& left, const Value& right, bool leftFirst)
{
    // consume very fast case
    if (LIKELY(left.isInt32() && right.isInt32())) {
        return left.asInt32() <= right.asInt32();
    }

    if (LIKELY(left.isNumber() && right.isNumber())) {
        return left.asNumber() <= right.asNumber();
    }

    return abstractRelationalComparisonOrEqualSlowCase(state, left, right, leftFirst);
}

bool ByteCodeInterpreter::abstractRelationalComparisonSlowCase(ExecutionState& state, const Value& left, const Value& right, bool leftFirst)
{
    Value lval(Value::ForceUninitialized);
    Value rval(Value::ForceUninitialized);
    if (leftFirst) {
        lval = left.toPrimitive(state);
        rval = right.toPrimitive(state);
    } else {
        rval = right.toPrimitive(state);
        lval = left.toPrimitive(state);
    }

    // http://www.ecma-international.org/ecma-262/5.1/#sec-11.8.5
    if (lval.isInt32() && rval.isInt32()) {
        return lval.asInt32() < rval.asInt32();
    } else if (lval.isString() && rval.isString()) {
        return *lval.asString() < *rval.asString();
    } else {
        double n1 = lval.toNumber(state);
        double n2 = rval.toNumber(state);
        return n1 < n2;
    }
}

bool ByteCodeInterpreter::abstractRelationalComparisonOrEqualSlowCase(ExecutionState& state, const Value& left, const Value& right, bool leftFirst)
{
    Value lval(Value::ForceUninitialized);
    Value rval(Value::ForceUninitialized);
    if (leftFirst) {
        lval = left.toPrimitive(state);
        rval = right.toPrimitive(state);
    } else {
        rval = right.toPrimitive(state);
        lval = left.toPrimitive(state);
    }

    if (lval.isInt32() && rval.isInt32()) {
        return lval.asInt32() <= rval.asInt32();
    } else if (lval.isString() && rval.isString()) {
        return *lval.asString() <= *rval.asString();
    } else {
        double n1 = lval.toNumber(state);
        double n2 = rval.toNumber(state);
        return n1 <= n2;
    }
}

inline std::pair<bool, Value> ByteCodeInterpreter::getObjectPrecomputedCaseOperation(ExecutionState& state, Object* obj, const PropertyName& name, GetObjectInlineCache& inlineCache)
{
    Object* targetObj = obj;
    unsigned currentCacheIndex = 0;
    const size_t cacheFillCount = inlineCache.m_cache.size();
    for (; currentCacheIndex < cacheFillCount; currentCacheIndex++) {
        const GetObjectInlineCacheData& data = inlineCache.m_cache[currentCacheIndex];
        const ObjectStructureChain* const cachedHiddenClassChain = &data.m_cachedhiddenClassChain;
        const size_t& cachedIndex = data.m_cachedIndex;
        const size_t cSiz = cachedHiddenClassChain->size() - 1;
        for (size_t i = 0; i < cSiz; i++) {
            if (UNLIKELY((*cachedHiddenClassChain)[i] != obj->structure())) {
                goto GetObjecPreComputedCacheMiss;
            }
            const Value& proto = obj->getPrototype(state);
            if (LIKELY(proto.isObject())) {
                obj = proto.asObject();
            } else {
                goto GetObjecPreComputedCacheMiss;
            }
        }
        if (LIKELY((*cachedHiddenClassChain)[cSiz] == obj->structure())) {
            if (cachedIndex != SIZE_MAX) {
                return std::make_pair(true, obj->getOwnPropertyUtilForObject(state, cachedIndex, targetObj));
            } else {
                return std::make_pair(false, Value());
            }
        }
    GetObjecPreComputedCacheMiss : {
    }
    }

    // cache miss.
    inlineCache.m_executeCount++;
    if (inlineCache.m_executeCount <= 3) {
        auto result = obj->get(state, ObjectPropertyName(state, name));
        return std::make_pair(result.hasValue(), result.value());
    }

    obj = targetObj;
    inlineCache.m_cache.insert(0, GetObjectInlineCacheData());
    currentCacheIndex = 0;
    ASSERT(&inlineCache.m_cache[0] == &inlineCache.m_cache[currentCacheIndex]);
    ObjectStructureChain* cachedHiddenClassChain = &inlineCache.m_cache[currentCacheIndex].m_cachedhiddenClassChain;
    size_t* cachedHiddenClassIndex = &inlineCache.m_cache[currentCacheIndex].m_cachedIndex;
    while (true) {
        cachedHiddenClassChain->push_back(obj->structure());
        size_t idx = obj->structure()->findProperty(state, name);
        if (idx != SIZE_MAX) {
            *cachedHiddenClassIndex = idx;
            break;
        }
        const Value& proto = obj->getPrototype(state);
        if (proto.isObject()) {
            obj = proto.asObject();
        } else
            break;
    }

    if (*cachedHiddenClassIndex != SIZE_MAX) {
        return std::make_pair(true, obj->getOwnPropertyUtilForObject(state, *cachedHiddenClassIndex, targetObj));
    } else {
        return std::make_pair(false, Value());
    }
}

inline void ByteCodeInterpreter::setObjectPreComputedCaseOperation(ExecutionState& state, Object* obj, const PropertyName& name, const Value& value, SetObjectInlineCache& inlineCache)
{
    Object* originalObject = obj;
    if (inlineCache.m_cachedIndex != SIZE_MAX && inlineCache.m_cachedhiddenClassChain[0] == obj->structure()) {
        ASSERT(inlineCache.m_cachedhiddenClassChain.size() == 1);
        // cache hit!
        obj->setOwnPropertyThrowsExceptionWhenStrictMode(state, inlineCache.m_cachedIndex, value);
        return;
    } else if (inlineCache.m_hiddenClassWillBe) {
        int cSiz = inlineCache.m_cachedhiddenClassChain.size();
        bool miss = false;
        for (int i = 0; i < cSiz - 1; i++) {
            if (inlineCache.m_cachedhiddenClassChain[i] != obj->structure()) {
                miss = true;
                break;
            } else {
                Value o = obj->getPrototype(state);
                if (!o.isObject()) {
                    miss = true;
                    break;
                }
                obj = o.asObject();
            }
        }
        if (!miss) {
            if (inlineCache.m_cachedhiddenClassChain[cSiz - 1] == obj->structure()) {
                // cache hit!
                obj = originalObject;
                obj->m_values.push_back(value);
                obj->m_structure = inlineCache.m_hiddenClassWillBe;
                return;
            }
        }
    }

    // cache miss
    inlineCache.invalidateCache();

    obj = originalObject;

    size_t idx = obj->structure()->findProperty(state, name);
    if (idx != SIZE_MAX) {
        // own property
        inlineCache.m_cachedIndex = idx;
        inlineCache.m_cachedhiddenClassChain.push_back(obj->structure());

        obj->setOwnPropertyThrowsExceptionWhenStrictMode(state, inlineCache.m_cachedIndex, value);
    } else {
        inlineCache.m_cachedhiddenClassChain.push_back(obj->structure());
        Object* orgObject = obj;
        Value proto = obj->getPrototype(state);
        while (proto.isObject()) {
            obj = proto.asObject();
            inlineCache.m_cachedhiddenClassChain.push_back(obj->structure());
            proto = obj->getPrototype(state);
        }
        bool s = orgObject->set(state, ObjectPropertyName(state, name), value, obj);
        if (UNLIKELY(!s)) {
            if (state.inStrictMode())
                obj->throwCannotWriteError(state, name);

            inlineCache.invalidateCache();
            return;
        }
        inlineCache.m_hiddenClassWillBe = orgObject->structure();
    }
}

EnumerateObjectData* ByteCodeInterpreter::executeEnumerateObject(ExecutionState& state, Object* obj)
{
    EnumerateObjectData* data = new EnumerateObjectData();
    data->m_object = obj;

    Value target = data->m_object;

    size_t ownKeyCount = 0;
    bool shouldSearchProto = false;

    target.asObject()->enumeration(state, [&](const ObjectPropertyName&, const ObjectPropertyDescriptor& desc) -> bool {
        if (desc.isEnumerable()) {
            ownKeyCount++;
        }
        return true;
    });
    data->m_hiddenClassChain.push_back(target.asObject()->structure());

    std::unordered_set<String*, std::hash<String*>, std::equal_to<String*>, gc_malloc_ignore_off_page_allocator<String*>> keyStringSet;

    target = target.asObject()->getPrototype(state);
    while (target.isObject()) {
        if (!shouldSearchProto) {
            target.asObject()->enumeration(state, [&](const ObjectPropertyName& name, const ObjectPropertyDescriptor& desc) -> bool {
                if (desc.isEnumerable()) {
                    shouldSearchProto = true;
                    return false;
                }
                return true;
            });
        }
        data->m_hiddenClassChain.push_back(target.asObject()->structure());
        target = target.asObject()->getPrototype(state);
    }

    target = obj;
    if (shouldSearchProto) {
        while (target.isObject()) {
            target.asObject()->enumeration(state, [&](const ObjectPropertyName& name, const ObjectPropertyDescriptor& desc) -> bool {
                if (desc.isEnumerable()) {
                    String* key = name.toValue(state).toString(state);
                    auto iter = keyStringSet.find(key);
                    if (iter == keyStringSet.end()) {
                        keyStringSet.insert(key);
                        data->m_keys.pushBack(name.toValue(state));
                    }
                }
                return true;
            });
            target = target.asObject()->getPrototype(state);
        }
    } else {
        size_t idx = 0;
        data->m_keys.resizeWithUninitializedValues(ownKeyCount);
        target.asObject()->enumeration(state, [&](const ObjectPropertyName& name, const ObjectPropertyDescriptor& desc) -> bool {
            if (desc.isEnumerable()) {
                data->m_keys[idx++] = name.toValue(state);
            }
            return true;
        });
        ASSERT(ownKeyCount == idx);
    }

    return data;
}

EnumerateObjectData* ByteCodeInterpreter::updateEnumerateObjectData(ExecutionState& state, EnumerateObjectData* data)
{
    EnumerateObjectData* newData = executeEnumerateObject(state, data->m_object);
    std::vector<Value, gc_malloc_ignore_off_page_allocator<Value>> oldKeys;
    if (data->m_keys.size()) {
        oldKeys.insert(oldKeys.end(), &data->m_keys[0], &data->m_keys[data->m_keys.size() - 1] + 1);
    }
    std::vector<Value, gc_malloc_ignore_off_page_allocator<Value>> differenceKeys;
    for (size_t i = 0; i < newData->m_keys.size(); i++) {
        const Value& key = newData->m_keys[i];
        if (std::find(oldKeys.begin(), oldKeys.begin() + data->m_idx, key) == oldKeys.begin() + data->m_idx) {
            // If a property that has not yet been visited during enumeration is deleted, then it will not be visited.
            if (std::find(oldKeys.begin() + data->m_idx, oldKeys.end(), key) != oldKeys.end()) {
                // If new properties are added to the object being enumerated during enumeration,
                // the newly added properties are not guaranteed to be visited in the active enumeration.
                differenceKeys.push_back(key);
            }
        }
    }
    data = newData;
    data->m_keys.clear();
    data->m_keys.resizeWithUninitializedValues(differenceKeys.size());
    for (size_t i = 0; i < differenceKeys.size(); i++) {
        data->m_keys[i] = differenceKeys[i];
    }
    return data;
}
}