#include "Escargot.h"
#include "ESValue.h"

#include "vm/ESVMInstance.h"
#include "runtime/ExecutionContext.h"
#include "runtime/Environment.h"
#include "ast/AST.h"
#include "jit/ESJIT.h"

#include "Yarr.h"

#include "fast-dtoa.h"
#include "bignum-dtoa.h"

namespace escargot {

enum Flags {
  NO_FLAGS = 0,
  EMIT_POSITIVE_EXPONENT_SIGN = 1,
  EMIT_TRAILING_DECIMAL_POINT = 2,
  EMIT_TRAILING_ZERO_AFTER_POINT = 4,
  UNIQUE_ZERO = 8
};

void CreateDecimalRepresentation(
        int flags_,
    const char* decimal_digits,
    int length,
    int decimal_point,
    int digits_after_point,
    double_conversion::StringBuilder* result_builder) {
  // Create a representation that is padded with zeros if needed.
  if (decimal_point <= 0) {
      // "0.00000decimal_rep".
    result_builder->AddCharacter('0');
    if (digits_after_point > 0) {
      result_builder->AddCharacter('.');
      result_builder->AddPadding('0', -decimal_point);
      ASSERT(length <= digits_after_point - (-decimal_point));
      result_builder->AddSubstring(decimal_digits, length);
      int remaining_digits = digits_after_point - (-decimal_point) - length;
      result_builder->AddPadding('0', remaining_digits);
    }
  } else if (decimal_point >= length) {
    // "decimal_rep0000.00000" or "decimal_rep.0000"
    result_builder->AddSubstring(decimal_digits, length);
    result_builder->AddPadding('0', decimal_point - length);
    if (digits_after_point > 0) {
      result_builder->AddCharacter('.');
      result_builder->AddPadding('0', digits_after_point);
    }
  } else {
    // "decima.l_rep000"
    ASSERT(digits_after_point > 0);
    result_builder->AddSubstring(decimal_digits, decimal_point);
    result_builder->AddCharacter('.');
    ASSERT(length - decimal_point <= digits_after_point);
    result_builder->AddSubstring(&decimal_digits[decimal_point],
                                 length - decimal_point);
    int remaining_digits = digits_after_point - (length - decimal_point);
    result_builder->AddPadding('0', remaining_digits);
  }
  if (digits_after_point == 0) {
    if ((flags_ & EMIT_TRAILING_DECIMAL_POINT) != 0) {
      result_builder->AddCharacter('.');
    }
    if ((flags_ & EMIT_TRAILING_ZERO_AFTER_POINT) != 0) {
      result_builder->AddCharacter('0');
    }
  }
}

void CreateExponentialRepresentation(
        int flags_,
        const char* decimal_digits,
        int length,
        int exponent,
        double_conversion::StringBuilder* result_builder) {
      ASSERT(length != 0);
      result_builder->AddCharacter(decimal_digits[0]);
      if (length != 1) {
        result_builder->AddCharacter('.');
        result_builder->AddSubstring(&decimal_digits[1], length-1);
      }
      result_builder->AddCharacter('e');
      if (exponent < 0) {
        result_builder->AddCharacter('-');
        exponent = -exponent;
      } else {
        if ((flags_ & EMIT_POSITIVE_EXPONENT_SIGN) != 0) {
          result_builder->AddCharacter('+');
        }
      }
      if (exponent == 0) {
        result_builder->AddCharacter('0');
        return;
      }
      ASSERT(exponent < 1e4);
      const int kMaxExponentLength = 5;
      char buffer[kMaxExponentLength + 1];
      buffer[kMaxExponentLength] = '\0';
      int first_char_pos = kMaxExponentLength;
      while (exponent > 0) {
        buffer[--first_char_pos] = '0' + (exponent % 10);
        exponent /= 10;
      }
      result_builder->AddSubstring(&buffer[first_char_pos],
                                   kMaxExponentLength - first_char_pos);
    }

ESStringData::ESStringData(double number)
{
    m_hashData.m_isHashInited =  false;

    if(number == 0) {
        operator += ('0');
        return ;
    }
    const int flags = UNIQUE_ZERO | EMIT_POSITIVE_EXPONENT_SIGN;
    bool sign = false;
    if(number < 0) {
        sign = true;
        number = -number;
    }
    // The maximal number of digits that are needed to emit a double in base 10.
    // A higher precision can be achieved by using more digits, but the shortest
    // accurate representation of any double will never use more digits than
    // kBase10MaximalLength.
    // Note that DoubleToAscii null-terminates its input. So the given buffer
    // should be at least kBase10MaximalLength + 1 characters long.
    const int kBase10MaximalLength = 17;
    const int kDecimalRepCapacity = kBase10MaximalLength + 1;
    char decimal_rep[kDecimalRepCapacity];
    int decimal_rep_length;
    int decimal_point;
    double_conversion::Vector<char> vector(decimal_rep, kDecimalRepCapacity);
    bool fast_worked = FastDtoa(number, double_conversion::FAST_DTOA_SHORTEST, 0, vector, &decimal_rep_length, &decimal_point);
    if(!fast_worked) {
        BignumDtoa(number, double_conversion::BIGNUM_DTOA_SHORTEST, 0, vector, &decimal_rep_length, &decimal_point);
        vector[decimal_rep_length] = '\0';
    }

/*    reserve(decimal_rep_length + sign ? 1 : 0);
    if(sign)
        operator +=('-');
    for(unsigned i = 0; i < decimal_rep_length ; i ++) {
        operator +=(decimal_rep[i]);
    }*/

    const int bufferLength = 128;
    char buffer[bufferLength];
    double_conversion::StringBuilder builder(buffer, bufferLength);

    int exponent = decimal_point - 1;
    const int decimal_in_shortest_low_ = -6;
    const int decimal_in_shortest_high_ = 21;
    if ((decimal_in_shortest_low_ <= exponent) &&
        (exponent < decimal_in_shortest_high_)) {
      CreateDecimalRepresentation(flags, decimal_rep, decimal_rep_length,
                                  decimal_point,
                                  double_conversion::Max(0, decimal_rep_length - decimal_point),
                                  &builder);
    } else {
      CreateExponentialRepresentation(flags, decimal_rep, decimal_rep_length, exponent,
              &builder);
    }

    if(sign)
        operator += ('-');
    char* buf = builder.Finalize();
    while(*buf) {
        operator += (*buf);
        buf++;
    }

}

uint32_t ESString::tryToUseAsIndex()
{
    const u16string& s = string();
    bool allOfCharIsDigit = true;
    uint32_t number = 0;
    for(unsigned i = 0; i < s.length() ; i ++) {
        char16_t c = s[i];
        if(c < '0' || c > '9') {
            allOfCharIsDigit = false;
            break;
        } else {
            number = number*10 + (c-'0');
        }
    }
    if(allOfCharIsDigit) {
        return number;
    }
    return ESValue::ESInvalidIndexValue;
}

ESString* ESString::substring(int from, int to) const
{
    ASSERT(0 <= from && from <= to && to <= (int)length());
    if(UNLIKELY(m_string == NULL)) {
        escargot::ESRopeString* rope = (escargot::ESRopeString *)this;
        if(to - from == 1) {
            int len_left = rope->m_left->length();
            char16_t c;
            if (to <= len_left) {
                c = rope->m_left->stringData()->c_str()[from];
            } else {
                c = rope->m_left->stringData()->c_str()[from - len_left];
            }
            if(c < ESCARGOT_ASCII_TABLE_MAX) {
                return strings->asciiTable[c].string();
            }
        }
        int len_left = rope->m_left->length();
        if (to <= len_left) {
            u16string ret(std::move(rope->m_left->stringData()->substr(from, to-from)));
            return ESString::create(std::move(ret));
        } else if (len_left <= from) {
            u16string ret(std::move(rope->m_right->stringData()->substr(from - len_left, to-from)));
            return ESString::create(std::move(ret));
        } else {
            ESString* lstr = nullptr;
            if (from == 0)
                lstr = rope->m_left;
            else {
                u16string left(std::move(rope->m_left->stringData()->substr(from, len_left - from)));
                lstr = ESString::create(std::move(left));
            }
            ESString* rstr = nullptr;
            if (to == length())
                rstr = rope->m_right;
            else {
                u16string right(std::move(rope->m_right->stringData()->substr(0, to - len_left)));
                rstr = ESString::create(std::move(right));
            }
            return ESRopeString::createAndConcat(lstr, rstr);
        }
        ensureNormalString();
    }
    if(to - from == 1) {
        if(string()[from] < ESCARGOT_ASCII_TABLE_MAX) {
            return strings->asciiTable[string()[from]].string();
        }
    }
    u16string ret(std::move(m_string->substr(from, to-from)));
    return ESString::create(std::move(ret));
}


bool ESString::match(ESPointer* esptr, RegexMatchResult& matchResult, bool testOnly, size_t startIndex) const
{
    //NOTE to build normal string(for rope-string), we should call ensureNormalString();
    ensureNormalString();

    ESRegExpObject::Option option = ESRegExpObject::Option::None;
    const u16string* regexSource;
    JSC::Yarr::BytecodePattern* byteCode = NULL;
    ESString* tmpStr;
    if (esptr->isESRegExpObject()) {
        escargot::ESRegExpObject* o = esptr->asESRegExpObject();
        regexSource = &esptr->asESRegExpObject()->source()->string();
        option = esptr->asESRegExpObject()->option();
        byteCode = esptr->asESRegExpObject()->bytecodePattern();
    } else {
        tmpStr = ESValue(esptr).toString();
        regexSource = tmpStr->stringData();
    }

    bool isGlobal = option & ESRegExpObject::Option::Global;
    if(!byteCode) {
        JSC::Yarr::ErrorCode yarrError;
        JSC::Yarr::YarrPattern yarrPattern(*regexSource, option & ESRegExpObject::Option::IgnoreCase, option & ESRegExpObject::Option::MultiLine, &yarrError);
        if (yarrError) {
            matchResult.m_subPatternNum = 0;
            return false;
        }
        WTF::BumpPointerAllocator *bumpAlloc = ESVMInstance::currentInstance()->bumpPointerAllocator();
        JSC::Yarr::OwnPtr<JSC::Yarr::BytecodePattern> ownedBytecode = JSC::Yarr::byteCompileEscargot(yarrPattern, bumpAlloc);
        byteCode = ownedBytecode.leakPtr();
        if(esptr->isESRegExpObject()) {
            esptr->asESRegExpObject()->setBytecodePattern(byteCode);
        }
    }

    unsigned subPatternNum = byteCode->m_body->m_numSubpatterns;
    matchResult.m_subPatternNum = (int) subPatternNum;
    size_t length = m_string->length();
    if(length) {
        size_t start = startIndex;
        unsigned result = 0;
        const char16_t *chars = m_string->data();
        unsigned* outputBuf = (unsigned int*)alloca(sizeof (unsigned) * 2 * (subPatternNum + 1));
        outputBuf[1] = start;
        do {
            start = outputBuf[1];
            memset(outputBuf,-1,sizeof (unsigned) * 2 * (subPatternNum + 1));
            if(start >= length)
                break;
            result = JSC::Yarr::interpret(NULL, byteCode, chars, length, start, outputBuf);
            if(result != JSC::Yarr::offsetNoMatch) {
                if(UNLIKELY(testOnly)) {
                    return true;
                }
                std::vector<ESString::RegexMatchResult::RegexMatchResultPiece> piece;
                piece.reserve(subPatternNum + 1);

                for(unsigned i = 0; i < subPatternNum + 1 ; i ++) {
                    ESString::RegexMatchResult::RegexMatchResultPiece p;
                    p.m_start = outputBuf[i*2];
                    p.m_end = outputBuf[i*2 + 1];
                    piece.push_back(p);
                }
                matchResult.m_matchResults.push_back(std::move(piece));
                if(!isGlobal)
                    break;
                if(start == outputBuf[1]) {
                    outputBuf[1]++;
                    if(outputBuf[1] > length) {
                        break;
                    }
                }
            } else {
                break;
            }
        } while(result != JSC::Yarr::offsetNoMatch);
    }
    return matchResult.m_matchResults.size();
}

ESObject* ESObject::create(size_t initialKeyCount)
{
    return new ESObject(ESPointer::Type::ESObject, ESVMInstance::currentInstance()->globalObject()->objectPrototype(), initialKeyCount);
}

ESObject::ESObject(ESPointer::Type type, ESValue __proto__, size_t initialKeyCount)
    : ESPointer(type)
{
    m_flags.m_isExtensible = true;
    m_flags.m_isGlobalObject = false;
    m_flags.m_isEverSetAsPrototypeObject = false;

    m_hiddenClassData.reserve(initialKeyCount);
    m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForObject();

    m_hiddenClassData.push_back(ESValue((ESPointer *)ESVMInstance::currentInstance()->object__proto__AccessorData()));

    set__proto__(__proto__);
}

const unsigned ESArrayObject::MAX_FASTMODE_SIZE;

ESArrayObject::ESArrayObject(int length)
    : ESObject((Type)(Type::ESObject | Type::ESArrayObject), ESVMInstance::currentInstance()->globalObject()->arrayPrototype(), 3)
    , m_vector(0)
    , m_fastmode(true)
{
    m_length = 0;
    if (length == -1)
        convertToSlowMode();
    else if (length > 0) {
        setLength(length);
    }

    //defineAccessorProperty(strings->length.string(), ESVMInstance::currentInstance()->arrayLengthAccessorData(), true, false, false);
    m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForArrayObject();
    m_hiddenClassData.push_back((ESPointer *)ESVMInstance::currentInstance()->arrayLengthAccessorData());
}

ESRegExpObject::ESRegExpObject(escargot::ESString* source, const Option& option)
    : ESObject((Type)(Type::ESObject | Type::ESRegExpObject), ESVMInstance::currentInstance()->globalObject()->regexpPrototype())
{
    m_source = source;
    m_option = option;
    m_bytecodePattern = NULL;
    m_lastIndex = 0;
    m_lastExecutedString = NULL;
}

void ESRegExpObject::setSource(escargot::ESString* src)
{
    m_bytecodePattern = NULL;
    m_source = src;
}
void ESRegExpObject::setOption(const Option& option)
{
    if(((m_option & ESRegExpObject::Option::MultiLine) != (option & ESRegExpObject::Option::MultiLine)) ||
            ((m_option & ESRegExpObject::Option::IgnoreCase) != (option & ESRegExpObject::Option::IgnoreCase))
            ) {
        m_bytecodePattern = NULL;
    }
    m_option = option;
}

ESFunctionObject::ESFunctionObject(LexicalEnvironment* outerEnvironment, CodeBlock* cb, escargot::ESString* name, unsigned length)
    : ESObject((Type)(Type::ESObject | Type::ESFunctionObject), ESVMInstance::currentInstance()->globalFunctionPrototype(), 4)
{
    m_name = name;
    m_outerEnvironment = outerEnvironment;
    m_codeBlock = cb;
    m_protoType = ESObject::create(2);

    //m_protoType.asESPointer()->asESObject()->defineDataProperty(strings->constructor.string(), true, false, true, this);
    m_protoType.asESPointer()->asESObject()->m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForPrototypeObject();
    m_protoType.asESPointer()->asESObject()->m_hiddenClassData.push_back(this);

    // $19.2.4 Function Instances
    // these define in ESVMInstance::ESVMInstance()
    //defineDataProperty(strings->length, false, false, true, ESValue(length));
    //defineAccessorProperty(strings->prototype.string(), ESVMInstance::currentInstance()->functionPrototypeAccessorData(), true, false, false);
    //defineDataProperty(strings->name.string(), false, false, true, name);
    m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForFunctionObject();
    m_hiddenClassData.push_back(ESValue(length));
    m_hiddenClassData.push_back(ESValue((ESPointer *)ESVMInstance::currentInstance()->functionPrototypeAccessorData()));
    m_hiddenClassData.push_back(ESValue(name));
}

ESFunctionObject::ESFunctionObject(LexicalEnvironment* outerEnvironment, NativeFunctionType fn, escargot::ESString* name, unsigned length)
    : ESFunctionObject(outerEnvironment, (CodeBlock *)NULL, name, length)
{
    m_codeBlock = CodeBlock::create();
    m_codeBlock->pushCode(ExecuteNativeFunction(fn), NULL);
    m_codeBlock->m_isBuiltInFunction = true;
#ifdef ENABLE_ESJIT
    m_codeBlock->m_dontJIT = true;
#endif
    m_name = name;
}

ALWAYS_INLINE void functionCallerInnerProcess(ExecutionContext* newEC, ESFunctionObject* fn, ESValue& receiver, ESValue arguments[], const size_t& argumentCount, ESVMInstance* ESVMInstance)
{
    bool strict = fn->codeBlock()->shouldUseStrictMode();
    newEC->setStrictMode(strict);

    //http://www.ecma-international.org/ecma-262/6.0/#sec-ordinarycallbindthis
    if(!strict) {
        if(receiver.isUndefinedOrNull()) {
            receiver = ESVMInstance->globalObject();
        } else {
            receiver = receiver.toObject();
        }
    }

    ((FunctionEnvironmentRecord *)ESVMInstance->currentExecutionContext()->environment()->record())->bindThisValue(receiver);
    DeclarativeEnvironmentRecord* functionRecord = ESVMInstance->currentExecutionContext()->environment()->record()->toDeclarativeEnvironmentRecord();

    if(UNLIKELY(fn->codeBlock()->m_needsActivation)) {
        const InternalAtomicStringVector& params = fn->codeBlock()->m_params;
        const ESStringVector& nonAtomicParams = fn->codeBlock()->m_nonAtomicParams;
        for(unsigned i = 0; i < params.size() ; i ++) {
            if(i < argumentCount) {
                *functionRecord->bindingValueForActivationMode(i) = arguments[i];
            }
        }
    } else {
        const InternalAtomicStringVector& params = fn->codeBlock()->m_params;
        const ESStringVector& nonAtomicParams = fn->codeBlock()->m_nonAtomicParams;
        for(unsigned i = 0; i < params.size() ; i ++) {
            if(i < argumentCount) {
                ESVMInstance->currentExecutionContext()->cachedDeclarativeEnvironmentRecordESValue()[i] = arguments[i];
            }
        }
    }
}

ESValue ESFunctionObject::call(ESVMInstance* instance, const ESValue& callee, const ESValue& receiverInput, ESValue arguments[], const size_t& argumentCount, bool isNewExpression)
{
    ESValue result(ESValue::ESForceUninitialized);
    if(LIKELY(callee.isESPointer() && callee.asESPointer()->isESFunctionObject())) {
        ESValue receiver = receiverInput;
        ExecutionContext* currentContext = instance->currentExecutionContext();
        ESFunctionObject* fn = callee.asESPointer()->asESFunctionObject();

        if(UNLIKELY(fn->codeBlock()->m_needsActivation)) {
            instance->m_currentExecutionContext = new ExecutionContext(LexicalEnvironment::newFunctionEnvironment(arguments, argumentCount, fn), true, isNewExpression,
                    currentContext,
                    arguments, argumentCount);
            functionCallerInnerProcess(instance->m_currentExecutionContext, fn, receiver, arguments, argumentCount, instance);
            //ESVMInstance->invalidateIdentifierCacheCheckCount();
            //execute;
            result = interpret(instance, fn->codeBlock());
            instance->m_currentExecutionContext = currentContext;
        } else {
            ESValue* storage = (::escargot::ESValue *)alloca(sizeof(::escargot::ESValue) * fn->m_codeBlock->m_innerIdentifiers.size());
            FunctionEnvironmentRecord envRec(
                    arguments, argumentCount,
                    storage,
                    &fn->m_codeBlock->m_innerIdentifiers);

            //envRec.m_functionObject = fn;
            //envRec.m_newTarget = receiver;

            LexicalEnvironment env(&envRec, fn->outerEnvironment());
            ExecutionContext ec(&env, false, isNewExpression, currentContext, arguments, argumentCount, storage);
            instance->m_currentExecutionContext = &ec;
            functionCallerInnerProcess(&ec, fn, receiver, arguments, argumentCount, instance);
            //ESVMInstance->invalidateIdentifierCacheCheckCount();
            //execute;
#ifdef ENABLE_ESJIT
            ESJIT::JITFunction jitFunction = fn->codeBlock()->m_cachedJITFunction;
            if (!jitFunction && !fn->codeBlock()->m_dontJIT && fn->codeBlock()->m_executeCount > 7) {
#ifndef NDEBUG
                if (ESVMInstance::currentInstance()->m_verboseJIT)
                    printf("Trying JIT Compile for function %s...\n", fn->codeBlock()->m_nonAtomicId ? (fn->codeBlock()->m_nonAtomicId->utf8Data()):"(anonymous)");
#endif
                jitFunction = reinterpret_cast<ESJIT::JITFunction>(ESJIT::JITCompile(fn->codeBlock()));
                if (jitFunction) {
#ifndef NDEBUG
                    if (ESVMInstance::currentInstance()->m_verboseJIT)
                        printf("> Compilation successful! Cache jit function %p\n", jitFunction);
#endif
                    fn->codeBlock()->m_cachedJITFunction = jitFunction;
                } else {
#ifndef NDEBUG
                    if (ESVMInstance::currentInstance()->m_verboseJIT)
                        printf("> Compilation failed! disable jit compilation from now on\n");
#endif
                    fn->codeBlock()->m_dontJIT = true;
                }
            }

            if (jitFunction) {
                result = ESValue::fromRawDouble(jitFunction(instance));
                // printf("JIT Result %s\n", result.toString()->utf8Data());
                if (ec.inOSRExit()) {
                    ASSERT(result.isInt32());
                    printf("OSR EXIT from tmp%d\n", result.asInt32());
                    RELEASE_ASSERT_NOT_REACHED();
                }
            } else {
                //printf("JIT failed! Execute interpreter\n");
                result = interpret(instance, fn->codeBlock());
                fn->codeBlock()->m_executeCount++;
            }
#else
            result = interpret(instance, fn->codeBlock());
#endif
            instance->m_currentExecutionContext = currentContext;
        }
    } else {
        throw ESValue(TypeError::create(ESString::create(u"Callee is not a function object")));
    }

    return result;
}

ESDateObject::ESDateObject(ESPointer::Type type)
       : ESObject((Type)(Type::ESObject | Type::ESDateObject), ESVMInstance::currentInstance()->globalObject()->datePrototype())
{
    m_isCacheDirty = true;
}

void ESDateObject::parseYmdhmsToDate(struct tm* timeinfo, int year, int month, int date, int hour, int minute, int second) {
      char buffer[255];
      snprintf(buffer, 255, "%d-%d-%d-%d-%d-%d", year, month, date, hour, minute, second);
      strptime(buffer, "%Y-%m-%d-%H-%M-%S", timeinfo);
}

void ESDateObject::parseStringToDate(struct tm* timeinfo, escargot::ESString* istr) {
      int len = istr->length();
      char* buffer = (char*)istr->utf8Data();
      if (isalpha(buffer[0])) {
          strptime(buffer, "%B %d %Y %H:%M:%S %z", timeinfo);
      } else if (isdigit(buffer[0])) {
          strptime(buffer, "%m/%d/%Y %H:%M:%S", timeinfo);
      }
      GC_free(buffer);
}

const double hoursPerDay = 24.0;
const double minutesPerHour = 60.0;
const double secondsPerHour = 60.0 * 60.0;
const double secondsPerMinute = 60.0;
const double msPerSecond = 1000.0;
const double msPerMinute = 60.0 * 1000.0;
const double msPerHour = 60.0 * 60.0 * 1000.0;
const double msPerDay = 24.0 * 60.0 * 60.0 * 1000.0;
const double msPerMonth = 2592000000.0;

static inline double ymdhmsToSeconds(long year, int mon, int day, int hour, int minute, double second)
{
    double days = (day - 32075)
        + floor(1461 * (year + 4800.0 + (mon - 14) / 12) / 4)
        + 367 * (mon - 2 - (mon - 14) / 12 * 12) / 12
        - floor(3 * ((year + 4900.0 + (mon - 14) / 12) / 100) / 4)
        - 2440588;
    return ((days * hoursPerDay + hour) * minutesPerHour + minute) * secondsPerMinute + second;
}


void ESDateObject::setTimeValue() {
    clock_gettime(CLOCK_REALTIME,&m_time);
    m_isCacheDirty = true;
}

void ESDateObject::setTimeValue(const ESValue str) {
    escargot::ESString* istr = str.toString();
    parseStringToDate(&m_cachedTM, istr);
    m_cachedTM.tm_isdst = true;
    m_time.tv_sec = ymdhmsToSeconds(m_cachedTM.tm_year+1900, m_cachedTM.tm_mon + 1, m_cachedTM.tm_mday, m_cachedTM.tm_hour, m_cachedTM.tm_min, m_cachedTM.tm_sec);
}

void ESDateObject::setTimeValue(int year, int month, int date, int hour, int minute, int second, int millisecond) {
    parseYmdhmsToDate(&m_cachedTM, year, month, date, hour, minute, second);
    m_cachedTM.tm_isdst = true;
    m_time.tv_sec = ymdhmsToSeconds(m_cachedTM.tm_year+1900, m_cachedTM.tm_mon + 1, m_cachedTM.tm_mday, m_cachedTM.tm_hour, m_cachedTM.tm_min, m_cachedTM.tm_sec);
}

void ESDateObject::resolveCache()
{
    if(m_isCacheDirty) {
        memcpy(&m_cachedTM, ESVMInstance::currentInstance()->computeLocalTime(m_time), sizeof (tm));
        m_isCacheDirty = false;
    }
}

int ESDateObject::getDate() {
    resolveCache();
    return m_cachedTM.tm_mday;
}

int ESDateObject::getDay() {
    resolveCache();
    return m_cachedTM.tm_wday;
}

int ESDateObject::getFullYear() {
    resolveCache();
    return m_cachedTM.tm_year + 1900;
}

int ESDateObject::getHours() {
    resolveCache();
    return m_cachedTM.tm_hour;
}

int ESDateObject::getMinutes() {
    resolveCache();
    return m_cachedTM.tm_min;
}

int ESDateObject::getMonth() {
    resolveCache();
    return m_cachedTM.tm_mon;
}

int ESDateObject::getSeconds() {
    resolveCache();
    return m_cachedTM.tm_sec;
}

int ESDateObject::getTimezoneOffset() {
    return ESVMInstance::currentInstance()->timezoneOffset();

}

void ESDateObject::setTime(double t) {
    time_t raw_t = (time_t) floor(t);
    m_time.tv_sec = raw_t / 1000;
    m_time.tv_nsec = (raw_t % 10000) * 1000000;

    m_isCacheDirty = true;
}

ESStringObject::ESStringObject(escargot::ESString* str)
    : ESObject((Type)(Type::ESObject | Type::ESStringObject), ESVMInstance::currentInstance()->globalObject()->stringPrototype())
{
    m_stringData = str;

    //$21.1.4.1 String.length
    defineAccessorProperty(strings->length.string(), ESVMInstance::currentInstance()->stringObjectLengthAccessorData(), false, true, false);
}

ESNumberObject::ESNumberObject(double value)
    : ESObject((Type)(Type::ESObject | Type::ESNumberObject), ESVMInstance::currentInstance()->globalObject()->numberPrototype())
{
    m_primitiveValue = value;
}

ESBooleanObject::ESBooleanObject(bool value)
    : ESObject((Type)(Type::ESObject | Type::ESBooleanObject), ESVMInstance::currentInstance()->globalObject()->booleanPrototype())
{
    m_primitiveValue = value;
}

ESErrorObject::ESErrorObject(escargot::ESString* message)
       : ESObject((Type)(Type::ESObject | Type::ESErrorObject), ESVMInstance::currentInstance()->globalObject()->errorPrototype())
{
    set(strings->message, message);
    set(strings->name, strings->Error.string());
    escargot::ESFunctionObject* fn = ESVMInstance::currentInstance()->globalObject()->error();
}

ReferenceError::ReferenceError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->ReferenceError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->referenceErrorPrototype());
}

TypeError::TypeError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->TypeError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->typeErrorPrototype());
}

RangeError::RangeError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->RangeError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->rangeErrorPrototype());
}

SyntaxError::SyntaxError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->SyntaxError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->syntaxErrorPrototype());
}

ESArrayBufferObject::ESArrayBufferObject(ESPointer::Type type)
    : ESObject((Type)(Type::ESObject | Type::ESArrayBufferObject), ESVMInstance::currentInstance()->globalObject()->arrayBufferPrototype()),
    m_data(NULL),
    m_bytelength(0)
{
    set__proto__(ESVMInstance::currentInstance()->globalObject()->arrayBufferPrototype());
}

ESArrayBufferView::ESArrayBufferView(ESPointer::Type type, ESValue __proto__)
       : ESObject((Type)(Type::ESObject | Type::ESArrayBufferView | type), __proto__)
{
}

ESValue ESTypedArrayObjectWrapper::get(int key)
{
    switch (m_arraytype) {
    case TypedArrayType::Int8Array:
        return (reinterpret_cast<ESInt8Array *>(this))->get(key);
    case TypedArrayType::Uint8Array:
        return (reinterpret_cast<ESUint8Array *>(this))->get(key);
    case TypedArrayType::Uint8ClampedArray:
        return (reinterpret_cast<ESUint8ClampedArray *>(this))->get(key);
    case TypedArrayType::Int16Array:
        return (reinterpret_cast<ESInt16Array *>(this))->get(key);
    case TypedArrayType::Uint16Array:
        return (reinterpret_cast<ESUint16Array *>(this))->get(key);
    case TypedArrayType::Int32Array:
        return (reinterpret_cast<ESInt32Array *>(this))->get(key);
    case TypedArrayType::Uint32Array:
        return (reinterpret_cast<ESUint32Array *>(this))->get(key);
    case TypedArrayType::Float32Array:
        return (reinterpret_cast<ESFloat32Array *>(this))->get(key);
    case TypedArrayType::Float64Array:
        return (reinterpret_cast<ESFloat64Array *>(this))->get(key);
    }
    RELEASE_ASSERT_NOT_REACHED();
}
bool ESTypedArrayObjectWrapper::set(int key, ESValue val)
{
    switch (m_arraytype) {
    case TypedArrayType::Int8Array:
        return (reinterpret_cast<ESInt8Array *>(this))->set(key, val);
    case TypedArrayType::Uint8Array:
        return (reinterpret_cast<ESUint8Array *>(this))->set(key, val);
    case TypedArrayType::Uint8ClampedArray:
        return (reinterpret_cast<ESUint8ClampedArray *>(this))->set(key, val);
    case TypedArrayType::Int16Array:
        return (reinterpret_cast<ESInt16Array *>(this))->set(key, val);
    case TypedArrayType::Uint16Array:
        return (reinterpret_cast<ESUint16Array *>(this))->set(key, val);
    case TypedArrayType::Int32Array:
        return (reinterpret_cast<ESInt32Array *>(this))->set(key, val);
    case TypedArrayType::Uint32Array:
        return (reinterpret_cast<ESUint32Array *>(this))->set(key, val);
    case TypedArrayType::Float32Array:
        return (reinterpret_cast<ESFloat32Array *>(this))->set(key, val);
    case TypedArrayType::Float64Array:
        return (reinterpret_cast<ESFloat64Array *>(this))->set(key, val);
    }
    RELEASE_ASSERT_NOT_REACHED();
}


}
