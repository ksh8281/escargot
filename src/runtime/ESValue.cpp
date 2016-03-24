#include "Escargot.h"
#include "ESValue.h"

#include "vm/ESVMInstance.h"
#include "runtime/ExecutionContext.h"
#include "runtime/Environment.h"
#include "ast/AST.h"
#include "jit/ESJIT.h"
#include "bytecode/ByteCode.h"

#include "Yarr.h"

#include "fast-dtoa.h"
#include "bignum-dtoa.h"

namespace escargot {

ESHiddenClassPropertyInfo dummyPropertyInfo(nullptr, true, true, true, true);

ESValue ESValue::toPrimitiveSlowCase(PrimitiveTypeHint preferredType) const
{
    ASSERT(!isPrimitive());
    ESObject* obj = asESPointer()->asESObject();
    if (preferredType == PrimitiveTypeHint::PreferString) {
        ESValue toString = obj->get(ESValue(strings->toString.string()));
        if (toString.isESPointer() && toString.asESPointer()->isESFunctionObject()) {
            ESValue str = ESFunctionObject::call(ESVMInstance::currentInstance(), toString, obj, NULL, 0, false);
            if (str.isPrimitive())
                return str;
        }

        ESValue valueOf = obj->get(ESValue(strings->valueOf.string()));
        if (valueOf.isESPointer() && valueOf.asESPointer()->isESFunctionObject()) {
            ESValue val = ESFunctionObject::call(ESVMInstance::currentInstance(), valueOf, obj, NULL, 0, false);
            if (val.isPrimitive())
                return val;
        }
    } else { // preferNumber
        ESValue valueOf = obj->get(ESValue(strings->valueOf.string()));
        if (valueOf.isESPointer() && valueOf.asESPointer()->isESFunctionObject()) {
            ESValue val = ESFunctionObject::call(ESVMInstance::currentInstance(), valueOf, obj, NULL, 0, false);
            if (val.isPrimitive())
                return val;
        }

        ESValue toString = obj->get(ESValue(strings->toString.string()));
        if (toString.isESPointer() && toString.asESPointer()->isESFunctionObject()) {
            ESValue str = ESFunctionObject::call(ESVMInstance::currentInstance(), toString, obj, NULL, 0, false);
            if (str.isPrimitive())
                return str;
        }
    }
    ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create()));
    RELEASE_ASSERT_NOT_REACHED();
}

ESString* ESValue::toStringSlowCase() const
{
    ASSERT(!isESString());
    if (isInt32()) {
        int num = asInt32();
        if (num >= 0 && num < ESCARGOT_STRINGS_NUMBERS_MAX)
            return strings->numbers[num].string();
        return ESString::create(num);
    } else if (isNumber()) {
        double d = asNumber();
        if (std::isnan(d))
            return strings->NaN.string();
        if (std::isinf(d)) {
            if (std::signbit(d))
                return strings->NegativeInfinity.string();
            else
                return strings->Infinity.string();
        }
        // convert -0.0 into 0.0
        // in c++, d = -0.0, d == 0.0 is true
        if (d == 0.0)
            d = 0;

        return ESString::create(d);
    } else if (isUndefined()) {
        return strings->undefined.string();
    } else if (isNull()) {
        return strings->null.string();
    } else if (isBoolean()) {
        if (asBoolean())
            return strings->stringTrue.string();
        else
            return strings->stringFalse.string();
    } else {
        return toPrimitive(PreferString).toString();
    }
}

bool ESValue::abstractEqualsToSlowCase(const ESValue& val)
{
    if (isNumber() && val.isNumber()) {
        double a = asNumber();
        double b = val.asNumber();

        if (std::isnan(a) || std::isnan(b))
            return false;
        else if (a == b)
            return true;

        return false;
    } else {
        if (isUndefinedOrNull() && val.isUndefinedOrNull())
            return true;

        if (isNumber() && val.isESString()) {
            // If Type(x) is Number and Type(y) is String,
            // return the result of the comparison x == ToNumber(y).
            return asNumber() == val.toNumber();
        } else if (isESString() && val.isNumber()) {
            // If Type(x) is String and Type(y) is Number,
            // return the result of the comparison ToNumber(x) == y.
            return val.asNumber() == toNumber();
        } else if (isBoolean()) {
            // If Type(x) is Boolean, return the result of the comparison ToNumber(x) == y.
            // return the result of the comparison ToNumber(x) == y.
            ESValue x(toNumber());
            return x.abstractEqualsTo(val);
        } else if (val.isBoolean()) {
            // If Type(y) is Boolean, return the result of the comparison x == ToNumber(y).
            // return the result of the comparison ToNumber(x) == y.
            return abstractEqualsTo(ESValue(val.toNumber()));
        } else if ((isESString() || isNumber()) && val.isObject()) {
            // If Type(x) is either String, Number, or Symbol and Type(y) is Object, then
            if (val.asESPointer()->isESDateObject())
                return abstractEqualsTo(val.toPrimitive(ESValue::PreferString));
            else
                return abstractEqualsTo(val.toPrimitive());
        } else if (isObject() && (val.isESString() || val.isNumber())) {
            // If Type(x) is Object and Type(y) is either String, Number, or Symbol, then
            if (asESPointer()->isESDateObject())
                return toPrimitive(ESValue::PreferString).abstractEqualsTo(val);
            else
                return toPrimitive().abstractEqualsTo(val);
        }

        if (isESPointer() && val.isESPointer()) {
            ESPointer* o = asESPointer();
            ESPointer* comp = val.asESPointer();

            if (o->isESString() && comp->isESString())
                return *o->asESString() == *comp->asESString();
            return equalsTo(val);
        }
    }
    return false;
}

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

ASCIIString dtoa(double number)
{
    ASCIIString str;
    if (number == 0) {
        str.append({'0'});
        return std::move(str);
    }
    const int flags = UNIQUE_ZERO | EMIT_POSITIVE_EXPONENT_SIGN;
    bool sign = false;
    if (number < 0) {
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
    if (!fast_worked) {
        BignumDtoa(number, double_conversion::BIGNUM_DTOA_SHORTEST, 0, vector, &decimal_rep_length, &decimal_point);
        vector[decimal_rep_length] = '\0';
    }

    /* reserve(decimal_rep_length + sign ? 1 : 0);
    if (sign)
        operator +=('-');
    for (unsigned i = 0; i < decimal_rep_length; i ++) {
        operator +=(decimal_rep[i]);
    }*/

    const int bufferLength = 128;
    char buffer[bufferLength];
    double_conversion::StringBuilder builder(buffer, bufferLength);

    int exponent = decimal_point - 1;
    const int decimal_in_shortest_low_ = -6;
    const int decimal_in_shortest_high_ = 21;
    if ((decimal_in_shortest_low_ <= exponent)
        && (exponent < decimal_in_shortest_high_)) {
            CreateDecimalRepresentation(flags, decimal_rep, decimal_rep_length,
                decimal_point,
                double_conversion::Max(0, decimal_rep_length - decimal_point),
                &builder);
    } else {
        CreateExponentialRepresentation(flags, decimal_rep, decimal_rep_length, exponent,
            &builder);
    }
    if (sign)
        str += '-';
    char* buf = builder.Finalize();
    while (*buf) {
        str += *buf;
        buf++;
    }
    return std::move(str);
}

uint32_t ESString::tryToUseAsIndex()
{
    if (isASCIIString()) {
        bool allOfCharIsDigit = true;
        uint32_t number = 0;
        size_t len = length();
        const char* data = asciiData();
        for (unsigned i = 0; i < len; i ++) {
            char c = data[i];
            if (c < '0' || c > '9') {
                allOfCharIsDigit = false;
                break;
            } else {
                uint32_t cnum = c-'0';
                if (number > (std::numeric_limits<uint32_t>::max() - cnum) / 10)
                    return ESValue::ESInvalidIndexValue;
                number = number*10 + cnum;
            }
        }
        if (allOfCharIsDigit) {
            return number;
        }
    } else {
        bool allOfCharIsDigit = true;
        uint32_t number = 0;
        size_t len = length();
        const char16_t* data = utf16Data();
        for (unsigned i = 0; i < len; i ++) {
            char16_t c = data[i];
            if (c < '0' || c > '9') {
                allOfCharIsDigit = false;
                break;
            } else {
                uint32_t cnum = c-'0';
                if (number > (std::numeric_limits<uint32_t>::max() - cnum) / 10)
                    return ESValue::ESInvalidIndexValue;
                number = number*10 + cnum;
            }
        }
        if (allOfCharIsDigit) {
            return number;
        }
    }
    return ESValue::ESInvalidIndexValue;
}

ESString* ESString::substring(int from, int to) const
{
    ASSERT(0 <= from && from <= to && to <= (int)length());
    if (to - from == 1) {
        char16_t c;
        c = charAt(from);
        if (c < ESCARGOT_ASCII_TABLE_MAX) {
            return strings->asciiTable[c].string();
        } else {
            return ESString::create(c);
        }
    }

    if (isASCIIString()) {
        return ESString::create(asASCIIString()->substr(from, to-from));
    } else {
        return ESString::create(asUTF16String()->substr(from, to-from));
    }
}

unsigned PropertyDescriptor::defaultAttributes = Configurable | Enumerable | Writable;

PropertyDescriptor::PropertyDescriptor(ESObject* obj)
    : m_value(ESValue::ESEmptyValue)
    , m_getter(ESValue::ESEmptyValue)
    , m_setter(ESValue::ESEmptyValue)
    , m_attributes(0)
    , m_seenAttributes(0)
{
    if (obj->hasProperty(strings->enumerable.string())) {
        setEnumerable(obj->get(strings->enumerable.string()).toBoolean());
    }
    if (obj->hasProperty(strings->configurable.string())) {
        setConfigurable(obj->get(strings->configurable.string()).toBoolean());
    }
    if (obj->hasProperty(strings->writable.string())) {
        setWritable(obj->get(strings->writable.string()).toBoolean());
    }
    if (obj->hasProperty(strings->value.string())) {
        setValue(obj->get(strings->value.string()));
    }
    if (obj->hasProperty(strings->set.string())) {
        setSetter(obj->get(strings->set.string()));
    }
    if (obj->hasProperty(strings->get.string())) {
        setGetter(obj->get(strings->get.string()));
    }
}

bool PropertyDescriptor::writable() const
{
    ASSERT(!isAccessorDescriptor());
    return (m_attributes & Writable);
}

bool PropertyDescriptor::enumerable() const
{
    return (m_attributes & Enumerable);
}

bool PropertyDescriptor::configurable() const
{
    return (m_attributes & Configurable);
}

bool PropertyDescriptor::isDataDescriptor() const
{
    return m_value || (m_seenAttributes & WritablePresent);
}

bool PropertyDescriptor::isGenericDescriptor() const
{
    return !isAccessorDescriptor() && !isDataDescriptor();
}

bool PropertyDescriptor::isAccessorDescriptor() const
{
    return m_getter || m_setter;
}

ESValue PropertyDescriptor::getter() const
{
    ASSERT(isAccessorDescriptor());
    return m_getter;
}

ESValue PropertyDescriptor::setter() const
{
    ASSERT(isAccessorDescriptor());
    return m_setter;
}

ESFunctionObject* PropertyDescriptor::getterObject() const
{
    ASSERT(isAccessorDescriptor() && hasGetter());
    return m_getter.isESPointer() && m_getter.asESPointer()->isESFunctionObject() ? m_getter.asESPointer()->asESFunctionObject() : nullptr;
}

ESFunctionObject* PropertyDescriptor::setterObject() const
{
    ASSERT(isAccessorDescriptor() && hasSetter());
    return m_setter.isESPointer() && m_setter.asESPointer()->isESFunctionObject() ? m_setter.asESPointer()->asESFunctionObject() : nullptr;
}

void PropertyDescriptor::setWritable(bool writable)
{
    if (writable)
        m_attributes |= Writable;
    else
        m_attributes &= ~Writable;
    m_seenAttributes |= WritablePresent;
}

void PropertyDescriptor::setEnumerable(bool enumerable)
{
    if (enumerable)
        m_attributes |= Enumerable;
    else
        m_attributes &= ~Enumerable;
    m_seenAttributes |= EnumerablePresent;
}

void PropertyDescriptor::setConfigurable(bool configurable)
{
    if (configurable)
        m_attributes |= Configurable;
    else
        m_attributes &= ~Configurable;
    m_seenAttributes |= ConfigurablePresent;
}

void PropertyDescriptor::setSetter(ESValue setter)
{
    m_setter = setter;
    m_attributes |= Accessor;
    m_attributes &= ~Writable;
}

void PropertyDescriptor::setGetter(ESValue getter)
{
    m_getter = getter;
    m_attributes |= Accessor;
    m_attributes &= ~Writable;
}

// ES5.1 8.10.4 FromPropertyDescriptor
ESValue PropertyDescriptor::fromPropertyDescriptor(ESObject* descSrc, ESString* propertyName, size_t idx)
{
    bool isActualDataProperty = false;
    if (descSrc->isESArrayObject() && idx == 1) {
        isActualDataProperty = true;
    }
    const ESHiddenClassPropertyInfo& propertyInfo = descSrc->hiddenClass()->propertyInfo(idx);
    ESObject* obj = ESObject::create();
    if (propertyInfo.m_flags.m_isDataProperty || isActualDataProperty) {
        obj->set(strings->value.string(), descSrc->hiddenClass()->read(descSrc, descSrc, propertyName, idx));
        obj->set(strings->writable.string(), ESValue(propertyInfo.m_flags.m_isWritable));
    } else if (descSrc->accessorData(idx)->getJSGetter()
        || descSrc->accessorData(idx)->getJSSetter()
        || (!descSrc->accessorData(idx)->getNativeGetter() && !descSrc->accessorData(idx)->getNativeSetter())) {
        ESObject* getDesc = ESObject::create();
        getDesc->set(strings->value.string(), descSrc->accessorData(idx)->getJSGetter() ? descSrc->accessorData(idx)->getJSGetter() : ESValue());
        getDesc->set(strings->writable.string(), ESValue(true));
        getDesc->set(strings->enumerable.string(), ESValue(true));
        getDesc->set(strings->configurable.string(), ESValue(true));
        ESValue getStr = strings->get.string();
        obj->defineOwnProperty(getStr, getDesc, false);

        ESObject* setDesc = ESObject::create();
        setDesc->set(strings->value.string(), descSrc->accessorData(idx)->getJSSetter() ? descSrc->accessorData(idx)->getJSSetter() : ESValue());
        setDesc->set(strings->writable.string(), ESValue(true));
        setDesc->set(strings->enumerable.string(), ESValue(true));
        setDesc->set(strings->configurable.string(), ESValue(true));
        ESValue setStr = strings->set.string();
        obj->defineOwnProperty(setStr, setDesc, false);
    } else {
        obj->set(strings->value.string(), descSrc->hiddenClass()->read(descSrc, descSrc, propertyName, idx));
        descSrc->accessorData(idx)->setGetterAndSetterTo(obj, &propertyInfo);
    }
    obj->set(strings->enumerable.string(), ESValue(propertyInfo.m_flags.m_isEnumerable));
    obj->set(strings->configurable.string(), ESValue(propertyInfo.m_flags.m_isConfigurable));
    return obj;
}

// For ArrayFastMode
ESValue PropertyDescriptor::fromPropertyDescriptorForIndexedProperties(ESObject* obj, uint32_t index)
{
    if (obj->isESArrayObject() && obj->asESArrayObject()->isFastmode()) {
        if (index != ESValue::ESInvalidIndexValue) {
            if (LIKELY(index < obj->asESArrayObject()->length())) {
                ESValue e = obj->asESArrayObject()->data()[index];
                if (LIKELY(!e.isEmpty())) {
                    ESObject* ret = ESObject::create();
                    ret->set(strings->value.string(), e);
                    ret->set(strings->writable.string(), ESValue(true));
                    ret->set(strings->enumerable.string(), ESValue(true));
                    ret->set(strings->configurable.string(), ESValue(true));
                    return ret;
                }
            }
        }
    }
    if (obj->isESStringObject()) {
        if (index != ESValue::ESInvalidIndexValue) {
            if (LIKELY(index < obj->asESStringObject()->length())) {
                ESValue e = obj->asESStringObject()->getCharacterAsString(index);
                ESObject* ret = ESObject::create();
                ret->set(strings->value.string(), e);
                ret->set(strings->writable.string(), ESValue(false));
                ret->set(strings->enumerable.string(), ESValue(true));
                ret->set(strings->configurable.string(), ESValue(false));
                return ret;
            }
        }
    }
    return ESValue();
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
    m_flags.m_extraData = 0;

    m_extraPointerData = NULL;

    m_hiddenClassData.reserve(initialKeyCount);
    m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForObject();

    m_hiddenClassData.push_back(ESValue((ESPointer *)ESVMInstance::currentInstance()->object__proto__AccessorData()));

    set__proto__(__proto__);
}

void ESObject::setValueAsProtoType(const ESValue& obj)
{
    if (obj.isObject()) {
        obj.asESPointer()->asESObject()->m_flags.m_isEverSetAsPrototypeObject = true;
        obj.asESPointer()->asESObject()->m_hiddenClass->setHasEverSetAsPrototypeObjectHiddenClass();
        if (obj.asESPointer()->isESArrayObject()) {
            obj.asESPointer()->asESArrayObject()->convertToSlowMode();
        }
        if (obj.asESPointer()->asESObject()->hiddenClass()->hasIndexedProperty()) {
            ESVMInstance::currentInstance()->globalObject()->somePrototypeObjectDefineIndexedProperty();
        }
    }
}

bool ESObject::defineOwnProperty(const ESValue& P, const PropertyDescriptor& desc, bool throwFlag)
{
    ESObject* O = this;

    // ToPropertyDescriptor : (start) we need to seperate this part
    bool descHasEnumerable = desc.hasEnumerable();
    bool descHasConfigurable = desc.hasConfigurable();
    bool descHasWritable = desc.hasWritable();
    bool descHasValue = desc.hasValue();
    bool descHasGetter = desc.hasGetter();
    bool descHasSetter = desc.hasSetter();

    escargot::ESFunctionObject* descGet = NULL;
    escargot::ESFunctionObject* descSet = NULL;
    if (descHasGetter || descHasSetter) {
        if (descHasGetter) {
            ESValue get = desc.getter(); // 8.10.5 ToPropertyDescriptor 7.a
            if (!(get.isESPointer() && get.asESPointer()->isESFunctionObject()) && !get.isUndefined())
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ToPropertyDescriptor 7.b")))); // 8.10.5 ToPropertyDescriptor 7.b
            else if (!get.isUndefined())
                descGet = get.asESPointer()->asESFunctionObject(); // 8.10.5 ToPropertyDescriptor 7.c
        }
        if (descHasSetter) {
            ESValue set = desc.setter(); // 8.10.5 ToPropertyDescriptor 8.a
            if (!(set.isESPointer() && set.asESPointer()->isESFunctionObject()) && !set.isUndefined())
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ToPropertyDescriptor 8.b")))); // 8.10.5 ToPropertyDescriptor 8.b
            else if (!set.isUndefined())
                descSet = set.asESPointer()->asESFunctionObject(); // 8.10.5 ToPropertyDescriptor 8.c
        }
        if (descHasValue || descHasWritable)
            ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, Property cannot have [getter|setter] and [value|writable] together"))));
    }
    // ToPropertyDescriptor : (end)

    // 1
    // Our ESObject::getOwnProperty differs from [[GetOwnProperty]] in Spec
    // Hence, we use OHasCurrent and propertyInfo of current instead of Property Descriptor which is return of [[GetOwnProperty]] here.
    bool OHasCurrent = true;
    size_t idx = O->hiddenClass()->findProperty(P.toString());
    ESValue current = ESValue();
    if (idx != SIZE_MAX)
        current = O->hiddenClass()->read(O, O, P.toString(), idx);
    else {
        if (O->isESArrayObject()
            && (descHasEnumerable || descHasWritable || descHasConfigurable || descHasValue || descHasGetter || descHasSetter)
            && O->hasOwnProperty(P)) {
            current = O->getOwnProperty(P);
            if (descHasGetter || descHasSetter) {
                O->defineAccessorProperty(P, new ESPropertyAccessorData(descGet, descSet),
                    descHasWritable ? desc.writable() : true,
                    descHasEnumerable ? desc.enumerable() : true,
                    descHasConfigurable ? desc.configurable() : true);
            } else {
                O->defineDataProperty(P, descHasWritable ? desc.writable() : true,
                    descHasEnumerable ? desc.enumerable() : true,
                    descHasConfigurable ? desc.configurable() : true, descHasValue ? desc.value() : current);
            }
            return true;
        } else {
            current = O->getOwnProperty(P);
            if (current.isUndefined())
                OHasCurrent = false;
        }
    }

    // 2
    bool extensible = O->isExtensible();

    // 3, 4
    bool isDescDataDescriptor = desc.isDataDescriptor();
    bool isDescGenericDescriptor = desc.isGenericDescriptor();
    if (!OHasCurrent) {
        // 3
        if (!extensible) {
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty"))));
            else
                return false;
        } else { // 4
//            O->deleteProperty(P);
            if (isDescDataDescriptor || isDescGenericDescriptor) {
                // Refer to Table 7 of ES 5.1 for default attribute values
                O->defineDataProperty(P, descHasWritable ? desc.writable() : false,
                    descHasEnumerable ? desc.enumerable() : false,
                    descHasConfigurable ? desc.configurable() : false, desc.value());
            } else {
                ASSERT(desc.isAccessorDescriptor());
                O->defineAccessorProperty(P, new ESPropertyAccessorData(descGet, descSet),
                    descHasWritable ? desc.writable() : false,
                    descHasEnumerable ? desc.enumerable() : false,
                    descHasConfigurable ? desc.configurable() : false);
            }
            return true;
        }
    }

    // 5
    if (!descHasEnumerable && !descHasWritable && !descHasConfigurable && !descHasValue && !descHasGetter && !descHasSetter)
        return true;

    // 6
    idx = O->hiddenClass()->findProperty(P.toString());
    const ESHiddenClassPropertyInfo& propertyInfo = O->hiddenClass()->propertyInfo(idx);
    if ((!descHasEnumerable || desc.enumerable() == propertyInfo.m_flags.m_isEnumerable)
        && (!descHasWritable || ((propertyInfo.m_flags.m_isDataProperty || O->accessorData(idx)->getNativeGetter() || O->accessorData(idx)->getNativeSetter()) && (desc.writable() == propertyInfo.m_flags.m_isWritable)))
        && (!descHasConfigurable || desc.configurable() == propertyInfo.m_flags.m_isConfigurable)
        && (!descHasValue || ((propertyInfo.m_flags.m_isDataProperty || O->accessorData(idx)->getNativeGetter() || O->accessorData(idx)->getNativeSetter()) && desc.value().equalsToByTheSameValueAlgorithm(current)))
        && (!descHasGetter || (O->get(strings->get.string()).isESPointer() && O->get(strings->get.string()).asESPointer()->isESFunctionObject()
            && descGet == O->get(strings->get.string()).asESPointer()->asESFunctionObject()))
        && (!descHasSetter || (O->get(strings->set.string()).isESPointer() && O->get(strings->set.string()).asESPointer()->isESFunctionObject()
            && descSet == O->get(strings->set.string()).asESPointer()->asESFunctionObject())))
        return true;

    // 7
    if (!propertyInfo.m_flags.m_isConfigurable) {
        if (descHasConfigurable && desc.configurable()) {
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 7.a"))));
            else
                return false;
        } else {
            if (descHasEnumerable && propertyInfo.m_flags.m_isEnumerable != desc.enumerable()) {
                if (throwFlag)
                    ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 7.b"))));
                else
                    return false;
            }
        }
    }

    // 8, 9, 10, 11
    bool isCurrentDataDescriptor = propertyInfo.m_flags.m_isDataProperty || O->accessorData(idx)->getNativeGetter() || O->accessorData(idx)->getNativeSetter();
    bool shouldRemoveOriginalNativeGetterSetter = false;
    if (isDescGenericDescriptor) { // 8
    } else if (isCurrentDataDescriptor != isDescDataDescriptor) { // 9
        if (!propertyInfo.m_flags.m_isConfigurable) { // 9.a
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Object.DefineOwnProperty 9.a"))));
            else
                return false;
        }
        if (isCurrentDataDescriptor) { // 9.b
            O->deleteProperty(P);
            O->defineAccessorProperty(P, new ESPropertyAccessorData(descGet, descSet), descHasWritable ? desc.writable() : false, descHasEnumerable ? desc.enumerable() : propertyInfo.m_flags.m_isEnumerable, descHasConfigurable ? desc.configurable() : propertyInfo.m_flags.m_isConfigurable, true);
        } else { // 9.c
            O->deleteProperty(P);
            O->defineDataProperty(P, descHasWritable ? desc.writable() : false, descHasEnumerable ? desc.enumerable() : propertyInfo.m_flags.m_isEnumerable, descHasConfigurable ? desc.configurable() : propertyInfo.m_flags.m_isConfigurable, desc.value(), true);
        }
        return true;
    } else if (isCurrentDataDescriptor && isDescDataDescriptor) { // 10
        if (!propertyInfo.m_flags.m_isConfigurable) {
            if (!propertyInfo.m_flags.m_isWritable) {
                if (desc.writable()) {
                    if (throwFlag)
                        ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 10.a.i"))));
                    else
                        return false;
                } else {
                    if (descHasValue && current != desc.value()) {
                        if (throwFlag)
                            ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 10.a.ii"))));
                        else
                            return false;
                    }
                }
            }
        }
        if (UNLIKELY(O->isESArgumentsObject() && !propertyInfo.m_flags.m_isDataProperty)) { // ES6.0 $9.4.4.2
            O->set(P, desc.value());
            shouldRemoveOriginalNativeGetterSetter = true;
        }
    } else {
        ASSERT(!propertyInfo.m_flags.m_isDataProperty && desc.isAccessorDescriptor());
        if (!propertyInfo.m_flags.m_isConfigurable) {
            if (descHasSetter && (descSet != O->accessorData(idx)->getJSSetter())) {
                if (throwFlag)
                    ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 11.a.i"))));
                else
                    return false;
            }
            if (descHasGetter && (descGet != O->accessorData(idx)->getJSGetter())) {
                if (throwFlag)
                    ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, DefineOwnProperty 11.a.ii"))));
                else
                    return false;
            }
        }
    }

    //

    // 12
    if (descHasGetter || descHasSetter || (!propertyInfo.m_flags.m_isDataProperty && !O->accessorData(idx)->getNativeGetter() && !O->accessorData(idx)->getNativeSetter())) {
        escargot::ESFunctionObject* getter = descGet;
        escargot::ESFunctionObject* setter = descSet;
        if (!propertyInfo.m_flags.m_isDataProperty) {
            ESPropertyAccessorData* currentAccessorData = O->accessorData(idx);
            if (!descHasGetter && currentAccessorData->getJSGetter()) {
                getter = currentAccessorData->getJSGetter();
            }
            if (!descHasSetter && currentAccessorData->getJSSetter()) {
                setter = currentAccessorData->getJSSetter();
            }
        }
        O->deleteProperty(P, true);
        O->defineAccessorProperty(P, new ESPropertyAccessorData(getter, setter),
            descHasWritable ? desc.writable() : propertyInfo.m_flags.m_isWritable,
            descHasEnumerable ? desc.enumerable(): propertyInfo.m_flags.m_isEnumerable,
            descHasConfigurable ? desc.configurable() : propertyInfo.m_flags.m_isConfigurable, true);
    } else if (!propertyInfo.m_flags.m_isDataProperty && (O->accessorData(idx)->getNativeGetter() || O->accessorData(idx)->getNativeSetter()) && !shouldRemoveOriginalNativeGetterSetter) {
        escargot::ESNativeGetter getter = O->accessorData(idx)->getNativeGetter();
        escargot::ESNativeSetter setter = O->accessorData(idx)->getNativeSetter();
        O->set(P, descHasValue ? desc.value() : current);

        O->deleteProperty(P, true);
        O->defineAccessorProperty(P, new ESPropertyAccessorData(getter, setter),
            descHasWritable ? desc.writable() : propertyInfo.m_flags.m_isWritable,
            descHasEnumerable ? desc.enumerable(): propertyInfo.m_flags.m_isEnumerable,
            descHasConfigurable ? desc.configurable() : propertyInfo.m_flags.m_isConfigurable, true);

    } else {
        O->deleteProperty(P, true);
        O->defineDataProperty(P,
            descHasWritable ? desc.writable() : propertyInfo.m_flags.m_isWritable,
            descHasEnumerable ? desc.enumerable(): propertyInfo.m_flags.m_isEnumerable,
            descHasConfigurable ? desc.configurable() : propertyInfo.m_flags.m_isConfigurable,
            descHasValue ? desc.value() : current, true);
    }

    // 13
    return true;
}

// ES 5.1: 8.12.9
bool ESObject::defineOwnProperty(const ESValue& P, ESObject* obj, bool throwFlag)
{
    return defineOwnProperty(P, PropertyDescriptor { obj }, throwFlag);
}

const unsigned ESArrayObject::MAX_FASTMODE_SIZE;

ESArrayObject::ESArrayObject(int length)
    : ESObject((Type)(Type::ESObject | Type::ESArrayObject), ESVMInstance::currentInstance()->globalObject()->arrayPrototype(), 3)
    , m_vector(0)
{
    if (!ESVMInstance::currentInstance()->globalObject()->didSomePrototypeObjectDefineIndexedProperty())
        m_flags.m_isFastMode = true;
    m_length = 0;
    if (length == -1)
        convertToSlowMode();
    else if (length > 0) {
        setLength(length);
    } else {
        m_vector.reserve(6);
    }

    // defineAccessorProperty(strings->length.string(), ESVMInstance::currentInstance()->arrayLengthAccessorData(), true, false, false);
    m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForArrayObject();
    m_hiddenClassData.push_back((ESPointer *)ESVMInstance::currentInstance()->arrayLengthAccessorData());
}

bool ESArrayObject::defineOwnProperty(const ESValue& P, const PropertyDescriptor& desc, bool throwFlag)
{
    ESArrayObject* A = this;
    ESObject* O = this;
    uint32_t index;

    // ToPropertyDescriptor : (start) we need to seperate this part
    bool descHasEnumerable = desc.hasEnumerable();
    bool descHasConfigurable = desc.hasConfigurable();
    bool descHasWritable = desc.hasWritable();
    bool descHasValue = desc.hasValue();
    bool descHasGetter = desc.hasGetter();
    bool descHasSetter = desc.hasSetter();

    escargot::ESFunctionObject* descGet = NULL;
    escargot::ESFunctionObject* descSet = NULL;
    if (descHasGetter || descHasSetter) {
        if (descHasGetter) {
            ESValue get = desc.getter(); // 8.10.5 ToPropertyDescriptor 7.a
            if (!(get.isESPointer() && get.asESPointer()->isESFunctionObject()) && !get.isUndefined())
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ToPropertyDescriptor 7.b")))); // 8.10.5 ToPropertyDescriptor 7.b
            else if (!get.isUndefined())
                descGet = get.asESPointer()->asESFunctionObject(); // 8.10.5 ToPropertyDescriptor 7.c
        }
        if (descHasSetter) {
            ESValue set = desc.setter(); // 8.10.5 ToPropertyDescriptor 8.a
            if (!(set.isESPointer() && set.asESPointer()->isESFunctionObject()) && !set.isUndefined())
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ToPropertyDescriptor 8.b")))); // 8.10.5 ToPropertyDescriptor 8.b
            else if (!set.isUndefined())
                descSet = set.asESPointer()->asESFunctionObject(); // 8.10.5 ToPropertyDescriptor 8.c
        }
        if (descHasValue || descHasWritable)
            ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("Type error, Property cannot have [getter|setter] and [value|writable] together"))));
    }
    // ToPropertyDescriptor : (end)

    // 1
    ESValue oldLenDesc = A->getOwnProperty(strings->length.string());
    ASSERT(!oldLenDesc.isUndefined());
//    ASSERT(!escargot::PropertyDescriptor::IsAccessorDescriptor(oldLenDesc)); // Our implementation differs from Spec so that length property is accessor property.
    size_t lengthIdx = O->hiddenClass()->findProperty(strings->length.string());
//    ASSERT(O->hiddenClass()->findProperty(strings->length.string()) == 1);
    const ESHiddenClassPropertyInfo& oldLePropertyInfo = O->hiddenClass()->propertyInfo(lengthIdx);

    // 2
    size_t oldLen = oldLenDesc.toUint32();

    // 3
    ESValue lenStr = strings->length.string();
    if (*P.toString() == *strings->length.string()) {
        ESValue descV = desc.value();
        // 3.a
        if (!descHasValue) {
            return A->asESObject()->defineOwnProperty(P, desc, throwFlag);
        }
        // 3.b
        ESObject* newLenDesc = ESObject::create();
        if (descHasEnumerable)
            newLenDesc->set(strings->enumerable.string(), ESValue(desc.enumerable()));
        if (descHasWritable)
            newLenDesc->set(strings->writable.string(), ESValue(desc.writable()));
        if (descHasConfigurable)
            newLenDesc->set(strings->configurable.string(), ESValue(desc.configurable()));
        if (descHasValue)
            newLenDesc->set(strings->value.string(), descV);
        if (descHasGetter)
            newLenDesc->set(strings->get.string(), descGet);
        if (descHasSetter)
            newLenDesc->set(strings->set.string(), descSet);

        // 3.c
        uint32_t newLen = descV.toUint32();

        // 3.d
        if (newLen != descV.toNumber())
            ESVMInstance::currentInstance()->throwError(ESValue(RangeError::create(ESString::create("ArrayObject.DefineOwnProperty 3.d"))));

        // 3.e
        newLenDesc->set(strings->value.string(), ESValue(newLen));

        // 3.f
        if (newLen >= oldLen)
            return A->asESObject()->defineOwnProperty(P, newLenDesc, throwFlag);

        // 3.g
        if (!oldLePropertyInfo.m_flags.m_isWritable) {
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ArrayObject.DefineOwnProperty 3.g"))));
            else
                return false;
        }

        // 3.h
        bool newLenDescHasWritable = newLenDesc->hasProperty(strings->writable.string());
        bool newLenDescW = newLenDesc->get(strings->writable.string()).toBoolean();
        bool newWritable;
        if (!newLenDescHasWritable || newLenDescW)
            newWritable = true;
        else {
            newWritable = false;
            newLenDesc->set(strings->writable.string(), ESValue(true));
        }

        // 3.j
        bool succeeded = A->asESObject()->defineOwnProperty(P, newLenDesc, throwFlag);

        // 3.k
        if (!succeeded)
            return false;

        // 3.l
        while (newLen < oldLen) {
            oldLen--;
            bool deleteSucceeded = A->deleteProperty(ESValue(oldLen).toString());
            if (!deleteSucceeded) {
                newLenDesc->set(strings->value.string(), ESValue(oldLen+1));
                if (!newWritable)
                    newLenDesc->set(strings->writable.string(), ESValue(false));
                A->asESObject()->defineOwnProperty(P, newLenDesc, false);
                if (throwFlag)
                    ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ArrayObject.DefineOwnProperty 3.l.iii"))));
                else
                    return false;
            }
        }

        // 3.m
        if (!newWritable) {
            ESObject* descWritableIsFalse = ESObject::create();
            descWritableIsFalse->set(strings->writable.string(), ESValue(false));
            A->asESObject()->defineOwnProperty(P, descWritableIsFalse, false);
            return true;
        }
        return true;
    } else if ((index = P.toIndex()) != ESValue::ESInvalidIndexValue) { // 4
        // 4.a

        // 4.b
        if (index >= oldLen && !oldLePropertyInfo.m_flags.m_isWritable) {
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ArrayObject.DefineOwnProperty 4.b"))));
            else
                return false;
        }

        // 4.c
        bool succeeded = A->asESObject()->defineOwnProperty(P, desc, false);

        // 4.d
        if (!succeeded) {
            if (throwFlag)
                ESVMInstance::currentInstance()->throwError(ESValue(TypeError::create(ESString::create("ArrayObject.DefineOwnProperty 4.d"))));
            else
                return false;
        }

        // 4.e
        if (index >= oldLen) {
            ESObject* oldLenDescAsObject = ESObject::create();
            oldLenDescAsObject->set(strings->value.string(), ESValue(index + 1));
            A->asESObject()->defineOwnProperty(lenStr, oldLenDescAsObject, false);
        }

        // 4.f
        return true;
    }

    // 5
    return A->asESObject()->defineOwnProperty(P, desc, throwFlag);
}

// ES 5.1: 15.4.5.1
bool ESArrayObject::defineOwnProperty(const ESValue& P, ESObject* obj, bool throwFlag)
{
    return defineOwnProperty(P, PropertyDescriptor { obj }, throwFlag);
}

void ESArrayObject::setLength(unsigned newLength)
{
    if (m_flags.m_isFastMode) {
        if (shouldConvertToSlowMode(newLength)) {
            convertToSlowMode();
            ESObject::set(strings->length, ESValue(newLength));
            m_length = newLength;
            return;
        }
        if (newLength < m_length) {
            m_vector.resize(newLength);
        } else if (newLength > m_length) {
            if (m_vector.capacity() < newLength) {
                size_t reservedSpace = std::min(MAX_FASTMODE_SIZE, (unsigned)(newLength*1.5f));
                m_vector.reserve(reservedSpace);
            }
            m_vector.resize(newLength, ESValue(ESValue::ESEmptyValue));
        }
    } else {
        unsigned currentLength = m_length;
        if (newLength < currentLength) {
            std::vector<unsigned> indexes;
            enumerationWithNonEnumerable([&](ESValue key, ESHiddenClassPropertyInfo* propertyInfo) {
                uint32_t index = key.toIndex();
                if (index != ESValue::ESInvalidIndexValue) {
                    if (index >= newLength && index < currentLength)
                        indexes.push_back(index);
                }
            });
            std::sort(indexes.begin(), indexes.end(), std::greater<uint32_t>());
            for (auto index : indexes) {
                if (deleteProperty(ESValue(index))) {
                    m_length--;
                    continue;
                }
                m_length = index + 1;
                if (ESVMInstance::currentInstance()->currentExecutionContext()->isStrictMode()) {
                    ESVMInstance::currentInstance()->throwError(TypeError::create(ESString::create(u"Unable to delete array property while setting array length")));
                }
                return;
            }
        }
    }
    m_length = newLength;
}

ESRegExpObject* ESRegExpObject::create(const ESValue patternValue, const ESValue optionValue)
{
    if (patternValue.isESPointer() && patternValue.asESPointer()->isESRegExpObject()) {
        if (optionValue.isUndefined()) {
            return patternValue.asESPointer()->asESRegExpObject();
        }
        ESVMInstance::currentInstance()->throwError(TypeError::create(ESString::create(u"Cannot supply flags when constructing one RegExp from another")));
    }

    ESRegExpObject* ret = ESRegExpObject::create(strings->defaultRegExpString.string(), ESRegExpObject::Option::None);
    ESRegExpObject::Option option = ESRegExpObject::Option::None;
    escargot::ESString* patternStr = patternValue.toString();
    escargot::ESString* optionStr = optionValue.isUndefined()? strings->emptyString.string(): optionValue.toString();
    for (size_t i = 0; i < optionStr->length(); i++) {
        switch (optionStr->charAt(i)) {
        case 'g':
            if (option & ESRegExpObject::Option::Global)
                ESVMInstance::currentInstance()->throwError(SyntaxError::create(ESString::create(u"RegExp has multiple 'g' flags")));
            option = (ESRegExpObject::Option) (option | ESRegExpObject::Option::Global);
            break;
        case 'i':
            if (option & ESRegExpObject::Option::IgnoreCase)
                ESVMInstance::currentInstance()->throwError(SyntaxError::create(ESString::create(u"RegExp has multiple 'i' flags")));
            option = (ESRegExpObject::Option) (option | ESRegExpObject::Option::IgnoreCase);
            break;
        case 'm':
            if (option & ESRegExpObject::Option::MultiLine)
                ESVMInstance::currentInstance()->throwError(SyntaxError::create(ESString::create(u"RegExp has multiple 'm' flags")));
            option = (ESRegExpObject::Option) (option | ESRegExpObject::Option::MultiLine);
            break;
        case 'y':
            if (option & ESRegExpObject::Option::Sticky)
                ESVMInstance::currentInstance()->throwError(SyntaxError::create(ESString::create(u"RegExp has multiple 'y' flags")));
            option = (ESRegExpObject::Option) (option | ESRegExpObject::Option::Sticky);
            break;
        default:
            ESVMInstance::currentInstance()->throwError(SyntaxError::create(ESString::create(u"RegExp has invalid flag")));
        }
    }

    if (option != ESRegExpObject::Option::None)
        ret->setOption(option);

    if (patternValue.isUndefined()) {
        ret->setSource(ESString::create("(?:)"));
    } else {
        bool success = ret->setSource(patternStr);
        if (!success)
            ESVMInstance::currentInstance()->throwError(ESValue(SyntaxError::create(ESString::create(u"RegExp has invalid source"))));
    }

    return ret;
}

ESRegExpObject::ESRegExpObject(escargot::ESString* source, const Option& option)
    : ESObject((Type)(Type::ESObject | Type::ESRegExpObject), ESVMInstance::currentInstance()->globalObject()->regexpPrototype())
{
    m_source = source;
    m_option = option;
    m_yarrPattern = NULL;
    m_bytecodePattern = NULL;
    m_lastIndex = ESValue(0);
    m_lastExecutedString = NULL;

    defineAccessorProperty(strings->source, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName) -> ESValue {
        return self->asESRegExpObject()->source();
    }, nullptr, false, false, false);

    defineAccessorProperty(strings->ignoreCase, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName) -> ESValue {
        return ESValue((bool)(self->asESRegExpObject()->option() & ESRegExpObject::Option::IgnoreCase));
    }, nullptr, false, false, false);

    defineAccessorProperty(strings->global, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName) -> ESValue {
        return ESValue((bool)(self->asESRegExpObject()->option() & ESRegExpObject::Option::Global));
    }, nullptr, false, false, false);

    defineAccessorProperty(strings->multiline, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName) -> ESValue {
        return ESValue((bool)(self->asESRegExpObject()->option() & ESRegExpObject::Option::MultiLine));
    }, nullptr, false, false, false);

    defineAccessorProperty(strings->lastIndex, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName) -> ESValue {
        return self->asESRegExpObject()->lastIndex();
    }, [](ESObject* self, ESObject* originalObj, ::escargot::ESString* propertyName, const ESValue& index)
    {
        self->asESRegExpObject()->setLastIndex(index);
    }, true, false, false);
}

bool ESRegExpObject::setSource(escargot::ESString* src)
{
    m_bytecodePattern = NULL;
    m_source = src;
    const char* yarrError = nullptr;
    m_yarrPattern = new JSC::Yarr::YarrPattern(*src, m_option & ESRegExpObject::Option::IgnoreCase, m_option & ESRegExpObject::Option::MultiLine, &yarrError);
    if (yarrError)
        return false;
    return true;
}
void ESRegExpObject::setOption(const Option& option)
{
    ASSERT(!yarrPattern());
    if (((m_option & ESRegExpObject::Option::MultiLine) != (option & ESRegExpObject::Option::MultiLine))
        || ((m_option & ESRegExpObject::Option::IgnoreCase) != (option & ESRegExpObject::Option::IgnoreCase))
        ) {
        m_bytecodePattern = NULL;
    }
    m_option = option;
}

bool ESRegExpObject::match(const escargot::ESString* str, RegexMatchResult& matchResult, bool testOnly, size_t startIndex)
{
    JSC::Yarr::BytecodePattern* byteCode = bytecodePattern();

    bool isGlobal = option() & ESRegExpObject::Option::Global;
    if (!byteCode) {
        const char* yarrError = nullptr;
        JSC::Yarr::YarrPattern* cachedYarrPattern;
        if (this->yarrPattern())
            cachedYarrPattern = this->yarrPattern();
        else
            cachedYarrPattern = new JSC::Yarr::YarrPattern(*source(), option() & ESRegExpObject::Option::IgnoreCase, option() & ESRegExpObject::Option::MultiLine, &yarrError);
        if (yarrError) {
            matchResult.m_subPatternNum = 0;
            return false;
        }
        WTF::BumpPointerAllocator *bumpAlloc = ESVMInstance::currentInstance()->bumpPointerAllocator();
        JSC::Yarr::OwnPtr<JSC::Yarr::BytecodePattern> ownedBytecode = JSC::Yarr::byteCompile(*cachedYarrPattern, bumpAlloc);
        byteCode = ownedBytecode.leakPtr();
        setBytecodePattern(byteCode);
    }

    unsigned subPatternNum = byteCode->m_body->m_numSubpatterns;
    matchResult.m_subPatternNum = (int) subPatternNum;
    size_t length = str->length();
    size_t start = startIndex;
    unsigned result = 0;
    const void* chars;
    if (str->isASCIIString())
        chars = str->asciiData();
    else
        chars = str->utf16Data();
    unsigned* outputBuf = (unsigned int*)alloca(sizeof(unsigned) * 2 * (subPatternNum + 1));
    outputBuf[1] = start;
    do {
        start = outputBuf[1];
        memset(outputBuf, -1, sizeof(unsigned) * 2 * (subPatternNum + 1));
        if (start > length)
            break;
        if (str->isASCIIString())
            result = JSC::Yarr::interpret(byteCode, (const char *)chars, length, start, outputBuf);
        else
            result = JSC::Yarr::interpret(byteCode, (const char16_t *)chars, length, start, outputBuf);
        if (result != JSC::Yarr::offsetNoMatch) {
            if (UNLIKELY(testOnly)) {
                return true;
            }
            std::vector<RegexMatchResult::RegexMatchResultPiece, pointer_free_allocator<RegexMatchResult::RegexMatchResultPiece> > piece;
            piece.reserve(subPatternNum + 1);

            for (unsigned i = 0; i < subPatternNum + 1; i ++) {
                RegexMatchResult::RegexMatchResultPiece p;
                p.m_start = outputBuf[i*2];
                p.m_end = outputBuf[i*2 + 1];
                piece.push_back(p);
            }
            matchResult.m_matchResults.push_back(std::move(piece));
            if (!isGlobal)
                break;
            if (start == outputBuf[1]) {
                outputBuf[1]++;
                if (outputBuf[1] > length) {
                    break;
                }
            }
        } else {
            break;
        }
    } while (result != JSC::Yarr::offsetNoMatch);
    return matchResult.m_matchResults.size();
}

ESArrayObject* ESRegExpObject::createRegExpMatchedArray(const RegexMatchResult& result, const escargot::ESString* input)
{
    escargot::ESArrayObject* arr = escargot::ESArrayObject::create();

    arr->defineOwnProperty(strings->index.string(),
        PropertyDescriptor { ESValue(result.m_matchResults[0][0].m_start), Writable | Enumerable | Configurable }, true);
    arr->defineOwnProperty(strings->input.string(),
        PropertyDescriptor { input, Writable | Enumerable | Configurable }, true);

    int idx = 0;
    for (unsigned i = 0; i < result.m_matchResults.size() ; i ++) {
        for (unsigned j = 0; j < result.m_matchResults[i].size() ; j ++) {
            if (result.m_matchResults[i][j].m_start == std::numeric_limits<unsigned>::max()) {
                arr->defineOwnProperty(ESValue(idx++),
                    PropertyDescriptor { ESValue(ESValue::ESUndefined), Writable | Enumerable | Configurable }, true);
            } else {
                arr->defineOwnProperty(ESValue(idx++),
                    PropertyDescriptor { input->substring(result.m_matchResults[i][j].m_start, result.m_matchResults[i][j].m_end),
                    Writable | Enumerable | Configurable },  true);
            }
        }
    }
    return arr;
}

ESFunctionObject::ESFunctionObject(LexicalEnvironment* outerEnvironment, CodeBlock* cb, escargot::ESString* name, unsigned length, bool hasPrototype, bool isBuiltIn)
    : ESObject((Type)(Type::ESObject | Type::ESFunctionObject), ESVMInstance::currentInstance()->globalFunctionPrototype(), 4)
{
    m_name = name;
    m_outerEnvironment = outerEnvironment;
    m_codeBlock = cb;
    m_flags.m_nonConstructor = false;
    m_protoType = ESObject::create(2);

    // m_protoType.asESPointer()->asESObject()->defineDataProperty(strings->constructor.string(), true, false, true, this);
    m_protoType.asESPointer()->asESObject()->m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForPrototypeObject();
    m_protoType.asESPointer()->asESObject()->m_hiddenClassData.push_back(this);

    // $19.2.4 Function Instances
    // these define in ESVMInstance::ESVMInstance()
    // defineDataProperty(strings->length, false, false, true, ESValue(length));
    // defineAccessorProperty(strings->prototype.string(), ESVMInstance::currentInstance()->functionPrototypeAccessorData(), true, false, false);
    // defineDataProperty(strings->name.string(), false, false, true, name);
    if (hasPrototype && !isBuiltIn) {
        m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForFunctionObject();
        m_hiddenClassData.push_back(ESValue(length));
        m_hiddenClassData.push_back(ESValue((ESPointer *)ESVMInstance::currentInstance()->functionPrototypeAccessorData()));
        m_hiddenClassData.push_back(ESValue(name));
    } else {
        m_hiddenClass = ESVMInstance::currentInstance()->initialHiddenClassForFunctionObjectWithoutPrototype();
        m_hiddenClassData.push_back(ESValue(length));
        m_hiddenClassData.push_back(ESValue(name));
    }
    if (cb && cb->shouldUseStrictMode()) {
        defineAccessorProperty(strings->arguments.string(), ESVMInstance::currentInstance()->throwerAccessorData(), false, false, false);
        defineAccessorProperty(strings->caller.string(), ESVMInstance::currentInstance()->throwerAccessorData(), false, false, false);
    }
    m_flags.m_isBoundFunction = false;
}

ESFunctionObject::ESFunctionObject(LexicalEnvironment* outerEnvironment, NativeFunctionType fn, escargot::ESString* name, unsigned length, bool isConstructor, bool isBuiltIn)
    : ESFunctionObject(outerEnvironment, (CodeBlock *)NULL, name, length, isConstructor, isBuiltIn)
{
    m_codeBlock = CodeBlock::create(CodeBlock::ExecutableType::FunctionCode, 0, true);
    m_codeBlock->m_hasCode = true;
    m_codeBlock->pushCode(ExecuteNativeFunction(fn));
#ifndef NDEBUG
    m_codeBlock->m_nonAtomicId = name;
#endif
#ifdef ENABLE_ESJIT
    m_codeBlock->m_dontJIT = true;
#endif
    m_name = name;
    if (!isConstructor)
        m_flags.m_nonConstructor = true;
    m_flags.m_isBoundFunction = false;
}

#ifdef ENABLE_ESJIT

ESValue executeJIT(ESFunctionObject* fn, ESVMInstance* instance, ExecutionContext& ec)
{
    ESValue result(ESValue::ESForceUninitialized);
#ifndef NDEBUG
    const char* functionName = fn->codeBlock()->m_nonAtomicId ? (fn->codeBlock()->m_nonAtomicId->utf8Data()):"(anonymous)";
#endif
    JITFunction jitFunction = fn->codeBlock()->m_cachedJITFunction;

    if (!jitFunction && !fn->codeBlock()->m_dontJIT) {

        // update profile data
        char* code = fn->codeBlock()->m_code.data();
        size_t siz = fn->codeBlock()->m_byteCodeIndexesHaveToProfile.size();
        for (unsigned i = 0; i < siz; i ++) {
            size_t pos = fn->codeBlock()->m_byteCodePositionsHaveToProfile[i];
            ByteCode* currentCode = (ByteCode *)&code[pos];
            Opcode opcode = fn->codeBlock()->m_extraData[fn->codeBlock()->m_byteCodeIndexesHaveToProfile[i]].m_opcode;
            switch (opcode) {
            case GetByIdOpcode: {
                reinterpret_cast<GetById*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetByIndexOpcode: {
                reinterpret_cast<GetByIndex*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetByIndexWithActivationOpcode: {
                reinterpret_cast<GetByIndexWithActivation*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetByGlobalIndexOpcode: {
                reinterpret_cast<GetByGlobalIndex*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectOpcode: {
                reinterpret_cast<GetObject*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectAndPushObjectOpcode: {
                reinterpret_cast<GetObjectAndPushObject*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectWithPeekingOpcode: {
                reinterpret_cast<GetObjectWithPeeking*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectPreComputedCaseOpcode: {
                reinterpret_cast<GetObjectPreComputedCase*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectWithPeekingPreComputedCaseOpcode: {
                reinterpret_cast<GetObjectWithPeekingPreComputedCase*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case GetObjectPreComputedCaseAndPushObjectOpcode: {
                reinterpret_cast<GetObjectPreComputedCaseAndPushObject*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case ThisOpcode: {
                reinterpret_cast<This*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case CallFunctionOpcode: {
                reinterpret_cast<CallFunction*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            case CallFunctionWithReceiverOpcode: {
                reinterpret_cast<CallFunctionWithReceiver*>(currentCode)->m_profile.updateProfiledType();
                break;
            }
            default:
                break;
            }
        }

        if (fn->codeBlock()->m_executeCount >= fn->codeBlock()->m_jitThreshold) {
            LOG_VJ("==========Trying JIT Compile for function %s... (codeBlock %p)==========\n", functionName, fn->codeBlock());
            size_t idx = 0;
            size_t bytecodeCounter = 0;
            bool compileNextTime = false;
            // check jit support for debug
#ifndef NDEBUG
            {
                char* code = fn->codeBlock()->m_code.data();
                char* end = &fn->codeBlock()->m_code.data()[fn->codeBlock()->m_code.size()];
                while (&code[idx] < end) {
                    Opcode opcode = fn->codeBlock()->m_extraData[bytecodeCounter].m_opcode;
                    switch (opcode) {
#define DECLARE_EXECUTE_NEXTCODE(opcode, pushCount, popCount, peekCount, JITSupported, hasProfileData) \
                    case opcode##Opcode: \
                        if (!JITSupported) { \
                            fn->codeBlock()->m_dontJIT = true; \
                            compileNextTime = true; \
                            LOG_VJ("> Unsupported ByteCode %s (idx %u). Stop trying JIT.\n", #opcode, (unsigned)idx); \
                        } \
                        idx += sizeof(opcode); \
                        bytecodeCounter++; \
                        break;
                        FOR_EACH_BYTECODE_OP(DECLARE_EXECUTE_NEXTCODE);
#undef DECLARE_EXECUTE_NEXTCODE
                    case OpcodeKindEnd:
                        break;
                    }
                }
            }
#endif
            // check profile data
            char* code = fn->codeBlock()->m_code.data();
            for (unsigned i = 0; i < fn->codeBlock()->m_byteCodeIndexesHaveToProfile.size(); i ++) {
                size_t pos = fn->codeBlock()->m_byteCodePositionsHaveToProfile[i];
                Opcode opcode = fn->codeBlock()->m_extraData[fn->codeBlock()->m_byteCodeIndexesHaveToProfile[i]].m_opcode;
                ByteCode* currentCode = (ByteCode *)&code[pos];
                switch (opcode) {
                case GetByIdOpcode: {
                    if (reinterpret_cast<GetById*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetByIndexOpcode: {
                    if (reinterpret_cast<GetByIndex*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetByIndexWithActivationOpcode: {
                    if (reinterpret_cast<GetByIndexWithActivation*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetByGlobalIndexOpcode: {
                    if (reinterpret_cast<GetByGlobalIndex*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectOpcode: {
                    if (reinterpret_cast<GetObject*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectAndPushObjectOpcode: {
                    if (reinterpret_cast<GetObjectAndPushObject*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectWithPeekingOpcode: {
                    if (reinterpret_cast<GetObjectWithPeeking*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectPreComputedCaseOpcode: {
                    if (reinterpret_cast<GetObjectPreComputedCase*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectWithPeekingPreComputedCaseOpcode: {
                    if (reinterpret_cast<GetObjectWithPeekingPreComputedCase*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case GetObjectPreComputedCaseAndPushObjectOpcode: {
                    if (reinterpret_cast<GetObjectPreComputedCaseAndPushObject*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case ThisOpcode: {
                    if (reinterpret_cast<This*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case CallFunctionOpcode: {
                    if (reinterpret_cast<CallFunction*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                case CallFunctionWithReceiverOpcode: {
                    if (reinterpret_cast<CallFunctionWithReceiver*>(currentCode)->m_profile.getType().isBottomType()) {
                        LOG_VJ("> Cannot Compile JIT Function due to idx %u is not profiled yet\n", (unsigned)pos);
                        compileNextTime = true;
                        break;
                    }
                    break;
                }
                default:
                    break;
                }
            }

            if (!compileNextTime) {
                jitFunction = reinterpret_cast<JITFunction>(ESJIT::JITCompile(fn->codeBlock(), instance));
                if (jitFunction) {
                    LOG_VJ("> Compilation successful for function %s (codeBlock %p)! Cache jit function %p\n", functionName, fn->codeBlock(), jitFunction);
#ifndef NDEBUG
                    if (ESVMInstance::currentInstance()->m_reportCompiledFunction) {
                        printf("%s ", fn->codeBlock()->m_nonAtomicId ? (fn->codeBlock()->m_nonAtomicId->utf8Data()):"(anonymous)");
                        ESVMInstance::currentInstance()->m_compiledFunctions++;
                    }
#endif
                    fn->codeBlock()->m_cachedJITFunction = jitFunction;
                } else {
                    LOG_VJ("> Compilation failed! disable jit compilation for function %s (codeBlock %p) from now on\n", functionName, fn->codeBlock());
                    fn->codeBlock()->m_dontJIT = true;
                    fn->codeBlock()->removeJITInfo();
                }
            } else {
                size_t threshold = fn->codeBlock()->m_jitThreshold;
                if (threshold > 1024) {
                    LOG_VJ("> No profile infos. Stop trying JIT for function %s.\n", functionName);
                    fn->codeBlock()->m_dontJIT = true;
                    fn->codeBlock()->removeJITInfo();
                } else {
                    LOG_VJ("> Doubling JIT compilation threshold from %d to %d for function %s\n", threshold, threshold*2, functionName);
                    fn->codeBlock()->m_jitThreshold *= 2;
                }
            }
        }
    }


    if (jitFunction) {
        unsigned stackSiz = fn->codeBlock()->m_requiredStackSizeInESValueSize * sizeof(ESValue);
#ifndef NDEBUG
        stackSiz *= 2;
#endif
        char* stackBuf = (char *)alloca(stackSiz);
        ec.setBp(stackBuf);

        fn->codeBlock()->m_recursionDepth++;
        result = ESValue::fromRawDouble(jitFunction(instance));
        fn->codeBlock()->m_recursionDepth--;
        // printf("JIT Result %s (%jx)\n", result.toString()->utf8Data(), result.asRawData());
        if (ec.inOSRExit()) {
            fn->codeBlock()->m_osrExitCount++;
            LOG_VJ("> OSR Exit from function %s (codeBlock %p), exit count %zu\n", functionName, fn->codeBlock(), fn->codeBlock()->m_osrExitCount);
            int32_t tmpIndex = result.asInt32();
            char* code = fn->codeBlock()->m_code.data();
            size_t idx = 0;
            size_t bytecodeCounter = 0;
            unsigned maxStackPos = 0;
            bool found = false;
            char* end = &fn->codeBlock()->m_code.data()[fn->codeBlock()->m_code.size()];
            while (&code[idx] < end) {
                if (found) {
                    break;
                }

                Opcode opcode = fn->codeBlock()->m_extraData[bytecodeCounter].m_opcode;
                ByteCodeExtraData* extraData = &fn->codeBlock()->m_extraData[bytecodeCounter];
                if (extraData->m_decoupledData->m_targetIndex0 == tmpIndex) {
                    maxStackPos = ec.getStackPos();
                    if (ec.executeNextByteCode()) {
                        found = true;
                    } else {
                        break;
                    }
                }

                switch (opcode) {
#define DECLARE_EXECUTE_NEXTCODE(code, pushCount, popCount, peekCount, JITSupported, hasProfileData) \
                case code##Opcode: \
                    idx += sizeof(code); \
                    bytecodeCounter++; \
                    continue;
                    FOR_EACH_BYTECODE_OP(DECLARE_EXECUTE_NEXTCODE);
#undef DECLARE_EXECUTE_NEXTCODE
                default:
                    RELEASE_ASSERT_NOT_REACHED();
                    break;
                }
            }
            if (fn->codeBlock()->m_osrExitCount >= ESVMInstance::currentInstance()->m_osrExitThreshold) {
                LOG_VJ("Too many exits; Disable JIT for function %s (codeBlock %p) from now on\n", functionName, fn->codeBlock());
                fn->codeBlock()->m_cachedJITFunction = nullptr;
                // Fixme(JMP): We have to compile to JIT code again when gathering enough type data
                fn->codeBlock()->m_dontJIT = true;
                if (!fn->codeBlock()->m_recursionDepth) {
                    fn->codeBlock()->removeJITInfo();
                    fn->codeBlock()->removeJITCode();
                }
#ifndef NDEBUG
                if (ESVMInstance::currentInstance()->m_reportOSRExitedFunction) {
                    printf("%s ", fn->codeBlock()->m_nonAtomicId ? (fn->codeBlock()->m_nonAtomicId->utf8Data()):"(anonymous)");
                    ESVMInstance::currentInstance()->m_osrExitedFunctions++;
                }
#endif
            }
            if (idx == fn->codeBlock()->m_code.size()) {
                result = ESValue();
            } else {
                result = interpret(instance, fn->codeBlock(), idx, maxStackPos);
            }
            fn->codeBlock()->m_executeCount++;
        }
        if (UNLIKELY(fn->codeBlock()->m_dontJIT)) {
            if (!fn->codeBlock()->m_recursionDepth) {
                fn->codeBlock()->removeJITInfo();
                fn->codeBlock()->removeJITCode();
            }
        }
    } else {
        result = interpret(instance, fn->codeBlock());
        fn->codeBlock()->m_executeCount++;
    }

    return result;
}

#endif

ALWAYS_INLINE void functionCallerInnerProcess(ExecutionContext* newEC, ESFunctionObject* fn, CodeBlock* cb, DeclarativeEnvironmentRecord* functionRecord, ESValue* stackStorage, const ESValue& receiver, ESValue arguments[], const size_t& argumentCount, ESVMInstance* ESVMInstance)
{
    // http://www.ecma-international.org/ecma-262/6.0/#sec-ordinarycallbindthis
    if (newEC->isStrictMode() || cb->m_isBuiltInFunction) {
        newEC->setThisBinding(receiver);
    } else {
        if (receiver.isUndefinedOrNull()) {
            newEC->setThisBinding(ESVMInstance->globalObject());
        } else {
            newEC->setThisBinding(receiver.toObject());
        }
    }

    // if FunctionExpressionNode has own name, should bind own function object
    if (cb->m_isFunctionExpression && cb->m_functionExpressionNameIndex != SIZE_MAX) {
        if (cb->m_isFunctionExpressionNameHeapAllocated) {
            *functionRecord->bindingValueForHeapAllocatedData(cb->m_functionExpressionNameIndex) = ESValue(fn);
        } else {
            stackStorage[cb->m_functionExpressionNameIndex] = ESValue(fn);
        }
    }

    const FunctionParametersInfoVector& info = cb->m_paramsInformation;
    size_t siz = std::min(argumentCount, info.size());
    if (UNLIKELY(cb->m_needsComplexParameterCopy)) {
        for (size_t i = 0; i < siz; i ++) {
            if (info[i].m_isHeapAllocated) {
                *functionRecord->bindingValueForHeapAllocatedData(info[i].m_index) = arguments[i];
            } else {
                stackStorage[info[i].m_index] = arguments[i];
            }
        }
    } else {
        for (size_t i = 0; i < siz; i ++) {
            stackStorage[i] = arguments[i];
        }
    }

}


ESValue ESFunctionObject::call(ESVMInstance* instance, const ESValue& callee, const ESValue& receiver, ESValue arguments[], const size_t& argumentCount, bool isNewExpression)
{
    char dummy;
    if (UNLIKELY(instance->m_stackStart - &dummy) > 4 * 1024 * 1024) // maximum call stack size : 4MB
        instance->throwError(RangeError::create(ESString::create("Maximum call stack size exceeded.")));
    ESValue result(ESValue::ESForceUninitialized);
    if (LIKELY(callee.isESPointer() && callee.asESPointer()->isESFunctionObject())) {
        ExecutionContext* currentContext = instance->currentExecutionContext();
        ESFunctionObject* fn = callee.asESPointer()->asESFunctionObject();
        CodeBlock* const cb = fn->codeBlock();

        if (UNLIKELY(!cb->m_hasCode)) {
            FunctionNode* node = (FunctionNode *)cb->m_ast;
            cb->m_stackAllocatedIdentifiersCount = node->m_stackAllocatedIdentifiersCount;
            cb->m_heapAllocatedIdentifiers = std::move(node->m_heapAllocatedIdentifiers);
            cb->m_paramsInformation = std::move(node->m_paramsInformation);
            cb->m_needsHeapAllocatedExecutionContext = node->m_needsHeapAllocatedExecutionContext;
            cb->m_needsToPrepareGenerateArgumentsObject = node->m_needsToPrepareGenerateArgumentsObject;
            cb->m_needsComplexParameterCopy = node->m_needsComplexParameterCopy;
            // cb->m_params = std::move(m_params);
            // FIXME copy params if needs future
            cb->m_isStrict = node->m_isStrict;
            cb->m_isFunctionExpression = node->isExpression();
            cb->m_argumentCount = node->m_params.size();
            cb->m_hasCode = true;
            cb->m_functionExpressionNameIndex = node->m_functionIdIndex;
            cb->m_isFunctionExpressionNameHeapAllocated = node->m_functionIdIndexNeedsHeapAllocation;
            cb->m_needsActivation = node->m_needsActivation;

            ByteCodeGenerateContext newContext(cb, false);
            node->body()->generateStatementByteCode(cb, newContext);

            cb->pushCode(ReturnFunction(), newContext, node);
            cb->m_ast = NULL;

#ifndef NDEBUG
            cb->m_id = node->m_id;
            cb->m_nonAtomicId = node->m_nonAtomicId;
            if (ESVMInstance::currentInstance()->m_reportUnsupportedOpcode) {
                char* code = cb->m_code.data();
                ByteCode* currentCode = (ByteCode *)(&code[0]);
                if (currentCode->m_orgOpcode != ExecuteNativeFunctionOpcode) {
                    dumpUnsupported(cb);
                }
            }
#endif
        }

        ESValue* stackStorage = (::escargot::ESValue *)alloca(sizeof(::escargot::ESValue) * cb->m_stackAllocatedIdentifiersCount);
        if (cb->m_needsHeapAllocatedExecutionContext) {
            auto FE = LexicalEnvironment::newFunctionEnvironment(cb->m_needsToPrepareGenerateArgumentsObject,
                stackStorage, cb->m_stackAllocatedIdentifiersCount, cb->m_heapAllocatedIdentifiers, arguments, argumentCount, fn, cb->m_needsActivation, cb->m_functionExpressionNameIndex);
            instance->m_currentExecutionContext = new ExecutionContext(FE, isNewExpression, cb->shouldUseStrictMode(), arguments, argumentCount);
            FunctionEnvironmentRecord* record = (FunctionEnvironmentRecord *)FE->record();
            functionCallerInnerProcess(instance->m_currentExecutionContext, fn, cb, record, stackStorage, receiver, arguments, argumentCount, instance);

#ifdef ENABLE_ESJIT
            result = executeJIT(fn, instance, *instance->m_currentExecutionContext);
#else
            result = interpret(instance, cb, 0, stackStorage, &record->heapAllocatedData());
#endif
            instance->m_currentExecutionContext = currentContext;
        } else {
            if (UNLIKELY(cb->m_needsToPrepareGenerateArgumentsObject)) {
                FunctionEnvironmentRecordWithArgumentsObject envRec(
                    arguments, argumentCount, fn,
                    stackStorage, cb->m_stackAllocatedIdentifiersCount, cb->m_heapAllocatedIdentifiers, cb->m_needsActivation, cb->m_functionExpressionNameIndex);
                LexicalEnvironment env(&envRec, fn->outerEnvironment());
                ExecutionContext ec(&env, isNewExpression, cb->shouldUseStrictMode(), arguments, argumentCount);
                instance->m_currentExecutionContext = &ec;
                functionCallerInnerProcess(&ec, fn, cb, &envRec, stackStorage, receiver, arguments, argumentCount, instance);
#ifdef ENABLE_ESJIT
                result = executeJIT(fn, instance, ec);
#else
                result = interpret(instance, cb, 0, stackStorage, &envRec.heapAllocatedData());
#endif
                instance->m_currentExecutionContext = currentContext;
            } else {
                FunctionEnvironmentRecord envRec(
                    stackStorage, cb->m_stackAllocatedIdentifiersCount, cb->m_heapAllocatedIdentifiers, cb->m_needsActivation, cb->m_functionExpressionNameIndex);
                LexicalEnvironment env(&envRec, fn->outerEnvironment());
                ExecutionContext ec(&env, isNewExpression, cb->shouldUseStrictMode(), arguments, argumentCount);
                instance->m_currentExecutionContext = &ec;
                functionCallerInnerProcess(&ec, fn, cb, &envRec, stackStorage, receiver, arguments, argumentCount, instance);
#ifdef ENABLE_ESJIT
                result = executeJIT(fn, instance, ec);
#else
                result = interpret(instance, cb, 0, stackStorage, &envRec.heapAllocatedData());
#endif
                instance->m_currentExecutionContext = currentContext;
            }
        }
    } else {
        instance->throwError(ESValue(TypeError::create(ESString::create("Callee is not a function object"))));
    }

    return result;
}

ESDateObject::ESDateObject(ESPointer::Type type)
    : ESObject((Type)(Type::ESObject | Type::ESDateObject), ESVMInstance::currentInstance()->globalObject()->datePrototype())
{
    m_isCacheDirty = true;
    m_hasValidDate = false;
}

void ESDateObject::parseYmdhmsToDate(struct tm* timeinfo, int year, int month, int date, int hour, int minute, int second)
{
    char buffer[255];
    snprintf(buffer, 255, "%d-%d-%d-%d-%d-%d", year, month + 1, date, hour, minute, second);
    strptime(buffer, "%Y-%m-%d-%H-%M-%S", timeinfo);
}

double ESDateObject::parseStringToDate(escargot::ESString* istr)
{
    struct tm timeinfo;
    double primitiveValue = 0.0;
    timeinfo.tm_mday = 1; // set special initial case for mday. if we don't initialize it, it would be set to 0

    char* buffer = (char*)istr->toNullableUTF8String().m_buffer;
    char* parse_returned;
    size_t fmt_length = istr->length();

    parse_returned = strptime(buffer, "%Y", &timeinfo); // Date format with UTC timezone
    if (buffer + fmt_length == parse_returned) {
        primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, 0, 1, 0, 0, 0) * 1000.;
        return primitiveValue;
    }

    parse_returned = strptime(buffer, "%B %d %Y %H:%M:%S %z", &timeinfo); // Date format with specific timezone
    if (buffer + fmt_length == parse_returned) { 
        primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec) * 1000.;
#if defined(__USE_BSD) || defined(__USE_MISC)
        primitiveValue = primitiveValue - timeinfo.tm_gmtoff * 1000;
#else
        primitiveValue = primitiveValue - timeinfo.__tm_gmtoff * 1000;
#endif
        return primitiveValue;
    }

    parse_returned = strptime(buffer, "%A %B %d %Y %H:%M:%S GMT", &timeinfo); // Date format with specific timezone
    if (parse_returned) { // consider as "%A %B %d %Y %H:%M:%S GMT+9 (KST)" like format
        // TODO : consider of setting daylightsaving flag (timeinfo->tm_isdst)
        timeinfo.tm_gmtoff = 1 * 60 * 60; // 1 hour
        if (*parse_returned == '-') {
            timeinfo.tm_gmtoff *= -1;
        }
        parse_returned++;
        int tz = *parse_returned - '0';
        if (*(parse_returned+1) != ' ') {
            tz *= 10;
            tz += *(parse_returned+1) - '0';
            timeinfo.tm_gmtoff *= tz;
        } else {
            timeinfo.tm_gmtoff *= tz;
        }
//        if (buffer + fmt_length == parse_returned) { 
            primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec) * 1000.;
#if defined(__USE_BSD) || defined(__USE_MISC)
            primitiveValue = primitiveValue - timeinfo.tm_gmtoff * 1000;
#else
            primitiveValue = primitiveValue - timeinfo.__tm_gmtoff * 1000;
#endif
            return primitiveValue;
//        }
    }

    parse_returned = strptime(buffer, "%Y-%m-%dT%H:%M:%S.", &timeinfo); // Date format with UTC timezone
    if (buffer + fmt_length == parse_returned + 3) { // for milliseconds part
        primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec) * 1000.;
        return primitiveValue;
    }

    parse_returned = strptime(buffer, "%Y-%m-%dT%H:%M:%S.", &timeinfo); // Date format with UTC timezone
    if (buffer + fmt_length == parse_returned + 4) { // for milliseconds part and 'Z' part
        primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec) * 1000.;
        return primitiveValue;
    }

    parse_returned = strptime(buffer, "%m/%d/%Y %H:%M:%S", &timeinfo); // Date format with local timezone
    if (buffer + fmt_length == parse_returned) {
        primitiveValue = ymdhmsToSeconds(timeinfo.tm_year+1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec) * 1000.;
        primitiveValue = toUTC(primitiveValue);
        return primitiveValue;
    }
    return std::numeric_limits<double>::quiet_NaN();    
}

int ESDateObject::daysInYear(long year)
{
    long y = year;
    if (y % 4 != 0) {
        return 365;
    } else if (y % 100 != 0) {
        return 366;
    } else if (y % 400 != 0) {
        return 365;
    } else { // y % 400 == 0
        return 366;
    }
}

int ESDateObject::dayFromYear(long year) // day number of the first day of year 'y'
{
    return 365 * (year - 1970) + floor((year - 1969) / 4) - floor((year - 1901) / 100) + floor((year - 1601) / 400);
}

long ESDateObject::yearFromTime(long long t)
{
    long estimate = ceil(t / msPerDay / 365.0) + 1970;
    while (makeDay(estimate, 0, 1) * msPerDay > t) {
        estimate--;
    }
    return estimate;
}

int ESDateObject::inLeapYear(long long t)
{
    int days = daysInYear(yearFromTime(t));
    if (days == 365) {
        return 0;
    } else if (days == 366) {
        return 1;
    }
    RELEASE_ASSERT_NOT_REACHED();
}

int ESDateObject::dayFromMonth(long year, int month)
{
    int ds[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (daysInYear(year) == 366) {
        ds[1] = 29;
    }
    int retval = 0;
    for (int i = 0;  i < month; i++) {
        retval += ds[i];
    }
    return retval;
}

int ESDateObject::monthFromTime(long long t)
{
    int dayWithinYear = day(t) - dayFromYear(yearFromTime(t));
    int leap = inLeapYear(t);
    if (dayWithinYear < 0) {
        RELEASE_ASSERT_NOT_REACHED();
    } else if (dayWithinYear < 31) {
        return 0;
    } else if (dayWithinYear < 59 + leap) {
        return 1;
    } else if (dayWithinYear < 90 + leap) {
        return 2;
    } else if (dayWithinYear < 120 + leap) {
        return 3;
    } else if (dayWithinYear < 151 + leap) {
        return 4;
    } else if (dayWithinYear < 181 + leap) {
        return 5;
    } else if (dayWithinYear < 212 + leap) {
        return 6;
    } else if (dayWithinYear < 243 + leap) {
        return 7;
    } else if (dayWithinYear < 273 + leap) {
        return 8;
    } else if (dayWithinYear < 304 + leap) {
        return 9;
    } else if (dayWithinYear < 334 + leap) {
        return 10;
    } else if (dayWithinYear < 365 + leap) {
        return 11;
    } else {
        RELEASE_ASSERT_NOT_REACHED();
    }
}

int ESDateObject::dateFromTime(long long t)
{
    int dayWithinYear = day(t) - dayFromYear(yearFromTime(t));
    int leap = inLeapYear(t);
    int retval = dayWithinYear - leap;
    switch (monthFromTime(t)) {
    case 0:
        return retval + 1 + leap;
    case 1:
        return retval - 30 + leap;
    case 2:
        return retval - 58;
    case 3:
        return retval - 89;
    case 4:
        return retval - 119;
    case 5:
        return retval - 150;
    case 6:
        return retval - 180;
    case 7:
        return retval - 211;
    case 8:
        return retval - 242;
    case 9:
        return retval - 272;
    case 10:
        return retval - 303;
    case 11:
        return retval - 333;
    default:
        RELEASE_ASSERT_NOT_REACHED();
    }
}

double ESDateObject::makeDay(long year, int month, int date)
{
    // TODO: have to check whether year or month is infinity
//    if(year == infinity || month == infinity){
//        return nan;
//    }
    // adjustment on argument[0],[1] is performed at setTimeValue(with 7 arguments) function
    ASSERT(month < 12);
    long ym = year;
    int mn = month;
    long long t = timeFromYear(ym) + dayFromMonth(ym, mn) * msPerDay;
    return day(t) + date - 1;
}

double ESDateObject::ymdhmsToSeconds(long year, int mon, int day, int hour, int minute, double second)
{
    return (makeDay(year, mon, day) * msPerDay + (hour * msPerHour + minute * msPerMinute + second * msPerSecond /* + millisecond */)) / 1000.0;
}

void ESDateObject::setTimeValue()
{
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    m_primitiveValue = time.tv_sec * (long long) 1000 + floor(time.tv_nsec / 1000000);
    m_isCacheDirty = true;
    m_hasValidDate = true;
}

void ESDateObject::setTimeValue(double t)
{
    setTime(t);
}
void ESDateObject::setTimeValue(const ESValue str)
{
    escargot::ESString* istr = str.toString();

    double primitiveValue = parseStringToDate(istr);
    if (isnan(primitiveValue)) {
        setTimeValueAsNaN();
    } else {
        m_primitiveValue = primitiveValue;
        if (m_primitiveValue <= 8640000000000000 && m_primitiveValue >= -8640000000000000) {
            m_isCacheDirty = true;
            m_hasValidDate = true;
        } else {
            setTimeValueAsNaN();
        }       
    }
/*    bool timezoneSet = false;
    if (!parseStringToDate(&m_cachedTM, &timezoneSet, istr)) {
        m_hasValidDate = false;
        return;
    }

    double primitiveValue = ymdhmsToSeconds(m_cachedTM.tm_year+1900, m_cachedTM.tm_mon, m_cachedTM.tm_mday, m_cachedTM.tm_hour, m_cachedTM.tm_min, m_cachedTM.tm_sec);
    double primitiveValueAsUTC;
    if (!timezoneSet) {
        primitiveValueAsUTC = toUTC(primitiveValue);
    } else {
#ifdef __USE_BSD         
        primitiveValueAsUTC = primitiveValue - m_cachedTM.tm_gmtoff * 1000;
#else
        primitiveValueAsUTC = primitiveValue - m_cachedTM.__tm_gmtoff * 1000;
#endif        
    }
    m_primitiveValue = primitiveValueAsUTC;
    if (m_primitiveValue <= 8640000000000000 && m_primitiveValue >= -8640000000000000) {
        m_isCacheDirty = false;
        m_hasValidDate = true;
    } else {
        setTimeValueAsNaN();
    }
    */
}

void ESDateObject::setTimeValue(int year, int month, int date, int hour, int minute, int second, int millisecond, bool convertToUTC)
{
    long ym = year + floor(month / 12);
    int mn = month % 12;
//    parseYmdhmsToDate(&m_cachedTM, ym, mn, date, hour, minute, second);
//    m_cachedTM.tm_isdst = true;
//    double primitiveValue = ymdhmsToSeconds(m_cachedTM.tm_year+1900, m_cachedTM.tm_mon, m_cachedTM.tm_mday, m_cachedTM.tm_hour, m_cachedTM.tm_min, m_cachedTM.tm_sec) * 1000. + (double) millisecond;
    double primitiveValue = ymdhmsToSeconds(ym, mn, date, hour, minute, second) * 1000. + (double) millisecond;
    if (convertToUTC) {
        primitiveValue = toUTC(primitiveValue);
    }

    m_primitiveValue = primitiveValue;
    if (m_primitiveValue <= 8640000000000000 && m_primitiveValue >= -8640000000000000) {
        m_isCacheDirty = false;
        m_hasValidDate = true;
    } else {
        setTimeValueAsNaN();
    }
    
    m_isCacheDirty = true;
    resolveCache();
}

void ESDateObject::resolveCache()
{
    if (m_isCacheDirty) {
        struct timespec time;
        time.tv_sec = m_primitiveValue / 1000;
        time.tv_nsec = (m_primitiveValue % 1000) * 1000000;
        memcpy(&m_cachedTM, ESVMInstance::currentInstance()->computeLocalTime(time), sizeof(tm));
        m_isCacheDirty = false;
    }
}

ESString* ESDateObject::toDateString()
{
    static char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    static char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    resolveCache();
    char buffer[512];
    if (!isnan(timeValueAsDouble())) {
        snprintf(buffer, 512, "%s %s %02d %d"
            , days[getDay()], months[getMonth()], getDate(), getFullYear());
        return ESString::create(buffer);
    } else {
        return ESString::create(u"Invalid Date");
    } 
}

ESString* ESDateObject::toTimeString()
{
    resolveCache();
    char buffer[512];
    if (!isnan(timeValueAsDouble())) {
        snprintf(buffer, 512, "%02d:%02d:%02d GMT%+.1g (%s)"
            , getHours(), getMinutes(), getSeconds()
            , getTimezoneOffset() / -3600.0, tzname[0]);
        return ESString::create(buffer);
    } else {
        return ESString::create(u"Invalid Date");
    }
}

ESString* ESDateObject::toFullString()
{
    resolveCache();
    if (!isnan(timeValueAsDouble())) {
        ::escargot::ESString* tmp = ESString::concatTwoStrings(toDateString(), ESString::create(u" "));
        return ESString::concatTwoStrings(tmp, toTimeString());
    } else {
        return ESString::create(u"Invalid Date");
    }
    
}
int ESDateObject::getDate()
{
    resolveCache();
    return m_cachedTM.tm_mday;
}

int ESDateObject::getDay()
{
    resolveCache();
    return m_cachedTM.tm_wday;
}

int ESDateObject::getFullYear()
{
    resolveCache();
    return m_cachedTM.tm_year + 1900;
}

int ESDateObject::getHours()
{
    resolveCache();
    return m_cachedTM.tm_hour;
}
int ESDateObject::getMilliseconds()
{
    return (int) (m_primitiveValue % 1000);
}
int ESDateObject::getMinutes()
{
    resolveCache();
    return m_cachedTM.tm_min;
}

int ESDateObject::getMonth()
{
    resolveCache();
    return m_cachedTM.tm_mon;
}

int ESDateObject::getSeconds()
{
    resolveCache();
    return m_cachedTM.tm_sec;
}

long ESDateObject::getTimezoneOffset()
{
    return ESVMInstance::currentInstance()->timezoneOffset();
}

void ESDateObject::setTime(double t)
{
    if (isnan(t)) {
        setTimeValueAsNaN();
        return;
    }

    m_primitiveValue = floor(t);
    
    if (m_primitiveValue <= 8640000000000000 && m_primitiveValue >= -8640000000000000) {
        m_isCacheDirty = true;
        m_hasValidDate = true;
    } else {
        setTimeValueAsNaN();
    }
}

int ESDateObject::getUTCDate()
{
    return dateFromTime(m_primitiveValue);
}

int ESDateObject::getUTCDay()
{
    RELEASE_ASSERT_NOT_REACHED();
}

int ESDateObject::getUTCFullYear()
{
    return yearFromTime(m_primitiveValue);
}

int ESDateObject::getUTCHours()
{
    return (long long) floor(m_primitiveValue / msPerHour) % (int) hoursPerDay;
}
int ESDateObject::getUTCMilliseconds()
{
    return m_primitiveValue % (int) msPerSecond;
}
int ESDateObject::getUTCMinutes()
{
    return (long long) floor(m_primitiveValue / msPerMinute) % (int) minutesPerHour;
}

int ESDateObject::getUTCMonth()
{
    return monthFromTime(m_primitiveValue);
}

int ESDateObject::getUTCSeconds()
{
    return (long long) floor(m_primitiveValue / msPerSecond) % (int) secondsPerMinute;
}

double ESDateObject::toUTC(double t)
{
    long tzOffsetAsSec = getTimezoneOffset(); // For example, it returns 28800 in GMT-8 zone
    return t + (double) tzOffsetAsSec * 1000.;
}

ESMathObject::ESMathObject(ESPointer::Type type)
    : ESObject((Type)(Type::ESObject | Type::ESMathObject), ESVMInstance::currentInstance()->globalObject()->objectPrototype())
{
}

ESStringObject::ESStringObject(escargot::ESString* str)
    : ESObject((Type)(Type::ESObject | Type::ESStringObject), ESVMInstance::currentInstance()->globalObject()->stringPrototype())
{
    setStringData(str);

    // $21.1.4.1 String.length
    defineAccessorProperty(strings->length.string(), ESVMInstance::currentInstance()->stringObjectLengthAccessorData(), false, false, false);
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
    if (message != strings->emptyString.string())
        set(strings->message, message);
    set(strings->name, strings->Error.string());
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

URIError::URIError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->URIError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->uriErrorPrototype());
}

EvalError::EvalError(escargot::ESString* message)
    : ESErrorObject(message)
{
    set(strings->name, strings->EvalError.string());
    set__proto__(ESVMInstance::currentInstance()->globalObject()->evalErrorPrototype());
}

ESArrayBufferObject::ESArrayBufferObject(ESPointer::Type type)
    : ESObject((Type)(Type::ESObject | Type::ESArrayBufferObject), ESVMInstance::currentInstance()->globalObject()->arrayBufferPrototype())
    , m_data(NULL)
    , m_bytelength(0)
{
    set__proto__(ESVMInstance::currentInstance()->globalObject()->arrayBufferPrototype());
}

ESArrayBufferView::ESArrayBufferView(ESPointer::Type type, ESValue __proto__)
    : ESObject((Type)(Type::ESObject | Type::ESArrayBufferView | type), __proto__)
{
}

ESValue ESTypedArrayObjectWrapper::get(uint32_t key)
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
bool ESTypedArrayObjectWrapper::set(uint32_t key, ESValue val)
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

ESArgumentsObject::ESArgumentsObject(FunctionEnvironmentRecordWithArgumentsObject* environment)
    : ESObject((Type)(Type::ESObject | Type::ESArgumentsObject), ESVMInstance::currentInstance()->globalObject()->objectPrototype(), 6)
    , m_environment(environment)
{
}

ESJSONObject::ESJSONObject(ESPointer::Type type)
    : ESObject((Type)(Type::ESObject | Type::ESJSONObject), ESVMInstance::currentInstance()->globalObject()->objectPrototype(), 6)
{
}

void ESPropertyAccessorData::setGetterAndSetterTo(ESObject* obj, const ESHiddenClassPropertyInfo* propertyInfo)
{
    if (m_jsGetter || m_jsSetter) {
        ASSERT(!m_nativeGetter && !m_nativeSetter);
        if (m_jsGetter)
            obj->set(strings->get.string(), m_jsGetter);
        else
            obj->set(strings->get.string(), ESValue());
        if (m_jsSetter)
            obj->set(strings->set.string(), m_jsSetter);
        else
            obj->set(strings->set.string(), ESValue());
        return;
    }

    if (m_nativeGetter || m_nativeSetter) {
        ASSERT(!m_jsGetter && !m_jsSetter);
        obj->set(strings->writable.string(), ESValue(propertyInfo->m_flags.m_isWritable));
        return;
    }
}


ESValue ESBindingSlot::getValueWithGetter(escargot::ESObject* obj, escargot::ESString* propertyName)
{
    ASSERT(m_slot);
    ASSERT(!m_isDataBinding);
    ESPropertyAccessorData* accessor = (ESPropertyAccessorData*)m_slot->asESPointer();
    return accessor->value(obj, ESValue(obj), propertyName);
}

void ESBindingSlot::setValueWithSetter(escargot::ESObject* obj, escargot::ESString* propertyName, const ESValue& value)
{
    ASSERT(m_slot);
    ASSERT(!m_isDataBinding);
    ESPropertyAccessorData* accessor = (ESPropertyAccessorData*)m_slot->asESPointer();
    accessor->setValue(obj, ESValue(obj), propertyName, value);
}


}
