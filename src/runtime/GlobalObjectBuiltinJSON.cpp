/*
 * Copyright (c) 2016-present Samsung Electronics Co., Ltd
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 */

#include "Escargot.h"
#include "GlobalObject.h"
#include "Context.h"
#include "VMInstance.h"
#include "StringObject.h"
#include "ArrayObject.h"
#include "TypedArrayObject.h"
#include "BooleanObject.h"
#include "BigIntObject.h"
#include "NativeFunctionObject.h"

#define RAPIDJSON_PARSE_DEFAULT_FLAGS kParseFullPrecisionFlag
#define RAPIDJSON_ERROR_CHARTYPE char
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/memorystream.h>
#include <rapidjson/internal/dtoa.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <rapidjson/error/en.h>

namespace Escargot {

template <typename Encoding>
struct JSONStringStream {
    typedef typename Encoding::Ch Ch;

    JSONStringStream(const Ch* src, size_t length)
        : src_(src)
        , head_(src)
        , tail_(src + length)
    {
    }

    Ch Peek() const
    {
        if (UNLIKELY(tail_ <= src_)) {
            return 0;
        }
        return *src_;
    }
    Ch Take()
    {
        if (UNLIKELY(tail_ <= src_)) {
            return 0;
        }
        return *src_++;
    }
    size_t Tell() const { return static_cast<size_t>(src_ - head_); }
    Ch* PutBegin()
    {
        RAPIDJSON_ASSERT(false);
        return 0;
    }
    void Put(Ch) { RAPIDJSON_ASSERT(false); }
    void Flush() { RAPIDJSON_ASSERT(false); }
    size_t PutEnd(Ch*)
    {
        RAPIDJSON_ASSERT(false);
        return 0;
    }

    const Ch* src_; //!< Current read position.
    const Ch* head_; //!< Original head of the string.
    const Ch* tail_;
};

template <typename CharType, typename JSONCharType>
static Value parseJSONWorker(ExecutionState& state, rapidjson::GenericValue<JSONCharType>& value)
{
    volatile int sp;
    size_t currentStackBase = (size_t)&sp;
#ifdef STACK_GROWS_DOWN
    if (UNLIKELY(state.stackLimit() > currentStackBase)) {
#else
    if (UNLIKELY(state.stackLimit() < currentStackBase)) {
#endif
        ErrorObject::throwBuiltinError(state, ErrorObject::RangeError, "Maximum call stack size exceeded");
    }

    if (value.IsBool()) {
        return Value(value.GetBool());
    } else if (value.IsInt()) {
        return Value(value.GetInt());
    } else if (value.IsUint()) {
        return Value(value.GetUint());
    } else if (value.IsInt64()) {
        return Value(value.GetInt64());
    } else if (value.IsUint64()) {
        return Value(value.GetUint64());
    } else if (value.IsDouble()) {
        return Value(value.GetDouble());
    } else if (value.IsNull()) {
        return Value(Value::Null);
    } else if (value.IsString()) {
        if (std::is_same<CharType, char16_t>::value) {
            const char16_t* chars = (const char16_t*)value.GetString();
            unsigned len = value.GetStringLength();
            if (isAllLatin1(chars, len)) {
                return new Latin1String(chars, len);
            } else {
                return new UTF16String(chars, len);
            }
        } else {
            const char* valueAsString = (const char*)value.GetString();
            if (isAllASCII(valueAsString, strlen(valueAsString))) {
                return new ASCIIString(valueAsString);
            } else {
                return new UTF16String(utf8StringToUTF16String(valueAsString, strlen(valueAsString)));
            }
        }
    } else if (value.IsArray()) {
        ArrayObject* arr = new ArrayObject(state);
        size_t i = 0;
        auto iter = value.Begin();
        while (iter != value.End()) {
            arr->defineOwnProperty(state, ObjectPropertyName(state, Value(i++)), ObjectPropertyDescriptor(parseJSONWorker<CharType, JSONCharType>(state, *iter), ObjectPropertyDescriptor::AllPresent));
            iter++;
        }
        return arr;
    } else if (value.IsObject()) {
        Object* obj = new Object(state);
        obj->markThisObjectDontNeedStructureTransitionTable();
        auto iter = value.MemberBegin();
        while (iter != value.MemberEnd()) {
            Value propertyName = parseJSONWorker<CharType, JSONCharType>(state, iter->name);
            ASSERT(propertyName.isString());
            obj->defineOwnProperty(state, ObjectPropertyName(state, propertyName), ObjectPropertyDescriptor(parseJSONWorker<CharType, JSONCharType>(state, iter->value), ObjectPropertyDescriptor::AllPresent));
            iter++;
        }
        return obj;
    } else {
        RELEASE_ASSERT_NOT_REACHED();
    }
}

template <typename CharType, typename JSONCharType>
Value parseJSON(ExecutionState& state, const CharType* data, size_t length)
{
    auto strings = &state.context()->staticStrings();
    rapidjson::GenericDocument<JSONCharType> jsonDocument;

    JSONStringStream<JSONCharType> stringStream(data, length);
    jsonDocument.ParseStream(stringStream);
    if (jsonDocument.HasParseError()) {
        ErrorObject::throwBuiltinError(state, ErrorObject::SyntaxError, strings->JSON.string(), true, strings->parse.string(), rapidjson::GetParseError_En(jsonDocument.GetParseError()));
    }

    return parseJSONWorker<CharType, JSONCharType>(state, jsonDocument);
}

String* codePointTo4digitString(int codepoint)
{
    StringBuilder ret;
    int d = 16 * 16 * 16;
    for (int i = 0; i < 4; ++i) {
        if (codepoint >= d) {
            char16_t c;
            if (codepoint / d < 10) {
                c = (codepoint / d) + '0';
            } else {
                c = (codepoint / d) - 10 + 'a';
            }
            codepoint %= d;
            ret.appendChar(c);
        } else {
            ret.appendChar('0');
        }
        d >>= 4;
    }

    return ret.finalize();
}

static Value builtinJSONParse(ExecutionState& state, Value thisValue, size_t argc, Value* argv, Optional<Object*> newTarget)
{
    auto strings = &state.context()->staticStrings();

    // 1, 2, 3
    String* JText = argv[0].toString(state);
    Value unfiltered;

    if (JText->has8BitContent()) {
        size_t len = JText->length();
        char16_t* char16Buf = new char16_t[len];
        std::unique_ptr<char16_t[]> buf(char16Buf);
        const LChar* srcBuf = JText->characters8();
        for (size_t i = 0; i < len; i++) {
            char16Buf[i] = srcBuf[i];
        }
        unfiltered = parseJSON<char16_t, rapidjson::UTF16<char16_t>>(state, buf.get(), JText->length());
    } else {
        unfiltered = parseJSON<char16_t, rapidjson::UTF16<char16_t>>(state, JText->characters16(), JText->length());
    }

    // 4
    Value reviver = argv[1];
    if (reviver.isCallable()) {
        Object* root = new Object(state);
        root->markThisObjectDontNeedStructureTransitionTable();
        root->defineOwnProperty(state, ObjectPropertyName(state, String::emptyString), ObjectPropertyDescriptor(unfiltered, ObjectPropertyDescriptor::AllPresent));
        std::function<Value(Value, const ObjectPropertyName&)> Walk;
        Walk = [&](Value holder, const ObjectPropertyName& name) -> Value {
            Value val = holder.asPointerValue()->asObject()->get(state, name).value(state, holder);
            if (val.isObject()) {
                if (val.asObject()->isArray(state)) {
                    Object* object = val.asObject();
                    uint32_t i = 0;
                    uint32_t len = object->length(state);
                    while (i < len) {
                        Value newElement = Walk(val, ObjectPropertyName(state, Value(i).toString(state)));
                        if (newElement.isUndefined()) {
                            object->deleteOwnProperty(state, ObjectPropertyName(state, Value(i).toString(state)));
                        } else {
                            object->defineOwnProperty(state, ObjectPropertyName(state, Value(i).toString(state)), ObjectPropertyDescriptor(newElement, ObjectPropertyDescriptor::AllPresent));
                        }
                        i++;
                    }
                } else if (val.asObject()->isTypedArrayObject()) {
                    ArrayBufferView* arrObject = val.asObject()->asArrayBufferView();
                    uint32_t i = 0;
                    uint32_t len = arrObject->arrayLength();
                    while (i < len) {
                        Value newElement = Walk(val, ObjectPropertyName(state, Value(i).toString(state)));
                        if (newElement.isUndefined()) {
                            arrObject->deleteOwnProperty(state, ObjectPropertyName(state, Value(i).toString(state)));
                        } else {
                            arrObject->defineOwnProperty(state, ObjectPropertyName(state, Value(i).toString(state)), ObjectPropertyDescriptor(newElement, ObjectPropertyDescriptor::AllPresent));
                        }
                        i++;
                    }
                } else {
                    Object* object = val.asObject();

                    ObjectPropertyNameVector keys;
                    if (!object->canUseOwnPropertyKeysFastPath()) {
                        auto keyValues = Object::enumerableOwnProperties(state, object, EnumerableOwnPropertiesType::Key);

                        for (size_t i = 0; i < keyValues.size(); ++i) {
                            keys.push_back(ObjectPropertyName(state, keyValues[i]));
                        }
                    } else {
                        object->enumeration(state, [](ExecutionState& state, Object* self, const ObjectPropertyName& P, const ObjectStructurePropertyDescriptor& desc, void* data) -> bool {
                            if (desc.isEnumerable()) {
                                ObjectPropertyNameVector* keys = (ObjectPropertyNameVector*)data;
                                keys->push_back(P);
                            }
                            return true;
                        },
                                            &keys);
                    }

                    for (auto key : keys) {
                        Value newElement = Walk(val, key);
                        if (newElement.isUndefined()) {
                            object->deleteOwnProperty(state, key);
                        } else {
                            object->defineOwnProperty(state, key, ObjectPropertyDescriptor(newElement, ObjectPropertyDescriptor::AllPresent));
                        }
                    }
                }
            }
            Value arguments[] = { name.toPlainValue(state), val };
            return Object::call(state, reviver, holder, 2, arguments);
        };
        return Walk(root, ObjectPropertyName(state, String::emptyString));
    }

    // 5
    return unfiltered;
}

static void builtinJSONArrayReplacerHelper(ExecutionState& state, ObjectPropertyNameVector& propertyList, Value property)
{
    String* item;

    if (property.isString()) {
        item = property.asString();
    } else if (property.isNumber()) {
        item = property.toString(state);
    } else if (property.isObject() && (property.asPointerValue()->isStringObject() || property.asPointerValue()->isNumberObject())) {
        item = property.toString(state);
    } else {
        return;
    }

    for (size_t i = 0; i < propertyList.size(); i++) {
        ObjectPropertyName& v = propertyList[i];
        if (v.toObjectStructurePropertyName(state).equals(item)) {
            return;
        }
    }
    propertyList.push_back(ObjectPropertyName(state, Value(item)));
}

static Optional<String*> builtinJSONStringifyStr(ExecutionState& state, ObjectPropertyName key, Object* holder,
                                                 StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList);
static String* builtinJSONStringifyJA(ExecutionState& state, Object* obj,
                                      StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList);
static String* builtinJSONStringifyJO(ExecutionState& state, Object* value,
                                      StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList);
static void builtinJSONStringifyQuote(ExecutionState& state, ObjectPropertyName value, LargeStringBuilder& product);
static String* builtinJSONStringifyQuote(ExecutionState& state, String* value);

// https://www.ecma-international.org/ecma-262/6.0/#sec-serializejsonproperty
static Optional<String*> builtinJSONStringifyStr(ExecutionState& state, ObjectPropertyName key, Object* holder,
                                                 StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList)
{
    Value value = holder->get(state, key).value(state, holder);
    if (value.isObject() || value.isBigInt()) {
        Value toJson = Object::getV(state, value, ObjectPropertyName(state, strings->toJSON));
        if (toJson.isCallable()) {
            Value arguments[] = { key.toPlainValue(state) };
            value = Object::call(state, toJson, value, 1, arguments);
        }
    }

    if (!replacerFunc.isUndefined()) {
        Value arguments[] = { key.toPlainValue(state), value };
        value = Object::call(state, replacerFunc, holder, 2, arguments);
    }

    if (value.isObject()) {
        if (value.asObject()->isNumberObject()) {
            value = Value(value.toNumber(state));
        } else if (value.asObject()->isStringObject()) {
            value = Value(value.toString(state));
        } else if (value.asObject()->isBooleanObject()) {
            value = Value(value.asObject()->asBooleanObject()->primitiveValue());
        } else if (value.asObject()->isBigIntObject()) {
            value = Value(value.asObject()->asBigIntObject()->primitiveValue());
        }
    }
    if (value.isNull()) {
        return strings->null.string();
    }
    if (value.isBoolean()) {
        return value.asBoolean() ? strings->stringTrue.string() : strings->stringFalse.string();
    }
    if (value.isString()) {
        return builtinJSONStringifyQuote(state, value.asString());
    }
    if (value.isNumber()) {
        double d = value.toNumber(state);
        if (std::isfinite(d)) {
            return value.toString(state);
        }
        return strings->null.string();
    }
    if (value.isBigInt()) {
        ErrorObject::throwBuiltinError(state, ErrorObject::TypeError, "Could not serialize a BigInt");
    }
    if (value.isObject() && !value.isCallable()) {
        if (value.asObject()->isArray(state)) {
            return builtinJSONStringifyJA(state, value.asObject(), strings, replacerFunc, stack, indent, gap, propertyListTouched, propertyList);
        } else {
            return builtinJSONStringifyJO(state, value.asObject(), strings, replacerFunc, stack, indent, gap, propertyListTouched, propertyList);
        }
    }

    return nullptr;
}

// https://www.ecma-international.org/ecma-262/6.0/#sec-serializejsonarray
static String* builtinJSONStringifyJA(ExecutionState& state, Object* obj,
                                      StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList)
{
    // 1
    for (size_t i = 0; i < stack.size(); i++) {
        Value& v = stack[i];
        if (v == Value(obj)) {
            ErrorObject::throwBuiltinError(state, ErrorObject::TypeError, strings->JSON.string(), false, strings->stringify.string(), ErrorObject::Messages::GlobalObject_JAError);
        }
    }
    // 2
    stack.push_back(Value(obj));
    // 3
    String* stepback = indent;
    // 4
    StringBuilder newIndent;
    newIndent.appendString(indent);
    newIndent.appendString(gap);
    indent = newIndent.finalize(&state);

    // 6, 7
    uint32_t len = obj->length(state);

    // Each array element requires at least 1 character for the value, and 1 character for the separator
    if (len / 2 > STRING_MAXIMUM_LENGTH) {
        ErrorObject::throwBuiltinError(state, ErrorObject::RangeError, strings->JSON.string(), false, strings->stringify.string(), ErrorObject::Messages::GlobalObject_JAError);
    }

    // 8 ~ 9
    uint32_t index = 0;
    StringBuilder finalValue;
    bool first = true;
    String* seperator = strings->asciiTable[','].string();

    finalValue.appendChar('[');
    while (index < len) {
        if (first) {
            if (gap->length()) {
                finalValue.appendChar('\n');
                finalValue.appendString(indent);
                StringBuilder seperatorBuilder;
                seperatorBuilder.appendChar(',');
                seperatorBuilder.appendChar('\n');
                seperatorBuilder.appendString(indent);
                seperator = seperatorBuilder.finalize(&state);
            }
            first = false;
        } else {
            finalValue.appendString(seperator);
        }

        auto strP = builtinJSONStringifyStr(state, ObjectPropertyName(state, Value(index).toString(state)), obj, strings, replacerFunc, stack, indent, gap, propertyListTouched, propertyList);
        if (strP) {
            finalValue.appendString(strP.value());
        } else {
            finalValue.appendString(strings->null.string());
        }
        index++;
    }

    if (!first && gap->length()) {
        finalValue.appendChar('\n');
        finalValue.appendString(stepback);
    }

    finalValue.appendChar(']');

    // 11
    stack.pop_back();
    // 12
    indent = stepback;

    return finalValue.finalize(&state);
}

// https://www.ecma-international.org/ecma-262/6.0/#sec-serializejsonobject
static String* builtinJSONStringifyJO(ExecutionState& state, Object* value,
                                      StaticStrings* strings, Value replacerFunc, ValueVectorWithInlineStorage& stack, String* indent, String* gap, bool& propertyListTouched, ObjectPropertyNameVector& propertyList)
{
    // 1
    for (size_t i = 0; i < stack.size(); i++) {
        if (stack[i] == value) {
            ErrorObject::throwBuiltinError(state, ErrorObject::TypeError, strings->JSON.string(), false, strings->stringify.string(), ErrorObject::Messages::GlobalObject_JOError);
        }
    }
    // 2
    stack.push_back(Value(value));
    // 3
    String* stepback = indent;
    // 4
    StringBuilder newIndent;
    newIndent.appendString(indent);
    newIndent.appendString(gap);
    indent = newIndent.finalize(&state);
    // 5, 6
    ObjectPropertyNameVector k;
    if (propertyListTouched) {
        k = propertyList;
    } else {
        auto keyValues = Object::enumerableOwnProperties(state, value, EnumerableOwnPropertiesType::Key);
        for (size_t i = 0; i < keyValues.size(); ++i) {
            k.push_back(ObjectPropertyName(state, keyValues[i]));
        }
    }

    // 7 ~ 9
    LargeStringBuilder finalValue;
    bool first = true;
    int len = k.size();
    String* seperator = strings->asciiTable[','].string();

    finalValue.appendChar('{');
    for (int i = 0; i < len; ++i) {
        auto strP = builtinJSONStringifyStr(state, k[i], value, strings, replacerFunc, stack, indent, gap, propertyListTouched, propertyList);
        if (strP) {
            if (first) {
                if (gap->length()) {
                    finalValue.appendChar('\n');
                    finalValue.appendString(indent);
                    StringBuilder seperatorBuilder;
                    seperatorBuilder.appendChar(',');
                    seperatorBuilder.appendChar('\n');
                    seperatorBuilder.appendString(indent);
                    seperator = seperatorBuilder.finalize(&state);
                }
                first = false;
            } else {
                finalValue.appendString(seperator);
            }

            builtinJSONStringifyQuote(state, k[i], finalValue);
            finalValue.appendChar(':');
            if (gap->length() != 0) {
                finalValue.appendChar(' ');
            }
            finalValue.appendString(strP.value());
        }
    }

    if (!first && gap->length()) {
        finalValue.appendChar('\n');
        finalValue.appendString(stepback);
    }

    finalValue.appendChar('}');

    // 11
    stack.pop_back();
    // 12
    indent = stepback;

    return finalValue.finalize(&state);
}

// https://www.ecma-international.org/ecma-262/6.0/#sec-quotejsonstring
static void builtinJSONStringifyQuote(ExecutionState& state, String* value, LargeStringBuilder& product)
{
    auto bad = value->bufferAccessData();
    product.appendChar('"');
    for (size_t i = 0; i < bad.length; ++i) {
        char16_t c = bad.charAt(i);

        switch (c) {
        case u'\"':
        case u'\\':
            product.appendChar('\\');
            product.appendChar(c);
            break;
        case u'\b':
            product.appendChar('\\');
            product.appendChar('b');
            break;
        case u'\f':
            product.appendChar('\\');
            product.appendChar('f');
            break;
        case u'\n':
            product.appendChar('\\');
            product.appendChar('n');
            break;
        case u'\r':
            product.appendChar('\\');
            product.appendChar('r');
            break;
        case u'\t':
            product.appendChar('\\');
            product.appendChar('t');
            break;
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 11:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
        case 22:
        case 23:
        case 24:
        case 25:
        case 26:
        case 27:
        case 28:
        case 29:
        case 30:
        case 31:
            product.appendChar('\\');
            product.appendChar('u');
            product.appendString(codePointTo4digitString(c));
            break;
        default:
            product.appendChar(c);
        }
    }
    product.appendChar('"');
}
static String* builtinJSONStringifyQuote(ExecutionState& state, String* value)
{
    LargeStringBuilder product;
    builtinJSONStringifyQuote(state, value, product);
    return product.finalize(&state);
}

static void builtinJSONStringifyQuote(ExecutionState& state, ObjectPropertyName value, LargeStringBuilder& product)
{
    String* str = value.toObjectStructurePropertyName(state).plainString();
    builtinJSONStringifyQuote(state, str, product);
}

static Value builtinJSONStringify(ExecutionState& state, Value thisValue, size_t argc, Value* argv, Optional<Object*> newTarget)
{
    auto strings = &state.context()->staticStrings();

    // 1, 2, 3
    Value value = argv[0];
    Value replacer = argv[1];
    Value space = argv[2];
    String* indent = String::emptyString;
    ValueVectorWithInlineStorage stack;
    ObjectPropertyNameVector propertyList;
    bool propertyListTouched = false;

    // 4
    Value replacerFunc;
    if (replacer.isObject()) {
        if (replacer.isCallable()) {
            replacerFunc = replacer;
        } else if (replacer.asObject()->isArrayObject()) {
            propertyListTouched = true;
            ArrayObject* arrObject = replacer.asObject()->asArrayObject();

            std::vector<Value::ValueIndex> indexes;
            arrObject->enumeration(state, [](ExecutionState& state, Object* self, const ObjectPropertyName& P, const ObjectStructurePropertyDescriptor& desc, void* data) -> bool {
                Value::ValueIndex idx = P.toPlainValue(state).toNumber(state);
                if (idx != Value::InvalidIndexValue) {
                    std::vector<Value::ValueIndex>* indexes = (std::vector<Value::ValueIndex>*)data;
                    indexes->push_back(idx);
                }
                return true;
            },
                                   &indexes);
            std::sort(indexes.begin(), indexes.end(), std::less<Value::ValueIndex>());
            for (uint32_t i = 0; i < indexes.size(); ++i) {
                Value property = arrObject->get(state, ObjectPropertyName(state, Value(indexes[i]))).value(state, arrObject);
                builtinJSONArrayReplacerHelper(state, propertyList, property);
            }
        } else if (replacer.asObject()->isArray(state)) {
            propertyListTouched = true;
            Object* replacerObj = replacer.asObject();
            uint64_t len = replacerObj->length(state);
            uint64_t k = 0;

            while (k < len) {
                Value v = replacerObj->getIndexedProperty(state, Value(k)).value(state, replacerObj);
                builtinJSONArrayReplacerHelper(state, propertyList, v);
                k++;
            }
        }
    }

    // 5
    if (space.isObject()) {
        if (space.isPointerValue() && space.asPointerValue()->isNumberObject()) {
            space = Value(space.toNumber(state));
        } else if (space.isPointerValue() && space.asPointerValue()->isStringObject()) {
            space = space.toString(state);
        }
    }

    // 6, 7, 8
    String* gap = String::emptyString;
    if (space.isNumber()) {
        int space_cnt = std::min(space.toInteger(state), 10.0);
        if (space_cnt >= 1) {
            UTF8StringData gapData;
            gapData.resizeWithUninitializedValues(space_cnt);
            for (int i = 0; i < space_cnt; i++) {
                gapData[i] = ' ';
            }
            gap = new ASCIIString(gapData.data(), gapData.length());
        }
    } else if (space.isString()) {
        if (space.asString()->length() <= 10) {
            gap = space.asString();
        } else {
            gap = space.asString()->substring(0, 10);
        }
    }

    // 9
    Object* wrapper = new Object(state);
    // 10
    wrapper->defineOwnProperty(state, ObjectPropertyName(state, String::emptyString), ObjectPropertyDescriptor(value, ObjectPropertyDescriptor::AllPresent));
    auto str = builtinJSONStringifyStr(state, ObjectPropertyName(state, Value(String::emptyString)), wrapper, strings, replacerFunc, stack, indent, gap, propertyListTouched, propertyList);
    if (str) {
        return str.value();
    }
    return Value();
}

void GlobalObject::installJSON(ExecutionState& state)
{
    m_json = new Object(state);
    m_json->setGlobalIntrinsicObject(state);
    m_json->defineOwnPropertyThrowsException(state, ObjectPropertyName(state, Value(state.context()->vmInstance()->globalSymbols().toStringTag)),
                                             ObjectPropertyDescriptor(Value(state.context()->staticStrings().JSON.string()), (ObjectPropertyDescriptor::PresentAttribute)(ObjectPropertyDescriptor::ConfigurablePresent)));


    defineOwnProperty(state, ObjectPropertyName(state.context()->staticStrings().JSON),
                      ObjectPropertyDescriptor(m_json, (ObjectPropertyDescriptor::PresentAttribute)(ObjectPropertyDescriptor::WritablePresent | ObjectPropertyDescriptor::ConfigurablePresent)));

    m_jsonParse = new NativeFunctionObject(state, NativeFunctionInfo(state.context()->staticStrings().parse, builtinJSONParse, 2, NativeFunctionInfo::Strict));
    m_json->defineOwnProperty(state, ObjectPropertyName(state.context()->staticStrings().parse),
                              ObjectPropertyDescriptor(m_jsonParse,
                                                       (ObjectPropertyDescriptor::PresentAttribute)(ObjectPropertyDescriptor::WritablePresent | ObjectPropertyDescriptor::ConfigurablePresent)));

    m_jsonStringify = new NativeFunctionObject(state, NativeFunctionInfo(state.context()->staticStrings().stringify, builtinJSONStringify, 3, NativeFunctionInfo::Strict));
    m_json->defineOwnProperty(state, ObjectPropertyName(state.context()->staticStrings().stringify),
                              ObjectPropertyDescriptor(m_jsonStringify,
                                                       (ObjectPropertyDescriptor::PresentAttribute)(ObjectPropertyDescriptor::WritablePresent | ObjectPropertyDescriptor::ConfigurablePresent)));
}
} // namespace Escargot
