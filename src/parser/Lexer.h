/*
 * Copyright (c) 2016-present Samsung Electronics Co., Ltd
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
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

#ifndef __EscargotLexer__
#define __EscargotLexer__

#include "parser/esprima_cpp/esprima.h"

namespace Escargot {

namespace EscargotLexer {

enum Token {
    EOFToken,
    IdentifierToken,
    BooleanLiteralToken,
    KeywordToken,
    NullLiteralToken,
    NumericLiteralToken,
    PunctuatorToken,
    StringLiteralToken,
    RegularExpressionToken,
    TemplateToken,
    InvalidToken
};

enum PunctuatorKind : uint8_t {
    LeftParenthesis,
    RightParenthesis,
    LeftBrace,
    RightBrace,
    Period,
    PeriodPeriodPeriod,
    Comma,
    Colon,
    SemiColon,
    LeftSquareBracket,
    RightSquareBracket,
    GuessMark,
    Wave,
    UnsignedRightShift,
    RightShift,
    LeftShift,
    Plus,
    Minus,
    Multiply,
    Divide,
    Mod,
    ExclamationMark,
    StrictEqual,
    NotStrictEqual,
    Equal,
    NotEqual,
    LogicalAnd,
    LogicalOr,
    PlusPlus,
    MinusMinus,
    BitwiseAnd,
    BitwiseOr,
    BitwiseXor,
    LeftInequality,
    RightInequality,
    InPunctuator,
    InstanceOfPunctuator,

    Substitution,
    UnsignedRightShiftEqual,
    RightShiftEqual,
    LeftShiftEqual,
    PlusEqual,
    MinusEqual,
    MultiplyEqual,
    DivideEqual,
    ModEqual,
    // ExclamationMarkEqual,
    BitwiseAndEqual,
    BitwiseOrEqual,
    BitwiseXorEqual,
    LeftInequalityEqual,
    RightInequalityEqual,
    SubstitutionEnd,

    Arrow,
    PunctuatorKindEnd,
};

enum KeywordKind : uint8_t {
    NotKeyword,
    IfKeyword,
    InKeyword,
    DoKeyword,
    VarKeyword,
    ForKeyword,
    NewKeyword,
    TryKeyword,
    ThisKeyword,
    ElseKeyword,
    CaseKeyword,
    VoidKeyword,
    WithKeyword,
    EnumKeyword,
    AwaitKeyword,
    WhileKeyword,
    BreakKeyword,
    CatchKeyword,
    ThrowKeyword,
    ConstKeyword,
    ClassKeyword,
    SuperKeyword,
    ReturnKeyword,
    TypeofKeyword,
    DeleteKeyword,
    SwitchKeyword,
    ExportKeyword,
    ImportKeyword,
    DefaultKeyword,
    FinallyKeyword,
    ExtendsKeyword,
    FunctionKeyword,
    ContinueKeyword,
    DebuggerKeyword,
    InstanceofKeyword,
    StrictModeReservedWord,
    ImplementsKeyword,
    InterfaceKeyword,
    PackageKeyword,
    PrivateKeyword,
    ProtectedKeyword,
    PublicKeyword,
    StaticKeyword,
    YieldKeyword,
    LetKeyword,
    KeywordKindEnd
};

enum {
    LexerIsCharIdentStart = (1 << 0),
    LexerIsCharIdent = (1 << 1),
    LexerIsCharWhiteSpace = (1 << 2),
    LexerIsCharLineTerminator = (1 << 3)
};

extern char g_asciiRangeCharMap[128];

// ECMA-262 11.2 White Space
NEVER_INLINE bool isWhiteSpaceSlowCase(char16_t ch);
ALWAYS_INLINE bool isWhiteSpace(char16_t ch)
{
    if (LIKELY(ch < 128)) {
        return g_asciiRangeCharMap[ch] & LexerIsCharWhiteSpace;
    }
    return isWhiteSpaceSlowCase(ch);
}

// ECMA-262 11.3 Line Terminators
ALWAYS_INLINE bool isLineTerminator(char16_t ch)
{
    if (LIKELY(ch < 128)) {
        return g_asciiRangeCharMap[ch] & LexerIsCharLineTerminator;
    }
    return UNLIKELY(ch == 0x2028 || ch == 0x2029);
}

ALWAYS_INLINE bool isWhiteSpaceOrLineTerminator(char16_t ch)
{
    if (LIKELY(ch < 128)) {
        return g_asciiRangeCharMap[ch] & (LexerIsCharWhiteSpace | LexerIsCharLineTerminator);
    }
    return UNLIKELY(ch == 0x2028 || ch == 0x2029 || isWhiteSpaceSlowCase(ch));
}

struct ScanTemplateResult {
    // ScanTemplateResult is allocated on the GC heap
    // because it holds GC pointers, raw and member pointer of valueCooked.
    inline void* operator new(size_t size)
    {
        return GC_MALLOC(size);
    }
    void* operator new[](size_t size) = delete;

    UTF16StringData valueCooked;
    UTF16StringData valueRaw;
    bool head;
    bool tail;
};

struct ScanRegExpResult : public gc {
    ScanRegExpResult()
        : body(nullptr)
        , flags(nullptr)
    {
    }
    String* body;
    String* flags;
};

class ErrorHandler {
public:
    ErrorHandler()
    {
    }

    static void throwError(size_t index, size_t line, size_t col, String* description, ErrorObject::Code code);
};

namespace Messages {
extern const char* InvalidHexEscapeSequence;
extern const char* UnexpectedTokenIllegal;
extern const char* UnterminatedRegExp;
extern const char* TemplateOctalLiteral;
}

typedef std::tuple<StringBufferAccessData, String*> ScannerIDResult;

template<typename SourceReader>
class Scanner {
public:
    class ScannerResult {
    public:
        ScannerResult()
            : type(InvalidToken)
            , startWithZero(false)
            , octal(false)
            , hasAllocatedString(false)
            , hasNonComputedNumberLiteral(false)
            , hasKeywordButUseString(false)
            , prec(0)
            , lineNumber(0)
            , lineStart(0)
            , start(0)
            , end(0)
            , valueRegexp()
        {
        }

        // ScannerResult always allocated on the stack
        MAKE_STACK_ALLOCATED();

        unsigned char type : 4;
        bool startWithZero : 1;
        bool octal : 1;
        bool hasAllocatedString : 1;
        bool hasIntegerNumberLiteral : 1;
        bool hasNonComputedNumberLiteral : 1;
        bool hasKeywordButUseString : 1;
        unsigned char prec : 8; // max prec is 11
        // we don't needs init prec.
        // prec is initialized by another function before use

        size_t lineNumber;
        size_t lineStart;
        size_t start;
        size_t end;

        union {
            PunctuatorKind valuePunctuatorKind;
            String* valueStringIfNewlyAllocated;
            uint32_t valueInteger; // TODO use 64bit integer in 64bit
            double* valueNumber;
            ScanTemplateResult* valueTemplate;
            ScanRegExpResult* valueRegexp;
            KeywordKind valueKeywordKind;
        };

        StringView relatedSource(const StringView& source) const;
        StringView valueStringLiteral(Scanner<SourceReader>* scannerInstance);
        Value valueStringLiteralForAST(Scanner<SourceReader>* scannerInstance);
        double valueNumberLiteral(Scanner<SourceReader>* scannerInstance);

        inline operator bool() const
        {
            return this->type != InvalidToken;
        }

        inline void reset()
        {
            this->type = InvalidToken;
        }

        void setResult(Token type, size_t lineNumber, size_t lineStart, size_t start, size_t end)
        {
            this->type = type;
            this->startWithZero = false;
            this->octal = false;
            this->hasAllocatedString = false;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->hasKeywordButUseString = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;
            this->valueNumber = nullptr;
        }

        void setPunctuatorResult(size_t lineNumber, size_t lineStart, size_t start, size_t end, PunctuatorKind p)
        {
            this->type = Token::PunctuatorToken;
            this->startWithZero = false;
            this->octal = false;
            this->hasAllocatedString = false;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->hasKeywordButUseString = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;
            this->valuePunctuatorKind = p;
        }

        void setKeywordResult(size_t lineNumber, size_t lineStart, size_t start, size_t end, KeywordKind p)
        {
            this->type = Token::KeywordToken;
            this->startWithZero = false;
            this->octal = false;
            this->hasAllocatedString = false;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->hasKeywordButUseString = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;
            this->valueKeywordKind = p;
        }

        void setResult(Token type, String* s, size_t lineNumber, size_t lineStart, size_t start, size_t end, bool octal = false)
        {
            this->type = type;
            this->startWithZero = false;
            this->octal = octal;
            this->hasKeywordButUseString = true;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;

            this->hasAllocatedString = true;
            this->valueStringIfNewlyAllocated = s;
        }

        void setStringViewKindResult(Token type, size_t lineNumber, size_t lineStart, size_t start, size_t end, bool octal = false)
        {
            this->type = type;
            this->startWithZero = false;
            this->octal = octal;
            this->hasKeywordButUseString = true;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;

            this->hasAllocatedString = false;
            this->valueStringIfNewlyAllocated = nullptr;
        }

        void setNumericLiteralResult(double value, size_t lineNumber, size_t lineStart, size_t start, size_t end, bool hasNonComputedNumberLiteral)
        {
            this->type = Token::NumericLiteralToken;
            this->startWithZero = false;
            this->octal = false;
            this->hasAllocatedString = false;
            this->hasKeywordButUseString = true;
            this->hasNonComputedNumberLiteral = hasNonComputedNumberLiteral;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;

            if (!hasNonComputedNumberLiteral) {
                if (uint32_t(value) != value || (!value && std::signbit(value) /* test -0.0 */)) {
                    this->hasIntegerNumberLiteral = false;
                    this->valueNumber = new(PointerFreeGC) double(value);
                } else {
                    this->hasIntegerNumberLiteral = true;
                    this->valueInteger = value;
                }
            }
        }

        void setTemplateTokenResult(ScanTemplateResult* value, size_t lineNumber, size_t lineStart, size_t start, size_t end)
        {
            this->type = Token::TemplateToken;
            this->startWithZero = false;
            this->octal = false;
            this->hasAllocatedString = false;
            this->hasKeywordButUseString = true;
            this->hasNonComputedNumberLiteral = false;
            this->hasIntegerNumberLiteral = false;
            this->lineNumber = lineNumber;
            this->lineStart = lineStart;
            this->start = start;
            this->end = end;
            this->valueTemplate = value;
        }

    private:
        void constructStringLiteral(Scanner<SourceReader>* scannerInstance);
        void constructStringLiteralHelperAppendUTF16(Scanner<SourceReader>* scannerInstance, char16_t ch, UTF16StringDataNonGCStd& stringUTF16, bool& isEveryCharLatin1);
    };

    // Keep ScannerResult small
    COMPILE_ASSERT(sizeof(ScannerResult) <= sizeof(size_t) * 6, "");

    StringView source;
    StringBufferAccessData sourceBufferAccessData;
    ::Escargot::Context* escargotContext;
    // trackComment: boolean;

    size_t length;
    size_t index;
    size_t lineNumber;
    size_t lineStart;

    ~Scanner()
    {
    }

    Scanner(::Escargot::Context* escargotContext, StringView code, size_t startLine = 0, size_t startColumn = 0);

    // Scanner always allocated on the stack
    MAKE_STACK_ALLOCATED();

    bool eof()
    {
        return index >= length;
    }

    ALWAYS_INLINE void throwUnexpectedToken(const char* message = Messages::UnexpectedTokenIllegal)
    {
        ErrorHandler::throwError(this->index, this->lineNumber, this->index - this->lineStart + 1, new ASCIIString(message), ErrorObject::SyntaxError);
    }

    // ECMA-262 11.4 Comments

    void skipSingleLineComment(void);
    void skipMultiLineComment(void);

    char16_t sourceCharAt(const size_t& idx)
    {
        SourceReader reader;
        return reader(sourceBufferAccessData, idx);
    }

    ALWAYS_INLINE void scanComments()
    {
        bool start = (this->index == 0);
        while (LIKELY(!this->eof())) {
            char16_t ch = sourceCharAt(this->index);

            if (isWhiteSpace(ch)) {
                ++this->index;
            } else if (isLineTerminator(ch)) {
                ++this->index;
                if (ch == 0x0D && sourceCharAt(this->index) == 0x0A) {
                    ++this->index;
                }
                ++this->lineNumber;
                this->lineStart = this->index;
                start = true;
            } else if (ch == 0x2F) { // U+002F is '/'
                ch = sourceCharAt(this->index + 1);
                if (ch == 0x2F) {
                    this->index += 2;
                    this->skipSingleLineComment();
                    start = true;
                } else if (ch == 0x2A) { // U+002A is '*'
                    this->index += 2;
                    this->skipMultiLineComment();
                } else {
                    break;
                }
            } else if (start && ch == 0x2D) { // U+002D is '-'
                // U+003E is '>'
                if ((sourceCharAt(this->index + 1) == 0x2D) && (sourceCharAt(this->index + 2) == 0x3E)) {
                    // '-->' is a single-line comment
                    this->index += 3;
                    this->skipSingleLineComment();
                } else {
                    break;
                }
            } else if (ch == 0x3C) { // U+003C is '<'
                if (this->length > this->index + 4) {
                    if (sourceCharAt(this->index + 1) == '!'
                        && sourceCharAt(this->index + 2) == '-'
                        && sourceCharAt(this->index + 3) == '-') {
                        this->index += 4; // `<!--`
                        this->skipSingleLineComment();
                    } else {
                        break;
                    }
                } else {
                    break;
                }

            } else {
                break;
            }
        }
    }

    bool isFutureReservedWord(const StringView& id);

    template <typename T>
    bool isStrictModeReservedWord(const T& id)
    {
        const StringBufferAccessData& data = id.bufferAccessData();
        switch (data.length) {
        case 3: // let
            return data.equalsSameLength("let");
        case 5: // yield
            return data.equalsSameLength("yield");
        case 6: // static public
            return data.equalsSameLength("static") || data.equalsSameLength("public");
        case 7: // private package
            return data.equalsSameLength("private") || data.equalsSameLength("package");
        case 9: // protected interface
            return data.equalsSameLength("protected") || data.equalsSameLength("interface");
        case 10: // implements
            return data.equalsSameLength("implements");
        }

        return false;
    }

    template <typename T>
    bool isRestrictedWord(const T& id)
    {
        const StringBufferAccessData& data = id.bufferAccessData();
        if (data.length == 4) {
            return data.equalsSameLength("eval");
        } else if (data.length == 9) {
            return data.equalsSameLength("arguments");
        } else {
            return false;
        }
    }

    char32_t codePointAt(size_t i)
    {
        char32_t cp, first, second;
        cp = sourceCharAt(i);
        if (cp >= 0xD800 && cp <= 0xDBFF) {
            second = sourceCharAt(i + 1);
            if (second >= 0xDC00 && second <= 0xDFFF) {
                first = cp;
                cp = (first - 0xD800) * 0x400 + second - 0xDC00 + 0x10000;
            }
        }

        return cp;
    }

    // ECMA-262 11.8.6 Template Literal Lexical Components
    void scanTemplate(Scanner<SourceReader>::ScannerResult* token, bool head = false);

    // ECMA-262 11.8.5 Regular Expression Literals
    void scanRegExp(Scanner<SourceReader>::ScannerResult* token);

    void lex(Scanner<SourceReader>::ScannerResult* token);

private:
    ALWAYS_INLINE char16_t peekChar()
    {
        return sourceCharAt(this->index);
    }

    char32_t scanHexEscape(char prefix);
    char32_t scanUnicodeCodePointEscape();

    uint16_t octalToDecimal(char16_t ch, bool octal);

    ScannerIDResult getIdentifier();
    ScannerIDResult getComplexIdentifier();

    // ECMA-262 11.7 Punctuators
    void scanPunctuator(Scanner<SourceReader>::ScannerResult* token, char16_t ch0);

    // ECMA-262 11.8.3 Numeric Literals
    void scanHexLiteral(Scanner<SourceReader>::ScannerResult* token, size_t start);
    void scanBinaryLiteral(Scanner<SourceReader>::ScannerResult* token, size_t start);
    void scanOctalLiteral(Scanner<SourceReader>::ScannerResult* token, char16_t prefix, size_t start);

    bool isImplicitOctalLiteral();
    void scanNumericLiteral(Scanner<SourceReader>::ScannerResult* token);

    // ECMA-262 11.8.4 String Literals
    void scanStringLiteral(Scanner<SourceReader>::ScannerResult* token);

    // ECMA-262 11.6 Names and Keywords
    ALWAYS_INLINE void scanIdentifier(Scanner<SourceReader>::ScannerResult* token, char16_t ch0);

    String* scanRegExpBody();
    String* scanRegExpFlags();
};

class SourceReaderAny {
public:
    char16_t operator()(const StringBufferAccessData& d, const size_t& idx)
    {
        return d.charAt(idx);
    }
};

class SourceReader8Bit {
public:
    char16_t operator()(const StringBufferAccessData& d, const size_t& idx)
    {
        return d.uncheckedCharAtFor8Bit(idx);
    }
};

class SourceReader16Bit {
public:
    char16_t operator()(const StringBufferAccessData& d, const size_t& idx)
    {
        return d.uncheckedCharAtFor16Bit(idx);
    }
};

}
}

#endif
