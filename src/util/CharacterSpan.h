/*
 * Copyright (c) 2026-present Samsung Electronics Co., Ltd
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 * USA
 */

#ifndef __EscargotCharacterSpan__
#define __EscargotCharacterSpan__

#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace Escargot {

// Find either delimiter or a code unit below limit in a non-null, bounded
// span. Word-parallel tests work on every architecture and byte order.
// On little-endian machines the first flagged lane is the exact first match;
// subtraction may only flag additional, later lanes through a borrow. Other
// byte orders check the flagged word scalarly.
// Comparisons use unsigned code units; limit must be at most half their range.
template <bool collectCodeUnits, typename CharType>
inline size_t findFirstOfOrLessThanImpl(const CharType* data, size_t length,
                                        CharType first, CharType second, CharType limit,
                                        uintptr_t& codeUnits)
{
    static_assert(sizeof(CharType) == 1 || sizeof(CharType) == 2, "Expected 8-bit or UTF-16 code units");
    using Word = uintptr_t;
    using UnsignedChar = typename std::make_unsigned<CharType>::type;
    constexpr unsigned bits = sizeof(CharType) * 8;
    constexpr Word ones = std::numeric_limits<Word>::max() / ((Word(1) << bits) - 1);
    constexpr Word highBits = ones << (bits - 1);
    constexpr size_t lanes = sizeof(Word) / sizeof(CharType);
    const Word firstWord = ones * static_cast<UnsignedChar>(first);
    const Word secondWord = ones * static_cast<UnsignedChar>(second);
    const Word limitWord = ones * static_cast<UnsignedChar>(limit);
    ASSERT(static_cast<UnsignedChar>(limit) <= (Word(1) << (bits - 1)));

    size_t position = 0;
    while (length - position >= lanes) {
        Word word;
        memcpy(&word, data + position, sizeof(word));
        const Word a = word ^ firstWord;
        const Word b = word ^ secondWord;
        const Word firstMatch = (a - ones) & ~a & highBits;
        const Word secondMatch = (b - ones) & ~b & highBits;
        const Word belowLimit = (word - limitWord) & ~word & highBits;
        const Word matches = firstMatch | secondMatch | belowLimit;
        if (matches) {
#if defined(ESCARGOT_LITTLE_ENDIAN) && (defined(COMPILER_GCC) || defined(COMPILER_CLANG))
            const size_t firstLane = __builtin_ctzll(static_cast<unsigned long long>(matches)) / bits;
            if (collectCodeUnits) {
                codeUnits |= word & ((Word(1) << (firstLane * bits)) - 1);
            }
            return position + firstLane;
#else
            break;
#endif
        }
        if (collectCodeUnits) {
            codeUnits |= word;
        }
        position += lanes;
    }
    while (position < length) {
        const UnsignedChar c = static_cast<UnsignedChar>(data[position]);
        if (c == static_cast<UnsignedChar>(first) || c == static_cast<UnsignedChar>(second) || c < static_cast<UnsignedChar>(limit)) {
            break;
        }
        if (collectCodeUnits) {
            codeUnits |= c;
        }
        position++;
    }
    return position;
}

template <typename CharType>
inline size_t findFirstOfOrLessThan(const CharType* data, size_t length,
                                    CharType first, CharType second, CharType limit)
{
    uintptr_t ignored = 0;
    return findFirstOfOrLessThanImpl<false>(data, length, first, second, limit, ignored);
}

// Also accumulate the code-unit bits of the accepted prefix, excluding the
// delimiter and every later lane. A caller can classify width while scanning.
template <typename CharType>
inline size_t findFirstOfOrLessThan(const CharType* data, size_t length,
                                    CharType first, CharType second, CharType limit,
                                    uintptr_t& codeUnits)
{
    return findFirstOfOrLessThanImpl<true>(data, length, first, second, limit, codeUnits);
}

} // namespace Escargot

#endif
