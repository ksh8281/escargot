#ifndef __EscargotPropertyName__
#define __EscargotPropertyName__

#include "runtime/AtomicString.h"
#include "runtime/ExecutionState.h"
#include "runtime/String.h"

namespace Escargot {

class PropertyName {
public:
    PropertyName()
    {
        m_data = ((size_t)AtomicString().string()) + 1;
        ASSERT(m_data);
    }

    PropertyName(ExecutionState& state, const AtomicString& atomicString)
    {
        m_data = ((size_t)atomicString.string() + 1);
        ASSERT(m_data);
    }

    PropertyName(const AtomicString& atomicString)
    {
        m_data = ((size_t)atomicString.string() + 1);
        ASSERT(m_data);
    }

    PropertyName(ExecutionState& state, String* string);
    String* string() const
    {
        if (hasAtomicString()) {
            return (String*)(m_data - 1);
        } else {
            return (String*)m_data;
        }
    }

    AtomicString atomicString(ExecutionState& state)
    {
        if (hasAtomicString()) {
            return AtomicString((String*)(m_data - 1));
        } else {
            return AtomicString(state, (String*)m_data);
        }
    }

    ALWAYS_INLINE friend bool operator==(const PropertyName& a, const PropertyName& b);
    ALWAYS_INLINE friend bool operator!=(const PropertyName& a, const PropertyName& b);

    size_t rawData() const
    {
        return m_data;
    }

protected:
    ALWAYS_INLINE bool hasAtomicString() const
    {
        return LIKELY(m_data & 1);
    }

    size_t m_data;
    // AtomicString <- saves its (String* | 1)
    // String* <- saves pointer
};

ALWAYS_INLINE bool operator==(const PropertyName& a, const PropertyName& b)
{
    if (LIKELY(LIKELY(a.hasAtomicString()) && LIKELY(b.hasAtomicString()))) {
        return a.m_data == b.m_data;
    } else {
        return a.string()->equals(b.string());
    }
}

ALWAYS_INLINE bool operator!=(const PropertyName& a, const PropertyName& b)
{
    return !operator==(a, b);
}
}

#endif