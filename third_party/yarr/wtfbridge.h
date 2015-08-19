/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef yarr_wtfbridge_h
#define yarr_wtfbridge_h

/*
 * WTF compatibility layer. This file provides various type and data
 * definitions for use by Yarr.
 */

#include <stdio.h>
#include <stdarg.h>
#include "Escargot.h"
#define WTF_OS_UNIX 1
#define JS_HOWMANY(x,y) (((x)+(y)-1)/(y))
#define JS_ROUNDUP(x,y) (JS_HOWMANY(x,y)*(y))
template <class T>
struct AlignmentTestStruct
{
    char c;
    T t;
};

/* This macro determines the alignment requirements of a type. */
#define JS_ALIGNMENT_OF(t_) \
  (sizeof(AlignmentTestStruct<t_>) - sizeof(t_))

//#include "jsstr.h"
//#include "jsprvtd.h"
//#include "vm/String.h"
//#include "assembler/wtf/Platform.h"
//#include "assembler/jit/ExecutableAllocator.h"
//#include "js/TemplateLib.h"

template <typename T, typename U> struct IsSameType {
    static const bool value = false;
};

template <typename T> struct IsSameType<T, T> {
    static const bool value = true;
};

#define CRASH RELEASE_ASSERT_NOT_REACHED
#include "CheckedArithmetic.h"

/*
 * Basic type definitions.
 */

#define JS_EXPORT_PRIVATE
#define WTF_MAKE_FAST_ALLOCATED
#define NO_RETURN_DUE_TO_ASSERT
typedef escargot::u16string String;
typedef escargot::u16string UString;
typedef char16_t UChar;
typedef char LChar;

#include "PageAllocation.h"

enum TextCaseSensitivity {
    TextCaseSensitive,
    TextCaseInsensitive
};


namespace JSC { namespace Yarr {
class Unicode {
  public:
    //TODO use ICU!
    static UChar toUpper(UChar c) { return toupper(c); }
    static UChar toLower(UChar c) { return tolower(c); }
};


template <typename T> struct RemovePointer {
    typedef T Type;
};

template <typename T> struct RemovePointer<T*> {
    typedef T Type;
};

template <typename T> struct RemoveReference {
    typedef T Type;
};

template <typename T> struct RemoveReference<T&> {
    typedef T Type;
};


#ifdef NDEBUG
#define CHECK_REF_COUNTED_LIFECYCLE 0
#else
#define CHECK_REF_COUNTED_LIFECYCLE 1
#endif

// This base class holds the non-template methods and attributes.
// The RefCounted class inherits from it reducing the template bloat
// generated by the compiler (technique called template hoisting).
class RefCountedBase {
public:
    void ref()
    {
#if CHECK_REF_COUNTED_LIFECYCLE
        ASSERT(!m_deletionHasBegun);
        ASSERT(!m_adoptionIsRequired);
#endif
        ++m_refCount;
    }

    bool hasOneRef() const
    {
#if CHECK_REF_COUNTED_LIFECYCLE
        ASSERT(!m_deletionHasBegun);
#endif
        return m_refCount == 1;
    }

    unsigned refCount() const
    {
        return m_refCount;
    }

    void relaxAdoptionRequirement()
    {
#if CHECK_REF_COUNTED_LIFECYCLE
        ASSERT(!m_deletionHasBegun);
        ASSERT(m_adoptionIsRequired);
        m_adoptionIsRequired = false;
#endif
    }

protected:
    RefCountedBase()
        : m_refCount(1)
#if CHECK_REF_COUNTED_LIFECYCLE
        , m_deletionHasBegun(false)
        , m_adoptionIsRequired(true)
#endif
    {
    }

    ~RefCountedBase()
    {
#if CHECK_REF_COUNTED_LIFECYCLE
        ASSERT(m_deletionHasBegun);
        ASSERT(!m_adoptionIsRequired);
#endif
    }

    // Returns whether the pointer should be freed or not.
    bool derefBase()
    {
#if CHECK_REF_COUNTED_LIFECYCLE
        ASSERT(!m_deletionHasBegun);
        ASSERT(!m_adoptionIsRequired);
#endif

        ASSERT(m_refCount);
        unsigned tempRefCount = m_refCount - 1;
        if (!tempRefCount) {
#if CHECK_REF_COUNTED_LIFECYCLE
            m_deletionHasBegun = true;
#endif
            return true;
        }
        m_refCount = tempRefCount;
        return false;
    }

#if CHECK_REF_COUNTED_LIFECYCLE
    bool deletionHasBegun() const
    {
        return m_deletionHasBegun;
    }
#endif

private:

#if CHECK_REF_COUNTED_LIFECYCLE
    friend void adopted(RefCountedBase*);
#endif

    unsigned m_refCount;
#if CHECK_REF_COUNTED_LIFECYCLE
    bool m_deletionHasBegun;
    bool m_adoptionIsRequired;
#endif
};

#if CHECK_REF_COUNTED_LIFECYCLE
inline void adopted(RefCountedBase* object)
{
    if (!object)
        return;
    ASSERT(!object->m_deletionHasBegun);
    object->m_adoptionIsRequired = false;
}
#endif

template<typename T> class RefCounted : public RefCountedBase {
    //WTF_MAKE_NONCOPYABLE(RefCounted); WTF_MAKE_FAST_ALLOCATED;
public:
    void deref()
    {
        if (derefBase())
            delete static_cast<T*>(this);
    }

protected:
    RefCounted() { }
    ~RefCounted()
    {
    }
};

template<typename T> class PassRef;
template<typename T> class PassRefPtr;
template<typename T> class Ref;
template<typename T> class RefPtr;

template<typename T> PassRef<T> adoptRef(T&);

inline void adopted(const void*) { }

template<typename T> class PassRef {
public:
    PassRef(T&);
    PassRef(PassRef&&);
    template<typename U> PassRef(PassRef<U>);

    const T& get() const;
    T& get();

    void dropRef();
    T& leakRef() WARN_UNUSED_RETURN;

#ifndef NDEBUG
    ~PassRef();
#endif

private:
    friend PassRef adoptRef<T>(T&);

    template<typename U> friend class PassRef;
    template<typename U> friend class PassRefPtr;
    template<typename U> friend class Ref;
    template<typename U> friend class RefPtr;

    enum AdoptTag { Adopt };
    PassRef(T&, AdoptTag);

    T& m_reference;

#ifndef NDEBUG
    bool m_gaveUpReference;
#endif
};

template<typename T> inline PassRef<T>::PassRef(T& reference)
    : m_reference(reference)
#ifndef NDEBUG
    , m_gaveUpReference(false)
#endif
{
    reference.ref();
}

template<typename T> inline PassRef<T>::PassRef(PassRef&& other)
    : m_reference(other.leakRef())
#ifndef NDEBUG
    , m_gaveUpReference(false)
#endif
{
}

template<typename T> template<typename U> inline PassRef<T>::PassRef(PassRef<U> other)
    : m_reference(other.leakRef())
#ifndef NDEBUG
    , m_gaveUpReference(false)
#endif
{
}

#ifndef NDEBUG

template<typename T> PassRef<T>::~PassRef()
{
    ASSERT(m_gaveUpReference);
}

#endif

template<typename T> inline void PassRef<T>::dropRef()
{
    ASSERT(!m_gaveUpReference);
    m_reference.deref();
#ifndef NDEBUG
    m_gaveUpReference = true;
#endif
}

template<typename T> inline const T& PassRef<T>::get() const
{
    ASSERT(!m_gaveUpReference);
    return m_reference;
}

template<typename T> inline T& PassRef<T>::get()
{
    ASSERT(!m_gaveUpReference);
    return m_reference;
}

template<typename T> inline T& PassRef<T>::leakRef()
{
#ifndef NDEBUG
    ASSERT(!m_gaveUpReference);
    m_gaveUpReference = true;
#endif
    return m_reference;
}

template<typename T> inline PassRef<T>::PassRef(T& reference, AdoptTag)
    : m_reference(reference)
#ifndef NDEBUG
    , m_gaveUpReference(false)
#endif
{
}

template<typename T> inline PassRef<T> adoptRef(T& reference)
{
    adopted(&reference);
    return PassRef<T>(reference, PassRef<T>::Adopt);
}

template<typename T, typename... Args> inline PassRef<T> createRefCounted(Args&&... args)
{
    return adoptRef(*new T(std::forward<Args>(args)...));
}

template<typename T> PassRefPtr<T> adoptRef(T*);

template<typename T> ALWAYS_INLINE void refIfNotNull(T* ptr)
{
    if (LIKELY(ptr != nullptr))
        ptr->ref();
}

template<typename T> ALWAYS_INLINE void derefIfNotNull(T* ptr)
{
    if (LIKELY(ptr != nullptr))
        ptr->deref();
}

template<typename T> class PassRefPtr {
public:
    PassRefPtr() : m_ptr(nullptr) { }
    PassRefPtr(T* ptr) : m_ptr(ptr) { refIfNotNull(ptr); }
    // It somewhat breaks the type system to allow transfer of ownership out of
    // a const PassRefPtr. However, it makes it much easier to work with PassRefPtr
    // temporaries, and we don't have a need to use real const PassRefPtrs anyway.
    PassRefPtr(const PassRefPtr& o) : m_ptr(o.leakRef()) { }
    template<typename U> PassRefPtr(const PassRefPtr<U>& o) : m_ptr(o.leakRef()) { }

    ALWAYS_INLINE ~PassRefPtr() { derefIfNotNull(m_ptr); }

    template<typename U> PassRefPtr(const RefPtr<U>&);
    template<typename U> PassRefPtr(PassRef<U> reference) : m_ptr(&reference.leakRef()) { }

    T* get() const { return m_ptr; }

    T* leakRef() const WARN_UNUSED_RETURN;

    T& operator*() const { return *m_ptr; }
    T* operator->() const { return m_ptr; }

    bool operator!() const { return !m_ptr; }

    // This conversion operator allows implicit conversion to bool but not to other integer types.
    typedef T* (PassRefPtr::*UnspecifiedBoolType);
    operator UnspecifiedBoolType() const { return m_ptr ? &PassRefPtr::m_ptr : nullptr; }

    friend PassRefPtr adoptRef<T>(T*);

private:
    PassRefPtr& operator=(const PassRefPtr&) = delete;

    enum AdoptTag { Adopt };
    PassRefPtr(T* ptr, AdoptTag) : m_ptr(ptr) { }

    mutable T* m_ptr;
};

template<typename T> template<typename U> inline PassRefPtr<T>::PassRefPtr(const RefPtr<U>& o)
    : m_ptr(o.get())
{
    T* ptr = m_ptr;
    refIfNotNull(ptr);
}

template<typename T> inline T* PassRefPtr<T>::leakRef() const
{
    T* ptr = m_ptr;
    m_ptr = nullptr;
    return ptr;
}

template<typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, const PassRefPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, const RefPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const RefPtr<T>& a, const PassRefPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, U* b)
{
    return a.get() == b;
}

template<typename T, typename U> inline bool operator==(T* a, const PassRefPtr<U>& b)
{
    return a == b.get();
}

template<typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, const PassRefPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, const RefPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const RefPtr<T>& a, const PassRefPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, U* b)
{
    return a.get() != b;
}

template<typename T, typename U> inline bool operator!=(T* a, const PassRefPtr<U>& b)
{
    return a != b.get();
}

template<typename T> inline PassRefPtr<T> adoptRef(T* p)
{
    adopted(p);
    return PassRefPtr<T>(p, PassRefPtr<T>::Adopt);
}

template<typename T, typename U> inline PassRefPtr<T> static_pointer_cast(const PassRefPtr<U>& p)
{
    return adoptRef(static_cast<T*>(p.leakRef()));
}

template<typename T> inline T* getPtr(const PassRefPtr<T>& p)
{
    return p.get();
}

// Unlike most of our smart pointers, PassOwnPtr can take either the pointer type or the pointed-to type.

template<typename T> class OwnPtr;
template<typename T> class PassOwnPtr;
template<typename T> PassOwnPtr<T> adoptPtr(T*);

template<typename T> class PassOwnPtr {
public:
    typedef typename RemovePointer<T>::Type ValueType;
    typedef ValueType* PtrType;

    PassOwnPtr() : m_ptr(0) { }
    PassOwnPtr(std::nullptr_t) : m_ptr(0) { }

    // It somewhat breaks the type system to allow transfer of ownership out of
    // a const PassOwnPtr. However, it makes it much easier to work with PassOwnPtr
    // temporaries, and we don't have a need to use real const PassOwnPtrs anyway.
    PassOwnPtr(const PassOwnPtr& o) : m_ptr(o.leakPtr()) { }
    template<typename U> PassOwnPtr(const PassOwnPtr<U>& o) : m_ptr(o.leakPtr()) { }

    ~PassOwnPtr() { deleteOwnedPtr(m_ptr); }

    PtrType get() const { return m_ptr; }

    PtrType leakPtr() const WARN_UNUSED_RETURN;

    ValueType& operator*() const { ASSERT(m_ptr); return *m_ptr; }
    PtrType operator->() const { ASSERT(m_ptr); return m_ptr; }

    bool operator!() const { return !m_ptr; }

    // This conversion operator allows implicit conversion to bool but not to other integer types.
    typedef PtrType PassOwnPtr::*UnspecifiedBoolType;
    operator UnspecifiedBoolType() const { return m_ptr ? &PassOwnPtr::m_ptr : 0; }

    PassOwnPtr& operator=(const PassOwnPtr&) { COMPILE_ASSERT(!sizeof(T*), PassOwnPtr_should_never_be_assigned_to); return *this; }

    template<typename U> friend PassOwnPtr<U> adoptPtr(U*);

private:
    explicit PassOwnPtr(PtrType ptr) : m_ptr(ptr) { }

    // We should never have two OwnPtrs for the same underlying object (otherwise we'll get
    // double-destruction), so these equality operators should never be needed.
    template<typename U> bool operator==(const PassOwnPtr<U>&) { COMPILE_ASSERT(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    template<typename U> bool operator!=(const PassOwnPtr<U>&) { COMPILE_ASSERT(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    template<typename U> bool operator==(const OwnPtr<U>&) { COMPILE_ASSERT(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    template<typename U> bool operator!=(const OwnPtr<U>&) { COMPILE_ASSERT(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }

    mutable PtrType m_ptr;
};

template<typename T> inline typename PassOwnPtr<T>::PtrType PassOwnPtr<T>::leakPtr() const
{
    PtrType ptr = m_ptr;
    m_ptr = 0;
    return ptr;
}

template<typename T, typename U> inline bool operator==(const PassOwnPtr<T>& a, const PassOwnPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const PassOwnPtr<T>& a, const OwnPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const OwnPtr<T>& a, const PassOwnPtr<U>& b)
{
    return a.get() == b.get();
}

template<typename T, typename U> inline bool operator==(const PassOwnPtr<T>& a, U* b)
{
    return a.get() == b;
}

template<typename T, typename U> inline bool operator==(T* a, const PassOwnPtr<U>& b)
{
    return a == b.get();
}

template<typename T, typename U> inline bool operator!=(const PassOwnPtr<T>& a, const PassOwnPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const PassOwnPtr<T>& a, const OwnPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const OwnPtr<T>& a, const PassOwnPtr<U>& b)
{
    return a.get() != b.get();
}

template<typename T, typename U> inline bool operator!=(const PassOwnPtr<T>& a, U* b)
{
    return a.get() != b;
}

template<typename T, typename U> inline bool operator!=(T* a, const PassOwnPtr<U>& b)
{
    return a != b.get();
}

template<typename T> inline PassOwnPtr<T> adoptPtr(T* ptr)
{
    return PassOwnPtr<T>(ptr);
}

template<typename T, typename U> inline PassOwnPtr<T> static_pointer_cast(const PassOwnPtr<U>& p)
{
    return adoptPtr(static_cast<T*>(p.leakPtr()));
}

template<typename T, typename U> inline PassOwnPtr<T> const_pointer_cast(const PassOwnPtr<U>& p)
{
    return adoptPtr(const_cast<T*>(p.leakPtr()));
}

template<typename T> inline T* getPtr(const PassOwnPtr<T>& p)
{
    return p.get();
}

template <typename T> inline void deleteOwnedPtr(T* ptr)
{
    typedef char known[sizeof(T) ? 1 : -1];
    if (sizeof(known))
        delete ptr;
}

template<typename T> class PassOwnPtr;
template<typename T> PassOwnPtr<T> adoptPtr(T*);

template<typename T> class OwnPtr {
public:
    typedef T ValueType;
    typedef ValueType* PtrType;

    OwnPtr() : m_ptr(0) { }
    OwnPtr(std::nullptr_t) : m_ptr(0) { }

    // See comment in PassOwnPtr.h for why this takes a const reference.
    template<typename U> OwnPtr(const PassOwnPtr<U>& o);

    ~OwnPtr() { deleteOwnedPtr(m_ptr); }

    PtrType get() const { return m_ptr; }

    void clear();
    PassOwnPtr<T> release();
    PtrType leakPtr() WARN_UNUSED_RETURN;

    ValueType& operator*() const { ASSERT(m_ptr); return *m_ptr; }
    PtrType operator->() const { ASSERT(m_ptr); return m_ptr; }

    bool operator!() const { return !m_ptr; }

    // This conversion operator allows implicit conversion to bool but not to other integer types.
    typedef PtrType OwnPtr::*UnspecifiedBoolType;
    operator UnspecifiedBoolType() const { return m_ptr ? &OwnPtr::m_ptr : 0; }

    OwnPtr& operator=(const PassOwnPtr<T>&);
    OwnPtr& operator=(std::nullptr_t) { clear(); return *this; }
    template<typename U> OwnPtr& operator=(const PassOwnPtr<U>&);

    OwnPtr(OwnPtr&&);
    template<typename U> OwnPtr(OwnPtr<U>&&);

    OwnPtr& operator=(OwnPtr&&);
    template<typename U> OwnPtr& operator=(OwnPtr<U>&&);

    void swap(OwnPtr& o) { std::swap(m_ptr, o.m_ptr); }

private:
    explicit OwnPtr(PtrType ptr) : m_ptr(ptr) { }

    // We should never have two OwnPtrs for the same underlying object (otherwise we'll get
    // double-destruction), so these equality operators should never be needed.
    //template<typename U> bool operator==(const OwnPtr<U>&) { static_assert(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    //template<typename U> bool operator!=(const OwnPtr<U>&) { static_assert(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    //template<typename U> bool operator==(const PassOwnPtr<U>&) { static_assert(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }
    //template<typename U> bool operator!=(const PassOwnPtr<U>&) { static_assert(!sizeof(U*), OwnPtrs_should_never_be_equal); return false; }

    PtrType m_ptr;
};

template<typename T> template<typename U> inline OwnPtr<T>::OwnPtr(const PassOwnPtr<U>& o)
    : m_ptr(o.leakPtr())
{
}

template<typename T> inline void OwnPtr<T>::clear()
{
    PtrType ptr = m_ptr;
    m_ptr = 0;
    deleteOwnedPtr(ptr);
}

template<typename T> inline PassOwnPtr<T> OwnPtr<T>::release()
{
    PtrType ptr = m_ptr;
    m_ptr = 0;
    return adoptPtr(ptr);
}

template<typename T> inline typename OwnPtr<T>::PtrType OwnPtr<T>::leakPtr()
{
    PtrType ptr = m_ptr;
    m_ptr = 0;
    return ptr;
}

template<typename T> inline OwnPtr<T>& OwnPtr<T>::operator=(const PassOwnPtr<T>& o)
{
    PtrType ptr = m_ptr;
    m_ptr = o.leakPtr();
    ASSERT(!ptr || m_ptr != ptr);
    deleteOwnedPtr(ptr);
    return *this;
}

template<typename T> template<typename U> inline OwnPtr<T>& OwnPtr<T>::operator=(const PassOwnPtr<U>& o)
{
    PtrType ptr = m_ptr;
    m_ptr = o.leakPtr();
    ASSERT(!ptr || m_ptr != ptr);
    deleteOwnedPtr(ptr);
    return *this;
}

template<typename T> inline OwnPtr<T>::OwnPtr(OwnPtr<T>&& o)
    : m_ptr(o.leakPtr())
{
}

template<typename T> template<typename U> inline OwnPtr<T>::OwnPtr(OwnPtr<U>&& o)
    : m_ptr(o.leakPtr())
{
}

template<typename T> inline auto OwnPtr<T>::operator=(OwnPtr&& o) -> OwnPtr&
{
    ASSERT(!o || o != m_ptr);
    OwnPtr ptr = std::move(o);
    swap(ptr);
    return *this;
}

template<typename T> template<typename U> inline auto OwnPtr<T>::operator=(OwnPtr<U>&& o) -> OwnPtr&
{
    ASSERT(!o || o != m_ptr);
    OwnPtr ptr = std::move(o);
    swap(ptr);
    return *this;
}

template<typename T> inline void swap(OwnPtr<T>& a, OwnPtr<T>& b)
{
    a.swap(b);
}

template<typename T, typename U> inline bool operator==(const OwnPtr<T>& a, U* b)
{
    return a.get() == b;
}

template<typename T, typename U> inline bool operator==(T* a, const OwnPtr<U>& b)
{
    return a == b.get();
}

template<typename T, typename U> inline bool operator!=(const OwnPtr<T>& a, U* b)
{
    return a.get() != b;
}

template<typename T, typename U> inline bool operator!=(T* a, const OwnPtr<U>& b)
{
    return a != b.get();
}

template<typename T> inline typename OwnPtr<T>::PtrType getPtr(const OwnPtr<T>& p)
{
    return p.get();
}


template<typename T>
class Ref {
    T &val;
  public:
    Ref(T &val) : val(val) { }
    operator T&() const { return val; }
};

enum HashTableDeletedValueType { HashTableDeletedValue };

 template<typename T> class RefPtr {
     //WTF_MAKE_FAST_ALLOCATED;
 public:
     ALWAYS_INLINE RefPtr() : m_ptr(nullptr) { }
     ALWAYS_INLINE RefPtr(T* ptr) : m_ptr(ptr) { refIfNotNull(ptr); }
     ALWAYS_INLINE RefPtr(const RefPtr& o) : m_ptr(o.m_ptr) { refIfNotNull(m_ptr); }
     template<typename U> RefPtr(const RefPtr<U>& o) : m_ptr(o.get()) { refIfNotNull(m_ptr); }

     ALWAYS_INLINE RefPtr(RefPtr&& o) : m_ptr(o.release().leakRef()) { }
     template<typename U> RefPtr(RefPtr<U>&& o) : m_ptr(o.release().leakRef()) { }

     // See comments in PassRefPtr.h for an explanation of why this takes a const reference.
     template<typename U> RefPtr(const PassRefPtr<U>&);

     template<typename U> RefPtr(PassRef<U>);

     // Hash table deleted values, which are only constructed and never copied or destroyed.
     RefPtr(HashTableDeletedValueType) : m_ptr(hashTableDeletedValue()) { }
     bool isHashTableDeletedValue() const { return m_ptr == hashTableDeletedValue(); }

     ALWAYS_INLINE ~RefPtr() { derefIfNotNull(m_ptr); }

     T* get() const { return m_ptr; }

     void clear();
     PassRefPtr<T> release() { PassRefPtr<T> tmp = adoptRef(m_ptr); m_ptr = nullptr; return tmp; }
     PassRef<T> releaseNonNull() { ASSERT(m_ptr); PassRef<T> tmp = adoptRef(*m_ptr); m_ptr = nullptr; return tmp; }

     T& operator*() const { return *m_ptr; }
     ALWAYS_INLINE T* operator->() const { return m_ptr; }

     bool operator!() const { return !m_ptr; }

     // This conversion operator allows implicit conversion to bool but not to other integer types.
     typedef T* (RefPtr::*UnspecifiedBoolType);
     operator UnspecifiedBoolType() const { return m_ptr ? &RefPtr::m_ptr : nullptr; }

     RefPtr& operator=(const RefPtr&);
     RefPtr& operator=(T*);
     RefPtr& operator=(const PassRefPtr<T>&);
     template<typename U> RefPtr& operator=(const RefPtr<U>&);
     template<typename U> RefPtr& operator=(const PassRefPtr<U>&);
     RefPtr& operator=(RefPtr&&);
     template<typename U> RefPtr& operator=(RefPtr<U>&&);
     template<typename U> RefPtr& operator=(PassRef<U>);

     void swap(RefPtr&);

     static T* hashTableDeletedValue() { return reinterpret_cast<T*>(-1); }

 private:
     T* m_ptr;
 };

 template<typename T> template<typename U> inline RefPtr<T>::RefPtr(const PassRefPtr<U>& o)
     : m_ptr(o.leakRef())
 {
 }

 template<typename T> template<typename U> inline RefPtr<T>::RefPtr(PassRef<U> reference)
     : m_ptr(&reference.leakRef())
 {
 }

 template<typename T> inline void RefPtr<T>::clear()
 {
     T* ptr = m_ptr;
     m_ptr = nullptr;
     derefIfNotNull(ptr);
 }

 template<typename T> inline RefPtr<T>& RefPtr<T>::operator=(const RefPtr& o)
 {
     RefPtr ptr = o;
     swap(ptr);
     return *this;
 }

 template<typename T> template<typename U> inline RefPtr<T>& RefPtr<T>::operator=(const RefPtr<U>& o)
 {
     RefPtr ptr = o;
     swap(ptr);
     return *this;
 }

 template<typename T> inline RefPtr<T>& RefPtr<T>::operator=(T* optr)
 {
     RefPtr ptr = optr;
     swap(ptr);
     return *this;
 }

 template<typename T> inline RefPtr<T>& RefPtr<T>::operator=(const PassRefPtr<T>& o)
 {
     RefPtr ptr = o;
     swap(ptr);
     return *this;
 }

 template<typename T> template<typename U> inline RefPtr<T>& RefPtr<T>::operator=(const PassRefPtr<U>& o)
 {
     RefPtr ptr = o;
     swap(ptr);
     return *this;
 }

 template<typename T> inline RefPtr<T>& RefPtr<T>::operator=(RefPtr&& o)
 {
     RefPtr ptr = std::move(o);
     swap(ptr);
     return *this;
 }

 template<typename T> template<typename U> inline RefPtr<T>& RefPtr<T>::operator=(RefPtr<U>&& o)
 {
     RefPtr ptr = std::move(o);
     swap(ptr);
     return *this;
 }

 template<typename T> template<typename U> inline RefPtr<T>& RefPtr<T>::operator=(PassRef<U> reference)
 {
     RefPtr ptr = std::move(reference);
     swap(ptr);
     return *this;
 }

 template<class T> inline void RefPtr<T>::swap(RefPtr& o)
 {
     std::swap(m_ptr, o.m_ptr);
 }

 template<class T> inline void swap(RefPtr<T>& a, RefPtr<T>& b)
 {
     a.swap(b);
 }

 template<typename T, typename U> inline bool operator==(const RefPtr<T>& a, const RefPtr<U>& b)
 {
     return a.get() == b.get();
 }

 template<typename T, typename U> inline bool operator==(const RefPtr<T>& a, U* b)
 {
     return a.get() == b;
 }

 template<typename T, typename U> inline bool operator==(T* a, const RefPtr<U>& b)
 {
     return a == b.get();
 }

 template<typename T, typename U> inline bool operator!=(const RefPtr<T>& a, const RefPtr<U>& b)
 {
     return a.get() != b.get();
 }

 template<typename T, typename U> inline bool operator!=(const RefPtr<T>& a, U* b)
 {
     return a.get() != b;
 }

 template<typename T, typename U> inline bool operator!=(T* a, const RefPtr<U>& b)
 {
     return a != b.get();
 }

 template<typename T, typename U> inline RefPtr<T> static_pointer_cast(const RefPtr<U>& p)
 {
     return RefPtr<T>(static_cast<T*>(p.get()));
 }

 template<typename T> inline T* getPtr(const RefPtr<T>& p)
 {
     return p.get();
 }


template<typename T, size_t N = 0>
class Vector {
  public:
    std::vector<T> impl;
  public:
    Vector() {}

    Vector(const Vector &v) {
        append(v);
    }

    size_t size() const {
        return impl.size();
    }

    T &operator[](size_t i) {
        return impl[i];
    }

    const T &operator[](size_t i) const {
        return impl[i];
    }

    T &at(size_t i) {
        return impl[i];
    }

    const T *begin() const {
        return impl.begin();
    }

    T &last() {
        return impl.back();
    }

    bool isEmpty() const {
        return impl.empty();
    }

    template <typename U>
    void append(const U &u) {
        impl.push_back(static_cast<T>(u));
    }

    template <size_t M>
    void append(const Vector<T,M> &v) {
        impl.append(v.impl);
    }

    void insert(size_t i, const T& t) {
        impl.insert(impl.begin() + i, t);
    }

    void remove(size_t i) {
        impl.erase(impl.begin() + i);
    }

    void clear() {
        return impl.clear();
    }

    void shrink(size_t newLength) {
        ASSERT(newLength <= impl.size());
        impl.resize(newLength);
    }

    void shrinkToFit()
    {

    }

    void swap(Vector &other) {
        impl.swap(other.impl);
    }

    void deleteAllValues() {
        auto iter = impl.begin();
        while(iter != impl.end()) {
            delete *iter;
            iter++;
        }
        //clear();
    }

    void reserve(size_t capacity) {
        impl.reserve(capacity);
    }
};


template <typename T, size_t N>
inline void
deleteAllValues(Vector<T, N> &v) {
    v.deleteAllValues();
}

static inline void
dataLog(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

 /*
  * Do-nothing version of a macro used by WTF to avoid unused
  * parameter warnings.
  */
#define UNUSED_PARAM(e)

} /* namespace Yarr */

} /* namespace JSC */

namespace WTF {
/*
 * Sentinel value used in Yarr.
 */
const size_t notFound = size_t(-1);

}

#define JS_EXPORT_PRIVATE

#endif /* yarr_wtfbridge_h */
