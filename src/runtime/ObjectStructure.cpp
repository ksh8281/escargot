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
#include "Object.h"
#include "runtime/Context.h"
#include "runtime/VMInstance.h"

namespace Escargot {

#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
namespace {

constexpr size_t ObjectStructureProfileKindCount = 3;
constexpr size_t ObjectStructureProfileExactSizeLimit = 128;
constexpr size_t ObjectStructureProfileSizeBucketCount = 136;

struct ObjectStructureProfileSizeCounters {
    uint64_t creations { 0 };
    uint64_t finds { 0 };
    uint64_t hits { 0 };
    uint64_t misses { 0 };
    uint64_t lastCacheHits { 0 };
    uint64_t comparisons { 0 };
    uint64_t adds { 0 };
    uint64_t removes { 0 };
    uint64_t replaces { 0 };
};

class ObjectStructureProfile {
public:
    ObjectStructureProfile()
        : m_enabled(getenv("ESCARGOT_OBJECT_STRUCTURE_PROFILE") != nullptr)
    {
    }

    ~ObjectStructureProfile()
    {
        if (!m_enabled) {
            return;
        }

        const char* label = getenv("ESCARGOT_OBJECT_STRUCTURE_PROFILE");
        fprintf(stderr, "OSP_BEGIN label=%s\n", label && *label ? label : "unnamed");
        fprintf(stderr,
                "OSP_CONFIG access_cache_min=%d transition_max=%d transition_map_min=%d\n",
                ESCARGOT_OBJECT_STRUCTURE_ACCESS_CACHE_BUILD_MIN_SIZE,
                ESCARGOT_OBJECT_STRUCTURE_TRANSITION_MODE_MAX_SIZE,
                ESCARGOT_OBJECT_STRUCTURE_TRANSITION_MAP_MIN_SIZE);

        for (size_t kind = 0; kind < ObjectStructureProfileKindCount; kind++) {
            uint64_t creations = 0;
            uint64_t finds = 0;
            uint64_t hits = 0;
            uint64_t misses = 0;
            uint64_t cacheHits = 0;
            uint64_t comparisons = 0;
            uint64_t adds = 0;
            uint64_t removes = 0;
            uint64_t replaces = 0;
            for (size_t bucket = 0; bucket < ObjectStructureProfileSizeBucketCount; bucket++) {
                const auto& counter = m_size[kind][bucket];
                creations += counter.creations;
                finds += counter.finds;
                hits += counter.hits;
                misses += counter.misses;
                cacheHits += counter.lastCacheHits;
                comparisons += counter.comparisons;
                adds += counter.adds;
                removes += counter.removes;
                replaces += counter.replaces;
            }
            fprintf(stderr,
                    "OSP_KIND kind=%s creations=%llu finds=%llu hits=%llu misses=%llu cache_hits=%llu comparisons=%llu adds=%llu removes=%llu replaces=%llu atomic_names=%llu non_atomic_names=%llu symbol_names=%llu\n",
                    kindName(kind),
                    static_cast<unsigned long long>(creations),
                    static_cast<unsigned long long>(finds),
                    static_cast<unsigned long long>(hits),
                    static_cast<unsigned long long>(misses),
                    static_cast<unsigned long long>(cacheHits),
                    static_cast<unsigned long long>(comparisons),
                    static_cast<unsigned long long>(adds),
                    static_cast<unsigned long long>(removes),
                    static_cast<unsigned long long>(replaces),
                    static_cast<unsigned long long>(m_nameKinds[kind][0]),
                    static_cast<unsigned long long>(m_nameKinds[kind][1]),
                    static_cast<unsigned long long>(m_nameKinds[kind][2]));

            for (size_t bucket = 0; bucket < ObjectStructureProfileSizeBucketCount; bucket++) {
                const auto& counter = m_size[kind][bucket];
                if (!(counter.creations || counter.finds || counter.adds || counter.removes || counter.replaces)) {
                    continue;
                }
                char sizeLabel[32];
                formatSizeBucket(bucket, sizeLabel, sizeof(sizeLabel));
                fprintf(stderr,
                        "OSP_SIZE kind=%s properties=%s creations=%llu finds=%llu hits=%llu misses=%llu cache_hits=%llu comparisons=%llu adds=%llu removes=%llu replaces=%llu\n",
                        kindName(kind), sizeLabel,
                        static_cast<unsigned long long>(counter.creations),
                        static_cast<unsigned long long>(counter.finds),
                        static_cast<unsigned long long>(counter.hits),
                        static_cast<unsigned long long>(counter.misses),
                        static_cast<unsigned long long>(counter.lastCacheHits),
                        static_cast<unsigned long long>(counter.comparisons),
                        static_cast<unsigned long long>(counter.adds),
                        static_cast<unsigned long long>(counter.removes),
                        static_cast<unsigned long long>(counter.replaces));
            }
        }

        fprintf(stderr,
                "OSP_TRANSITION vector_lookups=%llu vector_comparisons=%llu vector_reuses=%llu map_lookups=%llu map_reuses=%llu vector_to_map=%llu exits=%llu\n",
                static_cast<unsigned long long>(m_transitionVectorLookups),
                static_cast<unsigned long long>(m_transitionVectorComparisons),
                static_cast<unsigned long long>(m_transitionVectorReuses),
                static_cast<unsigned long long>(m_transitionMapLookups),
                static_cast<unsigned long long>(m_transitionMapReuses),
                static_cast<unsigned long long>(m_transitionVectorToMap),
                static_cast<unsigned long long>(m_transitionExits));
        fprintf(stderr,
                "OSP_MAP builds=%llu rebuilds=%llu lazy_builds=%llu inserts=%llu allocated_bytes=%llu max_allocation=%llu dense_builds=%llu non_atomic_builds=%llu hash_calls=%llu content_hash_calls=%llu index_conversions=%llu insert_probes=%llu max_insert_probe=%llu find_last_cache=%llu find_dense=%llu dense_hits=%llu dense_misses=%llu find_hash=%llu hash_hits=%llu hash_misses=%llu find_probes=%llu max_find_probe=%llu linear_fallbacks=%llu fallback_comparisons=%llu\n",
                static_cast<unsigned long long>(m_mapBuilds),
                static_cast<unsigned long long>(m_mapRebuilds),
                static_cast<unsigned long long>(m_mapLazyBuilds),
                static_cast<unsigned long long>(m_mapInserts),
                static_cast<unsigned long long>(m_mapAllocatedBytes),
                static_cast<unsigned long long>(m_mapMaxAllocation),
                static_cast<unsigned long long>(m_mapDenseBuilds),
                static_cast<unsigned long long>(m_mapNonAtomicBuilds),
                static_cast<unsigned long long>(m_mapHashCalls),
                static_cast<unsigned long long>(m_mapContentHashCalls),
                static_cast<unsigned long long>(m_mapIndexConversions),
                static_cast<unsigned long long>(m_mapInsertProbes),
                static_cast<unsigned long long>(m_mapMaxInsertProbe),
                static_cast<unsigned long long>(m_mapLastCacheHits),
                static_cast<unsigned long long>(m_mapDenseFinds),
                static_cast<unsigned long long>(m_mapDenseHits),
                static_cast<unsigned long long>(m_mapDenseMisses),
                static_cast<unsigned long long>(m_mapHashFinds),
                static_cast<unsigned long long>(m_mapHashHits),
                static_cast<unsigned long long>(m_mapHashMisses),
                static_cast<unsigned long long>(m_mapFindProbes),
                static_cast<unsigned long long>(m_mapMaxFindProbe),
                static_cast<unsigned long long>(m_mapLinearFallbacks),
                static_cast<unsigned long long>(m_mapFallbackComparisons));
        for (size_t bucket = 0; bucket < ObjectStructureProfileSizeBucketCount; bucket++) {
            if (!(m_mapBuildsBySize[bucket] || m_mapFindsBySize[bucket])) {
                continue;
            }
            char sizeLabel[32];
            formatSizeBucket(bucket, sizeLabel, sizeof(sizeLabel));
            fprintf(stderr,
                    "OSP_MAP_SIZE properties=%s builds=%llu finds=%llu allocated_bytes=%llu\n",
                    sizeLabel,
                    static_cast<unsigned long long>(m_mapBuildsBySize[bucket]),
                    static_cast<unsigned long long>(m_mapFindsBySize[bucket]),
                    static_cast<unsigned long long>(m_mapBytesBySize[bucket]));
        }
        fprintf(stderr, "OSP_END\n");
    }

    bool enabled() const { return m_enabled; }

    void recordCreation(ObjectStructureProfileKind kind, size_t size)
    {
        m_size[kindIndex(kind)][sizeBucket(size)].creations++;
    }

    void recordFind(ObjectStructureProfileKind kind, size_t size,
                    const ObjectStructurePropertyName& name, bool hit,
                    bool lastCacheHit, size_t comparisons)
    {
        size_t k = kindIndex(kind);
        auto& counter = m_size[k][sizeBucket(size)];
        counter.finds++;
        counter.hits += hit;
        counter.misses += !hit;
        counter.lastCacheHits += lastCacheHit;
        counter.comparisons += comparisons;
        if (name.hasAtomicString()) {
            m_nameKinds[k][0]++;
        } else if (name.isSymbol()) {
            m_nameKinds[k][2]++;
        } else {
            m_nameKinds[k][1]++;
        }
        if (kind == ObjectStructureProfileKind::WithMap) {
            m_mapFindsBySize[sizeBucket(size)]++;
        }
    }

    void recordOperation(ObjectStructureProfileKind kind, size_t size, char operation)
    {
        auto& counter = m_size[kindIndex(kind)][sizeBucket(size)];
        if (operation == 'a') {
            counter.adds++;
        } else if (operation == 'r') {
            counter.removes++;
        } else {
            counter.replaces++;
        }
    }

    void recordTransitionLookup(bool map, bool reused, size_t comparisons)
    {
        if (map) {
            m_transitionMapLookups++;
            m_transitionMapReuses += reused;
        } else {
            m_transitionVectorLookups++;
            m_transitionVectorComparisons += comparisons;
            m_transitionVectorReuses += reused;
        }
    }

    void recordTransitionVectorToMap() { m_transitionVectorToMap++; }
    void recordTransitionExit() { m_transitionExits++; }

    void recordMapBuild(size_t size, size_t bytes, bool rebuild,
                        bool dense, bool nonAtomic)
    {
        m_mapBuilds++;
        m_mapRebuilds += rebuild;
        m_mapAllocatedBytes += bytes;
        m_mapMaxAllocation = std::max(m_mapMaxAllocation, static_cast<uint64_t>(bytes));
        m_mapDenseBuilds += dense;
        m_mapNonAtomicBuilds += nonAtomic;
        size_t bucket = sizeBucket(size);
        m_mapBuildsBySize[bucket]++;
        m_mapBytesBySize[bucket] += bytes;
    }

    void recordMapLazyBuild() { m_mapLazyBuilds++; }
    void recordMapInsert() { m_mapInserts++; }
    void recordMapHash(bool content)
    {
        m_mapHashCalls++;
        m_mapContentHashCalls += content;
    }
    void recordMapIndexConversion() { m_mapIndexConversions++; }
    void recordMapInsertProbes(size_t probes)
    {
        m_mapInsertProbes += probes;
        m_mapMaxInsertProbe = std::max(m_mapMaxInsertProbe, static_cast<uint64_t>(probes));
    }
    void recordMapLastCache() { m_mapLastCacheHits++; }
    void recordMapDenseFind(bool hit)
    {
        m_mapDenseFinds++;
        m_mapDenseHits += hit;
        m_mapDenseMisses += !hit;
    }
    void recordMapHashFind(bool hit, size_t probes)
    {
        m_mapHashFinds++;
        m_mapHashHits += hit;
        m_mapHashMisses += !hit;
        m_mapFindProbes += probes;
        m_mapMaxFindProbe = std::max(m_mapMaxFindProbe, static_cast<uint64_t>(probes));
    }
    void recordMapLinearFallback(size_t comparisons)
    {
        m_mapLinearFallbacks++;
        m_mapFallbackComparisons += comparisons;
    }

private:
    static size_t kindIndex(ObjectStructureProfileKind kind)
    {
        return static_cast<size_t>(kind);
    }

    static const char* kindName(size_t kind)
    {
        static const char* names[] = { "without-transition", "with-transition", "with-map" };
        return names[kind];
    }

    static size_t sizeBucket(size_t size)
    {
        if (size <= ObjectStructureProfileExactSizeLimit) {
            return size;
        }
        if (size <= 255) return 129;
        if (size <= 511) return 130;
        if (size <= 1023) return 131;
        if (size <= 4095) return 132;
        if (size <= 16383) return 133;
        if (size <= 65535) return 134;
        return 135;
    }

    static void formatSizeBucket(size_t bucket, char* output, size_t outputSize)
    {
        if (bucket <= ObjectStructureProfileExactSizeLimit) {
            snprintf(output, outputSize, "%zu", bucket);
            return;
        }
        static const char* ranges[] = {
            "129-255", "256-511", "512-1023", "1024-4095",
            "4096-16383", "16384-65535", "65536+"
        };
        snprintf(output, outputSize, "%s", ranges[bucket - 129]);
    }

    bool m_enabled;
    ObjectStructureProfileSizeCounters m_size[ObjectStructureProfileKindCount][ObjectStructureProfileSizeBucketCount] {};
    uint64_t m_nameKinds[ObjectStructureProfileKindCount][3] {};
    uint64_t m_transitionVectorLookups { 0 };
    uint64_t m_transitionVectorComparisons { 0 };
    uint64_t m_transitionVectorReuses { 0 };
    uint64_t m_transitionMapLookups { 0 };
    uint64_t m_transitionMapReuses { 0 };
    uint64_t m_transitionVectorToMap { 0 };
    uint64_t m_transitionExits { 0 };
    uint64_t m_mapBuilds { 0 };
    uint64_t m_mapRebuilds { 0 };
    uint64_t m_mapLazyBuilds { 0 };
    uint64_t m_mapInserts { 0 };
    uint64_t m_mapAllocatedBytes { 0 };
    uint64_t m_mapMaxAllocation { 0 };
    uint64_t m_mapDenseBuilds { 0 };
    uint64_t m_mapNonAtomicBuilds { 0 };
    uint64_t m_mapHashCalls { 0 };
    uint64_t m_mapContentHashCalls { 0 };
    uint64_t m_mapIndexConversions { 0 };
    uint64_t m_mapInsertProbes { 0 };
    uint64_t m_mapMaxInsertProbe { 0 };
    uint64_t m_mapLastCacheHits { 0 };
    uint64_t m_mapDenseFinds { 0 };
    uint64_t m_mapDenseHits { 0 };
    uint64_t m_mapDenseMisses { 0 };
    uint64_t m_mapHashFinds { 0 };
    uint64_t m_mapHashHits { 0 };
    uint64_t m_mapHashMisses { 0 };
    uint64_t m_mapFindProbes { 0 };
    uint64_t m_mapMaxFindProbe { 0 };
    uint64_t m_mapLinearFallbacks { 0 };
    uint64_t m_mapFallbackComparisons { 0 };
    uint64_t m_mapBuildsBySize[ObjectStructureProfileSizeBucketCount] {};
    uint64_t m_mapFindsBySize[ObjectStructureProfileSizeBucketCount] {};
    uint64_t m_mapBytesBySize[ObjectStructureProfileSizeBucketCount] {};
};

ObjectStructureProfile& objectStructureProfile()
{
    static ObjectStructureProfile profile;
    return profile;
}

inline bool objectStructureProfileEnabled()
{
    return objectStructureProfile().enabled();
}

} // namespace

void recordObjectStructureProfileCreation(ObjectStructureProfileKind kind, size_t propertyCount)
{
    if (objectStructureProfileEnabled()) {
        objectStructureProfile().recordCreation(kind, propertyCount);
    }
}

#define OBJECT_STRUCTURE_PROFILE(code)        \
    do {                                      \
        if (objectStructureProfileEnabled()) { \
            objectStructureProfile().code;   \
        }                                     \
    } while (0)
#define OBJECT_STRUCTURE_PROFILE_VALUE(code) do { code; } while (0)
#else
#define OBJECT_STRUCTURE_PROFILE(code) do { } while (0)
#define OBJECT_STRUCTURE_PROFILE_VALUE(code) do { } while (0)
#endif

void* ObjectStructureItemVector::operator new(size_t size)
{
    static MAY_THREAD_LOCAL bool typeInited = false;
    static MAY_THREAD_LOCAL GC_descr descr;
    if (!typeInited) {
        GC_word obj_bitmap[GC_BITMAP_SIZE(ObjectStructureItemVector)] = { 0 };
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureItemVector, m_buffer));
        descr = GC_make_descriptor(obj_bitmap, GC_WORD_LEN(ObjectStructureItemVector));
        typeInited = true;
    }
    return GC_MALLOC_EXPLICITLY_TYPED(size, descr);
}

void* ObjectStructureWithoutTransition::operator new(size_t size)
{
    static MAY_THREAD_LOCAL bool typeInited = false;
    static MAY_THREAD_LOCAL GC_descr descr;
    if (!typeInited) {
        GC_word obj_bitmap[GC_BITMAP_SIZE(ObjectStructureWithoutTransition)] = { 0 };
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithoutTransition, m_properties));
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithoutTransition, m_lastFoundPropertyName));
        descr = GC_make_descriptor(obj_bitmap, GC_WORD_LEN(ObjectStructureWithoutTransition));
        typeInited = true;
    }
    return GC_MALLOC_EXPLICITLY_TYPED(size, descr);
}

ObjectStructure* ObjectStructure::create(Context* ctx, ObjectStructureItemTightVector&& properties, bool preferTransition)
{
    bool hasIndexStringAsPropertyName = false;
    bool hasSymbol = false;
    bool hasNonAtomicPropertyName = false;
    bool hasEnumerableProperty = true;

    for (size_t i = 0; i < properties.size(); i++) {
        const ObjectStructurePropertyName& propertyName = properties[i].m_propertyName;
#ifndef NDEBUG
        // there should be no duplicated properties
        for (size_t j = i + 1; j < properties.size(); j++) {
            ASSERT(propertyName != properties[j].m_propertyName);
        }
#endif
        if (propertyName.isSymbol()) {
            hasSymbol = true;
        }
        if (!hasIndexStringAsPropertyName) {
            hasIndexStringAsPropertyName |= propertyName.isIndexString();
        }

        hasNonAtomicPropertyName |= !propertyName.hasAtomicString();
        hasEnumerableProperty |= properties[i].m_descriptor.isEnumerable();
    }

    if (!isTransitionModeAvailable(properties.size())) {
        return new ObjectStructureWithMap(hasIndexStringAsPropertyName, hasSymbol, hasEnumerableProperty, std::move(properties));
    } else if (preferTransition) {
        return new ObjectStructureWithTransition(std::move(properties), hasIndexStringAsPropertyName, hasSymbol, hasNonAtomicPropertyName, hasEnumerableProperty);
    } else {
        return new ObjectStructureWithoutTransition(new ObjectStructureItemVector(std::move(properties)), hasIndexStringAsPropertyName, hasSymbol, hasNonAtomicPropertyName, hasEnumerableProperty);
    }
}

std::pair<size_t, Optional<const ObjectStructureItem*>> ObjectStructureWithoutTransition::findProperty(const ObjectStructurePropertyName& s)
{
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
    size_t profileComparisons = 0;
#endif
    size_t size = m_properties->size();
    OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
    if (m_properties->size() && m_lastFoundPropertyName == s) {
        uint16_t lastIndex = lastFoundPropertyIndex();
        if (lastIndex == std::numeric_limits<uint16_t>::max()) {
            OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, false, true, profileComparisons));
            return std::make_pair(std::numeric_limits<size_t>::max(), Optional<const ObjectStructureItem*>());
        }
        OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, true, true, profileComparisons));
        return std::make_pair(lastIndex, &(*m_properties)[lastIndex]);
    }
    m_lastFoundPropertyName = s;
    setLastFoundPropertyIndex(std::numeric_limits<uint16_t>::max());

    if (LIKELY(s.hasAtomicString())) {
        if (LIKELY(!m_hasNonAtomicPropertyName)) {
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if ((*m_properties)[i].m_propertyName.rawValue() == s.rawValue()) {
                    setLastFoundPropertyIndex(i);
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &(*m_properties)[i]);
                }
            }
        } else {
            AtomicString as = s.asAtomicString();
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if ((*m_properties)[i].m_propertyName == as) {
                    setLastFoundPropertyIndex(i);
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &(*m_properties)[i]);
                }
            }
        }
    } else if (s.isSymbol()) {
        if (m_hasSymbolPropertyName) {
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if ((*m_properties)[i].m_propertyName == s) {
                    setLastFoundPropertyIndex(i);
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &(*m_properties)[i]);
                }
            }
        }
    } else {
        for (size_t i = 0; i < size; i++) {
            OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
            if ((*m_properties)[i].m_propertyName == s) {
                setLastFoundPropertyIndex(i);
                OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, true, false, profileComparisons));
                return std::make_pair(i, &(*m_properties)[i]);
            }
        }
    }

    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithoutTransition, size, s, false, false, profileComparisons));
    return std::make_pair(SIZE_MAX, Optional<const ObjectStructureItem*>());
}

const ObjectStructureItem& ObjectStructureWithoutTransition::readProperty(size_t idx)
{
    return m_properties->at(idx);
}

const ObjectStructureItem* ObjectStructureWithoutTransition::properties() const
{
    return m_properties->data();
}

size_t ObjectStructureWithoutTransition::propertyCount() const
{
    return m_properties->size();
}

ObjectStructure* ObjectStructureWithoutTransition::addProperty(const ObjectStructurePropertyName& name, const ObjectStructurePropertyDescriptor& desc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithoutTransition, m_properties->size(), 'a'));
    ObjectStructureItem newItem(name, desc);
    bool nameIsIndexString = m_hasIndexPropertyName ? true : name.isIndexString();
    bool nameIsSymbol = m_hasSymbolPropertyName ? true : name.isSymbol();
    bool hasNonAtomicName = m_hasNonAtomicPropertyName ? true : !name.hasAtomicString();
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : desc.isEnumerable();

    ObjectStructure* newStructure;
    ObjectStructureItemVector* propertiesForNewStructure;
    if (m_isReferencedByInlineCache) {
        propertiesForNewStructure = new ObjectStructureItemVector(*m_properties, newItem);
    } else {
        m_properties->push_back(newItem);
        propertiesForNewStructure = m_properties;
        m_properties = nullptr;
    }

    if (propertiesForNewStructure->size() > ESCARGOT_OBJECT_STRUCTURE_ACCESS_CACHE_BUILD_MIN_SIZE) {
        newStructure = new ObjectStructureWithMap(propertiesForNewStructure, nullptr, nameIsIndexString, nameIsSymbol, hasEnumerableProperty);
    } else {
        newStructure = new ObjectStructureWithoutTransition(propertiesForNewStructure, nameIsIndexString, nameIsSymbol, hasNonAtomicName, hasEnumerableProperty);
    }

    return newStructure;
}

ObjectStructure* ObjectStructureWithoutTransition::removeProperty(size_t pIndex)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithoutTransition, m_properties->size(), 'r'));
    ObjectStructureItemVector* newProperties = new ObjectStructureItemVector();
    size_t ps = m_properties->size();
    newProperties->resizeFitWithUninitializedValues(ps - 1);

    size_t newIdx = 0;
    bool hasIndexString = false;
    bool hasSymbol = false;
    bool hasNonAtomicName = false;
    bool hasEnumerableProperty = false;
    for (size_t i = 0; i < ps; i++) {
        if (i == pIndex)
            continue;
        hasIndexString = hasIndexString | (*m_properties)[i].m_propertyName.isIndexString();
        hasSymbol = hasSymbol | (*m_properties)[i].m_propertyName.isSymbol();
        hasNonAtomicName = hasNonAtomicName | !(*m_properties)[i].m_propertyName.hasAtomicString();
        hasEnumerableProperty = hasEnumerableProperty | (*m_properties)[i].m_descriptor.isEnumerable();
        (*newProperties)[newIdx].m_propertyName = (*m_properties)[i].m_propertyName;
        (*newProperties)[newIdx].m_descriptor = (*m_properties)[i].m_descriptor;
        newIdx++;
    }

    auto newStructure = new ObjectStructureWithoutTransition(newProperties, hasIndexString, hasSymbol, hasNonAtomicName, hasEnumerableProperty);
    if (!m_isReferencedByInlineCache) {
        m_properties = nullptr;
    }
    return newStructure;
}

ObjectStructure* ObjectStructureWithoutTransition::replacePropertyDescriptor(size_t idx, const ObjectStructurePropertyDescriptor& newDesc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithoutTransition, m_properties->size(), 'p'));
    ObjectStructureItemVector* newProperties = m_properties;

    if (m_isReferencedByInlineCache) {
        newProperties = new ObjectStructureItemVector(*m_properties);
    } else {
        m_properties = nullptr;
    }
    newProperties->at(idx).m_descriptor = newDesc;
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : newDesc.isEnumerable();
    return new ObjectStructureWithoutTransition(newProperties, m_hasIndexPropertyName, m_hasSymbolPropertyName, m_hasNonAtomicPropertyName, hasEnumerableProperty);
}

void* ObjectStructureWithTransition::operator new(size_t size)
{
    static MAY_THREAD_LOCAL bool typeInited = false;
    static MAY_THREAD_LOCAL GC_descr descr;
    if (!typeInited) {
        GC_word obj_bitmap[GC_BITMAP_SIZE(ObjectStructureWithTransition)] = { 0 };
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithTransition, m_properties));
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithTransition, m_transitionTableVectorBuffer));
        descr = GC_make_descriptor(obj_bitmap, GC_WORD_LEN(ObjectStructureWithTransition));
        typeInited = true;
    }
    return GC_MALLOC_EXPLICITLY_TYPED(size, descr);
}

std::pair<size_t, Optional<const ObjectStructureItem*>> ObjectStructureWithTransition::findProperty(const ObjectStructurePropertyName& s)
{
    size_t size = m_properties.size();
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
    size_t profileComparisons = 0;
#endif

    if (LIKELY(s.hasAtomicString())) {
        if (LIKELY(!m_hasNonAtomicPropertyName)) {
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if (m_properties[i].m_propertyName.rawValue() == s.rawValue()) {
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &m_properties[i]);
                }
            }
        } else {
            AtomicString as = s.asAtomicString();
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if (m_properties[i].m_propertyName == as) {
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &m_properties[i]);
                }
            }
        }
    } else if (s.isSymbol()) {
        if (m_hasSymbolPropertyName) {
            for (size_t i = 0; i < size; i++) {
                OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
                if (m_properties[i].m_propertyName == s) {
                    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithTransition, size, s, true, false, profileComparisons));
                    return std::make_pair(i, &m_properties[i]);
                }
            }
        }
    } else {
        for (size_t i = 0; i < size; i++) {
            OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
            if (m_properties[i].m_propertyName == s) {
                OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithTransition, size, s, true, false, profileComparisons));
                return std::make_pair(i, &m_properties[i]);
            }
        }
    }

    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithTransition, size, s, false, false, profileComparisons));
    return std::make_pair(SIZE_MAX, Optional<const ObjectStructureItem*>());
}

const ObjectStructureItem& ObjectStructureWithTransition::readProperty(size_t idx)
{
    return m_properties[idx];
}

const ObjectStructureItem* ObjectStructureWithTransition::properties() const
{
    return m_properties.data();
}

size_t ObjectStructureWithTransition::propertyCount() const
{
    return m_properties.size();
}

ObjectStructure* ObjectStructureWithTransition::addProperty(const ObjectStructurePropertyName& name, const ObjectStructurePropertyDescriptor& desc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithTransition, m_properties.size(), 'a'));
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
    size_t profileTransitionComparisons = 0;
#endif
    if (m_doesTransitionTableUseMap) {
        auto iter = m_transitionTableMap->find(ObjectStructureTransitionMapItem(name, desc));
        if (iter != m_transitionTableMap->end()) {
            OBJECT_STRUCTURE_PROFILE(recordTransitionLookup(true, true, 0));
            return iter->second;
        }
        OBJECT_STRUCTURE_PROFILE(recordTransitionLookup(true, false, 0));
    } else {
        size_t len = m_transitionTableVectorBufferSize;
        for (size_t i = 0; i < len; i++) {
            OBJECT_STRUCTURE_PROFILE_VALUE(profileTransitionComparisons++);
            const auto& item = m_transitionTableVectorBuffer[i];
            if (item.m_descriptor == desc && item.m_propertyName == name) {
                OBJECT_STRUCTURE_PROFILE(recordTransitionLookup(false, true, profileTransitionComparisons));
                return item.m_structure;
            }
        }
        OBJECT_STRUCTURE_PROFILE(recordTransitionLookup(false, false, profileTransitionComparisons));
    }

    ObjectStructureItem newItem(name, desc);
    bool nameIsIndexString = m_hasIndexPropertyName ? true : name.isIndexString();
    bool hasSymbol = m_hasSymbolPropertyName ? true : name.isSymbol();
    bool hasNonAtomicName = m_hasNonAtomicPropertyName ? true : !name.hasAtomicString();
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : desc.isEnumerable();
    ObjectStructure* newObjectStructure;

    size_t nextSize = m_properties.size() + 1;
    // ObjectStructureWithTransition cannot directly convert to ObjectStructureWithMap by just adding one property
    ASSERT(nextSize < ESCARGOT_OBJECT_STRUCTURE_ACCESS_CACHE_BUILD_MIN_SIZE);
    if (nextSize > ESCARGOT_OBJECT_STRUCTURE_TRANSITION_MODE_MAX_SIZE || nameIsIndexString) {
        OBJECT_STRUCTURE_PROFILE(recordTransitionExit());
        ObjectStructureItemVector* newProperties = new ObjectStructureItemVector(m_properties, newItem);
        newObjectStructure = new ObjectStructureWithoutTransition(newProperties, nameIsIndexString, hasSymbol, hasNonAtomicName, hasEnumerableProperty);
    } else {
        ObjectStructureItemTightVector newProperties(m_properties, newItem);
        newObjectStructure = new ObjectStructureWithTransition(std::move(newProperties), nameIsIndexString, hasSymbol, hasNonAtomicName, hasEnumerableProperty);
        ObjectStructureTransitionVectorItem newTransitionItem(name, desc, newObjectStructure);

        if (m_doesTransitionTableUseMap) {
            m_transitionTableMap->insert(std::make_pair(ObjectStructureTransitionMapItem(newTransitionItem.m_propertyName, newTransitionItem.m_descriptor),
                                                        newTransitionItem.m_structure));
        } else {
            if (m_transitionTableVectorBufferSize + 1 > ESCARGOT_OBJECT_STRUCTURE_TRANSITION_MAP_MIN_SIZE) {
                OBJECT_STRUCTURE_PROFILE(recordTransitionVectorToMap());
                ObjectStructureTransitionTableMap* transitionTableMap = new (GC) ObjectStructureTransitionTableMap();
                for (size_t i = 0; i < m_transitionTableVectorBufferSize; i++) {
                    transitionTableMap->insert(std::make_pair(ObjectStructureTransitionMapItem(m_transitionTableVectorBuffer[i].m_propertyName, m_transitionTableVectorBuffer[i].m_descriptor),
                                                              m_transitionTableVectorBuffer[i].m_structure));
                }
                transitionTableMap->insert(std::make_pair(ObjectStructureTransitionMapItem(newTransitionItem.m_propertyName, newTransitionItem.m_descriptor),
                                                          newTransitionItem.m_structure));

                GC_FREE(m_transitionTableVectorBuffer);
                m_doesTransitionTableUseMap = true;
                m_transitionTableMap = transitionTableMap;
                m_transitionTableVectorBufferCapacity = 0;
                m_transitionTableVectorBufferSize = 0;
            } else {
                if (m_transitionTableVectorBufferCapacity <= (size_t)(m_transitionTableVectorBufferSize + 1)) {
                    m_transitionTableVectorBufferCapacity = std::min(computeVectorAllocateSize(m_transitionTableVectorBufferSize + 1), (size_t)std::numeric_limits<uint8_t>::max());
                    m_transitionTableVectorBuffer = (ObjectStructureTransitionVectorItem*)GC_REALLOC_NO_SHRINK(m_transitionTableVectorBuffer, sizeof(ObjectStructureTransitionVectorItem) * m_transitionTableVectorBufferCapacity);
                }
                m_transitionTableVectorBuffer[m_transitionTableVectorBufferSize] = newTransitionItem;
                m_transitionTableVectorBufferSize++;
            }
        }
    }

    return newObjectStructure;
}

ObjectStructure* ObjectStructureWithTransition::removeProperty(size_t pIndex)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithTransition, m_properties.size(), 'r'));
    ObjectStructureItemVector* newProperties = new ObjectStructureItemVector();
    newProperties->resizeFitWithUninitializedValues(m_properties.size() - 1);
    size_t pc = m_properties.size();

    size_t newIdx = 0;
    bool hasIndexString = false;
    bool hasSymbol = false;
    bool hasNonAtomicName = false;
    bool hasEnumerableProperty = false;
    for (size_t i = 0; i < pc; i++) {
        if (i == pIndex)
            continue;
        hasIndexString = hasIndexString | m_properties[i].m_propertyName.isIndexString();
        hasSymbol = hasSymbol | m_properties[i].m_propertyName.isSymbol();
        hasNonAtomicName = hasNonAtomicName | !m_properties[i].m_propertyName.hasAtomicString();
        hasEnumerableProperty = hasEnumerableProperty | m_properties[i].m_descriptor.isEnumerable();
        (*newProperties)[newIdx].m_propertyName = m_properties[i].m_propertyName;
        (*newProperties)[newIdx].m_descriptor = m_properties[i].m_descriptor;
        newIdx++;
    }

    return new ObjectStructureWithoutTransition(newProperties, hasIndexString, hasSymbol, hasNonAtomicName, hasEnumerableProperty);
}

ObjectStructure* ObjectStructureWithTransition::replacePropertyDescriptor(size_t idx, const ObjectStructurePropertyDescriptor& newDesc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithTransition, m_properties.size(), 'p'));
    ObjectStructureItemVector* newProperties = new ObjectStructureItemVector(m_properties);
    newProperties->at(idx).m_descriptor = newDesc;
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : newDesc.isEnumerable();
    return new ObjectStructureWithoutTransition(newProperties, m_hasIndexPropertyName, m_hasSymbolPropertyName, m_hasNonAtomicPropertyName, hasEnumerableProperty);
}

ObjectStructure* ObjectStructureWithTransition::convertToNonTransitionStructure()
{
    ObjectStructureItemVector* newProperties = new ObjectStructureItemVector(m_properties);
    return new ObjectStructureWithoutTransition(newProperties, m_hasIndexPropertyName, m_hasSymbolPropertyName, m_hasNonAtomicPropertyName, m_hasEnumerableProperty);
}

uint8_t PropertyNameMapWithCache::entryWidth(size_t count)
{
    // Zero marks an empty bucket; all live entries store propertyIndex + 1.
    if (count <= UINT8_MAX) {
        return sizeof(uint8_t);
    }
    if (count <= UINT16_MAX) {
        return sizeof(uint16_t);
    }
    ASSERT(count <= UINT32_MAX);
    return sizeof(uint32_t);
}

PropertyNameMapWithCache::PropertyNameMapWithCache(const ObjectStructureItemVector& properties)
{
    rebuild(properties);
}

size_t PropertyNameMapWithCache::hash(const ObjectStructurePropertyName& name) const
{
    // Atomic names normally use pointer identity. If a template contributed
    // non-atomic strings, hash all strings by content so equal names agree.
    size_t value;
    if (LIKELY(!m_hasNonAtomicNames)) {
        OBJECT_STRUCTURE_PROFILE(recordMapHash(false));
        // Atomic-string hashing is pointer identity, and symbols already use
        // their raw pointer. Clearing the atomic-string tag produces exactly
        // the same input without repeating the type dispatch in hashValue().
        value = name.rawValue() & ~static_cast<size_t>(OBJECT_PROPERTY_NAME_ATOMIC_STRING_VIAS);
    } else {
        OBJECT_STRUCTURE_PROFILE(recordMapHash(name.isPlainString()));
        value = name.isPlainString() ? name.plainString()->hashValue() : name.rawValue();
    }
    // Mix aligned pointers before masking off the low bits. Use 32-bit
    // arithmetic here to keep this inexpensive on ARM32 as well.
    uint32_t mixed = static_cast<uint32_t>(value ^ (value >> 16));
    mixed *= 0x9e3779b9U;
    return mixed ^ (mixed >> 16);
}

template <typename Entry>
void PropertyNameMapWithCache::insertEntry(const ObjectStructurePropertyName& name, size_t index)
{
    auto entries = static_cast<Entry*>(m_entries);
    if (m_denseCapacity) {
        OBJECT_STRUCTURE_PROFILE(recordMapIndexConversion());
        uint32_t numeric = name.tryToUseAsIndexProperty();
        if (numeric < m_denseCapacity) {
            entries[m_capacity + numeric] = static_cast<Entry>(index + 1);
            return;
        }
    }
    size_t slot = hash(name) & (m_capacity - 1);
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
    size_t profileProbes = 0;
#endif
    while (entries[slot]) {
        OBJECT_STRUCTURE_PROFILE_VALUE(profileProbes++);
        slot = (slot + 1) & (m_capacity - 1);
    }
    OBJECT_STRUCTURE_PROFILE(recordMapInsertProbes(profileProbes));
    entries[slot] = static_cast<Entry>(index + 1);
    m_occupied++;
}

template <typename Entry>
size_t PropertyNameMapWithCache::findEntry(const ObjectStructurePropertyName& name, const ObjectStructureItemVector& properties) const
{
    auto entries = static_cast<const Entry*>(m_entries);
    if (m_denseCapacity) {
        OBJECT_STRUCTURE_PROFILE(recordMapIndexConversion());
        uint32_t numeric = name.tryToUseAsIndexProperty();
        if (numeric < m_denseCapacity) {
            auto entry = entries[m_capacity + numeric];
            OBJECT_STRUCTURE_PROFILE(recordMapDenseFind(entry != 0));
            return entry ? static_cast<size_t>(entry) - 1 : SIZE_MAX;
        }
    }
    size_t slot = hash(name) & (m_capacity - 1);
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
    size_t profileProbes = 0;
#endif
    while (auto entry = entries[slot]) {
        OBJECT_STRUCTURE_PROFILE_VALUE(profileProbes++);
        size_t index = static_cast<size_t>(entry) - 1;
        const auto& candidate = properties[index].m_propertyName;
        if (candidate.rawValue() == name.rawValue()
            || (m_hasNonAtomicNames && name.isPlainString() && candidate == name)) {
            OBJECT_STRUCTURE_PROFILE(recordMapHashFind(true, profileProbes));
            return index;
        }
        slot = (slot + 1) & (m_capacity - 1);
    }
    OBJECT_STRUCTURE_PROFILE(recordMapHashFind(false, profileProbes));
    return SIZE_MAX;
}

void PropertyNameMapWithCache::rebuild(const ObjectStructureItemVector& properties)
{
    m_size = properties.size();
    m_entryWidth = entryWidth(m_size);
    m_hasNonAtomicNames = false;

    // A bounded low-index region, enabled only when at least half full.
    // A sparse key such as "4294967294" must never size this allocation.
    size_t denseCapacity = 16;
    while (denseCapacity < m_size) {
        denseCapacity *= 2;
    }
    size_t denseCount = 0;
    for (size_t i = 0; i < m_size; i++) {
        const auto& name = properties[i].m_propertyName;
        m_hasNonAtomicNames |= !name.hasAtomicString() && name.isPlainString();
        OBJECT_STRUCTURE_PROFILE(recordMapIndexConversion());
        denseCount += name.tryToUseAsIndexProperty() < denseCapacity;
    }
    if (denseCount < denseCapacity / 2) {
        denseCapacity = 0;
        denseCount = 0;
    }
    m_denseCapacity = denseCapacity;
    m_capacity = 4;
    while (m_size - denseCount > m_capacity - std::max(m_capacity / 4, static_cast<size_t>(1))) {
        m_capacity *= 2;
    }

    void* oldEntries = m_entries;
    size_t bytes = (m_capacity + m_denseCapacity) * m_entryWidth;
    OBJECT_STRUCTURE_PROFILE(recordMapBuild(m_size, bytes, oldEntries != nullptr, m_denseCapacity != 0, m_hasNonAtomicNames));
    m_entries = GC_MALLOC_ATOMIC(bytes);
    memset(m_entries, 0, bytes);
    m_occupied = 0;
    for (size_t i = 0; i < m_size; i++) {
        const auto& name = properties[i].m_propertyName;
        if (m_entryWidth == sizeof(uint8_t)) {
            insertEntry<uint8_t>(name, i);
        } else if (m_entryWidth == sizeof(uint16_t)) {
            insertEntry<uint16_t>(name, i);
        } else {
            insertEntry<uint32_t>(name, i);
        }
    }
    if (oldEntries) {
        GC_FREE(oldEntries);
    }
    if (m_size) {
        m_lastName = properties[m_size - 1].m_propertyName;
        m_lastIndex = m_size - 1;
    }
}

void PropertyNameMapWithCache::insert(const ObjectStructureItemVector& properties)
{
    OBJECT_STRUCTURE_PROFILE(recordMapInsert());
    ASSERT(properties.size() == m_size + 1);
    const auto& name = properties[properties.size() - 1].m_propertyName;
    if (m_denseCapacity) {
        OBJECT_STRUCTURE_PROFILE(recordMapIndexConversion());
    }
    bool dense = m_denseCapacity && name.tryToUseAsIndexProperty() < m_denseCapacity;
    if (entryWidth(properties.size()) != m_entryWidth
        || (!dense && m_occupied + 1 > m_capacity - std::max(m_capacity / 4, static_cast<size_t>(1)))
        || (!m_hasNonAtomicNames && !name.hasAtomicString() && name.isPlainString())) {
        rebuild(properties);
        return;
    }
    if (m_entryWidth == sizeof(uint8_t)) {
        insertEntry<uint8_t>(name, m_size);
    } else if (m_entryWidth == sizeof(uint16_t)) {
        insertEntry<uint16_t>(name, m_size);
    } else {
        insertEntry<uint32_t>(name, m_size);
    }
    m_lastName = name;
    m_lastIndex = m_size++;
}

size_t PropertyNameMapWithCache::find(const ObjectStructurePropertyName& name, const ObjectStructureItemVector& properties)
{
    if (name == m_lastName) {
        OBJECT_STRUCTURE_PROFILE(recordMapLastCache());
        return m_lastIndex;
    }
    m_lastName = name;
    // A non-atomic query can compare equal to an atomic stored name even
    // though its hash is content-based. This uncommon path must still work.
    if (UNLIKELY(!m_hasNonAtomicNames && !name.hasAtomicString() && name.isPlainString())) {
        m_lastIndex = SIZE_MAX;
#if defined(ESCARGOT_OBJECT_STRUCTURE_PROFILE)
        size_t profileComparisons = 0;
#endif
        for (size_t i = 0; i < properties.size(); i++) {
            OBJECT_STRUCTURE_PROFILE_VALUE(profileComparisons++);
            if (properties[i].m_propertyName == name) {
                m_lastIndex = i;
                break;
            }
        }
        OBJECT_STRUCTURE_PROFILE(recordMapLinearFallback(profileComparisons));
    } else if (m_entryWidth == sizeof(uint8_t)) {
        m_lastIndex = findEntry<uint8_t>(name, properties);
    } else if (m_entryWidth == sizeof(uint16_t)) {
        m_lastIndex = findEntry<uint16_t>(name, properties);
    } else {
        m_lastIndex = findEntry<uint32_t>(name, properties);
    }
    return m_lastIndex;
}

void* ObjectStructureWithMap::operator new(size_t size)
{
    static MAY_THREAD_LOCAL bool typeInited = false;
    static MAY_THREAD_LOCAL GC_descr descr;
    if (!typeInited) {
        GC_word obj_bitmap[GC_BITMAP_SIZE(ObjectStructureWithMap)] = { 0 };
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithMap, m_properties));
        GC_set_bit(obj_bitmap, GC_WORD_OFFSET(ObjectStructureWithMap, m_propertyNameMap));
        descr = GC_make_descriptor(obj_bitmap, GC_WORD_LEN(ObjectStructureWithMap));
        typeInited = true;
    }
    return GC_MALLOC_EXPLICITLY_TYPED(size, descr);
}


std::pair<size_t, Optional<const ObjectStructureItem*>> ObjectStructureWithMap::findProperty(const ObjectStructurePropertyName& s)
{
    if (!m_propertyNameMap) {
        OBJECT_STRUCTURE_PROFILE(recordMapLazyBuild());
        m_propertyNameMap = createPropertyNameMap(m_properties);
    }
    auto idx = m_propertyNameMap->find(s, *m_properties);
    if (idx == SIZE_MAX) {
        OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithMap, m_properties->size(), s, false, false, 0));
        return std::make_pair(SIZE_MAX, Optional<const ObjectStructureItem*>());
    }
    OBJECT_STRUCTURE_PROFILE(recordFind(ObjectStructureProfileKind::WithMap, m_properties->size(), s, true, false, 0));
    return std::make_pair(idx, &(m_properties->data()[idx]));
}

const ObjectStructureItem& ObjectStructureWithMap::readProperty(size_t idx)
{
    return m_properties->at(idx);
}

const ObjectStructureItem* ObjectStructureWithMap::properties() const
{
    return m_properties->data();
}

size_t ObjectStructureWithMap::propertyCount() const
{
    return m_properties->size();
}

ObjectStructure* ObjectStructureWithMap::addProperty(const ObjectStructurePropertyName& name, const ObjectStructurePropertyDescriptor& desc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithMap, m_properties->size(), 'a'));
    ObjectStructureItem newItem(name, desc);
    bool nameIsIndexString = m_hasIndexPropertyName ? true : name.isIndexString();
    bool hasSymbol = m_hasSymbolPropertyName ? true : name.isSymbol();
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : desc.isEnumerable();

    ObjectStructureItemVector* newProperties;
    Optional<PropertyNameMapWithCache*> newPropertyNameMap;

    if (m_isReferencedByInlineCache) {
        newProperties = new ObjectStructureItemVector(*m_properties, newItem);
    } else {
        newProperties = m_properties;
        newProperties->push_back(newItem);
        m_properties = nullptr;
        if (m_propertyNameMap) {
            m_propertyNameMap->insert(*newProperties);
            ASSERT(m_propertyNameMap->size() == newProperties->size());
            newPropertyNameMap = m_propertyNameMap;
            m_propertyNameMap = nullptr;
        }
    }

    ObjectStructure* newStructure = new ObjectStructureWithMap(newProperties, newPropertyNameMap, nameIsIndexString, hasSymbol, hasEnumerableProperty);
    return newStructure;
}

ObjectStructure* ObjectStructureWithMap::removeProperty(size_t pIndex)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithMap, m_properties->size(), 'r'));
    ObjectStructureItemVector* newProperties = new ObjectStructureItemVector();
    size_t ps = m_properties->size();
    newProperties->resizeFitWithUninitializedValues(ps - 1);

    size_t newIdx = 0;
    bool hasIndexString = false;
    bool hasSymbol = false;
    bool hasNonAtomicName = false;
    bool hasEnumerableProperty = false;
    for (size_t i = 0; i < ps; i++) {
        if (i == pIndex)
            continue;
        hasIndexString = hasIndexString | (*m_properties)[i].m_propertyName.isIndexString();
        hasSymbol = hasSymbol | (*m_properties)[i].m_propertyName.isSymbol();
        hasEnumerableProperty = hasEnumerableProperty | (*m_properties)[i].m_descriptor.isEnumerable();
        hasNonAtomicName = hasNonAtomicName | !(*m_properties)[i].m_propertyName.hasAtomicString();
        (*newProperties)[newIdx].m_propertyName = (*m_properties)[i].m_propertyName;
        (*newProperties)[newIdx].m_descriptor = (*m_properties)[i].m_descriptor;
        newIdx++;
    }

    if (!m_isReferencedByInlineCache) {
        m_properties = nullptr;
        m_propertyNameMap = nullptr;
    }
    if (newProperties->size() > ESCARGOT_OBJECT_STRUCTURE_ACCESS_CACHE_BUILD_MIN_SIZE) {
        return new ObjectStructureWithMap(newProperties, nullptr, hasIndexString, hasSymbol, hasEnumerableProperty);
    } else {
        return new ObjectStructureWithoutTransition(newProperties, hasIndexString, hasSymbol, hasNonAtomicName, hasEnumerableProperty);
    }
}

ObjectStructure* ObjectStructureWithMap::replacePropertyDescriptor(size_t idx, const ObjectStructurePropertyDescriptor& newDesc)
{
    OBJECT_STRUCTURE_PROFILE(recordOperation(ObjectStructureProfileKind::WithMap, m_properties->size(), 'p'));
    ObjectStructureItemVector* newProperties = m_properties;
    auto newPropertyNameMap = m_propertyNameMap;

    if (m_isReferencedByInlineCache) {
        newProperties = new ObjectStructureItemVector(*m_properties);
        newPropertyNameMap = nullptr;
    } else {
        m_properties = nullptr;
        m_propertyNameMap = nullptr;
    }

    newProperties->at(idx).m_descriptor = newDesc;
    bool hasEnumerableProperty = m_hasEnumerableProperty ? true : newDesc.isEnumerable();
    return new ObjectStructureWithMap(newProperties, newPropertyNameMap, m_hasIndexPropertyName, m_hasSymbolPropertyName, hasEnumerableProperty);
}
} // namespace Escargot
