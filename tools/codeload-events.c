#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include "gc/gc.h"

static unsigned evals, mask, use_walk, gen, nrows, walk_count, walk_attempts;
static unsigned long walk_start, walk_allocated;
struct row {
    unsigned tag, eval, attempts, walks;
    unsigned long gc, heap, unmapped, free, allocated, disabled, extra, peak, walk_bytes;
};
static struct row rows[1024];

static void dump(void) {
    fprintf(stderr, "EVENT_SUMMARY %u %u %u %lu %u\n", evals, walk_attempts, walk_count, walk_allocated, nrows);
    for (unsigned i=0; i<nrows; ++i) {
        struct row *r=&rows[i];
        fprintf(stderr, "EVENT %u %u %u %u %lu %lu %lu %lu %lu %lu %lu %lu %lu\n",
                r->tag,r->eval,r->attempts,r->walks,r->gc,r->heap,r->unmapped,r->free,
                r->allocated,r->disabled,r->extra,r->peak,r->walk_bytes);
    }
}
__attribute__((constructor)) static void setup(void) {
    const char *s=getenv("CODELOAD_CONTROL");
    if (!s || strlen(s)!=3) abort();
    gen=s[0]-'0'; mask=s[1]-'0'; use_walk=s[2]-'0';
    atexit(dump);
}
void __real_GC_enable_generational(size_t);
void __wrap_GC_enable_generational(size_t threshold) {
    if (gen) __real_GC_enable_generational(threshold);
}
void __real_GC_configure_allocation_pattern(unsigned);
void __wrap_GC_configure_allocation_pattern(unsigned mode) {
    if (gen) __real_GC_configure_allocation_pattern(mode);
}
unsigned __wrap_GC_allocation_pattern_mode(void) {
    unsigned bit=evals==0 ? 1 : evals<=1600 ? 2 : 4;
    return (mask & bit) ? 4 : 0;
}
int codeload_walk(void) {
    ++walk_attempts;
    if (!use_walk) return 0;
    ++walk_count;
    walk_start=GC_get_total_bytes();
    return 1;
}
void codeload_walk_end(void) {
    walk_allocated+=GC_get_total_bytes()-walk_start;
}
void codeload_event(int tag,unsigned long gc,unsigned long heap,unsigned long unmapped,
                    unsigned long free_bytes,unsigned long allocated,unsigned long disabled,unsigned long extra) {
    struct rusage usage;
    if (nrows==sizeof(rows)/sizeof(rows[0])) return;
    getrusage(RUSAGE_SELF,&usage);
    rows[nrows++]=(struct row){tag,evals,walk_attempts,walk_count,gc,heap,unmapped,free_bytes,allocated,
                              disabled,extra,usage.ru_maxrss,walk_allocated};
}
void codeload_eval(void) {
    ++evals;
    if (evals==1 || evals==1601 || evals%100==0 || (evals>1600 && evals%5==0)) {
        struct mallinfo2 info=mallinfo2();
        codeload_event(3,GC_get_gc_no(),GC_get_heap_size()+GC_get_unmapped_bytes(),
                       GC_get_unmapped_bytes(),GC_get_free_bytes(),GC_get_total_bytes(),
                       GC_is_disabled(),info.uordblks+info.hblkhd);
    }
}
