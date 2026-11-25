#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gc/gc.h"

/* The runner gives all cells the same argv and equal-length env values.
   Switch only the two collector setup calls and the compiler's site filter.
   CodeLoad's zero executed sites is checked independently in the report. */
static int generational, refined;
__attribute__((constructor)) static void read_controls(void)
{
    const char *control = getenv("CODELOAD_CONTROL");
    if (!control || strlen(control) != 2)
        abort();
    generational = control[0] == '1';
    refined = control[1] == '1';
}

void __real_GC_enable_generational(size_t);
void __wrap_GC_enable_generational(size_t threshold)
{
    if (generational)
        __real_GC_enable_generational(threshold);
}

void __real_GC_configure_allocation_pattern(unsigned);
void __wrap_GC_configure_allocation_pattern(unsigned mode)
{
    if (generational)
        __real_GC_configure_allocation_pattern(mode);
}

unsigned __wrap_GC_allocation_pattern_mode(void)
{
    return refined ? 4 : 0;
}
