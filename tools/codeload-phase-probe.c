#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/resource.h>

/* Observe the frozen driver's existing shell opens and prints. Use kernel
   accounting so no JS allocations or collector API calls are added. */
static void checkpoint(const char *event, const char *name)
{
    int saved_errno = errno;
    char status[8192];
    long rss = -1;
    struct rusage usage;
    int fd = open("/proc/self/status", O_RDONLY);
    if (fd >= 0) {
        ssize_t n = read(fd, status, sizeof(status) - 1);
        close(fd);
        if (n > 0) {
            status[n] = 0;
            const char *line = strstr(status, "\nVmRSS:");
            if (line != NULL)
                rss = strtol(line + 7, NULL, 10);
        }
    }
    if (getrusage(RUSAGE_SELF, &usage) == 0)
        dprintf(STDERR_FILENO, "PHASE %s %s rss_kib=%ld hwm_kib=%ld\n",
                event, name, rss, usage.ru_maxrss);
    errno = saved_errno;
}

FILE *fopen(const char *path, const char *mode)
{
    static FILE *(*real_fopen)(const char *, const char *);
    if (!real_fopen)
        real_fopen = dlsym(RTLD_NEXT, "fopen");
    size_t n = strlen(path);
    if (n > 3 && strcmp(path + n - 3, ".js") == 0 && strcmp(mode, "rb") == 0)
        checkpoint("load", strrchr(path, '/') ? strrchr(path, '/') + 1 : path);
    return real_fopen(path, mode);
}

int puts(const char *s)
{
    static int (*real_puts)(const char *);
    if (!real_puts)
        real_puts = dlsym(RTLD_NEXT, "puts");
    if (strcmp(s, "Richards: Skipped") == 0)
        checkpoint("begin", "suites");
    if (strncmp(s, "CodeLoad: ", 10) == 0)
        checkpoint("end", "codeload");
    if (strncmp(s, "ITER ", 5) == 0 || strncmp(s, "DONE ", 5) == 0)
        checkpoint("step", s);
    return real_puts(s);
}
