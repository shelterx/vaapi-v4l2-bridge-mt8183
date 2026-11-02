#ifndef V4L2SL_LOGSITE_H
#define V4L2SL_LOGSITE_H

/*
 * Failure logging without spam.
 *
 * Every failure kind has one site: it is counted always (cheap, atomic),
 * printed rarely (first occurrence immediately, then at most one line per
 * interval, noting how many similar lines were suppressed), and summarised
 * once at v4l2sl_terminate as `name=total` for every non-zero counter. A
 * clean run prints nothing. Rare, serious events (the one-strike HW->SW
 * fallback) are counted but printed unconditionally, not rate-limited.
 *
 * Each printed line carries a human-readable wall-clock stamp so bridge
 * messages line up against dmesg / journalctl -k.
 *
 * See failure-logging.md.
 */

#include <stdatomic.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define V4L2SL_MAX_LOGSITES 32
#define V4L2SL_LOG_INTERVAL_MS_DEFAULT 2000

struct v4l2sl_logsite {
    const char   *name;
    atomic_ulong  total;       /* every occurrence                        */
    atomic_ulong  suppressed;  /* occurrences since the last printed line */
    atomic_long   next_ms;     /* earliest time the next line may print   */
};

/* Registry, defined in v4l2stateless.c. */
extern struct v4l2sl_logsite *g_v4l2sl_logsites[V4L2SL_MAX_LOGSITES];
extern int g_v4l2sl_nlogsites;

/* Monotonic time in ms, for rate-limiting. */
static inline long v4l2sl_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* Human-readable wall-clock stamp, e.g. "2026-10-30T12:34:56.789". */
static inline void v4l2sl_logstamp(char *buf, size_t buflen)
{
    struct timespec ts;
    time_t sec;
    struct tm tm;
    int used;

    clock_gettime(CLOCK_REALTIME, &ts);
    sec = ts.tv_sec;
    localtime_r(&sec, &tm);
    used = snprintf(buf, buflen, "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec,
                    (int)(ts.tv_nsec / 1000000));
    if (used < 0 || (size_t)used >= buflen)
        buf[buflen - 1] = '\0';
}

/* Interval between printed lines, overridable with V4L2SL_LOG_INTERVAL_MS. */
static inline long v4l2sl_log_interval_ms(void)
{
    static long cached = 0;
    if (cached == 0) {
        const char *e = getenv("V4L2SL_LOG_INTERVAL_MS");
        long v = e ? strtol(e, NULL, 10) : 0;
        cached = (v > 0) ? v : V4L2SL_LOG_INTERVAL_MS_DEFAULT;
    }
    return cached;
}

static inline void v4l2sl_logsite_register(struct v4l2sl_logsite *s)
{
    if (g_v4l2sl_nlogsites < V4L2SL_MAX_LOGSITES)
        g_v4l2sl_logsites[g_v4l2sl_nlogsites++] = s;
}

/* Count an occurrence without printing (for events that print on their own). */
static inline void v4l2sl_logsite_count(struct v4l2sl_logsite *s)
{
    atomic_fetch_add(&s->total, 1);
}

/* Unconditional timestamped line (rare, serious events). */
static inline void v4l2sl_log(const char *fmt, ...)
{
    char ts[32];
    va_list ap;

    v4l2sl_logstamp(ts, sizeof ts);
    va_start(ap, fmt);
    fprintf(stderr, "%s v4l2stateless: ", ts);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

#define V4L2SL_LOGSITE(var, label) \
    static struct v4l2sl_logsite var = { .name = (label) }

/* Register a site with the exit-summary registry at load time. */
#define V4L2SL_LOGSITE_REG(var) \
    __attribute__((constructor)) static void v4l2sl_reg_##var(void) \
    { v4l2sl_logsite_register(&var); }

/*
 * Count the occurrence and print at most one line per interval. Only one
 * thread prints per interval (the compare-and-swap on next_ms), even with
 * several streams running. The "suppressed" note rides on the next line
 * after a quiet period, not at the moment of the burst.
 */
#define V4L2SL_LOG_LIMITED(site, fmt, ...)                                      \
    do {                                                                        \
        atomic_fetch_add(&(site)->total, 1);                                    \
        long now_ = v4l2sl_now_ms();                                            \
        long due_ = atomic_load(&(site)->next_ms);                              \
        char tsbuf_[32];                                                        \
        if (now_ >= due_ &&                                                     \
            atomic_compare_exchange_strong(&(site)->next_ms, &due_,             \
                                           now_ + v4l2sl_log_interval_ms())) {  \
            unsigned long skipped_ = atomic_exchange(&(site)->suppressed, 0);   \
            v4l2sl_logstamp(tsbuf_, sizeof tsbuf_);                             \
            fprintf(stderr, "%s v4l2stateless: " fmt, tsbuf_, ##__VA_ARGS__);   \
            if (skipped_)                                                       \
                fprintf(stderr, "  (+%lu similar suppressed)\n", skipped_);     \
        } else {                                                                \
            atomic_fetch_add(&(site)->suppressed, 1);                           \
        }                                                                       \
    } while (0)

/* One line at v4l2sl_terminate: name=total for every non-zero counter. */
void v4l2sl_logsite_summary(void);

#endif /* V4L2SL_LOGSITE_H */
