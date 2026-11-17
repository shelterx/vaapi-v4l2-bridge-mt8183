/*
 * v4l2sl_capture_cpu_view(): the lazy CPU mapping the async de-tile worker
 * uses to redo a failed hardware job in software.
 *
 * Needs no device: the ioctl hook answers QUERYBUF and the mapping falls back
 * to anonymous memory. Checks that the mapping is idempotent, that concurrent
 * callers (decode thread + worker) map a slot exactly once and agree on the
 * pointers, that out-of-range input is refused, and that a one-plane format
 * (no UV plane to de-tile) is reported as unusable.
 */

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "v4l2stateless.h"

static int g_fail;
static atomic_int g_querybuf;
static int g_planes = 2;

static void expect_true(int cond, const char *tag)
{
    if (!cond) {
        fprintf(stderr, "FAIL %s\n", tag);
        g_fail++;
    } else {
        printf("OK %s\n", tag);
    }
}

static int mock_ioctl(int fd, unsigned long request, void *arg)
{
    (void)fd;
    if (request == VIDIOC_QUERYBUF) {
        struct v4l2_buffer *buf = arg;

        atomic_fetch_add(&g_querybuf, 1);
        buf->length = (uint32_t)g_planes;
        buf->m.planes[0].length = 4096;
        if (g_planes > 1)
            buf->m.planes[1].length = 2048;
        return 0;
    }
    errno = ENOTTY;
    return -1;
}

struct racer {
    struct v4l2sl_context *ctx;
    int slot;
    const uint8_t *y, *uv;
    int rc;
};

static void *racer_main(void *p)
{
    struct racer *r = p;

    int created = 0;

    r->rc = v4l2sl_capture_cpu_view(r->ctx, r->slot, &r->y, &r->uv, &created);
    return NULL;
}

int main(void)
{
    struct v4l2sl_context *ctx = calloc(1, sizeof(*ctx));
    const uint8_t *y = NULL, *uv = NULL, *y2 = NULL, *uv2 = NULL;
    pthread_t th[8];
    struct racer r[8];
    int i, before, same = 1, created = 0;

    v4l2sl_set_ioctl_hook(mock_ioctl);
    ctx->v4l2_fd = 5;               /* never used: the hook answers */
    ctx->capture_bufs_allocd = 4;

    /* 1: first call maps, second call is a no-op returning the same view */
    expect_true(v4l2sl_capture_cpu_view(ctx, 1, &y, &uv, &created) == 0 &&
                y && uv && created, "maps-both-planes");
    before = atomic_load(&g_querybuf);
    expect_true(v4l2sl_capture_cpu_view(ctx, 1, &y2, &uv2, &created) == 0 &&
                y2 == y && uv2 == uv && !created, "idempotent-same-pointers");
    expect_true(atomic_load(&g_querybuf) == before, "no-second-querybuf");

    /* 2: eight threads race for one fresh slot */
    before = atomic_load(&g_querybuf);
    for (i = 0; i < 8; i++) {
        r[i] = (struct racer){ .ctx = ctx, .slot = 3, .rc = -2 };
        pthread_create(&th[i], NULL, racer_main, &r[i]);
    }
    for (i = 0; i < 8; i++)
        pthread_join(th[i], NULL);
    for (i = 0; i < 8; i++)
        if (r[i].rc != 0 || r[i].y != r[0].y || r[i].uv != r[0].uv)
            same = 0;
    expect_true(same, "race-all-agree");
    expect_true(atomic_load(&g_querybuf) == before + 1, "race-maps-once");

    /* 3: bounds and bad arguments */
    expect_true(v4l2sl_capture_cpu_view(ctx, 4, &y2, &uv2, &created) < 0,
                "index-beyond-allocated");
    expect_true(v4l2sl_capture_cpu_view(ctx, -1, &y2, &uv2, &created) < 0,
                "negative");
    expect_true(v4l2sl_capture_cpu_view(NULL, 0, &y2, &uv2, &created) < 0,
                "null-ctx");
    expect_true(v4l2sl_capture_cpu_view(ctx, 0, NULL, &uv2, &created) < 0,
                "null-out");

    /* 4: a one-plane format has no UV plane to de-tile */
    g_planes = 1;
    expect_true(v4l2sl_capture_cpu_view(ctx, 2, &y2, &uv2, &created) < 0,
                "single-plane-unusable");

    /* 5: releasing a transient mapping lets the slot be mapped again, and the
     * release path is what keeps a failure-path recovery from pinning CMA */
    g_planes = 2;
    v4l2sl_capture_cpu_release(ctx, 1);
    before = atomic_load(&g_querybuf);
    created = 0;
    expect_true(v4l2sl_capture_cpu_view(ctx, 1, &y2, &uv2, &created) == 0 &&
                created && atomic_load(&g_querybuf) == before + 1,
                "release-then-remap");

    v4l2sl_set_ioctl_hook(NULL);
    for (i = 0; i < V4L2SL_MAX_CAPTURE_BUFS; i++) {
        free(ctx->capture_buf_ptr[i]);
        free(ctx->capture_uv_ptr[i]);
    }
    free(ctx);
    return g_fail ? 1 : 0;
}
