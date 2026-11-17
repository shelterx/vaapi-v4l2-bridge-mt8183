/*
 * test_detile_slots.c — a de-tile job belongs to one context.
 *
 * The async de-tile slot used to be a file-static: starting a job joined
 * whatever job was in flight, whichever stream queued it. With one 10 ms MDP3
 * stream that serialised every GPU stream's hand-off (and, because the pull
 * runs on the decode thread, their decode too). The slot now lives on the
 * context, and this pins that down:
 *
 *   - a queued job is "pending" for its own surface only;
 *   - waiting on a different context does not join it, and does not clear it;
 *   - waiting on the owning context joins it;
 *   - the result is byte-identical to the synchronous de-tile;
 *   - waiting on a NULL context is a no-op;
 *   - two contexts de-tiling at the same time keep their own sources, slots
 *     and results (the per-context slot must not let one job overwrite or
 *     complete another).
 *
 * Only the software de-tile runs here, so no GPU, decoder or display is
 * needed; without a render node (GBM bo allocation) the test skips.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <gbm.h>

#include "v4l2stateless.h"

#define SKIP 77

#define AW 640u
#define AH 384u
#define WIDTH 640u
#define HEIGHT 360u

static int g_fail;

static void expect(int cond, const char *tag)
{
    if (!cond) {
        fprintf(stderr, "FAIL %s\n", tag);
        g_fail++;
    } else {
        printf("OK %s\n", tag);
    }
}

static void ctx_init(struct v4l2sl_context *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    pthread_mutex_init(&ctx->adetile.lock, NULL);
    pthread_mutex_init(&ctx->cap_map_lock, NULL);
}

struct worker {
    struct v4l2sl_context *ctx;
    struct v4l2sl_surface *surf;
    const uint8_t *ysrc;
    const uint8_t *usrc;
    int iterations;
    int bad;
};

/* Detile this context's surface repeatedly; the job slot is per context, so
 * two of these running at once must not interfere. */
static void *worker_main(void *p)
{
    struct worker *w = p;

    for (int i = 0; i < w->iterations; i++) {
        if (v4l2sl_gbm_surface_detile_async(w->surf, w->ysrc, w->usrc,
                                            AW, AH) != 0)
            w->bad++;
        v4l2sl_gbm_detile_wait(w->ctx);
        if (v4l2sl_gbm_detile_pending_for(w->surf) != 0)
            w->bad++;
    }
    return NULL;
}

static void surface_init(struct v4l2sl_surface *s, struct v4l2sl_context *ctx)
{
    memset(s, 0, sizeof(*s));
    s->width = WIDTH;
    s->height = HEIGHT;
    s->format = VA_FOURCC_NV12;
    s->stride = AW;
    s->aligned_h = AH;
    s->capture_owner = ctx;
    s->buf_index = 0;
}

int main(void)
{
    struct v4l2sl_context ctx_a, ctx_b;
    struct v4l2sl_surface s_a, s_ref;
    uint8_t *ysrc, *usrc;
    size_t ysz = (AW / 16) * (AH / 32) * 512;
    size_t usz = (AW / 16) * (AH / 32) * 256;
    int rc;

    ctx_init(&ctx_a);
    ctx_init(&ctx_b);
    surface_init(&s_a, &ctx_a);
    surface_init(&s_ref, &ctx_a);   /* reference: synchronous de-tile */

    if (v4l2sl_gbm_surface_ensure(&s_a) || v4l2sl_gbm_surface_ensure(&s_ref)) {
        printf("SKIP: no GBM bo (no render node/CMA?)\n");
        return SKIP;
    }

    ysrc = malloc(ysz);
    usrc = malloc(usz);
    if (!ysrc || !usrc)
        return 1;
    for (size_t i = 0; i < ysz; i++)
        ysrc[i] = (uint8_t)(i * 7 + 3);
    for (size_t i = 0; i < usz; i++)
        usrc[i] = (uint8_t)(i * 13 + 11);

    /* Baseline: the synchronous de-tile into a second bo. */
    expect(v4l2sl_gbm_surface_detile_mm21(&s_ref, ysrc, usrc, AW, AH) == 0,
           "sync-detile");

    /* Queue an asynchronous job on context A. */
    rc = v4l2sl_gbm_surface_detile_async(&s_a, ysrc, usrc, AW, AH);
    expect(rc == 0, "async-queued");
    expect(v4l2sl_gbm_detile_pending_for(&s_a) == 1, "pending-for-owner");

    /* The point of the per-context slot: another stream's wait is a no-op. */
    v4l2sl_gbm_detile_wait(&ctx_b);
    expect(v4l2sl_gbm_detile_pending_for(&s_a) == 1,
           "other-context-wait-leaves-job");

    /* A NULL context (hand-built surface, no owner) waits for nothing. */
    v4l2sl_gbm_detile_wait(NULL);
    expect(v4l2sl_gbm_detile_pending_for(&s_a) == 1, "null-wait-noop");

    /* The owning context joins it. */
    v4l2sl_gbm_detile_wait(&ctx_a);
    expect(v4l2sl_gbm_detile_pending_for(&s_a) == 0, "owner-wait-joins");

    /* And the asynchronous result matches the synchronous one byte for byte. */
    {
        uint32_t stride_a = 0, stride_r = 0;
        void *map_a = NULL, *map_r = NULL;
        const uint8_t *a, *r;
        uint32_t uv = s_a.bo_uv_rows ? s_a.bo_uv_rows : HEIGHT;
        size_t rows = uv + (uv + 1) / 2;
        size_t bad = 0;

        a = gbm_bo_map(s_a.gbm_bo, 0, 0, WIDTH, rows, GBM_BO_TRANSFER_READ,
                       &stride_a, &map_a);
        r = gbm_bo_map(s_ref.gbm_bo, 0, 0, WIDTH, rows, GBM_BO_TRANSFER_READ,
                       &stride_r, &map_r);
        if (!a || !r) {
            printf("FAIL bo-map\n");
            g_fail++;
        } else {
            for (size_t y = 0; y < rows; y++)
                bad += memcmp(a + y * stride_a, r + y * stride_r,
                              AW < stride_a ? AW : stride_a) != 0;
            expect(bad == 0, "async-matches-sync");
            gbm_bo_unmap(s_ref.gbm_bo, map_r);
            gbm_bo_unmap(s_a.gbm_bo, map_a);
        }
    }

    /* Two contexts in parallel: separate sources, separate bos, separate
     * slots. If the slot were shared, one stream's job would be joined (or
     * its result taken) by the other and the bos would disagree with their
     * own sources. */
    {
        struct v4l2sl_context pa, pb;
        struct v4l2sl_surface sa, sb;
        struct worker wa, wb;
        pthread_t ta, tb;
        uint8_t *ya = malloc(ysz), *ua = malloc(usz);
        uint8_t *yb = malloc(ysz), *ub = malloc(usz);
        int same = 1;

        ctx_init(&pa);
        ctx_init(&pb);
        surface_init(&sa, &pa);
        surface_init(&sb, &pb);
        if (!ya || !ua || !yb || !ub || v4l2sl_gbm_surface_ensure(&sa) ||
            v4l2sl_gbm_surface_ensure(&sb)) {
            printf("SKIP: parallel phase needs two bos\n");
            return SKIP;
        }
        for (size_t i = 0; i < ysz; i++) {
            ya[i] = (uint8_t)(i * 3 + 1);
            yb[i] = (uint8_t)(i * 5 + 2);
        }
        for (size_t i = 0; i < usz; i++) {
            ua[i] = (uint8_t)(i * 11 + 7);
            ub[i] = (uint8_t)(i * 17 + 9);
        }
        wa = (struct worker){ &pa, &sa, ya, ua, 10, 0 };
        wb = (struct worker){ &pb, &sb, yb, ub, 10, 0 };
        pthread_create(&ta, NULL, worker_main, &wa);
        pthread_create(&tb, NULL, worker_main, &wb);
        pthread_join(ta, NULL);
        pthread_join(tb, NULL);
        expect(wa.bad == 0 && wb.bad == 0, "parallel-no-failures");

        /* Each bo must match a synchronous de-tile of its *own* source. */
        for (int which = 0; which < 2; which++) {
            struct v4l2sl_surface ref;
            const uint8_t *ry = which ? yb : ya;
            const uint8_t *ru = which ? ub : ua;
            struct v4l2sl_surface *got = which ? &sb : &sa;
            uint32_t sg = 0, sr = 0;
            void *mg = NULL, *mr = NULL;
            const uint8_t *g, *r;
            uint32_t uv = got->bo_uv_rows ? got->bo_uv_rows : HEIGHT;
            size_t rows = uv + (uv + 1) / 2;
            size_t bad = 0;

            surface_init(&ref, which ? &pb : &pa);
            if (v4l2sl_gbm_surface_ensure(&ref) ||
                v4l2sl_gbm_surface_detile_mm21(&ref, ry, ru, AW, AH) != 0) {
                same = 0;
                break;
            }
            g = gbm_bo_map(got->gbm_bo, 0, 0, WIDTH, rows,
                           GBM_BO_TRANSFER_READ, &sg, &mg);
            r = gbm_bo_map(ref.gbm_bo, 0, 0, WIDTH, rows,
                           GBM_BO_TRANSFER_READ, &sr, &mr);
            if (!g || !r) {
                same = 0;
                break;
            }
            for (size_t y = 0; y < rows; y++)
                bad += memcmp(g + y * sg, r + y * sr,
                              AW < sg ? AW : sg) != 0;
            gbm_bo_unmap(ref.gbm_bo, mr);
            gbm_bo_unmap(got->gbm_bo, mg);
            v4l2sl_gbm_surface_destroy(&ref);
            if (bad)
                same = 0;
        }
        expect(same, "parallel-results-match-own-source");

        v4l2sl_gbm_surface_destroy(&sa);
        v4l2sl_gbm_surface_destroy(&sb);
        pthread_mutex_destroy(&pa.adetile.lock);
        pthread_mutex_destroy(&pb.adetile.lock);
        pthread_mutex_destroy(&pa.cap_map_lock);
        pthread_mutex_destroy(&pb.cap_map_lock);
        free(ya);
        free(ua);
        free(yb);
        free(ub);
    }

    free(ysrc);
    free(usrc);
    v4l2sl_gbm_surface_destroy(&s_a);
    v4l2sl_gbm_surface_destroy(&s_ref);
    pthread_mutex_destroy(&ctx_a.adetile.lock);
    pthread_mutex_destroy(&ctx_b.adetile.lock);
    pthread_mutex_destroy(&ctx_a.cap_map_lock);
    pthread_mutex_destroy(&ctx_b.cap_map_lock);
    return g_fail ? 1 : 0;
}
