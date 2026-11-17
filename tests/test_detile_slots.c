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
 *   - waiting on a NULL context is a no-op.
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
