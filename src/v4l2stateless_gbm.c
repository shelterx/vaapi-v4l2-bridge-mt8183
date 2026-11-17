/*
 * v4l2stateless_gbm.c — GBM-backed display surfaces.
 *
 * Why a GBM bo instead of exporting the capture buffer: the MT8183
 * decoder (/dev/video5) outputs only tiled MM21, which the GL/ANGLE
 * sampler cannot read as NV12. (EXPBUF of the capture CMA itself works
 * fine on MT8183 — the earlier EINVAL was a debug artifact; see
 * docs/mt8183-notes.md §9 — but it would hand the GPU the raw tiled
 * plane, which is exactly the bug that caused the green/violet flicker.)
 * So pull_capture de-tiles MM21 -> NV12 and copies the frame into a
 * driver-owned linear GBM bo on the render node; the export path hands
 * out that single-object NV12 dmabuf descriptor (Y at offset 0, UV at
 * offset stride*h) that Chromium's zero-copy GL path requires
 * (vaapi_wrapper rejects num_objects != 1).
 * (Historical note: this file exists because on the original RK3588
 * target the VPU capture CMA could not be EXPBUF'd at all — it hung —
 * so a GBM bo was the only exportable buffer from the start.)
 *
 * Platform notes:
 *   - The bo is R8 geometry with h + ceil(h/2) rows and the NV12 planes
 *     live at byte offsets, because the render node refuses multiplanar
 *     YUV gbm bos (tests/gbm_probe.c; that probe ran on the RK3588 box
 *     with panthor/Mesa 26.0.8, and the R8 shape is what real Chromium
 *     playback uses on this Mali-G72 box).
 *   - Never hand out a GR88-only image as the primary shape: Mesa
 *     per-plane GR88 dmabuf import samples black.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <gbm.h>
#include <drm_fourcc.h>
#include <va/va.h>
#include <va/va_drmcommon.h>
#include <pthread.h>
#include <time.h>

#include "v4l2stateless.h"

/*
 * gbm_bo_map()/gbm_bo_unmap() go through the shared GBM device's DRI screen,
 * and panfrost flushes that screen's context on the way in and out. Two
 * threads mapping at once corrupt its batch table (seen: SIGSEGV inside
 * panfrost_batch_init from dri_flush). With one de-tile job per context that
 * is reachable - two software de-tiles, or one and a memfd refill - so the
 * map/unmap pairs are serialised here. Holding the lock across the copy is
 * deliberate: the hazard is Mesa's, not ours.
 */
static pthread_mutex_t g_bo_map_lock = PTHREAD_MUTEX_INITIALIZER;

static struct gbm_device *g_gbm_dev;
static int g_gbm_fd = -1;
static int g_gbm_failed;

static struct gbm_device *v4l2sl_gbm_device(void)
{
    const char *node;

    if (g_gbm_dev)
        return g_gbm_dev;
    if (g_gbm_failed)
        return NULL;
    node = getenv("V4L2SL_RENDER_NODE");
    if (!node)
        node = "/dev/dri/renderD128";
    g_gbm_fd = open(node, O_RDWR | O_CLOEXEC);
    if (g_gbm_fd < 0) {
        fprintf(stderr, "v4l2stateless: gbm: open %s failed: %s\n",
                node, strerror(errno));
        g_gbm_failed = 1;
        return NULL;
    }
    g_gbm_dev = gbm_create_device(g_gbm_fd);
    if (!g_gbm_dev) {
        fprintf(stderr, "v4l2stateless: gbm_create_device(%s) failed\n", node);
        close(g_gbm_fd);
        g_gbm_fd = -1;
        g_gbm_failed = 1;
    }
    return g_gbm_dev;
}

struct gbm_device *v4l2sl_gbm_device_ref(void)
{
    return v4l2sl_gbm_device();
}

static void copy_plane(uint8_t *dst, uint32_t dst_stride, const uint8_t *src,
                       uint32_t src_stride, uint32_t width, uint32_t rows)
{
    if (dst_stride == src_stride) {
        memcpy(dst, src, (size_t)dst_stride * rows);
        return;
    }
    for (uint32_t y = 0; y < rows; y++)
        memcpy(dst + (size_t)dst_stride * y, src + (size_t)src_stride * y,
               width);
}

int v4l2sl_gbm_surface_upload(struct v4l2sl_surface *s, const void *src,
                              uint32_t src_stride, uint32_t src_alh)
{
    uint32_t w = s->width, h = s->height;
    uint32_t uv = s ? (s->bo_uv_rows ? s->bo_uv_rows : h) : h;
    uint8_t *dst;
    void *map_data = NULL;
    uint32_t mstride = 0;

    if (!s || !s->gbm_bo || !src || !src_stride)
        return -1;
    /* vaPutImage / sync() write the same bo a pending de-tile may be
     * writing; join first so the two never interleave. */
    v4l2sl_gbm_detile_wait(s->capture_owner);
    pthread_mutex_lock(&g_bo_map_lock);
    dst = gbm_bo_map(s->gbm_bo, 0, 0, w, uv + (uv + 1) / 2,
                     GBM_BO_TRANSFER_WRITE, &mstride, &map_data);
    if (!dst) {
        pthread_mutex_unlock(&g_bo_map_lock);
        fprintf(stderr, "v4l2stateless: gbm: bo map failed\n");
        return -1;
    }
    copy_plane(dst, mstride, src, src_stride, w, h);
    copy_plane(dst + (size_t)mstride * uv, mstride,
               (const uint8_t *)src + (size_t)src_stride * src_alh,
               src_stride, w + (w & 1), (h + 1) / 2);
    gbm_bo_unmap(s->gbm_bo, map_data);
    pthread_mutex_unlock(&g_bo_map_lock);
    return 0;
}

/* Lazy memfd: refill the memfd snapshot (capture layout: Y@0,
 * UV@stride*aligned_h) from the bo (image layout: Y@0, UV@gbm_stride*h).
 * Only the visible rows/bytes are copied — padding is never read. */
int v4l2sl_surface_ensure_memfd(struct v4l2sl_surface *s)
{
    uint32_t sz, w, h, uvw;
    void *bo_map = NULL, *m;
    uint8_t *bo_data = NULL, *dst;
    uint32_t bo_stride = 0;

    if (!s || !v4l2sl_memfd_stale(s) || !s->gbm_bo)
        return 0;
    v4l2sl_gbm_detile_wait(s->capture_owner);   /* bo may still be written */
    v4l2sl_gpu_wait_surface(s); /* ... and the GPU may still be rendering it */
    if (s->memfd_fd < 0 || !s->stride || !s->aligned_h)
        return -1;
    w = s->width;
    h = s->height;
    uvw = w + (w & 1);
    uint32_t uv = s->bo_uv_rows ? s->bo_uv_rows : h;
    sz = v4l2sl_capture_plane_size(V4L2_PIX_FMT_NV12, s->stride, s->aligned_h);
    if (v4l2sl_surface_grow_memfd(s, sz) < 0)
        return -1;
    pthread_mutex_lock(&g_bo_map_lock);
    bo_data = gbm_bo_map(s->gbm_bo, 0, 0, w, uv + (uv + 1) / 2,
                         GBM_BO_TRANSFER_READ, &bo_stride, &bo_map);
    if (!bo_data) {
        pthread_mutex_unlock(&g_bo_map_lock);
        return -1;
    }
    m = v4l2sl_surface_map_memfd(s, sz);
    if (!m) {
        gbm_bo_unmap(s->gbm_bo, bo_map);
        pthread_mutex_unlock(&g_bo_map_lock);
        return -1;
    }
    dst = m;
    for (uint32_t y = 0; y < h; y++)
        memcpy(dst + (size_t)s->stride * y,
               bo_data + (size_t)bo_stride * y, w);
    dst = (uint8_t *)m + (size_t)s->stride * s->aligned_h;
    for (uint32_t y = 0; y < (h + 1) / 2; y++)
        memcpy(dst + (size_t)s->stride * y,
               bo_data + (size_t)bo_stride * (uv + y), uvw);
    gbm_bo_unmap(s->gbm_bo, bo_map);
    pthread_mutex_unlock(&g_bo_map_lock);
    s->last_writer = V4L2SL_WRITER_MEMFD;
    return 0;
}

/* Both hardware writers (MDP3 and the GPU shader) emit linear NV12 whose UV
 * plane starts at stride*aligned_h, because the MM21 capture geometry is the
 * aligned one (e.g. 1088 rows for 1080) and the tiled source has no padding
 * concept the shader could stop at.
 *
 * The software path keeps the visible layout (UV at stride*height), so its
 * behaviour — and every buffer already handed to Chromium — is unchanged. */
/* Deliberately the process-wide mode, not a context's: the bo layout is fixed
 * when the surface is created, before any context owns it. Both hardware tiers
 * write the same aligned layout and the software de-tiler writes whichever
 * layout the bo has, because every reader and writer keys off bo_uv_rows()
 * rather than assuming one. So tiers can be mixed (V4L2SL_DETILE=auto hands
 * MDP3 to one context and gpu/sw to the others) without a per-surface tier.
 * With auto this returns true, i.e. every surface gets the hardware layout. */
static int hw_layout_requested(void)
{
    return v4l2sl_detile_mode() != V4L2SL_DETILE_SW;
}

static uint32_t surface_uv_rows(const struct v4l2sl_surface *s)
{
    if (!hw_layout_requested())
        return s->height;
    if (s->aligned_h)
        return s->aligned_h;
    if (s->capture_owner && s->capture_owner->cap_height)
        return s->capture_owner->cap_height;
    return (s->height + 63) & ~63u;
}

int v4l2sl_gbm_surface_ensure(struct v4l2sl_surface *s)
{
    struct gbm_device *dev;
    uint32_t rows;

    if (!s || s->width == 0 || s->height == 0)
        return -1;
    if (s->gbm_bo)
        return 0;
    /* NV12 only; everything else keeps the clean fallback. cap_fourcc is
     * set by decode; VPP/upload surfaces carry the VA fourcc instead.
     * MM21 capture is de-tiled to NV12 into this bo, so allow it too. */
    if (s->cap_fourcc && s->cap_fourcc != V4L2_PIX_FMT_NV12 &&
        s->cap_fourcc != V4L2_PIX_FMT_MM21)
        return -1;
    if (s->format && s->format != VA_FOURCC_NV12)
        return -1;
    dev = v4l2sl_gbm_device();
    if (!dev)
        return -1;
    s->bo_uv_rows = surface_uv_rows(s);
    rows = s->bo_uv_rows + (s->bo_uv_rows + 1) / 2;
    s->gbm_bo = gbm_bo_create(dev, s->width, rows, GBM_FORMAT_R8,
                              GBM_BO_USE_LINEAR);
    if (!s->gbm_bo) {
        fprintf(stderr, "v4l2stateless: gbm: bo create %ux%u failed\n",
                s->width, rows);
        return -1;
    }
    s->gbm_stride = gbm_bo_get_stride_for_plane(s->gbm_bo, 0);
    v4l2sl_gbm_surface_sync(s);
    return 0;
}

/* Upload the surface's current pixel data into its bo. The pixels live in
 * either cpu_ptr (VPP output, vaPutImage uploads; image layout: Y@0,
 * UV@cpu_stride*h) or the memfd (decode pull_capture; capture layout).
 * The last_writer marker says which backing was written last. */
int v4l2sl_gbm_surface_sync(struct v4l2sl_surface *s)
{
    if (!s || !s->gbm_bo)
        return -1;
    if (s->last_writer == V4L2SL_WRITER_BO)
        return 0; /* bo already holds the latest frame (lazy memfd) */
    if (s->last_writer == V4L2SL_WRITER_CPU && s->cpu_ptr && s->cpu_stride)
        return v4l2sl_gbm_surface_upload(s, s->cpu_ptr, s->cpu_stride,
                                         s->height);
    if (s->last_writer == V4L2SL_WRITER_MEMFD && s->memfd_fd >= 0 &&
        s->stride && s->aligned_h) {
        uint32_t sz = v4l2sl_capture_plane_size(V4L2_PIX_FMT_NV12,
                                                s->stride, s->aligned_h);
        void *m = v4l2sl_surface_map_memfd(s, sz);
        int r;

        if (!m)
            return -1;
        r = v4l2sl_gbm_surface_upload(s, m, s->stride, s->aligned_h);
        if (r == 0)
            s->last_writer = V4L2SL_WRITER_BO; /* bo now holds the frame */
        return r;
    }
    return -1;
}

/* De-tile MM21 straight into the mapped bo. The bo has the R8 layout
 * (visible h rows, UV at stride*h), while the capture is tiled on the
 * aligned geometry, so Y and UV need independent pointers/strides and the
 * padding rows past h must not be written. */
/*
 * MM21 tiles are 16x32 bytes (luma) and 16x16 (chroma, 8x16 samples), laid out
 * row-major inside the tile, so every row of every tile block is a straight
 * 16-byte copy. Walk the destination row-major and let the 16-byte copy be a
 * single NEON load/store: the previous tile-major order wrote 16 bytes at a
 * time 1920 bytes apart, dirtying each 64-byte line three times.
 */
static void detile_planes_to(const uint8_t *ysrc, const uint8_t *usrc,
                             uint8_t *ydst, uint32_t ystride,
                             uint8_t *udst, uint32_t ustride,
                             uint32_t aw, uint32_t vis_h)
{
    uint32_t y_tc = aw / 16;
    uint32_t uvis = (vis_h + 1) / 2;

    for (uint32_t y = 0; y < vis_h; y++) {
        const uint8_t *ts = ysrc + (size_t)(y / 32) * y_tc * 512 +
                            (size_t)(y % 32) * 16;
        uint8_t *d = ydst + (size_t)y * ystride;

        for (uint32_t tc = 0; tc < y_tc; tc++)
            memcpy(d + (size_t)tc * 16, ts + (size_t)tc * 512, 16);
    }
    for (uint32_t y = 0; y < uvis; y++) {
        const uint8_t *ts = usrc + (size_t)(y / 16) * y_tc * 256 +
                            (size_t)(y % 16) * 16;
        uint8_t *d = udst + (size_t)y * ustride;

        for (uint32_t tc = 0; tc < y_tc; tc++)
            memcpy(d + (size_t)tc * 16, ts + (size_t)tc * 256, 16);
    }
}

/*
 * Asynchronous de-tile (V4L2SL_DETILE_ASYNC=1): the MM21 -> NV12 pass runs on
 * a worker thread so it overlaps the *next* decode instead of extending the
 * current frame's critical path. One job is in flight *per context*; the next
 * pull of that context waits for it, by which time it has had a whole decode
 * (~9 ms) to finish.
 *
 * The job reads the surface's capture slot and writes its GBM bo, so every
 * path that can free/reuse either must call v4l2sl_gbm_detile_wait(ctx) first:
 * context release, capture renegotiation, decode reset, surface destroy,
 * memfd refill, export, and vaSyncSurface.
 */
/* The job itself lives on the context (struct v4l2sl_async_detile): one job in
 * flight per stream, so a slow engine cannot serialise the others. */
static double g_last_wait_ms;
static double g_last_job_wall_ms;
static double g_last_job_cpu_ms;

double v4l2sl_gbm_detile_last_wait_ms(void) { return g_last_wait_ms; }
void v4l2sl_gbm_detile_last_job(double *wall_ms, double *cpu_ms)
{
    *wall_ms = g_last_job_wall_ms;
    *cpu_ms = g_last_job_cpu_ms;
}

/* Default ON: measured on MT8183 at 1080p60, overlapping the de-tile with
 * the next decode cuts dropped frames ~2.6x (34 -> 13 per 20 s on YouTube,
 * 15 -> 10 on a local clip). Opt out with V4L2SL_DETILE_ASYNC=0. */
int v4l2sl_gbm_detile_async_wanted(void)
{
    const char *e = getenv("V4L2SL_DETILE_ASYNC");

    if (e && (!strcmp(e, "0") || !strcmp(e, "off") || !strcmp(e, "no")))
        return 0;
    return 1;
}

static double ts_ms(const struct timespec *a, const struct timespec *b)
{
    return (b->tv_sec - a->tv_sec) * 1e3 + (b->tv_nsec - a->tv_nsec) / 1e6;
}

static void *async_detile_main(void *arg)
{
    struct v4l2sl_async_detile *a = arg;
    struct timespec w0, w1, c0, c1;
    double wall, cpu;

    clock_gettime(CLOCK_MONOTONIC, &w0);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
    if (a->hw) {
        /* Both hardware jobs take the decoder planes as dma-bufs and hand
         * back a complete bo; only the engine differs. a->ctx is non-NULL for
         * every hardware job: v4l2sl_gbm_detile_async_hw() rejects a NULL
         * context, and this is the only producer of a->hw jobs. */
        if (a->ctx->detile == V4L2SL_DETILE_GPU)
            a->ok = v4l2sl_gpu_detile(a->surf, a->y_fd, a->uv_fd) == 0;
        else
            a->ok = v4l2sl_mdp3_detile_prepared(a->ctx, a->surf,
                                                a->y_fd, a->uv_fd) == 0;
        if (!a->ok && a->rslot >= 0) {
            /* The engine failed (and has stopped writing the bo: the GPU
             * path waits for its own fence, MDP3 resets its streams). The
             * frame was already reported as decoded, so redo it here. */
            const uint8_t *ysrc, *usrc;
            int created = 0;

            if (v4l2sl_capture_cpu_view(a->ctx, a->rslot, &ysrc, &usrc,
                                        &created) == 0) {
                if (v4l2sl_gbm_surface_detile_mm21(a->surf, ysrc, usrc,
                                                   a->rstride, a->ralh) == 0) {
                    a->ok = 1;
                    a->recovered = 1;
                }
                /* Only drop the mapping if this recovery built it: the
                 * software de-tile path owns its mappings for the session. */
                if (created)
                    v4l2sl_capture_cpu_release(a->ctx, a->rslot);
            }
        }
    } else {
        a->ok = v4l2sl_gbm_surface_detile_mm21(a->surf, a->ysrc, a->usrc,
                                              a->aw, a->ah) == 0;
    }
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
    clock_gettime(CLOCK_MONOTONIC, &w1);
    wall = ts_ms(&w0, &w1);
    cpu = ts_ms(&c0, &c1);
    g_last_job_wall_ms = wall;
    g_last_job_cpu_ms = cpu;
    /* Both hardware jobs are expected to take ~5-10 ms; only flag real
     * outliers. */
    if (wall > (a->hw ? 20.0 : 8.0) && getenv("V4L2SL_PROFILE"))
        fprintf(stderr,
                "v4l2stateless: async de-tile slow: hw=%d wall=%.1f cpu=%.1f ms\n",
                a->hw, wall, cpu);
    return NULL;
}

/* Join this slot's in-flight job. The caller must hold a->lock; the worker
 * never takes it (it only touches the capture maps and the bo), so joining
 * under the lock is safe and makes wait+create atomic across contexts. */
static void detile_join_locked(struct v4l2sl_async_detile *a)
{
    struct timespec w0, w1;
    double waited;

    if (!a || !a->active)
        return;
    clock_gettime(CLOCK_MONOTONIC, &w0);
    pthread_join(a->th, NULL);
    clock_gettime(CLOCK_MONOTONIC, &w1);
    waited = ts_ms(&w0, &w1);
    if (waited > 0.5 && getenv("V4L2SL_PROFILE")) {
        static int n;
        if (n++ < 20)
            fprintf(stderr,
                    "v4l2stateless: async wait blocked %.1f ms\n", waited);
    }
    a->active = 0;
    if (a->ok && a->recovered) {
        static int n;

        a->recovered = 0;
        if (n++ < 5)
            fprintf(stderr,
                    "v4l2stateless: hardware de-tile job failed; frame redone "
                    "in software%s\n", n == 5 ? " (further reports muted)" : "");
    }
    if (!a->ok) {
        a->failed_once = 1;
        fprintf(stderr,
                "v4l2stateless: async de-tile job failed; using the "
                "synchronous path from now on\n");
    }
}

void v4l2sl_gbm_detile_wait(struct v4l2sl_context *ctx)
{
    struct v4l2sl_async_detile *a;

    if (!ctx)
        return;
    a = &ctx->adetile;
    pthread_mutex_lock(&a->lock);
    detile_join_locked(a);
    pthread_mutex_unlock(&a->lock);
}

/* True when this surface's context has a de-tile job reading its slot. */
int v4l2sl_gbm_detile_pending_for(const struct v4l2sl_surface *s)
{
    struct v4l2sl_async_detile *a;
    int r;

    if (!s || !s->capture_owner)
        return 0;
    a = &s->capture_owner->adetile;
    pthread_mutex_lock(&a->lock);
    r = a->active && a->surf == s;
    pthread_mutex_unlock(&a->lock);
    return r;
}

/* Hardware variant: the caller has already EXPBUF'd the decoder capture
 * planes, so the worker only talks to the MDP3 device (never the decoder fd).
 * Takes ownership of both fds. -1/-1 means the GL side already holds the
 * planes for this slot (cache hit) and no fds should be exported. */
int v4l2sl_gbm_detile_async_hw(struct v4l2sl_context *ctx,
                               struct v4l2sl_surface *s, int y_fd, int uv_fd,
                               int slot, uint32_t stride, uint32_t alh)
{
    int rc;

    struct v4l2sl_async_detile *a;

    if (!ctx || !s || !s->gbm_bo || (y_fd < 0) != (uv_fd < 0))
        return -1;
    a = &ctx->adetile;
    pthread_mutex_lock(&a->lock);
    detile_join_locked(a);
    if (a->failed_once) {
        pthread_mutex_unlock(&a->lock);
        return -1;
    }
    a->hw = 1;
    a->ctx = ctx;
    a->surf = s;
    a->y_fd = y_fd;
    a->uv_fd = uv_fd;
    a->rslot = slot;
    a->rstride = stride;
    a->ralh = alh;
    a->recovered = 0;
    a->ok = 0;
    rc = pthread_create(&a->th, NULL, async_detile_main, a);
    if (rc == 0) {
        a->active = 1;
    } else {
        a->hw = 0;
        a->rslot = -1;
    }
    pthread_mutex_unlock(&a->lock);
    return rc == 0 ? 0 : -1;
}

int v4l2sl_gbm_surface_detile_async(struct v4l2sl_surface *s,
                                    const uint8_t *ysrc, const uint8_t *usrc,
                                    uint32_t aw, uint32_t ah)
{
    int rc;

    struct v4l2sl_async_detile *a;

    if (!s || !s->gbm_bo || !ysrc || !usrc || !s->capture_owner)
        return -1;
    a = &s->capture_owner->adetile;
    pthread_mutex_lock(&a->lock);
    /* Finish this stream's previous job first: the caller's frame N-1 was
     * displayed with that bo, and its capture slot may be reused. */
    detile_join_locked(a);
    if (a->failed_once) {
        pthread_mutex_unlock(&a->lock);
        return -1;
    }
    a->hw = 0;
    a->ctx = s->capture_owner;
    a->y_fd = -1;
    a->uv_fd = -1;
    a->rslot = -1;
    a->recovered = 0;
    a->surf = s;
    a->ysrc = ysrc;
    a->usrc = usrc;
    a->aw = aw;
    a->ah = ah;
    a->ok = 0;
    rc = pthread_create(&a->th, NULL, async_detile_main, a);
    if (rc == 0)
        a->active = 1;
    pthread_mutex_unlock(&a->lock);
    return rc == 0 ? 0 : -1;
}

int v4l2sl_gbm_surface_detile_mm21(struct v4l2sl_surface *s,
                                   const uint8_t *ysrc, const uint8_t *usrc,
                                   uint32_t aw, uint32_t ah)
{
    /* A failed GPU job may still be writing this bo; the software fallback
     * rewrites it, so the two must not overlap. */
    v4l2sl_gpu_wait_surface(s);
    uint32_t w = s->width, h = s->height;
    void *map_data = NULL;
    uint32_t mstride = 0;
    uint8_t *dst;

    /* The capture plane is only ah rows tall. A surface taller than the
     * clamped capture (a client that ignored the advertised 2048x1088 limit,
     * which is never enforced at vaCreateSurfaces) must not read past it:
     * de-tile min(h,ah) rows and edge-fill the rest so the source read stays
     * in bounds and the bo has no unwritten band. */
    uint32_t vis_h = h < ah ? h : ah;
    uint32_t uv = s ? (s->bo_uv_rows ? s->bo_uv_rows : h) : h;

    if (!s || !s->gbm_bo || !ysrc || !usrc)
        return -1;
    pthread_mutex_lock(&g_bo_map_lock);
    dst = gbm_bo_map(s->gbm_bo, 0, 0, w, uv + (uv + 1) / 2,
                     GBM_BO_TRANSFER_WRITE, &mstride, &map_data);
    if (!dst) {
        pthread_mutex_unlock(&g_bo_map_lock);
        return -1;
    }
    if (aw > mstride) {   /* capture stride wider than the bo: fall back */
        gbm_bo_unmap(s->gbm_bo, map_data);
        pthread_mutex_unlock(&g_bo_map_lock);
        return -1;
    }
    detile_planes_to(ysrc, usrc, dst, mstride,
                     dst + (size_t)mstride * uv, mstride, aw, vis_h);
    if (vis_h > 0 && vis_h < h) {
        /* Surface taller than the capture: repeat the last valid row into
         * the extra luma/chroma rows (edge clamp) so nothing is left unwritten. */
        uint8_t *ylast = dst + (size_t)mstride * (vis_h - 1);
        for (uint32_t y = vis_h; y < h; y++)
            memcpy(dst + (size_t)mstride * y, ylast, w);
        uint8_t *ulast = dst + (size_t)mstride * (uv + (vis_h + 1) / 2 - 1);
        for (uint32_t y = (vis_h + 1) / 2; y < (h + 1) / 2; y++)
            memcpy(dst + (size_t)mstride * (uv + y), ulast, w);
    }
    gbm_bo_unmap(s->gbm_bo, map_data);
    pthread_mutex_unlock(&g_bo_map_lock);
    return 0;
}

void v4l2sl_gbm_surface_destroy(struct v4l2sl_surface *s)
{
    v4l2sl_gbm_detile_wait(s->capture_owner);
    /* The bo is about to be freed; the GPU may still be rendering into it. */
    v4l2sl_gpu_wait_surface(s);
    v4l2sl_gpu_forget_surface(s);
    if (s && s->gbm_bo) {
        v4l2sl_gpu_forget_bo(s->gbm_bo);
        gbm_bo_destroy(s->gbm_bo);
        s->gbm_bo = NULL;
    }
}

VAStatus v4l2sl_surface_fill_prime_gbm(const struct v4l2sl_surface *surf,
                                       uint32_t flags, void *descriptor)
{
    VADRMPRIMESurfaceDescriptor *desc = descriptor;
    uint32_t h, pitch;
    int fd;

    if (!surf || !surf->gbm_bo || !descriptor)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    v4l2sl_gbm_detile_wait(surf->capture_owner);   /* hand out a finished bo */
    fd = gbm_bo_get_fd_for_plane(surf->gbm_bo, 0);
    if (fd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;

    h = surf->height;
    pitch = surf->gbm_stride;
    {
        uint32_t uv = surf->bo_uv_rows ? surf->bo_uv_rows : h;

        v4l2sl_fill_prime_layers(desc, fd, pitch * (uv + (uv + 1) / 2),
                                 gbm_bo_get_modifier(surf->gbm_bo),
                                 VA_FOURCC_NV12, surf->width, h, pitch, uv,
                                 DRM_FORMAT_NV12, DRM_FORMAT_R8,
                                 DRM_FORMAT_GR88, flags);
    }
    return VA_STATUS_SUCCESS;
}
