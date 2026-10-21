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
    uint8_t *dst;
    void *map_data = NULL;
    uint32_t mstride = 0;

    if (!s || !s->gbm_bo || !src || !src_stride)
        return -1;
    dst = gbm_bo_map(s->gbm_bo, 0, 0, w, h + (h + 1) / 2,
                     GBM_BO_TRANSFER_WRITE, &mstride, &map_data);
    if (!dst) {
        fprintf(stderr, "v4l2stateless: gbm: bo map failed\n");
        return -1;
    }
    copy_plane(dst, mstride, src, src_stride, w, h);
    copy_plane(dst + (size_t)mstride * h, mstride,
               (const uint8_t *)src + (size_t)src_stride * src_alh,
               src_stride, w + (w & 1), (h + 1) / 2);
    gbm_bo_unmap(s->gbm_bo, map_data);
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
    v4l2sl_gbm_detile_wait();   /* bo may still be written by a pending job */
    if (s->memfd_fd < 0 || !s->stride || !s->aligned_h)
        return -1;
    w = s->width;
    h = s->height;
    uvw = w + (w & 1);
    sz = v4l2sl_capture_plane_size(V4L2_PIX_FMT_NV12, s->stride, s->aligned_h);
    if (v4l2sl_surface_grow_memfd(s, sz) < 0)
        return -1;
    bo_data = gbm_bo_map(s->gbm_bo, 0, 0, w, h + (h + 1) / 2,
                         GBM_BO_TRANSFER_READ, &bo_stride, &bo_map);
    if (!bo_data)
        return -1;
    m = v4l2sl_surface_map_memfd(s, sz);
    if (!m) {
        gbm_bo_unmap(s->gbm_bo, bo_map);
        return -1;
    }
    dst = m;
    for (uint32_t y = 0; y < h; y++)
        memcpy(dst + (size_t)s->stride * y,
               bo_data + (size_t)bo_stride * y, w);
    dst = (uint8_t *)m + (size_t)s->stride * s->aligned_h;
    for (uint32_t y = 0; y < (h + 1) / 2; y++)
        memcpy(dst + (size_t)s->stride * y,
               bo_data + (size_t)bo_stride * (h + y), uvw);
    gbm_bo_unmap(s->gbm_bo, bo_map);
    s->last_writer = V4L2SL_WRITER_MEMFD;
    return 0;
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
    rows = s->height + (s->height + 1) / 2;
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
static void detile_planes_to(const uint8_t *ysrc, const uint8_t *usrc,
                             uint8_t *ydst, uint32_t ystride,
                             uint8_t *udst, uint32_t ustride,
                             uint32_t aw, uint32_t ah, uint32_t vis_h)
{
    uint32_t y_tc = aw / 16, y_tr = ah / 32;
    uint32_t uvis = (vis_h + 1) / 2;

    for (uint32_t tr = 0; tr < y_tr; tr++)
        for (uint32_t tc = 0; tc < y_tc; tc++) {
            const uint8_t *ts = ysrc + (size_t)(tr * y_tc + tc) * 512;

            for (uint32_t r = 0; r < 32; r++) {
                uint32_t y = tr * 32 + r;

                if (y >= vis_h)
                    break;
                for (uint32_t c = 0; c < 16; c++)
                    ydst[(size_t)y * ystride + (size_t)tc * 16 + c] =
                        ts[r * 16 + c];
            }
        }
    for (uint32_t tr = 0; tr < y_tr; tr++)
        for (uint32_t tc = 0; tc < y_tc; tc++) {
            const uint8_t *ts = usrc + (size_t)(tr * y_tc + tc) * 256;

            for (uint32_t r = 0; r < 16; r++) {
                uint32_t y = tr * 16 + r;

                if (y >= uvis)
                    break;
                for (uint32_t c = 0; c < 16; c++)
                    udst[(size_t)y * ustride + (size_t)tc * 16 + c] =
                        ts[r * 16 + c];
            }
        }
}

void *v4l2sl_gbm_surface_map(struct v4l2sl_surface *s, uint32_t w, uint32_t h,
                            uint32_t *out_stride, void **out_data)
{
    uint32_t mstride = 0;
    void *map_data = NULL;
    void *dst;

    if (!s || !s->gbm_bo)
        return NULL;
    dst = gbm_bo_map(s->gbm_bo, 0, 0, w, h + (h + 1) / 2,
                     GBM_BO_TRANSFER_WRITE, &mstride, &map_data);
    if (dst) {
        if (out_stride)
            *out_stride = mstride;
        if (out_data)
            *out_data = map_data;
    }
    return dst;
}

void v4l2sl_gbm_surface_unmap(struct v4l2sl_surface *s, void *ptr, void *data)
{
    (void)ptr;
    if (s && s->gbm_bo && data)
        gbm_bo_unmap(s->gbm_bo, data);
}

/*
 * Asynchronous de-tile (V4L2SL_DETILE_ASYNC=1): the MM21 -> NV12 pass runs on
 * a worker thread so it overlaps the *next* decode instead of extending the
 * current frame's critical path. Only one job is in flight (the decoder is
 * serialized); the next pull waits for it, by which time it has had a whole
 * decode (~9 ms) to finish.
 *
 * The job reads the surface's capture slot and writes its GBM bo, so every
 * path that can free/reuse either must call v4l2sl_gbm_detile_wait() first:
 * context release, capture renegotiation, decode reset, surface destroy,
 * memfd refill, export, and vaSyncSurface.
 */
struct async_detile {
    pthread_t th;
    int active;
    int ok;
    int failed_once;
    struct v4l2sl_surface *surf;
    const uint8_t *ysrc;
    const uint8_t *usrc;
    uint32_t aw;
    uint32_t ah;
};

static struct async_detile g_adetile;
static pthread_mutex_t g_adetile_lock = PTHREAD_MUTEX_INITIALIZER;

int v4l2sl_gbm_detile_async_wanted(void)
{
    const char *e = getenv("V4L2SL_DETILE_ASYNC");

    return e && e[0] && e[0] != '0' && strcmp(e, "off") && strcmp(e, "no");
}

static double ts_ms(const struct timespec *a, const struct timespec *b)
{
    return (b->tv_sec - a->tv_sec) * 1e3 + (b->tv_nsec - a->tv_nsec) / 1e6;
}

static void *async_detile_main(void *arg)
{
    struct async_detile *a = arg;
    struct timespec w0, w1, c0, c1;
    double wall, cpu;

    clock_gettime(CLOCK_MONOTONIC, &w0);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
    a->ok = v4l2sl_gbm_surface_detile_mm21(a->surf, a->ysrc, a->usrc,
                                           a->aw, a->ah) == 0;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
    clock_gettime(CLOCK_MONOTONIC, &w1);
    wall = ts_ms(&w0, &w1);
    cpu = ts_ms(&c0, &c1);
    if (wall > 8.0 && getenv("V4L2SL_PROFILE"))
        fprintf(stderr,
                "v4l2stateless: async de-tile slow: wall=%.1f cpu=%.1f ms\n",
                wall, cpu);
    return NULL;
}

void v4l2sl_gbm_detile_wait(void)
{
    struct timespec w0, w1;
    double waited;

    pthread_mutex_lock(&g_adetile_lock);
    if (g_adetile.active) {
        clock_gettime(CLOCK_MONOTONIC, &w0);
        pthread_join(g_adetile.th, NULL);
        clock_gettime(CLOCK_MONOTONIC, &w1);
        waited = ts_ms(&w0, &w1);
        if (waited > 0.5 && getenv("V4L2SL_PROFILE")) {
            static int n;
            if (n++ < 20)
                fprintf(stderr,
                        "v4l2stateless: async wait blocked %.1f ms\n", waited);
        }
        g_adetile.active = 0;
        if (!g_adetile.ok) {
            g_adetile.failed_once = 1;
            fprintf(stderr,
                    "v4l2stateless: async de-tile job failed; using the "
                    "synchronous path from now on\n");
        }
    }
    pthread_mutex_unlock(&g_adetile_lock);
}

int v4l2sl_gbm_surface_detile_async(struct v4l2sl_surface *s,
                                    const uint8_t *ysrc, const uint8_t *usrc,
                                    uint32_t aw, uint32_t ah)
{
    int rc;

    if (!s || !s->gbm_bo || !ysrc || !usrc)
        return -1;
    /* Finish the previous job first: the caller's frame N-1 was displayed
     * with that bo, and its capture slot may be reused. */
    v4l2sl_gbm_detile_wait();

    pthread_mutex_lock(&g_adetile_lock);
    if (g_adetile.failed_once) {
        pthread_mutex_unlock(&g_adetile_lock);
        return -1;
    }
    g_adetile.surf = s;
    g_adetile.ysrc = ysrc;
    g_adetile.usrc = usrc;
    g_adetile.aw = aw;
    g_adetile.ah = ah;
    g_adetile.ok = 0;
    rc = pthread_create(&g_adetile.th, NULL, async_detile_main, &g_adetile);
    if (rc == 0)
        g_adetile.active = 1;
    pthread_mutex_unlock(&g_adetile_lock);
    return rc == 0 ? 0 : -1;
}

int v4l2sl_gbm_surface_detile_mm21(struct v4l2sl_surface *s,
                                   const uint8_t *ysrc, const uint8_t *usrc,
                                   uint32_t aw, uint32_t ah)
{
    uint32_t w = s->width, h = s->height;
    void *map_data = NULL;
    uint32_t mstride = 0;
    uint8_t *dst;

    if (!s || !s->gbm_bo || !ysrc || !usrc)
        return -1;
    dst = gbm_bo_map(s->gbm_bo, 0, 0, w, h + (h + 1) / 2,
                     GBM_BO_TRANSFER_WRITE, &mstride, &map_data);
    if (!dst)
        return -1;
    if (aw > mstride) {   /* capture stride wider than the bo: fall back */
        gbm_bo_unmap(s->gbm_bo, map_data);
        return -1;
    }
    detile_planes_to(ysrc, usrc, dst, mstride,
                     dst + (size_t)mstride * h, mstride, aw, ah, h);
    gbm_bo_unmap(s->gbm_bo, map_data);
    return 0;
}

void v4l2sl_gbm_surface_destroy(struct v4l2sl_surface *s)
{
    v4l2sl_gbm_detile_wait();
    if (s && s->gbm_bo) {
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
    v4l2sl_gbm_detile_wait();   /* hand out the bo only when fully written */
    fd = gbm_bo_get_fd_for_plane(surf->gbm_bo, 0);
    if (fd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;

    h = surf->height;
    pitch = surf->gbm_stride;
    v4l2sl_fill_prime_layers(desc, fd, pitch * (h + (h + 1) / 2),
                             gbm_bo_get_modifier(surf->gbm_bo),
                             VA_FOURCC_NV12, surf->width, h, pitch, h,
                             DRM_FORMAT_NV12, DRM_FORMAT_R8, DRM_FORMAT_GR88,
                             flags);
    return VA_STATUS_SUCCESS;
}
