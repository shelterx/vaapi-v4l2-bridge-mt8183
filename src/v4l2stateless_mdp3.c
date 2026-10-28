/*
 * v4l2stateless_mdp3.c — MDP3 m2m hardware de-tile (MM21 -> NV12).
 *
 * The MT8183 H.264 decoder (/dev/video5, mtk-vcodec) emits only tiled MM21.
 * MDP3 (/dev/videoN, driver "mdp3") de-tiles MM21 -> NV12 in a V4L2 M2M job.
 * This module drives that job with ZERO-COPY input: the decoder's MM21
 * capture planes are exported as dma-bufs (VIDIOC_EXPBUF) and imported into
 * the m2m OUTPUT queue, so the CPU never reads the tiled CMA. The m2m
 * CAPTURE queue is driver-allocated linear NV12; it is mmap'd and copied
 * into the surface's GBM bo (one linear pass, far cheaper than the software
 * de-tile's tile-scattered reads).
 *
 * Both facts were verified on hardware: the decoder emits two MM21 planes
 * (Y + UV, separate CMA blocks) and vb2's dma-contig import accepts their
 * exported dma-bufs. The m2m device is single-client, so this module is a
 * process-wide singleton: the first context to init it owns it; any other
 * context falls back to the software de-tile.
 *
 * Fallback: if init fails (no node, queues, streaming, or a format/size
 * mismatch) or a frame fails MDP3_FAIL_LIMIT times in a row, the HW path is
 * disabled for the session and the caller uses v4l2sl_mm21_detile instead.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <gbm.h>

#include "v4l2stateless.h"

#ifndef V4L2_PIX_FMT_MM21
#define V4L2_PIX_FMT_MM21 v4l2_fourcc('M', 'M', '2', '1')
#endif

#define MDP3_NBUFS         4

/* Stage timing (V4L2SL_PROFILE): shows how much of a job is dma-buf import
 * (QBUF) versus the wait for the hardware. */
static double g_stage[5];
static unsigned g_stage_n;

static int mdp3_time_stages(void)
{
    static int on = -1;

    if (on < 0)
        on = getenv("V4L2SL_PROFILE") ? 1 : 0;
    return on;
}

static double mdp3_now_ms(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static void mdp3_stage(int i, double ms)
{
    if (mdp3_time_stages())
        g_stage[i] += ms;
}

static void mdp3_stage_report(void)
{
    if (mdp3_time_stages() && ++g_stage_n % 120 == 0) {
        fprintf(stderr,
                "v4l2stateless: mdp3 stages avg ms: qcap=%.2f qout=%.2f "
                "poll=%.2f dcap=%.2f dout=%.2f\n",
                g_stage[0] / 120, g_stage[1] / 120, g_stage[2] / 120,
                g_stage[3] / 120, g_stage[4] / 120);
        for (int i = 0; i < 5; i++)
            g_stage[i] = 0;
    }
}
#define MDP3_DQ_TIMEOUT_MS 2000
#define MDP3_FAIL_LIMIT    3   /* consecutive failures -> disable for session */

/* Process-wide singleton: the m2m node is single-client. */
static struct v4l2sl_mdp3 g_mdp3 = { .fd = -1 };
static struct v4l2sl_context *g_mdp3_owner;
/* Two decode contexts can pull their first frame concurrently. Without this
 * lock both would open the single m2m device and reconfigure it under each
 * other, which corrupts the stream that initialised first. */
static pthread_mutex_t g_mdp3_lock = PTHREAD_MUTEX_INITIALIZER;

static int mdp3_reqbufs(int fd, uint32_t type, int count, uint32_t memory);

static int mdp3_log(void)
{
    const char *e = getenv("V4L2SL_DEBUG");

    return e && e[0] && e[0] != '0';
}

/* MDP3 is opt-in: with cacheable (non-coherent) capture buffers the software
 * de-tile is ~1.8 ms/frame at 1080p, while the m2m job is ~10 ms plus a
 * readback copy from uncached CMA — the HW path is ~10x slower per frame.
 * True when the caller asked for the MDP3 de-tile (V4L2SL_DETILE=hw),
 * regardless of whether init has run yet. */
int v4l2sl_mdp3_requested(void)
{
    return v4l2sl_detile_mode() == V4L2SL_DETILE_MDP3;
}

static int mdp3_stream(int fd, uint32_t type, int on)
{
    uint32_t t = type;   /* STREAMON/OFF take a pointer to a __u32 */

    return ioctl(fd, on ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &t);
}

/* poll() for a job completion, retrying on EINTR with whatever is left of the
 * budget. A signal is not a job failure, and treating it as one leaves both
 * queues holding buffers that the next frame cannot re-queue. */
static int mdp3_poll_ms(int fd, int timeout_ms)
{
    struct pollfd pfd;
    double deadline = mdp3_now_ms() + timeout_ms;

    pfd.fd = fd;
    pfd.events = POLLIN;
    for (;;) {
        double left = deadline - mdp3_now_ms();
        int r;

        if (left <= 0)
            return 0;
        r = poll(&pfd, 1, (int)left + 1);
        if (r >= 0)
            return r;
        if (errno != EINTR)
            return -1;
    }
}

/* Stop the engine and drop whatever is queued. Needed when a job is abandoned
 * (timeout, or a DQBUF that did not complete): the m2m keeps writing the
 * CAPTURE target - which is the surface's display bo - and the software
 * fallback would otherwise write that same buffer concurrently. STREAMOFF
 * aborts the job and returns every buffer to userspace; stream both queues
 * again so the next frame can start cleanly. */
static void mdp3_reset_streams(void)
{
    if (g_mdp3.fd < 0)
        return;
    if (mdp3_log())
        fprintf(stderr, "v4l2stateless: mdp3: resetting streams after a "
                        "failed job\n");
    mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0);
    mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0);
    mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 1);
    mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 1);
}

/* Caller must hold g_mdp3_lock. */
/* Tear the queues down and give the device up. The fd is deliberately NOT
 * closed here: this runs on the async worker, while the decode thread may
 * already hold the old descriptor number and be about to ioctl() it - if the
 * number were recycled by any other open() in the meantime, that ioctl would
 * land on an unrelated fd. The descriptor stays open (and unusable: out_bufs
 * is 0) until mdp3_init_locked() reclaims it, or until fini. */
static void mdp3_release_locked(void)
{
    if (g_mdp3.fd < 0)
        return;
    if (g_mdp3.streaming) {
        mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0);
        mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0);
        g_mdp3.streaming = 0;
    }
    mdp3_reqbufs(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0, V4L2_MEMORY_MMAP);
    mdp3_reqbufs(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0, V4L2_MEMORY_DMABUF);
    g_mdp3.out_bufs = 0;
    g_mdp3.cap_bufs = 0;
    g_mdp3_owner = NULL;
}

/* Decoder plane sizes without requiring the CPU mapping (the hardware path
 * deliberately does not map the capture slot). */
static uint32_t src_y_size(struct v4l2sl_context *ctx)
{
    if (ctx->capture_buf_size)
        return ctx->capture_buf_size;
    if (ctx->cap_stride && ctx->cap_height)
        return ctx->cap_stride * ctx->cap_height;
    return ctx->cap_sizeimage;
}

static uint32_t src_uv_size(struct v4l2sl_context *ctx)
{
    uint32_t y;

    if (ctx->capture_uv_size)
        return ctx->capture_uv_size;
    y = src_y_size(ctx);
    return y / 2;
}

/* Count a HW failure; give up on the HW path after MDP3_FAIL_LIMIT in a row
 * (the m2m is wedged or the dma-buf import stopped working). */
static int mdp3_fail(struct v4l2sl_context *ctx, const char *what, int err)
{
    ctx->mdp3.fail_count++;
    if (ctx->mdp3.fail_count < MDP3_FAIL_LIMIT)
        return -1;
    fprintf(stderr,
            "v4l2stateless: de-tile HW->SW (%d consecutive mdp3 failures: %s%s%s)\n",
            MDP3_FAIL_LIMIT, what, err ? ": " : "", err ? strerror(err) : "");
    ctx->mdp3.fail_count = 0;
    ctx->mdp3.active = 0;
    ctx->mdp3.disabled = 1;
    pthread_mutex_lock(&g_mdp3_lock);
    mdp3_release_locked();
    pthread_mutex_unlock(&g_mdp3_lock);
    return -1;
}

/* Auto-detect the mtk-mdp3 m2m node. /dev/videoN numbering is not stable
 * across boots, so scan for the driver by name (matches the standalone tool). */
static int mdp3_open_node(char *path_out, size_t sz)
{
    char path[32];
    int n;

    for (n = 0; n < 16; n++) {
        struct v4l2_capability cap;

        snprintf(path, sizeof(path), "/dev/video%d", n);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
            strstr((const char *)cap.driver, "mdp3")) {
            snprintf(path_out, sz, "%s", path);
            return fd;
        }
        close(fd);
    }
    return -1;
}

static int mdp3_s_fmt(int fd, uint32_t type, uint32_t fourcc,
                      uint32_t w, uint32_t h, struct v4l2_pix_format_mplane *out)
{
    struct v4l2_format fmt = { 0 };

    fmt.type = type;
    fmt.fmt.pix_mp.width = w;
    fmt.fmt.pix_mp.height = h;
    fmt.fmt.pix_mp.pixelformat = fourcc;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
        return -1;
    *out = fmt.fmt.pix_mp;
    return 0;
}

static int mdp3_reqbufs(int fd, uint32_t type, int count, uint32_t memory)
{
    struct v4l2_requestbuffers rb = { 0 };

    rb.count = (unsigned)count;
    rb.type = type;
    rb.memory = memory;
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
        return -1;
    return (int)rb.count;
}

static int mdp3_init_locked(struct v4l2sl_context *ctx, int width, int height)
{
    char path[32] = "?";
    struct v4l2_pix_format_mplane omp, cmp;
    uint32_t y_size, uv_size;

    if (!ctx)
        return -1;
    if (g_mdp3.fd >= 0) {
        /* Already initialized: reuse only for its owner. */
        if (g_mdp3_owner == ctx) {
            ctx->mdp3.active = 1;
            ctx->mdp3.disabled = 0;
            return 0;
        }
        if (!g_mdp3_owner) {
            /* Released by the failure policy. Nothing can be queued on it, so
             * it is safe to close here and open a fresh device below. */
            close(g_mdp3.fd);
            g_mdp3.fd = -1;
        } else {
            ctx->mdp3.disabled = 1;
            fprintf(stderr,
                    "v4l2stateless: de-tile path = SW (mdp3 in use by another context)\n");
            return -1;
        }
    }
    if (!v4l2sl_mdp3_requested()) {
        ctx->mdp3.disabled = 1;
        if (v4l2sl_detile_mode() == V4L2SL_DETILE_GPU)
            fprintf(stderr,
                    "v4l2stateless: de-tile path = SW (MDP3 not selected)\n");
        else
            fprintf(stderr,
                    "v4l2stateless: de-tile path = SW (cacheable capture; set "
                    "V4L2SL_DETILE=hw for MDP3, =gpu for the shader)\n");
        return -1;
    }

    /* The m2m needs the decoder's real capture geometry (aligned height). */
    y_size = src_y_size(ctx);
    uv_size = src_uv_size(ctx);
    if (width <= 0 || height <= 0 || !y_size || !uv_size) {
        ctx->mdp3.disabled = 1;
        return -1;
    }

    g_mdp3.fd = mdp3_open_node(path, sizeof(path));
    if (g_mdp3.fd < 0) {
        ctx->mdp3.disabled = 1;
        fprintf(stderr,
                "v4l2stateless: de-tile path = SW (no mtk-mdp3 m2m node)\n");
        return -1;
    }

    /* OUTPUT = MM21 (device input), CAPTURE = NV12 (device output). */
    if (mdp3_s_fmt(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                   V4L2_PIX_FMT_MM21, (uint32_t)width, (uint32_t)height,
                   &omp) < 0) {
        fprintf(stderr, "v4l2stateless: mdp3 S_FMT out (MM21) failed: %s\n",
                strerror(errno));
        goto fail;
    }
    if (mdp3_s_fmt(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                   V4L2_PIX_FMT_NV12, omp.width, omp.height, &cmp) < 0) {
        fprintf(stderr, "v4l2stateless: mdp3 S_FMT cap (NV12) failed: %s\n",
                strerror(errno));
        goto fail;
    }
    if (cmp.num_planes != 1 ||
        omp.plane_fmt[0].sizeimage > y_size ||
        omp.plane_fmt[1].sizeimage > uv_size) {
        fprintf(stderr,
                "v4l2stateless: mdp3 geometry mismatch (need %u/%u, have %u/%u)\n",
                omp.plane_fmt[0].sizeimage, omp.plane_fmt[1].sizeimage,
                y_size, uv_size);
        goto fail;
    }

    /* OUTPUT: dma-buf slots (the decoder's exported MM21 planes are imported
     * at QBUF). CAPTURE: dma-buf slots too — the surface's GBM bo is imported
     * each frame, so MDP3 writes the final linear NV12 directly and there is
     * no readback/copy at all. */
    g_mdp3.out_bufs = mdp3_reqbufs(g_mdp3.fd,
                                   V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                                   MDP3_NBUFS, V4L2_MEMORY_DMABUF);
    if (g_mdp3.out_bufs < 0) {
        fprintf(stderr, "v4l2stateless: mdp3 REQBUFS out failed: %s\n",
                strerror(errno));
        goto fail;
    }
    g_mdp3.cap_bufs = mdp3_reqbufs(g_mdp3.fd,
                                   V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                   MDP3_NBUFS, V4L2_MEMORY_DMABUF);
    if (g_mdp3.cap_bufs < 0) {
        fprintf(stderr, "v4l2stateless: mdp3 REQBUFS cap failed: %s\n",
                strerror(errno));
        goto fail;
    }
    if (g_mdp3.cap_bufs > V4L2SL_MAX_CAPTURE_BUFS)
        g_mdp3.cap_bufs = V4L2SL_MAX_CAPTURE_BUFS;

    g_mdp3.cap_stride = cmp.plane_fmt[0].bytesperline;
    g_mdp3.cap_width = cmp.width;
    g_mdp3.cap_height = cmp.height;
    g_mdp3.cap_size = cmp.plane_fmt[0].sizeimage;

    /* The m2m framework starts a job only when BOTH queues are streaming
     * (mtk-mdp3 does not set ignore_cap_streaming). */
    if (mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 1) < 0 ||
        mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 1) < 0) {
        fprintf(stderr, "v4l2stateless: mdp3 STREAMON failed: %s\n",
                strerror(errno));
        goto fail;
    }
    g_mdp3.streaming = 1;
    g_mdp3.fail_count = 0;
    g_mdp3.disabled = 0;
    g_mdp3_owner = ctx;

    ctx->mdp3.active = 1;
    ctx->mdp3.disabled = 0;

    fprintf(stderr,
            "v4l2stateless: de-tile path = HW (MDP3 m2m %s, %ux%u, %d bufs)\n",
            path, g_mdp3.cap_width, g_mdp3.cap_height, g_mdp3.cap_bufs);
    return 0;

fail:
    mdp3_release_locked();
    ctx->mdp3.active = 0;
    ctx->mdp3.disabled = 1;
    fprintf(stderr, "v4l2stateless: de-tile path = SW (mdp3 init failed)\n");
    return -1;
}

/* Run one MDP3 job with the decoder capture planes already exported. Used by
 * both the synchronous path and the async worker; takes ownership of y_fd and
 * uv_fd (closed once vb2 has referenced them). */
int v4l2sl_mdp3_detile_prepared(struct v4l2sl_context *ctx,
                                struct v4l2sl_surface *surf,
                                int y_fd, int uv_fd)
{
    struct v4l2_buffer b = { 0 };
    struct v4l2_plane planes[2] = { 0 };
    uint32_t y_size, uv_size, bo_rows;
    int cap_done, bo_fd, err;

    if (!ctx || !surf || !surf->gbm_bo || ctx->mdp3.disabled ||
        g_mdp3.fd < 0 || !g_mdp3.out_bufs || g_mdp3_owner != ctx ||
        y_fd < 0 || uv_fd < 0) {
        if (y_fd >= 0)
            close(y_fd);
        if (uv_fd >= 0)
            close(uv_fd);
        return -1;
    }

    y_size = src_y_size(ctx);
    uv_size = src_uv_size(ctx);
    bo_rows = surf->bo_uv_rows;
    if (!y_size || !uv_size || !bo_rows) {
        close(y_fd);
        close(uv_fd);
        return -1;
    }
    /* The m2m writes NV12 at its own bytesperline and the display bo is
     * sampled at its own pitch, so the two must be equal: a wider bo pitch
     * would displace every row by the difference (the picture shears). This
     * is fixed per stream, so stop trying instead of failing every frame. */
    if (surf->gbm_stride != g_mdp3.cap_stride) {
        close(y_fd);
        close(uv_fd);
        if (!ctx->mdp3.disabled) {
            ctx->mdp3.disabled = 1;
            fprintf(stderr,
                    "v4l2stateless: de-tile path = SW (bo pitch %u != mdp3 "
                    "pitch %u)\n", surf->gbm_stride, g_mdp3.cap_stride);
        }
        return -1;
    }
    if ((uint64_t)surf->gbm_stride * (bo_rows + (bo_rows + 1) / 2) <
        g_mdp3.cap_size) {
        close(y_fd);
        close(uv_fd);
        return -1;
    }

    /* Zero-copy output: import the surface's GBM bo as the CAPTURE buffer,
     * so the NV12 result is written straight into what Chromium samples. */
    bo_fd = gbm_bo_get_fd(surf->gbm_bo);
    if (bo_fd < 0) {
        close(y_fd);
        close(uv_fd);
        return mdp3_fail(ctx, "gbm_bo_get_fd", errno);
    }

    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.index = 0;
    b.memory = V4L2_MEMORY_DMABUF;
    b.length = 1;
    b.m.planes = planes;
    planes[0].m.fd = bo_fd;
    planes[0].length = g_mdp3.cap_size;
    {
        double t0 = mdp3_now_ms();

        err = ioctl(g_mdp3.fd, VIDIOC_QBUF, &b) < 0 ? errno : 0;
        mdp3_stage(0, mdp3_now_ms() - t0);
    }
    close(bo_fd);
    if (err) {
        close(y_fd);
        close(uv_fd);
        return mdp3_fail(ctx, "QBUF cap bo", err);
    }

    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.index = 0;
    b.memory = V4L2_MEMORY_DMABUF;
    b.length = 2;
    b.m.planes = planes;
    planes[0].m.fd = y_fd;
    planes[0].length = y_size;
    planes[0].bytesused = y_size;
    planes[1].m.fd = uv_fd;
    planes[1].length = uv_size;
    planes[1].bytesused = uv_size;
    {
        double t0 = mdp3_now_ms();

        err = ioctl(g_mdp3.fd, VIDIOC_QBUF, &b) < 0 ? errno : 0;
        mdp3_stage(1, mdp3_now_ms() - t0);
    }
    /* vb2 takes its own dma-buf reference at QBUF; the fds can go now. */
    close(y_fd);
    close(uv_fd);
    if (err)
        return mdp3_fail(ctx, "QBUF in", err);

    /* The driver pairs the queued OUTPUT with the queued CAPTURE and runs the
     * job; DQBUF returns the completed CAPTURE, already in the bo. */
    {
        double t0 = mdp3_now_ms();
        int pr = mdp3_poll_ms(g_mdp3.fd, MDP3_DQ_TIMEOUT_MS);

        mdp3_stage(2, mdp3_now_ms() - t0);
        if (pr <= 0) {
            /* Real timeout or error: the job may still be running, so stop it
             * before the software de-tile writes the same bo. */
            mdp3_reset_streams();
            return mdp3_fail(ctx, pr == 0 ? "poll timeout" : "poll error",
                             pr == 0 ? 0 : errno);
        }
    }

    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_DMABUF;
    b.length = 1;
    b.m.planes = planes;
    {
        double t0 = mdp3_now_ms();

        if (ioctl(g_mdp3.fd, VIDIOC_DQBUF, &b) < 0) {
            mdp3_reset_streams();
            return mdp3_fail(ctx, "DQBUF cap", errno);
        }
        mdp3_stage(3, mdp3_now_ms() - t0);
    }
    cap_done = (int)b.index;
    if (cap_done < 0 || cap_done >= g_mdp3.cap_bufs) {
        mdp3_reset_streams();
        return mdp3_fail(ctx, "bad capture index", 0);
    }

    /* Release the (now completed) OUTPUT buffer so its slot can be reused. */
    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.memory = V4L2_MEMORY_DMABUF;
    b.length = 2;
    b.m.planes = planes;
    {
        double t0 = mdp3_now_ms();

        if (ioctl(g_mdp3.fd, VIDIOC_DQBUF, &b) < 0) {
            mdp3_reset_streams();
            return mdp3_fail(ctx, "DQBUF in", errno);
        }
        mdp3_stage(4, mdp3_now_ms() - t0);
    }

    mdp3_stage_report();
    ctx->mdp3.fail_count = 0;
    if (mdp3_log())
        fprintf(stderr,
                "v4l2stateless: mdp3 detile cap=%d (HW zero-copy)\n",
                cap_done);
    return 0;
}

/* Synchronous convenience path: export the decoder planes, then run the job
 * on the calling thread. */
int v4l2sl_mdp3_init(struct v4l2sl_context *ctx, int width, int height)
{
    int ret;

    pthread_mutex_lock(&g_mdp3_lock);
    ret = mdp3_init_locked(ctx, width, height);
    pthread_mutex_unlock(&g_mdp3_lock);
    return ret;
}

int v4l2sl_mdp3_detile(struct v4l2sl_context *ctx, int buf_index,
                       struct v4l2sl_surface *surf)
{
    int y_fd, uv_fd, err;

    if (!ctx || !surf || ctx->mdp3.disabled || g_mdp3.fd < 0 ||
        g_mdp3_owner != ctx || buf_index < 0)
        return -1;

    y_fd = v4l2sl_capture_expbuf_plane(ctx, buf_index, 0);
    if (y_fd < 0)
        return mdp3_fail(ctx, "EXPBUF Y", errno);
    uv_fd = v4l2sl_capture_expbuf_plane(ctx, buf_index, 1);
    if (uv_fd < 0) {
        err = errno;
        close(y_fd);
        return mdp3_fail(ctx, "EXPBUF UV", err);
    }
    return v4l2sl_mdp3_detile_prepared(ctx, surf, y_fd, uv_fd);
}

void v4l2sl_mdp3_fini(struct v4l2sl_context *ctx)
{
    pthread_mutex_lock(&g_mdp3_lock);
    if (ctx && g_mdp3.fd >= 0 && g_mdp3_owner == ctx) {
        mdp3_release_locked();
        /* Safe here and only here: the caller has already joined the async
         * worker (v4l2sl_gbm_detile_wait) and reaped the GPU fences, so no
         * thread can be holding this descriptor number. release_locked()
         * deliberately leaves it open. */
        close(g_mdp3.fd);
        g_mdp3.fd = -1;
        ctx->mdp3.active = 0;
        ctx->mdp3.disabled = 1;
    }
    pthread_mutex_unlock(&g_mdp3_lock);
}
