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

#include "v4l2stateless.h"

#ifndef V4L2_PIX_FMT_MM21
#define V4L2_PIX_FMT_MM21 v4l2_fourcc('M', 'M', '2', '1')
#endif

#define MDP3_NBUFS         4
#define MDP3_DQ_TIMEOUT_MS 2000
#define MDP3_FAIL_LIMIT    3   /* consecutive failures -> disable for session */

/* Process-wide singleton: the m2m node is single-client. */
static struct v4l2sl_mdp3 g_mdp3 = { .fd = -1 };
static struct v4l2sl_context *g_mdp3_owner;

static int mdp3_reqbufs(int fd, uint32_t type, int count, uint32_t memory);

static int mdp3_log(void)
{
    const char *e = getenv("V4L2SL_DEBUG");

    return e && e[0] && e[0] != '0';
}

static int mdp3_forced_off(void)
{
    const char *e = getenv("V4L2SL_DETILE");

    return e && e[0] && (e[0] == 's' || e[0] == 'S');
}

static int mdp3_stream(int fd, uint32_t type, int on)
{
    uint32_t t = type;   /* STREAMON/OFF take a pointer to a __u32 */

    return ioctl(fd, on ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &t);
}

static void mdp3_release(void)
{
    int i;

    if (g_mdp3.fd < 0)
        return;
    if (g_mdp3.streaming) {
        mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0);
        mdp3_stream(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0);
        g_mdp3.streaming = 0;
    }
    for (i = 0; i < g_mdp3.cap_bufs; i++) {
        if (g_mdp3.cap_map[i] && g_mdp3.cap_map[i] != MAP_FAILED)
            munmap(g_mdp3.cap_map[i], g_mdp3.cap_size);
        g_mdp3.cap_map[i] = NULL;
    }
    mdp3_reqbufs(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 0, V4L2_MEMORY_MMAP);
    mdp3_reqbufs(g_mdp3.fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 0, V4L2_MEMORY_DMABUF);
    close(g_mdp3.fd);
    g_mdp3.fd = -1;
    g_mdp3.out_bufs = 0;
    g_mdp3.cap_bufs = 0;
    g_mdp3_owner = NULL;
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
    mdp3_release();
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

int v4l2sl_mdp3_init(struct v4l2sl_context *ctx, int width, int height)
{
    char path[32] = "?";
    struct v4l2_pix_format_mplane omp, cmp;
    uint32_t y_size, uv_size;
    int i;

    if (!ctx)
        return -1;
    if (g_mdp3.fd >= 0) {
        /* Already initialized: reuse only for its owner. */
        if (g_mdp3_owner == ctx) {
            ctx->mdp3.active = 1;
            ctx->mdp3.disabled = 0;
            return 0;
        }
        ctx->mdp3.disabled = 1;
        fprintf(stderr,
                "v4l2stateless: de-tile path = SW (mdp3 in use by another context)\n");
        return -1;
    }
    if (mdp3_forced_off()) {
        ctx->mdp3.disabled = 1;
        fprintf(stderr,
                "v4l2stateless: de-tile path = SW (V4L2SL_DETILE=sw)\n");
        return -1;
    }

    /* The m2m needs the decoder's real capture geometry (aligned height). */
    y_size = ctx->capture_buf_size;
    uv_size = ctx->capture_uv_size;
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
     * at QBUF). CAPTURE: driver-allocated linear NV12 (MMAP + mmap below). */
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
                                   MDP3_NBUFS, V4L2_MEMORY_MMAP);
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

    /* QUERYBUF + mmap each CAPTURE buffer (driver-allocated linear NV12). */
    for (i = 0; i < g_mdp3.cap_bufs; i++) {
        struct v4l2_buffer b = { 0 };
        struct v4l2_plane plane = { 0 };

        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.index = (unsigned)i;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = &plane;
        if (ioctl(g_mdp3.fd, VIDIOC_QUERYBUF, &b) < 0) {
            fprintf(stderr, "v4l2stateless: mdp3 QUERYBUF cap[%d] failed: %s\n",
                    i, strerror(errno));
            goto fail;
        }
        g_mdp3.cap_map[i] = mmap(NULL, plane.length,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 g_mdp3.fd, plane.m.mem_offset);
        if (g_mdp3.cap_map[i] == MAP_FAILED) {
            g_mdp3.cap_map[i] = NULL;
            fprintf(stderr, "v4l2stateless: mdp3 mmap cap[%d] failed: %s\n",
                    i, strerror(errno));
            goto fail;
        }
        /* Pre-queue it so a destination is always available. */
        memset(&b, 0, sizeof(b));
        memset(&plane, 0, sizeof(plane));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.index = (unsigned)i;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = &plane;
        if (ioctl(g_mdp3.fd, VIDIOC_QBUF, &b) < 0) {
            fprintf(stderr, "v4l2stateless: mdp3 QBUF cap[%d] failed: %s\n",
                    i, strerror(errno));
            goto fail;
        }
    }

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
    mdp3_release();
    ctx->mdp3.active = 0;
    ctx->mdp3.disabled = 1;
    fprintf(stderr, "v4l2stateless: de-tile path = SW (mdp3 init failed)\n");
    return -1;
}

int v4l2sl_mdp3_detile(struct v4l2sl_context *ctx, int buf_index,
                       void *dst, uint32_t dst_stride)
{
    struct v4l2_buffer b = { 0 };
    struct v4l2_plane planes[2] = { 0 };
    struct pollfd pfd;
    uint32_t y_size, uv_size;
    int cap_done, y_fd, uv_fd, err;

    if (!ctx || !dst || ctx->mdp3.disabled || g_mdp3.fd < 0 ||
        g_mdp3_owner != ctx || buf_index < 0)
        return -1;

    y_size = ctx->capture_buf_size;
    uv_size = ctx->capture_uv_size;
    if (!y_size || !uv_size)
        return -1;
    if (dst_stride < g_mdp3.cap_stride)
        return -1;   /* bo narrower than the capture: caller falls back */

    /*
     * Zero-copy input: export the decoder's two MM21 capture planes. The
     * OUTPUT slot is reused serially and dequeued after every job: a queued
     * DMABUF slot holds the previous dma-buf, so recycling it without DQBUF
     * would fail EBUSY once the ring wraps (4 frames).
     */
    y_fd = v4l2sl_capture_expbuf_plane(ctx, buf_index, 0);
    if (y_fd < 0)
        return mdp3_fail(ctx, "EXPBUF Y", errno);
    uv_fd = v4l2sl_capture_expbuf_plane(ctx, buf_index, 1);
    if (uv_fd < 0) {
        err = errno;
        close(y_fd);
        return mdp3_fail(ctx, "EXPBUF UV", err);
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
    planes[1].m.fd = uv_fd;
    planes[1].length = uv_size;
    err = ioctl(g_mdp3.fd, VIDIOC_QBUF, &b) < 0 ? errno : 0;
    /* vb2 takes its own dma-buf reference at QBUF; the fds can go now. */
    close(y_fd);
    close(uv_fd);
    if (err)
        return mdp3_fail(ctx, "QBUF in", err);

    /* The driver pairs the queued OUTPUT with the oldest queued CAPTURE and
     * runs the job; DQBUF returns the completed CAPTURE (linear NV12). */
    pfd.fd = g_mdp3.fd;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, MDP3_DQ_TIMEOUT_MS) <= 0)
        return mdp3_fail(ctx, "poll timeout", 0);

    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = 1;
    b.m.planes = planes;
    if (ioctl(g_mdp3.fd, VIDIOC_DQBUF, &b) < 0)
        return mdp3_fail(ctx, "DQBUF cap", errno);
    cap_done = (int)b.index;
    if (cap_done < 0 || cap_done >= g_mdp3.cap_bufs ||
        !g_mdp3.cap_map[cap_done])
        return mdp3_fail(ctx, "bad capture index", 0);

    /* One linear NV12 pass: mdp3 capture -> surface bo. */
    v4l2sl_copy_nv12((uint8_t *)dst, dst_stride,
                     (const uint8_t *)g_mdp3.cap_map[cap_done],
                     g_mdp3.cap_stride, g_mdp3.cap_height,
                     ctx->width < (int)g_mdp3.cap_stride
                         ? ctx->width : (int)g_mdp3.cap_stride,
                     ctx->height < (int)g_mdp3.cap_height
                         ? ctx->height : (int)g_mdp3.cap_height);

    /* Release the (now completed) OUTPUT buffer so its slot can be reused. */
    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.memory = V4L2_MEMORY_DMABUF;
    b.length = 2;
    b.m.planes = planes;
    if (ioctl(g_mdp3.fd, VIDIOC_DQBUF, &b) < 0)
        return mdp3_fail(ctx, "DQBUF in", errno);

    /* Re-queue the completed CAPTURE so a destination is always available. */
    memset(&b, 0, sizeof(b));
    memset(planes, 0, sizeof(planes));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.index = (unsigned)cap_done;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = 1;
    b.m.planes = planes;
    if (ioctl(g_mdp3.fd, VIDIOC_QBUF, &b) < 0)
        return mdp3_fail(ctx, "QBUF cap", errno);

    ctx->mdp3.fail_count = 0;
    if (mdp3_log())
        fprintf(stderr, "v4l2stateless: mdp3 detile idx=%d cap=%d (HW)\n",
                buf_index, cap_done);
    return 0;
}

void v4l2sl_mdp3_fini(struct v4l2sl_context *ctx)
{
    if (!ctx || g_mdp3.fd < 0 || g_mdp3_owner != ctx)
        return;
    mdp3_release();
    ctx->mdp3.active = 0;
    ctx->mdp3.disabled = 1;
}
