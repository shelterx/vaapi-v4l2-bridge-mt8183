/*
 * v4l2stateless — V4L2 Request API device and buffer management
 *
 * Handles:
 * - V4L2 device open and capability check
 * - Output queue (compressed bitstream) setup + mmap
 * - Capture queue (decoded frames) setup
 * - MEDIA_IOC_REQUEST_ALLOC
 * - Buffer queue/dequeue
 * - DMA-BUF export from capture buffers
 * - Request submission (ext ctrls + VIDIOC_SUBSCRIBE_EVENT)
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <va/va_drmcommon.h>
#include <drm_fourcc.h>

#include "v4l2stateless.h"
#include "v4l2stateless_probe.h"

static v4l2sl_ioctl_fn g_ioctl_hook;

/* filled by decode_submit, consumed by the matching pull_capture */
static double g_prof_pending_decode = -1;
static double g_prof_pending_detile = -1;
static int64_t g_prof_pending_frame_us;

void v4l2sl_set_ioctl_hook(v4l2sl_ioctl_fn fn)
{
    g_ioctl_hook = fn;
}

/* Helper to call ioctl with retry (shared via the header). */
int v4l2sl_xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = g_ioctl_hook ? g_ioctl_hook(fd, request, arg)
                         : ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

#define xioctl(fd, req, arg) v4l2sl_xioctl((fd), (req), (arg))

/* poll() with EINTR retry — a stray signal (profiler, SIGWINCH) must never
 * be mistaken for a decode timeout; the decode path turns timeouts into
 * full STREAMOFF/STREAMON queue resets. */
int v4l2sl_poll_intr(struct pollfd *fds, nfds_t n, int timeout)
{
    int r;

    do {
        r = poll(fds, n, timeout);
    } while (r < 0 && errno == EINTR);
    return r;
}

/*
 * Open a V4L2 video device and verify it's a stateless decoder
 */
int v4l2sl_open_device(const char *path)
{
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "v4l2stateless: open %s: %s\n", path, strerror(errno));
        return -1;
    }

    /* Verify capabilities */
    struct v4l2_capability cap = { 0 };
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        fprintf(stderr, "v4l2stateless: QUERYCAP failed on %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE)) {
        fprintf(stderr, "v4l2stateless: %s is not M2M mplane device\n", path);
        close(fd);
        return -1;
    }

    fprintf(stderr, "v4l2stateless: opened %s (%s)\n", path, cap.card);
    return fd;
}

/*
 * Open the media device associated with a V4L2 video device
 * video node → matching media node via sysfs (numbers need not match)
 */
int v4l2sl_open_media_for_device(const char *video_path)
{
    /* /dev/videoN lives next to /dev/mediaM on the same platform device.
     * Walk sysfs — do not assume mediaN == videoN (AV1 has been video4 → media3). */
    char media_path[128];
    int fd;

    if (v4l2sl_find_media_path(video_path, media_path, sizeof(media_path)) < 0) {
        fprintf(stderr, "v4l2stateless: no sysfs media* for %s\n", video_path);
        return -1;
    }

    fd = open(media_path, O_RDWR);
    if (fd < 0)
        fprintf(stderr, "v4l2stateless: open %s (for %s): %s\n",
                media_path, video_path, strerror(errno));
    else
        fprintf(stderr, "v4l2stateless: media for %s is %s\n", video_path, media_path);
    return fd;
}

/*
 * Allocate a V4L2 request
 */
int v4l2sl_request_alloc(int media_fd)
{
    int request_fd = -1;
    if (xioctl(media_fd, MEDIA_IOC_REQUEST_ALLOC, &request_fd) < 0) {
        fprintf(stderr, "v4l2stateless: REQUEST_ALLOC failed: %s\n", strerror(errno));
        return -1;
    }
    return request_fd;
}

/*
 * Setup output queue (compressed bitstream input)
 * Returns 0 on success, -1 on error
 */
int v4l2sl_setup_output_queue(int fd, uint32_t codec_format, int width, int height)
{
    /* Set output format (compressed input) */
    struct v4l2_format fmt = { 0 };
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = codec_format;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;
    /* hantro coded formats want sizeimage ~= luma size, not a 4MB guess.
     * GStreamer sizes H.264 OUTPUT as (width*height*pixel_bitdepth)/8;
     * 4:2:0 8-bit=12, 10-bit=15. A luma-sized buffer is too small for
     * 10-bit High10 and rkvdec then writes an empty NV15 capture. */
    if (codec_format == V4L2_PIX_FMT_AV1_FRAME ||
        codec_format == V4L2_PIX_FMT_VP8_FRAME ||
        codec_format == V4L2_PIX_FMT_MPEG2_SLICE)
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = (uint32_t)width * (uint32_t)height;
    else if (codec_format == V4L2_PIX_FMT_H264_SLICE)
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage =
            (uint32_t)width * (uint32_t)height * 15 / 8;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "v4l2stateless: S_FMT output failed: %s\n", strerror(errno));
        return -1;
    }

    /* Set codec-specific controls */
    struct v4l2_control ctrl = { 0 };

    if (codec_format == V4L2_PIX_FMT_H264_SLICE) {
        ctrl.id = V4L2_CID_STATELESS_H264_DECODE_MODE;
        ctrl.value = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED;
        if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0)
            fprintf(stderr, "v4l2stateless: set H264 decode mode failed: %s\n", strerror(errno));

        ctrl.id = V4L2_CID_STATELESS_H264_START_CODE;
        ctrl.value = V4L2_STATELESS_H264_START_CODE_ANNEX_B;
        if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0)
            fprintf(stderr, "v4l2stateless: set H264 start code failed: %s\n", strerror(errno));
    } else if (codec_format == V4L2_PIX_FMT_HEVC_SLICE) {
        ctrl.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE;
        ctrl.value = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED;
        if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0)
            fprintf(stderr, "v4l2stateless: set HEVC decode mode failed: %s\n", strerror(errno));

        ctrl.id = V4L2_CID_STATELESS_HEVC_START_CODE;
        ctrl.value = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B;
        if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0)
            fprintf(stderr, "v4l2stateless: set HEVC start code failed: %s\n", strerror(errno));
    }

    /* Request output buffers */
    struct v4l2_requestbuffers req = { 0 };
    req.count = V4L2SL_NUM_OUTPUT_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "v4l2stateless: REQBUFS output failed: %s\n", strerror(errno));
        return -1;
    }

    fprintf(stderr, "v4l2stateless: output queue: %d buffers\n", req.count);
    return req.count;  /* return number of allocated buffers */
}

/*
 * Setup capture queue (decoded frames output)
 * Returns 0 on success, -1 on error
 */
int v4l2sl_setup_capture_queue(int fd, int width, int height, uint32_t pixelformat)
{
    return v4l2sl_setup_capture_queue_count(fd, width, height, pixelformat,
                                            V4L2SL_NUM_CAPTURE_BUFS);
}

int v4l2sl_setup_capture_queue_count(int fd, int width, int height,
                                     uint32_t pixelformat, int count)
{
    /* Set capture format (decoded output). Default NV12; 10-bit/422 use
     * NV15/NV16/NV20 which rkvdec only accepts after a matching SPS. */
    struct v4l2_format fmt = { 0 };
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = pixelformat ? pixelformat : V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "v4l2stateless: S_FMT capture failed: %s\n", strerror(errno));
        return -1;
    }

    /* Request capture buffers. Ask for non-coherent (cacheable) memory:
     * the decoder's DMA is synced by vb2 around QBUF/DQBUF, and a cacheable
     * mapping turns the CPU de-tile from ~12 ms/frame into ~1.8 ms at 1080p
     * (a coherent CMA mapping is uncached and reads at ~380 MB/s). Fall back
     * to coherent buffers if the driver does not accept cache hints. */
    struct v4l2_requestbuffers req = { 0 };
    req.count = count > 0 ? count : V4L2SL_NUM_CAPTURE_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    req.flags = V4L2_MEMORY_FLAG_NON_COHERENT;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        req.flags = 0;
        if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            fprintf(stderr, "v4l2stateless: REQBUFS capture failed: %s\n",
                    strerror(errno));
            return -1;
        }
    }

    fprintf(stderr, "v4l2stateless: capture queue: %d buffers%s\n", req.count,
            (req.flags & V4L2_MEMORY_FLAG_NON_COHERENT) ? " (cacheable)" : "");
    return req.count;  /* return number of allocated buffers */
}

/*
 * Start streaming on a queue — required after REQBUFS and before QBUF
 * Returns 0 on success, -1 on error
 */
int v4l2sl_streamon(int fd, enum v4l2_buf_type type)
{
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "v4l2stateless: STREAMON(type=%u) failed: %s\n",
                type, strerror(errno));
        return -1;
    }
    return 0;
}

int v4l2sl_streamoff(int fd, enum v4l2_buf_type type)
{
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        fprintf(stderr, "v4l2stateless: STREAMOFF(type=%u) failed: %s\n",
                type, strerror(errno));
        return -1;
    }
    return 0;
}

static void release_ctx_capture_surfaces(struct v4l2sl_context *ctx);
static void v4l2sl_unmap_capture_buffers(struct v4l2sl_context *ctx);
static int v4l2sl_mmap_one_capture(struct v4l2sl_context *ctx, int index);

static int reqbufs_zero(int fd, enum v4l2_buf_type type)
{
    struct v4l2_requestbuffers req = { 0 };

    req.count = 0;
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "v4l2stateless: REQBUFS(0) type=%u failed: %s\n",
                type, strerror(errno));
        return -1;
    }
    return 0;
}

void v4l2sl_m2m_teardown(struct v4l2sl_context *ctx, struct v4l2sl_m2m_state *q)
{
    int i;

    if (!ctx || !q || !q->valid)
        return;
    if (ctx->v4l2_fd >= 0) {
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
        reqbufs_zero(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        reqbufs_zero(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    }
    for (i = 0; i < 3; i++) {
        if (q->out_map[i] && q->out_map[i] != MAP_FAILED)
            munmap(q->out_map[i], q->out_len[i]);
    }
    if (q->cap_map && q->cap_map != MAP_FAILED)
        munmap(q->cap_map, q->cap_len);
    memset(q, 0, sizeof(*q));
}

void v4l2sl_release_context_device(struct v4l2sl_context *ctx)
{
    int i;

    if (!ctx)
        return;

    v4l2sl_gbm_detile_wait();

    /* Persistent M2M queues hold V4L2 MMAP mappings — release them while
     * the device fd is still open. */
    v4l2sl_m2m_teardown(ctx, &ctx->vpp_q);
    v4l2sl_m2m_teardown(ctx, &ctx->jpeg_q);
    v4l2sl_mdp3_fini(ctx);

    if (ctx->v4l2_fd >= 0 && ctx->streamed) {
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
        ctx->streamed = 0;
    }

    if (ctx->request_fd >= 0) {
        close(ctx->request_fd);
        ctx->request_fd = -1;
    }

    for (i = 0; i < ctx->output_bufs_allocd && i < V4L2SL_NUM_OUTPUT_BUFS; i++) {
        if (ctx->output_buf_ptr[i] && ctx->output_buf_ptr[i] != MAP_FAILED) {
            munmap(ctx->output_buf_ptr[i], ctx->output_buf_size);
            ctx->output_buf_ptr[i] = NULL;
        }
    }
    v4l2sl_unmap_capture_buffers(ctx);

    if (ctx->v4l2_fd >= 0) {
        if (ctx->output_bufs_allocd > 0) {
            reqbufs_zero(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
            ctx->output_bufs_allocd = 0;
            ctx->n_free_out = 0;
        }
        if (ctx->capture_bufs_allocd > 0) {
            reqbufs_zero(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
            ctx->capture_bufs_allocd = 0;
            ctx->n_free_cap = 0;
        }
        close(ctx->v4l2_fd);
        ctx->v4l2_fd = -1;
    }

    if (ctx->media_fd >= 0) {
        close(ctx->media_fd);
        ctx->media_fd = -1;
    }

    release_ctx_capture_surfaces(ctx);
}

static void release_ctx_capture_surfaces(struct v4l2sl_context *ctx)
{
    int i;

    if (!ctx || !ctx->driver_data)
        return;
    for (i = 0; i < ctx->num_render_targets; i++) {
        VASurfaceID id = ctx->render_targets[i];
        struct v4l2sl_surface *s;

        s = v4l2sl_surface_by_id(ctx->driver_data, id);
        if (!s)
            continue;
        /* Keep the create-time memfd. Closing a V4L2 EXPBUF fd while the
         * GPU still holds a dup hangs the RK3588 CMA pool. */
        s->buf_index = -1;
        s->cap_view = NULL;
    }
}

/* Rebuild both free pools from scratch. Capture indices still claimed by a
 * render-target surface (surf->buf_index) stay claimed — begin_picture
 * returns them when the surface is re-targeted. */
static void rebuild_free_pools(struct v4l2sl_context *ctx)
{
    int claimed[V4L2SL_MAX_CAPTURE_BUFS] = { 0 };
    int i;

    ctx->n_free_out = 0;
    for (i = 0; i < ctx->output_bufs_allocd && i < V4L2SL_NUM_OUTPUT_BUFS; i++)
        v4l2sl_out_pool_push(ctx, i);

    if (ctx->driver_data && ctx->render_targets) {
        for (i = 0; i < ctx->num_render_targets; i++) {
            struct v4l2sl_surface *s =
                v4l2sl_surface_by_id(ctx->driver_data, ctx->render_targets[i]);

            if (s && s->buf_index >= 0 && s->buf_index < V4L2SL_MAX_CAPTURE_BUFS)
                claimed[s->buf_index] = 1;
        }
    }

    ctx->n_free_cap = 0;
    for (i = 0; i < ctx->capture_bufs_allocd && i < V4L2SL_MAX_CAPTURE_BUFS; i++)
        if (!claimed[i])
            v4l2sl_cap_pool_push(ctx, i);
}

void v4l2sl_decode_reset(struct v4l2sl_context *ctx)
{
    v4l2sl_gbm_detile_wait();   /* capture slots are about to be recycled */
    if (!ctx || ctx->v4l2_fd < 0)
        return;

    fprintf(stderr, "v4l2stateless: resetting decode queues\n");

    if (ctx->streamed) {
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
        ctx->streamed = 0;
    }
    /* STREAMOFF also drops every kernel DPB reference. */
    v4l2sl_av1_dpb_model_reset(ctx);

    if (ctx->request_fd >= 0) {
        close(ctx->request_fd);
        ctx->request_fd = -1;
    }

    /* STREAMOFF hands every queued buffer back in ERROR state (rkvdec and
     * hantro both do this in stop_streaming), so from this point every
     * allocated index is userspace-owned again. */
    rebuild_free_pools(ctx);
}


/*
 * Opt-in per-frame timing (V4L2SL_PROFILE=<window>, default 120 frames).
 * Decode and pull_capture run back-to-back on the decoder thread, so the
 * interval between frames shows whether a stutter comes from us or from
 * upstream pacing. Everything is a no-op unless the env var is set.
 */
static struct {
    int enabled, window, n, slow;
    double dec_sum, dec_max;
    double pull_sum, pull_max;
    double det_sum, det_max;
    double iv_sum, iv_max;
    int64_t last_frame_us;
} g_prof;

static int64_t prof_now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static FILE *g_trace;

static void prof_init(void)
{
    const char *e = getenv("V4L2SL_PROFILE");
    const char *tr = getenv("V4L2SL_TRACE");
    int prof_on = e && e[0] && e[0] != '0';
    int trace_on = tr && tr[0];

    if (!prof_on && !trace_on)
        return;
    g_prof.window = prof_on ? atoi(e) : 120;
    if (g_prof.window <= 0)
        g_prof.window = 120;
    if (trace_on) {
        g_trace = fopen(tr, "w");
        if (g_trace) {
            setvbuf(g_trace, NULL, _IOLBF, 0);
            fprintf(g_trace,
                    "rt_ms,t_us,decode_ms,detile_ms,pull_ms,wait_ms,job_wall_ms,job_cpu_ms\n");
        }
    }
    g_prof.enabled = 1;
    if (prof_on)
        fprintf(stderr, "v4l2stateless: profile on, window=%d frames\n",
                g_prof.window);
    if (g_trace)
        fprintf(stderr, "v4l2stateless: trace -> %s\n", tr);
}

static void prof_report(void)
{
    if (g_prof.n < g_prof.window)
        return;
    fprintf(stderr,
            "v4l2stateless: profile n=%d decode avg=%.1f max=%.1f | "
            "detile avg=%.1f max=%.1f | pull avg=%.1f max=%.1f | "
            "interval avg=%.1f max=%.1f | gaps>60ms=%d\n",
            g_prof.n, g_prof.dec_sum / g_prof.n, g_prof.dec_max,
            g_prof.det_sum / g_prof.n, g_prof.det_max,
            g_prof.pull_sum / g_prof.n, g_prof.pull_max,
            g_prof.iv_sum / g_prof.n, g_prof.iv_max, g_prof.slow);
    g_prof.n = 0;
    g_prof.dec_sum = g_prof.dec_max = 0;
    g_prof.pull_sum = g_prof.pull_max = 0;
    g_prof.det_sum = g_prof.det_max = 0;
    g_prof.iv_sum = g_prof.iv_max = 0;
    g_prof.slow = 0;
}

static void prof_frame(int64_t t0_us, int64_t t1_us, double decode_ms,
                       double detile_ms, double pull_ms)
{
    double iv;

    if (!g_prof.enabled)
        return;
    if (g_prof.last_frame_us) {
        iv = (t0_us - g_prof.last_frame_us) / 1000.0;
        g_prof.iv_sum += iv;
        if (iv > g_prof.iv_max)
            g_prof.iv_max = iv;
        if (iv > 60.0)
            g_prof.slow++;
    }
    g_prof.last_frame_us = t0_us;
    if (decode_ms >= 0) {
        g_prof.dec_sum += decode_ms;
        if (decode_ms > g_prof.dec_max)
            g_prof.dec_max = decode_ms;
    }
    if (detile_ms >= 0) {
        g_prof.det_sum += detile_ms;
        if (detile_ms > g_prof.det_max)
            g_prof.det_max = detile_ms;
    }
    if (pull_ms >= 0) {
        g_prof.pull_sum += pull_ms;
        if (pull_ms > g_prof.pull_max)
            g_prof.pull_max = pull_ms;
    }
    if (decode_ms > 25.0 || pull_ms > 8.0)
        fprintf(stderr,
                "v4l2stateless: profile slow frame decode=%.1f detile=%.1f "
                "pull=%.1f\n",
                decode_ms, detile_ms, pull_ms);
    if (g_trace) {
        struct timespec rt;
        double jw = -1.0, jc = -1.0;

        clock_gettime(CLOCK_REALTIME, &rt);
        v4l2sl_gbm_detile_last_job(&jw, &jc);
        fprintf(g_trace, "%.3f,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                rt.tv_sec * 1e3 + rt.tv_nsec / 1e6, (long long)t0_us,
                decode_ms, detile_ms, pull_ms,
                v4l2sl_gbm_detile_last_wait_ms(), jw, jc);
    }
    g_prof.n++;
    prof_report();
    (void)t1_us;
}

static int decode_submit_inner(struct v4l2sl_context *ctx, int out_buf_idx,
                               uint32_t bytesused, uint64_t timestamp);

int v4l2sl_decode_submit(struct v4l2sl_context *ctx, int out_buf_idx,
                         uint32_t bytesused, uint64_t timestamp)
{
    static int prof_ready;
    int64_t t0, t1;
    int ret;

    if (!prof_ready) {
        prof_ready = 1;
        prof_init();
    }
    if (!g_prof.enabled)
        return decode_submit_inner(ctx, out_buf_idx, bytesused, timestamp);
    t0 = prof_now_us();
    ret = decode_submit_inner(ctx, out_buf_idx, bytesused, timestamp);
    t1 = prof_now_us();
    g_prof_pending_decode = (t1 - t0) / 1000.0;
    g_prof_pending_frame_us = t0;
    return ret;
}

static int decode_submit_inner(struct v4l2sl_context *ctx, int out_buf_idx,
                         uint32_t bytesused, uint64_t timestamp)
{
    struct pollfd pfd;
    int cap_buf_idx, done_cap, done_out;

    if (!ctx || ctx->v4l2_fd < 0 || ctx->request_fd < 0)
        return -1;

    /* Re-decoding into a surface whose de-tile is still in flight would let
     * the decoder overwrite the capture slot the worker is reading. Overlap
     * stays intact for every other surface. */
    if (ctx->current_surface &&
        v4l2sl_gbm_detile_pending_for(ctx->current_surface))
        v4l2sl_gbm_detile_wait();

    cap_buf_idx = -1;
    if (v4l2sl_expbuf_export_wanted() && ctx->current_surface &&
        ctx->current_surface->buf_index >= 0)
        cap_buf_idx = ctx->current_surface->buf_index;
    if (cap_buf_idx < 0) {
        if (ctx->n_free_cap == 0) {
            fprintf(stderr, "v4l2stateless: no free capture buffer\n");

            v4l2sl_out_pool_push(ctx, out_buf_idx);
            return -1;
        }
        cap_buf_idx = ctx->free_cap_bufs[--ctx->n_free_cap];
    }

    if (v4l2sl_queue_output(ctx, out_buf_idx, bytesused, timestamp) < 0) {
        fprintf(stderr, "v4l2stateless: QBUF output[%d] failed\n", out_buf_idx);
        v4l2sl_out_pool_push(ctx, out_buf_idx);
        v4l2sl_cap_pool_push(ctx, cap_buf_idx);
        return -1;
    }

    /* Capture is queued bare: the stateless decoder spec forbids CAPTURE
     * buffers in a request (vb2 answers EPERM for them). */
    if (v4l2sl_queue_capture(ctx->v4l2_fd, cap_buf_idx, ctx->request_fd) < 0) {
        fprintf(stderr, "v4l2stateless: QBUF capture[%d] failed\n", cap_buf_idx);
        v4l2sl_cap_pool_push(ctx, cap_buf_idx);
        /* Recover the bitstream buffer so the pool does not leak. */
        pfd.fd = ctx->v4l2_fd;
        pfd.events = POLLOUT;
        if (v4l2sl_poll_intr(&pfd, 1, 200) > 0) {
            done_out = v4l2sl_dequeue_buffer(ctx->v4l2_fd,
                                             V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                                             NULL);
            if (done_out >= 0)
                v4l2sl_out_pool_push(ctx, done_out);
        }
        /* The output buffer was already bound to this request but the request
         * is never queued (no m2m job). Drop the request so its stale object
         * binding doesn't corrupt the next frame's request state. */
        if (ctx->request_fd >= 0) {
            close(ctx->request_fd);
            ctx->request_fd = -1;
        }
        return -1;
    }

    if (v4l2sl_submit_request(ctx->request_fd) < 0) {
        fprintf(stderr, "v4l2stateless: MEDIA_REQUEST_IOC_QUEUE failed\n");
        /* The buffers stay queued kernel-side: full reset, not just a
         * pool push — a request that failed to queue produces nothing. */
        v4l2sl_decode_reset(ctx);
        return -1;
    }

    pfd.fd = ctx->v4l2_fd;
    pfd.events = POLLIN;
    if (v4l2sl_poll_intr(&pfd, 1, 3000) <= 0) {
        fprintf(stderr, "v4l2stateless: decode timeout, resetting queues\n");
        v4l2sl_decode_reset(ctx);
        return -1;
    }

    {
        int cap_err = 0;

        done_cap = v4l2sl_dequeue_buffer(ctx->v4l2_fd,
                                         V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                         &cap_err);
        if (done_cap < 0) {
            fprintf(stderr, "v4l2stateless: no completed capture buffer\n");
            v4l2sl_decode_reset(ctx);
            return -1;
        }
        if (cap_err) {
            /* Corrupt frame: recycle the slot, recover the bitstream buffer
             * via the shared tail, and report -2 ("skipped") — callers mark
             * the surface instead of failing the vaEndPicture entrypoint
             * (Chrome caches VA-API failures for the whole session). */
            if (v4l2sl_debug)
                fprintf(stderr,
                        "v4l2stateless: capture buf[%d] V4L2_BUF_FLAG_ERROR -> skip\n",
                        done_cap);
            v4l2sl_cap_pool_push(ctx, done_cap);
            done_cap = -2;
        }
    }

    done_out = v4l2sl_dequeue_buffer(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, NULL);
    if (done_out < 0) {
        pfd.events = POLLOUT;
        if (v4l2sl_poll_intr(&pfd, 1, 200) > 0)
            done_out = v4l2sl_dequeue_buffer(ctx->v4l2_fd,
                                             V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                                             NULL);
    }
    if (done_out >= 0)
        v4l2sl_out_pool_push(ctx, done_out);

    /*
     * Do NOT MEDIA_REQUEST_IOC_REINIT here. The MTK worker for MT8183 (the
     * non-LAT arch) completes the request *after* it has already finished the
     * m2m job and returned the capture buffer:
     *
     *   v4l2_m2m_buf_done_and_job_finish(...);
     *   if (src_buf_req) v4l2_ctrl_request_complete(src_buf_req, &ctx->ctrl_hdl);
     *
     * so by the time we can DQBUF the capture buffer and reach this point the
     * worker may still be inside v4l2_ctrl_request_complete(). REINIT clears the
     * request's bound objects and would race that completion, tripping
     * media_request_object_bind()/v4l2_ctrl_request_complete() WARNs and leaving
     * the request state corrupt — that is what eventually wedges /dev/video5
     * (docs/mt8183-notes.md §10).
     *
     * Dropping the fd is safe: the kernel keeps the request alive via the
     * buffer's own reference until the worker is done with it. begin_picture()
     * allocates a fresh request next frame.
     */
    if (ctx->request_fd >= 0) {
        close(ctx->request_fd);
        ctx->request_fd = -1;
    }

    return done_cap;
}

/*
 * De-tile MM21 (16x32 Y tiles / 8x16 chroma tiles, row-major) -> linear NV12.
 * dst is aw*ah*3/2; aw aligned to 16, ah to 32. ysrc is the Y plane
 * (y_tc*y_tr*512 bytes, 16x32 tiles) and usrc the UV plane (y_tc*y_tr*256
 * bytes, 8x16 chroma-sample tiles on the SAME luma tile grid, 16 bytes per
 * row); the MT8183 driver keeps them in separate CMA blocks, so they arrive
 * as two distinct mappings. Reference: tools/mm21_detile.c.
 */
static void v4l2sl_mm21_detile(const uint8_t *ysrc, const uint8_t *usrc,
                               uint8_t *dst, uint32_t aw, uint32_t ah)
{
    uint32_t half_w = aw / 2;
    uint32_t y_tc = aw / 16, y_tr = ah / 32;
    const uint8_t *yt = ysrc;
    const uint8_t *uvt = usrc;
    uint8_t *yl = dst;
    uint8_t *uvl = dst + (size_t)aw * ah;

    for (uint32_t tr = 0; tr < y_tr; tr++)
        for (uint32_t tc = 0; tc < y_tc; tc++) {
            const uint8_t *ts = yt + (size_t)(tr * y_tc + tc) * 512;
            for (uint32_t r = 0; r < 32; r++)
                for (uint32_t c = 0; c < 16; c++)
                    yl[(size_t)(tr * 32 + r) * aw + (tc * 16 + c)] = ts[r * 16 + c];
        }
    /*
     * Chroma: the tile grid is IDENTICAL to the luma grid (y_tc x y_tr) — the
     * kernel doc says "the number of luma and chroma tiles are identical".
     * Each tile is 8x16 chroma samples (interleaved NV12, 256 bytes), one per
     * 16x32 luma tile; the plane is half_w x half_h chroma samples. (Using a
     * half_w/16 x half_h/16 grid of 16x16 tiles is WRONG and only shows up on
     * non-neutral chroma, e.g. coloured text -> grey text with cyan fringes.)
     */
    for (uint32_t tr = 0; tr < y_tr; tr++)
        for (uint32_t tc = 0; tc < y_tc; tc++) {
            const uint8_t *ts = uvt + (size_t)(tr * y_tc + tc) * 256;
            for (uint32_t r = 0; r < 16; r++)
                for (uint32_t c = 0; c < 8; c++) {
                    size_t di = ((size_t)(tr * 16 + r) * half_w + (tc * 8 + c)) * 2;
                    uvl[di] = ts[r * 16 + c * 2];
                    uvl[di + 1] = ts[r * 16 + c * 2 + 1];
                }
        }
}

/*
 * Pick the capture fourcc. Some decoders (MT8183 mtk-vcodec) only offer a
 * tiled capture format (MM21); if the requested linear fourcc is not
 * supported but MM21 is, use MM21 (the surface is de-tiled to NV12 later in
 * v4l2sl_surface_pull_capture).
 */
static uint32_t v4l2sl_capture_pick_fourcc(int fd, uint32_t requested)
{
    struct v4l2_fmtdesc fmt = { 0 };
    int has_requested = 0, has_mm21 = 0;

    if (requested == V4L2_PIX_FMT_MM21)
        return V4L2_PIX_FMT_MM21;

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    while (xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
        if (fmt.pixelformat == requested)
            has_requested = 1;
        if (fmt.pixelformat == V4L2_PIX_FMT_MM21)
            has_mm21 = 1;
        fmt.index++;
    }
    if (has_requested)
        return requested;
    if (has_mm21)
        return V4L2_PIX_FMT_MM21;
    return requested;
}

/*
 * Reconfigure capture (and coded OUTPUT size) when bit-depth, chroma or
 * resolution changes. Safe to call before the first STREAMON.
 */
int v4l2sl_ensure_capture(struct v4l2sl_context *ctx, int width, int height,
                          uint32_t pixelformat)
{
    int n_cap;
    uint32_t fourcc = v4l2sl_capture_pick_fourcc(
        ctx->v4l2_fd, pixelformat ? pixelformat : V4L2_PIX_FMT_NV12);
    int size_changed, fmt_changed;

    if (!ctx || ctx->v4l2_fd < 0 || width <= 0 || height <= 0)
        return -1;

    /* Display size from vaCreateContext is often 1920x1080 while the
     * coded size is 1920x1088. Reconfig only if the current capture
     * buffer cannot hold the new frame, or the fourcc changed. */
    size_changed = (ctx->cap_width == 0) ||
                   (width > (int)ctx->cap_width) ||
                   (height > (int)ctx->cap_height);
    fmt_changed = (ctx->cap_pixelformat != fourcc) ||
                  (ctx->capture_bufs_allocd <= 0);

    if (!size_changed && !fmt_changed)
        return 0;

    fprintf(stderr,
            "v4l2stateless: renegotiate capture %dx%d %.4s -> %dx%d %.4s streamed=%d\n",
            ctx->width, ctx->height, (char *)&ctx->cap_pixelformat,
            width, height, (char *)&fourcc, ctx->streamed);

    if (ctx->streamed) {
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        v4l2sl_streamoff(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
        ctx->streamed = 0;
        v4l2sl_av1_dpb_model_reset(ctx);
    }

    v4l2sl_gbm_detile_wait();   /* pending de-tile reads a capture slot */
    release_ctx_capture_surfaces(ctx);
    v4l2sl_unmap_capture_buffers(ctx);

    if (ctx->capture_bufs_allocd > 0) {
        struct v4l2_requestbuffers req = { 0 };
        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;
        if (xioctl(ctx->v4l2_fd, VIDIOC_REQBUFS, &req) < 0) {
            fprintf(stderr, "v4l2stateless: REQBUFS(0) capture failed: %s\n",
                    strerror(errno));
            return -1;
        }
        ctx->capture_bufs_allocd = 0;
        ctx->n_free_cap = 0;
    }

    if (size_changed) {
        /* A bigger coded size also means bigger bitstream buffers. vb2
         * refuses S_FMT(OUTPUT) with -EBUSY while buffers are allocated,
         * so the OUTPUT queue has to be rebuilt too (spec: dynamic
         * resolution change re-runs the OUTPUT S_FMT step). Best effort:
         * if the re-mmap fails we keep going with empty pointers — the
         * next QBUF will still carry the right bytesused. */
        struct v4l2_format ofmt = { 0 };
        struct v4l2_format cur = { 0 };
        int i, n_out;
        uint32_t codec_fcc = 0;

        ofmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        cur.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        if (ctx->output_bufs_allocd > 0 &&
            xioctl(ctx->v4l2_fd, VIDIOC_G_FMT, &cur) == 0 &&
            (cur.fmt.pix_mp.width < (uint32_t)width ||
             cur.fmt.pix_mp.height < (uint32_t)height)) {
            codec_fcc = cur.fmt.pix_mp.pixelformat;

            for (i = 0; i < ctx->output_bufs_allocd &&
                        i < V4L2SL_NUM_OUTPUT_BUFS; i++) {
                if (ctx->output_buf_ptr[i] && ctx->output_buf_ptr[i] != MAP_FAILED) {
                    munmap(ctx->output_buf_ptr[i], ctx->output_buf_size);
                    ctx->output_buf_ptr[i] = NULL;
                }
            }
            if (reqbufs_zero(ctx->v4l2_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) == 0) {
                ctx->output_bufs_allocd = 0;
                ctx->n_free_out = 0;

                ofmt = cur;
                ofmt.fmt.pix_mp.width = width;
                ofmt.fmt.pix_mp.height = height;
                if (xioctl(ctx->v4l2_fd, VIDIOC_S_FMT, &ofmt) < 0)
                    fprintf(stderr, "v4l2stateless: S_FMT output resize failed: %s\n",
                            strerror(errno));
                n_out = v4l2sl_setup_output_queue(ctx->v4l2_fd, codec_fcc,
                                                  width, height);
                if (n_out > 0) {
                    int j;

                    ctx->output_bufs_allocd = n_out;
                    if (v4l2sl_mmap_output_buffers(ctx->v4l2_fd, n_out,
                                                   ctx->output_buf_ptr,
                                                   &ctx->output_buf_size) < 0)
                        fprintf(stderr,
                                "v4l2stateless: warning: output re-mmap failed\n");
                    for (j = 0; j < n_out && j < V4L2SL_NUM_OUTPUT_BUFS; j++)
                        v4l2sl_out_pool_push(ctx, j);
                } else {
                    fprintf(stderr,
                            "v4l2stateless: warning: output queue rebuild failed\n");
                }
            }
        }
    }

    /* Capture re-allocation with graceful degradation: CMA can be nearly
     * exhausted (desktop GBM), and a smaller pool beats a dead context.
     * The spec puts the reference-depth burden on the client — 24 covers
     * every profile; a degraded pool only hurts deep B-pyramids. */
    {
        int want_cap = ctx->codec == V4L2SL_CODEC_AV1 ?
                       V4L2SL_NUM_CAPTURE_BUFS_AV1 : V4L2SL_NUM_CAPTURE_BUFS;
        n_cap = v4l2sl_setup_capture_queue_count(ctx->v4l2_fd, width, height,
                                                 fourcc, want_cap);
    }
    if (n_cap <= 0 && V4L2SL_NUM_CAPTURE_BUFS > 24) {
        fprintf(stderr, "v4l2stateless: retrying capture REQBUFS with 24 buffers\n");
        n_cap = v4l2sl_setup_capture_queue_count(ctx->v4l2_fd, width, height,
                                                 fourcc, 24);
    }
    if (n_cap <= 0 && V4L2SL_NUM_CAPTURE_BUFS > 8) {
        fprintf(stderr, "v4l2stateless: retrying capture REQBUFS with 8 buffers\n");
        n_cap = v4l2sl_setup_capture_queue_count(ctx->v4l2_fd, width, height,
                                                 fourcc, 8);
    }
    if (n_cap <= 0 && V4L2SL_NUM_CAPTURE_BUFS > 4) {
        fprintf(stderr, "v4l2stateless: retrying capture REQBUFS with 4 buffers\n");
        n_cap = v4l2sl_setup_capture_queue_count(ctx->v4l2_fd, width, height,
                                                 fourcc, 4);
    }
    if (n_cap <= 0)
        return -1;

    ctx->capture_bufs_allocd = n_cap;
    if (v4l2sl_debug)
        fprintf(stderr, "v4l2stateless: capture pool n=%d\n", n_cap);
    ctx->n_free_cap = 0;
    {
        int i;
        for (i = 0; i < n_cap && i < V4L2SL_NUM_CAPTURE_BUFS; i++)
            v4l2sl_cap_pool_push(ctx, i);
    }
    ctx->width = width;
    ctx->height = height;
    ctx->cap_pixelformat = fourcc;

    if (v4l2sl_get_capture_geometry(ctx->v4l2_fd, &ctx->cap_width, &ctx->cap_height,
                                    &ctx->cap_stride, &ctx->cap_sizeimage) == 0) {
        fprintf(stderr, "v4l2stateless: capture geometry %ux%u stride=%u size=%u fourcc=%.4s\n",
                ctx->cap_width, ctx->cap_height, ctx->cap_stride, ctx->cap_sizeimage,
                (char *)&fourcc);
    }
    /* Map capture slots lazily after DQBUF. Mapping all 24 CMA
     * buffers here pins ~75MB and hangs the RK3588. */
    v4l2sl_bind_capture_export(ctx);
    return 0;
}

/*
 * Read back the driver-chosen capture geometry.
 * The stride can be larger than the width (alignment) — image export must
 * use these values, not the display dimensions.
 */
int v4l2sl_get_capture_geometry(int fd, uint32_t *w, uint32_t *h,
                                uint32_t *stride, uint32_t *sizeimage)
{
    struct v4l2_format fmt = { 0 };
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        fprintf(stderr, "v4l2stateless: G_FMT capture failed: %s\n", strerror(errno));
        return -1;
    }

    *w = fmt.fmt.pix_mp.width;
    *h = fmt.fmt.pix_mp.height;
    *stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    *sizeimage = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    return 0;
}



static void v4l2sl_unmap_capture_buffers(struct v4l2sl_context *ctx)
{
    int i;

    if (!ctx)
        return;
    for (i = 0; i < V4L2SL_NUM_CAPTURE_BUFS; i++) {
        if (ctx->capture_buf_ptr[i] && ctx->capture_buf_ptr[i] != MAP_FAILED) {
            if (ctx->capture_buf_anon)
                free(ctx->capture_buf_ptr[i]);
            else
                munmap(ctx->capture_buf_ptr[i], ctx->capture_buf_size);
        }
        ctx->capture_buf_ptr[i] = NULL;
        if (ctx->capture_uv_ptr[i] && ctx->capture_uv_ptr[i] != MAP_FAILED) {
            if (ctx->capture_buf_anon)
                free(ctx->capture_uv_ptr[i]);
            else
                munmap(ctx->capture_uv_ptr[i], ctx->capture_uv_size);
        }
        ctx->capture_uv_ptr[i] = NULL;
    }
    ctx->capture_buf_size = 0;
    ctx->capture_uv_size = 0;
    ctx->capture_buf_anon = 0;
}

static int v4l2sl_mmap_one_capture(struct v4l2sl_context *ctx, int index)
{
    struct v4l2_buffer buf = { 0 };
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = { 0 };
    void *ptr, *uv_ptr = NULL;
    uint32_t length, uv_len = 0;
    int anon, npl;

    if (!ctx || ctx->v4l2_fd < 0 || index < 0 ||
        index >= ctx->capture_bufs_allocd || index >= V4L2SL_NUM_CAPTURE_BUFS)
        return -1;
    if (ctx->capture_buf_ptr[index] && ctx->capture_buf_ptr[index] != MAP_FAILED)
        return 0;

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    buf.length = VIDEO_MAX_PLANES;
    buf.m.planes = planes;
    if (xioctl(ctx->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
        fprintf(stderr, "v4l2stateless: QUERYBUF capture[%d] failed: %s\n",
                index, strerror(errno));
        return -1;
    }
    npl = (int)buf.length;
    length = planes[0].length;
    if (!length || npl < 1)
        return -1;
    if (npl >= 2)
        uv_len = planes[1].length;
    anon = g_ioctl_hook ? 1 : 0;

    if (anon) {
        ptr = calloc(1, length);
        if (!ptr)
            return -1;
        if (uv_len) {
            uv_ptr = calloc(1, uv_len);
            if (!uv_ptr) { free(ptr); return -1; }
        }
    } else {
        ptr = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                   ctx->v4l2_fd, planes[0].m.mem_offset);
        if (ptr == MAP_FAILED) {
            fprintf(stderr, "v4l2stateless: mmap capture[%d] failed: %s\n",
                    index, strerror(errno));
            return -1;
        }
        if (uv_len) {
            /* UV is a separate CMA block; map it at its own offset. */
            uv_ptr = mmap(NULL, uv_len, PROT_READ | PROT_WRITE, MAP_SHARED,
                          ctx->v4l2_fd, planes[1].m.mem_offset);
            if (uv_ptr == MAP_FAILED) {
                munmap(ptr, length);
                fprintf(stderr, "v4l2stateless: mmap capture[%d] uv failed: %s\n",
                        index, strerror(errno));
                return -1;
            }
        }
        if (v4l2sl_debug)
            fprintf(stderr, "v4l2stateless: capture mmap idx=%d y=%u uv=%u\n",
                    index, length, uv_len);
    }
    ctx->capture_buf_ptr[index] = ptr;
    ctx->capture_buf_size = length;
    ctx->capture_uv_ptr[index] = uv_ptr;
    ctx->capture_uv_size = uv_len;
    ctx->capture_buf_anon = anon;
    return 0;
}

/*
 * Mmap all output buffers after REQBUFS
 * ptrs[] must be at least V4L2SL_NUM_OUTPUT_BUFS
 * Returns number of mmap'd buffers or -1 on error
 */
int v4l2sl_mmap_output_buffers(int fd, int count, void **ptrs, uint32_t *size_out)
{
    if (count > V4L2SL_NUM_OUTPUT_BUFS)
        count = V4L2SL_NUM_OUTPUT_BUFS;

    for (int i = 0; i < count; i++) {
        struct v4l2_buffer buf = { 0 };
        struct v4l2_plane planes[1] = { 0 };

        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = 1;
        buf.m.planes = planes;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "v4l2stateless: QUERYBUF output[%d] failed: %s\n", i, strerror(errno));
            return -1;
        }

        void *ptr = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, planes[0].m.mem_offset);
        if (ptr == MAP_FAILED) {
            fprintf(stderr, "v4l2stateless: mmap output[%d] failed: %s\n", i, strerror(errno));
            return -1;
        }

        ptrs[i] = ptr;
        if (i == 0 && size_out)
            *size_out = planes[0].length;
    }

    return count;
}

/*
 * Queue a compressed bitstream buffer to the output queue.
 * The bitstream was already memcpy'd into the pre-mapped output buffer by
 * the caller — only its length travels with the QBUF.
 */
int v4l2sl_queue_output(struct v4l2sl_context *ctx, int buf_index,
                        uint32_t size, uint64_t timestamp)
{
    struct v4l2_buffer buf = { 0 };
    struct v4l2_plane planes[1] = { 0 };
    int fd = ctx->v4l2_fd;
    int request_fd = ctx->request_fd;

    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buf_index;
    buf.length = 1;
    buf.m.planes = planes;
    buf.request_fd = request_fd;
    buf.flags = V4L2_BUF_FLAG_REQUEST_FD;

    /* Timestamp is in nanoseconds (µs-aligned). vb2 stores the timeval
     * internally as ns again, so this must round-trip losslessly to match
     * DPB reference_ts values — which live in the same ns domain. */
    buf.timestamp.tv_sec = timestamp / 1000000000ULL;
    buf.timestamp.tv_usec = (timestamp % 1000000000ULL) / 1000ULL;

    /* hantro AV1 rejects QBUF unless plane length matches the mapped size.
     * Lengths are fixed per REQBUFS cycle — memoize per index. */
    if (buf_index >= 0 && buf_index < V4L2SL_NUM_OUTPUT_BUFS &&
        ctx->output_plane_len[buf_index]) {
        planes[0].length = ctx->output_plane_len[buf_index];
    } else {
        struct v4l2_buffer q = { 0 };
        struct v4l2_plane qp[1] = { 0 };
        q.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        q.memory = V4L2_MEMORY_MMAP;
        q.index = buf_index;
        q.length = 1;
        q.m.planes = qp;
        if (xioctl(fd, VIDIOC_QUERYBUF, &q) == 0) {
            planes[0].length = qp[0].length;
            if (buf_index >= 0 && buf_index < V4L2SL_NUM_OUTPUT_BUFS)
                ctx->output_plane_len[buf_index] = qp[0].length;
        }
    }
    planes[0].bytesused = size;

    if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "v4l2stateless: QBUF output[%d] failed: %s\n", buf_index, strerror(errno));
        return -1;
    }

    return 0;
}

/*
 * Queue a capture buffer (decoded frame output)
 */
int v4l2sl_queue_capture(int fd, int buf_index, int request_fd)
{
    struct v4l2_buffer buf = { 0 };
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = { 0 };

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buf_index;

    /* Multiplanar QBUF requires buf.length == the plane count. MM21 is 2
     * planes (Y + UV); a bare 1 makes vb2 reject the QBUF with EINVAL. Query
     * the buffer to learn the count (works for NV12's 1 plane too). */
    {
        struct v4l2_buffer q = { 0 };
        struct v4l2_plane qp[VIDEO_MAX_PLANES] = { 0 };
        q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        q.memory = V4L2_MEMORY_MMAP;
        q.index = buf_index;
        q.length = VIDEO_MAX_PLANES;
        q.m.planes = qp;
        if (xioctl(fd, VIDIOC_QUERYBUF, &q) < 0) {
            fprintf(stderr, "v4l2stateless: QUERYBUF capture[%d] failed: %s\n",
                    buf_index, strerror(errno));
            return -1;
        }
        buf.length = q.length;
    }
    buf.m.planes = planes;
    /* request_fd intentionally NOT set: only OUTPUT (bitstream) buffers are
     * request objects. The CAPTURE queue does not support requests — binding
     * one makes vb2 fail QBUF with EPERM. Capture buffers are queued bare;
     * the decoder fills whichever capture buffer is available when the
     * request on the OUTPUT side completes. */
    (void)request_fd;

    if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "v4l2stateless: QBUF capture[%d] failed: %s\n", buf_index, strerror(errno));
        return -1;
    }

    return 0;
}

int v4l2sl_surface_alloc_export_fd(struct v4l2sl_surface *s)
{
    uint32_t stride, h, sz;
    int fd;

    if (!s || s->width == 0 || s->height == 0)
        return -1;
    if (s->memfd_fd >= 0)
        return 0;

    stride = s->cpu_stride ? s->cpu_stride : s->width;
    h = s->height;
    sz = v4l2sl_va_image_size(s->format ? s->format : VA_FOURCC_NV12, stride, h);
    if (!sz)
        sz = stride * h * 3 / 2;

    fd = memfd_create("v4l2sl-surf", MFD_CLOEXEC);
    if (fd < 0)
        return -1;
    if (ftruncate(fd, (off_t)sz) < 0) {
        close(fd);
        return -1;
    }
    s->memfd_fd = fd;
    s->memfd_size = sz;
    if (!s->stride)
        s->stride = stride;
    if (!s->aligned_h)
        s->aligned_h = h;
    if (!s->cap_fourcc)
        s->cap_fourcc = V4L2_PIX_FMT_NV12;
    return 0;
}

int v4l2sl_surface_ensure_cpu(struct v4l2sl_surface *s)
{
    if (!s || !s->cpu_size)
        return -1;
    if (!s->cpu_ptr) {
        s->cpu_ptr = calloc(1, s->cpu_size);
        if (!s->cpu_ptr)
            return -1;
    }
    return 0;
}

/* Grow-only: a smaller `size` (resolution step-down) must never ftruncate —
 * clients may still hold mappings or dup'd fds of the larger memfd, and
 * shrinking drops pages under them (SIGBUS). */
int v4l2sl_surface_grow_memfd(struct v4l2sl_surface *s, uint32_t size)
{
    if (!s)
        return -1;
    if (s->memfd_fd < 0 && v4l2sl_surface_alloc_export_fd(s) < 0)
        return -1;
    if (size == 0 || s->memfd_fd < 0 || size <= s->memfd_size)
        return 0;
    if (ftruncate(s->memfd_fd, (off_t)size) < 0)
        return -1;
    s->memfd_size = size;
    return 0;
}

/* Persistent per-surface mapping of memfd_fd (RW shared). Grow-only:
 * when `need` exceeds the cached length the mapping is grown — via
 * mremap, or when derived images still borrow the current mapping (a
 * moving remap would pull the pages from under them) by parking the old
 * one as "retired" until the last borrower releases it. Callers must
 * NOT munmap the result — the surface owns it. NULL on failure. */
void *v4l2sl_surface_map_memfd(struct v4l2sl_surface *s, uint32_t need)
{
    void *nm;

    if (!s || s->memfd_fd < 0 || need == 0)
        return NULL;
    if (s->memfd_map && s->memfd_map_size >= need)
        return s->memfd_map;
    if (need > s->memfd_size && v4l2sl_surface_grow_memfd(s, need) < 0)
        return NULL;
    if (s->memfd_map) {
        if (s->memfd_borrows) {
            if (s->memfd_retired)
                munmap(s->memfd_retired, s->memfd_retired_size);
            s->memfd_retired = s->memfd_map;
            s->memfd_retired_size = s->memfd_map_size;
            nm = mmap(NULL, need, PROT_READ | PROT_WRITE, MAP_SHARED,
                      s->memfd_fd, 0);
        } else {
            nm = mremap(s->memfd_map, s->memfd_map_size, need,
                        MREMAP_MAYMOVE);
        }
        if (nm == MAP_FAILED)
            return NULL;
    } else {
        nm = mmap(NULL, need, PROT_READ | PROT_WRITE, MAP_SHARED,
                  s->memfd_fd, 0);
        if (nm == MAP_FAILED)
            return NULL;
    }
    s->memfd_map = nm;
    s->memfd_map_size = need;
    return s->memfd_map;
}

static int pull_capture_inner(struct v4l2sl_context *ctx,
                             struct v4l2sl_surface *surf, int buf_index);

int v4l2sl_surface_pull_capture(struct v4l2sl_context *ctx,
                                struct v4l2sl_surface *surf, int buf_index)
{
    int64_t t0, t1;
    int ret;

    if (!g_prof.enabled)
        return pull_capture_inner(ctx, surf, buf_index);
    t0 = prof_now_us();
    ret = pull_capture_inner(ctx, surf, buf_index);
    t1 = prof_now_us();
    prof_frame(g_prof_pending_frame_us ? g_prof_pending_frame_us : t0, t1,
               g_prof_pending_decode, g_prof_pending_detile,
               (t1 - t0) / 1000.0);
    g_prof_pending_decode = -1;
    g_prof_pending_detile = -1;
    return ret;
}

static int pull_capture_inner(struct v4l2sl_context *ctx,
                             struct v4l2sl_surface *surf, int buf_index)
{
    uint32_t sz, stride, alh, fcc;
    void *src, *dst;

    int hw_try, mmapped = 0;

    if (!ctx || !surf || ctx->v4l2_fd < 0 || buf_index < 0)
        return -1;
    /* Remember which context allocated this slot so surface destroy can
     * return it to the right pool (see destroy_surface_locked). */
    surf->capture_owner = ctx;

    stride = ctx->cap_stride ? ctx->cap_stride : (uint32_t)ctx->width;
    alh = ctx->cap_height ? ctx->cap_height : (uint32_t)ctx->height;
    fcc = ctx->cap_pixelformat ? ctx->cap_pixelformat : V4L2_PIX_FMT_NV12;

    /*
     * The hardware de-tile reads the capture buffer through dma-bufs, never
     * through the CPU mapping. Mapping a slot faults in ~2 MB of CMA that
     * this path will not touch, so defer it until the software de-tile is
     * actually needed (hardware disabled, failed, or a non-MM21 format).
     */
    hw_try = (fcc == V4L2_PIX_FMT_MM21) && surf->gbm_bo &&
             !ctx->mdp3.disabled && v4l2sl_mdp3_requested();
    if (!hw_try) {
        if (v4l2sl_mmap_one_capture(ctx, buf_index) < 0)
            return -1;
        if (!ctx->capture_buf_ptr[buf_index] ||
            ctx->capture_buf_ptr[buf_index] == MAP_FAILED)
            return -1;
        mmapped = 1;
    }

    if (fcc == V4L2_PIX_FMT_MM21) {
        /*
         * MT8183 decoder emits tiled MM21 (Y + UV in separate CMA blocks),
         * which Chromium/GPU can't consume. De-tile to linear NV12 in the
         * memfd snapshot — the EXPBUF/GBM zero-copy paths can't carry a
         * tiled format. Output is stride*alh*3/2 (NV12), larger than the Y
         * plane alone, so size the memfd to the de-tiled size.
         */
        v4l2sl_explog("v4l2stateless: pull_capture MM21 idx=%d\n", buf_index);
        surf->has_pic = 1;
        surf->buf_index = buf_index;
        surf->stride = stride;
        surf->aligned_h = alh;
        surf->cap_fourcc = V4L2_PIX_FMT_NV12;   /* de-tiled to linear NV12 */
        surf->de_tiled = 1;
        int64_t t_det0 = g_prof.enabled ? prof_now_us() : 0;
#define PROF_DETILE_END()                                                    \
        do {                                                                 \
            if (t_det0)                                                      \
                g_prof_pending_detile = (prof_now_us() - t_det0) / 1000.0;    \
        } while (0)

        /*
         * HW de-tile (MDP3 m2m VPU): export the decoder's MM21 capture planes
         * as dma-bufs and import them into the m2m OUTPUT queue, so the CPU
         * never touches the tiled CMA. The m2m's linear NV12 lands in the
         * surface bo. Lazy-init on the first MM21 frame; any init failure or
         * 3 consecutive frame failures disable it for the session and the
         * software de-tile below takes over.
         */
        if (surf->gbm_bo && !ctx->mdp3.disabled) {
            if (!ctx->mdp3.active) {
                ctx->mdp3.active = 1; /* init attempted */
                if (v4l2sl_mdp3_init(ctx, (int)stride, (int)alh) < 0)
                    ctx->mdp3.disabled = 1;
            }
            if (!ctx->mdp3.disabled && v4l2sl_gbm_detile_async_wanted()) {
                /* MDP3 imports the surface bo directly (zero-copy). The decoder
                 * planes are exported here so the worker never issues ioctls on
                 * the decoder fd while the next frame is being decoded. */
                int y_fd = v4l2sl_capture_expbuf_plane(ctx, buf_index, 0);
                int uv_fd = y_fd >= 0
                                ? v4l2sl_capture_expbuf_plane(ctx, buf_index, 1)
                                : -1;

                if (uv_fd >= 0 &&
                    v4l2sl_gbm_detile_async_hw(ctx, surf, y_fd, uv_fd) == 0) {
                    g_prof_pending_detile = 0.0;
                    surf->last_writer = V4L2SL_WRITER_BO;
                    return 0;
                }
                if (y_fd >= 0)
                    close(y_fd);
                if (uv_fd >= 0)
                    close(uv_fd);
            }
            if (!ctx->mdp3.disabled) {
                if (v4l2sl_mdp3_detile(ctx, buf_index, surf) == 0) {
                    PROF_DETILE_END();
                    surf->last_writer = V4L2SL_WRITER_BO;
                    return 0;
                }
            }
        }

        /* Software fallback: now the capture buffer must be CPU-mapped. */
        if (!mmapped) {
            if (v4l2sl_mmap_one_capture(ctx, buf_index) < 0)
                return -1;
            if (!ctx->capture_buf_ptr[buf_index] ||
                ctx->capture_buf_ptr[buf_index] == MAP_FAILED ||
                !ctx->capture_uv_ptr[buf_index])
                return -1;
            mmapped = 1;
        }
        src = ctx->capture_buf_ptr[buf_index];
        sz = ctx->capture_buf_size;
        if (ctx->cap_sizeimage && ctx->cap_sizeimage < sz)
            sz = ctx->cap_sizeimage;
        if (!sz)
            return -1;

        if (surf->gbm_bo && v4l2sl_gbm_detile_async_wanted() &&
            v4l2sl_gbm_surface_detile_async(surf, src,
                                            ctx->capture_uv_ptr[buf_index],
                                            stride, alh) == 0) {
            /* No inline de-tile: the job runs on the worker thread and its
             * cost is in the job_* trace columns. */
            g_prof_pending_detile = 0.0;
            surf->last_writer = V4L2SL_WRITER_BO;
            return 0;
        }

        /*
         * De-tile straight into the exported bo when there is one: the bo is
         * what Chromium samples, so writing the memfd and then copying it
         * into the bo costs an extra full-frame pass. The memfd stays a lazy
         * CPU-readback snapshot, refilled from the bo on demand by
         * v4l2sl_surface_ensure_memfd() (vaGetImage / vaDeriveImage).
         * Measured at 1080p: 13.6-16.4 ms/frame direct vs 16.4-18.0 ms for
         * de-tile -> memfd plus 1.5 ms memfd -> bo.
         */
        if (surf->gbm_bo &&
            v4l2sl_gbm_surface_detile_mm21(surf, src,
                                           ctx->capture_uv_ptr[buf_index],
                                           stride, alh) == 0) {
            PROF_DETILE_END();
            surf->last_writer = V4L2SL_WRITER_BO;
            return 0;
        }

        {
            uint32_t out_sz = stride * alh * 3 / 2;

            if (v4l2sl_surface_grow_memfd(surf, out_sz) < 0)
                return -1;
            dst = v4l2sl_surface_map_memfd(surf, out_sz);
            if (!dst)
                return -1;
        }
        v4l2sl_mm21_detile(src, ctx->capture_uv_ptr[buf_index],
                           (uint8_t *)dst, stride, alh);
        PROF_DETILE_END();
        surf->last_writer = V4L2SL_WRITER_MEMFD;
        /* No bo yet (or direct write refused): push the de-tiled frame into
         * the bo if one exists, else the memfd is the only copy. */
        if (surf->gbm_bo)
            (void)v4l2sl_gbm_surface_sync(surf);
        return 0;
    }
    /* Non-tiled capture (linear formats): hand out the capture mapping
     * directly, so the slot must be mapped here. */
    if (!mmapped) {
        if (v4l2sl_mmap_one_capture(ctx, buf_index) < 0)
            return -1;
        if (!ctx->capture_buf_ptr[buf_index] ||
            ctx->capture_buf_ptr[buf_index] == MAP_FAILED)
            return -1;
    }
    src = ctx->capture_buf_ptr[buf_index];
    sz = ctx->capture_buf_size;
    if (ctx->cap_sizeimage && ctx->cap_sizeimage < sz)
        sz = ctx->cap_sizeimage;
    if (!sz)
        return -1;
    surf->de_tiled = 0;

    if (v4l2sl_expbuf_export_wanted()) {
        /* Skip capture→GBM/memfd memcpy. Chrome EXPBUFs this index.
         * GetImage/Derive read capture_buf_ptr[buf_index] while it is live. */
        surf->has_pic = 1;
        surf->buf_index = buf_index;
        surf->stride = stride;
        surf->aligned_h = alh;
        surf->cap_fourcc = fcc;
        surf->cap_view = src;
        surf->last_writer = V4L2SL_WRITER_MEMFD;
        return 0;
    }

    /*
     * Lazy memfd: when this surface has a GBM display bo, the bo is the
     * per-frame snapshot and the memfd copy is skipped — CPU-readback
     * callers (vaGetImage / vaDeriveImage / VPP source) refill it on
     * demand via v4l2sl_surface_ensure_memfd(). Halves the per-frame
     * memcpy traffic for zero-copy clients (Chrome never reads back).
     * On upload failure fall back to the classic memfd snapshot so the
     * frame is never lost (the capture buffer recycles right after this).
     */
    if (surf->gbm_bo) {
        if (v4l2sl_gbm_surface_upload(surf, src, stride, alh) == 0) {
            surf->has_pic = 1;
            surf->buf_index = buf_index;
            surf->stride = stride;
            surf->aligned_h = alh;
            surf->cap_fourcc = fcc;
            surf->last_writer = V4L2SL_WRITER_BO;
            return 0;
        }
        fprintf(stderr,
                "v4l2stateless: gbm upload failed, using memfd snapshot\n");
    }

    if (v4l2sl_surface_grow_memfd(surf, sz) < 0)
        return -1;
    dst = v4l2sl_surface_map_memfd(surf, sz);
    if (!dst)
        return -1;
    memcpy(dst, src, sz);

    surf->has_pic = 1;
    surf->buf_index = buf_index;
    surf->stride = stride;
    surf->aligned_h = alh;
    surf->cap_fourcc = fcc;
    surf->last_writer = V4L2SL_WRITER_MEMFD;
    return 0;
}

int v4l2sl_bind_capture_export(struct v4l2sl_context *ctx)
{
    int i, n;
    uint32_t sz;

    if (!ctx || !ctx->driver_data || ctx->capture_bufs_allocd <= 0)
        return -1;

    sz = ctx->cap_sizeimage;
    if (!sz)
        sz = v4l2sl_capture_plane_size(ctx->cap_pixelformat ? ctx->cap_pixelformat
                                                            : V4L2_PIX_FMT_NV12,
                                       ctx->cap_stride, ctx->cap_height);

    n = ctx->num_render_targets;
    for (i = 0; i < n; i++) {
        VASurfaceID id;
        struct v4l2sl_surface *s;

        if (!ctx->render_targets)
            break;
        id = ctx->render_targets[i];
        s = v4l2sl_surface_by_id(ctx->driver_data, id);
        if (!s)
            continue;
        if (v4l2sl_surface_grow_memfd(s, sz) < 0)
            continue;
        s->stride = ctx->cap_stride;
        s->aligned_h = ctx->cap_height;
        s->cap_fourcc = ctx->cap_pixelformat;
    }
    return 0;
}

void v4l2sl_fill_prime_layers(VADRMPRIMESurfaceDescriptor *desc,
                              int fd, uint32_t object_size,
                              uint64_t modifier, uint32_t va_fourcc,
                              uint32_t width, uint32_t height,
                              uint32_t pitch, uint32_t chroma_row,
                              uint32_t combined_fmt, uint32_t luma_fmt,
                              uint32_t chroma_fmt, uint32_t flags)
{
    memset(desc, 0, sizeof(*desc));
    desc->fourcc = va_fourcc;
    desc->width = width;
    desc->height = height;
    desc->num_objects = 1;
    desc->objects[0].fd = fd;
    desc->objects[0].size = object_size;
    desc->objects[0].drm_format_modifier = modifier;

    if (flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) {
        desc->num_layers = 2;
        desc->layers[0].drm_format = luma_fmt;
        desc->layers[0].num_planes = 1;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].pitch[0] = pitch;
        desc->layers[1].drm_format = chroma_fmt;
        desc->layers[1].num_planes = 1;
        desc->layers[1].object_index[0] = 0;
        desc->layers[1].offset[0] = pitch * chroma_row;
        desc->layers[1].pitch[0] = pitch;
    } else {
        desc->num_layers = 1;
        desc->layers[0].drm_format = combined_fmt;
        desc->layers[0].num_planes = 2;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].object_index[1] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].offset[1] = pitch * chroma_row;
        desc->layers[0].pitch[0] = pitch;
        desc->layers[0].pitch[1] = pitch;
    }
}

VAStatus v4l2sl_surface_fill_prime(const struct v4l2sl_surface *surf,
                                   const struct v4l2sl_context *c,
                                   uint32_t flags,
                                   void *descriptor)
{
    VADRMPRIMESurfaceDescriptor *desc = descriptor;
    uint32_t stride, alh, cap_fcc, drm_fcc, plane_size;
    int fd;

    if (!surf || !descriptor)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (surf->memfd_fd < 0)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    stride = surf->stride ? surf->stride :
             (c && c->cap_stride ? c->cap_stride : surf->width);
    alh = surf->aligned_h ? surf->aligned_h :
          (c && c->cap_height ? c->cap_height : surf->height);
    cap_fcc = surf->cap_fourcc ? surf->cap_fourcc :
              (c && c->cap_pixelformat ? c->cap_pixelformat : V4L2_PIX_FMT_NV12);
    drm_fcc = v4l2sl_drm_fourcc_for_capture(cap_fcc);
    plane_size = v4l2sl_capture_plane_size(cap_fcc, stride, alh);

    fd = dup(surf->memfd_fd);
    if (fd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;

    v4l2sl_fill_prime_layers(desc, fd, plane_size, DRM_FORMAT_MOD_LINEAR,
                             (cap_fcc == V4L2_PIX_FMT_NV12) ?
                                 VA_FOURCC_NV12 :
                                 v4l2sl_va_fourcc_for_capture(cap_fcc),
                             surf->width, surf->height, stride, alh,
                             drm_fcc,
                             (cap_fcc == V4L2_PIX_FMT_NV15) ?
                                 DRM_FORMAT_R16 : DRM_FORMAT_R8,
                             (cap_fcc == V4L2_PIX_FMT_NV15) ?
                                 DRM_FORMAT_GR1616 : DRM_FORMAT_GR88,
                             flags);
    return VA_STATUS_SUCCESS;
}

/*
 * Dequeue a buffer from the given queue type
 * Returns buffer index or -1 on error
 */
/* flag_error (optional) is set when the dequeued buffer carried
 * V4L2_BUF_FLAG_ERROR — the index is still returned so the caller can
 * recycle the slot; the data itself must not be used. */
int v4l2sl_dequeue_buffer(int fd, enum v4l2_buf_type type, int *flag_error)
{
    struct v4l2_buffer buf = { 0 };
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = { 0 };

    buf.type = type;
    buf.memory = V4L2_MEMORY_MMAP;
    /* Multiplanar DQBUF requires buf.length >= the plane count (MM21 = 2);
     * the kernel fills in the actual count. VIDEO_MAX_PLANES covers all. */
    buf.length = VIDEO_MAX_PLANES;
    buf.m.planes = planes;

    if (flag_error)
        *flag_error = 0;
    if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN)
            fprintf(stderr, "v4l2stateless: DQBUF failed: %s\n", strerror(errno));
        return -1;
    }
    if (buf.flags & V4L2_BUF_FLAG_ERROR) {
        fprintf(stderr, "v4l2stateless: DQBUF type=%u idx=%u ERROR flags=0x%x bytesused=%u\n",
                (unsigned)type, buf.index, buf.flags, planes[0].bytesused);
        if (flag_error)
            *flag_error = 1;
    }

    return buf.index;
}

/*
 * Set V4L2 extended controls bound to a request.
 * Uses V4L2_CTRL_WHICH_REQUEST_VAL so the controls apply to this specific request
 * rather than being global defaults.
 *
 * request_fd: from MEDIA_IOC_REQUEST_ALLOC
 * v4l2_fd:    the video device fd (required by V4L2 to identify the device)
 * ctrls:      caller-allocated v4l2_ext_controls (may contain multiple controls)
 *
 * Returns 0 on success, -1 on error.
 */
int v4l2sl_set_request_controls(int request_fd, int v4l2_fd,
                                struct v4l2_ext_controls *ctrls)
{
    /*
     * Modern UAPI: `which` selects the request *mode* and the fd goes in its
     * own `request_fd` field. (The old scheme of OR-ing the fd into `which`
     * no longer matches the kernel's `cs->which == V4L2_CTRL_WHICH_REQUEST_VAL`
     * check, so the controls silently fell through to the current/global path:
     * the request got no controls object and the driver's
     * v4l2_ctrl_request_complete() tripped the media_request_object_bind WARN.)
     */
    ctrls->which = V4L2_CTRL_WHICH_REQUEST_VAL;
    ctrls->request_fd = request_fd;

    if (xioctl(v4l2_fd, VIDIOC_S_EXT_CTRLS, ctrls) < 0) {
        fprintf(stderr, "v4l2stateless: S_EXT_CTRLS (request_fd=%d) failed: %s "
                "(%d controls, first_id=0x%x)\n",
                request_fd, strerror(errno),
                ctrls->count, ctrls->count > 0 ? ctrls->controls[0].id : 0);
        return -1;
    }

    return 0;
}

/*
 * Set extended controls as plain (non-request) values. Some codecs bind
 * stream-level controls globally — e.g. the rkvdec HEVC SPS rejects the
 * request-scoped variant with EINVAL.
 */
int v4l2sl_set_global_controls(int v4l2_fd, struct v4l2_ext_controls *ctrls)
{
    ctrls->which = 0;  /* V4L2_CTRL_WHICH_CUR_VAL */

    if (xioctl(v4l2_fd, VIDIOC_S_EXT_CTRLS, ctrls) < 0) {
        fprintf(stderr, "v4l2stateless: S_EXT_CTRLS (global) failed: %s\n",
                strerror(errno));
        return -1;
    }
    return 0;
}

/* Queue the request; the decoder consumes the controls bound to it.
 * Returns 0 on success, -1 on error. */
int v4l2sl_submit_request(int request_fd)
{
    if (xioctl(request_fd, MEDIA_REQUEST_IOC_QUEUE, NULL) < 0) {
        fprintf(stderr, "v4l2stateless: REQUEST_IOC_QUEUE failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

void v4l2sl_explog(const char *fmt, ...)
{
    va_list ap;

    if (!v4l2sl_debug)
        return;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
}

int v4l2sl_expbuf_export_wanted(void)
{
    static int once, wanted;
    if (!once) {
        const char *e = getenv("V4L2SL_EXPBUF_EXPORT");

        /* Default ON after Chrome zero-copy proof. Opt out with =0. */
        wanted = 1;
        if (e && e[0] == '0')
            wanted = 0;
        once = 1;
        if (v4l2sl_debug)
            fprintf(stderr, "v4l2stateless: EXPBUF wanted=%d pid=%d\n",
                    wanted, getpid());
    }
    return wanted;
}

int v4l2sl_capture_expbuf(struct v4l2sl_context *ctx, int buf_index)
{
    struct v4l2_exportbuffer exp;

    if (!ctx || ctx->v4l2_fd < 0 || buf_index < 0)
        return -1;
    memset(&exp, 0, sizeof(exp));
    exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    exp.index = (unsigned)buf_index;
    exp.plane = 0;
    exp.flags = O_CLOEXEC | O_RDWR;
    if (v4l2sl_xioctl(ctx->v4l2_fd, VIDIOC_EXPBUF, &exp) < 0) {
        fprintf(stderr, "v4l2stateless: EXPBUF capture[%d] failed: %s\n",
                buf_index, strerror(errno));
        return -1;
    }
    return exp.fd;
}

int v4l2sl_capture_expbuf_plane(struct v4l2sl_context *ctx, int buf_index,
                                int plane)
{
    struct v4l2_exportbuffer exp;

    if (!ctx || ctx->v4l2_fd < 0 || buf_index < 0 || plane < 0)
        return -1;
    memset(&exp, 0, sizeof(exp));
    exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    exp.index = (unsigned)buf_index;
    exp.plane = (unsigned)plane;
    exp.flags = O_CLOEXEC | O_RDWR;
    if (v4l2sl_xioctl(ctx->v4l2_fd, VIDIOC_EXPBUF, &exp) < 0)
        return -1;
    return exp.fd;
}

int v4l2sl_claim_capture_for_export(struct v4l2sl_context *ctx,
                                    struct v4l2sl_surface *surf)
{
    int idx, w, h;
    uint32_t fourcc;

    if (!ctx || !surf || ctx->v4l2_fd < 0)
        return -1;
    w = ctx->width > 0 ? ctx->width : (int)surf->width;
    h = ctx->height > 0 ? ctx->height : (int)surf->height;
    fourcc = ctx->cap_pixelformat ? ctx->cap_pixelformat : V4L2_PIX_FMT_NV12;
    if (v4l2sl_ensure_capture(ctx, w, h, fourcc) < 0) {
        fprintf(stderr, "v4l2stateless: claim capture: setup failed surf=%u\n",
                (unsigned)surf->surface_id);
        return -1;
    }
    if (surf->buf_index >= 0)
        return 0;
    if (ctx->n_free_cap == 0) {
        fprintf(stderr, "v4l2stateless: claim capture: no free slot surf=%u\n",
                (unsigned)surf->surface_id);
        return -1;
    }
    idx = ctx->free_cap_bufs[--ctx->n_free_cap];
    surf->buf_index = idx;
    surf->capture_owner = ctx;
    surf->stride = ctx->cap_stride;
    surf->aligned_h = ctx->cap_height;
    surf->cap_fourcc = ctx->cap_pixelformat;
    /* Do not mmap here: mapping the whole Chrome pool pins ~75MB CMA and
     * has hung this SoC. EXPBUF does not need a userspace mapping. */
    v4l2sl_explog("v4l2stateless: claim capture idx=%d surf=%u\n",
                  idx, (unsigned)surf->surface_id);
    return 0;
}
