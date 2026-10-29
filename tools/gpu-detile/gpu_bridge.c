/*
 * gpu_bridge.c — measure the bridge's own MM21 de-tile implementations on
 * real decoder dma-bufs, without Chromium or a VA client.
 *
 * Links libv4l2sl_core and drives:
 *   - v4l2sl_gpu_detile()                 (V4L2SL_DETILE=gpu)
 *   - v4l2sl_gbm_surface_detile_mm21()    (the CPU de-tiler, same geometry)
 * on planes EXPBUF'd from the real MT8183 decoder, and checks the GPU output
 * byte-for-byte against the CPU one.
 *
 * It needs the decoder to itself: it opens /dev/video5 by driver name and
 * runs S_FMT/REQBUFS/EXPBUF on the capture queue, so stop any VA client first
 * (Chromium, ffmpeg). With a client streaming, the setup silently produces a
 * different geometry and the plane import then fails with a misleading
 * EGL error.
 *
 * Run it with V4L2SL_DETILE=gpu so the surface bo gets the hardware layout.
 *
 *   gcc -O2 -Isrc -o /tmp/gpu_bridge tools/gpu-detile/gpu_bridge.c \
 *       builddir/libv4l2sl_core.a \
 *       $(pkg-config --cflags --libs libva gbm egl glesv2 libdrm) -lpthread
 *   V4L2SL_DETILE=gpu /tmp/gpu_bridge 1920 1080 200
 *
 * Expected output at 1080p: "correctness (visible rows): 0 bad bytes  EXACT",
 * with the GPU job around 0.6 ms wall / 0.6 ms process CPU per frame and the
 * CPU de-tiler around 2.4 ms (median; the first iterations pay page faults).
 *
 * Notes:
 *   - The CPU figure is an optimistic floor: it de-tiles into a persistent
 *     mapping (the bridge maps per frame), and it excludes the per-frame
 *     export/import the GPU job pays.
 *   - The GPU figure is the real job cost including the dma-buf write-fence
 *     wait, which is what the async worker spends.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <gbm.h>
#include <linux/videodev2.h>
#include "v4l2stateless.h"

static int xio(int fd, unsigned long r, void *p)
{
    while (1) {
        if (ioctl(fd, r, p) == 0)
            return 0;
        if (errno != EINTR)
            return -1;
    }
}

static int open_drv(const char *name)
{
    char path[32];

    for (int i = 0; i < 16; i++) {
        struct v4l2_capability c = { 0 };
        int fd;

        snprintf(path, sizeof(path), "/dev/video%d", i);
        fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;
        if (xio(fd, VIDIOC_QUERYCAP, &c) == 0 && strstr((char *)c.driver, name))
            return fd;
        close(fd);
    }
    return -1;
}

static double now_ms(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static double thread_cpu_ms(void)
{
    struct timespec t;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

/* Whole-process CPU: with the GPU job on its own thread, the calling thread's
 * CPU time is not the cost that matters. */
static double proc_cpu_ms(void)
{
    struct timespec t;

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;

    return (x > y) - (x < y);
}

static void stats(const char *what, double *v, int n)
{
    if (n <= 0)
        return;
    qsort(v, (size_t)n, sizeof(*v), cmp_double);
    printf("%-22s median %6.3f  p95 %6.3f  max %6.3f ms\n", what,
           v[n / 2], v[(int)(n * 0.95)], v[n - 1]);
}

int main(int argc, char **argv)
{
    int w = argc > 1 ? atoi(argv[1]) : 1920;
    int h = argc > 2 ? atoi(argv[2]) : 1080;
    int n = argc > 3 ? atoi(argv[3]) : 100;
    struct v4l2_format fm = { 0 }, cf = { 0 };
    struct v4l2_requestbuffers rb = { 0 };
    struct v4l2_buffer b = { 0 };
    struct v4l2_plane pl[2] = { { 0 } };
    struct v4l2_pix_format_mplane dc;
    struct v4l2_exportbuffer e = { 0 };
    struct v4l2sl_surface gpu_s = { 0 }, cpu_s = { 0 };
    uint8_t *ym, *um;
    double *gw, *gc, *cw, *cc;
    int dfd, y_fd, uv_fd, stride, ah, bad = 0, gok = 0;

    dfd = open_drv("mtk-vcodec-dec");
    if (dfd < 0) {
        perror("open decoder");
        return 1;
    }
    fm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fm.fmt.pix_mp.width = (uint32_t)w;
    fm.fmt.pix_mp.height = (uint32_t)h;
    fm.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    fm.fmt.pix_mp.num_planes = 1;
    fm.fmt.pix_mp.plane_fmt[0].sizeimage = (uint32_t)w * h * 15 / 8;
    xio(dfd, VIDIOC_S_FMT, &fm);
    rb.count = 1;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    rb.memory = V4L2_MEMORY_MMAP;
    xio(dfd, VIDIOC_REQBUFS, &rb);

    cf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    cf.fmt.pix_mp.width = (uint32_t)w;
    cf.fmt.pix_mp.height = (uint32_t)h;
    cf.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MM21;
    cf.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (xio(dfd, VIDIOC_S_FMT, &cf)) {
        perror("decoder S_FMT MM21");
        return 1;
    }
    dc = cf.fmt.pix_mp;
    rb.count = 1;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    rb.memory = V4L2_MEMORY_MMAP;
    rb.flags = V4L2_MEMORY_FLAG_NON_COHERENT;
    if (xio(dfd, VIDIOC_REQBUFS, &rb)) {
        rb.flags = 0;
        if (xio(dfd, VIDIOC_REQBUFS, &rb)) {
            perror("decoder REQBUFS capture");
            return 1;
        }
    }
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.index = 0;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = 2;
    b.m.planes = pl;
    if (xio(dfd, VIDIOC_QUERYBUF, &b)) {
        perror("QUERYBUF");
        return 1;
    }
    ym = mmap(0, pl[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, dfd,
              pl[0].m.mem_offset);
    um = mmap(0, pl[1].length, PROT_READ | PROT_WRITE, MAP_SHARED, dfd,
              pl[1].m.mem_offset);
    if (ym == MAP_FAILED || um == MAP_FAILED) {
        perror("mmap capture");
        return 1;
    }
    /* Deterministic tile content (the decoder is not running). */
    for (uint32_t i = 0; i < pl[0].length; i++)
        ym[i] = (uint8_t)((i * 131) >> 3);
    for (uint32_t i = 0; i < pl[1].length; i++)
        um[i] = (uint8_t)((i * 67) ^ 0x5a);

    stride = dc.plane_fmt[0].bytesperline ? (int)dc.plane_fmt[0].bytesperline
                                          : w;
    ah = dc.height;
    printf("capture: %ux%u stride=%d planes Y=%u UV=%u\n", dc.width, dc.height,
           stride, pl[0].length, pl[1].length);

    e.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    e.index = 0;
    e.plane = 0;
    e.flags = O_CLOEXEC | O_RDWR;
    if (xio(dfd, VIDIOC_EXPBUF, &e)) {
        perror("EXPBUF y");
        return 1;
    }
    y_fd = e.fd;
    memset(&e, 0, sizeof(e));
    e.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    e.index = 0;
    e.plane = 1;
    e.flags = O_CLOEXEC | O_RDWR;
    if (xio(dfd, VIDIOC_EXPBUF, &e)) {
        perror("EXPBUF uv");
        return 1;
    }
    uv_fd = e.fd;

    if (v4l2sl_detile_mode() != V4L2SL_DETILE_GPU) {
        fprintf(stderr, "run me with V4L2SL_DETILE=gpu (mode=%d)\n",
                (int)v4l2sl_detile_mode());
        return 1;
    }

    /* Two surfaces with the same (hardware) layout: GPU-written, CPU-written.
     * The source-texture cache is keyed by capture_owner, so the GPU surface
     * needs a live (dummy) context even though the decoder is not running. */
    struct v4l2sl_context dummy_ctx = { 0 };
    dummy_ctx.v4l2_fd = dfd;
    gpu_s.width = (uint32_t)w;
    gpu_s.height = (uint32_t)h;
    gpu_s.format = VA_FOURCC_NV12;
    gpu_s.cap_fourcc = V4L2_PIX_FMT_MM21;
    gpu_s.stride = (uint32_t)stride;
    gpu_s.aligned_h = (uint32_t)ah;
    gpu_s.capture_owner = &dummy_ctx;
    gpu_s.buf_index = 0;
    cpu_s = gpu_s;
    if (v4l2sl_gbm_surface_ensure(&gpu_s) ||
        v4l2sl_gbm_surface_ensure(&cpu_s)) {
        fprintf(stderr, "surface bo alloc failed\n");
        return 1;
    }
    printf("bo: stride=%u rows=%u\n", gpu_s.gbm_stride,
           gpu_s.bo_uv_rows + (gpu_s.bo_uv_rows + 1) / 2);

    /* Correctness: GPU job vs the CPU de-tiler into an identically shaped bo. */
    {
        int fd2 = dup(y_fd), fd3 = dup(uv_fd);

        if (v4l2sl_gpu_detile(&gpu_s, fd2, fd3) == 0)
            gok = 1;
        else
            fprintf(stderr, "gpu job failed\n");
    }
    v4l2sl_gpu_wait_surface(&gpu_s);   /* CPU readback below */
    v4l2sl_gbm_surface_detile_mm21(&cpu_s, ym, um, (uint32_t)stride,
                                   (uint32_t)ah);
    {
        uint32_t ps = 0, p2 = 0;
        void *d1 = NULL, *d2 = NULL;
        uint8_t *a = gbm_bo_map(gpu_s.gbm_bo, 0, 0, (uint32_t)w,
                                (uint32_t)(ah + ah / 2), GBM_BO_TRANSFER_READ,
                                &ps, &d1);
        uint8_t *c = gbm_bo_map(cpu_s.gbm_bo, 0, 0, (uint32_t)w,
                                (uint32_t)(ah + ah / 2), GBM_BO_TRANSFER_READ,
                                &p2, &d2);

        if (a && c) {
            /* Visible rows only: the shader also writes the padding rows
             * between the visible height and the aligned height, the CPU
             * de-tiler leaves them alone. */
            for (int y = 0; y < h; y++) {
                uint8_t *ra = a + (size_t)y * ps;
                uint8_t *rc = c + (size_t)y * p2;

                for (int x = 0; x < w; x++)
                    if (ra[x] != rc[x])
                        bad++;
            }
            for (int y = 0; y < (h + 1) / 2; y++) {
                uint8_t *ra = a + (size_t)ah * ps + (size_t)y * ps;
                uint8_t *rc = c + (size_t)ah * p2 + (size_t)y * p2;

                for (int x = 0; x < w; x++)
                    if (ra[x] != rc[x])
                        bad++;
            }
            printf("correctness (visible rows): %d bad bytes%s\n", bad,
                   bad ? "" : "  EXACT");
        } else {
            printf("correctness: map failed\n");
        }
        if (a)
            gbm_bo_unmap(gpu_s.gbm_bo, d1);
        if (c)
            gbm_bo_unmap(cpu_s.gbm_bo, d2);
    }

    if (!gok)
        return 1;

    gw = calloc((size_t)n, sizeof(*gw));
    gc = calloc((size_t)n, sizeof(*gc));
    cw = calloc((size_t)n, sizeof(*cw));
    cc = calloc((size_t)n, sizeof(*cc));
    if (!gw || !gc || !cw || !cc)
        return 1;

    {
        double p0 = proc_cpu_ms(), w0 = now_ms();

        for (int i = 0; i < n; i++)
            if (v4l2sl_gpu_detile(&gpu_s, dup(y_fd), dup(uv_fd)) < 0) {
                fprintf(stderr, "gpu job %d failed\n", i);
                return 1;
            }
        printf("gpu job, whole process: %.3f ms cpu, %.3f ms wall per frame\n",
               (proc_cpu_ms() - p0) / n, (now_ms() - w0) / n);
    }
    for (int i = 0; i < n; i++) {
        double w0 = now_ms(), c0 = thread_cpu_ms();

        if (v4l2sl_gpu_detile(&gpu_s, dup(y_fd), dup(uv_fd)) < 0) {
            fprintf(stderr, "gpu job %d failed\n", i);
            return 1;
        }
        gc[i] = thread_cpu_ms() - c0;
        gw[i] = now_ms() - w0;
    }
    {
        double p0 = proc_cpu_ms();

        for (int i = 0; i < n; i++)
            v4l2sl_gbm_surface_detile_mm21(&cpu_s, ym, um, (uint32_t)stride,
                                           (uint32_t)ah);
        printf("cpu de-tile, whole process: %.3f ms cpu per frame\n",
               (proc_cpu_ms() - p0) / n);
    }
    for (int i = 0; i < n; i++) {
        double c0 = thread_cpu_ms();

        v4l2sl_gbm_surface_detile_mm21(&cpu_s, ym, um, (uint32_t)stride,
                                       (uint32_t)ah);
        cc[i] = thread_cpu_ms() - c0;
        cw[i] = cc[i];
    }

    printf("\nper frame, %d iterations, %dx%d (aligned %dx%d):\n", n, w, h,
           stride, ah);
    stats("gpu job wall", gw, n);
    stats("gpu job cpu", gc, n);
    stats("cpu de-tile (mapped)", cw, n);

    v4l2sl_gpu_fini();
    return bad == 0 ? 0 : 1;
}
