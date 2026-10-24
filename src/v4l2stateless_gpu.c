/*
 * v4l2stateless_gpu.c — MM21 → NV12 de-tile on the Mali GPU (panfrost).
 *
 * Selected with V4L2SL_DETILE=gpu. One GLES3 fragment-shader pass reads the
 * two tiled decoder planes as R8 EGLImages imported from the decoder's
 * dma-bufs and writes linear NV12 into the surface's GBM bo, which is the
 * buffer Chromium already samples. No readback, no CPU-side copy of the
 * tiled source.
 *
 * Status: measured, not shipped. Correctness is byte-exact against the CPU
 * de-tiler, and on MT8183 at 1080p one job costs 5.5 ms of wall time and
 * ~1.1 ms of CPU (whole process) versus 1.8 ms / 2.0 ms for the software
 * de-tile and ~10 ms / ~1.1 ms for the MDP3 job. At 720p the CPU saving is
 * gone (~1.0 vs 1.1 ms) because the fixed cost dominates. The open question
 * is Chromium: a bridge-private EGLDisplay inside Chromium's GPU process has
 * produced green/split output before, and this path creates one.
 * V4L2SL_DETILE=gpu is therefore opt-in and falls back to the software
 * de-tile on any failure, so a broken GPU path costs one frame, not the
 * session.
 *
 * Synchronisation: the job ends with glFlush() plus a dma-buf write fence
 * (DMA_BUF_IOCTL_EXPORT_SYNC_FILE) that the job waits for before returning,
 * so the caller only ever sees a fully written bo. Handing the bo to
 * Chromium without waiting is what the implicit-fence experiments showed to
 * be correct but GPU-serialising; here the wait happens on the async worker
 * thread, which is idle anyway.
 *
 * Threading: all GL runs on one persistent thread. The async de-tile worker
 * is created per frame, and binding an EGL context on a fresh thread each
 * time costs ~1 ms of CPU in EGL/gallium thread state — as much as the
 * de-tile itself — so the job is posted to a long-lived thread instead. The
 * plane fds stay owned by the caller, which blocks until the job finishes.
 *
 * Layout (must match the CPU de-tiler in v4l2stateless_gbm.c):
 *   source Y  : 16x32-byte tiles (512 B), imported R8 512 x tiles, pitch 512
 *   source UV : 8x16 chroma samples (256 B) on the same tile grid
 *   dest      : R8, width = bo stride, height = aligned_h + aligned_h/2, with
 *               UV at row aligned_h (the hardware layout, see
 *               surface_uv_rows()).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <drm_fourcc.h>
#include <linux/dma-buf.h>
#include "v4l2stateless.h"

/* Entry points not necessarily linkable as plain symbols. */
static PFNEGLCREATEIMAGEKHRPROC g_create_image;
static PFNEGLDESTROYIMAGEKHRPROC g_destroy_image;
static PFNEGLGETPLATFORMDISPLAYEXTPROC g_get_platform_display;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC g_image_target_texture;

static const char *VS =
    "#version 300 es\n"
    "void main(){\n"
    "  vec2 p = vec2((gl_VertexID == 1) ? 3.0 : -1.0,\n"
    "                (gl_VertexID == 2) ? 3.0 : -1.0);\n"
    "  gl_Position = vec4(p, 0.0, 1.0);\n"
    "}\n";

/* Luma for the top aligned_h rows, interleaved chroma below. Verified
 * byte-exact against v4l2sl_mm21_detile(); do not "simplify" the tile math:
 * the chroma tile is 8x16 samples (256 B), i.e. 8 sample pairs per row, and
 * (x & 1) picks U or V out of the interleaved pair. */
static const char *FS =
    "#version 300 es\n"
    "precision highp float; precision highp int;\n"
    "uniform sampler2D u_y, u_uv; uniform int u_tc, u_ah; out vec4 o;\n"
    "void main(){\n"
    "  int x = int(gl_FragCoord.x), y = int(gl_FragCoord.y), t, of; float v;\n"
    "  if (y < u_ah) {\n"
    "    t = (y / 32) * u_tc + (x / 16);\n"
    "    of = (y % 32) * 16 + (x % 16);\n"
    "    v = texelFetch(u_y, ivec2(of, t), 0).r;\n"
    "  } else {\n"
    "    int cy = y - u_ah, cx = x / 2;\n"
    "    t = (cy / 16) * u_tc + (cx / 8);\n"
    "    of = (cy % 16) * 16 + (cx % 8) * 2 + (x & 1);\n"
    "    v = texelFetch(u_uv, ivec2(of, t), 0).r;\n"
    "  }\n"
    "  o = vec4(v, 0.0, 0.0, 1.0);\n"
    "}\n";

/* Destination images are per-surface and stable, so importing one per frame
 * would be pure overhead. Keyed by bo pointer + geometry; small fixed ring so
 * a client cycling many surfaces cannot grow it without bound. */
struct dst_entry {
    struct gbm_bo *bo;
    uint32_t stride;
    uint32_t rows;
    EGLImageKHR img;
    GLuint tex;
    GLuint fbo;
};
#define DST_ENTRIES 16

static struct {
    EGLDisplay dpy;
    EGLContext ctx;
    EGLConfig cfg;
    EGLSurface surf;
    GLuint prog;
    GLint u_y, u_uv, u_tc, u_ah;
    GLuint vao;
    int state;              /* 0 = uninit, 1 = ready, -1 = failed */
    struct dst_entry dst[DST_ENTRIES];
    int dst_next;
} g_gpu;

static pthread_mutex_t g_gpu_lock = PTHREAD_MUTEX_INITIALIZER;

/* Set once any job fails; the session stays on the software de-tile. */
static int g_gpu_failed;

static double gpu_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static GLuint gpu_shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    GLint ok = 0;

    if (!s)
        return 0;
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];

        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "v4l2stateless: gpu: shader compile failed: %s\n",
                log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static GLuint gpu_program(void)
{
    GLuint p, v, f;
    GLint ok = 0;

    p = glCreateProgram();
    v = gpu_shader(GL_VERTEX_SHADER, VS);
    f = gpu_shader(GL_FRAGMENT_SHADER, FS);
    if (!p || !v || !f) {
        if (p)
            glDeleteProgram(p);
        if (v)
            glDeleteShader(v);
        if (f)
            glDeleteShader(f);
        return 0;
    }
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    /* The shaders are owned by the program once linked. */
    glDeleteShader(v);
    glDeleteShader(f);
    if (!ok) {
        char log[512];

        glGetProgramInfoLog(p, sizeof(log), NULL, log);
        fprintf(stderr, "v4l2stateless: gpu: link failed: %s\n", log);
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

static EGLImageKHR gpu_image_r8(int fd, uint32_t w, uint32_t h, uint32_t pitch)
{
    EGLint attr[] = {
        EGL_WIDTH, (EGLint)w,
        EGL_HEIGHT, (EGLint)h,
        EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_R8,
        EGL_DMA_BUF_PLANE0_FD_EXT, fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)pitch,
        EGL_NONE,
    };

    return g_create_image(g_gpu.dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT,
                          (EGLClientBuffer)NULL, attr);
}

static GLuint gpu_texture(EGLImageKHR img)
{
    GLuint t = 0;

    glGenTextures(1, &t);
    if (!t)
        return 0;
    glBindTexture(GL_TEXTURE_2D, t);
    g_image_target_texture(GL_TEXTURE_2D, img);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return t;
}

/* Caller holds g_gpu_lock. */
static int gpu_init_locked(void)
{
    struct gbm_device *dev;
    EGLint maj = 0, min = 0, ncfg = 0;
    const char *renderer = NULL;
    static const EGLint cfg_attr[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_NONE
    };
    static const EGLint ctx_attr[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE
    };

    if (g_gpu.state > 0)
        return 0;
    if (g_gpu.state < 0)
        return -1;

    dev = v4l2sl_gbm_device_ref();
    if (!dev) {
        g_gpu.state = -1;
        return -1;
    }
    g_create_image = (PFNEGLCREATEIMAGEKHRPROC)
        eglGetProcAddress("eglCreateImageKHR");
    g_destroy_image = (PFNEGLDESTROYIMAGEKHRPROC)
        eglGetProcAddress("eglDestroyImageKHR");
    g_get_platform_display = (PFNEGLGETPLATFORMDISPLAYEXTPROC)
        eglGetProcAddress("eglGetPlatformDisplayEXT");
    g_image_target_texture = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)
        eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!g_create_image || !g_destroy_image || !g_get_platform_display ||
        !g_image_target_texture) {
        fprintf(stderr, "v4l2stateless: gpu: EGL/GL extension entry points "
                        "missing\n");
        g_gpu.state = -1;
        return -1;
    }

    g_gpu.dpy = g_get_platform_display(EGL_PLATFORM_GBM_KHR, dev, NULL);
    if (g_gpu.dpy == EGL_NO_DISPLAY || !eglInitialize(g_gpu.dpy, &maj, &min)) {
        fprintf(stderr, "v4l2stateless: gpu: no EGL display on the gbm device "
                        "(0x%x)\n", eglGetError());
        g_gpu.dpy = EGL_NO_DISPLAY;
        g_gpu.state = -1;
        return -1;
    }
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        fprintf(stderr, "v4l2stateless: gpu: eglBindAPI failed\n");
        g_gpu.state = -1;
        return -1;
    }
    g_gpu.cfg = NULL;
    if (eglChooseConfig(g_gpu.dpy, cfg_attr, &g_gpu.cfg, 1, &ncfg) && ncfg) {
        static const EGLint pb_attr[] = {
            EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE
        };

        g_gpu.surf = eglCreatePbufferSurface(g_gpu.dpy, g_gpu.cfg, pb_attr);
    }
    g_gpu.ctx = g_gpu.cfg
                    ? eglCreateContext(g_gpu.dpy, g_gpu.cfg, EGL_NO_CONTEXT,
                                       ctx_attr)
                    : EGL_NO_CONTEXT;
    if (g_gpu.ctx == EGL_NO_CONTEXT) {
        /* No config picked (surfaceless panfrost): configless context. */
        static const EGLint ca[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_CONTEXT_MAJOR_VERSION_KHR, 3,
            EGL_NONE
        };

        g_gpu.ctx = eglCreateContext(g_gpu.dpy, (EGLConfig)EGL_NO_CONFIG_KHR,
                                     EGL_NO_CONTEXT, ca);
    }
    if (g_gpu.ctx == EGL_NO_CONTEXT ||
        !eglMakeCurrent(g_gpu.dpy, g_gpu.surf, g_gpu.surf, g_gpu.ctx)) {
        fprintf(stderr, "v4l2stateless: gpu: no EGL context (0x%x)\n",
                eglGetError());
        g_gpu.state = -1;
        return -1;
    }
    g_gpu.prog = gpu_program();
    if (!g_gpu.prog) {
        g_gpu.state = -1;
        return -1;
    }
    g_gpu.u_y = glGetUniformLocation(g_gpu.prog, "u_y");
    g_gpu.u_uv = glGetUniformLocation(g_gpu.prog, "u_uv");
    g_gpu.u_tc = glGetUniformLocation(g_gpu.prog, "u_tc");
    g_gpu.u_ah = glGetUniformLocation(g_gpu.prog, "u_ah");
    glGenVertexArrays(1, &g_gpu.vao);
    renderer = (const char *)glGetString(GL_RENDERER);
    g_gpu.state = 1;
    fprintf(stderr, "v4l2stateless: gpu: de-tile ready on %s (GLES3)\n",
            renderer ? renderer : "unknown renderer");
    return 0;
}

/* Caller holds g_gpu_lock. Returns the FBO for this bo, importing it once. */
static GLuint gpu_dst_fbo(struct v4l2sl_surface *s, int bo_fd)
{
    uint32_t stride = s->gbm_stride;
    uint32_t rows = s->bo_uv_rows + (s->bo_uv_rows + 1) / 2;
    struct dst_entry *e = NULL;

    for (int i = 0; i < DST_ENTRIES; i++) {
        struct dst_entry *c = &g_gpu.dst[i];

        if (c->bo == s->gbm_bo && c->stride == stride && c->rows == rows) {
            e = c;
            break;
        }
    }
    if (!e) {
        /* Reuse the next slot; drop whatever image it held. */
        e = &g_gpu.dst[g_gpu.dst_next];
        g_gpu.dst_next = (g_gpu.dst_next + 1) % DST_ENTRIES;
        if (e->fbo)
            glDeleteFramebuffers(1, &e->fbo);
        if (e->tex)
            glDeleteTextures(1, &e->tex);
        if (e->img != EGL_NO_IMAGE_KHR)
            g_destroy_image(g_gpu.dpy, e->img);
        memset(e, 0, sizeof(*e));
        e->bo = s->gbm_bo;
        e->stride = stride;
        e->rows = rows;
        e->img = gpu_image_r8(bo_fd, stride, rows, stride);
        if (e->img == EGL_NO_IMAGE_KHR)
            return 0;
        e->tex = gpu_texture(e->img);
        if (!e->tex)
            return 0;
        glGenFramebuffers(1, &e->fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, e->fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, e->tex, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) !=
            GL_FRAMEBUFFER_COMPLETE) {
            fprintf(stderr, "v4l2stateless: gpu: incomplete FBO for the bo\n");
            return 0;
        }
    } else {
        glBindFramebuffer(GL_FRAMEBUFFER, e->fbo);
    }
    return e->fbo;
}

/* Wait for the GPU to stop writing the bo. Returns 0 when the bo is complete. */
static int gpu_wait_write_fence(int bo_fd, double *wait_ms)
{
    struct dma_buf_export_sync_file ex = {
        .flags = DMA_BUF_SYNC_WRITE,
        .fd = -1,
    };
    struct pollfd pf;
    double t0;
    int r;

    if (ioctl(bo_fd, DMA_BUF_IOCTL_EXPORT_SYNC_FILE, &ex) < 0 || ex.fd < 0) {
        fprintf(stderr, "v4l2stateless: gpu: no write fence on the bo: %s\n",
                strerror(errno));
        if (ex.fd >= 0)
            close(ex.fd);
        return -1;
    }
    pf.fd = ex.fd;
    pf.events = POLLIN;
    t0 = gpu_now_ms();
    do {
        r = poll(&pf, 1, 1000);
    } while (r < 0 && errno == EINTR);
    *wait_ms = gpu_now_ms() - t0;
    close(ex.fd);
    if (r <= 0) {
        fprintf(stderr, "v4l2stateless: gpu: frame fence %s after %.1f ms\n",
                r == 0 ? "timed out" : "failed", *wait_ms);
        return -1;
    }
    return 0;
}

int v4l2sl_gpu_usable(void)
{
    return !g_gpu_failed;
}

static int gpu_job(struct v4l2sl_surface *s, int y_fd, int uv_fd);

/*
 * Persistent GL thread. The async de-tile worker is created per frame, and
 * binding an EGL context on a fresh thread each time costs ~1 ms of CPU in
 * EGL/gallium TLS setup — as much as the de-tile itself. One long-lived
 * thread costs one handoff per job instead.
 */
static struct {
    pthread_t th;
    pthread_mutex_t lock;
    pthread_cond_t work;
    pthread_cond_t done;
    int started;
    int quit;
    int posted;
    int finished;
    int rc;
    struct v4l2sl_surface *s;
    int y_fd;
    int uv_fd;
} g_req = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .work = PTHREAD_COND_INITIALIZER,
    .done = PTHREAD_COND_INITIALIZER,
    .y_fd = -1,
    .uv_fd = -1,
};

static void *gpu_thread_main(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_req.lock);
    for (;;) {
        while (!g_req.posted && !g_req.quit)
            pthread_cond_wait(&g_req.work, &g_req.lock);
        if (g_req.quit)
            break;
        pthread_mutex_unlock(&g_req.lock);
        g_req.rc = gpu_job(g_req.s, g_req.y_fd, g_req.uv_fd);
        pthread_mutex_lock(&g_req.lock);
        /* Clearing `posted` is the completion ack: the caller may not touch
         * the job fields (or the fds) until it sees it. */
        g_req.posted = 0;
        g_req.finished = 1;
        pthread_cond_signal(&g_req.done);
    }
    pthread_mutex_unlock(&g_req.lock);
    return NULL;
}

int v4l2sl_gpu_detile(struct v4l2sl_surface *s, int y_fd, int uv_fd)
{
    int rc;

    if (!s || !s->gbm_bo || y_fd < 0 || uv_fd < 0)
        goto fail;

    pthread_mutex_lock(&g_req.lock);
    if (!g_req.started) {
        if (pthread_create(&g_req.th, NULL, gpu_thread_main, NULL) != 0) {
            pthread_mutex_unlock(&g_req.lock);
            /* No thread: run it here rather than dropping the frame. */
            return gpu_job(s, y_fd, uv_fd);
        }
        g_req.started = 1;
    }
    /* One job in flight: the caller (async worker or pull) is serialised. */
    while (g_req.posted)
        pthread_cond_wait(&g_req.done, &g_req.lock);
    g_req.s = s;
    g_req.y_fd = y_fd;
    g_req.uv_fd = uv_fd;
    g_req.finished = 0;
    g_req.posted = 1;
    pthread_cond_signal(&g_req.work);
    while (!g_req.finished)
        pthread_cond_wait(&g_req.done, &g_req.lock);
    rc = g_req.rc;
    pthread_mutex_unlock(&g_req.lock);
    close(y_fd);
    close(uv_fd);
    return rc;
fail:
    if (y_fd >= 0)
        close(y_fd);
    if (uv_fd >= 0)
        close(uv_fd);
    return -1;
}

/* One de-tile job. Runs on the persistent GL thread (see gpu_dispatch) so
 * the EGL/gallium per-thread state is paid once per process, not per frame:
 * a fresh thread costs ~1 ms of CPU, which alone would cancel the win over
 * the CPU de-tiler. */
static int gpu_job(struct v4l2sl_surface *s, int y_fd, int uv_fd)
{
    struct timespec ts;
    double t0c;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    t0c = ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;

    uint32_t cap_stride, ah, tiles, aw;
    EGLImageKHR iy = EGL_NO_IMAGE_KHR, iu = EGL_NO_IMAGE_KHR;
    GLuint ty = 0, tu = 0, fbo;
    int bo_fd = -1, rc = -1, attempted = 0;
    double wait_ms = 0.0;
    double t0, t_makecur = 0.0, t_import = 0.0, t_draw = 0.0;

    if (!s || !s->gbm_bo || y_fd < 0 || uv_fd < 0)
        goto out;
    cap_stride = s->stride;
    ah = s->bo_uv_rows ? s->bo_uv_rows : s->aligned_h;
    /* The shader indexes tiles by (stride/16, aligned_h/32); a plane that is
     * not a whole number of tiles cannot be de-tiled this way. */
    if (!cap_stride || !ah || cap_stride % 16 || ah % 32 || s->gbm_stride < 16)
        goto out;
    tiles = (cap_stride / 16) * (ah / 32);
    /* Never write past the bo's row stride, and never read past the source
     * tile columns. */
    aw = s->gbm_stride < cap_stride ? s->gbm_stride : cap_stride;
    if (aw < 16)
        goto out;

    pthread_mutex_lock(&g_gpu_lock);
    attempted = 1;
    if (gpu_init_locked() < 0 || g_gpu_failed)
        goto unlock;
    /* Only this thread ever binds the context, and it stays bound across
     * jobs: tearing the binding down per job costs more than the de-tile. */
    t0 = gpu_now_ms();
    if (eglGetCurrentContext() != g_gpu.ctx &&
        !eglMakeCurrent(g_gpu.dpy, g_gpu.surf, g_gpu.surf, g_gpu.ctx))
        goto unlock;
    t_makecur = gpu_now_ms() - t0;

    /* A stale error from an earlier job must not be read as this draw's. */
    while (glGetError() != GL_NO_ERROR)
        ;
    t0 = gpu_now_ms();
    iy = gpu_image_r8(y_fd, 512, tiles, 512);
    iu = gpu_image_r8(uv_fd, 256, tiles, 256);
    t_import = gpu_now_ms() - t0;
    if (iy == EGL_NO_IMAGE_KHR || iu == EGL_NO_IMAGE_KHR) {
        fprintf(stderr, "v4l2stateless: gpu: plane import failed (y=%p/%d "
                        "u=%p/%d err=0x%x tid=%lu)\n",
                (void *)iy, y_fd, (void *)iu, uv_fd, eglGetError(),
                (unsigned long)pthread_self());
        goto release;
    }
    ty = gpu_texture(iy);
    tu = gpu_texture(iu);
    bo_fd = gbm_bo_get_fd(s->gbm_bo);
    if (!ty || !tu || bo_fd < 0)
        goto release;
    fbo = gpu_dst_fbo(s, bo_fd);
    if (!fbo)
        goto release;

    t0 = gpu_now_ms();
    glUseProgram(g_gpu.prog);
    glBindVertexArray(g_gpu.vao);
    glUniform1i(g_gpu.u_y, 0);
    glUniform1i(g_gpu.u_uv, 1);
    glUniform1i(g_gpu.u_tc, (GLint)(cap_stride / 16));
    glUniform1i(g_gpu.u_ah, (GLint)ah);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ty);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, tu);
    glViewport(0, 0, (GLsizei)aw, (GLsizei)(ah + ah / 2));
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glFlush();
    t_draw = gpu_now_ms() - t0;
    if (glGetError() != GL_NO_ERROR) {
        fprintf(stderr, "v4l2stateless: gpu: GL error during the draw\n");
        goto release;
    }
    if (gpu_wait_write_fence(bo_fd, &wait_ms) < 0)
        goto release;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    rc = 0;

release:
    if (ty)
        glDeleteTextures(1, &ty);
    if (tu)
        glDeleteTextures(1, &tu);
    if (iy != EGL_NO_IMAGE_KHR)
        g_destroy_image(g_gpu.dpy, iy);
    if (iu != EGL_NO_IMAGE_KHR)
        g_destroy_image(g_gpu.dpy, iu);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
unlock:
    pthread_mutex_unlock(&g_gpu_lock);
    if (rc < 0 && attempted && !g_gpu_failed) {
        /* One strike: the session drops to the software de-tile, and the
         * caller rewrites this same bo synchronously. */
        g_gpu_failed = 1;
        if (g_gpu.state > 0)
            fprintf(stderr, "v4l2stateless: de-tile HW->SW (gpu job failed; "
                            "software de-tile from now on)\n");
    }
out:
    /* The fds belong to the caller: it keeps them open until the job has
     * finished and closes them itself (single-threaded ownership). */
    if (bo_fd >= 0)
        close(bo_fd);
    if (getenv("V4L2SL_PROFILE") && rc == 0) {
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
        fprintf(stderr, "v4l2stateless: gpu job: makecur %.2f import %.2f "
                        "draw %.2f fence %.2f wall, cpu %.2f ms\n",
                t_makecur, t_import, t_draw, wait_ms,
                (ts.tv_sec * 1e3 + ts.tv_nsec / 1e6) - t0c);
    }
    return rc;
}

void v4l2sl_gpu_forget_bo(struct gbm_bo *bo)
{
    pthread_mutex_lock(&g_gpu_lock);
    for (int i = 0; i < DST_ENTRIES; i++) {
        struct dst_entry *e = &g_gpu.dst[i];

        if (e->bo != bo)
            continue;
        /* The bo is going away; the image and FBO must not outlive it. */
        if (g_gpu.dpy != EGL_NO_DISPLAY) {
            eglMakeCurrent(g_gpu.dpy, g_gpu.surf, g_gpu.surf, g_gpu.ctx);
            if (e->fbo)
                glDeleteFramebuffers(1, &e->fbo);
            if (e->tex)
                glDeleteTextures(1, &e->tex);
            if (e->img != EGL_NO_IMAGE_KHR)
                g_destroy_image(g_gpu.dpy, e->img);
            eglMakeCurrent(g_gpu.dpy, EGL_NO_SURFACE, EGL_NO_SURFACE,
                           EGL_NO_CONTEXT);
            eglReleaseThread();
        }
        memset(e, 0, sizeof(*e));
    }
    pthread_mutex_unlock(&g_gpu_lock);
}

void v4l2sl_gpu_fini(void)
{
    pthread_mutex_lock(&g_req.lock);
    if (g_req.started) {
        g_req.quit = 1;
        pthread_cond_signal(&g_req.work);
        pthread_mutex_unlock(&g_req.lock);
        pthread_join(g_req.th, NULL);
        g_req.started = 0;
        g_req.quit = 0;
    } else {
        pthread_mutex_unlock(&g_req.lock);
    }
    pthread_mutex_lock(&g_gpu_lock);
    for (int i = 0; i < DST_ENTRIES; i++) {
        struct dst_entry *e = &g_gpu.dst[i];

        if (e->fbo)
            glDeleteFramebuffers(1, &e->fbo);
        if (e->tex)
            glDeleteTextures(1, &e->tex);
        if (e->img != EGL_NO_IMAGE_KHR && g_gpu.dpy != EGL_NO_DISPLAY)
            g_destroy_image(g_gpu.dpy, e->img);
        memset(e, 0, sizeof(*e));
    }
    if (g_gpu.state > 0 && g_gpu.dpy != EGL_NO_DISPLAY) {
        eglMakeCurrent(g_gpu.dpy, EGL_NO_SURFACE, EGL_NO_SURFACE,
                       EGL_NO_CONTEXT);
        if (g_gpu.vao)
            glDeleteVertexArrays(1, &g_gpu.vao);
        if (g_gpu.prog)
            glDeleteProgram(g_gpu.prog);
        if (g_gpu.ctx != EGL_NO_CONTEXT)
            eglDestroyContext(g_gpu.dpy, g_gpu.ctx);
        if (g_gpu.surf != EGL_NO_SURFACE)
            eglDestroySurface(g_gpu.dpy, g_gpu.surf);
        eglTerminate(g_gpu.dpy);
    }
    memset(&g_gpu, 0, sizeof(g_gpu));
    pthread_mutex_unlock(&g_gpu_lock);
}
