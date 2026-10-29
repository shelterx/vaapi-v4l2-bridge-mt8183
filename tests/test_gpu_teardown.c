/*
 * test_gpu_teardown.c — vaTerminate must reap the GPU de-tile GL thread.
 *
 * The de-tile job runs on a persistent GL thread that owns a private
 * EGLDisplay (its own DRM fd, so panfrost never orders a consumer's draws
 * after it). That thread and everything it holds must go away when the client
 * terminates, or a vaTerminate -> vaInitialize cycle inherits a GL context
 * whose objects nobody can delete any more.
 *
 * Everything here goes through the driver shared object, never a static copy:
 * libva dlopens it, so dlopening the same path ourselves gives us the same
 * globals that vaTerminate will touch. Loading a second copy (linking
 * libv4l2sl_core) silently tests the wrong state - vaTerminate then runs a
 * driver whose thread never started.
 *
 * The job itself only needs to *start* the thread, so the source planes are
 * ordinary gbm bos and the shader happily produces garbage; the lifecycle is
 * the real one.
 *
 * Skips (exit 77) when there is no render node or no EGL, so `meson test`
 * stays green on machines without the GPU.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <dlfcn.h>
#include <gbm.h>
#include <va/va.h>
#include <va/va_drm.h>
#include "v4l2stateless.h"

#ifndef TEST_DRIVER_SO
#define TEST_DRIVER_SO "v4l2stateless_drv_video.so"
#endif

#define SKIP 77

static int gpu_threads(void)
{
    DIR *d = opendir("/proc/self/task");
    struct dirent *e;
    int n = 0;

    if (!d)
        return -1;
    while ((e = readdir(d))) {
        char path[64], comm[32] = "";
        FILE *f;

        if (e->d_name[0] == '.')
            continue;
        snprintf(path, sizeof(path), "/proc/self/task/%s/comm", e->d_name);
        if (!(f = fopen(path, "r")))
            continue;
        /* EGL starts Mesa threads of its own; only this one is ours. */
        if (fgets(comm, sizeof(comm), f) && !strncmp(comm, "v4l2sl-gpu", 10))
            n++;
        fclose(f);
    }
    closedir(d);
    return n;
}

/* The task entry can outlive pthread_join() by a scheduling instant, so poll
 * for the expected count instead of sampling once. A thread that never goes
 * away still fails, just 200 ms later. */
static int wait_gpu_threads(int want)
{
    for (int i = 0; i < 200; i++) {
        int n = gpu_threads();

        if (n == want)
            return n;
        usleep(1000);
    }
    return gpu_threads();
}

static int cycle(VADisplay *dpy_out, int drmfd)
{
    VADisplay dpy = vaGetDisplayDRM(drmfd);
    int major = 0, minor = 0;

    if (!dpy || vaInitialize(dpy, &major, &minor) != VA_STATUS_SUCCESS)
        return -1;
    *dpy_out = dpy;
    return 0;
}

int main(void)
{
    struct v4l2sl_context ctx;
    struct v4l2sl_surface surf;
    struct gbm_device *gbm;
    struct gbm_bo *y, *uv, *dst;
    int (*detile)(struct v4l2sl_surface *, int, int);
    VADisplay dpy;
    void *h;
    int drmfd, fails = 0;

    /*
     * Pin libva to the same shared object this test dlopens. Without it libva
     * picks up the installed driver and the two copies have separate globals,
     * so vaTerminate reaps a thread that was never started and the check
     * silently passes/fails for the wrong reason.
     */
    setenv("LIBVA_DRIVER_NAME", "v4l2stateless", 1);
    {
        char so[512];
        char *slash;

        snprintf(so, sizeof(so), "%s", TEST_DRIVER_SO);
        slash = strrchr(so, '/');
        if (slash) {
            *slash = '\0';
            setenv("LIBVA_DRIVERS_PATH", so, 1);
        }
    }

    drmfd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
    if (drmfd < 0) {
        printf("SKIP: no /dev/dri/renderD128\n");
        return SKIP;
    }
    gbm = gbm_create_device(drmfd);
    if (!gbm) {
        printf("SKIP: no gbm device\n");
        return SKIP;
    }
    y = gbm_bo_create(gbm, 1920, 1088, GBM_FORMAT_R8,
                      GBM_BO_USE_LINEAR | GBM_BO_USE_RENDERING);
    uv = gbm_bo_create(gbm, 1920, 1088, GBM_FORMAT_R8,
                       GBM_BO_USE_LINEAR | GBM_BO_USE_RENDERING);
    dst = gbm_bo_create(gbm, 1920, 1632, GBM_FORMAT_R8,
                        GBM_BO_USE_LINEAR | GBM_BO_USE_RENDERING);
    if (!y || !uv || !dst) {
        printf("SKIP: gbm bo allocation failed\n");
        return SKIP;
    }

    h = dlopen(TEST_DRIVER_SO, RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        printf("FAIL: dlopen %s: %s\n", TEST_DRIVER_SO, dlerror());
        return 1;
    }
    *(void **)&detile = dlsym(h, "v4l2sl_gpu_detile");
    if (!detile) {
        printf("FAIL: no v4l2sl_gpu_detile in %s\n", TEST_DRIVER_SO);
        return 1;
    }

    memset(&ctx, 0, sizeof(ctx));
    memset(&surf, 0, sizeof(surf));
    surf.width = 1920;
    surf.height = 1080;
    surf.stride = 1920;
    surf.aligned_h = 1088;
    surf.bo_uv_rows = 1088;
    surf.capture_owner = &ctx;
    surf.buf_index = 3;
    surf.gbm_bo = dst;
    surf.gbm_stride = 1920;

    /* Both cycles: job -> thread up -> vaTerminate -> thread gone. */
    for (int i = 0; i < 2; i++) {
        if (detile(&surf, gbm_bo_get_fd(y), gbm_bo_get_fd(uv)) != 0) {
            printf("SKIP: GPU de-tile unavailable (no EGL/panfrost?)\n");
            return SKIP;
        }
        {
            int n = wait_gpu_threads(1);

            if (n != 1) {
                printf("FAIL[%d]: GL thread did not start (%d)\n", i, n);
                fails++;
            }
        }
        if (cycle(&dpy, drmfd) < 0) {
            printf("FAIL[%d]: vaInitialize failed\n", i);
            return 1;
        }
        if (vaTerminate(dpy) != VA_STATUS_SUCCESS) {
            printf("FAIL[%d]: vaTerminate failed\n", i);
            fails++;
        }
        {
            int n = wait_gpu_threads(0);

            if (n != 0) {
                printf("FAIL[%d]: GL thread survived vaTerminate (%d)\n", i, n);
                fails++;
            }
        }
    }

    if (fails)
        return 1;
    printf("OK gpu teardown: GL thread reaped on vaTerminate, twice\n");
    return 0;
}
