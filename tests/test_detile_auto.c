/*
 * V4L2SL_DETILE tier selection: forced modes are returned unchanged, auto
 * gives MDP3 to one context at a time and the GPU to the rest. No hardware:
 * only the claim bookkeeping runs.
 */

#include <stdio.h>
#include <stdlib.h>

#include "v4l2stateless.h"

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

static enum v4l2sl_detile_mode resolve(const char *mode,
                                       struct v4l2sl_context *ctx)
{
    if (mode)
        setenv("V4L2SL_DETILE", mode, 1);
    else
        unsetenv("V4L2SL_DETILE");
    return v4l2sl_detile_resolve(ctx);
}

int main(void)
{
    struct v4l2sl_context *a = calloc(1, sizeof(*a));
    struct v4l2sl_context *b = calloc(1, sizeof(*b));
    struct v4l2sl_context *c = calloc(1, sizeof(*c));
    struct v4l2sl_context *d = calloc(1, sizeof(*d));

    if (!a || !b || !c || !d)
        return 1;

    /* Unset is auto: the ladder picks whatever this box can actually run. */
    /* Unset is auto: the ladder decides per context, so nothing is claimed
     * here - the auto cases below pin the choice it actually makes. */
    unsetenv("V4L2SL_DETILE");
    expect(v4l2sl_detile_mode() == V4L2SL_DETILE_AUTO, "unset-is-auto");

    /* Forced modes: same tier for every context, no claim taken. */
    expect(resolve("sw", a) == V4L2SL_DETILE_SW, "forced-sw");
    expect(resolve("gpu", a) == V4L2SL_DETILE_GPU, "forced-gpu");
    expect(resolve("hw", a) == V4L2SL_DETILE_MDP3, "forced-hw-a");
    expect(resolve("hw", b) == V4L2SL_DETILE_MDP3, "forced-hw-b-unchanged");

    /* Auto: first context gets MDP3, the others the best remaining tier. Which
     * one that is depends on this machine (the GPU path is probed, not
     * assumed), so the assertions follow the probe. */
    int gpu_ok = v4l2sl_gpu_probe();
    enum v4l2sl_detile_mode rest = gpu_ok ? V4L2SL_DETILE_GPU : V4L2SL_DETILE_SW;

    expect(resolve("auto", a) == V4L2SL_DETILE_MDP3, "auto-first-hw");
    expect(resolve("auto", b) == rest, gpu_ok ? "auto-second-gpu"
                                             : "auto-second-sw");
    expect(resolve("auto", c) == rest, "auto-third-same-tier");
    expect(resolve("auto", a) == V4L2SL_DETILE_MDP3, "auto-owner-stays-hw");

    /* A context that does not hold the claim releasing changes nothing. */
    v4l2sl_mdp3_fini(b);
    expect(resolve("auto", c) == rest, "auto-non-owner-fini");

    /* Releasing the claimant frees MDP3 for the next context. */
    v4l2sl_mdp3_fini(a);
    expect(resolve("auto", d) == V4L2SL_DETILE_MDP3, "auto-reclaim-after-fini");
    expect(resolve("auto", c) == rest, "auto-still-one-owner");

    v4l2sl_mdp3_fini(d);
    free(a);
    free(b);
    free(c);
    free(d);
    return g_fail ? 1 : 0;
}
