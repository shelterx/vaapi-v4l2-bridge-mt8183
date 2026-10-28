/*
 * v4l2stateless — VA-API to V4L2 Request API bridge
 * Internal data structures and function declarations
 */

#ifndef V4L2STATELESS_H
#define V4L2STATELESS_H

#include <stdint.h>
#include <pthread.h>
#include <poll.h>
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_dec_vp8.h>
#include <va/va_vpp.h>
#include <va/va_drmcommon.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

/* Standard constants missing from the older system headers installed here
 * (libva 2.22.0 / older kernel). Values are the official upstream ones, not
 * vendor hacks: NV15/NV20 are mainline V4L2 fourccs, MM21 is the MT8183 tiled
 * capture, and VAProfileH264High422=40 is libva 2.23.0 (commit 0d1d351). The
 * 10-bit / 4:2:2 paths are never exercised on the 8-bit MT8183 decoder, but
 * the code must still compile. */
#ifndef V4L2_PIX_FMT_NV15
#define V4L2_PIX_FMT_NV15     v4l2_fourcc('N', 'V', '1', '5') /* 10-bit NV12 */
#endif
#ifndef V4L2_PIX_FMT_NV20
#define V4L2_PIX_FMT_NV20     v4l2_fourcc('N', 'V', '2', '0') /* 10-bit NV12 */
#endif
#ifndef V4L2_PIX_FMT_MM21
#define V4L2_PIX_FMT_MM21     v4l2_fourcc('M', 'M', '2', '1') /* MT8183 tiled */
#endif
#ifndef VAProfileH264High422
#define VAProfileH264High422  40
#endif

struct gbm_bo;   /* opaque — only v4l2stateless_gbm.c includes <gbm.h> */

/* Codec identifiers */
enum v4l2sl_codec {
    V4L2SL_CODEC_H264,
    V4L2SL_CODEC_HEVC,
    V4L2SL_CODEC_AV1,
    V4L2SL_CODEC_VP8,
    V4L2SL_CODEC_MPEG2,
    V4L2SL_CODEC_JPEG_ENC,
    V4L2SL_CODEC_VPP,
};

/* Per-config state */
struct v4l2sl_config {
    VAConfigID config_id;
    VAProfile profile;
    VAEntrypoint entrypoint;
    enum v4l2sl_codec codec;
    unsigned int rt_format;
    const char *device_path;
    struct v4l2sl_config *next;
};

/* Single source of truth for which backing holds the freshest pixels:
 * CPU = cpu_ptr (VPP dst, vaPutImage), MEMFD = memfd snapshot,
 * BO = gbm bo (lazy path: memfd is stale until ensure_memfd refills it). */
enum v4l2sl_last_writer {
    V4L2SL_WRITER_NONE = 0,
    V4L2SL_WRITER_CPU = 1,
    V4L2SL_WRITER_MEMFD = 2,
    V4L2SL_WRITER_BO = 3,
};
#define v4l2sl_memfd_stale(s_) ((s_)->last_writer == V4L2SL_WRITER_BO)

extern int v4l2sl_debug;

/* Per-surface state */
struct v4l2sl_surface {
    VASurfaceID surface_id;
    unsigned int width;
    unsigned int height;
    unsigned int format;     /* VA fourcc (NV12/P010/YUY2/ARGB) */
    unsigned int rt_format;
    VASurfaceStatus status;
    int has_pic;             /* holds a decoded picture snapshot (bo/memfd) */
    int buf_index;           /* V4L2 capture buffer index, -1 if not allocated */
    struct v4l2sl_context *capture_owner; /* context that allocated buf_index */
    int memfd_fd;          /* memfd snapshot fd (CPU-readback vehicle) */
    uint32_t memfd_size;     /* current ftruncate size of memfd_fd (grow-only) */
    void *memfd_map;         /* persistent RW mapping of memfd_fd (grow-only) */
    uint32_t memfd_map_size; /* mapped length of memfd_map */
    void *memfd_retired;     /* superseded mapping still held by derived images */
    uint32_t memfd_retired_size;
    uint32_t memfd_borrows;  /* live derived images pointing at memfd_map */
    uint64_t timestamp;      /* V4L2 timestamp for reference tracking */
    uint32_t order_hint;     /* AV1 OrderHint of the frame last decoded here */
    uint8_t av1_level1;      /* AV1 KEY / level-1 ARF (hidden skip=0,0) */
    uint32_t stride;         /* negotiated capture geometry, set at decode */
    uint32_t aligned_h;
    uint32_t cap_fourcc;     /* V4L2 capture fourcc of attached decode buf */
    void *cap_view;          /* mmap of capture_buf_ptr[buf_index]; EXPBUF GetImage */
    uint8_t de_tiled;        /* MM21 capture was de-tiled to linear NV12 into
                              * the memfd. Export via the GBM bo (the raw
                              * tiled plane can't be EXPBUF'd as linear NV12,
                              * and Chrome caches the pool dma-buf, so the bo
                              * is synced from the memfd after the de-tile). */
    void *cpu_ptr;           /* software backing for upload / encode / VPP src */
    uint32_t cpu_size;
    uint32_t cpu_stride;
    struct gbm_bo *gbm_bo;   /* driver-owned display copy (linear, R8) */
    uint32_t gbm_stride;
    uint32_t bo_uv_rows;     /* Y rows before the UV plane inside gbm_bo.
                              * visible height for the software de-tile, the
                              * aligned capture height when MDP3 writes it. */
    int expbuf_fd;           /* VIDIOC_EXPBUF of capture, -1 if none (experiment) */
    int gpu_fence_fd;        /* pending GPU write fence for gbm_bo (GPU de-tile),
                              * -1 when the bo is idle. Only the CPU-readback
                              * paths wait on it: GPU consumers are ordered
                              * after the de-tile by the GPU queue itself. */
    uint8_t last_writer;     /* v4l2sl_last_writer: who wrote pixels last.
                              * memfd staleness is derived: writer == BO. */
};

/* Parameter buffer */
struct v4l2sl_buffer {
    VABufferID buffer_id;
    VABufferType type;
    unsigned int size;
    unsigned int num_elements;
    void *data;
    int mmapped;             /* data from mmap (derive_image) vs malloc */
    struct v4l2sl_surface *borrow_surf; /* mmapped==2: surface whose mapping
                                         * is borrowed (NULL = cpu_ptr borrow) */
    uint32_t fourcc;         /* VAImageBufferType: fourcc of the image */
    uint32_t pitch;          /* VAImageBufferType: allocation stride */
    struct v4l2sl_buffer *next;
};

/* Number of output/capture buffer slots */
#define V4L2SL_NUM_OUTPUT_BUFS  4
/* Hard ceiling on capture buffer indexes — sizes every per-context
 * array. Keep >= the largest requested count below. */
#define V4L2SL_MAX_CAPTURE_BUFS 40
#define V4L2SL_NUM_CAPTURE_BUFS 24
/* AV1 decode through Chrome WebCodecs (bilibili bwp) keeps one live
 * surface per queued VideoFrame; 24 starves after ~5s of playback.
 * hantro AV1 capture buffers do not come from CMA, so the deeper pool
 * is cheap there. */
#define V4L2SL_NUM_CAPTURE_BUFS_AV1 40

/* Slice-data buffers a single picture may carry (MPEG-2 hits one per MB
 * row; sized to match pending_buffers). */
#define V4L2SL_MAX_SLICE_DATAS 256

/* Collected decode-buffer roles from one vaRenderPicture batch
 * (v4l2sl_collect_decode_buffers). */
struct v4l2sl_collected {
    void *pic;              /* VAPictureParameterBufferType (last one wins) */
    void *iq;               /* VAIQMatrixBufferType (last one wins) */
    void *prob;             /* VAProbabilityBufferType (VP8) */
    void *slice_params[32]; /* VASliceParameterBufferType, in order */
    int n_slice_params;
    int n_slice_params_seen; /* unclamped count (overflow vs the 32-slot table) */
    const uint8_t *slice_datas[V4L2SL_MAX_SLICE_DATAS];
    uint32_t slice_sizes[V4L2SL_MAX_SLICE_DATAS];
    int n_slice_datas;
    const uint8_t *largest;     /* biggest SliceData buffer (VP8/AV1 style) */
    uint32_t largest_size;
};


/* Surface table size; IDs are recycled via a free stack so a long-lived
 * process can never index past the table. */
#define V4L2SL_MAX_SURFACES 4096

/* Persistent M2M queue state (VPP / JPEG encode): when the negotiated
 * setup is unchanged across pictures, S_FMT / S_CTRL / REQBUFS / QUERYBUF
 * and the plane mappings are skipped — steady state is STREAMON + QBUF +
 * DQBUF + STREAMOFF only (~18 -> ~7 ioctls, 4 mmap -> 0 per frame). A
 * setup change or any steady-state failure triggers a full teardown and
 * renegotiation on the next picture. */
struct v4l2sl_m2m_state {
    uint8_t  valid;
    uint32_t key[8];         /* codec-specific setup key */
    void    *out_map[3];     /* mapped OUTPUT planes (JPEG: Y, UV[, pad]) */
    size_t   out_len[3];
    int      out_planes;     /* negotiated OUTPUT plane count */
    void    *cap_map;        /* mapped CAPTURE plane */
    size_t   cap_len;
    struct v4l2_format ofmt; /* negotiated OUTPUT format (plane layout) */
};

/* MDP3 m2m HW de-tile state (MM21 -> NV12). Zero-copy: the decoder's MM21
 * capture dma-buf is imported into the OUTPUT queue each frame; the
 * CAPTURE queue's driver-allocated NV12 is mmap'd and read. If init fails
 * (no node / queues / streaming) or after consecutive per-frame failures the
 * HW path is disabled and the software de-tile is used instead. */
struct v4l2sl_mdp3 {
    int fd;             /* /dev/videoN (driver "mdp3"), -1 if absent */
    int out_bufs;       /* OUTPUT queue buffer count */
    int cap_bufs;       /* CAPTURE queue buffer count */
    uint32_t cap_stride; /* NV12 capture bytesperline */
    uint32_t cap_height; /* NV12 capture height (aligned) */
    uint32_t cap_width;  /* NV12 capture width */
    uint32_t cap_size;   /* NV12 capture plane size */
    int fail_count;     /* consecutive HW detile failures */
    int disabled;       /* HW disabled for this session (SW de-tile) */
    int active;         /* init attempted + succeeded for this context */
    int streaming;
};

/* Per-context state (one decode session) */
struct v4l2sl_context {
    VAContextID context_id;
    VAConfigID config_id;
    enum v4l2sl_codec codec;
    VAProfile profile;
    VAEntrypoint entrypoint;
    unsigned int rt_format;
    char *device_path;       /* owned strdup of the config's device path */
    struct v4l2sl_driver_data *driver_data;  /* back-pointer for surface lookup */
    int width;
    int height;

    int v4l2_fd;                 /* V4L2 video device fd */
    int media_fd;                /* V4L2 media device fd (for request alloc) */
    int request_fd;              /* Current V4L2 request */

    VASurfaceID *render_targets;
    int num_render_targets;

    struct v4l2sl_surface *current_surface;
    VASurfaceID current_surface_id;

    /* Collected buffers for current picture. MPEG-2 can be one slice
     * per MB row plus param/IQ buffers, so 32 is not enough. */
    struct v4l2sl_buffer *pending_buffers[256];
    int num_pending_buffers;

    struct v4l2sl_buffer *buffers;  /* Attached parameter buffers */
    struct v4l2sl_context *next;

    /* Output (bitstream) buffer management */
    int output_bufs_allocd;
    uint32_t output_buf_size;
    uint32_t output_plane_len[V4L2SL_NUM_OUTPUT_BUFS]; /* QUERYBUF memo */
    void  *output_buf_ptr[V4L2SL_NUM_OUTPUT_BUFS];
    int free_out_bufs[V4L2SL_NUM_OUTPUT_BUFS];
    int n_free_out;

    /* Capture (decoded frame) buffer management */
    int capture_bufs_allocd;
    int streamed;         /* STREAMON done (HEVC defers it past SPS) */
    void *capture_buf_ptr[V4L2SL_MAX_CAPTURE_BUFS];
    uint32_t capture_buf_size;
    /* MM21 UV plane (plane 1). The driver allocates Y and UV as separate,
     * non-contiguous CMA blocks, so the UV plane is mapped at its own offset
     * and de-tiled from this second pointer. NULL for single-plane formats. */
    void *capture_uv_ptr[V4L2SL_MAX_CAPTURE_BUFS];
    uint32_t capture_uv_size;
    /* DMABUF-allocated capture: per-buffer dma-buf fds (Y / UV), set when the
     * capture queue is allocated with V4L2_MEMORY_DMABUF so a frame can be
     * imported into the MDP3 m2m de-tiler (zero-copy). -1 = not available. */
    int capture_buf_fd[V4L2SL_MAX_CAPTURE_BUFS];
    int capture_uv_fd[V4L2SL_MAX_CAPTURE_BUFS];
    int capture_dmabuf;   /* capture queue allocated as DMABUF */
    int capture_buf_anon; /* calloc stub when ioctl hook is installed */
    int free_cap_bufs[V4L2SL_MAX_CAPTURE_BUFS];
    int n_free_cap;
    uint32_t cap_width;        /* driver-chosen capture geometry */
    uint32_t cap_height;       /* aligned height (e.g. 1088 for 1080p) */
    uint32_t cap_stride;       /* bytes per row, may exceed width */
    uint32_t cap_sizeimage;    /* per-buffer plane size */
    uint32_t cap_pixelformat;  /* V4L2 capture fourcc (NV12/NV15/NV16/NV20) */

    /* Frame counter for V4L2 timestamps (DPB reference matching) */
    uint64_t frame_count;

    /* Hot-path dedup: shadow copy of the last submitted GLOBAL
     * (sequence-level) control payload. Codec translates memcmp against
     * it and skip the global ioctl when unchanged. Contexts are
     * single-codec, so one slot suffices. All translates run under
     * g_v4l2sl_lock. */
    uint8_t g_ctrl_valid;
    /* 1088: this UAPI's padded v4l2_ctrl_h264_sps is 1048 bytes */
    uint8_t g_ctrl_payload[1088];

    /* AV1 refresh_frame_flags inference heuristics: VA does not expose
     * the bitmask. style: 0 = unknown, 1 = libaom-style (free slots from
     * index 1), 2 = SVT-AV1 RA pyramid. */
    struct {
        uint8_t style;
        uint8_t gop;           /* first hidden ARF order_hint (mini-GOP) */
        uint8_t l0_toggle;     /* SVT L0 slots 0-1-2 */
        uint8_t l1_toggle;     /* SVT L1 slots 3-4 */
        uint8_t have_first_arf;
        uint32_t key_oh;       /* order_hint of the current GOP's KEY */
        uint32_t l0_oh;        /* last SVT L0 ARF order_hint */
        uint32_t prev_l0_oh;   /* previous L0 ARF (mini-GOP length) */
        /* Kernel-DPB model for the AV1 copy-out release: which capture
         * buffer each reference slot holds, and which surface owns a
         * given buffer (to clear buf_index when it is released). */
        int model_active;      /* slot model owns releases (parsed truth) */
        int slot_buf[V4L2_AV1_TOTAL_REFS_PER_FRAME];
        VASurfaceID buf_owner[V4L2SL_MAX_CAPTURE_BUFS];
    } av1;

    /* Persistent M2M queues (stateful devices: RGA VPP / VEPU JPEG) */
    struct v4l2sl_m2m_state vpp_q;
    struct v4l2sl_m2m_state jpeg_q;
    /* MDP3 m2m HW de-tile (MM21 -> NV12); see v4l2sl_mdp3. */
    struct v4l2sl_mdp3 mdp3;
};

/* Driver global state */
struct v4l2sl_driver_data {
    struct v4l2sl_config *configs;
    VAConfigID next_config_id;

    /* Simple surface table */
    struct v4l2sl_surface *surfaces[V4L2SL_MAX_SURFACES];
    VASurfaceID next_surface_id;
    VASurfaceID free_surface_ids[V4L2SL_MAX_SURFACES];
    int n_free_surface_ids;

    struct v4l2sl_context *contexts;
    VAContextID next_context_id;

    VABufferID next_buffer_id;
    struct v4l2sl_buffer *orphan_buffers;

    /* Resolved at init by OUTPUT_MPLANE fourcc (empty if missing). */
    char dev_h264[64];
    char dev_hevc[64];
    char dev_av1[64];
    char dev_vp8[64];
    char dev_mpeg2[64];
    char dev_jpeg_enc[64];
    char dev_vpp[64];
};

/* Bounds-checked surface lookup. Every surfaces[id] access must go through
 * this — the table is fixed-size and IDs come from the client. */
static inline struct v4l2sl_surface *
v4l2sl_surface_by_id(struct v4l2sl_driver_data *dd, VASurfaceID id)
{
    if (!dd || id == VA_INVALID_ID || (unsigned)id >= V4L2SL_MAX_SURFACES)
        return NULL;
    return dd->surfaces[id];
}

static inline uint64_t v4l2sl_surface_ts(struct v4l2sl_driver_data *dd, VASurfaceID id)
{
    struct v4l2sl_surface *s = v4l2sl_surface_by_id(dd, id);

    return s ? s->timestamp : 0;
}

/* Bounded, duplicate-free pool pushes. A leaked index is better than a
 * write past the end of free_*_bufs[], which used to corrupt the context. */
static inline void v4l2sl_out_pool_push(struct v4l2sl_context *ctx, int idx)
{
    int i;

    if (!ctx || idx < 0 || idx >= V4L2SL_NUM_OUTPUT_BUFS)
        return;
    if (ctx->n_free_out >= V4L2SL_NUM_OUTPUT_BUFS)
        return;
    for (i = 0; i < ctx->n_free_out; i++)
        if (ctx->free_out_bufs[i] == idx)
            return;
    ctx->free_out_bufs[ctx->n_free_out++] = idx;
}

static inline void v4l2sl_cap_pool_push(struct v4l2sl_context *ctx, int idx)
{
    int i;

    if (!ctx || idx < 0 || idx >= V4L2SL_MAX_CAPTURE_BUFS)
        return;
    if (ctx->n_free_cap >= V4L2SL_MAX_CAPTURE_BUFS)
        return;
    for (i = 0; i < ctx->n_free_cap; i++)
        if (ctx->free_cap_bufs[i] == idx)
            return;
    ctx->free_cap_bufs[ctx->n_free_cap++] = idx;
}

/* Bounded, duplicate-free surface-ID recycle push. Same contract as the
 * pool pushes: a leaked ID beats a write past the table. */
static inline void v4l2sl_surface_id_push(struct v4l2sl_driver_data *dd,
                                          VASurfaceID id)
{
    int i;

    if (!dd || id == VA_INVALID_ID || (unsigned)id >= V4L2SL_MAX_SURFACES)
        return;
    if (dd->n_free_surface_ids >= V4L2SL_MAX_SURFACES)
        return;
    for (i = 0; i < dd->n_free_surface_ids; i++)
        if (dd->free_surface_ids[i] == id)
            return;
    dd->free_surface_ids[dd->n_free_surface_ids++] = id;
}

/* Device helpers (v4l2stateless_device.c) */
int v4l2sl_open_device(const char *path);
int v4l2sl_open_media_for_device(const char *video_path);
int v4l2sl_request_alloc(int media_fd);
int v4l2sl_setup_output_queue(int fd, uint32_t codec_format, int width, int height);
int v4l2sl_setup_capture_queue(int fd, int width, int height, uint32_t pixelformat);
int v4l2sl_setup_capture_queue_count(int fd, int width, int height,
                                     uint32_t pixelformat, int count);
int v4l2sl_streamon(int fd, enum v4l2_buf_type type);
int v4l2sl_streamoff(int fd, enum v4l2_buf_type type);
void v4l2sl_release_context_device(struct v4l2sl_context *ctx);
int v4l2sl_get_capture_geometry(int fd, uint32_t *w, uint32_t *h,
                                uint32_t *stride, uint32_t *sizeimage);
int v4l2sl_ensure_capture(struct v4l2sl_context *ctx, int width, int height,
                          uint32_t pixelformat);
int v4l2sl_queue_output(struct v4l2sl_context *ctx, int buf_index,
                        uint32_t size, uint64_t timestamp);
int v4l2sl_queue_capture(int fd, int buf_index, int request_fd);
/*
 * Submit one synchronous decode: pops a capture buffer, QBUFs the given
 * OUTPUT buffer (caller already memcpy'd the bitstream into it), queues the
 * request, waits for the decoded frame and recycles the bitstream buffer.
 * Returns the decoded capture buffer index, or -1 with both pools fully
 * restored (STREAMOFF reset) — the caller must NOT push back out_buf_idx
 * when this fails.
 */
int v4l2sl_decode_submit(struct v4l2sl_context *ctx, int out_buf_idx,
                         uint32_t bytesused, uint64_t timestamp);
/* STREAMOFF both queues and rebuild the free pools from scratch. Used to
 * recover from decode timeouts so a wedged job is never left in the kernel
 * (rkvdec returns every queued buffer in ERROR state on STREAMOFF). */
void v4l2sl_decode_reset(struct v4l2sl_context *ctx);
int v4l2sl_surface_alloc_export_fd(struct v4l2sl_surface *s);
int v4l2sl_surface_grow_memfd(struct v4l2sl_surface *s, uint32_t size);
/* Persistent per-surface mapping of memfd_fd (RW shared, grow-only; the
 * memfd itself only ever grows). Callers must NOT munmap the result —
 * the surface owns it and releases it at destroy. NULL on failure. */
void *v4l2sl_surface_map_memfd(struct v4l2sl_surface *s, uint32_t need);
/* Lazy CPU backing: decode-only surfaces never pay the multi-MB calloc;
 * the first CPU writer (put_image, VPP dst, upload) allocates it. */
int v4l2sl_surface_ensure_cpu(struct v4l2sl_surface *s);
int v4l2sl_surface_pull_capture(struct v4l2sl_context *ctx,
                                struct v4l2sl_surface *surf, int buf_index);
int v4l2sl_bind_capture_export(struct v4l2sl_context *ctx);
VAStatus v4l2sl_surface_fill_prime(const struct v4l2sl_surface *surf,
                                   const struct v4l2sl_context *c,
                                   uint32_t flags,
                                   void *descriptor);
/* GBM-backed display surfaces (v4l2stateless_gbm.c): the export vehicle for
 * decode surfaces. VPU capture buffers are never exported (chip bug); the
 * bo is panthor system memory kept in sync by pull_capture. */
int v4l2sl_gbm_surface_ensure(struct v4l2sl_surface *s);
/* The process-wide gbm device the surface bos are allocated from. Also what
 * the GPU de-tiler hangs its EGLDisplay on, so the bos are importable. */
struct gbm_device *v4l2sl_gbm_device_ref(void);
int v4l2sl_gbm_surface_sync(struct v4l2sl_surface *s);
int v4l2sl_gbm_surface_detile_mm21(struct v4l2sl_surface *s,
                                   const uint8_t *ysrc, const uint8_t *usrc,
                                   uint32_t aw, uint32_t ah);
/* Async variant (V4L2SL_DETILE_ASYNC=1): starts the de-tile on a worker
 * thread and returns. v4l2sl_gbm_detile_wait() joins the pending job; call it
 * before freeing/reusing the capture slot or the bo. */
int v4l2sl_gbm_detile_async_hw(struct v4l2sl_context *ctx,
                               struct v4l2sl_surface *s, int y_fd, int uv_fd);
int v4l2sl_gbm_surface_detile_async(struct v4l2sl_surface *s,
                                    const uint8_t *ysrc, const uint8_t *usrc,
                                    uint32_t aw, uint32_t ah);
int v4l2sl_gbm_detile_async_wanted(void);
void v4l2sl_gbm_detile_wait(void);
/* Metrics of the most recent async job, for V4L2SL_TRACE. */
double v4l2sl_gbm_detile_last_wait_ms(void);
void v4l2sl_gbm_detile_last_job(double *wall_ms, double *cpu_ms);
/* True when the pending job is reading this surface's capture slot. */
int v4l2sl_gbm_detile_pending_for(const struct v4l2sl_surface *s);
/* Map the bo for writing (HW de-tile writes NV12 into it); unmap when done.
 * Returns the mapped Y pointer (NULL on failure); *out_stride / *out_data
 * receive the bo stride and the unmap handle. */
void *v4l2sl_gbm_surface_map(struct v4l2sl_surface *s, uint32_t w, uint32_t h,
                            uint32_t *out_stride, void **out_data);
void v4l2sl_gbm_surface_unmap(struct v4l2sl_surface *s, void *ptr, void *data);
/* Lazy memfd: refill the memfd snapshot from the bo when a CPU-readback
 * caller (vaGetImage / vaDeriveImage / VPP source) needs it after a
 * bo-only pull_capture. */
int v4l2sl_surface_ensure_memfd(struct v4l2sl_surface *s);
int v4l2sl_gbm_surface_upload(struct v4l2sl_surface *s, const void *src,
                              uint32_t src_stride, uint32_t src_alh);
void v4l2sl_gbm_surface_destroy(struct v4l2sl_surface *s);
VAStatus v4l2sl_surface_fill_prime_gbm(const struct v4l2sl_surface *surf,
                                       uint32_t flags, void *descriptor);
int v4l2sl_capture_expbuf(struct v4l2sl_context *ctx, int buf_index);
/* Export one plane of an MMAP capture buffer as a dma-buf (zero-copy import
 * source for the MDP3 m2m de-tiler). Caller owns the returned fd. */
int v4l2sl_capture_expbuf_plane(struct v4l2sl_context *ctx, int buf_index,
                                int plane);
int v4l2sl_claim_capture_for_export(struct v4l2sl_context *ctx,
                                   struct v4l2sl_surface *surf);
int v4l2sl_expbuf_export_wanted(void);
void v4l2sl_explog(const char *fmt, ...);
typedef int (*v4l2sl_ioctl_fn)(int fd, unsigned long request, void *arg);
void v4l2sl_set_ioctl_hook(v4l2sl_ioctl_fn fn);
int v4l2sl_dequeue_buffer(int fd, enum v4l2_buf_type type, int *flag_error);
int v4l2sl_mmap_output_buffers(int fd, int count, void **ptrs, uint32_t *size_out);
int v4l2sl_set_request_controls(int request_fd, int v4l2_fd, struct v4l2_ext_controls *ctrls);
int v4l2sl_set_global_controls(int v4l2_fd, struct v4l2_ext_controls *ctrls);
int v4l2sl_submit_request(int request_fd);
/* Full teardown of a persistent M2M queue (STREAMOFF + REQBUFS(0) both
 * directions + drop the plane mappings). Safe no-op on an unused queue;
 * call before the context's device fd goes away. */
void v4l2sl_m2m_teardown(struct v4l2sl_context *ctx, struct v4l2sl_m2m_state *q);
/* Which de-tile implementation to use (V4L2SL_DETILE). Parsed in exactly one
 * place so the bo layout, the pull dispatch and the log messages cannot
 * disagree: a mismatch between them writes chroma at the wrong plane offset.
 * sw is the default; unrecognised values fall back to it. */
enum v4l2sl_detile_mode {
    V4L2SL_DETILE_SW = 0,
    V4L2SL_DETILE_MDP3,
    V4L2SL_DETILE_GPU,
};
enum v4l2sl_detile_mode v4l2sl_detile_mode(void);
/* True in MDP3 mode only. */
int v4l2sl_mdp3_requested(void);
/* MDP3 m2m HW de-tile (v4l2stateless_mdp3.c). init returns 0 if the HW path
 * is ready (node found, MM21-out/NV12-cap queues configured + streaming) or
 * -1 (the software de-tile is used instead). detile imports the MM21 capture
 * dma-buf of buf_index, DQBUFs the NV12, and copies it into dst (a linear
 * NV12 buffer at dst_stride). fini tears the queues down. */
int v4l2sl_mdp3_init(struct v4l2sl_context *ctx, int width, int height);
int v4l2sl_mdp3_detile(struct v4l2sl_context *ctx, int buf_index,
                       struct v4l2sl_surface *surf);
/* Same job with the decoder planes already exported (takes ownership). */
int v4l2sl_mdp3_detile_prepared(struct v4l2sl_context *ctx,
                                struct v4l2sl_surface *surf,
                                int y_fd, int uv_fd);
void v4l2sl_mdp3_fini(struct v4l2sl_context *ctx);
/* GPU de-tile (v4l2stateless_gpu.c, V4L2SL_DETILE=gpu): a panfrost GLES3
 * fragment-shader pass that reads the MM21 planes straight from the decoder
 * dma-bufs and writes linear NV12 into the surface bo. Same contract as the
 * MDP3 job: takes ownership of the plane fds, returns 0 when the bo is
 * complete (dma-buf write fence waited for), -1 to let the caller fall back
 * to the software de-tile for this frame. The first failure disables it for
 * the session. ALWAYS takes ownership of both fds, success or failure. */
int v4l2sl_gpu_detile(struct v4l2sl_surface *s, int y_fd, int uv_fd);
int v4l2sl_gpu_usable(void);
/* Wait for an outstanding GPU write to this surface's bo. Needed only before
 * the CPU touches the pixels (memfd refill, destroy); the display handoff is
 * ordered by the GPU queue and must not block. */
void v4l2sl_gpu_wait_surface(struct v4l2sl_surface *s);
/* Same, for a decoder capture slot about to be re-queued: the GPU job keeps
 * reading the plane dma-bufs after the worker returns. `fd` is the decoder's
 * fd, because slot numbers are only unique within one context. */
void v4l2sl_gpu_wait_capture(int fd, int buf_index);
/* Reap every outstanding GPU write fence; call before the decoder's capture
 * buffers are freed or renegotiated (they may still be read by the GPU). */
void v4l2sl_gpu_wait_pending(void);
/* Drop all GPU state pointing at a surface that is being destroyed. */
void v4l2sl_gpu_forget_surface(struct v4l2sl_surface *s);
/* Drop the cached destination image for a bo about to be destroyed; the
 * cache is keyed by pointer, so a recycled allocation must not inherit it. */
void v4l2sl_gpu_forget_bo(struct gbm_bo *bo);
/* Drop the cached source plane images of a context whose capture pool is
 * being freed (context release, capture renegotiation). A decode reset keeps
 * the pool and must not call this. */
void v4l2sl_gpu_forget_capture(struct v4l2sl_context *ctx);
void v4l2sl_gpu_fini(void);
/* One-pass decode-buffer collection shared by every codec translate. */
void v4l2sl_collect_decode_buffers(struct v4l2sl_buffer **buffers, int n,
                                   struct v4l2sl_collected *cb);
/* Give a capture slot back from a surface that holds no picture, for when a
 * context's free pool is exhausted by Chrome's up-front surface export.
 * Caller must hold the driver lock (g_v4l2sl_lock). */
int v4l2sl_reclaim_capture_slot(struct v4l2sl_context *ctx,
                                struct v4l2sl_surface *keep);
/* Single-object VADRMPRIMESurfaceDescriptor layout shared by the memfd
 * and GBM-bo export paths: 2 planes at `pitch`, chroma starting at row
 * `chroma_row`; SEPARATE_LAYERS splits them into two single-plane
 * layers (luma_fmt/chroma_fmt) instead of one `combined_fmt` layer. */
void v4l2sl_fill_prime_layers(VADRMPRIMESurfaceDescriptor *desc,
                              int fd, uint32_t object_size,
                              uint64_t modifier, uint32_t va_fourcc,
                              uint32_t width, uint32_t height,
                              uint32_t pitch, uint32_t chroma_row,
                              uint32_t combined_fmt, uint32_t luma_fmt,
                              uint32_t chroma_fmt, uint32_t flags);
/* poll() wrapper that retries EINTR (device.c) */
int v4l2sl_poll_intr(struct pollfd *fds, nfds_t n, int timeout);
/* EINTR-retrying ioctl (device.c); VPP/JPEG route through it so the test
 * ioctl hook covers them too. */
int v4l2sl_xioctl(int fd, unsigned long request, void *arg);

/* H.264 translation (v4l2stateless_h264.c) */
void h264_fill_sps(struct v4l2_ctrl_h264_sps *sps, const VAPictureParameterBufferH264 *pic,
                    VAProfile profile);
void h264_fill_pps(struct v4l2_ctrl_h264_pps *pps, const VAPictureParameterBufferH264 *pic,
                    const VASliceParameterBufferH264 *slice);
void h264_fill_decode_params(struct v4l2_ctrl_h264_decode_params *dec,
                             const VAPictureParameterBufferH264 *pic,
                             struct v4l2sl_driver_data *dd,
                             const struct v4l2_ctrl_h264_sps *sps,
                             const VASliceParameterBufferH264 *slice,
                             int nal_type, int nal_ref_idc);
/* First VCL NAL unit of a picture: type (5 = IDR) and nal_ref_idc, taken from
 * the bitstream because VA-API exposes neither. Returns 0 when the data holds
 * no readable VCL NAL header. */
int h264_first_vcl_nal(const uint8_t *data, uint32_t size,
                       int *nal_type, int *nal_ref_idc);
void h264_fill_scaling_matrix(struct v4l2_ctrl_h264_scaling_matrix *sm,
                              const VAIQMatrixBufferH264 *iq);
VAStatus v4l2sl_h264_translate(struct v4l2sl_context *ctx,
                               struct v4l2sl_buffer **buffers,
                               int num_buffers);

/* HEVC translation (v4l2stateless_hevc.c) */
VAStatus v4l2sl_hevc_translate(struct v4l2sl_context *ctx,
                               struct v4l2sl_buffer **buffers,
                               int num_buffers);

/* AV1 translation (v4l2stateless_av1.c) */
void v4l2sl_av1_dpb_model_reset(struct v4l2sl_context *ctx);
VAStatus v4l2sl_av1_translate(struct v4l2sl_context *ctx,
                              struct v4l2sl_buffer **buffers,
                              int num_buffers);

/* VP8 translation (v4l2stateless_vp8.c) */
void v4l2sl_vp8_fill_frame(struct v4l2_ctrl_vp8_frame *frame,
                           const VAPictureParameterBufferVP8 *pic,
                           const VASliceParameterBufferVP8 *slice,
                           const VAProbabilityDataBufferVP8 *prob,
                           const VAIQMatrixBufferVP8 *iq,
                           struct v4l2sl_driver_data *dd);
VAStatus v4l2sl_vp8_translate(struct v4l2sl_context *ctx,
                              struct v4l2sl_buffer **buffers,
                              int num_buffers);

/* MPEG-2 translation (v4l2stateless_mpeg2.c) */
void v4l2sl_mpeg2_fill_sequence(struct v4l2_ctrl_mpeg2_sequence *seq,
                                const VAPictureParameterBufferMPEG2 *pic,
                                VAProfile profile);
void v4l2sl_mpeg2_fill_picture(struct v4l2_ctrl_mpeg2_picture *vpic,
                               const VAPictureParameterBufferMPEG2 *pic,
                               struct v4l2sl_driver_data *dd);
void v4l2sl_mpeg2_fill_quant(struct v4l2_ctrl_mpeg2_quantisation *q,
                             const VAIQMatrixBufferMPEG2 *iq);
VAStatus v4l2sl_mpeg2_translate(struct v4l2sl_context *ctx,
                                struct v4l2sl_buffer **buffers,
                                int num_buffers);

/* JPEG encode / RGA VPP */
VAStatus v4l2sl_jpeg_encode(struct v4l2sl_context *ctx,
                            struct v4l2sl_buffer **buffers, int num_buffers);
VAStatus v4l2sl_vpp_run(struct v4l2sl_context *ctx,
                        struct v4l2sl_buffer **buffers, int num_buffers);
VAStatus v4l2sl_vpp_query_filters(VAProcFilterType *filters, unsigned int *n);
VAStatus v4l2sl_vpp_query_filter_caps(VAProcFilterType type, void *caps,
                                      unsigned int *n);
VAStatus v4l2sl_vpp_query_pipeline_caps(VAProcPipelineCaps *caps);

/* Pixel format helpers (v4l2stateless_format.c) */
uint32_t v4l2sl_capture_fourcc_from_rt(unsigned int rt_format);
uint32_t v4l2sl_capture_fourcc_from_sps(int bit_depth_minus8, int chroma_format_idc);
uint32_t v4l2sl_va_fourcc_for_capture(uint32_t v4l2_fourcc);
uint32_t v4l2sl_drm_fourcc_for_capture(uint32_t v4l2_fourcc);
uint32_t v4l2sl_capture_plane_size(uint32_t fourcc, uint32_t stride, uint32_t aligned_h);
uint32_t v4l2sl_va_image_size(uint32_t va_fourcc, uint32_t stride, uint32_t height);
uint32_t v4l2sl_default_image_stride(uint32_t va_fourcc, int width);
size_t v4l2sl_annexb_concat(const uint8_t * const *datas, const uint32_t *sizes,
                            int n, int prefix_len, uint8_t *dst, size_t dst_cap);
void v4l2sl_nv15_to_p010(uint8_t *dst, uint32_t dst_stride,
                         const uint8_t *src, uint32_t src_stride,
                         uint32_t src_aligned_h, int width, int height);
void v4l2sl_copy_nv12(uint8_t *dst, uint32_t dst_stride,
                      const uint8_t *src, uint32_t src_stride,
                      uint32_t src_aligned_h, int width, int height);
void v4l2sl_nv16_to_yuy2(uint8_t *dst, uint32_t dst_stride,
                         const uint8_t *src, uint32_t src_stride,
                         uint32_t src_aligned_h, int width, int height);
void v4l2sl_nv20_to_yuy2(uint8_t *dst, uint32_t dst_stride,
                         const uint8_t *src, uint32_t src_stride,
                         uint32_t src_aligned_h, int width, int height);
void v4l2sl_nv20_to_y210(uint8_t *dst, uint32_t dst_stride,
                         const uint8_t *src, uint32_t src_stride,
                         uint32_t src_aligned_h, int width, int height);

#endif /* V4L2STATELESS_H */
