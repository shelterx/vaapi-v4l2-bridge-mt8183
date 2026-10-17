# MT8183 notes — what in this tree is (isn't) relevant

This tree is a **VA-API → V4L2 bridge for Rockchip RK3588** (Orange Pi 5,
mainline kernel). It is *not* an MT8183 project — but several of its pieces
are directly reusable for the MT8183 `vaapi-v4l2-bridge` goal (HW H.264 →
NV12 for Chromium). This note records what transfers and what does not, so
we don't re-derive it.

Target end-state on MT8183: `mtk-vcodec-dec (MM21) → [de-tile] → NV12 →
Chromium`. The decoder (`/dev/video5`) emits **MM21 only**; MDP3
(`/dev/video0`) is the only MM21→NV12 path and its M2M job-completion (GCE
EOF) never fires on this kernel (see AGENTS.md).

---

## 1. The two device classes (ARCHITECTURE §6)

The bridge splits V4L2 devices into two models:

| Class | Devices (RK3588) | UAPI |
|-------|------------------|------|
| **Stateless Request API** | rkvdec H.264/HEVC, hantro AV1/VP8/MPEG2 | per-picture `MEDIA_REQUEST_IOC_QUEUE`, request-scoped `S_EXT_CTRLS` |
| **Stateful M2M** | RGA VPP (`/dev/video0`), VEPU JPEG (`/dev/video3`) | classic `S_FMT`/`REQBUFS`/mmap once, then `QBUF`+`DQBUF` per frame |

**MT8183 mapping (corrected 2026-10-15):**
- The MT8183 **decoder IS stateless** (`/dev/video5`: `S264`
  (`V4L2_PIX_FMT_H264_SLICE`) in → `MM21` out; per-picture
  `MEDIA_REQUEST_IOC_QUEUE`, request-scoped `S_EXT_CTRLS` via `/dev/media1`).
  So the tree's **stateless** translators (`v4l2stateless_h264.c`, etc.)
  **DO apply** and are what the bridge uses. (An earlier note wrongly said
  "stateful"; that was corrected.)
- MDP3 is a **stateful M2M** device — the **RGA VPP** (`v4l2stateless_vpp.c`)
  is the integration model for it, but it is **not needed**: the bridge
  de-tiles MM21→NV12 in software.

## 2. `v4l2stateless_vpp.c` — the M2M conversion pattern (RELEVANT)

This is the template for driving a stateful conversion device. Per picture:

```
copy source pixels INTO the mapped OUTPUT plane   (copy_rows_to)
STREAMON(out) + STREAMON(cap)
QBUF(cap) + QBUF(out)
poll(POLLIN|POLLOUT, 2000 ms)
DQBUF(cap)
copy result OUT of the mapped CAPTURE plane       (copy_rows_from)
STREAMOFF both
```

Setup is cached in `struct v4l2sl_m2m_state` (keyed by geometry+fourcc+
transform): unchanged setup skips `S_FMT`/`REQBUFS`/mmap — steady state is
`STREAMON`+`QBUF`+`DQBUF`+`STREAMOFF` only. `v4l2sl_m2m_teardown()` is the
full tear-down (STREAMOFF + `REQBUFS(0)` both ways + munmap).

**Our `tools/v4l2_mdp3_stream.c` already implements exactly this M2M
plumbing for MDP3** (and it works: S_FMT/REQBUFS/mmap/QBUF/STREAMON all
succeed). The only missing piece is the **HW job completing** — which is the
GCE EOF bug, not a userspace pattern problem.

**Gap:** RGA VPP is **all linear formats** (NV12/I420/YUY2/ARGB). There is
**no MM21/tiled handling** anywhere in the tree. `copy_rows_to/from` are
plain row memcpy's. So even a working MDP3 VPP would not de-tile by itself —
MDP3's HW does the de-tile, which is what's broken.

## 3. `v4l2stateless_format.c` — pixel converters (RELEVANT as structure)

All converters are **linear → linear** (NV15→P010, NV12 copy, NV16→YUY2,
NV20→YUY2/Y210). None handle tiling. Reusable primitives for a **SW MM21
de-tile**:

- `v4l2sl_copy_nv12(dst, dst_stride, src, src_stride, src_alh, w, h)` — the
  canonical "copy Y rows then UV rows" pattern (a de-tile writes the same
  NV12 layout, just from a tiled source).
- `v4l2sl_va_image_size` / `v4l2sl_default_image_stride` — output sizing
  (64-byte-aligned strides for NEON-friendly rows).
- The row-scratch pattern (`row_scratch_get`) for wide rows.

A de-tiler would be a new function here: read MM21 tiles (tile size/order —
see §5), scatter into a linear NV12 buffer. **Implemented** as
`v4l2sl_mm21_detile()` in `v4l2stateless_device.c` (not in this file) —
commits `08e2fd5` (initial) and `0a9b7d1` (chroma layout fix).

## 4. `v4l2stateless_probe.c` — device discovery (PARTIALLY RELEVANT)

- Decoders are picked by **stateless OUTPUT fourcc** (`H264_SLICE` etc.)
  *and* a working `MEDIA_IOC_REQUEST_ALLOC`. The MT8183 decoder is stateless,
  so `v4l2sl_pick_device_for_codec` **does** select `/dev/video5` (this is
  how Chromium binds it). Note the request-API check opens the media device —
  which a browser sandbox can block (that's the Firefox issue, §12).
- The **VPP node is picked by `has_rotate && !has_jpeg_cap &&
  !node_has_coded_output`**. MDP3 has `V4L2_CID_ROTATE` and its output
  (MM21) is *not* a "coded" fourcc — so this probe logic would **already
  select `/dev/video0` (MDP3) as the VPP device** on MT8183. That's a
  useful coincidence: the probe side is ~ready for MDP3-as-VPP.
- Results are cached per boot (`$XDG_RUNTIME_DIR/v4l2stateless-probe.cache`,
  keyed by boot_id) to avoid the node-reopen storm that hung the RK3588.

## 5. The MM21 tile layout (CORRECTED 2026-10-16)

Authoritative (Linux kernel `pixfmt-yuv-planar` + empirical match against a
software decode):
  - **Y plane: 16x32 tiles** (16 wide x 32 tall), 512 B/tile, row-major tile
    order (left-to-right, top-to-bottom). NOT 32x32 (that was a wrong guess).
  - **UV plane: the SAME tile grid as luma**, (aw/16) x (ah/32). Each tile is
    **8x16 CHROMA samples** (U,V interleaved = NV12), 256 B/tile — one tile per
    16x32 luma tile. The kernel doc says "the number of luma and chroma tiles
    are identical". The chroma plane is half_w x half_h.
  - width aligned to 16, height aligned to 32. Two non-contiguous planes.
  - Y and UV tile counts are both (aw/16) x (ah/32).

De-tile: for Y pixel (x,y): `tc=x/16, tr=y/32`,
`off=(tr*(aw/16)+tc)*512 + (y%32)*16 + (x%16)`. For UV chroma (cx,cy):
`tc=cx/8, tr=cy/16`, `off=(tr*(aw/16)+tc)*256 + (cy%16)*16 + (cx%8)*2`.
See `tools/mm21_detile.c` for a reference implementation.

> **Confirmed against libyuv `DetilePlane`** (`source/planar_functions.cc`):
> "tile width is 16 and assumed", `src_tile_stride = 16 * tile_height`, and
> `MM21ToNV12` calls it as `DetilePlane(y, ..., width, height, 32)` and
> `DetilePlane(uv, ..., (width+1)&~1, height/2, 16)`. So the chroma tile is
> **16 bytes x 16 rows = 256 B** (8 interleaved U/V pairs per row) — the kernel
> doc's "chroma pixels in 16x16 tiles" means 16 *bytes*, not 16 chroma samples.
> This matches the corrected formula above exactly.

> **Earlier layouts were WRONG.** The 2026-10-15 version used a
> (aw/32) x (ah/32) grid of 16x16 chroma tiles (512 B/tile). That matches the
> *luma* output fine but scrambles non-neutral chroma — it was only ever
> validated on luma (`YAVG`). Symptom: coloured text decoded as grey with cyan
> fringes; software decode showed the real colour. With the corrected layout the
> bridge's HW output is **pixel-exact (Y and UV) vs a software decode** of Big
> Buck Bunny. Reference: kernel "number of luma and chroma tiles identical".

## 6. What does NOT transfer

- ~~**`v4l2stateless_h264.c` / hevc / av1 / vp8 / mpeg2** — stateless Request
  API translators. MT8183 vcodec is stateful; these don't apply.~~ **WRONG —
  corrected 2026-10-15:** the MT8183 vcodec **is stateless**, so these
  translators DO apply and are exactly what the bridge uses.
- **EXPBUF zero-copy of the capture buffer** (`pull_capture` EXPBUF path) —
  the RK3588 VPU capture is NV12 and can be EXPBUF'd to the GPU. MT8183's
  capture is **MM21 (tiled)** and can't be EXPBUF'd as linear NV12. Instead
  the bridge de-tiles and exports a **GBM bo** (a real dma-buf Chromium
  imports) — see §9.
- **RGA-specific controls** (rotate/flip) — MDP3 has its own control set.

## 7. Reusable infrastructure (as-is)

- `v4l2stateless.c` — the whole VA-API front (vtable, surfaces, contexts,
  `vaBegin/Render/EndPicture`, `vaGetImage`, `vaExportSurfaceHandle`,
  pool accounting, locking). Arch-neutral; would host an MT8183 decoder +
  de-tile translator.
- `v4l2stateless_device.c` — M2M helpers (`m2m_teardown`, `streamon/off`,
  `pull_capture`, `xioctl`/`poll_intr`), buffer pools.
- `v4l2sl_xioctl` / `v4l2sl_poll_intr` — EINTR-safe ioctl/poll wrappers.

## 8. Invariants worth keeping (ARCHITECTURE §13)

- Chrome **caches the first VA failure for the session** → a bad first
  decode needs a relaunch. Keep decode failures as `VASurfaceSkipped` +
  success, never a hard error, or the session wedges.
- One request in flight per context; decode is synchronous.
- Do not grow `v4l2stateless.c` with bitstream parsing — one translator file
  per codec.

## 9. MM21 capture integration status (2026-10-16)

**Scope note:** this refactor is **MT8183-only** (RK3588 no longer relevant).

The decoder `/dev/video5` (stateless S264 in → MM21 out) is wired into the
bridge. Multiplane V4L2 gotchas fixed in `v4l2stateless_device.c` (all
verified against `tools/v4l2_mdp3_stream.c` conventions + live probes):

1. **QBUF** (`v4l2sl_queue_capture`): `buf.length` must be the real plane
   count (MM21 = 2); was hardcoded 1 → `QBUF … EINVAL`. Now queried via
   QUERYBUF.
2. **mmap** (`v4l2sl_mmap_one_capture`): MM21's Y and UV are **separate,
   non-contiguous CMA blocks** (probe: Y off=0x40000000 len=2088960,
   UV off=0x40001000 len=1044480 — NOT Y_off+Y_len). Must mmap each plane at
   its own `mem_offset`. Added `ctx->capture_uv_ptr[]`/`capture_uv_size`.
   Verified distinct (write-to-Y does not leak into UV, and vice versa).
3. **DQBUF** (`v4l2sl_dequeue_buffer`): `buf.length` must be >= plane count;
   set to `VIDEO_MAX_PLANES` (kernel fills the real count). Was 1 →
   `DQBUF … EINVAL`.
4. **De-tile** (`v4l2sl_mm21_detile`): takes separate `ysrc`/`usrc` (not a
   contiguous src). Math is byte-identical to the verified `tools/mm21_detile.c`.
5. **memfd sizing** (`v4l2sl_surface_pull_capture`): de-tiled NV12 is
   `stride*alh*3/2` (was sized to the Y-plane only → overflow).

Installed `.so` must be re-copied to `/usr/lib/aarch64-linux-gnu/dri/`
after each build (Chromium loads the installed copy, not `builddir/`).

### ⚠️ OPEN BLOCKER (later DISPROVEN — see the "RESOLVED" section below): capture buffer has NO decoded data (2026-10-16)

Video renders as **green/violet flicker** (opposite chroma values =
per-frame garbage). One-shot raw dump (`V4L2SL_DUMP_MM21=1`, added in
`v4l2sl_surface_pull_capture` → `/tmp/mm21_y.bin` + `/tmp/mm21_uv.bin`) of a
decompleted frame (idx 22) shows:

- **Y plane: min=0 max=18 avg=15.9, only 3 unique vals (16,0,18)** → near-black,
  NOT real luma (real luma spans 0–255).
- **UV plane: all 128 (1 unique)** → neutral, NOT real chroma.

So the buffer we de-tile is the decoder's **cleared/initial state** (Y=16
limited-range black, UV=128 neutral), not the decoded picture.

### Instrumentation results (2026-10-16, continued)

Added correlation logs in `v4l2sl_decode_submit` (queued vs dequeued cap idx
+ per-plane bytesused) and `v4l2sl_surface_pull_capture` (read idx + y/uv
pointers). Findings:

- **Association is CORRECT:** `cap_q == done_cap == read idx` (e.g. all 22).
- **bytesused is FULL on both planes:** `p0=2088960/2088960 p1=1044480/
  1044480`. But the driver's `vb2_set_plane_payload` sets this REGARDLESS of
  decode success — so full bytesused does NOT mean the data is present.
- **The mmaps are correct:** Y and UV map to distinct, non-overlapping CMA
  blocks; the read pointers are stable per index.
- **Decode mode is set:** `V4L2_CID_STATELESS_H264_DECODE_MODE =
  FRAME_BASED` (the bridge is frame-based, matching the MT8183 driver which
  parses slice headers in HW; SPS/PPS/decode_params are all set on the
  request). So the decode-mode is NOT the bug.
- **Driver source (mtk_vcodec_dec_stateless.c) confirms** the firmware
  writes directly to the queued CAPTURE buffer's CMA
  (`vdec_get_cap_buffer` → `vb2_plane_vaddr(dst_buf,0/1)` → firmware
  `y_fb_dma`/`c_fb_dma`); `cap_to_disp()` marks it done *during*
  `vdec_if_decode` (after the firmware finishes), so the data should be
  present at DQBUF time. No hidden DMA-BUF.

**Remaining suspect (later DISPROVEN — see the "RESOLVED" section below):**
the bridge allocates capture buffers with
`V4L2_MEMORY_MMAP`; GStreamer's `v4l2slh264dec` uses `V4L2_MEMORY_DMABUF`.
A one-shot `VIDIOC_EXPBUF` of the DQBUF'd capture buffer **fails with
EINVAL** — the MT8183 CMA allocator does not export this buffer as a
DMA-BUF, so the mmap path and the GStreamer DMABuf path may map the CMA
differently. Next:
1. Read the GStreamer `v4l2slh264dec` source — exactly how it allocates
   (DMABuf) and reads the capture buffer, and compare to the bridge's mmap.
2. Test switching the bridge capture queue to `V4L2_MEMORY_DMABUF` (like
   GStreamer) and see if the frame data appears.
3. If mmap is the culprit, the fix is to read via DMA-BUF (EXPBUF) — but that
   path is currently EINVAL, so first confirm the driver's EXPBUF support.

### Root-cause narrowing (2026-10-16, final)

- The bridge allocates **all** buffers (output + capture) with
  `V4L2_MEMORY_MMAP` and mmap's the capture planes via
  `planes[i].m.mem_offset`. GStreamer's `v4l2slh264dec` uses
  `V4L2_MEMORY_DMABUF` (dma-heap-backed), which is a DIFFERENT allocation.
- `VIDIOC_EXPBUF` on the bridge's MMAP capture buffer **fails with EINVAL**
  — the MT8183 CMA allocator does not implement `get_dma_buf` for the MMAP
  buffer, so it cannot be exported as a DMA-BUF. (GStreamer's DMABUF path
  works because it allocates from a dma-heap that DOES export DMA-BUFs.)
- `V4L2SL_EXPBUF_EXPORT=0` (forces the GBM-copy fallback, which reads the
  mmap) yields a CONSTANT solid-green image = the cleared mmap, confirming the
  mmap view is not where the firmware's frame lands.
- **Most likely root cause:** for a `V4L2_MEMORY_MMAP` capture buffer the
  MT8183 firmware writes the decoded frame to a CMA region that the userspace
  mmap does not map (mmap/dma-buf divergence in the driver's CMA handling),
  whereas the GStreamer `V4L2_MEMORY_DMABUF` allocation maps it correctly.
- **Fix direction (LATER DISPROVEN):** allocate the capture queue with
  `V4L2_MEMORY_DMABUF` and read via the exported DMA-BUF fd. This was a red
  herring: the `EXPBUF failed EINVAL` observation came from a **buggy debug
  call that never set `exp.type`** (the real `v4l2sl_capture_expbuf` sets it),
  so EXPBUF actually works and the MMAP view is fine.

### ✅ RESOLVED (2026-10-16): decode was fine; the EXPORT was wrong

Ground-truth comparison (bridge dump vs. GStreamer's known-good frame):
- The bridge's decoded capture buffer and GStreamer's frame contained the
  **same MM21 tiles** — repacking GStreamer's frame from its 54-wide tile grid
  (coded 864/16) to the driver's 56-wide grid (stride 896/16) matched the
  bridge's raw Y plane **100%**. The visible Y and UV regions matched
  **byte-for-byte**. So decode + de-tile + mmap were all correct all along.
- The frame-0 dump looked "cleared" only because frame 0 is a flat
  limited-range-gray frame (Y=16, UV=128); it WAS the frame.
- `ffmpeg ... -vf hwdownload` (which calls `vaExportSurfaceHandle`, the same
  entry point Chromium uses) produced correct NV12 (Y exactly equal to a
  software decode; UV differing only by HW-vs-SW rounding at text edges).

**Real root cause:** Chromium exports the entire VA surface pool **before the
first picture** and caches the imported dma-buf (`export surf ... idx=-1
de_tiled=0`). For a tiled capture, `v4l2sl_export_surface_handle` took the
EXPBUF branch and exported the **raw tiled MM21 plane labelled as linear
NV12**, then never re-exported — so Chromium sampled the raw tiled buffer
forever (green/violet, with horizontal streaks). `V4L2SL_EXPBUF_EXPORT=0`
took the GBM branch, which created the bo at export time and never re-synced
it after the de-tile → solid green.

**Fix (commit `0c527d5`):**
- At export time, claim a capture slot (so the negotiated fourcc is known),
  and if the capture is tiled (`cap_fourcc`/`cap_pixelformat == MM21` or the
  surface was already de-tiled), export a **GBM bo** instead of the raw plane.
  A GBM bo is a real dma-buf Chromium can import and sample.
- `v4l2sl_surface_pull_capture` de-tiles MM21 → NV12 into the memfd, then
  `v4l2sl_gbm_surface_sync()` pushes that frame into the bo (its layout puts
  chroma at `gbm_stride*height`, which is what the compositor expects).
- `v4l2sl_gbm_surface_ensure()` now accepts `cap_fourcc == MM21`.

Verified: Chromium plays the Norway 4K YouTube stream with the correct image
(593 decoded frames, clean screenshot). Remember to install the built `.so`
to `/usr/lib/aarch64-linux-gnu/dri/` before testing.

**Follow-up fix — chroma de-tile layout (`0a9b7d1`):** the MM21 **UV tile
geometry** was also wrong (16×16 chroma tiles on an `aw/32` grid). The correct
layout is the **same tile grid as luma**, each tile **8×16 chroma samples
(16 bytes × 16 rows = 256 B)** — see §5. It only showed up on coloured content
as grey text + cyan fringes; the bridge's HW output is now pixel-exact vs a
software decode.

---

## Bottom line for MT8183

- **MDP3-as-VPP** (option A): the integration pattern exists
  (`v4l2stateless_vpp.c` + `tools/v4l2_mdp3_stream.c`), the probe would pick
  it, but it's **blocked on the GCE EOF HW bug** — no userspace change fixes
  it.
- **SW de-tile** (option B): **DONE and working end-to-end in Chromium.**
  The decoder is stateless (S264→MM21); the bridge de-tiles MM21→NV12 in
  `v4l2sl_surface_pull_capture` and exports it to Chromium as a GBM bo. See the
  resolution above and `AGENTS.md`.

---

## 10. `/dev/video5` lockup (deadlock) — RESOLVED (2026-10-16)

**Status: real, reproduced several times; root cause found and fixed — see the
"RESOLVED" subsection at the end of this section.** It was a bridge race
(`MEDIA_REQUEST_IOC_REINIT` vs the driver completing the request); fix
`c01d651`. The historical evidence below is kept for reference.

### Symptom
`/dev/video5` becomes permanently unusable. Every new `open("/dev/video5")`
blocks (uninterruptible, `D` state) forever. Chromium's
`VideoCaptureService` (it probes `video5` as a camera) and the decoder both
hang; only a reboot clears it.

### Confirmed kernel evidence (captured while wedged)
Blocked tasks (`wchan`):

```
<tid>  chromium          D  fops_vcodec_open      <- mutex_lock(&dev->dev_mutex)
<tid>  chromium          D  fops_vcodec_open
<tid>  ThreadPoolSingl   D  v4l2_m2m_cancel_job   <- the holder
```

Stacks:
```
fops_vcodec_open+0x54/0x314 [mtk_vcodec_dec]   == return addr right after
                                                the `bl mutex_lock` at
                                                fops_vcodec_open+0x50
v4l2_m2m_cancel_job
v4l2_m2m_streamoff
v4l2_m2m_ioctl_streamoff
v4l_streamoff            <- VIDIOC_STREAMOFF
video_ioctl2 (holds dev->dev_mutex for the whole ioctl)
```

`fops_vcodec_open` was confirmed by disassembly: module text base
`0xffffbbf3dffc0680`, `fops_vcodec_open` at `...b4ec` (module offset `0xae6c`);
`+0x54 = 0xaec0` is the instruction after `bl mutex_lock` at `0xaebc`.

### Mechanism (driver design flaw)
`drivers/media/platform/mediatek/vcodec/decoder/mtk_vcodec_dec_drv.c` uses
`dev->dev_mutex` as **all three** of:
- the V4L2 file lock (`vfd_dec->lock = &dev->dev_mutex`, drv.c:433),
- the OUTPUT queue lock (`src_vq->lock`, dec.c:1016), and
- the CAPTURE queue lock (`dst_vq->lock`, dec.c:1032).

So **any** operation on `/dev/video5` serializes on one mutex. `VIDIOC_STREAMOFF`
-> `v4l2_m2m_streamoff` -> `v4l2_m2m_cancel_job` waits (under that mutex) for the
currently-running m2m decode job to finish. If that job never finishes, the wait
never returns, `dev_mutex` is held forever, and every later `open()` blocks.

`v4l2_m2m_cancel_job` waits for `job_flags & TRANS_RUNNING` to clear, which only
happens when `mtk_vdec_worker` calls `v4l2_m2m_job_finish`. So the wedge
requires a decode job that is stuck (or otherwise never reaches job_finish).

### Bridge-side observations
- The bridge serializes `vaEndPicture` (the blocking decode) and
  `vaDestroyContext` under the global `g_v4l2sl_lock`, so a decode cannot be
  in flight *during* teardown from a bridge-side race. Clean closes are safe.
- The bridge issues `VIDIOC_STREAMOFF` in exactly two places:
  - `v4l2sl_release_context_device()` — teardown (after the synchronous
    `v4l2sl_decode_submit` returned, so normally no job outstanding), and
  - `v4l2sl_decode_reset()` — the **decode-timeout** path (calls STREAMOFF
    while a job may be outstanding). `"decode timeout, resetting queues"` has
    been seen with the local `BigBuckBunny.mp4`.

### What did NOT reproduce it (so far)
- Clean `play -> SIGTERM` close: device stayed healthy, `open()` fine.
- Navigating away from a playing YouTube page: device stayed healthy.
- Two decode contexts alive at once (one destroyed, one still decoding):
  device stayed healthy.
So "normal close" alone is NOT the trigger. The remaining suspect is a decode
job that is stuck/queued at the moment of `VIDIOC_STREAMOFF` — most likely via
the **timeout -> `v4l2sl_decode_reset()`** path, or a genuinely wedged decoder
job.

### Recovery / prevention / candidate fixes
- **Recovery:** reboot only (the holder is a `D`-state kernel task; not
  killable).
- **Avoid:** one decoder instance at a time; don't SIGKILL mid-decode.
- **Candidate bridge fixes (untested):**
  1. Drain before STREAMOFF (queue EOS / `V4L2_DEC_CMD_STOP`, reap all
     output+capture buffers) so no job is outstanding.
  2. Do not STREAMOFF from `v4l2sl_decode_reset()`; mark the context dead and
     let close/teardown deal with it.
- **Real fix is in the driver** (`v4l2_m2m_cancel_job` under the shared
  `dev_mutex`; a robust cancel/timeout). Out of the bridge's reach.

### Reproduce attempt to try next
Play the local `BigBuckBunny.mp4` (which reliably hits the decode timeout),
then watch `/dev/video5` — if the timeout->reset path is the trigger, the wedge
should appear there deterministically.

### ✅ RESOLVED (2026-10-16): `MEDIA_REQUEST_IOC_REINIT` race

Root cause found by reading the real driver source (pulled `v6.18.34`):

`mtk_vdec_worker()` in `mtk_vcodec_dec_stateless.c`, for the MT8183 arch
(`!IS_VDEC_LAT_ARCH`), completes the request **after** it has already finished
the m2m job and returned the capture buffer:

```c
v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev_dec, ctx->m2m_ctx, state);
if (src_buf_req)
        v4l2_ctrl_request_complete(src_buf_req, &ctx->ctrl_hdl);   /* AFTER */
```

The bridge used to `MEDIA_REQUEST_IOC_REINIT` the request immediately after
DQBUF (in `v4l2sl_decode_submit`). REINIT **clears the request's bound objects
and lists**, so it raced the worker's `v4l2_ctrl_request_complete()` — which
then finds no controls object (`media_request_object_find` == NULL), hits the
fallback `media_request_object_bind()` (`WARN_ON(req->state != UPDATING &&
!= QUEUED)`, `drivers/media/mc/mc-request.c:416`) and corrupts the request's
internal lists/refcounts. That corruption is what eventually wedged
`/dev/video5`.

**Fix (`c01d651`):** stop REINIT-ing. `close()` the request fd after each frame
instead — the kernel keeps the request alive via the buffer's own reference
until the worker is done, and `v4l2sl_begin_picture()` allocates a fresh one
next frame. No shared-request mutation ⇒ no race.

**Verification:** 3 × (launch Chromium → play a YouTube H.264 stream → `pkill`
mid-playback). Each cycle decoded ~500 frames; after all three, `/dev/video5`
opened fine and there were no stuck `vcodec`/`m2m` tasks. Before the fix the
same repeated-launch pattern wedged the device. (Note the `pull_capture MM21`
frame print is behind `V4L2SL_DEBUG` now — count frames with it set.)

**Also fixed the same session: the request-control WARNs (`7ea290a`).** ~42
`media_request_object_bind` WARNs (mc-request.c:416) appeared during playback.
Root cause: `v4l2sl_set_request_controls()` wrote the request fd into
`ctrls->which` (`V4L2_CTRL_WHICH_REQUEST_VAL | fd`) — the pre-UAPI scheme. The
kernel routes on `cs->which == V4L2_CTRL_WHICH_REQUEST_VAL` **exactly** and
reads the fd from the separate `struct v4l2_ext_controls.request_fd` field, so
the ioctl silently fell through to the current/global path: the request got
**no controls object**, and the worker's `v4l2_ctrl_request_complete()` hit the
fallback `v4l2_ctrl_request_bind()` → WARN. Fix: set
`which = V4L2_CTRL_WHICH_REQUEST_VAL` and `request_fd = request_fd`. Now the
stateless SPS/PPS/decode_params are genuinely bound to each request. Verified
**42 → 0** WARNs (only the benign `__flush_work` one remains, which GStreamer's
`v4l2slh264dec` also produces). Does **not** change the BBB frame-90 stop (that
is Chromium-side, §11).

The underlying `v4l2_m2m_cancel_job()`-under-`dev->dev_mutex` fragility in the
driver remains (a truly stuck job could still deadlock), but the bridge no
longer triggers it.

---

## 11. Local `BigBuckBunny.mp4` stops at frame ~89 (2026-10-16)

**Symptom:** playing `file:///BigBuckBunny.mp4` in
Chromium via the bridge decodes exactly **89 frames** then stops with:
```
WARNING media/gpu/vaapi/vaapi_video_decoder.cc:437  DecoderStatus::1: error decoding stream
WARNING media/filters/decoder_stream.cc:210        DecoderStatus::108
WARNING media/renderers/video_renderer_impl.cc:613 PipelineStatus::PIPELINE_ERROR_DECODE
```
YouTube H.264 streams decode fine for hundreds/thousands of frames. Software
decode of the same file is also fine.

**Findings (what it is NOT):**
- **Not the bridge.** Counters added to `v4l2sl_begin_picture` /
  `v4l2sl_end_picture` give `begin == end == pull_capture == 89` — the 89th frame
  fully completes; **frame 90 never reaches the bridge** (no `vaBeginPicture`,
  no VA buffer op). Instrumented `v4l2sl_create_buffer` /
  `v4l2sl_render_picture` / `v4l2sl_h264_translate` / `decode_submit` produced
  **no error** at any point. The chroma de-tile is pixel-exact vs software for
  all frames (see §5).
- **Not the MM21 de-tile / capture path** (already pixel-exact).
- **Not `V4L2_BUF_FLAG_ERROR`** (the `-2` skip path never fires).
- **Not a decode timeout** (`resetting decode queues` count is 0 in this run).

**Where it actually fails:** inside Chromium's own `H264Decoder`
(`media/gpu/h264_decoder.cc`). `VaapiVideoDecoder::HandleDecodeTask` gets
`AcceleratedVideoDecoder::kDecodeError` from `decoder_->Decode()`. In that file
`kDecodeError` is only produced by `SET_ERROR_AND_RETURN()`, reached when an
accelerator call wrapped in `CHECK_ACCELERATOR_RESULT(...)` returns
`kFail`/`kNotSupported`. Candidate calls (non-encrypted path):
`PreprocessCurrentSlice()` (=> `FinishPrevFrameIfPresent` /
`ModifyReferencePicLists` / `first_mb_in_slice != 0`), `ParseSliceHeader`,
`StartNewFrame`, `ProcessCurrentSlice` (=> `SubmitSlice`). Since no VA call is
made for frame 90, the failure is in the **software parser / DPB / reference
list** logic, not the driver. The informative `DVLOG(1) << "Error during
 decode"` and the failing call name are **compiled out of the Debian release
build** (`DVLOG` is a no-op without `DCHECK_IS_ON()`), so the exact call can't
be read from the log.

**Stream shape (BBB):** Main profile, level 3.0, 1 B-frame, multi-slice
(≈5 slices + a 21-byte SEI per frame). SPS/PPS repeat every ~152 NALs
(~25 frames), i.e. not at frame 90.

**Isolation results (2026-10-16):** a `-c copy` remux of the original bitstream
still stops at 89; x264 re-encodes of the same video play through (`-bf 0` →
399 frames, single-slice → 405). Stripping the repeated in-band SPS/PPS doesn't
help. So the trigger is inherent to the original encode, not the container or
the parameter-set repetition.

**Stream shape (BBB) — the likely trigger:** Main profile, level 3.0,
`log2_max_frame_num_minus4=1` (→ frame_num wraps every 32),
`max_num_ref_frames=2`, B-frames used as references, and a P picture sharing a
`frame_num` with the preceding B (pic 88 B fn=8, pic 89 P fn=8). libavcodec and
GStreamer decode the identical bitstream fine; only Chromium's `H264Decoder`
rejects it, at access unit 90.

**Status:** not fixable in the bridge — a Chromium `H264Decoder` bug. The real
fix is upstream (a bug report with this file as reproducer). Chromium
`155.0.8059.79` (current security update) still fails. No debug symbols are
published for 155 (archive or `debuginfod.debian.net`), and Chromium 150 (which
does have a dbgsym) doesn't engage VA-API on this box, so the exact failing
line couldn't be captured at runtime.

---

## 12. Firefox 158: works, but needs the sandbox disabled (2026-10-16)

Tested Mozilla's `firefox` (non-ESR, **158.0**, from the Mozilla apt repo)
playing `BigBuckBunny.mp4` through the bridge (`LIBVA_DRIVER_NAME=v4l2stateless`,
`MOZ_ENABLE_WAYLAND=1`):

| run (clean probe cache) | probe result | decode |
|---|---|---|
| default (sandbox on) | `probe H.264 -> (none)` | software fallback |
| `MOZ_DISABLE_RDD_SANDBOX=1 MOZ_DISABLE_CONTENT_SANDBOX=1` | `probe H.264 -> /dev/video5` | **464 frames HW-decoded** |

Firefox loads the bridge (`va_openDriver() returns 0`) either way; the
difference is the **probe**. `v4l2sl_video_has_request_api()` opens the media
device (`/dev/media1`) and issues `MEDIA_IOC_REQUEST_ALLOC` to confirm the
stateless request API, and `scan_all_uncached` **drops any decoder node that
fails** (`v4l2stateless_probe.c` line 280). Firefox's RDD/content sandbox
blocks `/dev/media*`, so `/dev/video5` is dropped and Firefox falls back to
software. Chromium's GPU process can open `/dev/media1`, so it works there.
JPEG/VPP are still found because they don't need the request API.

**Conclusion:** Firefox works with the bridge (full HW H.264 decode), but only
with a relaxed sandbox. Options: run with `MOZ_DISABLE_RDD_SANDBOX=1` (a
security trade-off), allow `/dev/media*` through Firefox's sandbox policy, or
make the bridge probe not hard-require `/dev/media1` for H.264 detection.
