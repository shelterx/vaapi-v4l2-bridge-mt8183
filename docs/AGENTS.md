
# AGENTS.md — MT8183 work

Style: read HANDOFF.md, update this file with progress as we go.
**Focused commits only (one logical change each), and do NOT commit until the
USER says so** (no auto-commit, no big chunks). Commit subject: single line
prefixed `mediatek: <text>`.

## Notes

- Run Chromium with `--password-store=basic` (no keyring daemon on
  this box; avoids libsecret hang/errors at startup).

## Progress

- 2026-10-15: fixed `tools/v4l2_fmt_test.c` — dropped bogus ABI-era
  ioctl probing; parse `v4l2_pix_format_mplane` for both MPLANE queues
  (old code parsed the single-plane layout on MPLANE data → 0 planes).
  Verified on MDP3 (/dev/video0): MM21 out 1920x1088 2 planes
  (2088960+1044480), NV12 cap 1920x1080 1 plane (3110400); G_FMT on the
  same fd matches S_FMT. Note: MDP3 aligns MM21 height 1080→1088,
  capture stays 1080 (scale needed in conversion).

## Progress (cont.)

- 2026-10-15: `tools/v4l2_mdp3_stream.c` — standalone MDP3 streaming
  test: S_FMT MM21 out + NV12 cap (matched size), REQBUFS+mmap both
  queues, fill MM21 with constants (Y=0x11, UV=0x22/0x33 — constants
  survive de-tiling), STREAMON, DQBUF, verify NV12 output. Fixed two
  multiplanar ioctl gotchas: QUERYBUF requires b.memory=MMAP,
  QBUF requires b.length=plane count.

- 2026-10-15: MDP3 streaming test — full V4L2 plumbing now works
  (S_FMT/REQBUFS/QUERYBUF/mmap/QBUF/STREAMON all succeed). Key userspace
  fix: mmap offset must be `planes[i].m.mem_offset`, NOT `data_offset`
  (vb2 leaves data_offset=0 for mmap buffers → all planes collide at 0
  → capture-queue mmap EINVAL). **HW BLOCKER:** STREAMON succeeds but
  DQBUF times out — 0 frames for BOTH MM21→NV12 and NV12→NV12 identity.
  Job goes CMDQ→GCE→SCP (SCP remoteproc="running", mailbox="active")
  but the GCE completion event never fires → `v4l2_m2m_buf_done` never
  called. Kernel has no CONFIG_DYNAMIC_DEBUG/tracefs, so limited
  visibility. Needs a debug kernel or GCE/CMDQ/SCP bring-up.

- 2026-10-16: **MM21→NV12 SW de-tile integrated into the bridge**
  (option B; MDP3 HW path still blocked on GCE EOF). Changes in
  `src/v4l2stateless_device.c`: (1) `v4l2sl_mm21_detile()` de-tiles
  16x32 Y / 16x16 UV row-major tiles -> linear NV12 (same size); (2)
  `v4l2sl_capture_pick_fourcc()` selects MM21 when the capture queue
  only offers a tiled format; (3) `v4l2sl_surface_pull_capture()`
  de-tiles MM21 capture into the surface memfd and reports the surface
  as NV12 (EXPBUF/GBM zero-copy paths skipped for tiled). Build fixed
  in `src/v4l2stateless.h`: the installed libva 2.22.0 / kernel headers
  lack `V4L2_PIX_FMT_NV15/NV20/MM21` and `VAProfileH264High422` —
  defined with official values (High422=40 per libva 2.23.0). Also
  removed the stray `src/v4l2stateless.h.orig`. `meson compile` clean;
  3/4 unit tests pass (`export_recapture` needs a real DRM device,
  times out headless). Next: Chromium end-to-end H.264 playback.

- 2026-10-16: **MM21 multiplane capture fixed** (Chromium end-to-end was
  failing on the first decode). Three bugs in the de-tile path added above,
  all in `src/v4l2stateless_device.c`:
  1. `v4l2sl_queue_capture()` hardcoded `buf.length = 1`; multiplanar QBUF
     requires the real plane count (MM21 = 2) -> now queried via QUERYBUF.
  2. `v4l2sl_mmap_one_capture()` mapped only plane 0; MM21's UV plane was
     never mapped. The MT8183 driver keeps Y and UV in SEPARATE, non-
     contiguous CMA blocks (verified: UV offset != Y offset + Y len), so it
     now maps both planes into `capture_buf_ptr` (Y) and new
     `capture_uv_ptr` (UV).
  3. `v4l2sl_mm21_detile()` assumed a contiguous Y+UV `src`; it now takes
     separate `ysrc`/`usrc`. `v4l2sl_surface_pull_capture()` also sized the
     memfd to the Y-plane size (aw*ah) instead of the de-tiled NV12 size
     (aw*ah*3/2) -> overflow; fixed. `v4l2sl_unmap_capture_buffers()`
     unmaps both planes. Build clean, 3/4 unit tests pass. Reinstalled the
     .so (Chromium loads the stale one from /usr/lib/.../dri/ otherwise).

- 2026-10-16: **OPEN BLOCKER found via raw dump.** Chromium renders green/
  violet flicker. Added a one-shot raw dump (`V4L2SL_DUMP_MM21=1` in
  `v4l2sl_surface_pull_capture` -> `/tmp/mm21_{y,uv}.bin`) of a DQBUF'd
  capture buffer. Result: **Y plane min=0 max=18 avg=15.9 (3 unique: 16,0,18)**
  and **UV plane all 128 (constant)** = the decoder's CLEARED/initial state,
  NOT the decoded frame. The de-tile math and both mmaps are verified
  correct (distinct Y/UV blocks, math matches `tools/mm21_detile.c`); the
  bug is that we read the capture buffer **without/after the decoder writing
  the frame** — a timing or request->capture-buffer association issue in the
  stateless submit path. Next: instrument submit/DQBUF (bytesused,
  V4L2_BUF_FLAG_ERROR, output-side request done) and compare against how
  GStreamer's `v4l2slh264dec` orders DQBUF vs. read. Full detail in
  `docs/mt8183-notes.md` section 9.

- 2026-10-16: **Instrumented submit/DQBUF; ruled out association + decode-mode.**
  Added correlation logs (queued vs dequeued cap idx + per-plane bytesused in
  `v4l2sl_decode_submit`; read idx + y/uv pointers in
  `v4l2sl_surface_pull_capture`). Findings: (1) association is CORRECT
  (`cap_q == done_cap == read idx`); (2) bytesused is FULL on both planes but
  the driver sets it regardless of decode success; (3) mmaps are correct
  (distinct Y/UV); (4) decode mode IS set (`FRAME_BASED`, matching the MT8183
  driver) and SPS/PPS/decode_params are all set — so decode-mode is NOT the
  bug. Driver source confirms the firmware writes the queued CAPTURE buffer's
  CMA directly (no hidden DMA-BUF). **Remaining suspect:** the bridge uses
  `V4L2_MEMORY_MMAP` for capture buffers; GStreamer's `v4l2slh264dec` uses
  `V4L2_MEMORY_DMABUF`. A one-shot `VIDIOC_EXPBUF` of the DQBUF'd capture
  buffer **fails with EINVAL** — the MT8183 CMA allocator won't export it as a
  DMA-BUF, so the mmap path and the GStreamer DMABuf path may map the CMA
  differently. Next: (a) read GStreamer `v4l2slh264dec` source for its exact
  capture-buffer allocation/read; (b) test switching the bridge capture queue
  to `V4L2_MEMORY_DMABUF`. Full detail in `docs/mt8183-notes.md` section 9.

- 2026-10-16: **Root cause narrowed to MMAP vs DMABUF allocation.** The bridge
  allocates ALL buffers (output + capture) with `V4L2_MEMORY_MMAP`; GStreamer's
  `v4l2slh264dec` uses `V4L2_MEMORY_DMABUF` (dma-heap-backed). `VIDIOC_EXPBUF`
  on the bridge's MMAP capture buffer **fails EINVAL** (the MT8183 CMA allocator
  won't export it as a DMA-BUF), and `V4L2SL_EXPBUF_EXPORT=0` (GBM-copy, reads
  the mmap) gives a CONSTANT solid-green = the cleared mmap. Most likely: for a
  MMAP capture buffer the firmware writes the frame to a CMA region the
  userspace mmap doesn't map, while GStreamer's DMABUF allocation maps it
  correctly. **Fix direction:** allocate the capture queue with
  `V4L2_MEMORY_DMABUF` and read the frame via the exported DMA-BUF fd (not the
  V4L2-fd mmap). Larger change to `v4l2sl_setup_capture_queue` /
  `v4l2sl_mmap_one_capture` / the de-tile read path. Full detail in
  `docs/mt8183-notes.md` section 9.

- 2026-10-16: **RESOLVED: Chromium H.264 HW playback works end-to-end.** The
  MMAP-vs-DMABUF theory was a red herring. The real bug: the decode + de-tile
  were already correct (verified byte-for-byte against GStreamer's known-good
  frame), but `v4l2sl_export_surface_handle` was exporting the **raw tiled MM21
  capture plane as linear NV12** (EXPBUF branch) — and because Chromium exports
  the whole surface pool **before the first picture** and caches the imported
  dma-buf, it kept sampling that raw buffer forever (green/violet flicker).
  Fix: at export time, if the capture is tiled (`cap_fourcc == MM21`, known
  after claiming a capture slot), hand Chromium a **GBM bo** (real dma-buf)
  instead; `v4l2sl_surface_pull_capture` de-tiles into the memfd then
  `v4l2sl_gbm_surface_sync()` pushes the frame into that bo. Also allow MM21 in
  `v4l2sl_gbm_surface_ensure()`. Verified: Chromium plays the Norway 4K stream
  with the correct image (593 decoded frames, clean screenshot). Commits
  `0c527d5` (+ docs). Note: the bridge must be installed to
  `/usr/lib/aarch64-linux-gnu/dri/` for Chromium to pick it up.

## Open (priority order)

- **CHROMIUM BUG (not ours): `BigBuckBunny.mp4` stops at frame ~89.** Proven
  Chromium-side — frame 90 makes zero bridge calls (`begin==end==pull==89`), no
  bridge error, and libavcodec/GStreamer decode the same bitstream fine. The
  original encode is unusual (`log2_max_frame_num=5` → wrap 32,
  `max_num_ref_frames=2`, reference B-frames, P sharing a `frame_num` with the
  preceding B). Chromium 155.0.8059.79 (updated) still fails; x264 re-encodes
  of the same video play. Fix = upstream bug report. `docs/mt8183-notes.md` §11.
- **Firefox needs a relaxed sandbox.** Firefox 158 HW-decodes via the bridge
  only with `MOZ_DISABLE_RDD_SANDBOX=1 MOZ_DISABLE_CONTENT_SANDBOX=1` (464
  frames on BBB); otherwise its probe returns `H.264 -> (none)` because the
  bridge's decoder probe hard-requires opening `/dev/media1` +
  `MEDIA_IOC_REQUEST_ALLOC`, which Firefox's sandbox blocks. Chromium is fine
  (its GPU process can open `/dev/media1`). `docs/mt8183-notes.md` §12.
- **GATE: HW H.264 decode on /dev/video5 (stateless) — CONFIRMED WORKING**
  (see progress: ~372 MM21 frames from BBB via `v4l2slh264dec`).
- **DONE: MM21 -> NV12 SW de-tile in the bridge** (option B). MDP3 (HW,
  option A) is blocked on the GCE EOF bug, so the bridge de-tiles in
  userspace. `v4l2sl_surface_pull_capture` de-tiles MM21 capture -> NV12
  memfd; `v4l2sl_capture_pick_fourcc` selects MM21 when the decoder only
  offers a tiled capture. MM21 layout (CONFIRMED, see docs/mt8183-notes.md
  §5): 16x32 Y tiles / 16x16 UV tiles, row-major — NOT 32x32/32x16.
  Reference impl: `tools/mm21_detile.c` (verified on real BBB decode).
- **DONE: MM21 chroma de-tile fix (`0a9b7d1`)** — the chroma plane uses the
  **same tile grid as luma** (8×16 chroma samples / 256 B per tile), not a
  `aw/32` grid of 16×16 tiles. Bridge HW output is now pixel-exact vs software.
- **DONE: Chromium H.264 HW playback works** (Norway 4K, correct image). The
  tiled capture is exported as a GBM bo and filled by the de-tile; see the
  2026-10-16 progress entry. Remaining polish: the de-tile still writes the
  memfd and then copies memfd->bo (could de-tile straight into the bo map);
  smoothness/perf not yet measured.
- **FIXED: `/dev/video5` lockup.** Cause was a bridge race: the bridge called
  `MEDIA_REQUEST_IOC_REINIT` right after DQBUF, but the MTK worker completes
  the request (`v4l2_ctrl_request_complete`) **after**
  `v4l2_m2m_buf_done_and_job_finish` — so REINIT cleared the request's objects
  mid-completion, corrupting its lists/refcounts and wedging the device. Fix
  (`c01d651`): `close()` the request fd each frame instead of REINIT-ing (the
  kernel keeps the request alive via the buffer ref). Verified: 3 ×
  (YouTube play + kill) → ~500 frames each, device healthy, no stuck tasks;
  the old repeated-launch pattern no longer wedges it. Full writeup:
  `docs/mt8183-notes.md` §10.
- **FIXED: `media_request_object_bind` WARNs** (`7ea290a`). The bridge put the
  request fd in `ctrls->which` (`V4L2_CTRL_WHICH_REQUEST_VAL | fd`) — the old
  scheme. The kernel routes on `cs->which == V4L2_CTRL_WHICH_REQUEST_VAL`
  exactly and takes the fd from the separate `request_fd` field, so the
  controls silently fell through to the current/global path: the request got
  **no controls object**, and the worker's `v4l2_ctrl_request_complete()` tripped
  the fallback WARN. Fix: set `which = V4L2_CTRL_WHICH_REQUEST_VAL` and
  `request_fd = fd`. Verified 42 → 0 WARNs (only the benign `__flush_work` one
  remains, which GStreamer also produces).
- Deferred: debug the GCE/CMDQ/SCP job-completion path (MDP3) — needs a
  dynamic-debug kernel build; see hexdump0815/mt8183-chromebook-linux-kernel.
  Full MDP3 notes + leading hypothesis (`cmdq_thread_irq_handler` PC-based
  completion vs the EOC+jump packet) + next steps: `docs/mt8183-mdp3.md`.

## Firmware swap experiment (2026-10-15)

ChromeOS (on eMMC /dev/mmcblk0p3, current 2026 build) ships a DIFFERENT,
newer SCP firmware than our mainline linux-firmware one:
  - ours (linux-firmware 20250410): 401e9c02…  kukui_scp_v2.0.13324 (2022)
  - cros:                           537748da…  kukui_scp_v2.0.2798 (2026)
Both contain the MDP tile engine (DpEngine_IMG2O etc.); it's a build/version
difference. ChromeOS MDP3 uses the same CMDQ->GCE->SCP path as ours.

Test: swapped /lib/firmware/mediatek/mt8183/scp.img to the ChromeOS build
(original backed up to scp.img.mainline-bak), then reboot. After boot:
  - check `dmesg | grep 'FW version'` for the new build
  - re-run `sudo ./v4l2_mdp3_stream 1280 736 mm21`
  - if frames appear -> firmware was the fix; else revert (copy
    scp.img.mainline-bak back) and suspect the DTB/driver (kernel 5.10 vs 6.18).
  Revert: `sudo cp /lib/firmware/mtreek/mt8183/scp.img.mainline-bak \
            /lib/firmware/mediatek/mt8183/scp.img && sudo reboot`

### Firmware swap RESULT (2026-10-15): NO FIX

Swapped to cros scp.img (537748da, v2.0.2798), rebooted, confirmed
`SCP is ready. FW version kukui_scp_v2.0.2798-80cc0fafc4`. Re-ran
`v4l2_mdp3_stream 1280 736 mm21` -> STILL 0/4 frames. Firmware is NOT the
(sole) cause. (Left cros firmware in place; mainline backed up as
scp.img.mainline-bak. Revert if desired.)

### Ruled out / confirmed (2026-10-15)

- `mtk-mdp3 ... can't get SCP node` at probe is BENIGN/known: the
  `mediatek,scp` phandle is optional; driver falls back to finding SCP by
  compatible. Upstream even patches the message out. /dev/video0 registers
  fine, mdp->scp is set.
- DTB: MDP3 node (soc/dma-controller0@14001000, mt8183-mdp3-rdma) HAS
  gce-events (phandles 2,28 -> gce1 syscon@15020000) + gce-client-reg.
  Standard mainline layout; not a missing-property issue.
- Driver flow (mainline v6.18 mtk-mdp3): mdp_m2m_device_run does
  (1) mdp_vpu_process (IPI to SCP) then (2) mdp_cmdq_send (CMDQ job).
  In our run BOTH succeed (no "VPU MDP process failed" / "CMDQ sendtask
  failed" in dmesg). The job then waits for the EOF GCE event, which NEVER
  fires -> DQBUF timeout. So HW never reports frame-done.
- hexdump0815/linux-mainline-mediatek-mt81xx-kernel (our stb-cbm+ source)
  uses the SAME mainline v6.18 mdp3 driver; its mt8183 patches are
  DTB/display-related, NOT mdp3. kernel-extra-patches repo has no mdp3/gce
  patches. So MDP3 M2M is very likely non-functional in that kernel too.

### Pipeline reality (2026-10-15)

Decoder /dev/video5 = mtk-vcodec-dec supports ONLY MM21 (tiled), no NV12.
So: H.264 dec -> MM21 -> [MDP3] -> NV12. MDP3 is the only MM21->NV12 path
and it's broken. Options: (A) fix MDP3 (kernel debug: dyn dbg/ftrace, or
upstream patches — deep HW/kernel integration), (B) software de-tile
MM21->NV12 in userspace (HW decode + SW untile; moderate CPU), (C) pure SW
decode (libavcodec; high CPU, defeats HW purpose).

### Video device map (this box)

- /dev/video0 = mtk-mdp3 (broken conversion)
- /dev/video1,2 = uvcvideo (webcam)
- /dev/video3 = mtk-vcodec-enc
- /dev/video4 = mtk-jpeg-enc
- /dev/video5 = mtk-vcodec-dec (H.264/VP9 decoder, MM21 out only)

## Bridge source re-read + decoder API (2026-10-15)

Re-read the whole RK3588 bridge tree; documented what transfers to MT8183
in **`docs/mt8183-notes.md`**. Key points: the tree is a VA-API→V4L2 libva
backend for RK3588; VPP (RGA) is the stateful-M2M conversion pattern
(`v4l2stateless_vpp.c`); all format converters are linear (no tiling);
the probe would already pick `/dev/video0` (MDP3) as the VPP node
(`has_rotate && !has_coded_output`).

**CORRECTION — the decoder is STATELESS, not stateful.**
`tools/v4l2_dec_probe.c` (new: ENUM_FMT on both MPLANE queues) shows
`/dev/video5` advertises:
  - OUTPUT: **`S264` = `V4L2_PIX_FMT_H264_SLICE`** ("H.264 Parsed Slice Data")
  - CAPTURE: **`MM21`**

So MT8183 mtk-vcodec uses the **V4L2 stateless Request API** (per kernel
source: `uses_stateless_api = true`, `vdec_h264_req_if.c`), NOT the
stateful H.264 path. Consequences (reverses the earlier note):
  - The bridge's **stateless** H.264 translator pattern **IS applicable**.
  - GStreamer's stateless element **`v4l2slh264dec`** should bind to it
    (`v4l2h264dec`/`v4l2dec` stateful elements are NOT installed; only
    `v4l2slh264dec` is).
  - The bridge probe (`v4l2sl_pick_device_for_codec` looks for
    `H264_SLICE`) **would select `/dev/video5`** as the H.264 decoder —
    provided the sibling media node supports `MEDIA_IOC_REQUEST_ALLOC`.

**Installed:** `gstreamer1.0-tools` (gst-launch/gst-inspect), `ffmpeg`.

**HW H.264 DECODE: CONFIRMED WORKING (2026-10-15).**
`gst-launch-1.0 filesrc <bbB.h264> ! h264parse ! v4l2slh264dec !
filesink` on a 15s Big Buck Bunny stream (853x480 H.264, in
`BigBuckBunny.mp4` / elementary `/tmp/bbb.h264`) -> **~372 frames**
(231,413,760 B / 622,080 B-per-frame = 864x480 Y + 864x240 UV MM21).
So the MT8183 HW H.264 decoder **works** via the **stateless** path and
outputs **MM21**. The bridge is viable. (Earlier 320x240 `testsrc` run
yielded nothing; a real stream works. `v4l2slh264dec` has NO `device`
prop — it auto-detects; use `media-device` only for the media node.)

GOTCHAS hit (for the record):
  - Must DEMUX first: `filesrc ! qtdemux ... ! h264parse`, OR feed an
    Annex B elementary stream (`.h264` via `ffmpeg -c:v copy -f h264`).
    Feeding the MP4 container straight to `h264parse` -> "Broken bit stream".
  - `v4l2slh264dec` auto-detects the device (no `device` property).

**RESOLVED (was "STILL OPEN").** The de-tile is done — the bridge de-tiles
MM21→NV12 in `v4l2sl_surface_pull_capture` (commit `0a9b7d1`, chroma layout
corrected) and exports it to Chromium as a GBM bo. MDP3 (HW de-tile) remains
blocked on the GCE EOF bug but is no longer needed.

The "Modules linked in" dmesg traces were **a real bug and are fixed**: they
were the `media_request_object_bind` / `v4l2_ctrl_request_complete` WARNs from
`mtk_vdec_worker`, caused by the bridge putting the request fd in `ctrls->which`
instead of the `request_fd` field (`7ea290a`). `dmesg` is now clean. The only
residual kernel warning is a benign `__flush_work` (workqueue.c), which
GStreamer's `v4l2slh264dec` also produces — a driver teardown quirk, not the
bridge.
