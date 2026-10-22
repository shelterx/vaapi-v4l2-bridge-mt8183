# Hardware Video decode for MT8183 (Mali-G72 MC3)

This was developed using Chromium 155 and Firefox 158 on Debian Trixie under Wayland.  
Firefox needs two extra environment variables to be set:  
`MOZ_DISABLE_RDD_SANDBOX=1 MOZ_DISABLE_CONTENT_SANDBOX=1`  
It also needs the video settings from `scripts/firefox-vaapi-user.js` (copy it
into your Firefox profile as `user.js`).  
**NOTE**: Firefox might have performance issues in general; use Chromium if possible.
Chromium needs no extra configuration other than the environment variable below.  
Vulkan renderer does NOT work, you need to use GL.  

The h264ify extension is recommended for Chromium/Firefox to get HW Acceleration on YouTube.

> [!NOTE]
> **AI TRANSPARENCY**  
> Qwen 3.8 27b and Deepseek-4.1-Flash were used to make this possible.  
> Human feedback and testing was done, no automation was involved in the process.  


## REQUIRED environment variable

Every VA-API client must load **this** driver, not `panfrost_drv_video.so`:

```bash
export LIBVA_DRIVER_NAME=v4l2stateless
```

Persist it for graphical sessions (systemd user environment, not only `~/.bashrc`):

```bash
# ~/.config/environment.d/90-libva.conf
LIBVA_DRIVER_NAME=v4l2stateless
```

and/or `~/.profile`.

The flow then becomes:

```text
App (Chromium / Firefox)
  → libva
  → v4l2stateless driver
  → hardware decoder (/dev/video5)
  → tiled frame (MM21)
  → de-tile to normal NV12
  → shared GPU buffer
  → screen
```

Install the `.so` first (see Build / install). Check:

```bash
LIBVA_DRIVER_NAME=v4l2stateless vainfo --display drm --device /dev/dri/renderD128
```

Verify that it finds the VAProfiles:

```
vainfo: Supported profile and entrypoints
      VAProfileH264ConstrainedBaseline: VAEntrypointVLD
      VAProfileH264Main               : VAEntrypointVLD
      VAProfileH264High               : VAEntrypointVLD
      VAProfileH264High10             : VAEntrypointVLD
      <unknown profile>               : VAEntrypointVLD
      VAProfileJPEGBaseline           : VAEntrypointEncPicture
      VAProfileNone                   : VAEntrypointVideoProc
```

## Build / install

`meson.build` pins **`buildtype=release`** (`optimization=3`, `b_ndebug=if-release`).
A fresh `meson setup` compiles **`-O3 -DNDEBUG`** — not meson's default
`debug` (`-O0 -g`). Chrome loads the copy under `/usr/lib/.../dri/`, so
that install must come from this release build.

```bash
meson setup builddir          # release / -O3; do not pass -Dbuildtype=debug
ninja -C builddir
sudo cp -f builddir/v4l2stateless_drv_video.so \
  /usr/lib/aarch64-linux-gnu/dri/
export LIBVA_DRIVER_NAME=v4l2stateless
vainfo --display drm --device /dev/dri/renderD128
```

An **already-configured** `builddir` keeps whatever `buildtype` it was
set up with. If `meson configure builddir` shows `debug` / `optimization=0`,
reconfigure before ninja:

```bash
meson configure builddir -Dbuildtype=release -Doptimization=3 -Db_ndebug=if-release
ninja -C builddir
sudo cp -f builddir/v4l2stateless_drv_video.so \
  /usr/lib/aarch64-linux-gnu/dri/
```

## Optional settings

You normally do not need any of these.

```bash
# Use the CPU de-tile synchronously instead of in the background.
V4L2SL_DETILE_ASYNC=0

# Use the MDP3 hardware de-tile (works, but might be slower).
# The firmware must also support it.
V4L2SL_DETILE=hw

# Print frame timings every 120 frames, or write them to a CSV file.
V4L2SL_PROFILE=120
V4L2SL_TRACE=/tmp/frames.csv

# Ignore the cached device scan and look for the decoder again.
V4L2SL_PROBE_NOCACHE=1
```

## Folders

docs - implementation information  
scripts - kept from the original project, cdp helpers etc.  
src - the main source for vaapi-v4l2-bridge-mt8183  
tests - tests from the original project  
tools - some tools/tests used for mt8183

## Original project

This was based on https://github.com/pty819/vaapi-v4l2-bridge
