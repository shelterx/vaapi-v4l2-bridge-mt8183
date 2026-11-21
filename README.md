# Hardware Video decode for MT8183 (Mali-G72 MC3)

This was developed using Chromium 155 and Firefox 158 on Debian Trixie under Wayland. Continued development was done on Debian Sid.  
Confirmed working on kernel 6.18.34 but the recommended kernel is 6.18.60 or above.

The h264ify extension is recommended for Chromium/Firefox to get HW Acceleration on YouTube.

> [!NOTE]
> **AI TRANSPARENCY**  
> Qwen 3.8 27b and Deepseek-4.1-Flash were used to make this possible.  
> Human feedback and testing was done, no blind automation was involved in the process.

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

After installing run:

```bash
LIBVA_DRIVER_NAME=v4l2stateless vainfo --display drm --device /dev/dri/renderD128
```

Then verify that it finds the H264 VAProfiles:

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

## Application configuration

##### **REQUIRED environment variable**:

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

**NOTE**: Different desktop environments uses different config files, so be sure to set the env variable correct for those.

##### Wayland

**KDE/Gnome**:
KWin/Mutter seems to inferfere with the compositing somehow on panfrost causing  stutter in the web browsers and some apps, it seems to be unrelated to tbe v4l2 bridge.

[Wayfire](https://github.com/wayfirewm/wayfire) or [labwc](https://github.com/labwc/labwc) does not seem to have this issue, so use those or test which DE/Compositor that works best for you.

An easy way to test if you have stutter is to run the [VSync Tester](https://www.vsynctester.com/) in a browser.

**Chromium**:
No special configuration needed

**Firefox**:
Firefox seem to have performance issues in general, root cause not found yet. Use a chromium based browser if possible.

**mpv**:
Start mpv with the following args:
`mpv --vo=dmabuf-wayland --hwdec=vaapi <file.mp4>`

##### X11

Not supported. 

## Optional settings

You normally do not need any of these.

```bash
# Use the CPU de-tile synchronously instead of in the background.
V4L2SL_DETILE_ASYNC=0

# Default: auto. One stream gets the MDP3 hardware de-tile when this box has
# the m2m node, any further stream gets the GPU shader, and the CPU is the
# last resort. If the MDP3 node is missing the first stream takes the GPU
# instead of the CPU, and if MDP3 gives up mid-session (firmware that opens but
# cannot run a job) that stream steps down to the GPU, not to the CPU.
# Force one tier for every stream with =hw, =gpu or =sw.
V4L2SL_DETILE=auto

# Print frame timings every 120 frames, or write them to a CSV file.
V4L2SL_PROFILE=120
V4L2SL_TRACE=/tmp/frames.csv

# Ignore the cached device scan and look for the decoder again.
V4L2SL_PROBE_NOCACHE=1
```

## Flow

When using this bridge the flow becomes:

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

## Folders

docs - implementation information  
scripts - kept from the original project, cdp helpers etc.  
src - the main source for vaapi-v4l2-bridge-mt8183  
tests - tests from the original project  
tools - some tools/tests used for mt8183

## Original project

This was based on https://github.com/pty819/vaapi-v4l2-bridge
