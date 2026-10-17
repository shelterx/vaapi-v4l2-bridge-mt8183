# Hardware Video decode for MT8183 (Mali-G72 MC3)

This was developed using Chromium 155 and Firefox 158 on Debian Trixie under Wayland.  
Firefox needs two extra environment variables to be set:  
`MOZ_DISABLE_RDD_SANDBOX=1 MOZ_DISABLE_CONTENT_SANDBOX=1`  
Chromium need no extra configuration other than the environment variable below.  
Vulkan renderer does NOT work, you need to use GL.  

The h264ify extension is recommended for Chromium/Firefox to get HW Acceleration on Youtube.

> [!NOTE]
> **AI TRANSPARENCY**  
> Qwen 3.8 27b and Deepseek-4.1-Flash was used to make this possible.  
> Human feedback and testing was done, no automatation was involved in the process.  


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

and/or `~/.profile`. Then:

```text
app  →  libva  →  v4l2stateless_drv_video.so  →  /dev/video5 (mtk-vcodec-dec, stateless)
      →  MM21 capture  →  SW de-tile  →  NV12  →  GBM bo  →  GPU/compositor
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
`debug` (`-O0 -g`). Chrome and the matrix load the copy under
`/usr/lib/.../dri/`, so that install must come from this release build.

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

## Folders

docs - implementation information
scripts - original project, never used while implementing this for mt8183, kept for reference
src - the main source for vaapi-v4l2-bridge-mt8183
tests - tests from the original project
tools - some tools/tests used for mt8183.

## Original project

This was based on https://github.com/pty819/vaapi-v4l2-bridge?utm_source=chatgpt.com
