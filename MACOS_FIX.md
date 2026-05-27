# macOS standalone mode + fixes

Changes needed to run and reliably capture from the module on macOS without `viam-server`.

## 1. Standalone capture mode — `src/module/main.cpp`

Production main expects a unix-socket path and calls `vsdk::ModuleService::serve()`. There was no way to drive the camera locally for debugging.

Added `--standalone [out_dir] [--width N] [--height N] [--count N] [--delay SECONDS] [--timeout SECONDS] [--log-level=debug]`:

- Skips `ModuleService` entirely.
- Constructs a `Realsense<boost::synchronized_value<rs2::context>>` directly with an in-memory `vsdk::ResourceConfig` (sensors = `["color","depth"]`, no serial number → first device).
- Requests **640x480 by default** (see §3 for why); override with `--width`/`--height`.
- Captures `--count` framesets (default 1), sleeping `--delay` seconds between them (default 1.0; accepts fractional seconds). Each capture grabs **both color and depth**.
- Polls `get_images()` for up to `--timeout` seconds per capture, default 10 (the librealsense pipeline pushes frames asynchronously via callback; the first `latest_frameset_` arrival can take a few hundred ms; subsequent captures return promptly while streaming).
- Writes `color.jpg` / `depth.viam` for a single capture, or `color_NNN.jpg` / `depth_NNN.viam` (zero-padded index) when `--count > 1`, to `out_dir` (cwd by default), then tears down. (Point-cloud output was dropped from standalone — it's a color+depth capture tool now. Easy to re-add behind a flag if needed.)
- Keeps the same `geteuid() != 0` check on macOS that `serve()` has, with a usage hint.

Build (unchanged from production):
```
make conan-install-test
make conan-build-test
sudo build-conan/build/Release/viam-camera-realsense --standalone /tmp/out
```

## 2. macOS `set_option` pre-`pipe.start()` — `src/module/device_impl.hpp`

`disableAutoExposurePriority()` was called from `createSwD2CAlignConfig()` before the pipeline was started. On macOS this fails:

```
[disableAutoExposurePriority] Failed to disable Auto-Exposure Priority: failed to set power state
[assign_and_initialize_device] Failed to access/initialize device at index 0: failed to set power state
```

Root cause: `sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0)` issues a UVC extension-unit command over libusb. On macOS that fails with `RS2_USB_STATUS_ACCESS` because the macOS IOKit/UVC backend doesn't grant the command interface until the pipeline is actively streaming. The local `try/catch` swallowed the first throw, but the failure leaves the sensor in a stuck state and the very next call on it (`color_sensor.get_stream_profiles()` in `createSwD2CAlignConfig`) re-throws the same error, killing init.

Fix: skip the `set_option` call on macOS with `#if defined(__APPLE__) { return; }`. Linux path is unchanged.

Cost: the constant-FPS guarantee that `AUTO_EXPOSURE_PRIORITY = 0` provides (avoiding ~5–20 ms color/depth timestamp drift in changing light) is not enforced on macOS. Acceptable for now — pipeline starts and frames flow.

A safer long-term fix is to move device-firmware `set_option` calls to *after* `pipe.start()`, where the sensor's command channel is reliably available on macOS. Anything that goes through a UVC XU command (most exposure/gain/depth-table settings) has the same risk on macOS; the global-timestamp option works pre-start because it's handled in librealsense itself, not in firmware.

## 3. Default to 640x480 — macOS backend instability at higher resolution

`failed to set power state` (libusb `RS2_USB_STATUS_ACCESS`) turned out not to be tied to a single call. Across runs on the same machine/camera it fired at `set_option`, at `get_stream_profiles`, and at `pipe.start`; once the pipeline started but delivered no frames; and once threw `No device connected` after an ~11s hang. These are all USB control transfers to the camera — the librealsense 2.57.7 macOS UVC/libusb backend services them unreliably, and the failure rate is much higher at 1280×720 (the profile the module implicitly picked when no resolution was set).

Mitigation: `--standalone` requests 640×480 by default. After the device settles, 8/8 consecutive runs at 640×480 succeeded. Higher resolutions remain available via `--width`/`--height` for reproducing the flakiness. If a run does fail, simply retrying (and occasionally a physical replug) clears it. The production `viam-server` path can't retry by hand, so it also gets an automatic retry — see §4.

## 4. Retry device init on macOS — `src/module/realsense.hpp`

`--standalone` is forgiving because a human can just re-run it. Production isn't: `viam-server` calls `assign_and_initialize_device()` once during resource construction/reconfigure, and a single transient "failed to set power state" (the per-USB-transfer flakiness from §3) would leave the camera dead until the next reconfigure.

The per-device loop in `assign_and_initialize_device` already caught exceptions, but only to skip to the *next* connected device — it never retried the *same* device. Added a macOS-only retry around the `createDevice` + `startDevice` pair:

- `kDeviceInitAttempts` = 5 on macOS, 1 on Linux (Linux behavior is unchanged).
- Each attempt builds a *fresh* device wrapper (new `rs2::pipeline`/`rs2::config`) so a half-started pipe from a failed attempt is discarded (`device_ = nullptr` before retry) rather than reused.
- Backoff is `200 ms × attempt` (0.2/0.4/0.6/0.8 s between tries, ~2 s worst case).
- A `nullptr` from `createDevice` (unsupported camera model / no usable config) is treated as permanent and breaks immediately — no point retrying.
- If all attempts fail, it falls through to the hardware-reset recovery below (macOS) or throws (Linux); on a throw the existing outer `catch` logs and moves on to the next connected device, exactly as before.

### Hardware-reset recovery (macOS only)

Rapid open/close cycling (the standalone retry-loop torture test) can leave the camera fully *wedged* — every USB control transfer fails, and the ~2 s of in-loop backoff isn't enough to clear it. When the normal attempts return `Transient`, macOS now does a last resort:

1. `dev->hardware_reset()` — power-cycles the camera over USB.
2. Polls `realsense_ctx_->query_devices()` every 500 ms (up to 10 s) until the same serial re-enumerates.
3. Re-acquires the fresh device handle and runs the attempt loop once more.

Caveat: `hardware_reset()` is itself a control transfer, so on a badly wedged device the reset can also throw — in which case recovery gives up and the outer `catch` moves on (a physical replug is then the only fix). This recovery is compiled out on Linux (`#if defined(__APPLE__)`); observed behavior there is unchanged.

Note: on total failure the device's serial stays in `assigned_serials_` (pre-existing behavior — the serial is reserved when grabbed and not released on init failure). Not changed here; flagged as a separate question.

Note the module's profile matcher (`createSwD2CAlignConfig`) filters on width/height but not fps, so requesting 640×480 selects the first matching pair, which is 640×480@60.

## Library version

Already on the newest packaged build: `conanfile.py` pins `librealsense/2.57.7`, which is the max available in the `viamconan` conan remote (ConanCenter only has ≤2.56.5, and the README's macOS baseline is the older 2.57.6). The instability above is present in 2.57.7; a point upgrade is not expected to fix it and no newer conan package exists, so we stay on 2.57.7. A genuinely newer build would require packaging an upstream `realsenseai/librealsense` tag into a custom conan recipe.

## Verified

D435, librealsense 2.57.7, macOS 26.5 / Apple silicon. At 640×480: cold start to first frameset ~1 s; `color.jpg` is a valid 640×480 JPEG (~10 KB), `depth.viam` is 614 KB (640×480×2 + header), `pointcloud.pcd` ~4.9 MB. The `[get_images] no frameset available` lines that print during the polling window are expected — they stop as soon as the librealsense callback fires.
