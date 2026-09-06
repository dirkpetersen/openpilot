# AGNOS19-COMPAT — run 3devpnw (openpilot 0.11.1) on AGNOS 19.6 / 19.7

## Problem

The comma 3X was upgraded to **AGNOS 19.6** (by flashing `xnor-sync-master`, openpilot 0.11.2).
Installing **pnw-pilot `3devpnw`** (openpilot **0.11.1**, which pinned **AGNOS 17.2**) on top of it
**hangs at the comma logo** — boot never finishes.

Two independent breakages, both fixed here:

### 1. Unconditional AGNOS downgrade-flash (fixed by the pin)
`launch_chffrplus.sh:agnos_init` runs `if [ $(< /VERSION) != "$AGNOS_VERSION" ]` → a full 7-partition
AGNOS **downgrade**-flash (no direction / anti-rollback check). With `/VERSION`=19.6 and the old pin
17.2 this fired on every boot. **Fix:** pin `AGNOS_VERSION="19.6"` in `launch_env.sh` and swap
`system/hardware/tici/agnos.json` to the 19.6 partition hashes (schema is identical between 17.2 and
19.6). Versions now match → the flash block is skipped. (commit `agnos: pin 3devpnw to AGNOS 19.6`.)

### 2. AGNOS 19.6's baked venv dropped packages 0.11.1 needs (the real logo-hang)
The device has **no per-tree Python env** — every process runs on the image-baked
`/usr/local/venv`, built at image time from agnos-builder's `userspace/uv/uv.lock`. Between 17.2 and
19.6 that venv shrank **116 → 91 packages**. The ones 0.11.1 still imports:

| dropped pkg | consumer | failure without it |
|---|---|---|
| `bzip2`, `libjpeg`, `libyuv` | `SConstruct:44` `importlib.import_module(...)` | **scons dies before any target** → build fails |
| `casadi` | lat/long MPC codegen (build) + plannerd (runtime) | build fails at MPC step; onroad planner dead |
| `raylib` 5.5.0.2 | the **entire Python UI** (`system/ui`, `selfdrive/ui`) | UI never starts → comma logo forever |
| `pyserial`, `crcmod-plus` | `qcomgpsd` | GPS/mapd dead |
| `xattr` | loggerd → uploader | uploader crash-loop |
| `json-rpc` | athenad | athena/remote dead |

Post-pin the symptom is unchanged (logo forever) because scons now dies at `import bzip2` instead of
the flash loop. (Also present on 19.6 via renamed `comma-deps-*`: capnproto/eigen/ffmpeg/ncurses/
zeromq/zstd, plus psutil/cffi/pillow/sympy/cython/pycapnp 2.1.0 — same on both images. We still
overlay capnproto/eigen/ncurses/zeromq/zstd with 0.11.1's static builds, but take **ffmpeg from the
venv** — see the ffmpeg note below. `/TICI` is still created by 19.6, so the `larch64`/scons-cache
path is fine — verified.)

## Fix: a /data package overlay (survives reflash) + 3 tree edits

Do **not** touch the RO OS image. Stage the exact 17.2-era **aarch64** wheels 0.11.1 expects into
`/data/pnw/agnos19-compat/site-packages` and put that dir on PYTHONPATH for openpilot processes only.
PYTHONPATH precedes the venv site-packages, so the overlay intentionally shadows 19.6's newer builds
(raylib 6.x, shared ffmpeg) with the 5.5 / static builds this tree is written against.

**Overlay contents** (all verified downloadable, no on-device compiling): 8 commaai/dependencies
native-dep wheels (bzip2 1.0.8, capnproto 1.0.1, eigen 3.4.0, libjpeg 3.1.0, libyuv 1922.0,
ncurses 6.5, zeromq 4.3.5, zstd 1.5.6 — static libs+headers), raylib 5.5.0.2
(commaai/raylib-python-cffi release 8 — the exact 17.2 artifact), casadi 3.7.2, crcmod-plus 2.3.1,
xattr 1.3.0, pyserial 3.5, json-rpc 1.15.0, **plus the aiohttp 3.13.3 + av 16.1.0 closure**
(multidict/yarl/frozenlist/aiosignal/attrs/propcache/aiohappyeyeballs + typing_extensions 4.15.0):
manager's `prepare()` pre-imports `webrtcd`/`bodyteleop` (notcar, but `enabled=True`) which do
`from aiohttp import web` at module top, so those must import even though they never *run* on a car.
(`webcamerad`→cv2 needs nothing — it's `enabled=WEBCAM=False`, and `PythonProcess.prepare()` skips
disabled procs.)

> **⚠️ ffmpeg is deliberately NOT overlaid** (learned on the first on-car build). The
> commaai/dependencies ffmpeg wheel is the *off-device* build with VAAPI compiled into
> `libavutil.a`; on larch64, `system/loggerd/SConscript` intentionally does **not** link
> libva/libva-drm (the device ffmpeg has no VAAPI), so shadowing it makes `loggerd` fail to link
> with undefined `va*` symbols. The venv's `comma_deps_ffmpeg` (shared, no-VAAPI, device build) is
> the correct one — 0.11.1 imports it from the venv. Same principle didn't bite the other 5
> dual-present deps (capnproto/eigen/ncurses/zeromq/zstd): their static overlay builds match what
> 0.11.1 expects and compile clean.

**Tree edits (this branch):**
1. `launch_chffrplus.sh` — append the overlay to the launcher `PYTHONPATH` (covers build.py, scons,
   manager, all managed processes).
2. `SConstruct` — append the overlay to `env.ENV["PYTHONPATH"]` too; scons **replaces** the env for
   its codegen subprocesses (`python3 lat_mpc.py` → `from casadi import ...`), so the launcher's
   export alone is not inherited there.
3. `launch_chffrplus.sh:agnos_init` — `rm -f /data/scons_cache/config.lock` (mirrors 0.11.2;
   clears a stale SCons CacheDir lock left by the device's earlier 0.11.2 builds).

## Deploy

`/data` (incl. `/data/params`, i.e. your SSH key) survives openpilot reinstalls and AGNOS reflashes;
it is only wiped by a full factory reset. So switch code **in place over SSH** — never via the
installer, which is the reset path that lost the SSH key before.

```bash
# 0. one-time: get SSH back if locked out — reinstall xnor-sync-master (matches 19.6, boots),
#    add your GitHub SSH key in Settings, then come back here.

# 1. build + stage the overlay (does an on-device import test at the end)
COMMA_IP=192.168.13.154 ./scripts/deploy-agnos19-compat.sh      # SSH_PORT=22 home / 22 hotspot@172.20.10.10

# 2. point /data/openpilot at 3devpnw (which carries the pin + the 3 edits).
#    The submodule + prebuilt handling is REQUIRED if step 0 reinstalled xnor-sync-master:
#    a plain reset leaves the 6 submodules at 0.11.2-xnor content/URLs (broken matched-set),
#    and a leftover `prebuilt` file makes launch skip build.py entirely.
ssh comma@$COMMA_IP 'cd /data/openpilot && git fetch origin 3devpnw && git reset --hard FETCH_HEAD && \
  git submodule sync --recursive && git submodule update --init --recursive --force && \
  git lfs pull && rm -f prebuilt'

# 3. persistence guards + restart (see CLAUDE.md deploy section)
ssh comma@$COMMA_IP 'sudo rm -rf /data/safe_staging/finalized; touch /tmp/booted; \
  tmux kill-session -t comma 2>/dev/null; sudo systemctl restart comma'
```

**First boot is a from-scratch native build** (the scons cache holds 0.11.2 objects) — allow
**~10–20 min** at the spinner with visible progress before judging it hung.

### Verify after boot
- `ls -la /TICI /AGNOS` — both present.
- Offline import in the venv: `source /usr/local/venv/bin/activate;
  PYTHONPATH=/data/openpilot:/data/pnw/agnos19-compat/site-packages python3 -c "import bzip2,casadi,pyray,xattr"`.
- manager PIDs stable (not cycling); UI up; no exception in `tmux capture-pane -t comma -p`.
- `du -h selfdrive/modeld/models/*.onnx` — MB-scale, not byte-size LFS pointers (else `git lfs pull`).
- `git submodule status` — no `-` (uninitialized) or `+` (wrong SHA) rows; all six at pnw SHAs.

## First-boot gotchas (observed on the real deploy)
- **`/data/dirk/` is absent after an installer reinstall** (it's normally created by the root deploy
  toolchain, not the installer). `system/location_services/location_servicesd.py` (NON_ESSENTIAL
  overlay) hard-opens `/data/dirk/net_events.jsonl` and crash-loops if it's missing. Quick unblock:
  `mkdir -p /data/dirk && touch /data/dirk/net_events.jsonl`. (Proper fix belongs in that daemon —
  tolerate a missing file — and is unrelated to AGNOS.)
- **First build takes ~10–20 min** and the screen stays on the AGNOS splash the whole time — that is
  the build, not a hang. `pgrep scons` on the device confirms progress.

## AGNOS 19.7 (2026-09-05): the overlay is DELIBERATELY KEPT

`agnos197-2pnw` bumps the pin 19.6 → 19.7 and **changes nothing else**. The overlay stays exactly as
it is. This section records why that is safe, so the next bump is a re-check rather than a
re-investigation. Generic procedure: `docs/AGNOS-UPGRADE-PROCEDURE.md`.

**The overlay is not compensating for AGNOS being broken — it compensates for OUR TREE BEING OLD.**
There is no per-tree Python env; every process runs the image-baked `/usr/local/venv`, whose contents
track the openpilot version the AGNOS image was built for. 0.11.1 still imports things newer
openpilot dropped, and it is written against raylib 5.5 / static native deps rather than 19.x's
raylib 6.x / shared builds. An AGNOS bump can therefore never retire the overlay — only moving to
0.11.2 can, and we are deliberately not doing that.

### What was verified (all by inspecting the real 19.7 rootfs, not inferred)

The 19.7 `system` image was downloaded, its sha256 checked against the manifest
(`74ffc9c5…` = `hash`), unsparsified and read with `debugfs`. Against the live 19.6 device:

| check | 19.6 | 19.7 | verdict |
|---|---|---|---|
| baked Python | `lib/python3.12`, `pyvenv.cfg version_info = 3.12.3` | **identical** | overlay's `cpython-312` wheels stay ABI-correct |
| venv `site-packages` | 213 entries (`ls -A`; 212 without the one dotted dir) | **213, name-for-name identical** | nothing added, nothing dropped, no version moved |
| `raylib` in the venv | `comma_deps_raylib-6.0.0.1` | **same** | our 5.5 shadow-pin behaves exactly as it does today |
| ffmpeg layout | `…/site-packages/ffmpeg/install/lib/libavformat.so.61` | **same path, same soname** | the `LD_LIBRARY_PATH` line stays correct *and* still required (`/etc/ld.so.conf.d` unchanged) |

The only non-`.pyc` content changes anywhere in the venv are rebuilt-but-same-version C extensions
(`evdev` 1.9.3, `spidev` 3.8) and their `RECORD`s. **The 19.7 venv is, for our purposes, the 19.6
venv.** So the overlay is neither more nor less necessary than it is today.

### What 19.7 actually changes on a comma 3X

Manifest delta is `boot` + `system` only — `xbl`, `xbl_config`, `abl`, `aop`, `devcfg` are
byte-identical **relative to what slot `_a` is already running**, so no new bootloader code enters
the device. Note this is not the same as "only two partitions get written": the inactive slot `_b`
currently holds 17.2, so `flash_agnos_update` will write **all seven** partitions into `_b` — the five
bootloader ones just get the same content `_a` already runs. Between the two agnos-builder build SHAs
(`/BUILD`: 19.6 `132d0064`, 19.7 `7c3d9c0f`) there are exactly two functional commits:

- **`0e95fd5db7` "disable unused ALSA state restore"** → masks `alsa-restore.service` and
  `90-alsa-restore.rules` to `/dev/null`. This is upstream's "fix soundd/micd race". OS-side only.
- **`72c29bb290` "bump tinygrad GPU firmware" (#619)** → touches **only `userspace/files/amdgpu/`**
  (`gc_12_0_0_{me,mec,pfp}`, `smu_14_0_2`). That is AMD firmware for comma's x86 dev hardware; the
  3X's GPU is a Qualcomm Adreno. **Inert on this device**, and in particular it does *not* couple to
  the tinygrad pinned in our tree.

Everything else in the rootfs diff is rebuild noise (apt/dpkg logs, ccache stats, recompiled `.pyc`,
rebuilt CPython/Qt-example binaries). The `boot` image differs in **1529 bytes across 31 ranges** —
kernel version banner timestamp (`Aug 12 01:39:16 UTC 2026` → `Sep 2 00:28:38 UTC 2026`), GNU
build-IDs, and the signing certificate serial. The kernel **was recompiled** — what the evidence
supports is the narrower claim that there is **no kernel *source* change** between the two builds:
the agnos-builder history between the two `/BUILD` SHAs contains no kernel commit, and the 1529
differing bytes are all build-identity/signature fields, not code.

> **Provenance, and one trap in it.** The GitHub compare
> `commaai/agnos-builder/compare/132d0064...7c3d9c0f` is **merge-base-relative**, so it is easy to
> misread: `132d0064` (the 19.6 image's `/BUILD`) sits on PR #613's *branch*, and `c6a8e491ab` in the
> compare's list is that same PR *squashed onto master*. Its file list (kernel `1b22fd32→eccd1465`,
> `firmware/abl.img`, the zipapps) is therefore the whole 19.6 PR against pre-19.6 master — **not**
> the 19.6-image→19.7-image delta. The real image-to-image delta on master after that squash is just
> `0e95fd5db7` (ALSA), `72c29bb290` (AMD fw) and `7c3d9c0f18` (VERSION).
>
> **The kernel evidence is therefore the submodule pointer, not the compare** — and it is direct:
> `agnos-kernel-sdm845` is `eccd146599f2e2f159d951092642689bede91632` at **all three** of `132d0064`,
> `c6a8e491ab` and `7c3d9c0f18` (`gh api repos/commaai/agnos-builder/contents/agnos-kernel-sdm845?ref=<sha>`).
> The kernel pin did not move; the recompiled binary differs only in the 1529 build-identity bytes
> (`cmp -l`), banner `Aug 12` → `Sep 2`. Corroboration that the 19.6 image already carried the PR
> head's content: `abl.img`'s LFS oid at `132d0064` is `29fd7ed1…`, which is exactly the flashed `abl`
> hash (identical in both manifests), and `/usr/comma/{setup,reset,updater}` are byte-identical
> between the two extracted rootfs trees. There is no agnos-builder clone in this workbench — re-run
> those API reads at the next bump rather than trusting this paragraph.

### How the flash actually reaches the device (the two paths differ a LOT)

Both paths are triggered by the same string comparison, but they cost very different amounts of
driver time. Verified by reading `system/updated/updated.py` and `launch_chffrplus.sh`.

- **Auto-update path (what this device does — it tracks `3devpnw` and `DisableUpdates` is unset).**
  `updated.py:handle_agnos_update()` compares `HARDWARE.get_os_version()` against the pin sourced from
  the *incoming* tree, and if they differ it runs `flash_agnos_update()` **in the background, while the
  device is running**, writing the inactive slot and holding `Offroad_NeosUpdate` up meanwhile. It sets
  `set_consistent_flag(False)` first, so a half-flashed AGNOS can never be swapped in with an
  openpilot update. It does **not** call `swap()`. The next boot's `agnos_init` finds the slot already
  verifying, swaps and reboots. **Cost to the driver: one extra fast reboot.**
- **Manual/urgent git-deploy path** (`git fetch && git reset --hard` on the device, then restart).
  Nothing is pre-flashed, so `agnos.py --verify` fails at boot and `launch_chffrplus.sh:31` runs
  `updater_magic` in the FOREGROUND: an *"Update Required"* screen that waits for a tap, then does the
  whole ~976 MB download and flash before openpilot starts. **Cost: a tap-gated outage of roughly an
  hour.** Do not pick this path unless you mean to babysit it.

Two more things worth knowing before you schedule it:

- **Download is ~976 MB, not 4.72 GB.** The manifest `size` is the uncompressed partition; the
  transfer is the `.img.xz` (`boot` 14 MB + `system` 963 MB, HTTP HEAD-measured 2026-09-05). Our
  manifest has **no `casync_caibx`** entry, so `flash_partition` takes `extract_compressed_image` — a
  plain full download, not a casync delta, streamed **straight into the partition**. No staging file,
  so `/data` free space is not a constraint (relevant: `/data` was 90% full on 2026-09-05).
- **The metered-connection gate is not absolute.** `updated.py:487` skips fetching on
  `NetworkMetered` only while `UpdaterLastFetchTime` is under **3 days** old. After a 3-day gap it
  fetches regardless — so ~976 MB can land on LTE. If the truck will be away from WiFi, pause updates
  or make sure it fetches on WiFi first.

### The boot-path `python3` is the VENV python — `agnos.py --verify` is NOT broken

`agnos.py:13` has a top-level `import openpilot.system.updated.casync.casync`, and `casync.py:15`
does `from Crypto.Hash import SHA512`. `Crypto` (pycryptodome 3.23.0) exists **only** in
`/usr/local/venv/lib/python3.12/site-packages` — it is deliberately *not* in the agnos19-compat
overlay. That reads like `agnos_init`'s `agnos.py --verify` can never run, so the clean swap at
`launch_chffrplus.sh:29` can never fire and every AGNOS bump falls through to the tap-gated
`updater`. **That is wrong**, and it is an easy thing to "reproduce" incorrectly. Checked
end-to-end on the own-car 3X on 2026-09-05:

- `launch_chffrplus.sh:29` runs `$AGNOS_PY` **directly**, so its shebang `#!/usr/bin/env python3`
  resolves `python3` through **PATH** — it never names an interpreter.
- `comma.service` → `/usr/comma/comma.sh` → `source /etc/profile` → `/etc/profile:40`
  `source /usr/local/venv/bin/activate`. `/etc/profile:43` then prepends `/usr/comma/shims`, which
  contains only `pip`, `pip3`, `uv` — **no `python3`**, so it cannot re-point the interpreter.
- Measured from the **live running `manager.py`**'s `/proc/<pid>/environ` (the real boot env, and the
  same shell environment `agnos_init` runs in):
  ```
  VIRTUAL_ENV=/usr/local/venv
  PYTHONPATH=/data/openpilot:/data/pnw/agnos19-compat/site-packages
  PATH=/usr/comma/shims:/usr/local/venv/bin:/usr/local/.cargo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin
  ```
  `/usr/local/venv/bin` precedes `/usr/bin`, so `env python3` **is** the venv python and `Crypto`
  resolves normally.
- `agnos_init` is called at `launch_chffrplus.sh:137` — inside `launch`, **after** the overlay
  `PYTHONPATH` export. The overlay is genuinely required for this import chain (without it it dies
  on `import serial`, `system/hardware/tici/lpa.py:7`), and it is already on the path by then.

**Reproducing it correctly.** Use a login shell (so `/etc/profile` runs) and let the shebang choose
the interpreter. Do **not** hardcode `/usr/bin/python3`: that interpreter sees neither the venv nor
`Crypto`, and yields a `ModuleNotFoundError: No module named 'Crypto'` that does not occur at boot.

```bash
ssh comma@$COMMA_IP 'bash -lc "cd /data/openpilot && \
  export PYTHONPATH=/data/openpilot:/data/pnw/agnos19-compat/site-packages && \
  ./system/hardware/tici/agnos.py --help"'        # exit 0 -> the casync/Crypto import is fine
```

**⚠️ `--verify` is NOT read-only, so never use it as a probe.** On success `agnos.py`'s `__main__`
calls `swap()`, which `clear_partition_hash()`es the target slot (a write to the partition) and then
runs `abctl --set_active` — see the rollback section below. To test verification *without* swapping,
call the predicate the CLI branches on:

```bash
ssh comma@$COMMA_IP bash -l <<'EOF'
cd /data/openpilot
export PYTHONPATH=/data/openpilot:/data/pnw/agnos19-compat/site-packages
python3 -c 'from openpilot.system.hardware.tici.agnos import get_target_slot_number as g, verify_agnos_update as v
s = g(); print("target slot", s, "verifies:", v("system/hardware/tici/agnos.json", s))'
EOF
```

(Note whose manifest you point it at. On 2026-09-05 the device's own tree was still at `d54a88e`
with `AGNOS_VERSION="19.6"`, so `/data/openpilot/system/hardware/tici/agnos.json` is the **19.6**
manifest and the snippet above prints `False` — correctly, since `_b` holds 19.7 images. Point it at
the 19.7 manifest from `3devpnw` (`c96e18f9d2`) to see the real answer.)

Result on 2026-09-05 (device on 19.6/`_a`, 19.7 already flashed to `_b` by `updated.py`'s background
`flash_agnos_update()`): **all seven partitions of the 19.7 manifest verify `True` against slot `_b`**
— i.e. `--verify` would exit 0 and `launch_chffrplus.sh:29`'s `sudo reboot` would take the clean
one-fast-reboot swap. As a negative control, the same unmodified CLI pointed at the AGNOS **17.2**
manifest runs with no traceback and correctly exits 1 (all seven partitions `False`). Nothing needs
to be added to the overlay for this path.

**And do not "fix" it anyway** — putting `pycryptodome` in the overlay would be actively harmful, not
merely wasteful. `PYTHONPATH` precedes the venv's `site-packages`, so an overlay `Crypto` would
**shadow the image-baked pycryptodome 3.23.0 for every openpilot process** (uploader, `updated`,
casync) — a system-wide swap to fix a non-bug. Putting the venv itself on the boot `PYTHONPATH` is
worse still: it inverts the shadowing this whole overlay exists to create (see the top of this doc).
Note also that `pycryptodome` alone would not even have been sufficient: `/usr/bin/python3` cannot
import `agnos.py` without `zmq`, `numpy`, `capnp` and `zstandard` either — ~100 MB in total, all of
it already present in the venv the boot path actually uses.

Two loose ends this pass surfaced, flagged but **not** fixed here:

- **`agnos19-compat/overlay.tar.zst` is NOT git-LFS**, despite what `launch_chffrplus.sh:92`'s
  comment ("bundled in the repo via git-LFS") says. `git check-attr filter` returns `unspecified` and the
  committed blob is 68,937,948 raw bytes starting with the zstd magic `28 b5 2f fd` — a plain git
  object. Every byte added to it is permanent clone weight for everyone, with no LFS lazy-fetch.
- The PATH reasoning above is read off **19.6's** `/etc/profile`. The 19.7 audit in this doc compared
  `/usr/comma/*` and the venv, not `/etc/profile`. If a future AGNOS stops activating the venv there,
  this analysis needs re-running — that, not `Crypto`, is the thing that would actually break the
  swap path.

**Residual, stated honestly:** everything above proves the *verification* half. The `swap()` half —
`abctl --set_active` as user `comma`, plus its `while True` retry loop with no sleep or bail-out
(`agnos.py:266-272`) — has never been exercised on this device under 19.6. It is unmodified upstream
code that stock devices run on every AGNOS bump, and `comma` is in `disk`/`sudo`, so it is expected
to work; but the first real execution will be the 19.7 boot.

### If it goes wrong: rolling back off 19.7

**What actually happens if you just switch slots (verified by reading `agnos.py`, not assumed).**
`swap()` (`agnos.py:260-264`) calls `clear_partition_hash()` on every `full_check: false` partition of
the slot it is about to activate — that is `system` — *before* `abctl --set_active`. So the moment
19.7 goes live, `_b`'s `system` trailing marker is **zeroed**. If you then boot `_a` (19.6) with the
tree still pinned to `19.7`, `agnos_init` sees `/VERSION != $AGNOS_VERSION`, runs
`agnos.py --verify` against `_b`, reads that zeroed marker and **fails** — so
`launch_chffrplus.sh:29`'s `sudo reboot` never fires. Instead line 31 launches `updater_magic`, which
opens on the *"Update Required"* prompt and **waits for a tap**. AGNOS's own sshd is up, so you do get
a shell. **Do not tap Install** — that re-downloads ~1 GB and puts you straight back on 19.7.

So the failure mode is a stuck prompt you can SSH into, not an unrecoverable bounce. Fix it in this
order; the ordering is what matters:

```bash
# 0. STOP THE UPDATER FIRST. Without this, updated.py refetches the still-pinned tip and
#    background-flashes 19.7 into the inactive slot again -> the NEXT reboot flips forward.
#    (The loop honours this live, no reboot needed -- system/updated/updated.py:458.)
ssh comma@$COMMA_IP 'echo -n 1 > /data/params/d/DisableUpdates'

# 1. Un-pin the device's own tree.
ssh comma@$COMMA_IP "sed -i 's/AGNOS_VERSION=\"19.7\"/AGNOS_VERSION=\"19.6\"/' /data/openpilot/launch_env.sh"
ssh comma@$COMMA_IP 'grep AGNOS_VERSION /data/openpilot/launch_env.sh'   # must say 19.6

# 2. Drop any staged overlay, or step 1 is SILENTLY UNDONE at the next boot. The sed does not touch
#    .git, so the launcher's "tree has been modified" guard (launch_chffrplus.sh:48-51) does NOT
#    engage, and a finalized/.overlay_consistent tree is moved over /data/openpilot before agnos_init
#    ever runs -- restoring the 19.7 pin without a word.
ssh comma@$COMMA_IP 'sudo rm -rf /data/safe_staging/finalized'

# 3. Point the bootloader back at the slot you came from.
ssh comma@$COMMA_IP 'abctl --boot_slot'            # _a = slot 0, _b = slot 1
ssh comma@$COMMA_IP 'sudo abctl --set_active 0'    # 0 if you came from _a

# 4. Reboot (car DISENGAGED -- that is the only precondition).
ssh comma@$COMMA_IP 'sudo reboot'

# 5. Make it permanent: revert the AGNOS_VERSION commit on 3devpnw and push.
# 6. Only then re-enable updates.
ssh comma@$COMMA_IP 'echo -n 0 > /data/params/d/DisableUpdates'
```

**Alternative with no `abctl` at all** (slower, but fewer moving parts): while still on 19.7, revert
the pin commit and push. `updated.py`'s `handle_agnos_update()` then sees `19.7 != 19.6`, and
background-flashes **19.6 back into the inactive slot** — whose marker `swap()` zeroed, so it really
does re-flash rather than short-circuit. The next reboot swaps you onto it. Use this when the device
is healthy enough to update but you want off 19.7. ⚠️ But note `flash_agnos_update()` opens
with `abctl --set_unbootable` on the *target* slot — here `_a`, your known-good 19.6 — so for the
~1 h of that re-flash the device has **no bootable fallback** if the running 19.7 slot dies. Every
upgrade has this window; this is the one rollback path that spends the good slot to get it.

**If 19.7 does not boot at all.** The bootloader's A/B fallback returns you to `_a` on its own once
the new slot exhausts its boot attempts (`agnos_init` is what calls `abctl --set_success`, so a slot
that never finishes launching never gets marked good). *The exact retry count on this hardware was
not verified* — treat the fallback as "it should come back to `_a`", not as a guarantee. You then
land on the same Update Required prompt described above, with sshd up: **do not tap Install**, SSH in,
and run steps 0-6. **Do NOT reach for the tap-reset / factory-reset path** — that wipes `/data`,
taking the SSH key and the compat overlay with it, which is exactly what the Deploy section above
says never to do.

Verified 2026-09-05: the device is running slot **`_a` = 19.6** (ext4 superblock write time
2026-08-12, matching the 19.6 release), and the inactive slot **`_b` holds AGNOS 17.2**
(superblock 2026-03-11) — the leftover noted at the bottom of this doc. So the 19.7 flash overwrites
the stale 17.2 image and leaves a **known-good 19.6 in `_a`**, which is the rollback target. Note the
corollary: you only ever get ONE previous version back. The 19.6 images are also still fetchable at
their manifest URLs (HTTP 200 on 2026-09-05, `boot` 14 MB / `system` 1.01 GB `.img.xz`), and those
URLs live on in `git log -p -- system/hardware/tici/agnos.json`.

## Caveats / follow-ups
- **Auto-update:** this must stay merged into `3devpnw` (the one own-car device, already on 19.6), or
  the next update reinstalls the 17.2 pin and re-triggers the downgrade loop.
- **⚠️ Do NOT merge to `3testpnw` (friends channel) yet.** The overlay is a *manual per-device* deploy;
  the pin alone would make a friend's device (on 17.2) *upgrade*-flash to 19.6 (via `agnos_init` or
  `updated.py`'s background flash) and then hang exactly like this one, with no overlay staged. Hold
  until the overlay staging is automated (or done per-device) before this reaches `3testpnw`.
- **`Crypto` / `agnos.py --verify` is a false alarm.** The AGNOS swap path works; it runs under the
  venv python via PATH. See "The boot-path `python3` is the VENV python" above before adding anything
  to the overlay for it. Verified 2026-09-05.
- This is a **compat bridge**, not the endgame. The clean 19.6 answer is a real forward-port of the
  `*2pnw` features onto the 0.11.2 (`xnor-sync-master`) base.
- Inactive A/B slot may hold a part-written 17.2 image from the pre-pin flash attempts; `abctl
  --set_success` on the good slot once booted.
