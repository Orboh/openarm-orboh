# OpenArm Setup Log

**Host:** kotaueda-ThinkPad-P16-Gen-2 (Ubuntu 22.04 jammy)
**User:** kota-ueda (koutaueda@orboh.com)
**Started:** 2026-05-15
**Goal:** Bring up OpenArm leader + follower on this PC, then distill into a Claude Code custom agent (`.claude/agents/openarm-engineer.md`).

Official docs root: <https://docs.openarm.dev/>

---

## Documented official setup order

Per <https://docs.openarm.dev/software/setup/>:

1. **Motor ID Configuration** — `/software/setup/motor-id` (Damiao GUI tool, UART @ 921600 bps; usually pre-set at factory)
2. **Setup CAN Interface** — `/software/setup/can-setup`
3. **Motor Test** — `/software/setup/configure-test`
4. **Motor Configuration** — `/software/setup/motor-config`
5. **Hands-On: Demo Run** — `/software/setup/run-demo`

Then: **Teleop / Leader-Follower** — `/teleop/leader-follower/` (Unilateral → Bilateral) and **ROS2** — `/software/ros2/install`.

Canonical CAN IDs (OpenArm 7-DoF + gripper):

| Joint | Sender (motor → host) | Master (host → motor) |
|---|---|---|
| J1–J7 | 0x01–0x07 | 0x11–0x17 |
| J8 gripper | 0x08 | 0x18 |

---

## Phase 1 — APT package install (DONE 2026-05-15)

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y \
  can-utils \
  iproute2 \
  libopenarm-can-dev \
  openarm-can-utils
```

Installed versions:

- `libopenarm-can1` 1.2.8-1.ubuntu22.04.1
- `libopenarm-can-dev` 1.2.8-1.ubuntu22.04.1
- `python3-openarm-can` 1.2.8-1.ubuntu22.04.1
- `openarm-can-utils` 1.2.8-1.ubuntu22.04.1
- `can-utils` 2020.11.0-1
- `python3-can` 3.3.2
- `python3-serial` 3.5-1
- `iproute2` 5.15.0-1ubuntu2.1

PPA key fingerprint: `C11CCA79E6360E6B53729BE81A5C386E0BE68E77`

`ip link show` detected **4 CAN interfaces, all DOWN**: `can0`, `can1`, `can2`, `can3`. This is consistent with **2 USB-CAN-FD adapters × 2 channels each** — enough for one leader (L/R) + one follower (L/R) arm.

---

## Phase 2 — Bring up CAN interfaces (DONE 2026-05-15)

Result: all 4 interfaces UP, `ERROR-ACTIVE`, CAN-FD 1M/5M.

```
31: can0: state UP, bitrate 1000000, dbitrate 5000000, fd on, parentdev 1-5:1.0
    driver: pcan_usb_pro_fd   clock 80MHz
```

**Hardware identified:** PEAK-System **PCAN-USB Pro FD** (2-channel CAN-FD per device) ×2 → 4 channels total. Driver in-tree (`peak_usb`/`pcan_usb_pro_fd`), no DKMS needed.

### Commands used



Two equivalent options per docs.

### Option A — OpenArm helper (recommended)

```bash
sudo openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
sudo openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
sudo openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
sudo openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000
```

### Option B — manual

```bash
for IF in can0 can1 can2 can3; do
  sudo ip link set "$IF" down
  sudo ip link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on
  sudo ip link set "$IF" up
  sudo ip link set "$IF" txqueuelen 1000
done
```

### Verify

```bash
ip -details link show can0
candump -t a can0
```

Expected: state `ERROR-ACTIVE`, bitrate `1000000`, dbitrate `5000000`, `fd on`. `candump` should show heartbeat frames from any motor with power.

> NOTE: If a motor sends frames immediately on power-on, you may see traffic before any cansend. That's a quick way to confirm physical wiring.

---

## Phase 3 — Map each CAN bus → arm/side

For each up interface, in **terminal A**:

```bash
candump -t a canX
```

In **terminal B** (enable motor 1):

```bash
cansend canX 001#FFFFFFFFFFFFFFFC
```

The bus whose `011` ID replies has motor #1 attached. Repeat per bus.

### Phase 3 result (2026-05-15)

`./scripts/probe_can_buses.sh` returned `[OK] Motors responding ... 011,012,013,014,015,016,017,018` on **all 4 buses** → bimanual setup: **4 arms × 8 motors = 32 motors**, all healthy. Factory motor IDs intact.

USB topology:

| USB port | lsusb Dev | PEAK adapter | CAN ifaces |
|---|---|---|---|
| `1-5`  | `0c72:0011` Dev 018 | A | can0 (CH1) + can1 (CH2) |
| `1-12` | `0c72:0011` Dev 010 | B | can2 (CH1) + can3 (CH2) |

Each PEAK PCAN-USB Pro FD is 2-channel; CH1/CH2 of one device map to two consecutive `canX` indices.

**Confirmed mapping (2026-05-15):**

| CAN if | PEAK CH | USB ID_PATH | Arm | Side |
|---|---|---|---|---|
| can0 | A-CH1 | `pci-0000:00:14.0-usb-0:5:1.0`  | **Leader**   | **R** |
| can1 | A-CH2 | `pci-0000:00:14.0-usb-0:5:1.0`  | **Leader**   | **L** |
| can2 | B-CH1 | `pci-0000:00:14.0-usb-0:12:1.0` | **Follower** | **R** |
| can3 | B-CH2 | `pci-0000:00:14.0-usb-0:12:1.0` | **Follower** | **L** |

= **Adapter A = Leader pair / Adapter B = Follower pair.**

### Why udev rename won't work cleanly here

Both PCAN-USB Pro FD adapters report identical udev attributes (`ID_VENDOR_ID=0c72 ID_MODEL_ID=0011 ID_SERIAL=PEAK-System_Technik_GmbH_PCAN-USB_Pro_FD`). They are **not** distinguishable by hardware serial. The only invariant is the **USB physical port path** (`usb-0:5` vs `usb-0:12`).

**Operational implication:** label the two laptop USB ports (or cables) so that Adapter A always lives in the same port. With that constraint, USB path → role mapping is deterministic.

### ⚠ Physical USB port pinning is REQUIRED

The role table above (`can0=Leader-R`, etc.) is valid **only while both PEAK adapters stay in the same physical USB ports** on the laptop. Concretely:

- Adapter **A** must stay in the USB port whose kernel path is `usb-0:5` (visible as `pci-0000:00:14.0-usb-0:5:1.0` in `udevadm info`).
- Adapter **B** must stay in the USB port whose kernel path is `usb-0:12`.

If the adapters are plugged into different ports (or swapped between each other), the USB path changes and `openarm_canup.sh` will either:
- emit `OPENARM_UNKNOWN1=...` / `OPENARM_UNKNOWN2=...` for any unrecognized port, or
- silently mis-label Leader vs Follower if A and B are swapped between the two known ports.

**Operational rule:** put a label on each laptop USB port (`A` / `B`) and on each adapter cable, and never plug an adapter into a different port. If a hardware change forces a new port, update the `PORT_ROLE_CH1` / `PORT_ROLE_CH2` tables in `scripts/openarm_canup.sh`. (Per OpenArm official setup procedure the CAN-bus ↔ arm assignment is fixed, so as long as official cabling is followed this stays stable.)

### Approach: runtime USB-path → role resolver

`scripts/openarm_canup.sh` does both jobs in one go:

1. Brings up every PCAN channel with CAN-FD 1M/5M
2. Reads each interface's parent USB path and writes `~/.config/openarm/can_map.env`:
   ```sh
   export OPENARM_LEADER_R=can0
   export OPENARM_LEADER_L=can1
   export OPENARM_FOLLOWER_R=can2
   export OPENARM_FOLLOWER_L=can3
   ```
3. Any subsequent script can `source` this file and use `$OPENARM_FOLLOWER_L` etc. regardless of which `canN` index the kernel picked.

If the user ever plugs into a different USB port, the resolver re-detects on next run.

---

## Phase 4 — Motor smoke test

Per <https://docs.openarm.dev/software/setup/configure-test/>:

```bash
# enable motor 0x01 (no sudo needed for cansend if the user is in any standard group; if it errors, prefix sudo)
cansend canX 001#FFFFFFFFFFFFFFFC   # enable
cansend canX 001#FFFFFFFFFFFFFFFD   # disable
```

- Response frame on ID `0x11` (`011`) = healthy.
- Damiao motor LED turning **green** = CAN link healthy.
- Repeat for IDs `002..008` on each arm bus.

---

## Phase 5 — Demo

Per <https://docs.openarm.dev/software/setup/run-demo/>:

```bash
# from openarm-can-1.2.x.tar.gz (download from GitHub releases of enactic/openarm-can)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target openarm-can-demo
build/openarm-can-demo
```

Default demo drives motors `0x01, 0x02, 0x07` (DM4310) on **can0 with CAN-FD**. Confirm motors are at safe zero before running.

---

## Phase 6 — Leader-Follower teleop

Pages: `/teleop/leader-follower/` → Setup → Unilateral → Bilateral.

(TODO — fetch exact CLI / repo once setup pages render. The `/teleop/leader-follower/` URLs returned 404 to WebFetch on 2026-05-15 — likely client-side routed; open in browser.)

---

## Session end state — 2026-05-15

- 4 アーム × 2 モード (unilateral / bilateral) 全 8 通りで動作確認済み
- 32 モータの zero 較正 + 4 グリッパの単独 zero 焼き直し完了
- USB EMI 問題は USB ケーブルを motor cable から物理的に離すことで解決
- 残課題:
  - Follower R の向きが Leader R と数度ずれている (再較正で対応予定)
  - **破損したグリッパは zero 焼き直しでは揃えられない** ← 部品交換待ち
  - Adapter A の USB ポートが 5→6 に移動した観察 (`openarm_canup.sh` の PORT_ROLE 表を要更新)
  - bimanual 同時 teleop は未試行
  - GitHub Issue 提出、`.claude/agents/openarm-engineer.md` カスタムエージェント化、`safe_launch_teleop.sh` wrapper、udev rule

## Power-cycle として最終手段

`cansend` 連打しても個別モータの green LED が残る場合 (error latch 状態):

1. アーム電源スイッチ OFF
2. 10秒待つ
3. ON
4. PC 側で `sudo openarm_canup.sh` で CAN 再構成

これでモータ内部状態が完全リセットされる。

## Troubleshooting reference

- `sudo modprobe can can_raw slcan` if interfaces not visible
- `dmesg | grep -i can` for USB-CAN enumeration issues
- `ip -details -statistics link show canX` for tx/rx/error counters
- If `BUS-OFF`: bring down, reconfigure bitrate, bring up — physical termination missing or bitrate mismatch on bus
- Bitrate must match what was burned into the motors via Damiao tool; OpenArm factory default is **CAN-FD 1M arbitration / 5M data**

---

## Reference — quick command cheatsheet

```bash
# show all CAN ifs with state
ip -brief link show type can

# enable motor i (hex)
cansend canX 00i#FFFFFFFFFFFFFFFC

# disable motor i
cansend canX 00i#FFFFFFFFFFFFFFFD

# trace traffic with timestamps
candump -tA canX,0:0,#FFFFFFFF

# busload (kbit/s)
canbusload canX@1000000

# bring up via helper
sudo openarm-can-configure-socketcan canX -fd -b 1000000 -d 5000000
```
