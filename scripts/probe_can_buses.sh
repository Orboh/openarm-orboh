#!/usr/bin/env bash
# Probe each CAN-FD bus for OpenArm motors (IDs 0x01-0x08).
# For each interface listed, sends an enable frame to every motor ID and
# captures any responses on IDs 0x11-0x18. Then sends disable to be safe.
#
# Usage:  ./probe_can_buses.sh                  # probes can0..can3
#         ./probe_can_buses.sh can0 can2        # probes only listed ifaces
#
# Safe-by-design: each motor is enabled for ~150 ms then disabled. No motion
# commands are sent, only enable/disable. Motors will hold their current
# position with default low gains.

set -u

IFS_LIST=("${@:-can0 can1 can2 can3}")

probe_one() {
  local IF="$1"
  echo "================ ${IF} ================"
  if ! ip -brief link show "$IF" 2>/dev/null | grep -q UP; then
    echo "[skip] $IF is not UP"
    return
  fi

  local LOG
  LOG=$(mktemp)
  # Listen for ~1.5s while we send enables
  timeout 1.6 candump -t a "$IF" > "$LOG" 2>&1 &
  local DUMP_PID=$!
  sleep 0.2

  for ID in 001 002 003 004 005 006 007 008; do
    cansend "$IF" "${ID}#FFFFFFFFFFFFFFFC" 2>/dev/null
    sleep 0.05
  done
  wait "$DUMP_PID" 2>/dev/null

  # Immediately disable everything we just enabled
  for ID in 001 002 003 004 005 006 007 008; do
    cansend "$IF" "${ID}#FFFFFFFFFFFFFFFD" 2>/dev/null
    sleep 0.02
  done

  # Summarize: which response IDs (0x11..0x18) showed up
  local FOUND
  FOUND=$(grep -oE ' 01[1-8] ' "$LOG" | sort -u | tr -d ' ' | paste -sd, -)
  if [[ -n "$FOUND" ]]; then
    echo "[OK]  Motors responding on $IF: $FOUND"
  else
    echo "[--]  No motor responses on $IF"
  fi
  echo "--- raw frames (first 20) ---"
  head -n 20 "$LOG"
  echo
  rm -f "$LOG"
}

for IF in ${IFS_LIST[@]}; do
  probe_one "$IF"
done
