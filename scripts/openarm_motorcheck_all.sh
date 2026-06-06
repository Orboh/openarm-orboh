#!/usr/bin/env bash
# Run the OFFICIAL openarm-can-motor-check tool against all 32 motors.
# Verbatim per https://docs.openarm.dev/software/setup/motor-config Step 3.
# No motion is commanded. Pure parameter query / response check.
#
# Output: per-arm summary of which motors responded with parseable params.
# Even if Queried Control Mode returns -1 (firmware param-query mismatch we
# already observed), the "got a response at all" check is the real success
# criterion at this stage.

set -u

ARM_BUSES=(can0 can1 can2 can3)
ARM_LABELS=("can0 (was-Leader-R-now-Follower-R?)" \
            "can1 (was-Leader-L-now-Follower-L?)" \
            "can2 (was-Follower-R-now-Leader-R)"  \
            "can3 (was-Follower-L-now-Leader-L)")

for idx in "${!ARM_BUSES[@]}"; do
  IF="${ARM_BUSES[$idx]}"
  LABEL="${ARM_LABELS[$idx]}"
  echo
  echo "============================================================"
  echo " ${LABEL}"
  echo "============================================================"
  for SEND in 1 2 3 4 5 6 7 8; do
    RECV=$((SEND + 16))   # 0x10 offset → master ID
    echo
    echo "--- motor send=${SEND}  recv=${RECV}  iface=${IF} ---"
    openarm-can-motor-check "$SEND" "$RECV" "$IF" -fd 2>&1 \
      | sed -n '1,/^Reading motor parameters/p;/^=== Motor Parameters/,$p' \
      | head -20
  done
done
echo
echo "============================================================"
echo " DONE"
echo "============================================================"
