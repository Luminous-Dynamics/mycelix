#!/usr/bin/env bash
# Runs the full live-browser proof, relaunching fresh throwaway conductors
# between attempts, up to N times.
#
# Why this exists: `hc sandbox generate`'s DHT database occasionally hits a
# genuine, reproducible "SQL logic error" on its very first app_info call
# after a fresh launch — found live via the Pulse browser proof, under this
# session's typically-heavy concurrent system load (12+ other Claude
# sessions sharing the box). Once it happens on a given conductor instance,
# that instance is permanently poisoned — retrying the SAME call against
# the SAME conductor fails identically every time, confirmed by direct
# repro. A brand-new conductor pair, however, usually works fine (multiple
# clean end-to-end runs were captured this same session), so the fix that
# actually works is retrying the whole launch, not the call.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-5}"
SETTLE_SECONDS="${SETTLE_SECONDS:-30}"

export HC_PATH="${HC_PATH:-/var/tmp/hc-0.6.1/bin/hc}"
export PATH="/var/tmp/hc-0.6.1/bin:$PATH"

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  echo "=== [run-with-retry] attempt $attempt/$MAX_ATTEMPTS ==="

  "$SCRIPT_DIR/stop-conductors.sh" >/dev/null 2>&1 || true
  pkill -9 -f "mycelix-pulse-live-proof" >/dev/null 2>&1 || true
  sleep 2
  rm -rf /tmp/mycelix-pulse-live-proof
  rm -f "$SCRIPT_DIR"/.hc "$SCRIPT_DIR"/.hc_live_0 "$SCRIPT_DIR"/.hc_live_1

  if ! "$SCRIPT_DIR/launch-conductors.sh"; then
    echo "=== [run-with-retry] attempt $attempt: conductor launch failed, retrying ==="
    continue
  fi

  echo "=== [run-with-retry] letting conductors settle for ${SETTLE_SECONDS}s ==="
  sleep "$SETTLE_SECONDS"

  if node "$SCRIPT_DIR/live-proof.mjs"; then
    echo "=== [run-with-retry] PASSED on attempt $attempt ==="
    exit 0
  fi

  echo "=== [run-with-retry] attempt $attempt failed ==="
done

echo "=== [run-with-retry] all $MAX_ATTEMPTS attempts failed ==="
exit 1
