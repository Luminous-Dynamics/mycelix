#!/usr/bin/env bash
# Tears down the two throwaway conductors started by launch-conductors.sh.
set -uo pipefail

SANDBOX_ROOT="${SANDBOX_ROOT:-/tmp/mycelix-pulse-live-proof}"

for who in alice bob bootstrap relay; do
  pidfile="$SANDBOX_ROOT/$who.pid"
  if [[ -f "$pidfile" ]]; then
    pid="$(cat "$pidfile")"
    if kill -0 "$pid" 2>/dev/null; then
      echo "[stop-conductors] stopping $who (pid $pid)"
      kill -TERM "$pid" 2>/dev/null || true
      sleep 1
      kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
    fi
    rm -f "$pidfile"
  fi
done

echo "[stop-conductors] done. Sandbox data left at $SANDBOX_ROOT for inspection; delete manually if not needed."
