#!/usr/bin/env bash
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# Run a command with mycelix-unified (the 10-cluster cross-cluster test rig)
# installed + enabled on the shared conductor, then uninstall it again
# afterward — success or failure.
#
# Why: mycelix-unified's own manifest names it "mycelix-unified-testable" —
# it's a test rig, not a 24/7 resident. Verified 2026-07-08: merely having it
# *installed* (even disabled, never enabled) left the shared conductor at
# ~24GB RSS, because Holochain never releases a conductor process's memory
# for a disabled/uninstalled app at runtime — the only way to actually get
# the memory back is to never have installed it in the first place, or
# restart the conductor after uninstalling it. This script does the latter:
# install fresh, run your test, uninstall again so the next conductor
# restart (or a `systemctl restart mail-conductor`) starts clean.
#
# Usage:
#   ./scripts/with-unified-happ.sh -- <your test command...>
#
# Example:
#   ./scripts/with-unified-happ.sh -- cargo test --test cross_cluster_dispatch
#
# Note: this only removes the DB-level install record. The *current*
# conductor process's memory will NOT shrink until it's restarted
# (`sudo systemctl restart mail-conductor`) — this script does not restart
# it for you, since that briefly drops mycelix_mail's live connections too.

set -euo pipefail

ADMIN_PORT="${ADMIN_PORT:-33800}"
UNIFIED_HAPP_PATH="${UNIFIED_HAPP_PATH:-/srv/luminous-dynamics/mycelix-workspace/happs/mycelix-unified.happ}"
HC="${HC:-$HOME/.cargo/bin/hc}"

if [ "${1:-}" != "--" ]; then
  echo "Usage: $0 -- <command to run with mycelix-unified enabled>" >&2
  exit 1
fi
shift

if [ ! -f "$UNIFIED_HAPP_PATH" ]; then
  echo "ERROR: $UNIFIED_HAPP_PATH not found. Pack it first:" >&2
  echo "  hc app pack mycelix-workspace/happs/ -o $UNIFIED_HAPP_PATH" >&2
  exit 1
fi

cleanup() {
  echo "[with-unified-happ] Cleaning up: disabling + uninstalling mycelix-unified..."
  "$HC" sandbox call --running="$ADMIN_PORT" disable-app mycelix-unified 2>/dev/null || true
  "$HC" sandbox call --running="$ADMIN_PORT" uninstall-app mycelix-unified 2>/dev/null || true
  echo "[with-unified-happ] Done. Memory will not be released until the conductor is restarted:"
  echo "  sudo systemctl restart mail-conductor"
}
trap cleanup EXIT

echo "[with-unified-happ] Installing + enabling mycelix-unified on port $ADMIN_PORT..."
"$HC" sandbox call --running="$ADMIN_PORT" install-app "$UNIFIED_HAPP_PATH" --app-id mycelix-unified 2>/dev/null || true
"$HC" sandbox call --running="$ADMIN_PORT" enable-app mycelix-unified

echo "[with-unified-happ] Running: $*"
"$@"
