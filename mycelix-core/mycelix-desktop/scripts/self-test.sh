#!/usr/bin/env bash
# End-to-end check that the desktop shell can bring up a real Holochain conductor.
#
# WHY A SCRIPT: the interesting logic is reachable only via `mycelix-desktop
# --self-test`, and that binary resolves its conductor sidecar RELATIVE TO
# ITSELF (Tauri installs externalBin next to the executable). So a bare
# `cargo run` does not exercise the real path -- the binary has to sit beside
# the sidecars, exactly as the installed .deb arranges them. This assembles
# that layout and runs it, so the test means the same thing locally and in CI.
#
# Usage:  ./scripts/self-test.sh            # build + assemble + run
#         ./scripts/self-test.sh --no-build # reuse an existing release binary
#
# Exit 0 = conductor came up, admin websocket spoke MessagePack, list_apps
# round-tripped, and an app interface attached on an OS-assigned port.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [[ "${1:-}" != "--no-build" ]]; then
  echo "==> fetching sidecars (sha256-pinned, skipped if present)"
  ./scripts/fetch-sidecars.sh

  echo "==> building release binary"
  # Plain cargo, not `cargo tauri build`: we need the executable, not a bundle,
  # and skipping the bundler keeps this usable on hosts without one.
  (cd src-tauri && cargo build --release)
fi

TARGET_DIR="${CARGO_TARGET_DIR:-$HERE/src-tauri/target}"
BIN="$TARGET_DIR/release/mycelix-desktop"
[[ -x "$BIN" ]] || { echo "ERROR: no release binary at $BIN" >&2; exit 1; }

# Assemble the install layout the .deb produces: app + sidecars in one directory.
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
cp "$BIN" "$STAGE/"
for s in holochain lair-keystore; do
  src="src-tauri/binaries/${s}-x86_64-unknown-linux-gnu"
  [[ -f "$src" ]] || { echo "ERROR: missing sidecar $src (run scripts/fetch-sidecars.sh)" >&2; exit 1; }
  cp "$src" "$STAGE/$s"
done
chmod +x "$STAGE"/*

echo "==> running self-test from a faithful install layout"
"$STAGE/mycelix-desktop" --self-test
