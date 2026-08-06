#!/usr/bin/env bash
# Fetch the Holochain conductor + lair-keystore sidecar binaries that get bundled
# into the desktop app via Tauri's `externalBin`.
#
# WHY THIS EXISTS: upstream already publishes prebuilt binaries for all five
# target triples, so Mycelix never compiles Holochain for any platform. The
# asset naming convention `<name>-<target-triple>` is *exactly* what Tauri's
# externalBin expects, so one declaration resolves per-platform with no
# renaming. This is the step that makes Windows/macOS support CI + certificates
# rather than a porting project.
#
# The binaries are ~66MB for linux-x86_64 alone and are gitignored. Run this
# before `cargo tauri build`.
#
# Versions are welded together: holochain 0.6.1 <-> lair-keystore 0.6.3
# <-> holochain_client 0.8.1. See memory/holochain_client_version_offset_rule.md
# and mycelix-workspace/Cargo.toml [workspace.dependencies].
set -euo pipefail

HC_VERSION="holochain-0.6.1"
BASE="https://github.com/holochain/holochain/releases/download/${HC_VERSION}"
DEST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src-tauri/binaries"

# Only the host triple is fetched by default. Pass a triple to override, e.g.
#   ./scripts/fetch-sidecars.sh aarch64-apple-darwin
# Upstream publishes: x86_64/aarch64-unknown-linux-gnu,
# x86_64/aarch64-apple-darwin, x86_64-pc-windows-msvc (.exe).
TRIPLE="${1:-x86_64-unknown-linux-gnu}"

# sha256, verified 2026-07-31. A mismatch means upstream re-cut the release or
# the download was tampered with -- do NOT "fix" it by updating the hash without
# understanding why it changed.
declare -A SHA256=(
  ["holochain-x86_64-unknown-linux-gnu"]="423f1111773c83c4c4f07e0bb338289d9bf0c5fa53dd31414b05b0dc8119ada7"
  ["lair-keystore-x86_64-unknown-linux-gnu"]="4d13042d70803d9556bad5f2bfcd01e7daf33ee51ced16cd8d2377a321cceed0"
)

mkdir -p "$DEST"
for name in holochain lair-keystore; do
  asset="${name}-${TRIPLE}"
  [[ "$TRIPLE" == *windows* ]] && asset="${asset}.exe"
  target="${DEST}/${asset}"

  if [[ -f "$target" ]] && [[ -n "${SHA256[$asset]:-}" ]] \
     && echo "${SHA256[$asset]}  ${target}" | sha256sum -c --status 2>/dev/null; then
    echo "  ${asset}: already present and verified"
    continue
  fi

  echo "  fetching ${asset} ..."
  curl -sSL --fail -o "$target" "${BASE}/${asset}"
  chmod +x "$target"

  if [[ -n "${SHA256[$asset]:-}" ]]; then
    echo "${SHA256[$asset]}  ${target}" | sha256sum -c --status \
      || { echo "  ERROR: sha256 mismatch for ${asset}" >&2; rm -f "$target"; exit 1; }
    echo "  ${asset}: verified"
  else
    echo "  ${asset}: WARNING no pinned sha256 for this triple -- recording:"
    sha256sum "$target"
  fi
done

echo "Sidecars ready in ${DEST}"
