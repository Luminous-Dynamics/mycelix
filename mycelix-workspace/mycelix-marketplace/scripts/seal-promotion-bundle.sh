#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-}"
ARTIFACT_ROOT="${2:-$ROOT/.promotion/$PROFILE}"
OUTPUT="${3:-$ARTIFACT_ROOT/mycelix-promotion-$PROFILE.tar.gz}"
: "${MARKETPLACE_RELEASE_SIGNING_KEY:?set MARKETPLACE_RELEASE_SIGNING_KEY to an Ed25519 PEM private key}"
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/mycelix-seal.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
python3 "$ROOT/scripts/seal-promotion-bundle.py" --profile "$PROFILE" --artifact-root "$ARTIFACT_ROOT" --private-key "$MARKETPLACE_RELEASE_SIGNING_KEY" --output-dir "$STAGE" >/dev/null
EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "$ROOT" show -s --format=%ct HEAD)}"
mkdir -p "$(dirname "$OUTPUT")"
tar --sort=name --mtime="@$EPOCH" --owner=0 --group=0 --numeric-owner -C "$STAGE" -cf - mycelix-promotion-v1 | gzip -n > "$OUTPUT"
"$ROOT/scripts/verify-promotion-bundle.sh" "$OUTPUT"
echo "$OUTPUT"
