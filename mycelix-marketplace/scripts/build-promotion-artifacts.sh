#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-}"
: "${HC_BIN:=hc}"
OUT="${2:-$ROOT/.promotion/$PROFILE}"

case "$PROFILE" in base|arbitration|settlement|network) ;; *) echo "usage: $0 {base|arbitration|settlement|network} [output-dir]" >&2; exit 2;; esac
if [[ -e "$OUT" ]] && find "$OUT" -mindepth 1 -print -quit | grep -q .; then
  echo "promotion output must be empty: $OUT" >&2; exit 2
fi
mkdir -p "$OUT/artifacts" "$OUT/stage"

"$ROOT/scripts/check-promotion-environment.sh"
python3 "$ROOT/scripts/check-promotion-artifacts.py"
SOURCE_REVISION="${MARKETPLACE_SOURCE_REVISION:-$(git -C "$ROOT" rev-parse HEAD)}"
TARGET="$ROOT/backend/target/wasm32-unknown-unknown/release"

cargo build --manifest-path "$ROOT/backend/Cargo.toml" --locked --release --target wasm32-unknown-unknown --workspace
python3 "$ROOT/scripts/stage-marketplace-dna.py" \
  --target-dir "$TARGET" --stage-dir "$OUT/stage/dna" --receipt "$OUT/zome-artifacts.json"
(
  cd "$OUT/stage/dna"
  "$HC_BIN" dna pack .
)
DNA_FOUND="$(find "$OUT/stage/dna" -maxdepth 1 -type f -name '*.dna' -print -quit)"
[[ -n "$DNA_FOUND" ]] || { echo "hc dna pack produced no .dna" >&2; exit 1; }
cp "$DNA_FOUND" "$OUT/artifacts/mycelix_marketplace.dna"

HAPP_STAGE="$OUT/stage/happ"
mkdir -p "$HAPP_STAGE"
if [[ "$PROFILE" == settlement ]]; then
  : "${FINANCE_DNA_PATH:?settlement build requires FINANCE_DNA_PATH}"
  FINANCE_DNA_PATH="$(realpath "$FINANCE_DNA_PATH")"
  [[ -f "$FINANCE_DNA_PATH" ]] || { echo "Finance DNA missing: $FINANCE_DNA_PATH" >&2; exit 2; }
  python3 "$ROOT/scripts/render-marketplace-deployment.py" \
    --marketplace-dna "$OUT/artifacts/mycelix_marketplace.dna" \
    --finance-dna "$FINANCE_DNA_PATH" --output "$HAPP_STAGE/happ.yaml"
else
  cat > "$HAPP_STAGE/happ.yaml" <<EOF
---
manifest_version: "0"
name: mycelix_marketplace
description: Evidence-bound $PROFILE promotion deployment.
roles:
  - name: marketplace
    provisioning:
      strategy: create
      deferred: false
    dna:
      path: "$(realpath "$OUT/artifacts/mycelix_marketplace.dna")"
      modifiers:
        network_seed: ~
        properties: ~
      clone_limit: 0
EOF
fi
(
  cd "$HAPP_STAGE"
  "$HC_BIN" app pack .
)
HAPP_FOUND="$(find "$HAPP_STAGE" -maxdepth 1 -type f -name '*.happ' -print -quit)"
[[ -n "$HAPP_FOUND" ]] || { echo "hc app pack produced no .happ" >&2; exit 1; }
cp "$HAPP_FOUND" "$OUT/artifacts/mycelix_marketplace.happ"

python3 "$ROOT/scripts/check-promotion-toolchain.py" --json > "$OUT/toolchain.json"
python3 - "$OUT" "$PROFILE" "$SOURCE_REVISION" "${FINANCE_DNA_PATH:-}" <<'PY'
import hashlib,json,sys
from pathlib import Path
out=Path(sys.argv[1]); profile=sys.argv[2]; revision=sys.argv[3]; finance=sys.argv[4]
def item(path):
 p=Path(path); return {"path":str(p.resolve()),"sha256":hashlib.sha256(p.read_bytes()).hexdigest(),"size":p.stat().st_size}
manifest={"schema_version":1,"profile":profile,"source_revision":revision,
 "artifacts":{"happ":item(out/'artifacts/mycelix_marketplace.happ'),"marketplace_dna":item(out/'artifacts/mycelix_marketplace.dna'),"finance_dna":item(finance) if finance else None},
 "zome_receipt_sha256":hashlib.sha256((out/'zome-artifacts.json').read_bytes()).hexdigest(),
 "toolchain_receipt_sha256":hashlib.sha256((out/'toolchain.json').read_bytes()).hexdigest()}
(out/'artifact-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
sha256sum "$OUT"/artifacts/* "$OUT"/*.json > "$OUT/SHA256SUMS"
echo "$OUT/artifact-manifest.json"
