#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE="$ROOT/frontend-leptos/bridge"
PROFILE="${1:-}"

case "$PROFILE" in
  base|settlement|arbitration) ;;
  *) echo "usage: $0 {base|settlement|arbitration}" >&2; exit 2 ;;
esac

: "${MARKETPLACE_HAPP_PATH:?set MARKETPLACE_HAPP_PATH to the packed hApp}"
: "${MARKETPLACE_DNA_PATH:?set MARKETPLACE_DNA_PATH to the exact Marketplace DNA}"
: "${MARKETPLACE_EVIDENCE_DIR:?set MARKETPLACE_EVIDENCE_DIR to an empty output directory}"

if [[ -e "$MARKETPLACE_EVIDENCE_DIR" ]] && find "$MARKETPLACE_EVIDENCE_DIR" -mindepth 1 -print -quit | grep -q .; then
  echo "evidence directory must be empty: $MARKETPLACE_EVIDENCE_DIR" >&2
  exit 2
fi
mkdir -p "$MARKETPLACE_EVIDENCE_DIR"

export MARKETPLACE_SOURCE_REVISION="${MARKETPLACE_SOURCE_REVISION:-$(git -C "$ROOT" rev-parse HEAD)}"
export MARKETPLACE_HAPP_SHA256="$(sha256sum "$MARKETPLACE_HAPP_PATH" | awk '{print $1}')"
export MARKETPLACE_DNA_SHA256="$(sha256sum "$MARKETPLACE_DNA_PATH" | awk '{print $1}')"
export MARKETPLACE_CLIENT_VERSION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["dependencies"]["@holochain/client"])' "$BRIDGE/package.json")"

verify_args=(
  "$MARKETPLACE_EVIDENCE_DIR"
  --profile "$PROFILE"
  --source-revision "$MARKETPLACE_SOURCE_REVISION"
  --happ "$MARKETPLACE_HAPP_PATH"
  --marketplace-dna "$MARKETPLACE_DNA_PATH"
  --output "$MARKETPLACE_EVIDENCE_DIR/promotion.json"
)

npm --prefix "$BRIDGE" ci --ignore-scripts
npm --prefix "$BRIDGE" run test:conductor-lifecycle

if [[ "$PROFILE" == settlement ]]; then
  : "${FINANCE_DNA_PATH:?settlement promotion requires FINANCE_DNA_PATH}"
  export FINANCE_DNA_SHA256="$(sha256sum "$FINANCE_DNA_PATH" | awk '{print $1}')"
  npm --prefix "$BRIDGE" run test:conductor-settlement
  verify_args+=(--finance-dna "$FINANCE_DNA_PATH")
elif [[ "$PROFILE" == arbitration ]]; then
  npm --prefix "$BRIDGE" run test:conductor-arbitration
fi

python3 "$ROOT/scripts/verify-live-evidence.py" "${verify_args[@]}"
sha256sum "$MARKETPLACE_EVIDENCE_DIR"/*.json > "$MARKETPLACE_EVIDENCE_DIR/SHA256SUMS"
echo "promotion evidence written to $MARKETPLACE_EVIDENCE_DIR"
