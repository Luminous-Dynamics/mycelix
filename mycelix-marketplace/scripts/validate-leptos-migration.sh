#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LEPTOS="$ROOT/frontend-leptos"
BRIDGE_OUTPUT="$LEPTOS/apps/marketplace-web/public/holochain-bridge.js"

cleanup() {
  rm -f "$BRIDGE_OUTPUT"
}
trap cleanup EXIT

python3 "$ROOT/scripts/check-zome-contract.py"
python3 "$ROOT/scripts/check-leptos-client-contract.py"
python3 "$ROOT/scripts/check-leptos-vertical-slice.py"
python3 "$ROOT/scripts/check-leptos-lifecycle-slice.py"
python3 "$ROOT/scripts/check-arbitration-integrity.py"
python3 "$ROOT/scripts/check-finance-reputation-integrity.py"
python3 "$ROOT/scripts/check-current-claims.py"
python3 "$ROOT/scripts/check-deployment-promotion.py"
python3 "$ROOT/scripts/check-live-evidence-contract.py"
python3 "$ROOT/scripts/test-live-evidence-verifier.py"
python3 "$ROOT/scripts/check-promotion-artifacts.py"
python3 "$ROOT/scripts/check-disposable-promotion.py"
python3 "$ROOT/scripts/check-signed-promotion.py"
python3 "$ROOT/scripts/check-transaction-conflict-policy.py"
python3 "$ROOT/scripts/check-transaction-conflict-resolution.py"
python3 "$ROOT/scripts/check-network-promotion.py"
python3 "$ROOT/scripts/test-network-config.py"
python3 "$ROOT/scripts/test-network-evidence-verifier.py"
python3 "$ROOT/scripts/test-promotion-bundle.py"

npm --prefix "$LEPTOS/bridge" ci --ignore-scripts
npm --prefix "$LEPTOS/bridge" run typecheck
npm --prefix "$LEPTOS/bridge" run test:wire
npm --prefix "$LEPTOS/bridge" run test:runtime
npm --prefix "$LEPTOS/bridge" run check:conductor-lifecycle
npm --prefix "$LEPTOS/bridge" run check:conductor-arbitration
npm --prefix "$LEPTOS/bridge" run check:conductor-settlement
npm --prefix "$LEPTOS/bridge" run check:disposable-setup
npm --prefix "$LEPTOS/bridge" run check:network-setup
npm --prefix "$LEPTOS/bridge" run check:conductor-network
npm --prefix "$LEPTOS/bridge" run build
node --check "$BRIDGE_OUTPUT"

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is unavailable; Rust build validation was not run" >&2
  exit 2
fi

(
  cd "$LEPTOS"
  cargo fmt --all --check
  cargo test -p marketplace-domain
  cargo test -p marketplace-client --features dev-fixtures
  cargo check -p marketplace-web --target wasm32-unknown-unknown
  cargo check -p marketplace-web --target wasm32-unknown-unknown \
    --no-default-features --features dev-fixtures
)
