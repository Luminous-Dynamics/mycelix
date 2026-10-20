#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRISM="$ROOT/mycelix-prism"

fail=0

check_absent() {
  local pattern="$1"
  local description="$2"
  if rg -n --glob '*.rs' --glob '!legacy-archive/**' --glob '!prism-net/src/**' "$pattern" "$PRISM"; then
    echo "network-authority guard: forbidden $description"
    fail=1
  fi
}

check_absent 'reqwest::get\s*\(' 'raw reqwest::get call'
check_absent 'redirect\s*\(\s*reqwest::redirect::Policy::(limited|default)' 'automatic redirect policy'

if rg -n 'reqwest::Client::builder\s*\(' "$PRISM/prism-tauri/src"; then
  echo "network-authority guard: Tauri must use prism-net::SafeFetchClient"
  fail=1
fi

for required in   "$PRISM/prism-net/src/lib.rs"   "$PRISM/prism-proxy/src/main.rs"   "$PRISM/prism-serve/src/main.rs"   "$PRISM/prism-tauri/src/main.rs"
do
  if [[ ! -f "$required" ]]; then
    echo "network-authority guard: missing expected authority surface: $required"
    fail=1
  fi
done

if ! rg -q 'pub struct SafeFetchClient' "$PRISM/prism-net/src/lib.rs"; then
  echo "network-authority guard: SafeFetchClient definition missing"
  fail=1
fi

if ! rg -q 'SafeFetchClient' "$PRISM/prism-proxy/src/main.rs"; then
  echo "network-authority guard: prism-proxy is not using SafeFetchClient"
  fail=1
fi

if ! rg -q 'SafeFetchClient' "$PRISM/prism-serve/src/main.rs"; then
  echo "network-authority guard: prism-serve is not using SafeFetchClient"
  fail=1
fi

if ! rg -q 'SafeFetchClient' "$PRISM/prism-tauri/src/main.rs"; then
  echo "network-authority guard: prism-tauri is not using SafeFetchClient"
  fail=1
fi

if (( fail != 0 )); then
  exit 1
fi

echo "network-authority guard: PASS (static authority shape only)"
