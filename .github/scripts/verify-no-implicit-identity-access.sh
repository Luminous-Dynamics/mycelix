#!/usr/bin/env bash
set -euo pipefail

# Application consumers must use typed/fallible local identity access.
# Shared-core compatibility wrappers exist temporarily but are non-minting
# and panic-on-error; app logic should not call them directly.
roots=(
  mycelix-personal/apps/leptos/src
  mycelix-finance/apps/leptos/src
  mycelix-climate/apps/leptos/src
  mycelix-workspace/mycelix-personal/apps/leptos/src
  mycelix-workspace/mycelix-finance/apps/leptos/src
  mycelix-workspace/mycelix-climate/apps/leptos/src
)

for root in "${roots[@]}"; do
  [[ -d "$root" ]] || { echo "Missing identity-consumer source root: $root" >&2; exit 1; }
done

if rg -n --glob '*.rs' '\b(local_did|ensure_keypair|sign_with_local_identity)\s*\(' "${roots[@]}"; then
  echo "Direct compatibility identity access is forbidden in application code; use try_local_did / try_ensure_keypair / try_sign_with_local_identity and handle the typed result." >&2
  exit 1
fi

echo "PASS: all six app source trees avoid direct panic-on-error identity accessors."
