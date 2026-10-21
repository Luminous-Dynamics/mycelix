#!/usr/bin/env bash
set -euo pipefail

# Static regression guard for the six identity-dependent Leptos app shells.
# This is intentionally not a replacement for browser tests or source review.
files=(
  mycelix-personal/apps/leptos/src/app.rs
  mycelix-finance/apps/leptos/src/app.rs
  mycelix-climate/apps/leptos/src/app.rs
  mycelix-workspace/mycelix-personal/apps/leptos/src/app.rs
  mycelix-workspace/mycelix-finance/apps/leptos/src/app.rs
  mycelix-workspace/mycelix-climate/apps/leptos/src/app.rs
)

for file in "${files[@]}"; do
  [[ -f "$file" ]] || { echo "Missing guarded app shell: $file" >&2; exit 1; }

  # Startup must inspect posture and gate app rendering on protected/unlocked state.
  rg -q 'fn AppInner\(\)' "$file" || { echo "$file: missing AppInner" >&2; exit 1; }
  rg -q 'inspect_local_identity_posture\(\)' "$file" || { echo "$file: missing posture inspection" >&2; exit 1; }
  rg -q 'LocalIdentityStoragePosture::ProtectedUnlockedInSession' "$file" || { echo "$file: missing protected/unlocked gate state" >&2; exit 1; }
  rg -q '<IdentityBootstrapGate app_name="(Personal|Finance|Climate)" posture=posture' "$file" || { echo "$file: missing shared bootstrap gate fallback" >&2; exit 1; }

  # The ready subtree must use fallible access before adding app contexts.
  ready_line="$(rg -n 'fn (Personal|Finance|Climate)ReadyApp\(posture: RwSignal<LocalIdentityStoragePosture>\)' "$file" | cut -d: -f1 | head -n1)"
  access_line="$(rg -n 'try_provide_local_identity\(\)\.is_err\(\)' "$file" | cut -d: -f1 | head -n1)"
  [[ -n "$ready_line" && -n "$access_line" ]] || { echo "$file: missing ready subtree or fallible identity access" >&2; exit 1; }
  (( access_line > ready_line )) || { echo "$file: identity access must be inside the ready subtree" >&2; exit 1; }
  context_line="$(awk -F: -v start="$ready_line" '$1 > start && /provide_theme_context\(|provide_thermodynamic_context\(|provide_consciousness_context\(|provide_personal_context\(|provide_finance_context\(|provide_climate_context\(/ { print $1; exit }' "$file")"
  [[ -n "$context_line" ]] || { echo "$file: could not locate app context setup after identity access" >&2; exit 1; }
  (( access_line < context_line )) || { echo "$file: context setup moved before fallible identity access" >&2; exit 1; }

  # App shells may not reintroduce imperative access that creates a key during render.
  if rg -n '(^|[^[:alnum:]_])provide_local_identity\s*\(\s*\)|(^|[^[:alnum:]_])ensure_keypair\s*\(\s*\)' "$file"; then
    echo "$file: legacy identity provider/key-generator call found in app shell" >&2
    exit 1
  fi
done

echo "PASS: all six source and mirror app shells gate identity-dependent setup and use fallible identity access."
