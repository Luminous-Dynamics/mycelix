#!/usr/bin/env bash
set -euo pipefail

# Preserve the complete historical source-invariant suite byte-for-byte while
# advancing its single stale statement settlement-domain assertion from v6 to
# the root-trust-aware v7 domain. New v7 trust assertions live in the dedicated
# checkpoint/trust invariant gate; no historical invariant is skipped here.
tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT

sed \
  -e "s/evidenceKind: 'settlement_execution_v6'/evidenceKind: 'settlement_execution_v7'/g" \
  -e 's/checkpoint-aware v6 evidence domain/root-trust-aware v7 evidence domain/g' \
  scripts/check-source-invariants-base.sh > "$tmp"

bash "$tmp"
