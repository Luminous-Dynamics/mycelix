#!/usr/bin/env bash
set -euo pipefail

# Finance source-of-truth guard.
#
# The monorepo copy is canonical:
#   mycelix-workspace/mycelix-finance/
# The public standalone projection is:
#   mycelix-finance/
#
# The two trees intentionally have different standalone-specific files, and
# their flake.nix files differ because the layouts require different relative
# paths. Every other overlapping tracked file must be byte-identical.

canonical_root="mycelix-workspace/mycelix-finance"
projection_root="mycelix-finance"
intentionally_different="flake.nix"

test -d "$canonical_root"
test -d "$projection_root"

mapfile -t canonical_paths < <(
  git ls-tree -r --name-only HEAD -- "$canonical_root/" |
    sed "s#^$canonical_root/##" |
    sort
)
mapfile -t projection_paths < <(
  git ls-tree -r --name-only HEAD -- "$projection_root/" |
    sed "s#^$projection_root/##" |
    sort
)

common_count=0
mismatch_count=0

while IFS= read -r rel; do
  [ -n "$rel" ] || continue

  # Layout-specific flake files are intentionally different and are documented
  # as such in the source-of-truth record.
  if [ "$rel" = "$intentionally_different" ]; then
    continue
  fi

  common_count=$((common_count + 1))
  canonical_blob="$(git rev-parse "HEAD:$canonical_root/$rel")"
  projection_blob="$(git rev-parse "HEAD:$projection_root/$rel")"

  if [ "$canonical_blob" != "$projection_blob" ]; then
    printf 'SOURCE_PARITY_MISMATCH %s canonical=%s projection=%s\n' \
      "$rel" "$canonical_blob" "$projection_blob"
    mismatch_count=$((mismatch_count + 1))
  fi
done < <(comm -12 <(printf '%s\n' "${canonical_paths[@]}") <(printf '%s\n' "${projection_paths[@]}"))

printf 'FINANCE_SOURCE_PARITY common_paths=%d mismatches=%d\n' "$common_count" "$mismatch_count"

test "$mismatch_count" -eq 0
