#!/usr/bin/env bash
set -euo pipefail

# Finance source-of-truth guard.
#
# The monorepo copy is canonical:
#   mycelix-workspace/mycelix-finance/
# The public standalone projection is:
#   mycelix-finance/
#
# The two trees intentionally have different standalone-specific files.
# Every overlapping tracked file must be byte-identical after applying only
# the single deterministic Finance layout rewrite for flake.nix:
#   ../../nix/modules/holochain-base.nix -> ../nix/modules/holochain-base.nix

canonical_root="mycelix-workspace/mycelix-finance"
projection_root="mycelix-finance"

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

  common_count=$((common_count + 1))
  canonical_blob="$(git rev-parse "HEAD:$canonical_root/$rel")"
  projection_blob="$(git rev-parse "HEAD:$projection_root/$rel")"

  if [ "$rel" = "flake.nix" ]; then
    # The projection may change only the one relative path required by the
    # standalone directory layout. Compare its exact blob against the
    # canonical file after that deterministic transformation.
    expected_blob="$(
      git show "HEAD:$canonical_root/$rel" |
        sed 's#\.\./\.\./nix/modules/holochain-base\.nix#../nix/modules/holochain-base.nix#g' |
        git hash-object --stdin
    )"

    if [ "$projection_blob" != "$expected_blob" ]; then
      printf 'SOURCE_PARITY_MISMATCH %s canonical=%s expected_projection=%s actual_projection=%s\n'         "$rel" "$canonical_blob" "$expected_blob" "$projection_blob"
      mismatch_count=$((mismatch_count + 1))
    fi
  elif [ "$canonical_blob" != "$projection_blob" ]; then
    printf 'SOURCE_PARITY_MISMATCH %s canonical=%s projection=%s\n'       "$rel" "$canonical_blob" "$projection_blob"
    mismatch_count=$((mismatch_count + 1))
  fi
done < <(comm -12 <(printf '%s\n' "${canonical_paths[@]}") <(printf '%s\n' "${projection_paths[@]}"))

printf 'FINANCE_SOURCE_PARITY common_paths=%d mismatches=%d\n' "$common_count" "$mismatch_count"

test "$mismatch_count" -eq 0
