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
  canonical_entry="$(git ls-tree HEAD -- "$canonical_root/$rel")"
  projection_entry="$(git ls-tree HEAD -- "$projection_root/$rel")"

  test -n "$canonical_entry"
  test -n "$projection_entry"

  # Bind the complete Git tree entry, not just blob contents. This prevents
  # mode-only/type-only substitutions from escaping provenance checks.
  canonical_object="${canonical_entry%%$'\t'*}"
  projection_object="${projection_entry%%$'\t'*}"

  if [ "$rel" = "flake.nix" ]; then
    canonical_mode_type="$(printf '%s\n' "$canonical_object" | awk '{print $1, $2}')"
    projection_mode_type="$(printf '%s\n' "$projection_object" | awk '{print $1, $2}')"

    if [ "$canonical_mode_type" != "$projection_mode_type" ]; then
      printf 'SOURCE_PARITY_MISMATCH %s tree_entry canonical=%s projection=%s\n'         "$rel" "$canonical_mode_type" "$projection_mode_type"
      mismatch_count=$((mismatch_count + 1))
      continue
    fi

    # The projection may change only the one relative path required by the
    # standalone directory layout. Hash the transformed canonical file as a
    # raw byte stream so trailing newlines remain part of the identity.
    expected_blob="$(
      git show "HEAD:$canonical_root/$rel" |
        sed 's#\.\./\.\./nix/modules/holochain-base\.nix#../nix/modules/holochain-base.nix#g' |
        git hash-object --stdin
    )"
    projection_blob="$(printf '%s\n' "$projection_object" | awk '{print $3}')"

    if [ "$projection_blob" != "$expected_blob" ]; then
      printf 'SOURCE_PARITY_MISMATCH %s expected_projection_blob=%s actual_projection_blob=%s\n'         "$rel" "$expected_blob" "$projection_blob"
      mismatch_count=$((mismatch_count + 1))
    fi
  elif [ "$canonical_object" != "$projection_object" ]; then
    printf 'SOURCE_PARITY_MISMATCH %s tree_entry canonical=%s projection=%s\n'       "$rel" "$canonical_object" "$projection_object"
    mismatch_count=$((mismatch_count + 1))
  fi
done < <(comm -12 <(printf '%s\n' "${canonical_paths[@]}") <(printf '%s\n' "${projection_paths[@]}"))

printf 'FINANCE_SOURCE_PARITY common_paths=%d mismatches=%d\n' "$common_count" "$mismatch_count"

test "$mismatch_count" -eq 0
