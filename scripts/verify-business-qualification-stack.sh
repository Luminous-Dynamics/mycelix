#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

mapfile -t manifests < <(
  find crates -mindepth 2 -maxdepth 2 -type f -path 'crates/mycelix-business-*/Cargo.toml' -print | sort
)

if [[ ${#manifests[@]} -eq 0 ]]; then
  echo "no mycelix-business crate manifests found" >&2
  exit 1
fi

printf 'Business qualification manifests (%d):\n' "${#manifests[@]}"
printf '  %s\n' "${manifests[@]}"

for manifest in "${manifests[@]}"; do
  crate_dir="$(dirname "$manifest")"
  echo "::group::metadata $crate_dir"
  cargo metadata --manifest-path "$manifest" --no-deps --format-version 1 >/dev/null
  echo "::endgroup::"

done

for manifest in "${manifests[@]}"; do
  crate_dir="$(dirname "$manifest")"
  echo "::group::fmt $crate_dir"
  cargo fmt --manifest-path "$manifest" -- --check
  echo "::endgroup::"

done

for manifest in "${manifests[@]}"; do
  crate_dir="$(dirname "$manifest")"
  echo "::group::clippy $crate_dir"
  cargo clippy --manifest-path "$manifest" --all-targets -- -D warnings
  echo "::endgroup::"

done

for manifest in "${manifests[@]}"; do
  crate_dir="$(dirname "$manifest")"
  echo "::group::test $crate_dir"
  cargo test --manifest-path "$manifest"
  echo "::endgroup::"

done
