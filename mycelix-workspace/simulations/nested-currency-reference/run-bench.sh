#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
if [[ -n "$(git -C "$repo_root" status --porcelain)" ]]; then
  echo "Refusing to benchmark a dirty worktree; commit or discard changes first." >&2
  exit 2
fi

source_sha="$(git -C "$repo_root" rev-parse HEAD)"
profile_file="$repo_root/mycelix-core/docs/civilizational-os/federation/NESTED_CURRENCY_AND_CLEARING_PROFILE.md"
manifest="$repo_root/mycelix-workspace/simulations/nested-currency-reference/Cargo.toml"
profile_digest="$(sha256sum "$profile_file" | awk '{print $1}')"
rustc_version="$(rustc --version)"
rustc_commit="$(rustc --version --verbose | sed -n 's/^commit-hash: //p')"
cargo_version="$(cargo --version)"
if [[ -z "$rustc_commit" ]]; then
  echo "Unable to resolve rustc toolchain commit identity." >&2
  exit 2
fi

output="${NCF_OUTPUT_FILE:-${TMPDIR:-/tmp}/ncf-benchmark-${source_sha}.json}"
receipt="${NCF_RECEIPT_FILE:-${output}.receipt.txt}"
target_dir="${CARGO_TARGET_DIR:-${TMPDIR:-/tmp}/ncf-target-${source_sha}}"

NCF_SOURCE_SHA="$source_sha" \
NCF_PROFILE_DIGEST="$profile_digest" \
NCF_RUSTC_VERSION="$rustc_version" \
NCF_TOOLCHAIN_ID="rustc-commit:${rustc_commit}; ${cargo_version}" \
CARGO_TARGET_DIR="$target_dir" \
  cargo run --locked --manifest-path "$manifest" --release --bin ncf-bench -- "$@" > "$output"

output_digest="$(sha256sum "$output" | awk '{print $1}')"
{
  printf 'status=benchmark-completed-invariant-checked\n'
  printf 'source_commit=%s\n' "$source_sha"
  printf 'profile_digest_sha256=%s\n' "$profile_digest"
  printf 'rustc_version=%s\n' "$rustc_version"
  printf 'rustc_commit=%s\n' "$rustc_commit"
  printf 'cargo_version=%s\n' "$cargo_version"
  printf 'command='
  printf '%q ' cargo run --locked --manifest-path "$manifest" --release --bin ncf-bench -- "$@"
  printf '\n'
  printf 'benchmark_output=%s\n' "$output"
  printf 'benchmark_output_sha256=%s\n' "$output_digest"
} > "$receipt"

printf 'benchmark_output=%s\n' "$output"
printf 'benchmark_output_sha256=%s\n' "$output_digest"
printf 'evidence_receipt=%s\n' "$receipt"
