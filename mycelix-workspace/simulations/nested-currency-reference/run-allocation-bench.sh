#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(git rev-parse --show-toplevel)"
if [[ -n "$(git -C "$repo_root" status --porcelain)" ]]; then
  echo "refusing to benchmark a dirty checkout; commit or discard changes first" >&2
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
  echo "unable to resolve rustc toolchain commit identity" >&2
  exit 2
fi

output="${NCF_ALLOCATION_OUTPUT_FILE:-${TMPDIR:-/tmp}/ncf-allocation-benchmark-${source_sha}.json}"
receipt="${NCF_ALLOCATION_RECEIPT_FILE:-${output}.receipt.txt}"
target_dir="${CARGO_TARGET_DIR:-${TMPDIR:-/tmp}/ncf-allocation-target-${source_sha}}"

NCF_SOURCE_SHA="$source_sha" \
NCF_PROFILE_DIGEST="$profile_digest" \
NCF_RUSTC_VERSION="$rustc_version" \
NCF_TOOLCHAIN_ID="rustc-commit:${rustc_commit}; ${cargo_version}" \
CARGO_TARGET_DIR="$target_dir" \
  cargo run --locked --manifest-path "$manifest" --release --bin ncf-allocation-bench -- "$@" >"$output"

if ! grep -q '"schema":"mycelix.ncf-allocation-benchmark"' "$output"; then
  echo "benchmark did not emit the expected JSON schema" >&2
  exit 1
fi
if ! grep -q '"status":"completed_reference_simulation"' "$output"; then
  echo "benchmark completion marker missing" >&2
  exit 1
fi
report_count="$(grep -o '"policy":' "$output" | wc -l | tr -d ' ')"
if [[ "$report_count" != "12" ]]; then
  echo "expected twelve policy/mode reports, found $report_count" >&2
  exit 1
fi
safety_report_count="$(grep -o '"safety_violations":' "$output" | wc -l | tr -d ' ')"
if [[ "$safety_report_count" != "12" ]]; then
  echo "expected twelve safety-accounting fields, found $safety_report_count" >&2
  exit 1
fi
violation_count="$(grep -o '"safety_violations":[1-9][0-9]*' "$output" | wc -l | tr -d ' ')"
if [[ "$violation_count" != "0" ]]; then
  echo "one or more reports indicated safety accounting violations" >&2
  exit 1
fi
worst_shard_count="$(grep -o '"worst_shard_demand_served_ppm":' "$output" | wc -l | tr -d ' ')"
if [[ "$worst_shard_count" != "12" ]]; then
  echo "expected twelve worst-shard service fields, found $worst_shard_count" >&2
  exit 1
fi
shard_results_count="$(grep -o '"shard_results":\[' "$output" | wc -l | tr -d ' ')"
if [[ "$shard_results_count" != "12" ]]; then
  echo "expected twelve per-shard result arrays, found $shard_results_count" >&2
  exit 1
fi

output_digest="$(sha256sum "$output" | awk '{print $1}')"
binary="$target_dir/release/ncf-allocation-bench"
if [[ ! -s "$binary" ]]; then
  echo "allocation benchmark binary missing at expected release path" >&2
  exit 2
fi
binary_digest="$(sha256sum "$binary" | awk '{print $1}')"
{
  printf 'status=allocation-reference-simulation-completed\n'
  printf 'source_commit=%s\n' "$source_sha"
  printf 'profile_digest_sha256=%s\n' "$profile_digest"
  printf 'rustc_version=%s\n' "$rustc_version"
  printf 'rustc_commit=%s\n' "$rustc_commit"
  printf 'cargo_version=%s\n' "$cargo_version"
  printf 'command='
  printf '%q ' cargo run --locked --manifest-path "$manifest" --release --bin ncf-allocation-bench -- "$@"
  printf '\n'
  printf 'expected_policy_mode_reports=12\n'
  printf 'observed_policy_mode_reports=%s\n' "$report_count"
  printf 'safety_accounting_reports=%s\n' "$safety_report_count"
  printf 'safety_violation_reports=%s\n' "$violation_count"
  printf 'worst_shard_service_reports=%s\n' "$worst_shard_count"
  printf 'per_shard_result_arrays=%s\n' "$shard_results_count"
  printf 'benchmark_binary=%s\n' "$binary"
  printf 'benchmark_binary_sha256=%s\n' "$binary_digest"
  printf 'benchmark_output=%s\n' "$output"
  printf 'benchmark_output_sha256=%s\n' "$output_digest"
} >"$receipt"
receipt_digest="$(sha256sum "$receipt" | awk '{print $1}')"

printf 'benchmark_output=%s\n' "$output"
printf 'benchmark_output_sha256=%s\n' "$output_digest"
printf 'benchmark_binary_sha256=%s\n' "$binary_digest"
printf 'evidence_receipt=%s\n' "$receipt"
printf 'evidence_receipt_sha256=%s\n' "$receipt_digest"
