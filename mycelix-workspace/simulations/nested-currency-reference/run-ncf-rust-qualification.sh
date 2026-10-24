#!/usr/bin/env bash
set -Eeuo pipefail

# Exact-head qualification for the standalone NCF Rust reference crate.
# All checks run even after an earlier check fails; the final exit code is
# nonzero if format, test, or clippy fails. Logs and receipts stay outside git.

repo_root="$(git rev-parse --show-toplevel)"
manifest="$repo_root/mycelix-workspace/simulations/nested-currency-reference/Cargo.toml"
source_dir="$(dirname "$manifest")"

if [[ -n "$(git -C "$repo_root" status --porcelain)" ]]; then
  echo "refusing to qualify a dirty checkout; commit or discard changes first" >&2
  exit 2
fi
[[ -f "$manifest" ]] || { echo "missing manifest: $manifest" >&2; exit 2; }
[[ -f "$source_dir/Cargo.lock" ]] || { echo "missing lockfile: $source_dir/Cargo.lock" >&2; exit 2; }

source_commit="$(git -C "$repo_root" rev-parse HEAD)"
source_tree="$(git -C "$repo_root" rev-parse 'HEAD^{tree}')"
out_dir="${NCF_RUST_OUTPUT_DIR:-${TMPDIR:-/tmp}/ncf-rust-qualification-$source_commit}"
mkdir -p "$out_dir"

fmt_log="$out_dir/fmt.log"
test_log="$out_dir/test.log"
clippy_log="$out_dir/clippy.log"
toolchain_log="$out_dir/toolchain.txt"
receipt="$out_dir/receipt.json"

rustc --version --verbose >"$toolchain_log"
printf '\n%s\n' "$(cargo --version)" >>"$toolchain_log"

manifest_sha="$(sha256sum "$manifest" | awk '{print $1}')"
lock_sha="$(sha256sum "$source_dir/Cargo.lock" | awk '{print $1}')"
lib_sha="$(sha256sum "$source_dir/src/lib.rs" | awk '{print $1}')"
reservation_sha="$(sha256sum "$source_dir/src/reservation.rs" | awk '{print $1}')"
fencing_sha="$(sha256sum "$source_dir/src/fencing.rs" | awk '{print $1}')"
toolchain_sha="$(sha256sum "$toolchain_log" | awk '{print $1}')"

run_check() {
  local label="$1"
  local logfile="$2"
  shift 2
  local status=0

  echo "Running $label: $*" >&2
  set +e
  "$@" >"$logfile" 2>&1
  status=$?
  set -e

  if [[ "$status" -eq 0 ]]; then
    echo "$label: PASS" >&2
  else
    echo "$label: FAIL (exit $status)" >&2
    cat "$logfile" >&2
  fi
  printf '%s\n' "$status"
  return 0
}

fmt_status="$(run_check fmt "$fmt_log" cargo fmt --manifest-path "$manifest" -- --check)"
test_status="$(run_check test "$test_log" cargo test --locked --manifest-path "$manifest")"
clippy_status="$(run_check clippy "$clippy_log" cargo clippy --locked --manifest-path "$manifest" --all-targets -- -D warnings)"

fmt_log_sha="$(sha256sum "$fmt_log" | awk '{print $1}')"
test_log_sha="$(sha256sum "$test_log" | awk '{print $1}')"
clippy_log_sha="$(sha256sum "$clippy_log" | awk '{print $1}')"

fmt_result="passed"
test_result="passed"
clippy_result="passed"
overall_result="passed"
if [[ "$fmt_status" -ne 0 ]]; then fmt_result="failed"; overall_result="failed"; fi
if [[ "$test_status" -ne 0 ]]; then test_result="failed"; overall_result="failed"; fi
if [[ "$clippy_status" -ne 0 ]]; then clippy_result="failed"; overall_result="failed"; fi

cat >"$receipt" <<EOF
{
  "schema": "mycelix.ncf-rust-qualification-receipt",
  "schema_version": "1.0.0",
  "source_commit": "$source_commit",
  "source_tree": "$source_tree",
  "manifest_sha256": "$manifest_sha",
  "lockfile_sha256": "$lock_sha",
  "lib_sha256": "$lib_sha",
  "reservation_module_sha256": "$reservation_sha",
  "fencing_module_sha256": "$fencing_sha",
  "rust_toolchain_log_sha256": "$toolchain_sha",
  "fmt_status": "$fmt_result",
  "fmt_exit_code": $fmt_status,
  "test_status": "$test_result",
  "test_exit_code": $test_status,
  "generated_fencing_lifecycle_test": "ten_thousand_generated_fencing_lifecycles_preserve_capacity_bound",
  "generated_fencing_lifecycle_seed": "0x4e43462d46454e31",
  "generated_fencing_lifecycle_scenarios": 10000,
  "clippy_status": "$clippy_result",
  "clippy_exit_code": $clippy_status,
  "overall_status": "$overall_result",
  "fmt_log_sha256": "$fmt_log_sha",
  "test_log_sha256": "$test_log_sha",
  "clippy_log_sha256": "$clippy_log_sha",
  "claim_ceiling": "format/test/clippy qualification of this exact standalone Rust reference crate only; not distributed/Holochain safety or production ledger correctness"
}
EOF

receipt_sha="$(sha256sum "$receipt" | awk '{print $1}')"
echo "NCF Rust qualification receipt: $receipt"
echo "Receipt SHA-256: $receipt_sha"
echo "Results: fmt=$fmt_result test=$test_result clippy=$clippy_result overall=$overall_result"

if [[ "$overall_result" != "passed" ]]; then
  exit 1
fi
