#!/usr/bin/env bash
set -euo pipefail

: "${TLA2TOOLS_JAR:?Set TLA2TOOLS_JAR to the path of the pinned tla2tools.jar}"
if [[ ! -f "${TLA2TOOLS_JAR}" ]]; then
  printf 'TLA2TOOLS_JAR does not name a file: %s\n' "${TLA2TOOLS_JAR}" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL="TemporalFrontierNoninterference.tla"
TMP_BASE="${TMPDIR:-/tmp}"
TMP_ROOT="$(mktemp -d "${TMP_BASE}/mycelix-frontier-tlc.XXXXXX")"
trap 'rm -rf "$TMP_ROOT"' EXIT

run_tlc() {
  local label="$1"
  local config="$2"
  local work_dir="$TMP_ROOT/$label"
  mkdir -p "$work_dir"
  cp "$SCRIPT_DIR/$MODEL" "$work_dir/$MODEL"
  cp "$SCRIPT_DIR/$config" "$work_dir/$config"
  (
    cd "$work_dir"
    timeout 180s java -cp "$TLA2TOOLS_JAR" tlc2.TLC \
      -workers 1 -config "$config" "$MODEL"
  ) >"$TMP_ROOT/$label.log" 2>&1
}

# The correct specification must complete with no invariant violation.
if ! run_tlc correct TemporalFrontierNoninterference.cfg; then
  cat "$TMP_ROOT/correct.log" >&2
  printf 'FAIL: correct historical-frontier model did not complete cleanly\n' >&2
  exit 1
fi
if ! grep -Fq 'Model checking completed. No error has been found.' "$TMP_ROOT/correct.log"; then
  cat "$TMP_ROOT/correct.log" >&2
  printf 'FAIL: TLC did not report a clean correct-model completion\n' >&2
  exit 1
fi
printf 'PASS: correct model completed without invariant violations\n'

expect_counterexample() {
  local label="$1"
  local config="$2"
  local invariant="$3"

  if run_tlc "$label" "$config"; then
    cat "$TMP_ROOT/$label.log" >&2
    printf 'FAIL: mutant unexpectedly passed: %s\n' "$label" >&2
    return 1
  fi

  if ! grep -Fq "Invariant $invariant is violated." "$TMP_ROOT/$label.log"; then
    cat "$TMP_ROOT/$label.log" >&2
    printf 'FAIL: %s did not violate the expected invariant: %s\n' "$label" "$invariant" >&2
    return 1
  fi

  printf 'PASS: %s produced the expected counterexample (%s)\n' "$label" "$invariant"
}

# These are independent runs: one defect cannot mask the other.
expect_counterexample \
  latest-frontier-replay-mutant \
  TemporalFrontierNoninterference-Mutant.cfg \
  HistoricalReplayBound

expect_counterexample \
  certificate-relabel-mutant \
  TemporalFrontierNoninterference-Certificate-Mutant.cfg \
  ExactHistoricalCertificateBinding
