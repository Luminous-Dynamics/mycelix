#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
BIN="$ROOT/tools/pulse-multiconductor-audit/target/release/pulse-multiconductor-audit"
OUT_DIR="${1:-${RUNNER_TEMP:-/tmp}/pulse-multiconductor-benchmark}"
RUNS_PER_SIZE="${RUNS_PER_SIZE:-30}"
mkdir -p "$OUT_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "missing release binary: $BIN" >&2
  exit 2
fi

JSONL="$OUT_DIR/results.jsonl"
: > "$JSONL"

run_batch() {
  local events="$1"
  local repetitions="$2"
  local run
  for run in $(seq 1 "$repetitions"); do
    local report="$OUT_DIR/report-${events}-${run}.json"
    local usage="$OUT_DIR/usage-${events}-${run}.txt"
    /usr/bin/time -f "%M" -o "$usage" \
      "$BIN" --benchmark-events "$events" --output "$report"
    local rss_kib
    rss_kib="$(tr -d '[:space:]' < "$usage")"
    jq -c --argjson repetition "$run" --argjson peak_rss_kib "$rss_kib" \
      '. + {repetition: $repetition, peak_rss_kib: $peak_rss_kib}' \
      "$report" >> "$JSONL"
    echo "completed events=$events repetition=$run peak_rss_kib=$rss_kib"
  done
}

# Independent process-level repetitions. The 1M-event stretch is a single
# capacity probe; never use it as a p95 sample population.
run_batch 1000 "$RUNS_PER_SIZE"
run_batch 10000 "$RUNS_PER_SIZE"
run_batch 100000 "$RUNS_PER_SIZE"
run_batch 1000000 1

jq -s '
  group_by(.events)
  | map({
      events: .[0].events,
      repetitions: length,
      status_counts: (group_by(.status) | map({status: .[0].status, count: length})),
      median_audit_ns: ([.[].audit_ns] | sort | .[(length / 2 | floor)]),
      min_audit_ns: ([.[].audit_ns] | min),
      max_audit_ns: ([.[].audit_ns] | max),
      median_events_per_second: ([.[].audit_events_per_second] | sort | .[(length / 2 | floor)]),
      max_peak_rss_kib: ([.[].peak_rss_kib] | max)
    })
' "$JSONL" > "$OUT_DIR/summary.json"

cat "$OUT_DIR/summary.json"
