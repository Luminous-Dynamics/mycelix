#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: bash verify-split-manifest.sh <manifest.json>" >&2
  exit 64
fi

manifest="$1"
if [[ ! -f "$manifest" ]]; then
  echo "manifest does not exist: $manifest" >&2
  exit 66
fi

if ! jq -e 'type == "object"' "$manifest" >/dev/null; then
  echo "manifest is not a JSON object" >&2
  exit 65
fi

status="$(jq -r '.independence_audit.status // "missing"' "$manifest")"
if [[ "$status" != "passed" ]]; then
  echo "BLOCKED: manifest status is '$status', not 'passed'" >&2
  exit 2
fi

# Recompute safety-critical assertions from the actual group lists. Never trust
# the manifest's overlap_count or booleans without checking the assignments.
if ! jq -e '
  .grouping.source_key_review_state == "reviewed"
  and (.independence_audit.group_key_validated == true)
  and (.independence_audit.cross_split_overlap_checked == true)
  and (.independence_audit.cross_split_overlap_count == 0)
  and (.independence_audit.related_study_overlap_checked == true)
  and (.independence_audit.preprocessing_fit_on_train_only == true)
  and ((.assignments.train_groups | length) > 0)
  and ((.assignments.validation_groups | length) > 0)
  and ((.assignments.test_groups | length) > 0)
  and (
    [
      .assignments.train_groups[],
      .assignments.validation_groups[],
      .assignments.test_groups[],
      .assignments.excluded_groups[]?
    ] as $all
    | ($all | length) == ($all | unique | length)
  )
' "$manifest" >/dev/null; then
  echo "BLOCKED: source identity, preprocessing, non-empty splits, or group-disjointness invariant failed" >&2
  exit 2
fi

echo "PASS: grouped split manifest is internally disjoint and declares reviewed source keys"
echo "NOTE: this structural verifier does not prove that source-study identifiers are scientifically correct or that the estimand is identifiable"
