#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
verifier="$root/verify-split-manifest.sh"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

cat > "$tmp/valid.json" <<'JSON'
{
  "grouping": { "source_key_review_state": "reviewed" },
  "independence_audit": {
    "status": "passed",
    "group_key_validated": true,
    "cross_split_overlap_checked": true,
    "cross_split_overlap_count": 0,
    "related_study_overlap_checked": true,
    "preprocessing_fit_on_train_only": true
  },
  "assignments": {
    "train_groups": ["study-A", "study-B"],
    "validation_groups": ["study-C"],
    "test_groups": ["study-D"]
  }
}
JSON

cat > "$tmp/overlap.json" <<'JSON'
{
  "grouping": { "source_key_review_state": "reviewed" },
  "independence_audit": {
    "status": "passed",
    "group_key_validated": true,
    "cross_split_overlap_checked": true,
    "cross_split_overlap_count": 0,
    "related_study_overlap_checked": true,
    "preprocessing_fit_on_train_only": true
  },
  "assignments": {
    "train_groups": ["study-A", "study-B"],
    "validation_groups": ["study-C"],
    "test_groups": ["study-B"]
  }
}
JSON

cat > "$tmp/excluded-overlap.json" <<'JSON'
{
  "grouping": { "source_key_review_state": "reviewed" },
  "independence_audit": {
    "status": "passed",
    "group_key_validated": true,
    "cross_split_overlap_checked": true,
    "cross_split_overlap_count": 0,
    "related_study_overlap_checked": true,
    "preprocessing_fit_on_train_only": true
  },
  "assignments": {
    "train_groups": ["study-A", "study-B"],
    "validation_groups": ["study-C"],
    "test_groups": ["study-D"],
    "excluded_groups": ["study-B"]
  }
}
JSON

cat > "$tmp/unreviewed.json" <<'JSON'
{
  "grouping": { "source_key_review_state": "provisional" },
  "independence_audit": {
    "status": "passed",
    "group_key_validated": true,
    "cross_split_overlap_checked": true,
    "cross_split_overlap_count": 0,
    "related_study_overlap_checked": true,
    "preprocessing_fit_on_train_only": true
  },
  "assignments": {
    "train_groups": ["study-A"],
    "validation_groups": ["study-B"],
    "test_groups": ["study-C"]
  }
}
JSON

bash "$verifier" "$tmp/valid.json" >/dev/null
if bash "$verifier" "$tmp/overlap.json" >/dev/null 2>&1; then
  echo "FAIL: overlapping group fixture was incorrectly accepted" >&2
  exit 1
fi
if bash "$verifier" "$tmp/unreviewed.json" >/dev/null 2>&1; then
  echo "FAIL: unreviewed source-key fixture was incorrectly accepted" >&2
  exit 1
fi
if bash "$verifier" "$tmp/excluded-overlap.json" >/dev/null 2>&1; then
  echo "FAIL: excluded group overlapping with a split was incorrectly accepted" >&2
  exit 1
fi

echo "PASS: valid manifest accepted; cross-split, excluded-group, and unreviewed-source cases rejected"
