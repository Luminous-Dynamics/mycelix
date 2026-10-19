#!/usr/bin/env python3
"""Static evidence-contract validator for IG-007D1A workflow candidates.

This validator deliberately does not establish the D1A scientific claim.  It
only checks that a candidate workflow's machine-readable evidence assertions
are no stronger than the comparator logic that actually executes.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

VALIDATOR_ID = "ig-007d1a-evidence-contract-v1"


def inspect(text: str) -> dict[str, object]:
    compare = text.split("\n  compare:\n", 1)[1] if "\n  compare:\n" in text else ""
    checks = {
        "single_workflow_campaign_guard": "test \"$(git diff --name-only \"$PRODUCT_SHA..HEAD\")\" = \"$WORKFLOW_PATH\"" in text,
        "v4_failed_predecessor_bound": "PREDECESSOR_D1A_V4_SHA:" in text and "PREDECESSOR_D1A_V4_RUN_ID:" in text,
        "v5_superseded_candidate_bound": "PREDECESSOR_D1A_V5_SHA:" in text and "superseded_candidate_head_sha" in text,
        "d0p1_profile_blob_validated_by_comparator": "q['profile_git_blob_sha1']" in compare and "D0P1_PROFILE_BLOB" in compare,
        "capsule_recomputed_by_comparator": "cap={}" in compare and "r['product']['capsule']" in compare,
        "locked_inputs_recomputed_by_comparator": "locked={" in compare and "r['product']['locked_inputs']" in compare,
        "source_capsule_predicate_present": "'source_capsule_rederived':True" in compare,
        "receipt_schema_v7": "'schema':'ig-007d1a-receipt-v7'" in text,
        "comparison_schema_v7": "'schema':'ig-007d1a-comparison-v7'" in compare,
        "claim_ceiling_unchanged": "'claim_ceiling':'ReproducibleGovernanceBuildBytesBound'" in text,
    }
    contradictions = []
    if checks["source_capsule_predicate_present"] and not (
        checks["capsule_recomputed_by_comparator"] and checks["locked_inputs_recomputed_by_comparator"]
    ):
        contradictions.append("source_capsule_rederived predicate exceeds comparator evidence")
    if checks["receipt_schema_v7"] != checks["comparison_schema_v7"]:
        contradictions.append("producer/comparison schema generation is not jointly v7")
    required = [
        "single_workflow_campaign_guard",
        "v4_failed_predecessor_bound",
        "v5_superseded_candidate_bound",
        "d0p1_profile_blob_validated_by_comparator",
        "capsule_recomputed_by_comparator",
        "locked_inputs_recomputed_by_comparator",
        "source_capsule_predicate_present",
        "receipt_schema_v7",
        "comparison_schema_v7",
        "claim_ceiling_unchanged",
    ]
    missing = [name for name in required if not checks[name]]
    return {
        "validator_id": VALIDATOR_ID,
        "valid": not missing and not contradictions,
        "checks": checks,
        "missing": missing,
        "contradictions": contradictions,
        "grants_reproducibility_claim": False,
        "grants_product_qualification": False,
    }


def self_test() -> None:
    deficient = """
PREDECESSOR_D1A_V4_SHA: dead
jobs:
  build:
    run: test \"$(git diff --name-only \"$PRODUCT_SHA..HEAD\")\" = \"$WORKFLOW_PATH\"
  compare:
    run: |
      q=r['qualified_predecessor']
      out={'predicates':{'source_capsule_rederived':True},'claim_ceiling':'ReproducibleGovernanceBuildBytesBound'}
"""
    bad = inspect(deficient)
    assert not bad["valid"]
    assert "source_capsule_rederived predicate exceeds comparator evidence" in bad["contradictions"]

    corrected = """
PREDECESSOR_D1A_V4_SHA: dead
PREDECESSOR_D1A_V4_RUN_ID: '1'
PREDECESSOR_D1A_V5_SHA: cafe
jobs:
  build:
    run: |
      test \"$(git diff --name-only \"$PRODUCT_SHA..HEAD\")\" = \"$WORKFLOW_PATH\"
      r={'schema':'ig-007d1a-receipt-v7','claim_ceiling':'ReproducibleGovernanceBuildBytesBound','campaign':{'superseded_candidate_head_sha':'x'}}
  compare:
    run: |
      q=r['qualified_predecessor']; x=q['profile_git_blob_sha1']; y=os.environ['D0P1_PROFILE_BLOB']
      cap={}; locked={}
      if r['product']['capsule']!=cap: raise SystemExit()
      if r['product']['locked_inputs']!=locked: raise SystemExit()
      out={'schema':'ig-007d1a-comparison-v7','claim_ceiling':'ReproducibleGovernanceBuildBytesBound','predicates':{'source_capsule_rederived':True}}
"""
    good = inspect(corrected)
    assert good["valid"], good


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("workflow", nargs="?")
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    if args.self_test:
        self_test()
        print(json.dumps({"validator_id": VALIDATOR_ID, "self_test": "PASS"}, sort_keys=True))
        return 0
    if not args.workflow:
        p.error("workflow is required unless --self-test is used")
    result = inspect(Path(args.workflow).read_text(encoding="utf-8"))
    print(json.dumps(result, sort_keys=True))
    return 0 if result["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
