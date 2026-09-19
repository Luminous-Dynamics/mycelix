#!/usr/bin/env python3
"""Pure evaluator for CI-GOV-001J administrative observations (P0-P3)."""
from __future__ import annotations

import argparse
import copy
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

EVALUATOR_ID = "ci-gov-001j-admin-observation-evaluator-v1"
OBSERVATION_CONTRACT_ID = "ci-gov-001j-admin-observation-v1"
PROMOTION_CONTRACT_ID = "ci-gov-001j-authority-promotion-v1"
SHA40 = re.compile(r"^[0-9a-f]{40}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
EXPECTED_REPO = "Luminous-Dynamics/mycelix"
EXPECTED_REF = "refs/heads/ci-authority/v1"
EXPECTED_AUTHORITY_SHA = "979160704924d992c4f1c1af32279615c83ae062"
EXPECTED_QUAL_HEAD = "db0e85427f4482641decd8d9a8ff3c1a4ecd10b0"
EXPECTED_WORKFLOW = ".github/workflows/ruleset-generic-ci-authority.yml"


class EvidenceError(ValueError):
    pass


def require(ok: bool, msg: str) -> None:
    if not ok:
        raise EvidenceError(msg)


def parse_ts(value: Any, label: str) -> datetime:
    require(isinstance(value, str), f"{label} must be ISO-8601 string")
    try:
        dt = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise EvidenceError(f"{label} invalid timestamp: {exc}") from exc
    require(dt.tzinfo is not None, f"{label} must be timezone-aware")
    return dt.astimezone(timezone.utc)


def require_sha40(value: Any, label: str) -> str:
    require(isinstance(value, str) and SHA40.fullmatch(value) is not None,
            f"{label} must be lowercase 40-hex")
    return value


def require_sha256(value: Any, label: str) -> str:
    require(isinstance(value, str) and SHA256.fullmatch(value) is not None,
            f"{label} must be lowercase sha256")
    return value


def validate_contract(c: Any) -> None:
    require(isinstance(c, dict), "observation contract root must be object")
    require(c.get("observation_contract_id") == OBSERVATION_CONTRACT_ID,
            "observation contract identity drift")
    require(c.get("version") == 1, "observation contract version drift")
    require(c.get("issue") == 1705, "issue drift")
    require(c.get("promotion_contract_id") == PROMOTION_CONTRACT_ID,
            "promotion contract identity drift")
    require_sha40(c.get("promotion_contract_head"), "promotion_contract_head")
    require(c.get("repository") == EXPECTED_REPO, "repository drift")
    require(c.get("authority_ref") == EXPECTED_REF, "authority ref drift")
    require(c.get("expected_authority_source_sha") == EXPECTED_AUTHORITY_SHA,
            "authority source drift")
    sq = c.get("source_qualification")
    require(isinstance(sq, dict), "source_qualification missing")
    require(sq.get("pr") == 1704, "source qualification PR drift")
    require(sq.get("head_sha") == EXPECTED_QUAL_HEAD, "source qualification head drift")
    require(sq.get("required_claim") == "RulesetAuthoritySourceCapsuleBound",
            "source qualification claim drift")
    require(c.get("raw_payload_sha256_required") is True,
            "raw payload commitments must be required")
    require(c.get("screenshots_authoritative") is False,
            "screenshots must not be authoritative")
    require(c.get("normalized_only_evidence_forbidden") is True,
            "normalized-only evidence must be forbidden")
    sessions = c.get("capture_sessions")
    require(isinstance(sessions, dict), "capture_sessions missing")
    require(sessions.get("before_p4", {}).get("required_states") == ["P1", "P2", "P3"],
            "before_p4 state census drift")
    require(sessions.get("before_p4", {}).get("max_capture_span_seconds") == 300,
            "before_p4 freshness drift")
    require(sessions.get("before_p6", {}).get("required_states") == ["P1", "P2", "P4"],
            "before_p6 state census drift")
    require(sessions.get("before_p6", {}).get("max_capture_span_seconds") == 300,
            "before_p6 freshness drift")


def validate_p0(e: Any) -> dict[str, Any]:
    require(isinstance(e, dict), "P0 receipt missing")
    require(e.get("claim_ceiling") == "RulesetAuthoritySourceCapsuleBound",
            "P0 claim ceiling drift")
    require(e.get("claim_established") is True, "P0 claim not established")
    require(e.get("qualification_head_sha") == EXPECTED_QUAL_HEAD,
            "P0 qualification head drift")
    require_sha256(e.get("receipt_sha256"), "P0 receipt_sha256")
    require(isinstance(e.get("run_id"), int) and e["run_id"] > 0,
            "P0 run_id invalid")
    require(isinstance(e.get("artifact_id"), int) and e["artifact_id"] > 0,
            "P0 artifact_id invalid")
    return {"state":"P0", "claim":"RulesetAuthoritySourceCapsuleBound", "established":True}


def snapshot_common(s: Any, state: str, session_id: str) -> datetime:
    require(isinstance(s, dict), f"{state} snapshot missing")
    require(s.get("state") == state, f"{state} state tag drift")
    require(s.get("capture_session_id") == session_id, f"{state} capture-session drift")
    require_sha256(s.get("raw_payload_sha256"), f"{state} raw_payload_sha256")
    require(s.get("raw_payload_retained") is True, f"{state} raw payload must be retained")
    require(s.get("screenshot_only") is False, f"{state} screenshot-only evidence forbidden")
    return parse_ts(s.get("observed_at_utc"), f"{state}.observed_at_utc")


def validate_p1(s: Any, session_id: str) -> tuple[dict[str, Any], datetime]:
    ts = snapshot_common(s, "P1", session_id)
    require(s.get("observer") == "github_rest", "P1 observer must be github_rest")
    require(s.get("repository") == EXPECTED_REPO, "P1 repository drift")
    require(s.get("ref") == EXPECTED_REF, "P1 ref drift")
    require(s.get("exists") is True, "P1 authority ref absent")
    require(s.get("sha") == EXPECTED_AUTHORITY_SHA, "P1 source SHA mismatch")
    return ({"state":"P1", "claim":"AuthorityRefExactSourceBound", "established":True}, ts)


def validate_p2(s: Any, session_id: str) -> tuple[dict[str, Any], datetime]:
    ts = snapshot_common(s, "P2", session_id)
    require(s.get("observer") == "github_admin_rest", "P2 requires admin-capable REST observer")
    require(s.get("repository") == EXPECTED_REPO, "P2 repository drift")
    require(s.get("branch") == "ci-authority/v1", "P2 branch drift")
    p = s.get("protection")
    require(isinstance(p, dict), "P2 protection object missing")
    for key in ("block_deletion", "block_force_push", "block_unqualified_direct_update"):
        require(p.get(key) is True, f"P2 protection missing {key}")
    bypass = p.get("bypass_identities")
    require(isinstance(bypass, list), "P2 bypass identities must be explicit list")
    require(all(isinstance(x, str) and x for x in bypass), "P2 bypass identity invalid")
    require(len(bypass) == len(set(bypass)), "P2 bypass identities duplicated")
    review = s.get("review_attestation")
    require(isinstance(review, dict), "P2 review attestation missing")
    require(review.get("bypass_set_minimized") is True,
            "P2 bypass minimization not reviewed")
    require(isinstance(review.get("reviewer_identity"), str) and review["reviewer_identity"],
            "P2 reviewer identity missing")
    require_sha256(review.get("attestation_sha256"), "P2 attestation_sha256")
    return ({
        "state":"P2", "claim":"AuthorityRefProtectionBound", "established":True,
        "bypass_identities": bypass,
        "direct_observation": True,
        "review_attestation": True,
    }, ts)


def validate_p3(s: Any, session_id: str) -> tuple[dict[str, Any], datetime]:
    ts = snapshot_common(s, "P3", session_id)
    require(s.get("observer") in {"github_admin_api", "github_admin_ui_export"},
            "P3 requires admin capability observation")
    require(s.get("required_workflow_selector_visible") is True,
            "P3 required-workflow selector not observed")
    sel = s.get("selector")
    require(isinstance(sel, dict), "P3 selector missing")
    require(sel.get("source_repository") == EXPECTED_REPO, "P3 source repo drift")
    require(sel.get("source_branch") == "ci-authority/v1", "P3 source branch drift")
    require(sel.get("workflow_path") == EXPECTED_WORKFLOW, "P3 workflow path drift")
    require(isinstance(s.get("observer_identity"), str) and s["observer_identity"],
            "P3 observer identity missing")
    return ({"state":"P3", "claim":"RequiredWorkflowCapabilityObserved", "established":True}, ts)


def evaluate(contract: Any, evidence: Any) -> dict[str, Any]:
    validate_contract(contract)
    require(isinstance(evidence, dict), "evidence root must be object")
    require(evidence.get("schema") == "ci-gov-001j-admin-evidence-v1", "evidence schema drift")
    require(evidence.get("repository") == EXPECTED_REPO, "evidence repository drift")
    session_id = evidence.get("capture_session_id")
    require(isinstance(session_id, str) and 8 <= len(session_id) <= 128,
            "capture_session_id invalid")

    p0 = validate_p0(evidence.get("P0"))
    p1, t1 = validate_p1(evidence.get("P1"), session_id)
    p2, t2 = validate_p2(evidence.get("P2"), session_id)
    p3, t3 = validate_p3(evidence.get("P3"), session_id)
    times = [t1, t2, t3]
    span = (max(times) - min(times)).total_seconds()
    require(span <= 300, f"P1-P3 capture span too large: {span}s")
    require(t1 <= t2 <= t3, "P1-P3 observations must be monotonic")

    return {
        "evaluator_id": EVALUATOR_ID,
        "valid": True,
        "capture_session_id": session_id,
        "capture_span_seconds": span,
        "states": [p0, p1, p2, p3],
        "highest_state": "P3",
        "claim_ceiling": "RequiredWorkflowCapabilityObserved",
        "transition_guard_satisfied": "before_p4",
        "raw_payloads_retained": True,
        "screenshots_authoritative": False,
        "grants_p4_mutation": False,
        "grants_ruleset_activation": False,
        "grants_product_qualification": False,
    }


def fixture() -> tuple[dict[str, Any], dict[str, Any]]:
    contract = json.loads(Path("docs/ci/ruleset_authority_observation_v1.json").read_text())
    sid = "fixture-session-001"
    evidence = {
        "schema":"ci-gov-001j-admin-evidence-v1",
        "repository":EXPECTED_REPO,
        "capture_session_id":sid,
        "P0":{"claim_ceiling":"RulesetAuthoritySourceCapsuleBound","claim_established":True,
              "qualification_head_sha":EXPECTED_QUAL_HEAD,"receipt_sha256":"1"*64,"run_id":1,"artifact_id":1},
        "P1":{"state":"P1","capture_session_id":sid,"raw_payload_sha256":"2"*64,
              "raw_payload_retained":True,"screenshot_only":False,"observed_at_utc":"2026-09-19T00:00:00Z",
              "observer":"github_rest","repository":EXPECTED_REPO,"ref":EXPECTED_REF,"exists":True,
              "sha":EXPECTED_AUTHORITY_SHA},
        "P2":{"state":"P2","capture_session_id":sid,"raw_payload_sha256":"3"*64,
              "raw_payload_retained":True,"screenshot_only":False,"observed_at_utc":"2026-09-19T00:01:00Z",
              "observer":"github_admin_rest","repository":EXPECTED_REPO,"branch":"ci-authority/v1",
              "protection":{"block_deletion":True,"block_force_push":True,"block_unqualified_direct_update":True,
                            "bypass_identities":["github-admin:example"]},
              "review_attestation":{"bypass_set_minimized":True,"reviewer_identity":"reviewer:example",
                                    "attestation_sha256":"4"*64}},
        "P3":{"state":"P3","capture_session_id":sid,"raw_payload_sha256":"5"*64,
              "raw_payload_retained":True,"screenshot_only":False,"observed_at_utc":"2026-09-19T00:02:00Z",
              "observer":"github_admin_api","required_workflow_selector_visible":True,
              "selector":{"source_repository":EXPECTED_REPO,"source_branch":"ci-authority/v1",
                          "workflow_path":EXPECTED_WORKFLOW},"observer_identity":"admin:example"},
    }
    return contract, evidence


def must_fail(contract: dict[str, Any], evidence: dict[str, Any]) -> None:
    try:
        evaluate(contract, evidence)
    except EvidenceError:
        return
    raise AssertionError("unsafe evidence unexpectedly validated")


def self_test() -> None:
    contract, evidence = fixture()
    evaluate(contract, evidence)

    x = copy.deepcopy(evidence); x["P1"]["sha"] = "0"*40; must_fail(contract, x)
    x = copy.deepcopy(evidence); x["P2"]["protection"]["block_force_push"] = False; must_fail(contract, x)
    x = copy.deepcopy(evidence); x["P2"]["review_attestation"]["bypass_set_minimized"] = False; must_fail(contract, x)
    x = copy.deepcopy(evidence); x["P3"]["selector"]["source_branch"] = "main"; must_fail(contract, x)
    x = copy.deepcopy(evidence); x["P3"]["observed_at_utc"] = "2026-09-19T00:10:00Z"; must_fail(contract, x)
    x = copy.deepcopy(evidence); x["P2"]["raw_payload_retained"] = False; must_fail(contract, x)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contract", default="docs/ci/ruleset_authority_observation_v1.json")
    p.add_argument("--evidence")
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    try:
        if args.self_test:
            self_test()
            print(json.dumps({"evaluator_id":EVALUATOR_ID,"self_test":"PASS","claim_ceiling":"RequiredWorkflowCapabilityObserved","grants_p4_mutation":False,"grants_ruleset_activation":False}, sort_keys=True))
            return 0
        require(args.evidence is not None, "--evidence is required unless --self-test")
        contract = json.loads(Path(args.contract).read_text(encoding="utf-8"))
        evidence = json.loads(Path(args.evidence).read_text(encoding="utf-8"))
        print(json.dumps(evaluate(contract, evidence), sort_keys=True))
        return 0
    except (OSError, json.JSONDecodeError, EvidenceError, AssertionError) as exc:
        print(json.dumps({"evaluator_id":EVALUATOR_ID,"valid":False,"reason":str(exc),"claim_ceiling":"RequiredWorkflowCapabilityObserved","grants_p4_mutation":False,"grants_ruleset_activation":False,"grants_product_qualification":False}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
