#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
import pathlib
from datetime import datetime, timezone

ROOT = pathlib.Path(__file__).resolve().parents[2]
MAN = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_011_policy_decision.json"
CASE_IDS = [f"D-{i:02d}" for i in range(1, 22)]
VALID_OUTCOMES = {"allow", "deny", "unresolved"}


def fail(message: str) -> None:
    raise SystemExit("SYM-CIVIC-011 FAIL: " + message)


def instant(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)


def maybe_instant(value: object) -> datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        return instant(value)
    except (TypeError, ValueError):
        return None


def canonical(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def decision_identity_material(candidate: dict) -> dict:
    policy = candidate.get("policy", {})
    composition = candidate.get("composition", {})
    decision = candidate.get("decision", {})
    return {
        "subject_digest": candidate.get("subject_digest"),
        "policy_id": policy.get("id"),
        "policy_version": policy.get("version"),
        "policy_digest": policy.get("digest"),
        "policy_valid_from": policy.get("valid_from"),
        "policy_valid_until": policy.get("valid_until"),
        "required_positive_properties": sorted(policy.get("required_positive_properties", [])),
        "composition_digest": composition.get("digest"),
        "evidence_ids": sorted(composition.get("evidence_ids", [])),
        "evaluated_at": decision.get("evaluated_at"),
        "decision_valid_until": decision.get("valid_until"),
        "outcome": decision.get("outcome"),
    }


def decision_identity_digest(candidate: dict) -> str:
    material = canonical(decision_identity_material(candidate)).encode("utf-8")
    return "sha256:" + hashlib.sha256(material).hexdigest()


def policy_ok(candidate: dict) -> bool:
    policy = candidate.get("policy", {})
    decision = candidate.get("decision", {})
    required = (
        "id",
        "version",
        "digest",
        "actual_digest",
        "valid_from",
        "valid_until",
        "required_positive_properties",
    )
    if any(not policy.get(field) for field in required):
        return False
    if policy.get("digest") != policy.get("actual_digest"):
        return False

    uri = str(policy.get("uri", ""))
    if not uri or uri.rstrip("/").endswith("/latest"):
        return False

    valid_from = maybe_instant(policy.get("valid_from"))
    valid_until = maybe_instant(policy.get("valid_until"))
    evaluated = maybe_instant(decision.get("evaluated_at"))
    if valid_from is None or valid_until is None or evaluated is None:
        return False
    if valid_from > valid_until or evaluated < valid_from or evaluated > valid_until:
        return False

    if decision.get("policy_id") != policy["id"]:
        return False
    if decision.get("policy_version") != policy["version"]:
        return False
    if decision.get("policy_digest") != policy["digest"]:
        return False

    decision_valid_until = maybe_instant(decision.get("valid_until"))
    if decision.get("valid_until") and decision_valid_until is None:
        return False
    if decision_valid_until is not None:
        if decision_valid_until < evaluated or decision_valid_until > valid_until:
            return False

    return True


def provenance_ok(candidate: dict) -> bool:
    composition = candidate.get("composition", {})
    decision = candidate.get("decision", {})
    composition_evidence = composition.get("evidence_ids", [])
    decision_evidence = decision.get("evidence_ids", [])

    if not candidate.get("subject_digest"):
        return False
    if candidate.get("subject_digest") != decision.get("subject_digest"):
        return False
    if not composition.get("digest"):
        return False
    if decision.get("composition_digest") != composition.get("digest"):
        return False
    if not isinstance(composition_evidence, list) or not isinstance(decision_evidence, list):
        return False
    if len(composition_evidence) != len(set(composition_evidence)):
        return False
    if len(decision_evidence) != len(set(decision_evidence)):
        return False
    if sorted(decision_evidence) != sorted(composition_evidence):
        return False

    positive = composition.get("positive_properties", [])
    negative = composition.get("negative_properties", [])
    if not isinstance(positive, list) or not isinstance(negative, list):
        return False
    if set(positive) & set(negative):
        return False

    if candidate.get("policy_field_omitted"):
        return False
    if not decision.get("id") or not decision.get("time_created") or not decision.get("evaluated_at"):
        return False

    created = maybe_instant(decision["time_created"])
    evaluated = maybe_instant(decision["evaluated_at"])
    candidate_evaluated = maybe_instant(candidate.get("evaluation_time"))
    if created is None or evaluated is None or candidate_evaluated is None:
        return False
    if candidate_evaluated != evaluated or created > evaluated:
        return False

    valid_until = maybe_instant(decision.get("valid_until"))
    if decision.get("valid_until") and valid_until is None:
        return False
    if valid_until is not None and valid_until < evaluated:
        return False

    expected_identity = decision_identity_material(candidate)
    if decision.get("identity_material") != expected_identity:
        return False

    declared_digest = decision.get("identity_digest")
    if declared_digest and declared_digest != decision_identity_digest(candidate):
        return False

    prior = candidate.get("previous_identity_material")
    if prior is not None:
        if not candidate.get("previous_decision_id"):
            return False
        if prior == expected_identity and candidate.get("previous_decision_id") == decision.get("id"):
            return False
        if prior != expected_identity and candidate.get("previous_decision_id") == decision.get("id"):
            return False

    if candidate.get("exact_replay") and candidate.get("replay_of") != decision.get("id"):
        return False

    return policy_ok(candidate)


def semantic_outcome(candidate: dict) -> str:
    composition = candidate["composition"]
    if composition.get("status") in {"CONFLICT", "INCOMPARABLE"}:
        return "unresolved"

    required = set(candidate["policy"]["required_positive_properties"])
    positive = set(composition.get("positive_properties", []))
    negative = set(composition.get("negative_properties", []))

    if not required.issubset(positive):
        return "deny"
    if required.intersection(negative):
        return "deny"
    return "allow"


def independent_reference_outcome(candidate: dict) -> str:
    status = candidate.get("composition", {}).get("status")
    if status == "CONFLICT" or status == "INCOMPARABLE":
        return "unresolved"

    required = tuple(sorted(set(candidate.get("policy", {}).get("required_positive_properties", []))))
    positive = frozenset(candidate.get("composition", {}).get("positive_properties", []))
    negative = frozenset(candidate.get("composition", {}).get("negative_properties", []))

    for property_name in required:
        if property_name not in positive or property_name in negative:
            return "deny"
    return "allow"


def semantic_rejected(candidate: dict) -> bool:
    if not provenance_ok(candidate):
        return True

    outcome = candidate["decision"].get("outcome")
    if outcome not in VALID_OUTCOMES:
        return True

    evaluations = candidate.get("policy_evaluations", [])
    if evaluations:
        outcomes = {item.get("outcome") for item in evaluations}
        if len(outcomes) == 1 and outcome != next(iter(outcomes)):
            return True
        if len(outcomes) > 1:
            if candidate.get("policy_conflict_handling") != "preserve" or outcome != "unresolved":
                return True
        elif outcome != semantic_outcome(candidate):
            return True
    elif outcome != semantic_outcome(candidate):
        return True

    if candidate.get("negative_evidence_discarded") is True:
        return True

    probe = candidate.get("monotonicity_probe")
    if probe and probe.get("after") == "allow" and probe.get("before") in {"deny", "unresolved"}:
        return True

    stale = candidate.get("stale_removal_probe")
    if stale and stale.get("after") == "allow" and stale.get("before") in {"deny", "unresolved"}:
        return True
    if candidate.get("conflict_handling") == "collapse":
        return True
    if candidate.get("order_probe", {}).get("order_dependent") is True:
        return True
    if candidate.get("authorization_mapping") in {"authorize", "quorum_to_authorization"}:
        return True
    if candidate.get("interpretation_claim") in {"scientific_truth", "resolved_fact"}:
        return True
    if candidate.get("authority_claim") in {"civic_authority", "authorization"}:
        return True
    if candidate.get("replayed_after_expiry") is True:
        return True

    return False


def disposition(candidate: dict) -> str:
    if semantic_rejected(candidate):
        return "REJECT_DECISION_PROVENANCE"
    outcome = candidate["decision"]["outcome"]
    if outcome == "allow":
        return "DECISION_ALLOW"
    if outcome == "deny":
        return "DECISION_DENY"
    return "DECISION_UNRESOLVED"


def set_identity(candidate: dict) -> None:
    candidate["decision"]["identity_material"] = decision_identity_material(candidate)


def assert_no_oracles(case: dict) -> None:
    if set(case) != {"id", "family", "candidate", "note"}:
        fail(case.get("id", "?") + " fixture surface")
    blob = canonical(case).lower()
    if any(
        token in blob
        for token in ("expected_disposition", "expected_result", "oracle_verdict", "candidate_verdict")
    ):
        fail(case["id"] + " embedded oracle")
    if any(
        key in case["candidate"]
        for key in ("authorized_decision", "civic_authorization", "raw_payload", "raw_subject_identifier")
    ):
        fail(case["id"] + " prohibited field")


def main() -> None:
    document = json.loads(MAN.read_text(encoding="utf-8"))
    if document.get("schema") != "mycelix.sym-civic.policy-decision.v1":
        fail("schema")
    if document.get("program") != "SYM-CIVIC-011":
        fail("program")
    if document.get("analysis_role") != "research_only":
        fail("role")

    cases = document.get("cases", [])
    if [case.get("id") for case in cases] != CASE_IDS:
        fail("case order")

    for case in cases:
        assert_no_oracles(case)

    reference = {case["id"]: independent_reference_outcome(case["candidate"]) for case in cases}
    derived = {case["id"]: disposition(case["candidate"]) for case in cases}

    for case in cases:
        ident = case["id"]
        if reference[ident] != semantic_outcome(case["candidate"]):
            fail(ident + " reference-outcome disagreement")

    counts = {
        key: sum(value == key for value in derived.values())
        for key in ("REJECT_DECISION_PROVENANCE", "DECISION_ALLOW", "DECISION_DENY", "DECISION_UNRESOLVED")
    }
    expected_counts = {
        "REJECT_DECISION_PROVENANCE": 15,
        "DECISION_ALLOW": 4,
        "DECISION_DENY": 1,
        "DECISION_UNRESOLVED": 1,
    }

    print("SYM-CIVIC-011 DERIVED=" + canonical({"dispositions": derived, "counts": counts}))
    if counts != expected_counts:
        fail("disposition census")

    seed = copy.deepcopy(next(case["candidate"] for case in cases if case["id"] == "D-02"))
    seed_identity = copy.deepcopy(seed["decision"]["identity_material"])
    probes = []

    mutation = copy.deepcopy(seed)
    mutation["subject_digest"] = "sha256:mutated"
    probes.append(("subject_mutation", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["policy"]["actual_digest"] = "sha256:mutated"
    probes.append(("policy_bytes_mutation", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["composition"]["digest"] = "sha256:mutated"
    probes.append(("composition_binding_mutation", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["decision"]["composition_digest"] = "sha256:mutated"
    probes.append(("decision_composition_binding_mutation", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["decision"]["outcome"] = "deny"
    probes.append(("outcome_mutation_without_identity_update", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["decision"]["outcome"] = "deny"
    set_identity(mutation)
    mutation["previous_identity_material"] = seed_identity
    mutation["previous_decision_id"] = seed["decision"]["id"]
    probes.append(("outcome_mutation_with_reused_decision_id", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    required = mutation["policy"]["required_positive_properties"][0]
    mutation["composition"]["positive_properties"] = [
        prop for prop in mutation["composition"]["positive_properties"] if prop != required
    ]
    mutation["decision"]["outcome"] = "deny"
    mutation["decision"]["id"] = "decision-new"
    mutation["previous_identity_material"] = seed_identity
    mutation["previous_decision_id"] = seed["decision"]["id"]
    set_identity(mutation)
    probes.append(("required_evidence_deletion_new_decision", disposition(mutation), "DECISION_DENY"))

    mutation = copy.deepcopy(seed)
    required = mutation["policy"]["required_positive_properties"][0]
    mutation["composition"]["positive_properties"] = [
        prop for prop in mutation["composition"]["positive_properties"] if prop != required
    ]
    mutation["decision"]["outcome"] = "allow"
    set_identity(mutation)
    probes.append(("deny_to_allow_evidence_deletion_forgery", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["composition"]["evidence_ids"] = ["vr-b", "vr-a"]
    mutation["decision"]["evidence_ids"] = ["vr-b", "vr-a"]
    set_identity(mutation)
    probes.append(("evidence_order_permutation", disposition(mutation), "DECISION_ALLOW"))

    mutation = copy.deepcopy(seed)
    mutation["decision"]["identity_digest"] = "sha256:0"
    probes.append(("identity_digest_tamper", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["exact_replay"] = True
    mutation["replay_of"] = seed["decision"]["id"]
    probes.append(("exact_replay", disposition(mutation), "DECISION_ALLOW"))

    mutation = copy.deepcopy(seed)
    mutation["exact_replay"] = True
    mutation["replay_of"] = "different-decision"
    probes.append(("wrong_replay_identity", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    mutation = copy.deepcopy(seed)
    mutation["composition"]["status"] = "CONFLICT"
    mutation["decision"]["outcome"] = "allow"
    set_identity(mutation)
    probes.append(("conflict_collapse_without_probe_flag", disposition(mutation), "REJECT_DECISION_PROVENANCE"))

    if any(actual != expected for _, actual, expected in probes):
        fail("metamorphic probe")

    print(
        "SYM-CIVIC-011 METAMORPHIC="
        + canonical([{"probe": name, "disposition": actual} for name, actual, _ in probes])
    )

    receipts = []
    for case in cases:
        candidate = copy.deepcopy(case["candidate"])
        set_identity(candidate)
        receipts.append(
            {
                "id": case["id"],
                "disposition": derived[case["id"]],
                "decision_id": candidate.get("decision", {}).get("id"),
                "decision_identity_digest": decision_identity_digest(candidate),
                "subject_digest": candidate.get("subject_digest"),
                "policy_digest": candidate.get("policy", {}).get("digest"),
                "composition_digest": candidate.get("composition", {}).get("digest"),
                "evidence_ids": sorted(candidate.get("decision", {}).get("evidence_ids", [])),
                "identity_material": candidate.get("decision", {}).get("identity_material"),
            }
        )

    payload = {"program": document["program"], "schema": document["schema"], "cases": receipts}
    receipt_digest = hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()
    print(
        "SYM-CIVIC-011 PASS: 21 policy-decision cases; "
        "rejection=15; allow=4; deny=1; unresolved=1; "
        f"canonical receipt={receipt_digest}"
    )


if __name__ == "__main__":
    main()
