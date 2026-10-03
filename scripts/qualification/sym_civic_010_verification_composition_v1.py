#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
import pathlib
from datetime import datetime, timezone

ROOT = pathlib.Path(__file__).resolve().parents[2]
MAN = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_010_verification_composition.json"
CASE_IDS = [f"M-{i:02d}" for i in range(1, 21)]
KNOWN_VERIFIERS = {
    "https://example.org/verifier/v1",
    "https://example.org/verifier/v2",
    "https://example.org/security-verifier/v1",
    "https://example.org/quality-verifier/v1",
}

def fail(msg: str) -> None:
    raise SystemExit("SYM-CIVIC-010 FAIL: " + msg)

def instant(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)

def canonical(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))

def result_digest(result: dict) -> str:
    return hashlib.sha256(canonical(result).encode("utf-8")).hexdigest()

def freshness(result: dict, evaluation_time: datetime) -> str:
    if result.get("valid_until") is not None and evaluation_time > instant(result["valid_until"]):
        return "EXPIRED"
    return "CURRENT"

def result_provenance_ok(result: dict, subject: str, evaluation_time: datetime, required_scope: set[str]) -> bool:
    if result.get("subject_digest") != subject:
        return False
    verifier = result.get("verifier", {})
    if (
        verifier.get("id") not in KNOWN_VERIFIERS
        or not verifier.get("version")
        or verifier.get("status") != "active"
    ):
        return False
    signature = result.get("signature", {})
    if signature.get("valid") is not True or signature.get("canonical_bytes_match") is not True:
        return False
    policy = result.get("policy", {})
    if not policy.get("id") or not policy.get("version") or not policy.get("digest"):
        return False
    if not result.get("id") or not result.get("time_created"):
        return False
    if instant(result["time_created"]) > evaluation_time:
        return False
    scope = set(result.get("scope", []))
    if not required_scope.issubset(scope):
        return False
    for prop in result.get("properties", []):
        if prop.get("scope") not in scope:
            return False
    return True

def duplicate_identity_conflict(results: list[dict]) -> bool:
    seen: dict[str, str] = {}
    for result in results:
        rid = result.get("id")
        digest = result_digest(result)
        if rid in seen and seen[rid] != digest:
            return True
        seen[rid] = digest
    return False

def exact_duplicate_replay(results: list[dict]) -> bool:
    seen: set[tuple[str, str]] = set()
    found = False
    for result in results:
        key = (result.get("id", ""), result_digest(result))
        if key in seen:
            found = True
        seen.add(key)
    return found

def dedupe_exact_results(results: list[dict]) -> list[dict]:
    seen: set[tuple[str, str]] = set()
    unique: list[dict] = []
    for result in results:
        key = (result.get("id", ""), result_digest(result))
        if key not in seen:
            seen.add(key)
            unique.append(result)
    return unique

def current_assertions(results: list[dict], evaluation_time: datetime) -> list[dict]:
    out: list[dict] = []
    for result in results:
        if freshness(result, evaluation_time) != "CURRENT":
            continue
        for assertion in result.get("assertions", []):
            out.append(
                {
                    "result_id": result["id"],
                    "polarity": assertion.get("polarity", "positive"),
                    "property": assertion.get("property"),
                    "value": assertion.get("value"),
                    "type": assertion.get("type", "property"),
                }
            )
    return sorted(
        out,
        key=lambda item: (
            item["type"],
            item["property"],
            str(item["value"]),
            item["polarity"],
            item["result_id"],
        ),
    )

def conflict_present(assertions: list[dict]) -> bool:
    values: dict[tuple[str, str], set[str]] = {}
    for assertion in assertions:
        key = (assertion["type"], assertion["property"])
        values.setdefault(key, set()).add(
            assertion["polarity"] + ":" + str(assertion["value"])
        )
    return any(len(values_for_property) > 1 for values_for_property in values.values())

def composition_incomparable(results: list[dict]) -> bool:
    policies = {
        (
            result.get("policy", {}).get("id"),
            result.get("policy", {}).get("version"),
            result.get("policy", {}).get("digest"),
        )
        for result in results
    }
    if any(result.get("scope_relation") == "incomparable" for result in results):
        return True
    if len(policies) <= 1:
        return False
    return any(result.get("policy_relation") == "incomparable" for result in results)

def compose(case: dict) -> tuple[str, dict]:
    candidate = case["candidate"]
    subject = candidate["subject_digest"]
    evaluation_time = instant(candidate["evaluation_time"])
    required_scope = set(candidate.get("required_scope", []))
    results = candidate.get("results", [])

    if not results:
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "no_results"}
    if duplicate_identity_conflict(results):
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "duplicate_identity_conflict"}

    replayed = exact_duplicate_replay(results)
    results = dedupe_exact_results(results)

    if candidate.get("historical_reused_as_current") is True:
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "historical_reused_as_current"}

    for result in results:
        if not result_provenance_ok(result, subject, evaluation_time, required_scope):
            return "REJECT_COMPOSITION_PROVENANCE", {"reason": "result_provenance"}

    if candidate.get("composition_mode") != "order_independent":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "order_dependence"}
    if candidate.get("conflict_handling") == "collapse":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "conflict_collapsed"}
    if (
        candidate.get("stale_selection") == "stale_wins"
        and any(freshness(result, evaluation_time) == "EXPIRED" for result in results)
    ):
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "stale_dominates_current"}
    if candidate.get("negative_assertion_retention") == "discard":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "negative_assertion_discarded"}
    if candidate.get("authorization_mapping") == "quorum_to_authorization":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "quorum_as_authorization"}
    if candidate.get("scientific_resolution") == "resolved_fact":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "scientific_conflict_as_fact"}
    if candidate.get("monotonicity_probe") == "deletion_turns_deny_into_allow":
        return "REJECT_COMPOSITION_PROVENANCE", {"reason": "non_monotonic"}

    assertions = current_assertions(results, evaluation_time)
    if composition_incomparable(results):
        disposition = "COMPOSITION_INCOMPARABLE"
    elif conflict_present(assertions):
        if candidate.get("conflict_handling") != "preserve":
            return "REJECT_COMPOSITION_PROVENANCE", {"reason": "conflict_not_preserved"}
        disposition = "COMPOSITION_CONFLICT"
    else:
        disposition = "COMPOSITION_AGREEMENT"

    ordered_results = sorted(results, key=lambda result: result["id"])
    receipt = {
        "result_ids": [result["id"] for result in ordered_results],
        "freshness": {
            result["id"]: freshness(result, evaluation_time)
            for result in ordered_results
        },
        "assertions": assertions,
        "exact_duplicate_replay": replayed,
        "policy_incomparable": composition_incomparable(results),
    }
    return disposition, receipt

def main() -> None:
    document = json.loads(MAN.read_text(encoding="utf-8"))
    if document.get("schema") != "mycelix.sym-civic.verification-composition.v1":
        fail("schema")
    if document.get("program") != "SYM-CIVIC-010":
        fail("program")
    if document.get("analysis_role") != "research_only":
        fail("role")

    cases = document.get("cases", [])
    if [case.get("id") for case in cases] != CASE_IDS:
        fail("case order")

    for case in cases:
        if set(case) != {"id", "family", "candidate", "note"}:
            fail(case.get("id", "?") + " fixture surface")
        blob = canonical(case).lower()
        if any(
            token in blob
            for token in (
                "expected_disposition",
                "expected_result",
                "oracle_verdict",
                "candidate_verdict",
            )
        ):
            fail(case["id"] + " embedded oracle")
        if any(
            key in case["candidate"]
            for key in (
                "authorized_decision",
                "civic_authorization",
                "raw_payload",
                "raw_subject_identifier",
            )
        ):
            fail(case["id"] + " prohibited field")

    derived: dict[str, str] = {}
    receipts: dict[str, dict] = {}
    for case in cases:
        disposition, receipt = compose(case)
        derived[case["id"]] = disposition
        receipts[case["id"]] = receipt

    counts = {
        key: sum(value == key for value in derived.values())
        for key in (
            "REJECT_COMPOSITION_PROVENANCE",
            "COMPOSITION_AGREEMENT",
            "COMPOSITION_CONFLICT",
            "COMPOSITION_INCOMPARABLE",
        )
    }
    expected = {
        "REJECT_COMPOSITION_PROVENANCE": 9,
        "COMPOSITION_AGREEMENT": 6,
        "COMPOSITION_CONFLICT": 2,
        "COMPOSITION_INCOMPARABLE": 3,
    }
    print("SYM-CIVIC-010 DERIVED=" + canonical({"dispositions": derived, "counts": counts}))
    if counts != expected:
        fail("disposition census")

    seed = copy.deepcopy(next(case["candidate"] for case in cases if case["id"] == "M-15"))
    probes: list[tuple[str, str, str]] = []

    mutation = copy.deepcopy(seed)
    mutation["subject_digest"] = "sha256:mutated"
    probes.append(
        ("subject_digest_mutation", compose({"candidate": mutation})[0], "REJECT_COMPOSITION_PROVENANCE")
    )

    mutation = copy.deepcopy(seed)
    mutation["results"][0]["signature"]["valid"] = False
    probes.append(
        ("signature_mutation", compose({"candidate": mutation})[0], "REJECT_COMPOSITION_PROVENANCE")
    )

    original_disposition, original_receipt = compose({"candidate": seed})
    mutation = copy.deepcopy(seed)
    mutation["results"] = list(reversed(mutation["results"]))
    reversed_disposition, reversed_receipt = compose({"candidate": mutation})
    probes.append(
        ("order_permutation", reversed_disposition, "COMPOSITION_AGREEMENT")
    )
    if canonical(original_receipt) != canonical(reversed_receipt):
        fail("order permutation changed canonical receipt")

    mutation = copy.deepcopy(seed)
    mutation["results"][1]["assertions"][0]["value"] = "fail"
    mutation["conflict_handling"] = "preserve"
    probes.append(
        ("agreement_to_conflict", compose({"candidate": mutation})[0], "COMPOSITION_CONFLICT")
    )

    mutation = copy.deepcopy(seed)
    mutation["results"][0]["scope"] = ["quality"]
    probes.append(
        ("scope_mutation", compose({"candidate": mutation})[0], "REJECT_COMPOSITION_PROVENANCE")
    )

    if any(actual != expected_disposition for _, actual, expected_disposition in probes):
        fail("metamorphic probe")
    print(
        "SYM-CIVIC-010 METAMORPHIC="
        + canonical([{"probe": name, "disposition": actual} for name, actual, _ in probes])
    )

    payload = {
        "program": document["program"],
        "schema": document["schema"],
        "cases": [
            {
                "id": case["id"],
                "disposition": derived[case["id"]],
                "receipt": receipts[case["id"]],
            }
            for case in cases
        ],
    }
    digest = hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()
    print(
        "SYM-CIVIC-010 PASS: 20 composition cases; "
        "rejection=9; agreement=6; conflict=2; incomparable=3; "
        f"canonical receipt={digest}"
    )

if __name__ == "__main__":
    main()
