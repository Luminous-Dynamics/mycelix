#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_013_finality_basis.json"

PROGRAM = "SYM-CIVIC-013"
SCHEMA = "mycelix.sym-civic.finality-basis.v1"
REJECT = "REJECT_FINALITY_BASIS_PROVENANCE"
SUFFICIENT = "FINALITY_BASIS_SUFFICIENT"
INSUFFICIENT = "FINALITY_BASIS_INSUFFICIENT"
UNRESOLVED = "FINALITY_BASIS_UNRESOLVED"
RECEIPT_ANCHOR = "3b9c05f6dd4cf54603516ec8af207745dd9acd826581fb445a0dbbed47f1a87a"
EVIDENCE_FRESHNESS_BOUNDARY = "2026-10-03T23:00:00Z"

BASE_DECISION = {
    "id": "decision-a",
    "subject_digest": "sha256:subject-a",
    "rendered_at": "2026-10-03T01:00:00Z",
    "valid_until": "2026-10-05T00:00:00Z",
    "appeal_deadline": "2026-10-04T00:00:00Z",
    "deadline_profile_ref": "deadline/finality-window/a",
    "deadline_profile_digest": "sha256:deadline-a",
}
BASE_AUTHORITY = {
    "profile_ref": "authority/finality-basis/a",
    "profile_version": "2026-10",
    "profile_digest": "sha256:authority-basis-a",
    "issuer": "issuer-a",
    "trusted_issuers": ["issuer-a", "issuer-b"],
}


def canon(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def digest(value):
    return "sha256:" + hashlib.sha256(canon(value).encode("utf-8")).hexdigest()


def ts(value):
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def clone(value):
    return copy.deepcopy(value)


def apply(base, mutation):
    value = clone(base)
    for section, patch in mutation.items():
        if section in {"q", "replay", "reseal"}:
            continue
        value[section] = {**value.get(section, {}), **patch}
    return value


def required_decision_material(decision):
    return {
        key: decision.get(key)
        for key in (
            "id",
            "subject_digest",
            "rendered_at",
            "valid_until",
            "appeal_deadline",
            "deadline_profile_ref",
            "deadline_profile_digest",
        )
    }


def base_candidate(kind, q):
    decision = clone(BASE_DECISION)
    decision["identity_material"] = required_decision_material(decision)
    decision["identity_digest"] = digest(decision["identity_material"])

    authority = clone(BASE_AUTHORITY)

    if kind == "closed":
        basis = {
            "kind": "closed_window",
            "decision_id": "decision-a",
            "decision_identity_digest": decision["identity_digest"],
            "coverage": {
                "universe_id": "appeal-vds/a",
                "universe_profile": "appeal-coverage@1",
                "scope_id": "decision-a",
                "scope_digest": "sha256:scope-a",
                "start": "2026-10-03T01:00:00Z",
                "end": q,
                "cut": q,
                "complete": True,
                "proof_supported": True,
                "proof_type": "non-inclusion",
                "checkpoint_id": "checkpoint-a",
                "checkpoint_digest": "sha256:checkpoint-a",
                "proof_digest": "sha256:proof-a",
                "observed_appeals": [],
                "local_query_only": False,
                "receipt_id": "receipt-a",
                "receipt_digest": "sha256:receipt-a",
                "freshness_profile_cut": EVIDENCE_FRESHNESS_BOUNDARY,
            },
            "authority": authority,
            "deadline": {
                "deadline": decision["appeal_deadline"],
                "profile_ref": decision["deadline_profile_ref"],
                "profile_digest": decision["deadline_profile_digest"],
                "provenance_exact": True,
            },
            "qualification_time": q,
        }
    else:
        basis = {
            "kind": "terminal_resolution",
            "decision_id": "decision-a",
            "decision_identity_digest": decision["identity_digest"],
            "appeal": {
                "decision_id": "decision-a",
                "decision_identity_digest": decision["identity_digest"],
                "filed_at": "2026-10-03T08:00:00Z",
                "resolution_id": "resolution-a",
                "resolution_digest": "sha256:resolution-a",
                "resolved_at": "2026-10-03T10:00:00Z",
                "disposition": "AFFIRMED",
                "terminal": True,
                "future_relative_to_qualification": False,
                "authority_ref": "appellate-authority/a",
                "authority_digest": "sha256:app-authority-a",
            },
            "authority": authority,
            "qualification_time": q,
        }

    basis["identity_material"] = clone(basis)
    basis["identity_digest"] = digest(basis["identity_material"])
    return {"decision": decision, "basis": basis}


def reseal(candidate):
    decision = candidate["decision"]
    decision["identity_material"] = required_decision_material(decision)
    decision["identity_digest"] = digest(decision["identity_material"])

    basis = candidate["basis"]
    basis_without_identity = clone(basis)
    basis_without_identity.pop("identity_material", None)
    basis_without_identity.pop("identity_digest", None)
    basis["identity_material"] = clone(basis_without_identity)
    basis["identity_digest"] = digest(basis["identity_material"])


def candidate_for(spec):
    mutation = spec.get("mutation", {})
    q = mutation.get("q")
    if q is None:
        q = "2026-10-03T12:00:00Z" if spec["kind"] == "terminal" else "2026-10-04T00:00:00Z"

    candidate = base_candidate(spec["kind"], q)

    for section, patch in mutation.items():
        if section in {"q", "replay", "reseal"}:
            continue
        if section == "decision":
            candidate["decision"].update(patch)
        elif section == "basis":
            candidate["basis"].update(patch)
        elif section in {"coverage", "authority", "deadline", "appeal"}:
            candidate["basis"][section].update(patch)
        else:
            raise ValueError("unknown mutation section: " + section)

    if mutation.get("reseal"):
        reseal(candidate)
    if mutation.get("replay"):
        candidate["replay_of"] = "F-01"
    return candidate


def provenance(candidate):
    decision = candidate["decision"]
    basis = candidate["basis"]

    if decision.get("identity_material") != required_decision_material(decision):
        return False
    if decision.get("identity_digest") != digest(decision["identity_material"]):
        return False

    stored_basis = clone(basis)
    basis_identity_material = stored_basis.pop("identity_material", None)
    basis_identity_digest = stored_basis.pop("identity_digest", None)
    if basis_identity_material != stored_basis:
        return False
    if basis_identity_digest != digest(stored_basis):
        return False

    if basis.get("decision_id") != decision.get("id"):
        return False
    if basis.get("decision_identity_digest") != decision.get("identity_digest"):
        return False

    if ts(basis["qualification_time"]) < ts(decision["rendered_at"]):
        return False

    authority = basis.get("authority", {})
    if authority.get("profile_ref") != BASE_AUTHORITY["profile_ref"]:
        return False
    if authority.get("profile_version") != BASE_AUTHORITY["profile_version"]:
        return False
    if authority.get("profile_digest") != BASE_AUTHORITY["profile_digest"]:
        return False
    if authority.get("issuer") not in authority.get("trusted_issuers", []):
        return False

    if basis["kind"] == "closed_window":
        coverage = basis.get("coverage", {})
        if coverage.get("universe_id") != "appeal-vds/a":
            return False
        if coverage.get("universe_profile") != "appeal-coverage@1":
            return False
        if coverage.get("scope_id") != "decision-a":
            return False
        if coverage.get("scope_digest") != "sha256:scope-a":
            return False
        if coverage.get("checkpoint_id") != "checkpoint-a":
            return False
        if coverage.get("checkpoint_digest") != "sha256:checkpoint-a":
            return False
        if coverage.get("proof_digest") != "sha256:proof-a":
            return False
        if coverage.get("freshness_profile_cut") != EVIDENCE_FRESHNESS_BOUNDARY:
            return False

        deadline = basis.get("deadline", {})
        if deadline.get("deadline") != decision.get("appeal_deadline"):
            return False
        if deadline.get("profile_ref") != decision.get("deadline_profile_ref"):
            return False
        if deadline.get("profile_digest") != decision.get("deadline_profile_digest"):
            return False
        if deadline.get("provenance_exact") is not True:
            return False
    elif basis["kind"] == "terminal_resolution":
        appeal = basis.get("appeal", {})
        if appeal.get("decision_id") != decision.get("id"):
            return False
        if appeal.get("decision_identity_digest") != decision.get("identity_digest"):
            return False
        if appeal.get("resolution_id") != "resolution-a":
            return False
        if appeal.get("resolution_digest") != "sha256:resolution-a":
            return False
        if appeal.get("authority_ref") != "appellate-authority/a":
            return False
        if appeal.get("authority_digest") != "sha256:app-authority-a":
            return False
    else:
        return False

    if ts(basis["qualification_time"]) > ts(decision["valid_until"]):
        return "expired"
    return "valid"


def primary_qualify(candidate):
    prov = provenance(candidate)
    if prov is False:
        return REJECT
    if prov == "expired":
        return INSUFFICIENT

    decision = candidate["decision"]
    basis = candidate["basis"]
    q = ts(basis["qualification_time"])
    authority = basis["authority"]

    if authority.get("applicable_at_cut") is False:
        return INSUFFICIENT
    if authority.get("valid_until") and q > ts(authority["valid_until"]):
        return INSUFFICIENT

    if basis["kind"] == "closed_window":
        coverage = basis["coverage"]
        if q < ts(decision["appeal_deadline"]):
            return INSUFFICIENT
        if ts(coverage["start"]) > ts(decision["rendered_at"]):
            return INSUFFICIENT
        if ts(coverage["end"]) < q or ts(coverage["cut"]) < q:
            return INSUFFICIENT
        for field in ("receipt_fresh_until", "checkpoint_state_cut", "proof_fresh_until"):
            if field in coverage and q > ts(coverage[field]):
                if field == "receipt_fresh_until":
                    return REJECT
                return INSUFFICIENT
        if coverage.get("continuity_proven") is False:
            return INSUFFICIENT
        if coverage.get("proof_supported") is not True:
            return INSUFFICIENT
        if coverage.get("proof_type") != "non-inclusion":
            return INSUFFICIENT
        if coverage.get("complete") is not True:
            return INSUFFICIENT
        if coverage.get("local_query_only") is True:
            return INSUFFICIENT
        if coverage.get("observed_appeals") != []:
            return INSUFFICIENT
        return SUFFICIENT

    appeal = basis["appeal"]
    filed = ts(appeal["filed_at"])
    resolved = ts(appeal["resolved_at"])

    if resolved < filed:
        return REJECT
    if appeal.get("future_relative_to_qualification") is True:
        return REJECT
    if resolved > q:
        return REJECT
    if filed >= ts(decision["appeal_deadline"]):
        return INSUFFICIENT
    if appeal.get("terminal") is not True or appeal.get("disposition") != "AFFIRMED":
        return INSUFFICIENT
    return SUFFICIENT


def independent_qualify(candidate):
    decision = candidate["decision"]
    basis = candidate["basis"]
    if decision.get("identity_digest") != digest(decision.get("identity_material")):
        return REJECT
    basis_body = {k: v for k, v in basis.items() if k not in {"identity_material", "identity_digest"}}
    if basis.get("identity_material") != basis_body:
        return REJECT
    if basis.get("identity_digest") != digest(basis_body):
        return REJECT

    if basis.get("decision_id") != decision.get("id"):
        return REJECT
    if basis.get("decision_identity_digest") != decision.get("identity_digest"):
        return REJECT

    try:
        q = ts(basis["qualification_time"])
        rendered = ts(decision["rendered_at"])
        valid_until = ts(decision["valid_until"])
    except (KeyError, TypeError, ValueError):
        return REJECT

    if q < rendered:
        return REJECT
    if q > valid_until:
        return INSUFFICIENT

    authority = basis.get("authority", {})
    authority_ok = (
        authority.get("profile_ref") == BASE_AUTHORITY["profile_ref"]
        and authority.get("profile_version") == BASE_AUTHORITY["profile_version"]
        and authority.get("profile_digest") == BASE_AUTHORITY["profile_digest"]
        and authority.get("issuer") in authority.get("trusted_issuers", [])
    )
    if not authority_ok:
        return REJECT

    if authority.get("applicable_at_cut") is False:
        return INSUFFICIENT
    if authority.get("valid_until") and q > ts(authority["valid_until"]):
        return INSUFFICIENT

    if basis["kind"] == "terminal_resolution":
        appeal = basis["appeal"]
        if appeal.get("decision_id") != "decision-a" or appeal.get("decision_identity_digest") != decision["identity_digest"]:
            return REJECT
        if appeal.get("resolution_id") != "resolution-a" or appeal.get("resolution_digest") != "sha256:resolution-a":
            return REJECT
        if appeal.get("authority_ref") != "appellate-authority/a" or appeal.get("authority_digest") != "sha256:app-authority-a":
            return REJECT
        if ts(appeal["resolved_at"]) < ts(appeal["filed_at"]) or appeal.get("future_relative_to_qualification") is True or ts(appeal["resolved_at"]) > q:
            return REJECT
        if ts(appeal["filed_at"]) >= ts(decision["appeal_deadline"]):
            return INSUFFICIENT
        if appeal.get("terminal") is not True or appeal.get("disposition") != "AFFIRMED":
            return INSUFFICIENT
        return SUFFICIENT

    coverage = basis["coverage"]
    exact_refs = (
        coverage.get("universe_id") == "appeal-vds/a"
        and coverage.get("universe_profile") == "appeal-coverage@1"
        and coverage.get("scope_id") == "decision-a"
        and coverage.get("scope_digest") == "sha256:scope-a"
        and coverage.get("checkpoint_id") == "checkpoint-a"
        and coverage.get("checkpoint_digest") == "sha256:checkpoint-a"
        and coverage.get("proof_digest") == "sha256:proof-a"
    )
    if not exact_refs:
        return REJECT
    if coverage.get("freshness_profile_cut") != EVIDENCE_FRESHNESS_BOUNDARY:
        return REJECT

    deadline = basis["deadline"]
    deadline_ok = (
        deadline.get("deadline") == decision["appeal_deadline"]
        and deadline.get("profile_ref") == decision["deadline_profile_ref"]
        and deadline.get("profile_digest") == decision["deadline_profile_digest"]
        and deadline.get("provenance_exact") is True
    )
    if not deadline_ok:
        return REJECT

    if q < ts(decision["appeal_deadline"]):
        return INSUFFICIENT
    if ts(coverage["start"]) > rendered:
        return INSUFFICIENT
    if ts(coverage["end"]) < q or ts(coverage["cut"]) < q:
        return INSUFFICIENT
    for field in ("receipt_fresh_until", "checkpoint_state_cut", "proof_fresh_until"):
        if field in coverage and q > ts(coverage[field]):
            return REJECT if field == "receipt_fresh_until" else INSUFFICIENT
    if coverage.get("continuity_proven") is False:
        return INSUFFICIENT
    if coverage.get("proof_supported") is not True or coverage.get("proof_type") != "non-inclusion":
        return INSUFFICIENT
    if coverage.get("complete") is not True or coverage.get("local_query_only") is True:
        return INSUFFICIENT
    if coverage.get("observed_appeals") != []:
        return INSUFFICIENT
    return SUFFICIENT


def main():
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))

    assert document["schema"] == SCHEMA
    assert document["program"] == PROGRAM
    assert document["analysis_role"] == "research_only"
    assert document["parent_subject"] == "0fee3f3708e66a40b02420cbedfed14e33457aa9"
    assert document["canonical_receipt_anchor"] == RECEIPT_ANCHOR

    cases = document["cases"]
    assert [case["id"] for case in cases] == [f"F-{i:02d}" for i in range(1, 43)]

    for case in cases:
        assert set(case) == {"id", "family", "kind", "mutation"}
        lowered = canon(case).lower()
        assert not any(token in lowered for token in (
            "expected_disposition",
            "expected_result",
            "oracle_verdict",
            "candidate_verdict",
        ))

    candidates = {case["id"]: candidate_for(case) for case in cases}
    primary = {case_id: primary_qualify(candidates[case_id]) for case_id in candidates}
    independent = {case_id: independent_qualify(candidates[case_id]) for case_id in candidates}

    assert primary == independent, "independent qualifier disagreement"

    census = {
        REJECT: sum(value == REJECT for value in primary.values()),
        SUFFICIENT: sum(value == SUFFICIENT for value in primary.values()),
        INSUFFICIENT: sum(value == INSUFFICIENT for value in primary.values()),
        UNRESOLVED: sum(value == UNRESOLVED for value in primary.values()),
    }
    assert census == {REJECT: 17, SUFFICIENT: 5, INSUFFICIENT: 20, UNRESOLVED: 0}, census

    seed_closed = clone(candidates["F-01"])
    seed_terminal = clone(candidates["F-02"])

    metamorphic = []
    probes = [
        ("decision_semantics_mutation", lambda x: x["decision"].update({"subject_digest": "sha256:mutated"}), REJECT, False),
        ("basis_identity_tamper", lambda x: x["basis"].update({"identity_digest": "sha256:tampered"}), REJECT, False),
        ("completeness_removal", lambda x: x["basis"]["coverage"].update({"complete": False}), INSUFFICIENT, True),
        ("decisive_appeal_insertion", lambda x: x["basis"]["coverage"].update({"observed_appeals": ["appeal-live"]}), INSUFFICIENT, True),
        ("coverage_start_delay", lambda x: x["basis"]["coverage"].update({"start": "2026-10-03T02:00:00Z"}), INSUFFICIENT, True),
        ("coverage_cut_shortening", lambda x: x["basis"]["coverage"].update({"end": "2026-10-03T23:00:00Z", "cut": "2026-10-03T23:00:00Z"}), INSUFFICIENT, True),
        ("stale_coverage_cut_is_insufficiency", lambda x: x["basis"]["coverage"].update({"end": "2026-10-03T20:00:00Z", "cut": "2026-10-03T20:00:00Z"}), INSUFFICIENT, True),
        ("freshness_profile_commitment_mutation", lambda x: x["basis"]["coverage"].update({"freshness_profile_cut": "2026-10-03T22:59:59Z"}), REJECT, True),
        ("receipt_staleness", lambda x: x["basis"]["coverage"].update({"receipt_fresh_until": "2026-10-03T20:00:00Z"}), REJECT, True),
        ("unsupported_proof", lambda x: x["basis"]["coverage"].update({"proof_supported": False}), INSUFFICIENT, True),
        ("local_empty_query", lambda x: x["basis"]["coverage"].update({"local_query_only": True}), INSUFFICIENT, True),
        ("freshness_profile_binding_removal", lambda x: x["basis"]["coverage"].pop("freshness_profile_cut"), REJECT, False),
    ]

    for name, mutate, want, needs_reseal in probes:
        candidate = clone(seed_closed)
        mutate(candidate)
        if needs_reseal:
            reseal(candidate)
        got = primary_qualify(candidate)
        assert got == want, (name, got, want)
        metamorphic.append({"probe": name, "disposition": got})

    terminal_probes = [
        ("affirmed_to_changed", lambda x: x["basis"]["appeal"].update({"disposition": "CHANGED"}), INSUFFICIENT),
        ("resolution_before_filing", lambda x: x["basis"]["appeal"].update({"resolved_at": "2026-10-03T07:00:00Z"}), REJECT),
        ("future_resolution", lambda x: x["basis"]["appeal"].update({"future_relative_to_qualification": True}), REJECT),
    ]
    for name, mutate, want in terminal_probes:
        candidate = clone(seed_terminal)
        mutate(candidate)
        reseal(candidate)
        got = primary_qualify(candidate)
        assert got == want, (name, got, want)
        metamorphic.append({"probe": name, "disposition": got})

    receipt = {
        "program": PROGRAM,
        "schema": SCHEMA,
        "census": census,
        "cases": [
            {
                "id": case_id,
                "disposition": primary[case_id],
                "decision_identity": candidates[case_id]["decision"]["identity_digest"],
                "basis_identity": candidates[case_id]["basis"]["identity_digest"],
            }
            for case_id in [f"F-{i:02d}" for i in range(1, 43)]
        ],
    }

    print("SYM-CIVIC-013 DERIVED=" + canon(census))
    print("SYM-CIVIC-013 METAMORPHIC=" + canon(metamorphic))
    print("SYM-CIVIC-013 GENERATED_RECEIPT=" + digest(receipt))
    print("SYM-CIVIC-013 PASS")


if __name__ == "__main__":
    main()
