#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Independent Python-stdlib verifier for EPI-012T investigation capsule v0.1.

The verifier reads the frozen JSON fixture, validates its internal record semantics,
runs hostile in-memory mutations, and emits a deterministic JSON receipt.

PASS here is corpus verification only. It does not qualify the Rust EPI-012A crate,
Symthaea reasoning, any EPI admission, or any collection/action authority.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any, Callable

VERIFIER_PROFILE = "mycelix:epi-012t:python-stdlib-verifier:v1"
EXPECTED_GIT_BLOB_SHA1 = "0d11fb28ce351ae58bc026629d82bd03c6edf197"
EXPECTED_SCHEMA = "mycelix:epi:investigation-capsule:v0.1"
EXPECTED_PROFILE = "mycelix:epi-012:synthetic-reservoir:v0.1"
EXPECTED_CAPSULE_ID = "capsule:synthetic-reservoir:001"

EXPECTED_INVARIANTS = {
    "F2 does not rewrite F1",
    "invalidated assumption remains historical evidence",
    "Symthaea candidate remains candidate until explicit EPI admission",
    "UnknownCoverage zero-result does not establish absence",
    "finite-corpus absence remains corpus-scoped",
    "planner Pareto front does not grant collection authority",
    "blocked planner proposal remains blocked in capsule history",
    "domination witness is profile-relative historical analysis",
    "capsule integrity does not imply source truth",
    "capsule presence cannot reconstruct search/network/action authority",
    "protected omission is explicit rather than inferred as negative evidence",
    "rendering projection does not upgrade semantic authority",
}


class VerificationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)


def git_blob_sha1(raw: bytes) -> str:
    framed = b"blob " + str(len(raw)).encode("ascii") + b"\0" + raw
    return hashlib.sha1(framed).hexdigest()


def unique_by(records: list[dict[str, Any]], key: str, label: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for record in records:
        value = record.get(key)
        require(isinstance(value, str) and value, f"{label}: invalid {key}")
        require(value not in result, f"{label}: duplicate {key} {value}")
        result[value] = record
    return result


def exact_set(values: Any, expected: set[str], label: str) -> None:
    require(isinstance(values, list), f"{label} must be a list")
    require(len(values) == len(set(values)), f"{label} contains duplicates")
    require(set(values) == expected, f"{label} differs from frozen vector")


def validate_document(document: dict[str, Any]) -> dict[str, Any]:
    require(document.get("schema") == EXPECTED_SCHEMA, "schema mismatch")
    require(document.get("profile") == EXPECTED_PROFILE, "profile mismatch")
    require(document.get("capsule_id") == EXPECTED_CAPSULE_ID, "capsule id mismatch")
    require(document.get("authority") == "RecordOnly", "authority mismatch")
    require(document.get("reasoning_authority") is False, "reasoning authority must be false")
    require(document.get("collection_authority") is False, "collection authority must be false")
    require(document.get("action_authority") is False, "action authority must be false")

    artifacts = unique_by(document.get("artifacts", []), "id", "artifact")
    require(set(artifacts) == {"AR1", "AR2", "AR3", "AR4"}, "artifact set mismatch")

    frontiers = unique_by(document.get("frontiers", []), "id", "frontier")
    require(set(frontiers) == {"F1", "F2"}, "frontier set mismatch")
    f1 = frontiers["F1"]
    f2 = frontiers["F2"]
    require(f1.get("prior") is None, "F1 must have no prior")
    require(f2.get("prior") == "F1", "F2 prior must be F1")
    exact_set(f1.get("artifact_refs"), {"AR1", "AR2", "AR3"}, "F1 artifact refs")
    exact_set(f1.get("assumption_assessment_refs"), {"ASSUMP1:F1"}, "F1 assumption refs")
    exact_set(f1.get("candidate_bundle_refs"), {"SYMCAND:F1"}, "F1 candidate refs")
    exact_set(f1.get("dependency_assessment_refs"), set(), "F1 dependency refs")
    exact_set(f1.get("planner_trace_refs"), set(), "F1 planner refs")
    exact_set(f1.get("search_refs"), set(), "F1 search refs")

    exact_set(f2.get("artifact_refs"), {"AR1", "AR2", "AR3", "AR4"}, "F2 artifact refs")
    exact_set(f2.get("assumption_assessment_refs"), {"ASSUMP1:F2"}, "F2 assumption refs")
    exact_set(f2.get("candidate_bundle_refs"), {"SYMCAND:F2"}, "F2 candidate refs")
    exact_set(f2.get("dependency_assessment_refs"), {"DEP:G1"}, "F2 dependency refs")
    exact_set(f2.get("planner_trace_refs"), {"SYMPLAN:F2"}, "F2 planner refs")
    exact_set(f2.get("search_refs"), {"S1", "S2"}, "F2 search refs")

    assumptions = unique_by(document.get("assumption_ledger", []), "id", "assumption")
    require(set(assumptions) == {"ASSUMP1:F1", "ASSUMP1:F2"}, "assumption set mismatch")
    a1 = assumptions["ASSUMP1:F1"]
    a2 = assumptions["ASSUMP1:F2"]
    require(a1.get("assumption_id") == "ASSUMP1", "F1 assumption id mismatch")
    require(a1.get("frontier_ref") == "F1", "F1 assumption frontier mismatch")
    require(a1.get("status") == "DeclaredWorkingAssumption", "F1 assumption status mismatch")
    require(a1.get("supersedes") is None, "F1 assumption must not supersede")
    require(a2.get("assumption_id") == "ASSUMP1", "F2 assumption id mismatch")
    require(a2.get("frontier_ref") == "F2", "F2 assumption frontier mismatch")
    require(a2.get("status") == "InvalidatedWithinProfile", "F2 assumption status mismatch")
    require(a2.get("supersedes") == "ASSUMP1:F1", "F2 assumption supersession mismatch")
    exact_set(a2.get("contradicting_evidence_refs"), {"DEP:G1"}, "F2 contradicting evidence refs")

    dependencies = unique_by(document.get("dependency_assessments", []), "id", "dependency")
    require(set(dependencies) == {"DEP:G1"}, "dependency set mismatch")
    dep = dependencies["DEP:G1"]
    require(dep.get("frontier_ref") == "F2", "dependency frontier mismatch")
    exact_set(dep.get("artifact_refs"), {"AR1", "AR2", "AR3"}, "DEP:G1 artifact refs")
    require(dep.get("state") == "ObservedSharedLineageGroup", "dependency state mismatch")
    require(dep.get("forbidden_inference") == "IndependentCorroboration", "dependency forbidden inference mismatch")
    require(dep.get("scope_profile_ref") == "dependency:synthetic-explicit-lineage:v1", "dependency scope mismatch")

    searches = unique_by(document.get("searches", []), "id", "search")
    require(set(searches) == {"S1", "S2"}, "search set mismatch")
    s1 = searches["S1"]
    require(s1.get("frontier_ref") == "F2", "S1 frontier mismatch")
    require(s1.get("result_count") == 0, "S1 result count mismatch")
    require(s1.get("coverage") == "UnknownCoverage", "S1 coverage mismatch")
    require(s1.get("finding") == "NoMatchObservedUnderSearchProfile", "S1 finding mismatch")
    require(s1.get("permitted_interpretation") == "UnresolvedDueToUnknownCoverage", "S1 interpretation mismatch")
    require(s1.get("forbidden_interpretation") == "AbsentFromWorld", "S1 forbidden interpretation mismatch")
    require("finite_corpus_commitment" not in s1, "S1 must not carry finite corpus commitment")

    s2 = searches["S2"]
    require(s2.get("frontier_ref") == "F2", "S2 frontier mismatch")
    require(s2.get("result_count") == 0, "S2 result count mismatch")
    require(s2.get("coverage") == "ExhaustiveWithinDeclaredFiniteCorpus", "S2 coverage mismatch")
    require(s2.get("finding") == "AbsentFromExactFiniteCorpusCommitment", "S2 finding mismatch")
    require(s2.get("permitted_interpretation") == "AbsentWithinExactFiniteCorpus", "S2 interpretation mismatch")
    require(s2.get("forbidden_interpretation") == "DidNotOccurInWorld", "S2 forbidden interpretation mismatch")
    require(s2.get("finite_corpus_commitment") == "fixture:manual-override-corpus:v1", "S2 corpus commitment mismatch")

    candidates = unique_by(document.get("symthaea_candidates", []), "id", "candidate")
    require(set(candidates) == {"SYMCAND:F1", "SYMCAND:F2"}, "candidate set mismatch")
    for candidate_id, frontier_id in (("SYMCAND:F1", "F1"), ("SYMCAND:F2", "F2")):
        candidate = candidates[candidate_id]
        require(candidate.get("frontier_ref") == frontier_id, f"{candidate_id} frontier mismatch")
        require(candidate.get("status") == "CandidateUnadmitted", f"{candidate_id} status mismatch")
        external = candidate.get("external_subject")
        require(isinstance(external, dict), f"{candidate_id} missing external subject")
        require(external.get("repo") == "Luminous-Dynamics/symthaea", f"{candidate_id} repo mismatch")
        require(external.get("subject_kind") == "InvestigationCandidateBundle", f"{candidate_id} subject kind mismatch")

    planners = unique_by(document.get("planner_traces", []), "id", "planner")
    require(set(planners) == {"SYMPLAN:F2"}, "planner set mismatch")
    planner = planners["SYMPLAN:F2"]
    require(planner.get("frontier_ref") == "F2", "planner frontier mismatch")
    require(planner.get("preferred_hypothesis_ref") == "H1", "planner preferred hypothesis mismatch")
    require(planner.get("status") == "CandidateAnalysisOnly", "planner status mismatch")
    require(planner.get("execution_authority") is False, "planner execution authority must be false")
    external = planner.get("external_subject")
    require(isinstance(external, dict), "planner external subject missing")
    require(external.get("repo") == "Luminous-Dynamics/symthaea", "planner repo mismatch")
    require(external.get("head") == "7b1bfbc376e3efe7ed197bd846c0d2d3ec2b8da5", "planner head mismatch")
    require(external.get("fixture_blob") == "09331ce91386f2151e3681f66eb0c341f04aac89", "planner fixture mismatch")
    require(external.get("profile") == "symthaea:next-information:pareto-front:v1", "planner profile mismatch")
    exact_set(planner.get("disconfirmation_candidate_refs"), {"D1", "D2", "D4"}, "planner disconfirmation set")
    exact_set(planner.get("eligible_pareto_front_refs"), {"D1", "D2", "D3"}, "planner Pareto set")
    exact_set(planner.get("planning_limitations"), {"PriorFalsifierSearchUnknownCoverage"}, "planner limitations")

    blocked = planner.get("blocked_but_analytically_useful")
    require(isinstance(blocked, list) and len(blocked) == 1, "planner blocked set mismatch")
    require(blocked[0] == {"proposal_ref": "D4", "reason": "PrivacyBlocked"}, "D4 block mismatch")

    dominated = planner.get("dominated_candidates")
    require(isinstance(dominated, list) and len(dominated) == 1, "planner dominated set mismatch")
    require(
        dominated[0] == {
            "proposal_ref": "D5",
            "dominated_by": "D2",
            "witness_profile": "symthaea:next-information:pareto-front:v1",
        },
        "D5 domination witness mismatch",
    )

    omissions = unique_by(document.get("protected_omissions", []), "id", "omission")
    require(set(omissions) == {"OMIT1"}, "omission set mismatch")
    omit = omissions["OMIT1"]
    require(omit.get("subject_ref") == "planner-proposal:D4", "omission subject mismatch")
    require(omit.get("reason") == "ProtectedInformationNotEmbedded", "omission reason mismatch")
    require(omit.get("commitment_ref") == "protected:synthetic-d4-detail", "omission commitment mismatch")
    require(omit.get("raw_content_embedded") is False, "protected raw content must not be embedded")

    presentation = document.get("presentation_projection")
    require(isinstance(presentation, dict), "presentation projection missing")
    require(presentation.get("id") == "ATLAS:F2", "presentation id mismatch")
    require(presentation.get("frontier_ref") == "F2", "presentation frontier mismatch")
    require(presentation.get("capsule_ref") == EXPECTED_CAPSULE_ID, "presentation capsule mismatch")
    require(presentation.get("authority") == "RenderingOnly", "presentation authority mismatch")

    invariants = document.get("required_invariants")
    require(isinstance(invariants, list), "required_invariants must be a list")
    require(len(invariants) == len(set(invariants)), "duplicate required invariant")
    require(set(invariants) == EXPECTED_INVARIANTS, "required invariant set mismatch")

    # Cross-reference integrity.
    artifact_ids = set(artifacts)
    assumption_ids = set(assumptions)
    dependency_ids = set(dependencies)
    search_ids = set(searches)
    candidate_ids = set(candidates)
    planner_ids = set(planners)
    for frontier in frontiers.values():
        require(set(frontier.get("artifact_refs", [])).issubset(artifact_ids), "frontier has unknown artifact ref")
        require(set(frontier.get("assumption_assessment_refs", [])).issubset(assumption_ids), "frontier has unknown assumption ref")
        require(set(frontier.get("dependency_assessment_refs", [])).issubset(dependency_ids), "frontier has unknown dependency ref")
        require(set(frontier.get("search_refs", [])).issubset(search_ids), "frontier has unknown search ref")
        require(set(frontier.get("candidate_bundle_refs", [])).issubset(candidate_ids), "frontier has unknown candidate ref")
        require(set(frontier.get("planner_trace_refs", [])).issubset(planner_ids), "frontier has unknown planner ref")

    require(set(dep.get("artifact_refs", [])).issubset(artifact_ids), "dependency has unknown artifact")
    require(a2.get("supersedes") in assumption_ids, "assumption supersession is dangling")
    require(set(a2.get("contradicting_evidence_refs", [])).issubset(dependency_ids), "assumption contradicting evidence is dangling")

    return {
        "artifact_count": len(artifacts),
        "frontier_count": len(frontiers),
        "assumption_count": len(assumptions),
        "dependency_count": len(dependencies),
        "search_count": len(searches),
        "candidate_count": len(candidates),
        "planner_count": len(planners),
        "omission_count": len(omissions),
        "invariant_count": len(invariants),
    }


def expect_rejected(name: str, original: dict[str, Any], mutate: Callable[[dict[str, Any]], None]) -> dict[str, Any]:
    candidate = copy.deepcopy(original)
    mutate(candidate)
    try:
        validate_document(candidate)
    except VerificationError as exc:
        return {"name": name, "rejected": True, "reason_class": type(exc).__name__}
    raise VerificationError(f"hostile mutation unexpectedly accepted: {name}")


def hostile_mutations(document: dict[str, Any]) -> list[dict[str, Any]]:
    def frontier(candidate: dict[str, Any], frontier_id: str) -> dict[str, Any]:
        return next(item for item in candidate["frontiers"] if item["id"] == frontier_id)

    def assumption(candidate: dict[str, Any], assumption_id: str) -> dict[str, Any]:
        return next(item for item in candidate["assumption_ledger"] if item["id"] == assumption_id)

    def dependency(candidate: dict[str, Any]) -> dict[str, Any]:
        return next(item for item in candidate["dependency_assessments"] if item["id"] == "DEP:G1")

    def search(candidate: dict[str, Any], search_id: str) -> dict[str, Any]:
        return next(item for item in candidate["searches"] if item["id"] == search_id)

    def planner(candidate: dict[str, Any]) -> dict[str, Any]:
        return next(item for item in candidate["planner_traces"] if item["id"] == "SYMPLAN:F2")

    def rewrite_f1(candidate: dict[str, Any]) -> None:
        frontier(candidate, "F1")["artifact_refs"].append("AR4")

    def delete_f1_assumption(candidate: dict[str, Any]) -> None:
        candidate["assumption_ledger"] = [item for item in candidate["assumption_ledger"] if item["id"] != "ASSUMP1:F1"]

    def alter_dependency_membership(candidate: dict[str, Any]) -> None:
        dependency(candidate)["artifact_refs"] = ["AR1", "AR2"]

    def upgrade_s1_to_finite_absence(candidate: dict[str, Any]) -> None:
        s1 = search(candidate, "S1")
        s1["coverage"] = "ExhaustiveWithinDeclaredFiniteCorpus"
        s1["finding"] = "AbsentFromExactFiniteCorpusCommitment"
        s1["permitted_interpretation"] = "AbsentWithinExactFiniteCorpus"

    def promote_candidate(candidate: dict[str, Any]) -> None:
        next(item for item in candidate["symthaea_candidates"] if item["id"] == "SYMCAND:F2")["status"] = "Admitted"

    def remove_d4_block(candidate: dict[str, Any]) -> None:
        planner(candidate)["blocked_but_analytically_useful"] = []

    def change_planner_profile(candidate: dict[str, Any]) -> None:
        planner(candidate)["external_subject"]["profile"] = "symthaea:next-information:pareto-front:v2"

    def enable_planner_execution(candidate: dict[str, Any]) -> None:
        planner(candidate)["execution_authority"] = True

    def embed_protected_content(candidate: dict[str, Any]) -> None:
        candidate["protected_omissions"][0]["raw_content_embedded"] = True
        candidate["protected_omissions"][0]["raw_content"] = "forbidden"

    def upgrade_presentation(candidate: dict[str, Any]) -> None:
        candidate["presentation_projection"]["authority"] = "SemanticAuthority"

    return [
        expect_rejected("rewrite-f1-with-f2-evidence", document, rewrite_f1),
        expect_rejected("delete-superseded-assumption", document, delete_f1_assumption),
        expect_rejected("alter-dependency-membership", document, alter_dependency_membership),
        expect_rejected("upgrade-s1-without-corpus-commitment", document, upgrade_s1_to_finite_absence),
        expect_rejected("promote-candidate-without-admission", document, promote_candidate),
        expect_rejected("remove-d4-privacy-block", document, remove_d4_block),
        expect_rejected("change-planner-profile-with-stale-witness", document, change_planner_profile),
        expect_rejected("enable-planner-execution-authority", document, enable_planner_execution),
        expect_rejected("embed-protected-content", document, embed_protected_content),
        expect_rejected("upgrade-atlas-authority", document, upgrade_presentation),
    ]


def default_fixture_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "docs"
        / "architecture"
        / "fixtures"
        / "EPI_012_INVESTIGATION_CAPSULE_V0_1.json"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=default_fixture_path())
    args = parser.parse_args()

    raw = args.fixture.read_bytes()
    require(git_blob_sha1(raw) == EXPECTED_GIT_BLOB_SHA1, "fixture Git blob identity mismatch")

    try:
        document = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VerificationError("fixture is not valid UTF-8 JSON") from exc

    summary = validate_document(document)
    mutation_results = hostile_mutations(document)
    require(all(item["rejected"] for item in mutation_results), "one or more hostile mutations were accepted")

    receipt = {
        "status": "PASS",
        "verifier_profile": VERIFIER_PROFILE,
        "fixture": {
            "path": str(args.fixture),
            "byte_length": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "git_blob_sha1": git_blob_sha1(raw),
        },
        "authority": {
            "scope": "RecordOnly",
            "reasoning_authority": False,
            "collection_authority": False,
            "action_authority": False,
        },
        "summary": summary,
        "hostile_mutations": mutation_results,
        "nonclaims": [
            "EPI-012A Rust implementation qualified",
            "Symthaea reasoning qualified",
            "EPI admission authorized",
            "collection authorized",
            "action authorized",
            "factual truth established",
        ],
    }
    print(json.dumps(receipt, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except VerificationError as exc:
        print(json.dumps({"status": "FAIL", "verifier_profile": VERIFIER_PROFILE, "error": str(exc)}, sort_keys=True, separators=(",", ":")))
        raise SystemExit(1)
