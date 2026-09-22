#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Independent verifier for EPI-INTEG-001M investigation identity-map v0.1.

This verifier intentionally uses Python stdlib only and does not import Mycelix Rust code.
It validates the frozen JSON corpus and emits a deterministic JSON receipt to stdout.

A successful run establishes only internal consistency of the exact fixture corpus.
It does not qualify the Rust identity-map implementation, the Symthaea loop, any bridge
transport, EPI admission, factual truth, collection authority, or execution authority.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any, Callable

VERIFIER_PROFILE = "mycelix:epi-integ-001m:python-stdlib-verifier:v1"
EXPECTED_SCHEMA = "mycelix:epi-integ:investigation-identity-map:v0.1"
EXPECTED_PROFILE = "mycelix:symthaea:investigation-identity-map:v0.1"
EXPECTED_FIXTURE_GIT_BLOB_SHA1 = "9e72ea4fe6f0ed04001c69a12a290e20efb567c8"

EXPECTED_SOURCE_SUBJECT = {
    "repo": "Luminous-Dynamics/symthaea",
    "head": "90267527cba13a1e0be8c424ebce03f1f5726e46",
    "profile": "symthaea:closed-world-investigation-loop:v1",
}
EXPECTED_DESTINATION_SUBJECT = {
    "repo": "Luminous-Dynamics/mycelix",
    "head": "59dcabafd28b2020f09abe4544d731ab720143d1",
    "profile": "mycelix:epi:investigation-capsule:v1",
}

# source role, source local id, destination role, destination local id, binding profile
EXPECTED_CORRESPONDENCES = {
    ("ArtifactProjectionRef", "artifact:A1", "ArtifactRecordV1", "AR1", "exact-fixture-correspondence:v1"),
    ("ArtifactProjectionRef", "artifact:A2", "ArtifactRecordV1", "AR2", "exact-fixture-correspondence:v1"),
    ("ArtifactProjectionRef", "artifact:A3", "ArtifactRecordV1", "AR3", "exact-fixture-correspondence:v1"),
    ("FrontierRef", "F2", "FrontierRecordV1", "F2", "exact-fixture-correspondence:v1"),
    ("InformationProposalId", "D1", "PlannerHistoryProposalRef", "D1", "exact-fixture-correspondence:v1"),
    ("InformationProposalId", "D2", "PlannerHistoryProposalRef", "D2", "exact-fixture-correspondence:v1"),
    ("InformationProposalId", "D3", "PlannerHistoryProposalRef", "D3", "exact-fixture-correspondence:v1"),
    ("InformationProposalId", "D4", "PlannerHistoryProposalRef", "D4", "exact-fixture-correspondence:v1"),
    ("InformationProposalId", "D5", "PlannerHistoryProposalRef", "D5", "exact-fixture-correspondence:v1"),
    ("ToolProfileRef", "T_WEB_PUBLIC_TOPK", "MethodologySelectedProfileRef", "T_WEB_PUBLIC_TOPK", "exact-fixture-correspondence:v1"),
    (
        "ClosedWorldInvestigationRecordV1",
        "symthaea:closed-world-investigation-loop:v1",
        "ExternalCandidateRecordV1",
        "SYMCAND:F2",
        "external-subject-recording:v1",
    ),
}

EXPECTED_LOCAL_ONLY = {
    ("FrontierRecordV1", "F1", "MycelixLocalOnly"),
    ("ArtifactRecordV1", "AR4", "MycelixLocalOnly"),
    ("AssumptionAssessmentV1", "ASSUMP1:F1", "MycelixLocalOnly"),
    ("AssumptionAssessmentV1", "ASSUMP1:F2", "MycelixLocalOnly"),
    ("DependencyGroupV1", "DEP:G1", "MycelixLocalOnly"),
    ("PresentationProjectionV1", "ATLAS:F2", "MycelixLocalOnly"),
}

EXPECTED_NO_EXPORT = {
    ("ProtectedOmissionV1", "OMIT1", "NoExportByPolicy", False),
}

EXPECTED_INVARIANTS = {
    "same spelling is not sufficient for cross-repository identity",
    "every cross-repository correspondence is role-scoped",
    "every correspondence independently carries exact source and destination subject identity",
    "unmapped identities do not match by textual coincidence",
    "Mycelix-local historical state does not require an invented Symthaea counterpart",
    "protected omission is not exported as raw content",
    "fixture correspondence does not grant EPI admission",
    "fixture correspondence does not grant collection authority",
    "fixture correspondence does not grant execution authority",
}

BINDING_KEYS = {
    "binding_id",
    "binding_profile",
    "source_repo",
    "source_subject_head",
    "source_subject_profile",
    "source_role",
    "source_local_id",
    "destination_repo",
    "destination_subject_head",
    "destination_subject_profile",
    "destination_role",
    "destination_local_id",
    "binding_kind",
    "one_to_one",
}


class VerificationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)


def require_exact_keys(record: dict[str, Any], required: set[str], context: str) -> None:
    missing = required.difference(record)
    require(not missing, f"{context}: missing keys: {sorted(missing)}")


def git_blob_sha1(raw: bytes) -> str:
    framed = b"blob " + str(len(raw)).encode("ascii") + b"\0" + raw
    return hashlib.sha1(framed).hexdigest()


def source_key(binding: dict[str, Any]) -> tuple[str, str, str, str, str]:
    return (
        binding["source_repo"],
        binding["source_subject_head"],
        binding["source_subject_profile"],
        binding["source_role"],
        binding["source_local_id"],
    )


def destination_key(binding: dict[str, Any]) -> tuple[str, str, str, str, str]:
    return (
        binding["destination_repo"],
        binding["destination_subject_head"],
        binding["destination_subject_profile"],
        binding["destination_role"],
        binding["destination_local_id"],
    )


def binding_by_id(document: dict[str, Any], binding_id: str) -> dict[str, Any]:
    found = [b for b in document["correspondences"] if b.get("binding_id") == binding_id]
    require(len(found) == 1, f"expected exactly one binding {binding_id}")
    return found[0]


def validate_document(document: dict[str, Any]) -> dict[str, Any]:
    require(document.get("schema") == EXPECTED_SCHEMA, "schema mismatch")
    require(document.get("profile") == EXPECTED_PROFILE, "profile mismatch")
    require(document.get("authority") == "FixtureCorrespondenceOnly", "authority mismatch")
    require(document.get("admission_authority") is False, "admission authority must be false")
    require(document.get("collection_authority") is False, "collection authority must be false")
    require(document.get("execution_authority") is False, "execution authority must be false")
    require(document.get("source_subject") == EXPECTED_SOURCE_SUBJECT, "source subject mismatch")
    require(document.get("destination_subject") == EXPECTED_DESTINATION_SUBJECT, "destination subject mismatch")

    correspondences = document.get("correspondences")
    require(isinstance(correspondences, list), "correspondences must be a list")
    require(len(correspondences) == 11, "unexpected correspondence cardinality")

    binding_ids: set[str] = set()
    source_endpoints: set[tuple[str, str, str, str, str]] = set()
    destination_endpoints: set[tuple[str, str, str, str, str]] = set()
    actual_correspondences: set[tuple[str, str, str, str, str]] = set()

    for index, binding in enumerate(correspondences):
        require(isinstance(binding, dict), f"binding[{index}] must be an object")
        require_exact_keys(binding, BINDING_KEYS, f"binding[{index}]")
        require(binding["binding_kind"] == "CrossRepoCorrespondence", f"binding[{index}] kind mismatch")
        require(binding["one_to_one"] is True, f"binding[{index}] must be one-to-one")
        require(binding["source_repo"] == EXPECTED_SOURCE_SUBJECT["repo"], f"binding[{index}] source repo mismatch")
        require(binding["source_subject_head"] == EXPECTED_SOURCE_SUBJECT["head"], f"binding[{index}] source head mismatch")
        require(binding["source_subject_profile"] == EXPECTED_SOURCE_SUBJECT["profile"], f"binding[{index}] source profile mismatch")
        require(binding["destination_repo"] == EXPECTED_DESTINATION_SUBJECT["repo"], f"binding[{index}] destination repo mismatch")
        require(binding["destination_subject_head"] == EXPECTED_DESTINATION_SUBJECT["head"], f"binding[{index}] destination head mismatch")
        require(binding["destination_subject_profile"] == EXPECTED_DESTINATION_SUBJECT["profile"], f"binding[{index}] destination profile mismatch")

        binding_id = binding["binding_id"]
        require(binding_id not in binding_ids, f"duplicate binding id: {binding_id}")
        binding_ids.add(binding_id)

        src = source_key(binding)
        dst = destination_key(binding)
        require(src not in source_endpoints, f"conflicting/duplicate source endpoint in {binding_id}")
        require(dst not in destination_endpoints, f"conflicting/duplicate destination endpoint in {binding_id}")
        source_endpoints.add(src)
        destination_endpoints.add(dst)

        actual_correspondences.add(
            (
                binding["source_role"],
                binding["source_local_id"],
                binding["destination_role"],
                binding["destination_local_id"],
                binding["binding_profile"],
            )
        )

    require(actual_correspondences == EXPECTED_CORRESPONDENCES, "correspondence set differs from frozen vector")

    local_only_records = document.get("destination_local_only")
    require(isinstance(local_only_records, list), "destination_local_only must be a list")
    actual_local_only = {
        (record.get("role"), record.get("local_id"), record.get("classification"))
        for record in local_only_records
    }
    require(len(actual_local_only) == len(local_only_records), "duplicate local-only record")
    require(actual_local_only == EXPECTED_LOCAL_ONLY, "local-only set differs from frozen vector")

    no_export_records = document.get("no_export_by_policy")
    require(isinstance(no_export_records, list), "no_export_by_policy must be a list")
    actual_no_export = {
        (
            record.get("role"),
            record.get("local_id"),
            record.get("classification"),
            record.get("raw_content_mapped"),
        )
        for record in no_export_records
    }
    require(len(actual_no_export) == len(no_export_records), "duplicate no-export record")
    require(actual_no_export == EXPECTED_NO_EXPORT, "no-export set differs from frozen vector")

    mapped_destination_role_ids = {
        (binding["destination_role"], binding["destination_local_id"])
        for binding in correspondences
    }
    local_only_role_ids = {(role, local_id) for role, local_id, _ in actual_local_only}
    no_export_role_ids = {(role, local_id) for role, local_id, _, _ in actual_no_export}
    require(mapped_destination_role_ids.isdisjoint(local_only_role_ids), "mapped destination overlaps local-only identity")
    require(mapped_destination_role_ids.isdisjoint(no_export_role_ids), "mapped destination overlaps no-export identity")
    require(local_only_role_ids.isdisjoint(no_export_role_ids), "local-only identity overlaps no-export identity")

    invariants = document.get("required_invariants")
    require(isinstance(invariants, list), "required_invariants must be a list")
    actual_invariants = set(invariants)
    require(len(actual_invariants) == len(invariants), "duplicate required invariant")
    require(actual_invariants == EXPECTED_INVARIANTS, "required invariant set differs from frozen vector")

    return {
        "correspondence_count": len(correspondences),
        "local_only_count": len(local_only_records),
        "no_export_count": len(no_export_records),
        "invariant_count": len(invariants),
    }


def expect_rejected(
    name: str,
    original: dict[str, Any],
    mutate: Callable[[dict[str, Any]], None],
) -> dict[str, Any]:
    candidate = copy.deepcopy(original)
    mutate(candidate)
    try:
        validate_document(candidate)
    except VerificationError as exc:
        return {"name": name, "rejected": True, "reason_class": type(exc).__name__}
    raise VerificationError(f"hostile mutation unexpectedly accepted: {name}")


def hostile_mutations(document: dict[str, Any]) -> list[dict[str, Any]]:
    def change_a1_destination(candidate: dict[str, Any]) -> None:
        binding_by_id(candidate, "MAP:ART:A1")["destination_local_id"] = "AR2"

    def duplicate_source(candidate: dict[str, Any]) -> None:
        duplicate = copy.deepcopy(binding_by_id(candidate, "MAP:ART:A1"))
        duplicate["binding_id"] = "MUT:DUP-SOURCE"
        duplicate["destination_local_id"] = "ARX"
        candidate["correspondences"].append(duplicate)

    def duplicate_destination(candidate: dict[str, Any]) -> None:
        duplicate = copy.deepcopy(binding_by_id(candidate, "MAP:ART:A1"))
        duplicate["binding_id"] = "MUT:DUP-DESTINATION"
        duplicate["source_local_id"] = "artifact:AX"
        candidate["correspondences"].append(duplicate)

    def substitute_f2_role(candidate: dict[str, Any]) -> None:
        binding_by_id(candidate, "MAP:FRONTIER:F2")["source_role"] = "InformationProposalId"

    def substitute_source_repo(candidate: dict[str, Any]) -> None:
        binding_by_id(candidate, "MAP:ART:A1")["source_repo"] = "Luminous-Dynamics/not-symthaea"

    def substitute_source_head(candidate: dict[str, Any]) -> None:
        binding_by_id(candidate, "MAP:ART:A1")["source_subject_head"] = "0" * 40

    def substitute_source_profile(candidate: dict[str, Any]) -> None:
        binding_by_id(candidate, "MAP:ART:A1")["source_subject_profile"] = "symthaea:wrong-profile:v1"

    def remove_binding_head(candidate: dict[str, Any]) -> None:
        del binding_by_id(candidate, "MAP:ART:A1")["source_subject_head"]

    def export_protected_raw(candidate: dict[str, Any]) -> None:
        candidate["no_export_by_policy"][0]["raw_content_mapped"] = True

    def enable_admission(candidate: dict[str, Any]) -> None:
        candidate["admission_authority"] = True

    return [
        expect_rejected("artifact-a1-remapped-to-ar2", document, change_a1_destination),
        expect_rejected("duplicate-source-endpoint", document, duplicate_source),
        expect_rejected("duplicate-destination-endpoint", document, duplicate_destination),
        expect_rejected("f2-role-substitution", document, substitute_f2_role),
        expect_rejected("source-repository-substitution", document, substitute_source_repo),
        expect_rejected("source-head-substitution", document, substitute_source_head),
        expect_rejected("source-profile-substitution", document, substitute_source_profile),
        expect_rejected("per-binding-subject-coordinate-removed", document, remove_binding_head),
        expect_rejected("protected-raw-content-mapped", document, export_protected_raw),
        expect_rejected("admission-authority-enabled", document, enable_admission),
    ]


def default_fixture_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "docs"
        / "architecture"
        / "fixtures"
        / "EPI_INTEG_001M_INVESTIGATION_IDENTITY_MAP_V0_1.json"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=default_fixture_path())
    args = parser.parse_args()

    raw = args.fixture.read_bytes()
    require(git_blob_sha1(raw) == EXPECTED_FIXTURE_GIT_BLOB_SHA1, "fixture Git blob identity mismatch")

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
        "source_subject": EXPECTED_SOURCE_SUBJECT,
        "destination_subject": EXPECTED_DESTINATION_SUBJECT,
        "authority": {
            "scope": "FixtureCorrespondenceOnly",
            "admission_authority": False,
            "collection_authority": False,
            "execution_authority": False,
        },
        "summary": summary,
        "hostile_mutations": mutation_results,
        "nonclaims": [
            "Rust identity-map implementation qualified",
            "cross-repository bridge qualified",
            "EPI admission authorized",
            "collection authorized",
            "execution authorized",
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
