#!/usr/bin/env python3
"""Validate MYC-CONST-003CR2 cross-branch refinement evidence using stdlib + Git."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import re
import subprocess
import sys
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parents[2]
REPO = ROOT.parent
MANIFEST = ROOT / "specs/constitutional-refinement-crosswalk.v2.json"
SCHEMA = ROOT / "specs/constitutional-refinement-crosswalk.v2.schema.json"

EXPECTED_PARENT = {
    "semantic_head": "4edb56bd3c36ddc6277e2de5e0584a65e8c99b3c",
    "verifier_head": "651eb0c05fa70012b7eebc4e1497cb7cacf7c641",
    "qualification_run_id": 35319268716,
    "artifact_digest": "sha256:b8bacbf0467e23fe033042691acdeee732f532c5e6774715d26dadaecad415b1",
    "manifest_path": "mycelix-governance/specs/constitutional-refinement-crosswalk.v1.json",
    "manifest_git_blob_sha": "35476cb5b2dbacaca659b117e6dfaaacebed818d",
}

EXPECTED_SUBJECTS = {
    "MYC-CONST-003B2": {
        "semantic_head": "b19667a0399d46e7a21254e931c3149cfbd994fc",
        "verifier_head": "aaf1a97546b7cf33802d363f94567481ebeb8217",
        "qualification_run_id": 35283271597,
        "artifact_digest": "sha256:a0bc243f4c9efc20c2fd7b5482ad30c880c14ae8782da5c8a6a04ad2bf61a4b4",
        "mapping_status": "inherited_qualified",
    },
    "MYC-CONST-003B4": {
        "semantic_head": "037f61c15ff367a1518d98f7acc8ec0fa962c2f6",
        "verifier_head": "508e81ea93bd49692f54b234e20146dfe101afa3",
        "qualification_run_id": 35344734164,
        "artifact_digest": "sha256:d0cf0a406f22cae5d8b7ff9a66a9c3a0610c073e1c84f1cd38d1255367b2c6f2",
        "mapping_status": "candidate",
    },
    "MYC-CONST-003C3": {
        "semantic_head": "b9bb91353788aeb858c4e52422a87e2401d60a0e",
        "verifier_head": "18309d680a5d630bbb7f5443ab103ade9239a557",
        "qualification_run_id": 35314384788,
        "artifact_digest": "sha256:d26e566503fc1223d879d69ecdb4cf6978fc62664576ed04973067393c8feaab",
        "mapping_status": "unmapped",
    },
    "MYC-CONST-003D1A": {
        "semantic_head": "15b9c89adf0ac3c6c5a73681614d6bfcd368820a",
        "verifier_head": "70fbe906834fdef1ba69a80066dbbbaef200156f",
        "qualification_run_id": 35320314578,
        "artifact_digest": "sha256:ce9d7176bd1040ef552fa587e44ae8801a13b0363c0013d5428e5f95ff60430c",
        "mapping_status": "unmapped",
    },
}

EXPECTED_INHERITED_IDS = {
    "lifecycle.absent",
    "lifecycle.pending",
    "lifecycle.blocked",
    "lifecycle.finalized",
    "lifecycle.rejected",
    "lifecycle.revoked_closed",
    "lifecycle.halted",
}

EXPECTED_PENDING = {
    "MYC-CONST-003D1C": (
        "47d1d764323dbfaf991b5574cfde83abb7a3e4a4",
        "c4397f4bc9d6e016feba84d89de9f31298b55ba8",
        35349600389,
    ),
    "MYC-CONST-003D1D-E0": (
        "36a4fcffb6ca806570ebf439f9c36cf76401e7b2",
        "55c9e1d57d4cbfaa807dadefe7d1cac4f261a2e2",
        35357940683,
    ),
    "MYC-CONST-003D1D-E1A": (
        "249753017c3781f7a3259061aae16d7e0075e984",
        "ebe85798557a04c75d0bb9ce6c02349576e12b4f",
        35360097507,
    ),
}

REQUIRED_NONCLAIMS = {
    "not_whole_program_refinement",
    "not_c3_concrete_refinement",
    "not_d1a_runtime_refinement",
    "not_d1c_qualified",
    "not_event_persistence_qualified",
    "not_runtime_crash_consistency_refinement",
    "not_holochain_persistence",
    "not_deployment_currentness_qualified",
    "not_qualified",
}


def fail(message: str) -> None:
    raise ValueError(message)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    require(set(obj) == expected, f"{label} key census drift: {sorted(set(obj) ^ expected)}")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def git_blob(head: str, path: str) -> str:
    return git("rev-parse", f"{head}:{path}")


def git_text(head: str, path: str) -> str:
    return subprocess.check_output(
        ["git", "show", f"{head}:{path}"], cwd=REPO, text=True
    )


def parse_rust_struct_fields(text: str, struct_name: str) -> list[str]:
    match = re.search(
        rf"pub\s+struct\s+{re.escape(struct_name)}\s*\{{(?P<body>.*?)\n\}}",
        text,
        re.DOTALL,
    )
    if not match:
        fail(f"Rust struct not found: {struct_name}")
    fields = re.findall(r"^\s*pub\s+([A-Za-z_][A-Za-z0-9_]*)\s*:", match.group("body"), re.MULTILINE)
    require(bool(fields), f"no fields parsed for Rust struct {struct_name}")
    return fields


def parse_alloy_sig_fields(text: str, sig_name: str) -> list[str]:
    match = re.search(
        rf"(?:abstract\s+)?sig\s+{re.escape(sig_name)}\s*\{{(?P<body>.*?)\n\}}",
        text,
        re.DOTALL,
    )
    if not match:
        fail(f"Alloy sig not found: {sig_name}")
    fields = re.findall(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:", match.group("body"), re.MULTILINE)
    require(bool(fields), f"no fields parsed for Alloy sig {sig_name}")
    return fields


def validate_schema_file() -> None:
    schema = json.loads(SCHEMA.read_text())
    require(schema.get("$schema") == "https://json-schema.org/draft/2020-12/schema", "wrong JSON Schema draft")
    require(schema.get("$id") == "https://mycelix.dev/schemas/constitutional-refinement-crosswalk.v2.schema.json", "wrong schema id")
    require(schema.get("additionalProperties") is False, "root schema must be closed")


def validate_parent(parent: dict[str, Any]) -> None:
    exact_keys(parent, set(EXPECTED_PARENT), "parent_crosswalk")
    require(parent == EXPECTED_PARENT, "parent crosswalk evidence drift")
    actual_blob = git_blob(parent["semantic_head"], parent["manifest_path"])
    require(actual_blob == parent["manifest_git_blob_sha"], f"parent manifest blob mismatch: {actual_blob}")

    parent_manifest = json.loads(git_text(parent["semantic_head"], parent["manifest_path"]))
    parent_ids = {row["id"] for row in parent_manifest["relationships"]}
    require(parent_ids == EXPECTED_INHERITED_IDS, f"parent relationship census drift: {sorted(parent_ids)}")
    for row in parent_manifest["relationships"]:
        require(row.get("qualification_status") == "qualified", f"parent row not qualified: {row.get('id')}")


def validate_subjects(subjects: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    require(len(subjects) == len(EXPECTED_SUBJECTS), "qualified subject census drift")
    by_id: dict[str, dict[str, Any]] = {}
    required = {
        "id", "semantic_head", "verifier_head", "qualification_run_id", "artifact_digest",
        "evidence_status", "mapping_status", "reason",
    }
    for subject in subjects:
        exact_keys(subject, required, f"subject:{subject.get('id')}")
        subject_id = subject["id"]
        require(subject_id in EXPECTED_SUBJECTS, f"unexpected qualified subject: {subject_id}")
        require(subject_id not in by_id, f"duplicate qualified subject: {subject_id}")
        expected = EXPECTED_SUBJECTS[subject_id]
        for key in ("semantic_head", "verifier_head", "qualification_run_id", "artifact_digest", "mapping_status"):
            require(subject[key] == expected[key], f"{subject_id} evidence drift: {key}")
        require(subject["evidence_status"] == "qualified", f"{subject_id} is not qualified evidence")
        require(bool(subject["reason"].strip()), f"{subject_id} reason empty")
        by_id[subject_id] = subject
    require(set(by_id) == set(EXPECTED_SUBJECTS), "qualified subject IDs drift")
    return by_id


def validate_inherited(inherited: dict[str, Any]) -> None:
    exact_keys(
        inherited,
        {"source_subject_id", "source_crosswalk_semantic_head", "relationship_ids"},
        "inherited_relationships",
    )
    require(inherited["source_subject_id"] == "MYC-CONST-003B2", "inherited subject drift")
    require(inherited["source_crosswalk_semantic_head"] == EXPECTED_PARENT["semantic_head"], "inherited crosswalk head drift")
    ids = inherited["relationship_ids"]
    require(len(ids) == len(set(ids)), "duplicate inherited relationship id")
    require(set(ids) == EXPECTED_INHERITED_IDS, "inherited relationship census drift")


def validate_relationships(
    relationships: list[dict[str, Any]],
    subjects: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    require(len(relationships) == 1, "CR2 v0.2 must contain exactly one new candidate relationship")
    row = relationships[0]
    exact_keys(
        row,
        {"id", "subject_id", "mapping_status", "kind", "concrete", "formal", "field_map", "rationale"},
        "relationship",
    )
    require(row["id"] == "claim-binding.semantic-fields", "relationship id drift")
    require(row["subject_id"] == "MYC-CONST-003B4", "relationship subject drift")
    require(row["mapping_status"] == "candidate", "B4 mapping promoted before CR2 qualification")
    require(row["kind"] == "projection", "B4 relationship kind drift")
    require(bool(row["rationale"].strip()), "B4 mapping rationale empty")

    b4 = subjects["MYC-CONST-003B4"]
    concrete = row["concrete"]
    formal = row["formal"]
    exact_keys(concrete, {"semantic_head", "path", "git_blob_sha", "symbol", "metadata_fields"}, "B4 concrete")
    exact_keys(formal, {"semantic_head", "path", "git_blob_sha", "symbol"}, "B4 formal")
    require(concrete["semantic_head"] == b4["semantic_head"], "B4 concrete head not bound to qualified subject")
    require(formal["semantic_head"] == b4["semantic_head"], "B4 formal head not bound to qualified subject")
    require(concrete["symbol"] == "ClaimBinding", "B4 concrete symbol drift")
    require(formal["symbol"] == "Binding", "B4 formal symbol drift")
    require(concrete["metadata_fields"] == ["schema_version"], "B4 metadata field census drift")

    concrete_blob = git_blob(concrete["semantic_head"], concrete["path"])
    formal_blob = git_blob(formal["semantic_head"], formal["path"])
    require(concrete_blob == concrete["git_blob_sha"], f"B4 concrete blob mismatch: {concrete_blob}")
    require(formal_blob == formal["git_blob_sha"], f"B4 formal blob mismatch: {formal_blob}")

    rust_fields = parse_rust_struct_fields(git_text(concrete["semantic_head"], concrete["path"]), concrete["symbol"])
    alloy_fields = parse_alloy_sig_fields(git_text(formal["semantic_head"], formal["path"]), formal["symbol"])

    field_map = row["field_map"]
    require(bool(field_map), "B4 field map empty")
    mapped_concrete: list[str] = []
    mapped_formal: list[str] = []
    for index, item in enumerate(field_map):
        exact_keys(item, {"concrete", "formal"}, f"field_map[{index}]")
        mapped_concrete.append(item["concrete"])
        mapped_formal.append(item["formal"])
    require(len(mapped_concrete) == len(set(mapped_concrete)), "duplicate concrete B4 field mapping")
    require(len(mapped_formal) == len(set(mapped_formal)), "duplicate formal B4 field mapping")

    metadata = concrete["metadata_fields"]
    require(set(rust_fields) == set(mapped_concrete) | set(metadata), f"Rust ClaimBinding census mismatch source={rust_fields} mapped={mapped_concrete} metadata={metadata}")
    require(set(mapped_concrete).isdisjoint(metadata), "metadata field also mapped semantically")
    require(set(alloy_fields) == set(mapped_formal), f"Alloy Binding census mismatch source={alloy_fields} mapped={mapped_formal}")
    require(len(mapped_concrete) == 9 and len(mapped_formal) == 9, "B4 semantic field cardinality must be exactly nine")

    expected_pairs = {
        ("claim_id", "claimId"),
        ("envelope_digest", "envelope"),
        ("nonce", "nonce"),
        ("use_index", "useIndex"),
        ("jurisdiction", "jurisdiction"),
        ("matter", "matter"),
        ("target_digest", "target"),
        ("payload_digest", "payload"),
        ("budget_id", "budget"),
    }
    actual_pairs = {(item["concrete"], item["formal"]) for item in field_map}
    require(actual_pairs == expected_pairs, f"B4 field correspondence drift: {sorted(actual_pairs ^ expected_pairs)}")

    return {
        "rust_claim_binding_fields": rust_fields,
        "alloy_binding_fields": alloy_fields,
        "mapped_pairs": sorted([list(pair) for pair in actual_pairs]),
        "concrete_blob": concrete_blob,
        "formal_blob": formal_blob,
    }


def validate_mapping_separation(
    subjects: dict[str, dict[str, Any]], relationships: list[dict[str, Any]]
) -> None:
    relationship_subjects = {row["subject_id"] for row in relationships}
    require(relationship_subjects == {"MYC-CONST-003B4"}, "candidate relationship subject census drift")
    for subject_id, subject in subjects.items():
        status = subject["mapping_status"]
        if status == "candidate":
            require(subject_id in relationship_subjects, f"candidate subject lacks relationship: {subject_id}")
        if status == "unmapped":
            require(subject_id not in relationship_subjects, f"qualified-but-unmapped subject silently promoted: {subject_id}")
    require(subjects["MYC-CONST-003C3"]["mapping_status"] == "unmapped", "C3 concrete refinement overclaimed")
    require(subjects["MYC-CONST-003D1A"]["mapping_status"] == "unmapped", "D1A runtime refinement overclaimed")


def validate_pending(items: list[dict[str, Any]]) -> None:
    require(len(items) == len(EXPECTED_PENDING), "pending subject census drift")
    required = {"id", "semantic_head", "verifier_head", "qualification_run_id", "status", "reason"}
    seen: set[str] = set()
    for item in items:
        exact_keys(item, required, f"pending:{item.get('id')}")
        subject_id = item["id"]
        require(subject_id in EXPECTED_PENDING, f"unexpected pending subject: {subject_id}")
        require(subject_id not in seen, f"duplicate pending subject: {subject_id}")
        seen.add(subject_id)
        expected = EXPECTED_PENDING[subject_id]
        require(
            (item["semantic_head"], item["verifier_head"], item["qualification_run_id"]) == expected,
            f"pending evidence identity drift: {subject_id}",
        )
        require(item["status"] in {"queued", "in_progress", "prepared"}, f"pending status promoted without receipt: {subject_id}")
        require(bool(item["reason"].strip()), f"pending reason empty: {subject_id}")
    require(seen == set(EXPECTED_PENDING), "pending subject IDs drift")


def validate_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    exact_keys(
        manifest,
        {
            "schema", "crosswalk_id", "version", "status", "parent_crosswalk", "subjects",
            "inherited_relationships", "relationships", "pending_subjects", "non_claims",
        },
        "root",
    )
    require(manifest["schema"] == "mycelix.constitutional-refinement-crosswalk.v2", "schema identity drift")
    require(manifest["crosswalk_id"] == "mycelix.constitutional-refinement-evidence.v2", "crosswalk id drift")
    require(manifest["version"] == "0.2.0", "crosswalk version drift")
    require(manifest["status"] == "draft", "CR2 semantic manifest cannot self-promote to qualified")

    validate_parent(manifest["parent_crosswalk"])
    subjects = validate_subjects(manifest["subjects"])
    validate_inherited(manifest["inherited_relationships"])
    census = validate_relationships(manifest["relationships"], subjects)
    validate_mapping_separation(subjects, manifest["relationships"])
    validate_pending(manifest["pending_subjects"])
    require(REQUIRED_NONCLAIMS <= set(manifest["non_claims"]), "required CR2 non-claims missing")
    return census


def self_test(manifest: dict[str, Any]) -> None:
    mutations: list[tuple[str, Any]] = []

    def mutated(name: str, fn: Any) -> None:
        candidate = copy.deepcopy(manifest)
        fn(candidate)
        mutations.append((name, candidate))

    mutated("promote-c3-without-mapping", lambda p: p["subjects"][2].__setitem__("mapping_status", "candidate"))
    mutated("promote-d1a-without-mapping", lambda p: p["subjects"][3].__setitem__("mapping_status", "candidate"))
    mutated("b4-head-drift", lambda p: p["relationships"][0]["concrete"].__setitem__("semantic_head", EXPECTED_PARENT["semantic_head"]))
    mutated("b4-blob-drift", lambda p: p["relationships"][0]["concrete"].__setitem__("git_blob_sha", "0" * 40))
    mutated("drop-payload-field", lambda p: p["relationships"][0].__setitem__("field_map", [x for x in p["relationships"][0]["field_map"] if x["concrete"] != "payload_digest"]))
    mutated("map-schema-version", lambda p: p["relationships"][0]["field_map"].append({"concrete": "schema_version", "formal": "schemaVersion"}))
    mutated("drop-parent-row", lambda p: p["inherited_relationships"].__setitem__("relationship_ids", p["inherited_relationships"]["relationship_ids"][:-1]))
    mutated("parent-artifact-drift", lambda p: p["parent_crosswalk"].__setitem__("artifact_digest", "sha256:" + "0" * 64))
    mutated("mark-d1c-completed", lambda p: p["pending_subjects"][0].__setitem__("status", "completed"))
    mutated("drop-not-qualified", lambda p: p.__setitem__("non_claims", [x for x in p["non_claims"] if x != "not_qualified"]))

    for name, candidate in mutations:
        try:
            validate_manifest(candidate)
        except (ValueError, subprocess.CalledProcessError):
            continue
        fail(f"mutation survived: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    validate_schema_file()
    manifest = json.loads(MANIFEST.read_text())
    census = validate_manifest(manifest)
    if args.self_test:
        self_test(manifest)

    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "crosswalk_id": manifest["crosswalk_id"],
        "version": manifest["version"],
        "status": manifest["status"],
        "qualified_subjects": [x["id"] for x in manifest["subjects"]],
        "candidate_relationships": [x["id"] for x in manifest["relationships"]],
        "unmapped_qualified_subjects": [x["id"] for x in manifest["subjects"] if x["mapping_status"] == "unmapped"],
        "pending_subjects": [x["id"] for x in manifest["pending_subjects"]],
        "census": census,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
