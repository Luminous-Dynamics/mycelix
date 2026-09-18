#!/usr/bin/env python3
"""Verify the frozen JIT-0 design contract, profile, and registries."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import tomllib

ROOT = Path(__file__).resolve().parent
LOCK = tomllib.loads((ROOT / "JIT.lock").read_text())

EXPECTED_FILES = [
    "normative.md",
    "envelope.schema.json",
    "state-machine.toml",
    "failure-codes.toml",
    "conformance.md",
    "containment-receipt.schema.json",
    "profiles/assure-002b-l0-v1.toml",
]
HASH_KEYS = {
    "normative.md": "normative_md",
    "envelope.schema.json": "envelope_schema_json",
    "state-machine.toml": "state_machine_toml",
    "failure-codes.toml": "failure_codes_toml",
    "conformance.md": "conformance_md",
    "containment-receipt.schema.json": "containment_receipt_schema_json",
    "profiles/assure-002b-l0-v1.toml": "profile_assure_002b_l0_v1_toml",
}
EXPECTED_STATES = {
    "Prepared",
    "AuthorizationValidated",
    "JitRegistered",
    "BootedClean",
    "SubjectBound",
    "Executing",
    "ResultSealed",
    "LogsExported",
    "Quarantined",
    "Destroyed",
}
HAPPY_PATH = [
    ("Prepared", "AuthorizationValidated"),
    ("AuthorizationValidated", "JitRegistered"),
    ("JitRegistered", "BootedClean"),
    ("BootedClean", "SubjectBound"),
    ("SubjectBound", "Executing"),
    ("Executing", "ResultSealed"),
    ("ResultSealed", "LogsExported"),
    ("LogsExported", "Destroyed"),
]
QUARANTINE_SOURCES = {
    "Prepared",
    "AuthorizationValidated",
    "JitRegistered",
    "BootedClean",
    "SubjectBound",
    "Executing",
    "ResultSealed",
    "LogsExported",
}
PROFILE = {
    "version": 1,
    "name": "assure-002b-l0-v1",
    "repository": "Luminous-Dynamics/mycelix",
    "subject_commit": "9a2460e97f0bb0b90306e90e7c241b89e21224ec",
    "subject_tree": "9cbb5789431ca95af5001c0015e5d2a37de18d34",
    "expected_parent_commit": "d17a6ae19fb71159b54b266fa760c1989ffc0a63",
    "qualified_parent_receipt_sha256": "f4bdeacc8e4b38f0aad31d03788a8c105790f60667608fb43665ac321454c897",
    "spec_root_sha256": "46550610da111888f862d4bfc610f6dc41edfcbcdcd0579377e367df16486973",
    "coverage_root_sha256": "32bff983f4cdc9d3393e6e4381f8ed470ca29d08e13f8bebe7d1973a3bd0c1e1",
    "corpus_root_sha256": "784ff870e0b9815b015be1bbaa32f0dcc615cb3c22c614ba8bf9389fe4ccaef8",
    "dependency_lock_sha256": "9a0babe726db931fb24087af4e1321f034e15b29a03d1b9249cd6176b3525a8b",
    "rustc": "rustc 1.98.1 (48a229cea 2026-09-01)",
    "cargo": "cargo 1.98.1 (797e8a9bc 2026-08-05)",
    "target": "x86_64-unknown-linux-gnu",
    "receipt_schema": "mycelix.assure.002b.qualification.receipt.v0.1",
    "containment_receipt_schema": "mycelix.jit.execution-containment.receipt.v0.1",
    "manual_obligations": [
        "CODEC-001",
        "CODEC-011",
        "CODEC-012",
        "CODEC-013",
        "CODEC-014",
        "CODEC-015",
    ],
}
PROFILE_SHA256 = "9c9bea898f07a068e8d4c4be7d39adc81bf11a5b4ea9b9fc19d44b90c42a6a9b"
JCS_SAFE_INTEGER_MAX = 9_007_199_254_740_991


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fail(message: str) -> None:
    raise SystemExit(message)


if LOCK.get("protocol") != "MYCELIX-JIT-QUAL":
    fail("unexpected protocol")
if (LOCK.get("major"), LOCK.get("minor")) != (0, 1):
    fail("unexpected protocol version")
if LOCK.get("status") != "design-freeze-candidate":
    fail("unexpected status")
if LOCK.get("hash") != "sha256":
    fail("unexpected hash suite")
if LOCK.get("spec_domain") != "MYCELIX-JIT-QUAL/V0.1/SPEC":
    fail("unexpected spec domain")
if LOCK.get("order") != EXPECTED_FILES:
    fail("unexpected file order")

file_hashes = LOCK.get("file_hashes", {})
actual_hashes: dict[str, str] = {}
for name in EXPECTED_FILES:
    data = (ROOT / name).read_bytes()
    digest = sha256_bytes(data)
    actual_hashes[name] = digest
    if file_hashes.get(HASH_KEYS[name]) != digest:
        fail(f"file hash mismatch: {name}")

root_hasher = hashlib.sha256()
root_hasher.update(LOCK["spec_domain"].encode("utf-8"))
for name in EXPECTED_FILES:
    encoded_name = name.encode("utf-8")
    root_hasher.update(len(encoded_name).to_bytes(4, "big"))
    root_hasher.update(encoded_name)
    root_hasher.update(bytes.fromhex(actual_hashes[name]))

root = root_hasher.hexdigest()
if LOCK.get("spec_root_sha256") != root:
    fail("spec root mismatch")

profile_path = ROOT / "profiles/assure-002b-l0-v1.toml"
if sha256_bytes(profile_path.read_bytes()) != PROFILE_SHA256:
    fail("profile SHA-256 mismatch")
profile = tomllib.loads(profile_path.read_text())
if profile != PROFILE:
    fail("profile manifest does not match frozen v0.1 profile")

schema = json.loads((ROOT / "envelope.schema.json").read_text())
if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
    fail("unexpected JSON Schema dialect")
if schema.get("additionalProperties") is not False:
    fail("envelope must reject unknown top-level fields")
properties = schema.get("properties", {})
if properties.get("repository", {}).get("const") != PROFILE["repository"]:
    fail("v0.1 repository scope is not frozen")
if properties.get("qualification_profile", {}).get("enum") != [PROFILE["name"]]:
    fail("unexpected qualification profile surface")
if properties.get("expected_parent_commit") != {"$ref": "#/$defs/gitSha1"}:
    fail("v0.1 predecessor must be mandatory and non-null")
if properties.get("expires_at_unix_micros", {}).get("maximum") != JCS_SAFE_INTEGER_MAX:
    fail("expiry must remain within exact JCS integer range")
if properties.get("receipt_schema", {}).get("const") != PROFILE["receipt_schema"]:
    fail("qualification receipt schema mismatch")
if properties.get("containment_receipt_schema", {}).get("const") != PROFILE["containment_receipt_schema"]:
    fail("containment receipt schema mismatch")
toolchain = properties.get("toolchain", {}).get("properties", {})
if toolchain.get("rustc", {}).get("const") != PROFILE["rustc"]:
    fail("rustc identity is not frozen")
if toolchain.get("cargo", {}).get("const") != PROFILE["cargo"]:
    fail("cargo identity is not frozen")
if toolchain.get("target", {}).get("const") != PROFILE["target"]:
    fail("target identity is not frozen")
required = set(schema.get("required", []))
for field in {
    "qualification_profile_sha256",
    "qualified_parent_receipt_sha256",
    "coverage_root_sha256",
    "corpus_root_sha256",
    "controller_policy_sha256",
    "runner_group_id",
    "expected_parent_commit",
    "operator_authorization_sha256",
    "containment_receipt_schema",
}:
    if field not in required:
        fail(f"required envelope binding missing: {field}")

workflow_pattern = properties.get("workflow", {}).get("properties", {}).get("path", {}).get("pattern")
if workflow_pattern is None:
    fail("workflow path pattern missing")
workflow_re = re.compile(workflow_pattern)
for good in [
    ".github/workflows/jit-qualification.yml",
    ".github/workflows/assure/jit-qualification.yaml",
]:
    if workflow_re.fullmatch(good) is None:
        fail(f"workflow path false rejection: {good}")
for bad in [
    ".github/workflows/../evil.yml",
    ".github/workflows/./evil.yml",
    ".github/workflows/a/../evil.yml",
    ".github/workflows/a/./evil.yml",
    ".github/workflows/a//evil.yml",
    "/.github/workflows/evil.yml",
    ".github/workflows/evil.txt",
]:
    if workflow_re.fullmatch(bad) is not None:
        fail(f"workflow path traversal/shape accepted: {bad}")

containment = json.loads((ROOT / "containment-receipt.schema.json").read_text())
if containment.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
    fail("unexpected containment JSON Schema dialect")
if containment.get("additionalProperties") is not False:
    fail("containment receipt must reject unknown top-level fields")
cp = containment.get("properties", {})
if cp.get("schema", {}).get("const") != PROFILE["containment_receipt_schema"]:
    fail("containment receipt schema identity mismatch")
for field in ["started_at_unix_micros", "destroyed_at_unix_micros"]:
    if cp.get(field, {}).get("maximum") != JCS_SAFE_INTEGER_MAX:
        fail(f"{field} must remain within exact JCS integer range")
required_containment = set(containment.get("required", []))
for field in {
    "execution_envelope_sha256",
    "qualification_receipt_sha256",
    "qualification_result",
    "containment_result",
    "runner_group_id",
    "runner_registration_id",
    "runner_image_sha256",
    "controller_policy_sha256",
    "network_policy_sha256",
    "allowed_commands_sha256",
    "log_bundle_sha256",
    "destruction_evidence_sha256",
    "failure_codes",
}:
    if field not in required_containment:
        fail(f"required containment binding missing: {field}")
conditions = {}
for clause in containment.get("allOf", []):
    result = clause.get("if", {}).get("properties", {}).get("containment_result", {}).get("const")
    failure_shape = clause.get("then", {}).get("properties", {}).get("failure_codes", {})
    if result is not None:
        conditions[result] = failure_shape
if conditions != {
    "PASS": {"maxItems": 0},
    "FAIL": {"minItems": 1},
}:
    fail("containment PASS/FAIL failure-code coherence is not frozen")
if cp.get("failure_codes", {}).get("items", {}).get("pattern") != r"^J0(?:0[1-9]|1[0-9]|2[0-3])_[A-Z0-9_]+$":
    fail("containment failure-code registry surface mismatch")

states = tomllib.loads((ROOT / "state-machine.toml").read_text())
transitions = states.get("transition", [])
edges = [(t["from"], t["to"]) for t in transitions]
if len(edges) != len(set(edges)):
    fail("duplicate lifecycle transition")
if states.get("initial") != "Prepared":
    fail("unexpected initial state")
if states.get("terminal") != ["Destroyed"]:
    fail("unexpected terminal state")
seen_states = {s for edge in edges for s in edge}
if seen_states != EXPECTED_STATES:
    fail("unexpected lifecycle state set")
if any(edge not in edges for edge in HAPPY_PATH):
    fail("happy path is incomplete")
if {source for source, target in edges if target == "Quarantined"} != QUARANTINE_SOURCES:
    fail("unexpected quarantine transition surface")
if [edge for edge in edges if edge[0] == "Quarantined"] != [("Quarantined", "Destroyed")]:
    fail("Quarantined must transition only to Destroyed")
if any(source == "Destroyed" for source, _ in edges):
    fail("Destroyed must have no outgoing transitions")
if any(target == "Prepared" for _, target in edges):
    fail("Prepared must have no incoming transitions")

failures = tomllib.loads((ROOT / "failure-codes.toml").read_text())
if failures.get("version") != 1:
    fail("unexpected failure registry version")
entries = failures.get("failure", [])
ids = [entry["id"] for entry in entries]
codes = [entry["code"] for entry in entries]
if len(ids) != len(set(ids)) or len(codes) != len(set(codes)):
    fail("duplicate failure id/code")
if ids != list(range(1, 24)):
    fail("failure ids must be exactly J001-J023 in v0.1")
allowed_stages = {"admission", "binding", "registration", "containment", "execution"}
if any(entry["stage"] not in allowed_stages for entry in entries):
    fail("unknown failure stage")

normative_text = (ROOT / "normative.md").read_text()
if "JIT-055 — Containment receipt consistency." not in normative_text:
    fail("relational containment invariant is missing")

print(f"jit_spec_root_sha256={root}")
print(f"qualification_profile_sha256={PROFILE_SHA256}")
