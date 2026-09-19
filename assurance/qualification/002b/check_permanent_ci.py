#!/usr/bin/env python3
"""Fail-closed static guard for the ASSURE-002B permanent-CI composition."""

from __future__ import annotations

from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parents[3]
LINEAGE = tomllib.loads(Path(__file__).with_name("lineage.toml").read_text())
CI = (ROOT / ".github/workflows/ci.yml").read_text()
TEMP = ROOT / ".github/workflows/assure-002b-probe.yml"

EXPECTED = {
    "qualified_002a_commit": "d17a6ae19fb71159b54b266fa760c1989ffc0a63",
    "qualified_002a_receipt_sha256": "f4bdeacc8e4b38f0aad31d03788a8c105790f60667608fb43665ac321454c897",
    "temporary_002b_subject_commit": "9a2460e97f0bb0b90306e90e7c241b89e21224ec",
    "temporary_002b_subject_tree": "9cbb5789431ca95af5001c0015e5d2a37de18d34",
    "temporary_002b_receipt_sha256": "d643f34328e9e862ce50038b8166c93399e9c1a078c75f4cb36f56eb02c5a454",
    "scheduler_main_ancestor": "a85369699099d4c7524e502e531735eed4ab36f4",
    "spec_root_sha256": "46550610da111888f862d4bfc610f6dc41edfcbcdcd0579377e367df16486973",
    "coverage_sha256": "32bff983f4cdc9d3393e6e4381f8ed470ca29d08e13f8bebe7d1973a3bd0c1e1",
    "corpus_root_sha256": "784ff870e0b9815b015be1bbaa32f0dcc615cb3c22c614ba8bf9389fe4ccaef8",
    "cargo_lock_sha256": "9a0babe726db931fb24087af4e1321f034e15b29a03d1b9249cd6176b3525a8b",
    "rust_toolchain": "1.98.1",
}


def fail(message: str) -> None:
    raise SystemExit(message)


if LINEAGE.get("protocol") != "MYCELIX-ASSURE/002B-PERMANENT-CI" or LINEAGE.get("version") != 1:
    fail("unexpected permanent-CI lineage protocol/version")
if LINEAGE.get("status") != "permanent-ci-candidate":
    fail("unexpected permanent-CI lineage status")
for key, expected in EXPECTED.items():
    if LINEAGE.get(key) != expected:
        fail(f"lineage mismatch: {key}")
if LINEAGE.get("temporary_002b_run_id") != 35323729046:
    fail("temporary run mismatch")
if LINEAGE.get("temporary_002b_job_id") != 105531857097:
    fail("temporary job mismatch")
if LINEAGE.get("temporary_002b_artifact_id") != 10579669658:
    fail("temporary artifact id mismatch")
if LINEAGE.get("temporary_002b_artifact_digest") != "sha256:0fabfb8a6f26f51246c9217bfcf0a28dd4d4ab36f9d33e57df639d3459dd5294":
    fail("temporary artifact digest mismatch")
if TEMP.exists():
    fail("temporary ASSURE-002B qualifier must not survive permanent-CI fold")

for required in [
    "cancel-in-progress: true",
    "github.event.pull_request.draft == false",
    "assurance: ${{ steps.filter.outputs.assurance }}",
    "- 'assurance/**'",
    "- 'crates/mycelix-assurance-core/**'",
    "test-assurance:",
    "actions/checkout@11d5960a326750d5838078e36cf38b85af677262",
    "RUSTUP_TOOLCHAIN: 1.98.1",
    EXPECTED["spec_root_sha256"],
    EXPECTED["coverage_sha256"],
    EXPECTED["corpus_root_sha256"],
    EXPECTED["cargo_lock_sha256"],
    "cargo check --locked --all-targets",
    "cargo test --locked --all-targets",
    "cargo clippy --locked --all-targets -- -D warnings",
    "- changes",
    "- test-assurance",
    "require_result",
    "needs.changes.outputs.assurance",
]:
    if required not in CI:
        fail(f"required permanent-CI contract text missing: {required}")

try:
    assurance_job = CI.split("\n  test-assurance:\n", 1)[1].split("\n  # Summary gate", 1)[0]
    summary_job = CI.split("\n  ci-pass:\n", 1)[1]
except IndexError as exc:
    raise SystemExit("unable to isolate permanent assurance/summary jobs") from exc

for required in [
    "persist-credentials: false",
    "ref: ${{ github.event.pull_request.head.sha || github.sha }}",
    "${{ github.event.pull_request.head.sha }}",
    "python3 assurance/spec/v1/check_spec.py",
    "python3 assurance/vectors/v1/l0/check_vectors.py",
    "python3 assurance/qualification/002b/check_permanent_ci.py",
    "default feature set must remain empty",
    "002B must not have build.rs",
    "#![forbid(unsafe_code)]",
    "git status --porcelain --untracked-files=all",
    "Emit canonical permanent qualification receipt",
    "mycelix.assure.002b.permanent-ci.receipt.v0.1",
    '"qualification_subject_commit": subject',
    '"qualification_subject_tree": tree',
    '"temporary_002b_receipt_sha256": lineage["temporary_002b_receipt_sha256"]',
    '"workflow_sha256": workflow_sha256',
    '"github_run_id": os.environ["GITHUB_RUN_ID"]',
    "assure-002b-permanent-receipt.json",
    "assure-002b-permanent-receipt.sha256",
    "actions/upload-artifact@ea165f8d65b6e75b540449e92b4886f43607fa02",
    "if-no-files-found: error",
    "retention-days: 30",
]:
    if required not in assurance_job:
        fail(f"required assurance-job guard missing: {required}")

for forbidden in [
    "actions/upload-artifact@v",
    "actions/checkout@v4\n        with:\n          ref: ${{ github.event.pull_request.head.sha || github.sha }}",
]:
    if forbidden in assurance_job:
        fail(f"mutable evidence action/reference present: {forbidden}")

for required in [
    'require_result "changes"',
    'require_result "format"',
    'require_conditional "assurance"',
    "needs.test-assurance.result",
]:
    if required not in summary_job:
        fail(f"required fail-closed summary guard missing: {required}")

print("assure_002b_permanent_ci_static=PASS")
print(f"temporary_receipt_sha256={EXPECTED['temporary_002b_receipt_sha256']}")
print(f"scheduler_main_ancestor={EXPECTED['scheduler_main_ancestor']}")
