#!/usr/bin/env python3
"""Verify the identity and semantics of a D6U trusted evidence attestation."""

import hashlib
import json
import os
import re
import sys
from pathlib import Path


PREDICATE_TYPE = "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
PREDICATE_SCHEMA = "d6u-trusted-runtime-evidence/v1"
SUBJECT_NAMES = (
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_record(path: Path) -> dict[str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines and lines[0] == "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE"
    record: dict[str, str] = {}
    for line in lines[1:]:
        key, separator, value = line.partition("=")
        assert separator and key and key not in record
        record[key] = value
    return record


def expected_subjects(evidence_dir: Path) -> list[dict]:
    return [
        {
            "name": name,
            "digest": {"sha256": sha256(evidence_dir / name)},
        }
        for name in SUBJECT_NAMES
    ]


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_d6u_trusted_attestation.py ATTESTATION_JSON")

    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    assert isinstance(report, list) and report, "attestation verification returned no results"

    repo = os.environ["GITHUB_REPOSITORY"]
    run_id = os.environ["GITHUB_RUN_ID"]
    run_attempt = os.environ["GITHUB_RUN_ATTEMPT"]
    workflow_ref = os.environ["GITHUB_WORKFLOW_REF"]
    workflow_sha = os.environ["GITHUB_WORKFLOW_SHA"]
    source_sha = os.environ["GITHUB_SHA"]
    source_ref = os.environ["GITHUB_REF"]
    policy_version = int(os.environ["D6U_TRUSTED_POLICY_VERSION"])
    evidence_dir = Path(os.environ["D6U_TRUSTED_EVIDENCE_DIR"])
    subject_path = Path(os.environ["D6U_ATTESTATION_SUBJECT"])

    record = load_record(evidence_dir / "d6u-runtime-evidence.txt")
    subject_digest = sha256(subject_path)
    subjects = expected_subjects(evidence_dir)

    expected_workflow = (
        f"{repo}/.github/workflows/d6u-trusted-evidence-attestation.yml"
    )
    expected_san = (
        f"https://github.com/{expected_workflow}@refs/heads/main"
    )
    expected_run_uri = (
        f"https://github.com/{repo}/actions/runs/{run_id}/attempts/{run_attempt}"
    )

    assert workflow_ref == f"{repo}/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main"
    assert re.fullmatch(r"[0-9a-f]{40}", workflow_sha)
    assert re.fullmatch(r"[0-9a-f]{40}", source_sha)
    assert source_ref == "refs/heads/main"

    matches = []
    for entry in report:
        result = entry.get("verificationResult", {})
        certificate = result.get("signature", {}).get("certificate", {})
        statement = result.get("statement", {})
        statement_subjects = statement.get("subject", [])
        predicate = statement.get("predicate", {})

        certificate_ok = (
            certificate.get("subjectAlternativeName") == expected_san
            and certificate.get("issuer") == "https://token.actions.githubusercontent.com"
            and certificate.get("githubWorkflowRepository") == repo
            and certificate.get("githubWorkflowRef") == "refs/heads/main"
            and certificate.get("sourceRepositoryURI") == f"https://github.com/{repo}"
            and certificate.get("sourceRepositoryDigest") == source_sha
            and certificate.get("runnerEnvironment") == "github-hosted"
            and certificate.get("runInvocationURI") == expected_run_uri
        )

        predicate_ok = (
            statement.get("predicateType") == PREDICATE_TYPE
            and predicate.get("schema") == PREDICATE_SCHEMA
            and predicate.get("attestation_kind") == "verified-runtime-evidence"
            and predicate.get("claim_ceiling") == record["claim_ceiling"]
            and predicate.get("policy_version") == policy_version
            and predicate.get("source") == {
                "repository": record["source_repository"],
                "branch": record["source_branch"],
                "commit": record["source_commit"],
            }
            and predicate.get("trigger") == {
                "workflow_name": record["trigger_workflow_name"],
                "workflow_path": record["trigger_workflow_path"],
                "run_id": int(record["trigger_workflow_run_id"]),
                "run_attempt": int(record["trigger_workflow_run_attempt"]),
            }
            and predicate.get("executor") == {
                "workflow_name": "D6U Exact-Head Runtime Executor",
                "workflow_path": record["executor_workflow_file_path"],
                "run_id": int(record["executor_run_id"]),
                "run_attempt": int(record["executor_run_attempt"]),
                "workflow_commit": record["executor_workflow_commit_sha"],
            }
            and predicate.get("subjects") == subjects
            and predicate.get("evidence") == {
                "case_coverage": record["case_coverage"],
                "supplemental_coverage": record["supplemental_coverage"],
                "application_check_coverage": record["application_check_coverage"],
                "case_outcome_classes": record["case_outcome_classes"].split(","),
                "runtime": record["runtime"],
                "hdk": record["hdk"],
                "hdi": record["hdi"],
                "unsupported_cases": record["unsupported_cases"].split(","),
            }
            and predicate.get("nonclaims") == [
                "semantic-truth",
                "production-safety",
                "legal-authority",
                "actuation-authority",
            ]
        )

        subject_binding_ok = (
            statement.get("subject") == subjects
            and any(
                subject.get("digest", {}).get("sha256") == subject_digest
                for subject in statement_subjects
            )
        )

        if certificate_ok and predicate_ok and subject_binding_ok:
            matches.append(entry)

    assert len(matches) == 1, (
        f"expected exactly one current-run D6U evidence attestation: {len(matches)}"
    )
    print(
        "verified D6U trusted evidence attestation: "
        f"subject={subject_digest}, run={run_id}, attempt={run_attempt}, "
        f"policy=v{policy_version}"
    )


if __name__ == "__main__":
    main()
