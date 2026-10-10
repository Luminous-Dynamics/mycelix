#!/usr/bin/env python3
"""Verify the identity and D6U evidence predicate of a trusted attestation."""

import hashlib
import json
import os
import sys
from pathlib import Path

PREDICATE_TYPE = "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
CANONICAL_PREDICATE_SCHEMA = "d6u-trusted-runtime-evidence/v1"
ATTESTATION_PREDICATE_SCHEMA = "d6u-trusted-runtime-evidence-attestation/v2"
SUBJECT_NAMES = (
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
)
NONCLAIMS = [
    "semantic-truth",
    "production-safety",
    "legal-authority",
    "actuation-authority",
]



if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")

def require(condition: bool, message: str = "verification invariant failed") -> None:
    if not condition:
        raise AssertionError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_record(path: Path) -> dict[str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    require(lines and lines[0] == "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE")
    record: dict[str, str] = {}
    for line in lines[1:]:
        key, separator, value = line.partition("=")
        require(separator and key and key not in record)
        record[key] = value
    return record


def expected_subjects(evidence_dir: Path) -> list[dict]:
    return [
        {"name": name, "digest": {"sha256": sha256(evidence_dir / name)}}
        for name in SUBJECT_NAMES
    ]


def canonical_subjects(value: list[dict]) -> tuple[tuple[str, str], ...]:
    require(isinstance(value, list))
    normalized: list[tuple[str, str]] = []
    for subject in value:
        require(isinstance(subject, dict))
        require(set(subject) == {"name", "digest"})
        name = subject["name"]
        digest = subject["digest"]
        require(isinstance(name, str) and name)
        require(isinstance(digest, dict))
        require(set(digest) == {"sha256"})
        sha = digest["sha256"]
        require(isinstance(sha, str) and len(sha) == 64)
        require(all(ch in "0123456789abcdef" for ch in sha))
        normalized.append((name, sha))
    require(len(set(normalized)) == len(normalized))
    return tuple(sorted(normalized))

def verify_canonical_predicate(
    predicate: dict,
    record: dict[str, str],
    subjects: list[dict],
    expected_policy_version: int,
) -> bool:
    require(isinstance(predicate, dict))
    event_repository = os.environ["D6U_TRIGGER_REPOSITORY"]
    event_branch = os.environ["D6U_TRIGGER_HEAD_BRANCH"]
    event_head_sha = os.environ["D6U_TRIGGER_HEAD_SHA"]
    event_run_id = int(os.environ["D6U_TRIGGER_RUN_ID"])
    event_run_attempt = int(os.environ["D6U_TRIGGER_RUN_ATTEMPT"])
    require(record["source_repository"] == event_repository)
    require(record["source_branch"] == event_branch)
    require(record["source_commit"] == event_head_sha)
    require(int(record["executor_run_id"]) == event_run_id)
    require(int(record["executor_run_attempt"]) == event_run_attempt)
    require(
        set(predicate) == {
            "attestation_kind", "claim_ceiling", "evidence", "executor", "nonclaims",
            "policy_version", "schema", "source", "subjects", "trigger",
        }
    )
    return (
        predicate.get("schema") == CANONICAL_PREDICATE_SCHEMA
        and predicate.get("attestation_kind") == "verified-runtime-evidence"
        and predicate.get("claim_ceiling") == record["claim_ceiling"]
        and predicate.get("policy_version") == expected_policy_version
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
        and canonical_subjects(predicate.get("subjects", [])) == canonical_subjects(subjects)
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
        and predicate.get("nonclaims") == NONCLAIMS
    )


def verify_commitment_entry(
    entry: dict,
    record: dict[str, str],
    subjects: list[dict],
    canonical_predicate_sha256: str,
) -> bool:
    repo = os.environ["GITHUB_REPOSITORY"]
    run_id = os.environ["GITHUB_RUN_ID"]
    run_attempt = os.environ["GITHUB_RUN_ATTEMPT"]
    expected_policy_version = int(os.environ["D6U_TRUSTED_POLICY_VERSION"])
    expected_source_digest = os.environ["GITHUB_SHA"]
    expected_san = "https://github.com/" + repo + "/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main"
    expected_run_uri = "https://github.com/" + repo + "/actions/runs/" + run_id + "/attempts/" + run_attempt
    try:
        require(isinstance(entry, dict))
        result = entry.get("verificationResult", {})
        certificate = result.get("signature", {}).get("certificate", {})
        statement = result.get("statement", {})
        predicate = statement.get("predicate", {})
        statement_subjects = statement.get("subject", [])
        verified_timestamps = result.get("verifiedTimestamps", [])
        require(statement.get("predicateType") == PREDICATE_TYPE)
        require(isinstance(predicate, dict))
        require(
            set(predicate) == {
                "attestation_kind",
                "canonical_predicate_sha256",
                "claim_ceiling",
                "policy_version",
                "schema",
            }
        )
        require(predicate.get("schema") == ATTESTATION_PREDICATE_SCHEMA)
        require(predicate.get("attestation_kind") == "verified-runtime-evidence")
        require(predicate.get("claim_ceiling") == record["claim_ceiling"])
        require(predicate.get("policy_version") == expected_policy_version)
        digest = predicate.get("canonical_predicate_sha256")
        require(isinstance(digest, str) and len(digest) == 64)
        require(all(ch in "0123456789abcdef" for ch in digest))
        require(digest == canonical_predicate_sha256)
        require(certificate.get("subjectAlternativeName") == expected_san)
        require(certificate.get("issuer") == "https://token.actions.githubusercontent.com")
        require(certificate.get("githubWorkflowRepository") == repo)
        require(certificate.get("githubWorkflowRef") == "refs/heads/main")
        require(certificate.get("sourceRepositoryURI") == "https://github.com/" + repo)
        require(certificate.get("sourceRepositoryDigest") == expected_source_digest)
        require(certificate.get("runnerEnvironment") == "github-hosted")
        require(certificate.get("runInvocationURI") == expected_run_uri)
        require(
            any(
                isinstance(timestamp, dict) and timestamp.get("type") == "Tlog"
                for timestamp in verified_timestamps
            )
        )
        require(canonical_subjects(statement_subjects) == canonical_subjects(subjects))
    except (AssertionError, AttributeError, KeyError, TypeError, ValueError):
        return False
    return True

def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_d6u_trusted_attestation.py ATTESTATION_JSON")

    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    require(isinstance(report, list) and report, "attestation verification returned no results")

    evidence_dir = Path(os.environ["D6U_TRUSTED_EVIDENCE_DIR"])
    subject_path = Path(os.environ["D6U_ATTESTATION_SUBJECT"])
    record = load_record(evidence_dir / "d6u-runtime-evidence.txt")
    subjects = expected_subjects(evidence_dir)
    subject_digest = sha256(subject_path)
    canonical_path = evidence_dir / "d6u-trusted-evidence-predicate.json"
    canonical_predicate = json.loads(canonical_path.read_text(encoding="utf-8"))
    require(isinstance(canonical_predicate, dict))
    require(
        verify_canonical_predicate(
            canonical_predicate,
            record,
            subjects,
            int(os.environ["D6U_TRUSTED_POLICY_VERSION"]),
        )
    )
    canonical_predicate_sha256 = sha256(canonical_path)

    matches = [
        entry
        for entry in report
        if verify_commitment_entry(
            entry,
            record,
            subjects,
            canonical_predicate_sha256,
        )
        and any(
            subject.get("digest", {}).get("sha256") == subject_digest
            for subject in entry.get("verificationResult", {})
            .get("statement", {})
            .get("subject", [])
        )
    ]

    require(
        len(matches) == 1,
        f"expected exactly one current-run D6U evidence attestation: {len(matches)}",
    )
    print(
        "verified D6U trusted evidence attestation: "
        f"subject={subject_digest}, run={os.environ['GITHUB_RUN_ID']}, "
        f"attempt={os.environ['GITHUB_RUN_ATTEMPT']}, "
        f"policy=v{os.environ['D6U_TRUSTED_POLICY_VERSION']}"
    )


if __name__ == "__main__":
    main()
