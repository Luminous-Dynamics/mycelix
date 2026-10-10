#!/usr/bin/env python3
"""Emit the canonical predicate for a trusted D6U evidence attestation."""

import hashlib
import json
import sys
from pathlib import Path



if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")

ROOT = Path(__file__).parents[2]
POLICY = ROOT / "docs/integral/d6u-trusted-builder-policy.json"

SUBJECTS = (
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
)

EXPECTED_STATUS = "runtime-reference-evidence"
PREDICATE_TYPE = "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"


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


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: emit_d6u_trusted_attestation_predicate.py "
            "ARTIFACT_DIR OUTPUT_JSON"
        )

    artifact_dir = Path(sys.argv[1]).resolve()
    output_path = Path(sys.argv[2]).resolve()
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    record = load_record(artifact_dir / "d6u-runtime-evidence.txt")

    assert policy["claim_ceiling"] == "ReferenceModelOnly"
    assert record["status"] == EXPECTED_STATUS
    assert record["claim_ceiling"] == policy["claim_ceiling"]
    assert record["source_repository"] == "Luminous-Dynamics/mycelix"
    assert record["source_branch"] == policy["source_branch"]
    assert record["source_commit"]
    assert record["attestation_status"] == "deferred-to-trusted-builder"

    subjects = [
        {
            "name": name,
            "digest": {"sha256": sha256(artifact_dir / name)},
        }
        for name in SUBJECTS
    ]

    predicate = {
        "schema": "d6u-trusted-runtime-evidence/v1",
        "attestation_kind": "verified-runtime-evidence",
        "claim_ceiling": policy["claim_ceiling"],
        "policy_version": policy["policy_version"],
        "source": {
            "repository": record["source_repository"],
            "branch": record["source_branch"],
            "commit": record["source_commit"],
        },
        "trigger": {
            "workflow_name": record["trigger_workflow_name"],
            "workflow_path": record["trigger_workflow_path"],
            "run_id": int(record["trigger_workflow_run_id"]),
            "run_attempt": int(record["trigger_workflow_run_attempt"]),
        },
        "executor": {
            "workflow_name": "D6U Exact-Head Runtime Executor",
            "workflow_path": record["executor_workflow_file_path"],
            "run_id": int(record["executor_run_id"]),
            "run_attempt": int(record["executor_run_attempt"]),
            "workflow_commit": record["executor_workflow_commit_sha"],
        },
        "subjects": subjects,
        "evidence": {
            "case_coverage": record["case_coverage"],
            "supplemental_coverage": record["supplemental_coverage"],
            "application_check_coverage": record["application_check_coverage"],
            "case_outcome_classes": record["case_outcome_classes"].split(","),
            "runtime": record["runtime"],
            "hdk": record["hdk"],
            "hdi": record["hdi"],
            "unsupported_cases": record["unsupported_cases"].split(","),
        },
        "nonclaims": [
            "semantic-truth",
            "production-safety",
            "legal-authority",
            "actuation-authority",
        ],
    }

    assert policy["attestation_verification"]["predicate_type"] == PREDICATE_TYPE
    output_path.write_text(
        json.dumps(predicate, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    print(
        "emitted canonical D6U trusted evidence predicate: "
        f"policy=v{policy['policy_version']}, source={record['source_commit']}"
    )


if __name__ == "__main__":
    main()
