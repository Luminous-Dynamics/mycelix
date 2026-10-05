#!/usr/bin/env python3
"""Verify the immutable identity of a D6U trusted attestation."""

import hashlib
import json
import os
import re
import sys
from pathlib import Path


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

    subject_path = os.environ["D6U_ATTESTATION_SUBJECT"]
    subject_digest = hashlib.sha256(
        Path(subject_path).read_bytes()
    ).hexdigest()

    matches = []
    for entry in report:
        result = entry.get("verificationResult", {})
        certificate = result.get("signature", {}).get("certificate", {})
        statement = result.get("statement", {})
        subjects = statement.get("subject", [])

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
        statement_ok = (
            statement.get("predicateType") == "https://slsa.dev/provenance/v1"
            and any(
                subject.get("digest", {}).get("sha256") == subject_digest
                for subject in subjects
            )
        )
        if certificate_ok and statement_ok:
            matches.append(entry)

    assert len(matches) == 1, (
        f"expected exactly one current-run attestation match: {len(matches)}"
    )
    print(
        "verified D6U trusted attestation identity: "
        f"subject={subject_digest}, run={run_id}, attempt={run_attempt}"
    )


if __name__ == "__main__":
    main()
