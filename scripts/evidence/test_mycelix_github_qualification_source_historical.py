from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mycelix_evidence_manifest import MANIFEST_VERSION, validate_manifest  # noqa: E402
from mycelix_github_qualification_source import (  # noqa: E402
    PROFILE_VERSION,
    normalize_github_qualification_source_v1,
    project_github_technical_evidence_v1,
)

REPOSITORY_ID = 1176351975

HISTORICAL_CASES = (
    {
        "name": "MYC-EVID-001A default pull-request checkout",
        "run_id": 34912377654,
        "run_attempt": 1,
        "workflow_path": ".github/workflows/mycelix-evidence-manifest-v1.yml",
        "run_head_sha": "094de4cf08ea155eeccbc59ccd9378ff2f03029d",
        "run_head_tree_sha": "56a92092c396ef821b69ef9bfc733bbfaa34563d",
        "technical_profile": "mycelix-evidence-manifest-v1",
    },
    {
        "name": "MYC-CAP-002A default pull-request checkout",
        "run_id": 34959974354,
        "run_attempt": 1,
        "workflow_path": ".github/workflows/myc-cap-002a-return-envelope-v1.yml",
        "run_head_sha": "270b852e0ac744dfca3a2cb966bf53fce78f2ab9",
        "run_head_tree_sha": "dc004022298bcf784126e0785f2caf4677fa3799",
        "technical_profile": "myc-cap-002a-return-envelope-v1",
    },
)


def profile(case):
    return {
        "profile_version": PROFILE_VERSION,
        "source_profile": "github-qualification-source-v1",
        "technical_profile": case["technical_profile"],
        "repository_id": REPOSITORY_ID,
        "workflow_path": case["workflow_path"],
        "designated_subject_sha": case["run_head_sha"],
        "designated_subject_tree_sha": case["run_head_tree_sha"],
        "allowed_workflow_events": ["pull_request"],
        "expected_receipt_profile": "historical-qualification-receipt-v1",
    }


def source(case, *, conclusion="success"):
    return {
        "provider": "github",
        "source_evidence_id": (
            f"github:run{case['run_id']}:attempt{case['run_attempt']}:historical-fixture"
        ),
        "repository_id": REPOSITORY_ID,
        "workflow_path": case["workflow_path"],
        "workflow_event": "pull_request",
        "run_id": case["run_id"],
        "run_attempt": case["run_attempt"],
        "run_head_sha": case["run_head_sha"],
        "workflow_status": "completed",
        "workflow_conclusion": conclusion,
        "tested_subject_sha": None,
        "tested_tree_sha": None,
        "exact_subject_assertion": "unproven",
        "receipt_sha256": None,
        "receipt_subject_sha": None,
        "receipt_profile": None,
        "source_references": [
            f"api:actions/runs/{case['run_id']}",
            f"repo:{case['workflow_path']}@{case['run_head_sha']}",
        ],
    }


def manifest_for(record):
    return {
        "manifest_version": MANIFEST_VERSION,
        "technical_evidence": [record],
        "security_evidence": [],
        "commercial_evidence": [],
        "capital_evidence": [],
        "claims": [],
        "nonclaims": [],
    }


class HistoricalExactSubjectRegressions(unittest.TestCase):
    def test_real_successful_default_checkout_runs_do_not_become_exact_head_pass(self):
        for case in HISTORICAL_CASES:
            with self.subTest(case=case["name"]):
                normalized = normalize_github_qualification_source_v1(profile(case), source(case))
                self.assertEqual(normalized["run_head_sha"], case["run_head_sha"])
                self.assertIsNone(normalized["tested_subject_sha"])
                self.assertFalse(normalized["exact_subject_proven"])
                self.assertEqual(normalized["disposition"], "UNSUPPORTED")

                record = project_github_technical_evidence_v1(
                    profile(case),
                    source(case),
                    evidence_id="HISTORICAL-DEFAULT-CHECKOUT",
                    designated_current_subject_sha=case["run_head_sha"],
                )
                self.assertEqual(record["disposition"], "UNSUPPORTED")
                self.assertIsNone(record["receipt_sha256"])
                validate_manifest(manifest_for(record))

    def test_same_unproven_execution_identity_cannot_become_exact_head_fail(self):
        for case in HISTORICAL_CASES:
            with self.subTest(case=case["name"]):
                normalized = normalize_github_qualification_source_v1(
                    profile(case), source(case, conclusion="failure")
                )
                self.assertFalse(normalized["exact_subject_proven"])
                self.assertEqual(normalized["disposition"], "UNSUPPORTED")


if __name__ == "__main__":
    unittest.main()
