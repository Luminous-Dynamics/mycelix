from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mycelix_evidence_manifest import MANIFEST_VERSION, validate_manifest  # noqa: E402
from mycelix_github_qualification_source import (  # noqa: E402
    PROFILE_VERSION,
    SourceAdapterError,
    canonical_normalized_source_bytes_v1,
    normalize_github_qualification_source_v1,
    normalized_source_sha256_v1,
    project_github_technical_evidence_v1,
)

SHA_A = "a" * 40
SHA_B = "b" * 40
TREE_A = "c" * 40
RECEIPT = "d" * 64


def profile():
    return {
        "profile_version": PROFILE_VERSION,
        "source_profile": "github-qualification-source-v1",
        "technical_profile": "agent-runtime-v1",
        "repository_id": 1176351975,
        "workflow_path": ".github/workflows/qual.yml",
        "designated_subject_sha": SHA_A,
        "designated_subject_tree_sha": TREE_A,
        "allowed_workflow_events": ["pull_request", "push"],
        "expected_receipt_profile": "qual-receipt-v1",
    }


def source(**changes):
    value = {
        "provider": "github",
        "source_evidence_id": "github:run42:attempt1:fixture",
        "repository_id": 1176351975,
        "workflow_path": ".github/workflows/qual.yml",
        "workflow_event": "pull_request",
        "run_id": 42,
        "run_attempt": 1,
        "run_head_sha": SHA_A,
        "workflow_status": "completed",
        "workflow_conclusion": "success",
        "tested_subject_sha": SHA_A,
        "tested_tree_sha": TREE_A,
        "exact_subject_assertion": "runtime_exact_checkout",
        "receipt_sha256": RECEIPT,
        "receipt_subject_sha": SHA_A,
        "receipt_profile": "qual-receipt-v1",
        "source_references": ["api:runs/42", "api:runs/42/jobs"],
    }
    value.update(changes)
    return value


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


class GitHubQualificationSourceTests(unittest.TestCase):
    def test_exact_runtime_checkout_with_bound_receipt_projects_pass(self):
        normalized = normalize_github_qualification_source_v1(profile(), source())
        self.assertTrue(normalized["exact_subject_proven"])
        self.assertEqual(normalized["disposition"], "PASS")
        record = project_github_technical_evidence_v1(
            profile(), source(), evidence_id="AGENT-002", designated_current_subject_sha=SHA_A
        )
        self.assertEqual(record["receipt_sha256"], RECEIPT)
        manifest = manifest_for(record)
        self.assertIs(validate_manifest(manifest), manifest)

    def test_profile_cannot_disable_receipt_requirement(self):
        weakened = profile()
        weakened["require_receipt_for_pass"] = False
        with self.assertRaisesRegex(SourceAdapterError, "exact keys required"):
            normalize_github_qualification_source_v1(weakened, source())

    def test_unproven_nonterminal_runs_are_unsupported(self):
        for status in ("queued", "in_progress"):
            normalized = normalize_github_qualification_source_v1(
                profile(),
                source(
                    workflow_status=status,
                    workflow_conclusion=None,
                    tested_subject_sha=None,
                    tested_tree_sha=None,
                    exact_subject_assertion="unproven",
                    receipt_sha256=None,
                    receipt_subject_sha=None,
                    receipt_profile=None,
                ),
            )
            self.assertFalse(normalized["exact_subject_proven"])
            self.assertEqual(normalized["disposition"], "UNSUPPORTED")

    def test_exact_proven_nonterminal_runs_are_pending(self):
        for status in ("queued", "in_progress"):
            normalized = normalize_github_qualification_source_v1(
                profile(),
                source(
                    workflow_status=status,
                    workflow_conclusion=None,
                    receipt_sha256=None,
                    receipt_subject_sha=None,
                    receipt_profile=None,
                ),
            )
            self.assertTrue(normalized["exact_subject_proven"])
            self.assertEqual(normalized["disposition"], "PENDING")

    def test_run_level_failure_is_not_semantic_red(self):
        normalized = normalize_github_qualification_source_v1(
            profile(),
            source(
                workflow_conclusion="failure",
                receipt_sha256=None,
                receipt_subject_sha=None,
                receipt_profile=None,
            ),
        )
        self.assertTrue(normalized["exact_subject_proven"])
        self.assertEqual(normalized["disposition"], "NOT_ASSESSED")

    def test_other_terminal_results_are_not_semantic_red(self):
        for conclusion in ("cancelled", "skipped", "timed_out", "action_required"):
            normalized = normalize_github_qualification_source_v1(
                profile(),
                source(
                    workflow_conclusion=conclusion,
                    receipt_sha256=None,
                    receipt_subject_sha=None,
                    receipt_profile=None,
                ),
            )
            self.assertEqual(normalized["disposition"], "NOT_ASSESSED")

    def test_unproven_subject_success_and_failure_are_symmetric(self):
        for conclusion in ("success", "failure"):
            normalized = normalize_github_qualification_source_v1(
                profile(),
                source(
                    workflow_conclusion=conclusion,
                    tested_subject_sha=None,
                    tested_tree_sha=None,
                    exact_subject_assertion="unproven",
                    receipt_sha256=None,
                    receipt_subject_sha=None,
                    receipt_profile=None,
                ),
            )
            self.assertFalse(normalized["exact_subject_proven"])
            self.assertEqual(normalized["disposition"], "UNSUPPORTED")

    def test_merge_ref_only_does_not_become_exact_head_result(self):
        for conclusion in ("success", "failure"):
            normalized = normalize_github_qualification_source_v1(
                profile(),
                source(
                    workflow_conclusion=conclusion,
                    tested_subject_sha=SHA_B,
                    exact_subject_assertion="merge_ref_only",
                    receipt_sha256=None,
                    receipt_subject_sha=None,
                    receipt_profile=None,
                ),
            )
            self.assertEqual(normalized["run_head_sha"], SHA_A)
            self.assertEqual(normalized["tested_subject_sha"], SHA_B)
            self.assertEqual(normalized["disposition"], "UNSUPPORTED")

    def test_detached_replay_preserves_workflow_head_and_exact_tested_subject(self):
        normalized = normalize_github_qualification_source_v1(
            profile(), source(run_head_sha=SHA_B, exact_subject_assertion="detached_exact_replay")
        )
        self.assertEqual(normalized["run_head_sha"], SHA_B)
        self.assertEqual(normalized["tested_subject_sha"], SHA_A)
        self.assertTrue(normalized["exact_subject_proven"])
        self.assertEqual(normalized["disposition"], "PASS")

    def test_tree_equivalence_is_not_commit_identity(self):
        normalized = normalize_github_qualification_source_v1(
            profile(),
            source(
                run_head_sha=SHA_B,
                tested_subject_sha=SHA_B,
                tested_tree_sha=TREE_A,
                exact_subject_assertion="tree_equivalence_only",
                receipt_sha256=None,
                receipt_subject_sha=None,
                receipt_profile=None,
            ),
        )
        self.assertFalse(normalized["exact_subject_proven"])
        self.assertEqual(normalized["disposition"], "UNSUPPORTED")

    def test_missing_receipt_cannot_pass(self):
        normalized = normalize_github_qualification_source_v1(
            profile(), source(receipt_sha256=None, receipt_subject_sha=None, receipt_profile=None)
        )
        self.assertTrue(normalized["exact_subject_proven"])
        self.assertEqual(normalized["disposition"], "NOT_ASSESSED")

    def test_receipt_cannot_substitute_for_missing_tested_subject(self):
        with self.assertRaisesRegex(SourceAdapterError, "cannot substitute"):
            normalize_github_qualification_source_v1(
                profile(),
                source(
                    tested_subject_sha=None,
                    tested_tree_sha=None,
                    exact_subject_assertion="unproven",
                ),
            )

    def test_receipt_subject_and_profile_mismatch_reject(self):
        with self.assertRaisesRegex(SourceAdapterError, "receipt/tested-subject mismatch"):
            normalize_github_qualification_source_v1(profile(), source(receipt_subject_sha=SHA_B))
        with self.assertRaisesRegex(SourceAdapterError, "profile mismatch"):
            normalize_github_qualification_source_v1(
                profile(), source(receipt_profile="other-receipt-v1")
            )

    def test_attempt_zero_is_rejected(self):
        with self.assertRaisesRegex(SourceAdapterError, "positive integer"):
            normalize_github_qualification_source_v1(profile(), source(run_attempt=0))

    def test_repository_workflow_and_event_substitution_reject(self):
        cases = [
            ({"repository_id": 1}, "repository mismatch"),
            ({"workflow_path": ".github/workflows/other.yml"}, "workflow path mismatch"),
            ({"workflow_event": "workflow_dispatch"}, "event not admitted"),
        ]
        for changes, message in cases:
            with self.subTest(changes=changes):
                with self.assertRaisesRegex(SourceAdapterError, message):
                    normalize_github_qualification_source_v1(profile(), source(**changes))

    def test_source_evidence_identity_is_required_and_preserved(self):
        normalized = normalize_github_qualification_source_v1(profile(), source())
        self.assertEqual(normalized["source_evidence_id"], "github:run42:attempt1:fixture")
        with self.assertRaisesRegex(SourceAdapterError, "expected non-empty trimmed string"):
            normalize_github_qualification_source_v1(profile(), source(source_evidence_id=""))

    def test_semantically_equal_projection_does_not_collapse_source_identity(self):
        first = normalize_github_qualification_source_v1(profile(), source())
        second_source = source(
            run_attempt=2, source_evidence_id="github:run42:attempt2:fixture"
        )
        second = normalize_github_qualification_source_v1(profile(), second_source)
        first_record = project_github_technical_evidence_v1(
            profile(), source(), evidence_id="AGENT-002", designated_current_subject_sha=SHA_A
        )
        second_record = project_github_technical_evidence_v1(
            profile(), second_source, evidence_id="AGENT-002", designated_current_subject_sha=SHA_A
        )
        self.assertEqual(first_record, second_record)
        self.assertNotEqual(first["source_evidence_id"], second["source_evidence_id"])
        self.assertNotEqual(first["run_attempt"], second["run_attempt"])
        self.assertNotEqual(
            normalized_source_sha256_v1(first), normalized_source_sha256_v1(second)
        )

    def test_historical_pass_remains_pass_while_currentness_is_separate(self):
        record = project_github_technical_evidence_v1(
            profile(), source(), evidence_id="AGENT-002", designated_current_subject_sha=SHA_B
        )
        self.assertEqual(record["disposition"], "PASS")
        self.assertEqual(record["subject_sha"], SHA_A)
        self.assertEqual(record["designated_current_subject_sha"], SHA_B)
        validate_manifest(manifest_for(record))

    def test_canonical_source_projection_is_deterministic(self):
        normalized = normalize_github_qualification_source_v1(profile(), source())
        clone = copy.deepcopy(normalized)
        self.assertEqual(
            canonical_normalized_source_bytes_v1(normalized),
            canonical_normalized_source_bytes_v1(clone),
        )
        self.assertEqual(
            normalized_source_sha256_v1(normalized), normalized_source_sha256_v1(clone)
        )


if __name__ == "__main__":
    unittest.main()
