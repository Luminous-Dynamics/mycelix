from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mycelix_github_qualification_source import PROFILE_VERSION as SOURCE_PROFILE_VERSION  # noqa: E402
from mycelix_provider_source_evidence import (  # noqa: E402
    GITHUB_QUALIFICATION_SOURCE_KIND,
    PROFILE_VERSION,
    SourceEvidenceError,
    bind_github_qualification_source_v1,
    canonical_bound_source_bytes_v1,
    normalize_bound_github_qualification_source_v1,
    validate_source_evidence_envelope_v1,
    validate_source_evidence_set_v1,
)

REPOSITORY_ID = 1176351975
SHA_A = "a" * 40
TREE_A = "b" * 40
RECEIPT = "c" * 64
EVIDENCE_ID = "github:run42:attempt1:fixture"


def policy():
    return {
        "profile_version": PROFILE_VERSION,
        "provider_profile": "github-actions-rest-v1",
        "acquisition_profile": "authenticated-github-api-plus-artifact-v1",
        "source_kind": GITHUB_QUALIFICATION_SOURCE_KIND,
        "repository_id": REPOSITORY_ID,
    }


def source_profile():
    return {
        "profile_version": SOURCE_PROFILE_VERSION,
        "source_profile": "github-qualification-source-v1",
        "technical_profile": "qual-v1",
        "repository_id": REPOSITORY_ID,
        "workflow_path": ".github/workflows/qual.yml",
        "designated_subject_sha": SHA_A,
        "designated_subject_tree_sha": TREE_A,
        "allowed_workflow_events": ["push"],
        "expected_receipt_profile": "qual-receipt-v1",
    }


def object_refs():
    return [
        {"kind": "workflow_run", "reference": "api:actions/runs/42"},
        {"kind": "workflow_jobs", "reference": "api:actions/runs/42/jobs"},
        {
            "kind": "tested_subject_proof",
            "reference": "receipt:tested-subject-proof:42:1",
        },
        {
            "kind": "qualification_receipt",
            "reference": "artifact:qualification-receipt:9001",
        },
    ]


def envelope(**changes):
    value = {
        "profile_version": PROFILE_VERSION,
        "provider_profile": "github-actions-rest-v1",
        "acquisition_profile": "authenticated-github-api-plus-artifact-v1",
        "source_kind": GITHUB_QUALIFICATION_SOURCE_KIND,
        "repository_id": REPOSITORY_ID,
        "provider_object_refs": object_refs(),
        "run_id": 42,
        "run_attempt": 1,
        "evidence_id": EVIDENCE_ID,
    }
    value.update(changes)
    return value


def source(**changes):
    value = {
        "provider": "github",
        "source_evidence_id": EVIDENCE_ID,
        "repository_id": REPOSITORY_ID,
        "workflow_path": ".github/workflows/qual.yml",
        "workflow_event": "push",
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
        "source_references": [item["reference"] for item in object_refs()],
    }
    value.update(changes)
    return value


class ProviderSourceEvidenceTests(unittest.TestCase):
    def test_valid_envelope_binds_before_normalization(self):
        bound = normalize_bound_github_qualification_source_v1(
            policy(), envelope(), source_profile(), source()
        )
        self.assertEqual(bound["source_evidence"]["evidence_id"], EVIDENCE_ID)
        self.assertEqual(bound["source_evidence"]["run_id"], 42)
        self.assertEqual(bound["source_evidence"]["run_attempt"], 1)
        self.assertEqual(bound["normalized_source"]["disposition"], "PASS")

    def test_policy_substitution_rejects(self):
        cases = [
            ({"provider_profile": "other-provider-v1"}, "provider_profile"),
            ({"acquisition_profile": "other-acquisition-v1"}, "acquisition_profile"),
            ({"source_kind": "other-source-kind-v1"}, "source_kind"),
            ({"repository_id": 1}, "repository_id"),
        ]
        for changes, field in cases:
            with self.subTest(field=field):
                bad = envelope(**changes)
                with self.assertRaisesRegex(SourceEvidenceError, "policy mismatch"):
                    validate_source_evidence_envelope_v1(policy(), bad)

    def test_run_attempt_repository_and_evidence_id_substitution_reject(self):
        cases = [
            ({"repository_id": 1}, "repository_id"),
            ({"run_id": 43}, "run_id"),
            ({"run_attempt": 2}, "run_attempt"),
            ({"source_evidence_id": "github:run42:attempt2:fixture"}, "source_evidence_id"),
        ]
        for changes, field in cases:
            with self.subTest(field=field):
                with self.assertRaisesRegex(SourceEvidenceError, "envelope mismatch"):
                    bind_github_qualification_source_v1(
                        policy(), envelope(), source(**changes)
                    )

    def test_every_envelope_reference_must_be_present_in_raw_source(self):
        refs = source()["source_references"][:-1]
        with self.assertRaisesRegex(SourceEvidenceError, "missing envelope references"):
            bind_github_qualification_source_v1(
                policy(), envelope(), source(source_references=refs)
            )

    def test_receipt_bearing_source_requires_receipt_or_artifact_provenance(self):
        refs = [item for item in object_refs() if item["kind"] != "qualification_receipt"]
        raw = source(source_references=[item["reference"] for item in refs])
        with self.assertRaisesRegex(SourceEvidenceError, "receipt/artifact provenance"):
            bind_github_qualification_source_v1(
                policy(), envelope(provider_object_refs=refs), raw
            )

    def test_exact_subject_assertion_requires_proof_provenance(self):
        refs = [item for item in object_refs() if item["kind"] != "tested_subject_proof"]
        raw = source(source_references=[item["reference"] for item in refs])
        with self.assertRaisesRegex(SourceEvidenceError, "tested-subject proof provenance"):
            bind_github_qualification_source_v1(
                policy(), envelope(provider_object_refs=refs), raw
            )

    def test_unproven_subject_does_not_require_tested_subject_proof(self):
        refs = [
            item
            for item in object_refs()
            if item["kind"] not in {"tested_subject_proof", "qualification_receipt"}
        ]
        raw = source(
            tested_subject_sha=None,
            tested_tree_sha=None,
            exact_subject_assertion="unproven",
            receipt_sha256=None,
            receipt_subject_sha=None,
            receipt_profile=None,
            workflow_conclusion="failure",
            source_references=[item["reference"] for item in refs],
        )
        bound = normalize_bound_github_qualification_source_v1(
            policy(), envelope(provider_object_refs=refs), source_profile(), raw
        )
        self.assertEqual(bound["normalized_source"]["disposition"], "UNSUPPORTED")

    def test_duplicate_provider_object_reference_rejects(self):
        refs = object_refs()
        refs.append(copy.deepcopy(refs[0]))
        with self.assertRaisesRegex(SourceEvidenceError, "duplicate object reference"):
            validate_source_evidence_envelope_v1(
                policy(), envelope(provider_object_refs=refs)
            )

    def test_provider_object_order_is_not_semantic(self):
        first = validate_source_evidence_envelope_v1(policy(), envelope())
        reversed_refs = list(reversed(object_refs()))
        second = validate_source_evidence_envelope_v1(
            policy(), envelope(provider_object_refs=reversed_refs)
        )
        self.assertEqual(first, second)

    def test_same_evidence_id_cannot_name_different_envelopes(self):
        first = envelope()
        second = envelope(run_attempt=2)
        with self.assertRaisesRegex(SourceEvidenceError, "one evidence identity"):
            validate_source_evidence_set_v1(policy(), [first, second])

    def test_duplicate_identical_envelope_collapses_in_bounded_set(self):
        result = validate_source_evidence_set_v1(
            policy(), [envelope(), copy.deepcopy(envelope())]
        )
        self.assertEqual(len(result), 1)

    def test_new_attempt_with_new_identity_is_preserved_separately(self):
        second = envelope(
            run_attempt=2,
            evidence_id="github:run42:attempt2:fixture",
        )
        result = validate_source_evidence_set_v1(policy(), [envelope(), second])
        self.assertEqual(len(result), 2)
        self.assertEqual([item["run_attempt"] for item in result], [1, 2])

    def test_canonical_bound_bytes_must_pass_full_validation_path(self):
        first = canonical_bound_source_bytes_v1(
            policy(), envelope(), source_profile(), source()
        )
        second = canonical_bound_source_bytes_v1(
            policy(), copy.deepcopy(envelope()), source_profile(), copy.deepcopy(source())
        )
        self.assertEqual(first, second)

        with self.assertRaises(SourceEvidenceError):
            canonical_bound_source_bytes_v1(
                policy(), envelope(run_attempt=2), source_profile(), source()
            )


if __name__ == "__main__":
    unittest.main()
