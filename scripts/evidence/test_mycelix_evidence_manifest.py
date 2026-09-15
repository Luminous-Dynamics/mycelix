from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mycelix_evidence_manifest import (  # noqa: E402
    MANIFEST_VERSION,
    ManifestError,
    manifest_sha256,
    render_markdown,
    validate_manifest,
)

SHA_A = "a" * 40
SHA_B = "b" * 40
RECEIPT = "c" * 64


def valid_manifest():
    return {
        "manifest_version": MANIFEST_VERSION,
        "technical_evidence": [
            {
                "id": "AGENT-002",
                "profile": "agent-runtime-v1",
                "subject_sha": SHA_A,
                "designated_current_subject_sha": SHA_A,
                "workflow_status": "completed",
                "workflow_conclusion": "success",
                "disposition": "PASS",
                "receipt_sha256": RECEIPT,
                "dependencies": [],
                "nonclaims": [
                    "Identity qualification is not effect authority."
                ],
            }
        ],
        "security_evidence": [
            {
                "id": "AUTH-EXT-001",
                "profile": "external-review-v1",
                "disposition": "PASS",
                "subject_ref": (
                    "agent-runtime-v1@"
                    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                ),
                "nonclaims": ["Review scope is bounded."],
            }
        ],
        "commercial_evidence": [
            {
                "id": "PILOT-001",
                "profile": "controlled-pilot-v1",
                "disposition": "PASS",
                "subject_ref": "design-partner:synth-001",
                "paid": True,
                "nonclaims": ["Pilot success is not product-market fit."],
            }
        ],
        "capital_evidence": [
            {
                "id": "CAP-001",
                "profile": "capital-source-v1",
                "disposition": "PASS",
                "subject_ref": "instrument:synth-001",
                "nonclaims": ["Funding is not technical qualification."],
            }
        ],
        "claims": [
            {
                "id": "claim-qualified",
                "text": "AGENT-002 exact subject is qualified.",
                "authority": "Qualified",
                "evidence_refs": ["technical:AGENT-002"],
                "model_ref": None,
                "nonclaims": [],
            },
            {
                "id": "claim-assured",
                "text": "The bounded subject received the named external review.",
                "authority": "ExternallyAssured",
                "evidence_refs": ["security:AUTH-EXT-001"],
                "model_ref": None,
                "nonclaims": [],
            },
        ],
        "nonclaims": ["This manifest does not establish valuation."],
    }


class ManifestTests(unittest.TestCase):
    def test_positive_manifest(self):
        manifest = valid_manifest()
        self.assertIs(validate_manifest(manifest), manifest)
        self.assertEqual(
            render_markdown(manifest),
            render_markdown(copy.deepcopy(manifest)),
        )
        self.assertEqual(
            manifest_sha256(manifest),
            manifest_sha256(copy.deepcopy(manifest)),
        )

    def test_queued_cannot_be_pass(self):
        manifest = valid_manifest()
        item = manifest["technical_evidence"][0]
        item["workflow_status"] = "queued"
        item["workflow_conclusion"] = None
        with self.assertRaisesRegex(ManifestError, "cannot be PASS"):
            validate_manifest(manifest)

    def test_failed_workflow_cannot_be_pass(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0]["workflow_conclusion"] = "failure"
        with self.assertRaisesRegex(ManifestError, "PASS requires completed/success"):
            validate_manifest(manifest)

    def test_pass_requires_receipt(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0]["receipt_sha256"] = None
        with self.assertRaisesRegex(ManifestError, "PASS requires receipt_sha256"):
            validate_manifest(manifest)

    def test_historical_pass_is_preserved_but_not_current(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0][
            "designated_current_subject_sha"
        ] = SHA_B
        manifest["claims"][0]["authority"] = "Observed"
        validate_manifest(manifest)
        self.assertIn("Historical/Stale", render_markdown(manifest))

    def test_stale_pass_cannot_support_current_qualified_claim(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0][
            "designated_current_subject_sha"
        ] = SHA_B
        with self.assertRaisesRegex(ManifestError, "current technical PASS"):
            validate_manifest(manifest)

    def test_customer_evidence_cannot_self_promote_to_qualified(self):
        manifest = valid_manifest()
        manifest["claims"][0]["evidence_refs"] = ["commercial:PILOT-001"]
        with self.assertRaisesRegex(ManifestError, "current technical PASS"):
            validate_manifest(manifest)

    def test_capital_evidence_cannot_self_promote_to_qualified(self):
        manifest = valid_manifest()
        manifest["claims"][0]["evidence_refs"] = ["capital:CAP-001"]
        with self.assertRaisesRegex(ManifestError, "current technical PASS"):
            validate_manifest(manifest)

    def test_external_assurance_requires_security_pass(self):
        manifest = valid_manifest()
        manifest["security_evidence"][0]["disposition"] = "NOT_ASSESSED"
        with self.assertRaisesRegex(ManifestError, "security PASS"):
            validate_manifest(manifest)

    def test_projected_requires_model_ref(self):
        manifest = valid_manifest()
        claim = manifest["claims"][0]
        claim["authority"] = "Projected"
        claim["model_ref"] = None
        with self.assertRaisesRegex(ManifestError, "requires model_ref"):
            validate_manifest(manifest)

    def test_unknown_keys_fail_closed(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0]["verified"] = True
        with self.assertRaisesRegex(ManifestError, "unknown keys"):
            validate_manifest(manifest)

    def test_malformed_sha_fails_closed(self):
        manifest = valid_manifest()
        manifest["technical_evidence"][0]["subject_sha"] = "ABC"
        with self.assertRaisesRegex(ManifestError, "40-hex"):
            validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
