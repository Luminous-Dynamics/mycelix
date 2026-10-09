import copy
import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "qualification"))
from check_sym_civic_019_trusted_selftest_report import (  # noqa: E402
    Reject,
    git_blob_sha,
    strict_json,
    validate_report,
    verify_candidate_input_bindings,
)


ROOTS = {str(size): f"{size:064x}" for size in range(1, 17)}
NEGATIVE_REASONS = {
    "WRONG_ENTRY_BYTES": "inclusion root mismatch",
    "RECEIPT_SIGNATURE_MUTATION": "Receipt signature verification failed",
    "LEAF_INDEX_EQUALS_TREE_SIZE": "leaf_index out of range",
    "PROOF_PATH_EXTRA_NODE": "unused inclusion path nodes",
    "EMBEDDED_ROOT_PAYLOAD_MUTATION": "inclusion root mismatch",
    "VDS_SELECTOR_MUTATION": "Receipt VDS mismatch",
    "MALFORMED_VDP_SHAPE": "Receipt VDP schema mismatch",
    "DUPLICATE_COSE_HEADER_LABEL": "duplicate CBOR map key",
    "NON_MINIMAL_CBOR": "non-minimal CBOR",
    "PATH_NODE_WRONG_LENGTH": "path node must be 32 bytes",
    "TREE_SIZE_ABOVE_LIMIT": "tree_size exceeds local synthetic profile limit",
}
CORPUS_PATH = "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json"
QUALIFICATION_MANIFEST_PATH = "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json"
EXPECTED_BLOB_PATHS = {
    ".github/workflows/sym-civic-019-receipt-proof-binding.yml",
    "mycelix-workspace/docs/civic-resilience/scitt-cose-v1/manifest.json",
    QUALIFICATION_MANIFEST_PATH,
    CORPUS_PATH,
    "scripts/qualification/sym_civic_019_receipt_proof_binding_v1.py",
    "scripts/qualification/sym_civic_019_scitt_cose_interop_v1.py",
}


def path_length(tree_size, leaf_index):
    if tree_size == 1:
        return 0
    split = 1 << ((tree_size - 1).bit_length() - 1)
    if leaf_index < split:
        return path_length(split, leaf_index) + 1
    return path_length(tree_size - split, leaf_index - split) + 1


def valid_fixture(candidate_root):
    corpus_bytes = b"synthetic candidate corpus fixture\n"
    qualification_manifest_bytes = b'{"schema":"synthetic qualification fixture"}\n'
    (candidate_root / CORPUS_PATH).parent.mkdir(parents=True, exist_ok=True)
    (candidate_root / QUALIFICATION_MANIFEST_PATH).parent.mkdir(parents=True, exist_ok=True)
    (candidate_root / CORPUS_PATH).write_bytes(corpus_bytes)
    (candidate_root / QUALIFICATION_MANIFEST_PATH).write_bytes(qualification_manifest_bytes)

    expected_blobs = {path: "f" * 40 for path in EXPECTED_BLOB_PATHS}
    expected_blobs[CORPUS_PATH] = git_blob_sha(corpus_bytes)
    expected_blobs[QUALIFICATION_MANIFEST_PATH] = git_blob_sha(qualification_manifest_bytes)

    root_hash = ROOTS["1"]
    identities = {
        "A": {
            "kid_hex": "54532d412d303139",
            "issuer": "https://ts-a.synthetic.mycelix",
            "public_key_pem": "synthetic-fixture-public-key-a",
        },
        "B": {
            "kid_hex": "54532d422d303139",
            "issuer": "https://ts-b.synthetic.mycelix",
            "public_key_pem": "synthetic-fixture-public-key-b",
        },
    }
    statement_sha = "a" * 64
    receipt_a_sha = "e" * 64
    receipt_b_sha = "f" * 64
    semantic = {
        "profile": "RFC9162_SHA256_SYNTHETIC_V1",
        "vds_id": 1,
        "vdp_id": -1,
        "entry_sha256": statement_sha,
        "statement_sha256": statement_sha,
        "receipt_a_sha256": receipt_a_sha,
        "receipt_b_sha256": receipt_b_sha,
        "root_hash": root_hash,
        "merkle_root": "b" * 64,
        "tree_size": 1,
        "leaf_index": 0,
        "subject": "stmt-019-001",
        "identities": identities,
        "statement_issuer": "https://arp.synthetic.mycelix",
    }
    trusted_manifest = {
        "schema": "MYCELIX-SYM-CIVIC-019-TRUSTED-ADMISSION-V1",
        "qualification_workflow": {},
        "trusted_admission_workflow": {},
        "pr_number": 4671,
        "parent_subject": "parent",
        "candidate_subject": "candidate",
        "expected_changed_files": [],
        "expected_blob_sha": expected_blobs,
        "required_successful_jobs": [],
        "semantic_expectations": semantic,
        "trusted_semantic_oracle": {
            "path": "scripts/qualification/verify_sym_civic_019_trusted_semantics_v1.py",
            "claim_ceiling": "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
            "candidate_code_executed": False,
            "negative_control_count": 11,
            "tree_size_max_inclusive": 2**62,
            "positive_inclusion_matrix": {
                "tree_sizes": list(range(1, 17)),
                "expected_case_count": 136,
                "expected_root_sha256_by_tree_size": ROOTS,
            },
        },
    }

    cases = []
    for size in range(1, 17):
        for index in range(size):
            root = ROOTS[str(size)]
            cases.append({
                "tree_size": size,
                "leaf_index": index,
                "path_length": path_length(size, index),
                "expected_root_sha256": root,
                "independently_computed_root_sha256": root,
                "reference_root_matches_trusted": True,
                "observed_root_sha256": root,
                "verdict": "PASS",
            })

    def receipt_summary(label, receipt_sha):
        identity = identities[label]
        return {
            "receipt_sha256": receipt_sha,
            "protected_sha256": "1" * 64,
            "kid_hex": identity["kid_hex"],
            "issuer": identity["issuer"],
            "subject": semantic["subject"],
            "tree_size": semantic["tree_size"],
            "leaf_index": semantic["leaf_index"],
            "root_hash": semantic["root_hash"],
            "proof_sha256": "2" * 64,
        }

    report = {
        "schema": "MYCELIX-SYM-CIVIC-019-TRUSTED-SEMANTIC-ORACLE-V6",
        "claim_ceiling": "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
        "candidate_input_only": True,
        "candidate_code_executed": False,
        "profile": semantic["profile"],
        "vds_id": semantic["vds_id"],
        "vdp_id": semantic["vdp_id"],
        "receipt_a": receipt_summary("A", receipt_a_sha),
        "receipt_b": receipt_summary("B", receipt_b_sha),
        "statement_sha256": statement_sha,
        "statement_issuer": semantic["statement_issuer"],
        "statement_subject": semantic["subject"],
        "entry_sha256": semantic["entry_sha256"],
        "independent_merkle_fixture": "PASS",
        "positive_inclusion_matrix": {
            "tree_sizes_tested": list(range(1, 17)),
            "case_count": 136,
            "all_pass": True,
            "trusted_expected_roots_sha256_by_tree_size": ROOTS,
            "reference_roots_all_match_trusted": True,
            "cases": cases,
        },
        "negative_controls": [
            {"name": name, "verdict": "REJECT", "reason": reason}
            for name, reason in NEGATIVE_REASONS.items()
        ],
        "negative_control_count": 11,
        "tree_size_max_inclusive": 2**62,
        "tree_size_limit_scope": "LOCAL_SYNTHETIC_PROFILE_NOT_RFC_REQUIREMENT",
        "negative_controls_all_rejected": True,
        "candidate_qualification_manifest_sha256": hashlib.sha256(qualification_manifest_bytes).hexdigest(),
        "candidate_corpus_git_blob_sha": expected_blobs[CORPUS_PATH],
        "issuer_signature_verification": "NOT_EVALUATED",
        "result": "PASS",
    }
    return report, trusted_manifest


class TrustedSelfTestReportCheckerTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.candidate_root = Path(self.tempdir.name)
        self.report, self.manifest = valid_fixture(self.candidate_root)

    def validate(self, report=None, manifest=None):
        report = self.report if report is None else report
        manifest = self.manifest if manifest is None else manifest
        validate_report(report, manifest)
        verify_candidate_input_bindings(report, manifest, self.candidate_root)

    def rejected(self, report=None, manifest=None):
        with self.assertRaises(Reject):
            self.validate(report, manifest)

    def test_valid_fixture_is_accepted(self):
        self.validate()

    def test_extra_top_level_field_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["unreviewed_claim"] = True
        self.rejected(report)

    def test_extra_nested_matrix_field_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["positive_inclusion_matrix"]["debug"] = "unreviewed"
        self.rejected(report)

    def test_uppercase_digest_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["statement_sha256"] = "A" * 64
        self.rejected(report)

    def test_malformed_git_blob_id_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["candidate_corpus_git_blob_sha"] = "g" * 40
        self.rejected(report)

    def test_missing_positive_case_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["positive_inclusion_matrix"]["cases"].pop()
        self.rejected(report)

    def test_duplicate_positive_pair_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["positive_inclusion_matrix"]["cases"][2]["leaf_index"] = 0
        self.rejected(report)

    def test_boolean_does_not_count_as_integer_leaf_index(self):
        report = copy.deepcopy(self.report)
        report["positive_inclusion_matrix"]["cases"][0]["leaf_index"] = False
        self.rejected(report)

    def test_wrong_negative_control_reason_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["negative_controls"][0]["reason"] = "some other reason"
        self.rejected(report)

    def test_receipt_issuer_must_match_trusted_identity(self):
        report = copy.deepcopy(self.report)
        report["receipt_a"]["issuer"] = "https://attacker.invalid"
        self.rejected(report)

    def test_report_candidate_digest_must_match_actual_manifest_bytes(self):
        report = copy.deepcopy(self.report)
        report["candidate_qualification_manifest_sha256"] = "9" * 64
        self.rejected(report)

    def test_candidate_corpus_tampering_is_rejected(self):
        (self.candidate_root / CORPUS_PATH).write_bytes(b"tampered corpus\n")
        with self.assertRaisesRegex(Reject, "actual bytes"):
            verify_candidate_input_bindings(self.report, self.manifest, self.candidate_root)

    def test_candidate_manifest_tampering_is_rejected(self):
        (self.candidate_root / QUALIFICATION_MANIFEST_PATH).write_bytes(b'{"tampered":true}\n')
        with self.assertRaisesRegex(Reject, "actual bytes"):
            verify_candidate_input_bindings(self.report, self.manifest, self.candidate_root)

    def test_duplicate_json_members_are_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "duplicate.json"
            path.write_text('{"schema":"first","schema":"second"}', encoding="utf-8")
            with self.assertRaisesRegex(Reject, "duplicate JSON member"):
                strict_json(path)

    def test_nonstandard_nan_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "nan.json"
            path.write_text('{"claim": NaN}', encoding="utf-8")
            with self.assertRaisesRegex(Reject, "non-standard JSON constant"):
                strict_json(path)


if __name__ == "__main__":
    unittest.main()
