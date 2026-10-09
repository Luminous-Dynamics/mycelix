import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "qualification"))
from check_sym_civic_019_trusted_selftest_report import Reject, strict_json, validate_report  # noqa: E402


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


def path_length(tree_size, leaf_index):
    if tree_size == 1:
        return 0
    split = 1 << ((tree_size - 1).bit_length() - 1)
    if leaf_index < split:
        return path_length(split, leaf_index) + 1
    return path_length(tree_size - split, leaf_index - split) + 1


def valid_fixture():
    trusted_manifest = {
        "schema": "MYCELIX-SYM-CIVIC-019-TRUSTED-ADMISSION-V1",
        "qualification_workflow": {},
        "trusted_admission_workflow": {},
        "pr_number": 4671,
        "parent_subject": "parent",
        "candidate_subject": "candidate",
        "expected_changed_files": [],
        "expected_blob_sha": {},
        "required_successful_jobs": [],
        "semantic_expectations": {},
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
    report = {
        "schema": "MYCELIX-SYM-CIVIC-019-TRUSTED-SEMANTIC-ORACLE-V6",
        "claim_ceiling": "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
        "candidate_input_only": True,
        "candidate_code_executed": False,
        "profile": "RFC9162_SHA256_SYNTHETIC_V1",
        "vds_id": 1,
        "vdp_id": -1,
        "receipt_a": {},
        "receipt_b": {},
        "statement_sha256": "a" * 64,
        "statement_issuer": "https://arp.synthetic.mycelix",
        "statement_subject": "stmt-019-001",
        "entry_sha256": "b" * 64,
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
        "candidate_qualification_manifest_sha256": "c" * 64,
        "candidate_corpus_git_blob_sha": "d" * 40,
        "issuer_signature_verification": "NOT_EVALUATED",
        "result": "PASS",
    }
    return report, trusted_manifest


class TrustedSelfTestReportCheckerTests(unittest.TestCase):
    def setUp(self):
        self.report, self.manifest = valid_fixture()

    def rejected(self, report=None, manifest=None):
        with self.assertRaises(Reject):
            validate_report(
                self.report if report is None else report,
                self.manifest if manifest is None else manifest,
            )

    def test_valid_fixture_is_accepted(self):
        validate_report(self.report, self.manifest)

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
        report["positive_inclusion_matrix"]["cases"][1]["leaf_index"] = 0
        self.rejected(report)

    def test_boolean_does_not_count_as_integer_leaf_index(self):
        report = copy.deepcopy(self.report)
        report["positive_inclusion_matrix"]["cases"][0]["leaf_index"] = False
        self.rejected(report)

    def test_wrong_negative_control_reason_is_rejected(self):
        report = copy.deepcopy(self.report)
        report["negative_controls"][0]["reason"] = "some other reason"
        self.rejected(report)

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
