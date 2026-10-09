#!/usr/bin/env python3
"""Strict, non-authoritative validator for the SYM-CIVIC-019 oracle self-test report."""
import argparse
import hashlib
import json
from pathlib import Path


class Reject(Exception):
    pass


def require(condition, message):
    if not condition:
        raise Reject(message)

def reject_duplicate_members(items):
    result = {}
    for key, value in items:
        if key in result:
            raise Reject("duplicate JSON member: " + key)
        result[key] = value
    return result

def reject_non_json_constant(value):
    raise Reject("non-standard JSON constant: " + value)


def strict_json(path):
    try:
        return json.loads(
            Path(path).read_text(encoding="utf-8"),
            object_pairs_hook=reject_duplicate_members,
            parse_constant=reject_non_json_constant,
        )
    except json.JSONDecodeError as exc:
        raise Reject("invalid JSON: " + str(exc)) from exc


def validate_report(report, trusted_manifest):
    require(isinstance(report, dict), "report must be a JSON object")
    require(
        set(report) == {
            "schema", "claim_ceiling", "candidate_input_only", "candidate_code_executed",
            "profile", "vds_id", "vdp_id", "receipt_a", "receipt_b",
            "statement_sha256", "statement_issuer", "statement_subject", "entry_sha256",
            "independent_merkle_fixture", "positive_inclusion_matrix", "negative_controls",
            "negative_control_count", "tree_size_max_inclusive", "tree_size_limit_scope",
            "negative_controls_all_rejected", "candidate_qualification_manifest_sha256",
            "candidate_corpus_git_blob_sha", "issuer_signature_verification", "result",
        },
        "report top-level schema must be exact",
    )
    require(report.get("schema") == "MYCELIX-SYM-CIVIC-019-TRUSTED-SEMANTIC-ORACLE-V6", "schema")
    require(report.get("claim_ceiling") == "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
            "claim ceiling")
    require(report.get("candidate_input_only") is True, "candidate-input-only claim")
    require(report.get("candidate_code_executed") is False, "candidate code execution claim")
    require(report.get("profile") == "RFC9162_SHA256_SYNTHETIC_V1", "profile")
    require(type(report.get("vds_id")) is int and report["vds_id"] == 1, "VDS identifier")
    require(type(report.get("vdp_id")) is int and report["vdp_id"] == -1, "VDP identifier")
    require(report.get("independent_merkle_fixture") == "PASS", "independent Merkle fixture")
    require(report.get("result") == "PASS", "overall result")
    require(report.get("statement_issuer") == "https://arp.synthetic.mycelix",
            "trusted Signed Statement issuer binding")
    require(report.get("statement_subject") == "stmt-019-001",
            "trusted Signed Statement subject binding")
    require(report.get("negative_control_count") == 11, "negative-control count")
    matrix = report.get("positive_inclusion_matrix")
    require(isinstance(matrix, dict), "positive inclusion matrix report")
    require(
        set(matrix) == {
            "tree_sizes_tested", "case_count", "all_pass",
            "trusted_expected_roots_sha256_by_tree_size",
            "reference_roots_all_match_trusted", "cases",
        },
        "positive inclusion matrix schema must be exact",
    )
    for digest_field in (
        "statement_sha256", "entry_sha256",
        "candidate_qualification_manifest_sha256",
    ):
        require(
            isinstance(report.get(digest_field), str)
            and len(report[digest_field]) == 64
            and all(ch in "0123456789abcdef" for ch in report[digest_field]),
            digest_field + " must be a lowercase SHA-256 digest",
        )
    require(
        isinstance(report.get("candidate_corpus_git_blob_sha"), str)
        and len(report["candidate_corpus_git_blob_sha"]) == 40
        and all(ch in "0123456789abcdef" for ch in report["candidate_corpus_git_blob_sha"]),
        "candidate corpus blob identity must be a lowercase Git SHA-1",
    )
    require(matrix.get("tree_sizes_tested") == list(range(1, 17)), "positive inclusion tree sizes")
    require(matrix.get("case_count") == 136, "positive inclusion case count")
    require(matrix.get("all_pass") is True, "positive inclusion matrix all pass")
    require(isinstance(trusted_manifest, dict), "trusted manifest must be a JSON object")
    require(trusted_manifest.get("schema") == "MYCELIX-SYM-CIVIC-019-TRUSTED-ADMISSION-V1",
            "trusted manifest schema")
    require(
        set(trusted_manifest) == {
            "schema", "qualification_workflow", "trusted_admission_workflow",
            "pr_number", "parent_subject", "candidate_subject", "expected_changed_files",
            "expected_blob_sha", "required_successful_jobs", "semantic_expectations",
            "trusted_semantic_oracle",
        },
        "trusted manifest top-level schema must be exact",
    )
    trusted_oracle = trusted_manifest["trusted_semantic_oracle"]
    require(isinstance(trusted_oracle, dict), "trusted oracle must be an object")
    require(
        set(trusted_oracle) == {
            "path", "claim_ceiling", "candidate_code_executed", "negative_control_count",
            "tree_size_max_inclusive", "positive_inclusion_matrix",
        },
        "trusted oracle schema must be exact",
    )
    require(trusted_oracle.get("path") == "scripts/qualification/verify_sym_civic_019_trusted_semantics_v1.py",
            "trusted oracle path")
    require(trusted_oracle.get("claim_ceiling") == "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
            "trusted oracle claim ceiling")
    require(trusted_oracle.get("candidate_code_executed") is False,
            "trusted oracle candidate-code execution claim")
    require(type(trusted_oracle.get("negative_control_count")) is int
            and trusted_oracle["negative_control_count"] == 11,
            "trusted oracle negative-control count")
    require(type(trusted_oracle.get("tree_size_max_inclusive")) is int
            and trusted_oracle["tree_size_max_inclusive"] == 2**62,
            "trusted oracle tree-size cap")
    trusted_matrix = trusted_oracle["positive_inclusion_matrix"]
    require(isinstance(trusted_matrix, dict), "trusted matrix must be an object")
    require(
        set(trusted_matrix) == {
            "tree_sizes", "expected_case_count", "expected_root_sha256_by_tree_size",
        },
        "trusted matrix schema must be exact",
    )
    require(trusted_matrix.get("tree_sizes") == list(range(1, 17)),
            "trusted matrix tree sizes must be exact")
    require(type(trusted_matrix.get("expected_case_count")) is int
            and trusted_matrix["expected_case_count"] == 136,
            "trusted matrix case count must be exact")
    trusted_roots = trusted_matrix["expected_root_sha256_by_tree_size"]
    require(isinstance(trusted_roots, dict), "trusted root vector must be an object")
    require(set(trusted_roots) == {str(size) for size in range(1, 17)},
            "trusted root vector keys must be exact")
    require(
        all(
            isinstance(digest, str)
            and len(digest) == 64
            and all(ch in "0123456789abcdef" for ch in digest)
            for digest in trusted_roots.values()
        ),
        "every trusted root commitment must be a lowercase SHA-256 digest",
    )

    semantic = trusted_manifest["semantic_expectations"]
    require(isinstance(semantic, dict), "semantic expectations must be an object")
    require(
        set(semantic) == {
            "profile", "vds_id", "vdp_id", "entry_sha256", "statement_sha256",
            "receipt_a_sha256", "receipt_b_sha256", "root_hash", "merkle_root",
            "tree_size", "leaf_index", "subject", "identities", "statement_issuer",
        },
        "semantic expectations schema must be exact",
    )
    for key in ("profile", "vds_id", "vdp_id", "entry_sha256", "statement_sha256",
                "statement_issuer", "subject"):
        require(report.get({
            "subject": "statement_subject",
        }.get(key, key)) == semantic[key], "report semantic binding: " + key)
    require(report.get("statement_sha256") == report.get("entry_sha256"),
            "Signed Statement bytes and entry digest must be bound")
    require(
        report.get("tree_size_max_inclusive") == trusted_oracle["tree_size_max_inclusive"],
        "report tree-size cap must equal trusted profile",
    )
    expected_blob_paths = {
        ".github/workflows/sym-civic-019-receipt-proof-binding.yml",
        "mycelix-workspace/docs/civic-resilience/scitt-cose-v1/manifest.json",
        "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json",
        "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json",
        "scripts/qualification/sym_civic_019_receipt_proof_binding_v1.py",
        "scripts/qualification/sym_civic_019_scitt_cose_interop_v1.py",
    }
    expected_blobs = trusted_manifest["expected_blob_sha"]
    require(isinstance(expected_blobs, dict) and set(expected_blobs) == expected_blob_paths,
            "trusted candidate blob map schema must be exact")
    require(
        all(
            isinstance(blob_sha, str)
            and len(blob_sha) == 40
            and all(ch in "0123456789abcdef" for ch in blob_sha)
            for blob_sha in expected_blobs.values()
        ),
        "trusted candidate Git blob IDs must be lowercase SHA-1 hex",
    )
    corpus_relpath = "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json"
    qualification_manifest_relpath = "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json"
    require(report.get("candidate_corpus_git_blob_sha") == expected_blobs[corpus_relpath],
            "candidate corpus Git blob identity must equal the trusted manifest pin")
    require(
        matrix.get("trusted_expected_roots_sha256_by_tree_size") == trusted_roots,
        "report root commitments must equal trusted manifest",
    )
    require(
        matrix.get("reference_roots_all_match_trusted") is True,
        "all independently computed reference roots must match trusted commitments",
    )
    cases = matrix.get("cases")
    require(isinstance(cases, list) and len(cases) == 136,
            "positive inclusion matrix rows")
    require(
        all(
            isinstance(row, dict)
            and set(row) == {
                "tree_size", "leaf_index", "path_length", "expected_root_sha256",
                "independently_computed_root_sha256", "reference_root_matches_trusted",
                "observed_root_sha256", "verdict",
            }
            and row.get("verdict") == "PASS"
            for row in cases
        ),
        "every positive inclusion case must have the exact schema and pass",
    )
    expected_pairs = [
        (tree_size, leaf_index)
        for tree_size in range(1, 17)
        for leaf_index in range(tree_size)
    ]
    observed_pairs = []
    for row in cases:
        require(type(row.get("tree_size")) is int, "tree_size must be an integer")
        require(type(row.get("leaf_index")) is int, "leaf_index must be an integer")
        observed_pairs.append((row["tree_size"], row["leaf_index"]))
    require(
        observed_pairs == expected_pairs,
        "matrix must cover each tree-size/leaf-index pair exactly once in canonical order",
    )
    def reference_path_length(tree_size, leaf_index):
        if tree_size == 1:
            return 0
        split = 1 << ((tree_size - 1).bit_length() - 1)
        if leaf_index < split:
            return reference_path_length(split, leaf_index) + 1
        return reference_path_length(tree_size - split, leaf_index - split) + 1
    require(
        all(
            type(row.get("path_length")) is int
            and row["path_length"] == reference_path_length(row["tree_size"], row["leaf_index"])
            and row["path_length"] <= 4
            for row in cases
        ),
        "every proof path length must match the canonical recursive tree shape",
    )
    require(
        all(
            row.get("expected_root_sha256") == trusted_roots.get(str(row.get("tree_size")))
            and row.get("independently_computed_root_sha256") == trusted_roots.get(str(row.get("tree_size")))
            and row.get("observed_root_sha256") == trusted_roots.get(str(row.get("tree_size")))
            and row.get("reference_root_matches_trusted") is True
            for row in cases
        ),
        "every proof row must match its manifest-pinned expected root digest",
    )
    require(report.get("tree_size_max_inclusive") == 2**62, "local synthetic profile tree-size cap")
    require(report.get("tree_size_limit_scope") == "LOCAL_SYNTHETIC_PROFILE_NOT_RFC_REQUIREMENT",
            "local profile cap must not be claimed as RFC requirement")
    require(report.get("negative_controls_all_rejected") is True, "negative-control verdict")
    require(report.get("issuer_signature_verification") == "NOT_EVALUATED", "issuer signature non-claim")
    expected_negative_reasons = {
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
    identities = semantic["identities"]
    require(isinstance(identities, dict) and set(identities) == {"A", "B"},
            "trusted receipt identities must be exactly A and B")
    expected_receipt_keys = {
        "receipt_sha256", "protected_sha256", "kid_hex", "issuer", "subject",
        "tree_size", "leaf_index", "root_hash", "proof_sha256",
    }
    for label, report_key in (("A", "receipt_a"), ("B", "receipt_b")):
        identity = identities[label]
        require(isinstance(identity, dict)
                and set(identity) == {"kid_hex", "issuer", "public_key_pem"},
                "trusted identity schema: " + label)
        receipt = report.get(report_key)
        require(isinstance(receipt, dict) and set(receipt) == expected_receipt_keys,
                "receipt summary schema: " + label)
        for digest_key in ("receipt_sha256", "protected_sha256", "root_hash", "proof_sha256"):
            require(
                isinstance(receipt.get(digest_key), str)
                and len(receipt[digest_key]) == 64
                and all(ch in "0123456789abcdef" for ch in receipt[digest_key]),
                "receipt " + label + " " + digest_key + " must be lowercase SHA-256",
            )
        require(receipt["receipt_sha256"] == semantic["receipt_" + label.lower() + "_sha256"],
                "receipt digest differs from trusted expectation: " + label)
        require(receipt["kid_hex"] == identity["kid_hex"], "receipt KID binding: " + label)
        require(receipt["issuer"] == identity["issuer"], "receipt issuer binding: " + label)
        require(receipt["subject"] == semantic["subject"], "receipt subject binding: " + label)
        require(receipt["tree_size"] == semantic["tree_size"], "receipt tree-size binding: " + label)
        require(receipt["leaf_index"] == semantic["leaf_index"], "receipt leaf-index binding: " + label)
        require(receipt["root_hash"] == semantic["root_hash"], "receipt root binding: " + label)
        require(type(receipt["tree_size"]) is int and type(receipt["leaf_index"]) is int,
                "receipt proof coordinates must be integers: " + label)
    negative_controls = report.get("negative_controls")
    require(
        isinstance(negative_controls, list) and len(negative_controls) == 11,
        "negative-control rows",
    )
    require(
        [row.get("name") for row in negative_controls if isinstance(row, dict)]
        == list(expected_negative_reasons),
        "negative-control names must be unique and in the exact trusted sequence",
    )
    require(
        all(
            set(row) == {"name", "verdict", "reason"}
            and row.get("verdict") == "REJECT"
            and row.get("reason") == expected_negative_reasons[row["name"]]
            for row in negative_controls
        ),
        "each negative control must have exact schema and reach its expected rejection reason",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", required=True)
    parser.add_argument("--trusted-manifest", required=True)
    parser.add_argument("--candidate-root", required=True)
    args = parser.parse_args()
    try:
        report = strict_json(args.report)
        trusted_manifest = strict_json(args.trusted_manifest)
        candidate_root = Path(args.candidate_root)
        expected_blobs = trusted_manifest.get("expected_blob_sha", {})
        candidate_files = {
            "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json":
                "candidate_corpus_git_blob_sha",
            "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json":
                "candidate_qualification_manifest_sha256",
        }
        for relative_path, report_field in candidate_files.items():
            try:
                raw = (candidate_root / relative_path).read_bytes()
            except OSError as exc:
                raise Reject("candidate input unavailable: " + relative_path) from exc
            if report_field == "candidate_corpus_git_blob_sha":
                blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\\0" + raw).hexdigest()
                if blob != report.get(report_field) or blob != expected_blobs.get(relative_path):
                    raise Reject("candidate corpus identity differs from pinned input")
            else:
                digest = hashlib.sha256(raw).hexdigest()
                if digest != report.get(report_field):
                    raise Reject("candidate qualification manifest digest differs from input")
                blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\\0" + raw).hexdigest()
                if blob != expected_blobs.get(relative_path):
                    raise Reject("candidate qualification manifest blob differs from trusted pin")
        validate_report(report, trusted_manifest)
    except (Reject, KeyError, TypeError, ValueError) as exc:
        raise SystemExit("self-test report reject: " + str(exc)) from exc
    print("non-authoritative trusted-oracle self-test contract: PASS")


if __name__ == "__main__":
    main()
