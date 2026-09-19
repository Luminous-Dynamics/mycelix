#!/usr/bin/env python3
"""Verify the frozen ASSURE-V1 L0 codec corpus and requirement coverage."""

from __future__ import annotations

import hashlib
from pathlib import Path
import re
import sys
import tomllib

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
MANIFEST = HERE / "manifest.toml"
COVERAGE = HERE / "coverage.toml"
SPEC_LOCK = REPO / "assurance/spec/v1/SPEC.lock"
CODEC_SPEC = REPO / "assurance/spec/v1/codec.md"
FAILURE_REGISTRY = REPO / "assurance/spec/v1/failure-codes.toml"
RUST_CODEC_TEST = REPO / "crates/mycelix-assurance-core/tests/codec.rs"
ID_RE = re.compile(r"^V[0-9]{3}_[A-Z0-9_]+$")
REQ_RE = re.compile(r"^CODEC-[0-9]{3}$")
FAIL_RE = re.compile(r"^D[0-9]{3}_[A-Z0-9_]+$")
HASH_RE = re.compile(r"^[0-9a-f]{64}$")
SPEC_REQ_RE = re.compile(r"^(CODEC-[0-9]{3})\.", re.MULTILINE)
RUST_VECTOR_CONST_RE = re.compile(r"^const (V[0-9]{3}_[A-Z0-9_]+): &\[u8\] =", re.MULTILINE)
RUST_TEST_FN_RE = re.compile(r"^fn ([a-z][a-z0-9_]*)\(", re.MULTILINE)


def fail(message: str) -> None:
    print(f"ASSURE-V1 L0 VECTOR CHECK FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_spec_requirements() -> set[str]:
    text = CODEC_SPEC.read_text(encoding="utf-8")
    requirements = set(SPEC_REQ_RE.findall(text))
    if not requirements:
        fail("codec.md contains no CODEC-* normative requirements")
    return requirements


def load_spec_root() -> str:
    with SPEC_LOCK.open("rb") as handle:
        lock = tomllib.load(handle)
    root = lock.get("spec_root_sha256")
    if not isinstance(root, str) or not HASH_RE.fullmatch(root):
        fail("SPEC.lock has invalid spec_root_sha256")
    return root


def load_decode_failures() -> set[str]:
    with FAILURE_REGISTRY.open("rb") as handle:
        registry = tomllib.load(handle)
    rows = registry.get("failure")
    if not isinstance(rows, list):
        fail("failure-code registry has no [[failure]] entries")

    decode_codes: set[str] = set()
    for row in rows:
        code = row.get("code")
        stage = row.get("stage")
        if stage == "decode":
            if not isinstance(code, str) or not FAIL_RE.fullmatch(code):
                fail(f"invalid decode-stage failure code in registry: {code!r}")
            if code in decode_codes:
                fail(f"duplicate decode-stage failure code in registry: {code}")
            decode_codes.add(code)
    if not decode_codes:
        fail("failure-code registry contains no decode-stage codes")
    return decode_codes


def rust_test_surface() -> tuple[set[str], set[str]]:
    text = RUST_CODEC_TEST.read_text(encoding="utf-8")
    constants = RUST_VECTOR_CONST_RE.findall(text)
    if len(constants) != len(set(constants)):
        fail("Rust codec tests contain duplicate vector constant names")
    tests = set(RUST_TEST_FN_RE.findall(text))
    return set(constants), tests


def check_requirement_coverage(
    requirements: set[str], vector_ids: set[str], rust_tests: set[str]
) -> tuple[int, int]:
    with COVERAGE.open("rb") as handle:
        coverage = tomllib.load(handle)

    if coverage.get("version") != 1:
        fail("expected coverage version = 1")
    if coverage.get("spec") != "MYCELIX-ASSURE/V1/L0":
        fail("unexpected coverage spec id")

    rows = coverage.get("coverage")
    if not isinstance(rows, list) or not rows:
        fail("coverage.toml contains no [[coverage]] entries")

    row_ids = [row.get("id") for row in rows]
    if row_ids != sorted(row_ids):
        fail("coverage entries must be ordered lexicographically by requirement id")
    if len(row_ids) != len(set(row_ids)):
        fail("coverage contains duplicate requirement ids")
    if set(row_ids) != requirements:
        fail(
            "coverage requirement set differs from codec.md: "
            f"extra={sorted(set(row_ids)-requirements)}, "
            f"missing={sorted(requirements-set(row_ids))}"
        )

    automated_count = 0
    manual_count = 0
    for row in rows:
        requirement = row["id"]
        automated = row.get("automated")
        manual = row.get("manual")
        if not isinstance(automated, list) or not isinstance(manual, list):
            fail(f"{requirement}: automated/manual must be arrays")
        if not automated and not manual:
            fail(f"{requirement}: has no declared verification mechanism")

        for item in automated:
            if not isinstance(item, str):
                fail(f"{requirement}: non-string automated evidence")
            if item.startswith("vector:"):
                vector_id = item.removeprefix("vector:")
                if vector_id not in vector_ids:
                    fail(f"{requirement}: unknown vector evidence {vector_id}")
            elif item.startswith("rust-test:"):
                test_name = item.removeprefix("rust-test:")
                if test_name not in rust_tests:
                    fail(f"{requirement}: unknown Rust test evidence {test_name}")
            else:
                fail(f"{requirement}: unknown automated evidence kind {item!r}")
            automated_count += 1

        for item in manual:
            if not isinstance(item, str) or not item.strip():
                fail(f"{requirement}: manual evidence descriptions must be non-empty strings")
            manual_count += 1

    return automated_count, manual_count


def main() -> None:
    requirements = load_spec_requirements()
    spec_root = load_spec_root()
    decode_failures = load_decode_failures()

    with MANIFEST.open("rb") as handle:
        manifest = tomllib.load(handle)

    if manifest.get("version") != 1:
        fail("expected manifest version = 1")
    if manifest.get("root_format") != 3:
        fail("expected root_format = 3")
    if manifest.get("corpus") != "MYCELIX-ASSURE/V1/L0":
        fail("unexpected corpus name")

    domain = manifest.get("corpus_domain")
    if domain != "MYCELIX-ASSURE/V1/VECTORS/L0":
        fail("unexpected corpus domain")

    expected_spec_root = manifest.get("spec_root_sha256")
    if spec_root != expected_spec_root:
        fail(f"spec root mismatch: SPEC.lock={spec_root}, manifest={expected_spec_root}")

    coverage_digest = hashlib.sha256(COVERAGE.read_bytes()).hexdigest()
    expected_coverage_digest = manifest.get("coverage_sha256")
    if coverage_digest != expected_coverage_digest:
        fail(
            f"coverage digest mismatch: got {coverage_digest}, "
            f"expected {expected_coverage_digest}"
        )

    vectors = manifest.get("vector")
    if not isinstance(vectors, list) or not vectors:
        fail("manifest contains no [[vector]] entries")

    manifest_ids = [row.get("id") for row in vectors]
    if manifest_ids != sorted(manifest_ids):
        fail("vector entries must be ordered lexicographically by id")

    ids: set[str] = set()
    files: set[str] = set()
    root = hashlib.sha256()
    root.update(domain.encode("utf-8"))
    root.update(b"\x00ROOT-FORMAT-3\x00SPEC\x00")
    root.update(bytes.fromhex(spec_root))

    for row in vectors:
        vector_id = row.get("id")
        filename = row.get("file")
        expected_sha = row.get("sha256")
        requirement = row.get("requirement")
        expectation = row.get("expect")

        if not isinstance(vector_id, str) or not ID_RE.fullmatch(vector_id):
            fail(f"invalid vector id: {vector_id!r}")
        if vector_id in ids:
            fail(f"duplicate vector id: {vector_id}")
        ids.add(vector_id)

        if not isinstance(filename, str) or "/" in filename or not filename.endswith(".cbor"):
            fail(f"{vector_id}: invalid file name")
        if filename in files:
            fail(f"duplicate vector file: {filename}")
        files.add(filename)

        if not isinstance(expected_sha, str) or not HASH_RE.fullmatch(expected_sha):
            fail(f"{vector_id}: invalid sha256")
        if not isinstance(requirement, str) or not REQ_RE.fullmatch(requirement):
            fail(f"{vector_id}: invalid requirement id")
        if requirement not in requirements:
            fail(f"{vector_id}: requirement {requirement} is not defined in codec.md")
        if expectation not in {"accept", "reject"}:
            fail(f"{vector_id}: expectation must be accept or reject")

        failure = row.get("failure")
        if expectation == "accept":
            if failure is not None:
                fail(f"{vector_id}: accepted vector must not declare failure")
            failure_for_root = ""
        else:
            if not isinstance(failure, str) or not FAIL_RE.fullmatch(failure):
                fail(f"{vector_id}: rejected vector requires a decode failure code")
            if failure not in decode_failures:
                fail(f"{vector_id}: failure {failure} is not a registered decode-stage code")
            failure_for_root = failure

        path = HERE / filename
        if not path.is_file():
            fail(f"{vector_id}: missing fixture {filename}")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != expected_sha:
            fail(f"{vector_id}: digest mismatch for {filename}")

        root.update(vector_id.encode("utf-8"))
        root.update(b"\x00")
        root.update(filename.encode("utf-8"))
        root.update(b"\x00")
        root.update(bytes.fromhex(expected_sha))
        root.update(requirement.encode("utf-8"))
        root.update(b"\x00")
        root.update(expectation.encode("utf-8"))
        root.update(b"\x00")
        root.update(failure_for_root.encode("utf-8"))
        root.update(b"\x00")

    on_disk = {p.name for p in HERE.glob("*.cbor")}
    if on_disk != files:
        fail(
            f"fixture set differs from manifest: extra={sorted(on_disk-files)}, "
            f"missing={sorted(files-on_disk)}"
        )

    rust_constants, rust_tests = rust_test_surface()
    if rust_constants != ids:
        fail(
            "Rust fixture constants differ from manifest ids: "
            f"extra={sorted(rust_constants-ids)}, missing={sorted(ids-rust_constants)}"
        )

    automated_count, manual_count = check_requirement_coverage(requirements, ids, rust_tests)

    root.update(b"COVERAGE\x00")
    root.update(bytes.fromhex(coverage_digest))

    expected_root = manifest.get("corpus_root_sha256")
    if not isinstance(expected_root, str) or not HASH_RE.fullmatch(expected_root):
        fail("invalid corpus_root_sha256")
    actual_root = root.hexdigest()
    if actual_root != expected_root:
        fail(f"corpus root mismatch: got {actual_root}, expected {expected_root}")

    print(
        f"ASSURE-V1 L0 vectors OK: sha256:{expected_root}; "
        f"spec_root={spec_root}; vectors={len(vectors)}; "
        f"requirements={len(requirements)}; decode_failures={len(decode_failures)}; "
        f"rust_constants={len(ids)}; automated_evidence={automated_count}; "
        f"manual_evidence={manual_count}; coverage_sha256={coverage_digest}"
    )


if __name__ == "__main__":
    main()
