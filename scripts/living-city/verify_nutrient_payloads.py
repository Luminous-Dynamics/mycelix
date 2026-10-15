#!/usr/bin/env python3
"""Offline verification of Living-City nutrient receipt payload commitments.

This tool resolves only explicit repo-file: locators, hashes exact raw bytes,
rejects ambiguous JSON, and binds native Symtropy fields to the normalized
envelope. It performs no network access and never treats remote/opaque locators
as verified.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path, PurePosixPath
from typing import Any

REPO_FILE_PREFIX = "repo-file:"
SHA256_RE = re.compile(r"^[a-f0-9]{64}$")
BOUND_FIELDS = (
    "scenario_id", "target_id", "nutrient_id", "nutrient_definition_ref",
    "unit", "target_scope_ref", "period_ref", "value_scale",
    "total_edible_delivered_grams", "supply_numerator", "target_numerator",
    "denominator", "shortfall_numerator", "composition_source_refs",
    "delivery_source_refs", "target_source_ref", "status",
)


def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def reject_nonstandard_constant(token: str) -> None:
    raise ValueError(f"non-standard JSON numeric constant: {token}")


def resolve_repo_file(payload_ref: Any, repo_root: Path) -> tuple[Path | None, str | None]:
    """Return a resolved in-root regular file or an explicit failure status."""
    if not isinstance(payload_ref, str) or not payload_ref:
        return None, "invalid_reference"
    if not payload_ref.startswith(REPO_FILE_PREFIX):
        return None, "external_unverified"
    raw = payload_ref[len(REPO_FILE_PREFIX):]
    if not raw or "\\" in raw or "\x00" in raw or raw.startswith("/") or re.match(r"^[A-Za-z]:", raw):
        return None, "invalid_reference"

    # Validate the literal POSIX path before normalizing away '.' or '..'.
    raw_parts = raw.split("/")
    pure = PurePosixPath(raw)
    if pure.is_absolute() or any(part in ("", ".", "..") for part in raw_parts):
        return None, "invalid_reference"
    try:
        root = repo_root.resolve(strict=True)
        resolved = root.joinpath(*pure.parts).resolve(strict=True)
    except FileNotFoundError:
        return None, "missing_payload"
    except (OSError, RuntimeError):
        return None, "unreadable_payload"
    if not resolved.is_relative_to(root):
        return None, "invalid_reference"
    if not resolved.is_file():
        return None, "unreadable_payload"
    return resolved, None


def _payload_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def verify_receipt_payload(receipt: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    receipt_id, payload_ref, expected = receipt.get("receipt_id"), receipt.get("payload_ref"), receipt.get("payload_sha256")
    base = {"receipt_id": receipt_id, "payload_ref": payload_ref}
    if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
        return {**base, "status": "invalid_commitment", "reason": "payload_sha256 must be 64 lowercase hexadecimal characters"}

    path, path_error = resolve_repo_file(payload_ref, repo_root)
    if path_error:
        explanations = {
            "external_unverified": "only repo-file: locators can be verified offline; no network fetch was attempted",
            "invalid_reference": "locator is not a safe repository-relative file reference",
            "missing_payload": "referenced payload file does not exist",
            "unreadable_payload": "referenced payload is not a readable regular file inside the repository root",
        }
        return {**base, "status": path_error, "reason": explanations[path_error]}
    try:
        actual = _payload_sha256(path)
        raw_bytes = path.read_bytes()
    except OSError:
        return {**base, "status": "unreadable_payload", "reason": "payload bytes could not be read"}
    if actual != expected:
        return {**base, "status": "hash_mismatch", "expected_sha256": expected, "actual_sha256": actual,
                "reason": "SHA-256 of exact on-disk bytes does not match the envelope commitment"}
    try:
        native = json.loads(raw_bytes.decode("utf-8"), parse_constant=reject_nonstandard_constant,
                            object_pairs_hook=reject_duplicate_keys)
    except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
        return {**base, "status": "invalid_payload_json", "actual_sha256": actual, "reason": str(exc)}
    if not isinstance(native, dict):
        return {**base, "status": "payload_binding_mismatch", "actual_sha256": actual,
                "mismatched_fields": ["<root>"], "reason": "native payload must be a JSON object"}
    mismatched = [field for field in BOUND_FIELDS
                  if field not in native or field not in receipt or native[field] != receipt[field]]
    if mismatched:
        return {**base, "status": "payload_binding_mismatch", "actual_sha256": actual,
                "mismatched_fields": mismatched,
                "reason": "hashed native payload fields do not exactly match the normalized receipt envelope"}
    return {**base, "status": "verified", "actual_sha256": actual,
            "binding_fields_checked": list(BOUND_FIELDS),
            "reason": "exact payload bytes match SHA-256 and native fields match the envelope"}


def verify_review_payloads(review: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    receipts = review.get("nutrient_assessments", [])
    if not isinstance(receipts, list):
        return {"status": "blocked", "ok": False, "records": [],
                "errors": ["nutrient_assessments must be an array"]}
    real = [r for r in receipts if isinstance(r, dict) and r.get("receipt_origin") == "symtropy_output"]
    if not real:
        return {"status": "no_real_receipts", "ok": False, "records": [],
                "errors": ["no Symtropy-origin nutrient payload was available to verify; this is not a verification pass"]}
    ids = [r.get("receipt_id") for r in real]
    duplicate_ids = {rid for rid in ids if ids.count(rid) > 1}
    records = []
    for receipt in real:
        record = verify_receipt_payload(receipt, repo_root)
        if receipt.get("receipt_id") in duplicate_ids:
            record["status"] = "duplicate_receipt_id"
            record["reason"] = "real payload receipts must have unique receipt_id values"
        records.append(record)
    ok = all(record.get("status") == "verified" for record in records)
    return {
        "status": "verified" if ok else "blocked",
        "ok": ok,
        "records": records,
        "summary": {"real_receipts": len(records),
                    "verified": sum(record.get("status") == "verified" for record in records),
                    "blocked": sum(record.get("status") != "verified" for record in records)},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("review_json", type=Path, help="review JSON containing nutrient_assessments")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd(),
                        help="root against which repo-file: locators are resolved (default: current directory)")
    args = parser.parse_args()
    try:
        review = json.loads(args.review_json.read_text(encoding="utf-8"),
                            parse_constant=reject_nonstandard_constant,
                            object_pairs_hook=reject_duplicate_keys)
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "blocked", "ok": False,
                          "errors": [f"review JSON could not be loaded: {exc}"]}, indent=2), file=sys.stderr)
        return 2
    if not isinstance(review, dict):
        print(json.dumps({"status": "blocked", "ok": False,
                          "errors": ["review JSON root must be an object"]}, indent=2), file=sys.stderr)
        return 2
    report = verify_review_payloads(review, args.repo_root)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
