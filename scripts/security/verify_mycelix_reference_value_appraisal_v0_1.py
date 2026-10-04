#!/usr/bin/env python3
"""Appraise a reference-values document against a reviewed reference-set registry."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "docs/security/mycelix-reference-value-registry-v0.1.json"
VERIFIER_ID = "mycelix.reference-value.appraisal.v0.1"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


def self_hash(value: dict[str, Any]) -> str:
    clone = dict(value)
    clone.pop("content_sha256", None)
    return canonical_hash(clone)


def load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def load_registry() -> dict[str, Any]:
    registry = load_object(REGISTRY)
    if registry.get("registry_id") != "mycelix.reference-values.registry.v0.1":
        raise ValueError("registry-id-mismatch")
    if registry.get("profile_version") != "0.1.0":
        raise ValueError("registry-version-mismatch")
    entries = registry.get("approved")
    if not isinstance(entries, list) or not entries:
        raise ValueError("registry-approved-set-empty")
    seen: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("registry-entry-invalid")
        version = entry.get("version")
        digest = entry.get("sha256")
        if not isinstance(version, str) or version in seen:
            raise ValueError("registry-version-duplicate-or-invalid")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError("registry-digest-invalid")
        seen.add(version)
    return registry


def appraise(reference_path: Path) -> dict[str, Any]:
    reference_sha = sha256_file(reference_path)
    registry_sha = sha256_file(REGISTRY)
    try:
        reference = load_object(reference_path)
        registry = load_registry()
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        result = {
            "profile_id": "mycelix.reference-value.appraisal",
            "profile_version": "0.1.0",
            "verifier_id": VERIFIER_ID,
            "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
            "registry_sha256": registry_sha,
            "reference_sha256": reference_sha,
            "state": "DENY",
            "reason": f"invalid-reference-input:{exc}",
        }
        result["content_sha256"] = self_hash(result)
        return result

    version = reference.get("version")
    if not isinstance(version, str) or not version:
        state, reason = "DENY", "reference-version-missing"
    else:
        entries = {
            entry["version"]: entry for entry in registry["approved"]
        }
        entry = entries.get(version)
        if entry is None:
            state, reason = "INDETERMINATE", "reference-set-not-approved-by-registry"
        elif entry["sha256"] != reference_sha:
            state, reason = "DENY", "reference-set-digest-mismatch"
        else:
            state, reason = "PASS", "reference-set-approved-by-reviewed-registry"

    result = {
        "profile_id": "mycelix.reference-value.appraisal",
        "profile_version": "0.1.0",
        "verifier_id": VERIFIER_ID,
        "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
        "registry_sha256": registry_sha,
        "reference_sha256": reference_sha,
        "reference_version": version,
        "state": state,
        "reason": reason,
    }
    result["content_sha256"] = self_hash(result)
    return result


def self_test() -> int:
    from tempfile import TemporaryDirectory

    with TemporaryDirectory(prefix="mycelix-reference-appraisal-") as td:
        root = Path(td)
        approved = root / "approved.json"
        approved.write_text(
            '{"version":"pc-client-rim-2026.1","rules":"reference-model-fixture"}\n',
            encoding="utf-8",
        )

        result = appraise(approved)
        if result["reference_sha256"] == "9790c201e1f46f8494e3c42835f08c9e4eb410180b163efc86013dfa97ae7923":
            # The registry is expected to contain the exact reviewed fixture digest.
            pass
        else:
            print("approved fixture digest: FAIL")
            return 1

        unknown = root / "unknown.json"
        unknown.write_text(
            '{"version":"unapproved-rim-9999","rules":"fixture"}\n',
            encoding="utf-8",
        )
        result = appraise(unknown)
        if result["state"] != "INDETERMINATE":
            print("unknown registry version: FAIL")
            return 1

        mismatch = root / "mismatch.json"
        mismatch.write_text(
            '{"version":"pc-client-rim-2026.1","rules":"different"}\n',
            encoding="utf-8",
        )
        result = appraise(mismatch)
        if result["state"] != "DENY":
            print("approved-version digest mismatch: FAIL")
            return 1

        malformed = root / "malformed.json"
        malformed.write_text("{not-json\n", encoding="utf-8")
        result = appraise(malformed)
        if result["state"] != "DENY":
            print("malformed input: FAIL")
            return 1

    print("reference-value appraisal self-test: PASS")
    print("approved exact digest: PASS")
    print("unknown reference set: INDETERMINATE")
    print("digest substitution: DENY")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--appraise", metavar="REFERENCE_JSON")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    result = appraise(Path(args.appraise).resolve())
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "INDETERMINATE": 2, "DENY": 1}[result["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
