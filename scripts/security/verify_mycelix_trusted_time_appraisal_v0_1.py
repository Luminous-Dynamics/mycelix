#!/usr/bin/env python3
"""Appraise trusted-time evidence with explicit freshness and nonce binding."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "docs/security/mycelix-trusted-time-registry-v0.1.json"
VERIFIER_ID = "mycelix.trusted-time.appraisal.v0.1"
APPROVED_REGISTRY_SHA256 = "451ca270db7b95ac12fda9ef96ba536cefe243c43068e1320cf93e21d2a7acf7"


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


def result(state: str, reason: str, reference_sha: str, registry_sha: str) -> dict[str, Any]:
    out = {
        "profile_id": "mycelix.trusted-time.appraisal",
        "profile_version": "0.1.0",
        "verifier_id": VERIFIER_ID,
        "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
        "registry_sha256": registry_sha,
        "reference_sha256": reference_sha,
        "state": state,
        "reason": reason,
    }
    out["content_sha256"] = self_hash(out)
    return out


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("trusted-time statement must be an object")
    return value


def appraise(path: Path, nonce_path: Path, now_unix: int | None) -> dict[str, Any]:
    reference_sha = sha256_file(path)
    registry_sha = sha256_file(REGISTRY)

    if registry_sha != APPROVED_REGISTRY_SHA256:
        return result("DENY", "registry-integrity-mismatch", reference_sha, registry_sha)

    try:
        reference = load(path)
        registry = load(REGISTRY)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return result("DENY", f"invalid-time-input:{exc}", reference_sha, registry_sha)

    nonce_sha = sha256_file(nonce_path)
    version = registry.get("profile_version")
    if version != "0.1.0":
        return result("DENY", "registry-version-mismatch", reference_sha, registry_sha)

    source_id = reference.get("source_id")
    if not isinstance(source_id, str) or not source_id:
        return result("DENY", "time-source-id-missing", reference_sha, registry_sha)

    approved_sources = {
        entry.get("source_id"): entry
        for entry in registry.get("approved", [])
        if isinstance(entry, dict)
    }
    source = approved_sources.get(source_id)
    if source is None:
        return result("INDETERMINATE", "time-source-not-approved", reference_sha, registry_sha)

    if reference.get("policy_trusted") is not True:
        return result("DENY", "time-policy-not-trusted", reference_sha, registry_sha)
    if reference.get("local_clock_only") is True:
        return result("DENY", "local-clock-is-not-authoritative", reference_sha, registry_sha)

    if reference.get("nonce_sha256") != nonce_sha:
        return result("DENY", "time-nonce-binding-mismatch", reference_sha, registry_sha)

    asserted = reference.get("asserted_unix")
    valid_until = reference.get("valid_until_unix")
    if not isinstance(asserted, int) or asserted < 0:
        return result("DENY", "asserted-time-invalid", reference_sha, registry_sha)
    if not isinstance(valid_until, int) or valid_until < asserted:
        return result("DENY", "time-validity-window-invalid", reference_sha, registry_sha)

    registry_max_age = source.get("max_age_seconds")
    if not isinstance(registry_max_age, int) or registry_max_age < 0:
        return result("DENY", "registry-max-age-invalid", reference_sha, registry_sha)
    if valid_until - asserted > registry_max_age:
        return result("DENY", "time-validity-window-exceeds-registry", reference_sha, registry_sha)

    if now_unix is None:
        return result("INDETERMINATE", "freshness-evaluation-time-unavailable", reference_sha, registry_sha)

    if now_unix < asserted:
        return result("INDETERMINATE", "evaluation-time-before-attested-time", reference_sha, registry_sha)
    if now_unix > valid_until:
        return result("DENY", "trusted-time-expired", reference_sha, registry_sha)

    return {
        **result("PASS", "trusted-time-source-and-freshness-approved", reference_sha, registry_sha),
        "source_id": source_id,
        "asserted_unix": asserted,
        "valid_until_unix": valid_until,
        "evaluation_unix": now_unix,
        "nonce_sha256": nonce_sha,
    }


def self_test() -> int:
    from tempfile import TemporaryDirectory

    with TemporaryDirectory(prefix="mycelix-trusted-time-") as td:
        root = Path(td)
        reference = root / "trusted-time.json"
        nonce = root / "nonce.bin"
        nonce.write_bytes(b"external-verifier-nonce-v1")
        reference.write_text(
            json.dumps(
                {
                    "source_id": "mycelix.reference-model.trusted-time",
                    "asserted_unix": 1800000000,
                    "valid_until_unix": 1800000060,
                    "nonce_sha256": sha256_file(nonce),
                    "policy_trusted": True,
                    "local_clock_only": False,
                },
                indent=2,
                sort_keys=True,
            ) + "\n",
            encoding="utf-8",
        )
        approved = appraise(reference, nonce, 1800000030)
        if approved["state"] != "PASS":
            print("approved trusted time: FAIL")
            return 1

        expired = appraise(reference, nonce, 1800000061)
        if expired["state"] != "DENY":
            print("expired trusted time: FAIL")
            return 1

        unknown = root / "unknown.json"
        unknown.write_text(
            json.dumps(
                {
                    "source_id": "unapproved-time-source",
                    "asserted_unix": 1800000000,
                    "valid_until_unix": 1800000060,
                    "nonce_sha256": sha256_file(nonce),
                    "policy_trusted": True,
                    "local_clock_only": False,
                }
            ),
            encoding="utf-8",
        )
        unknown_result = appraise(unknown, nonce, 1800000030)
        if unknown_result["state"] != "INDETERMINATE":
            print("unknown source: FAIL")
            return 1

        bad_nonce = root / "bad-nonce.json"
        bad_nonce.write_text(reference.read_text(encoding="utf-8").replace(
            sha256_file(nonce), "00" * 32
        ), encoding="utf-8")
        if appraise(bad_nonce, nonce, 1800000030)["state"] != "DENY":
            print("nonce mismatch: FAIL")
            return 1

    print("trusted-time appraisal self-test: PASS")
    print("approved fresh source: PASS")
    print("expired statement: DENY")
    print("unknown source: INDETERMINATE")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--appraise", metavar="TRUSTED_TIME_JSON")
    parser.add_argument("--nonce-file", required=False)
    parser.add_argument("--now-unix", type=int, required=False)
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if not args.nonce_file:
        parser.error("--nonce-file is required with --appraise")

    result = appraise(
        Path(args.appraise).resolve(),
        Path(args.nonce_file).resolve(),
        args.now_unix,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "INDETERMINATE": 2, "DENY": 1}[result["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
