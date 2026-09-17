#!/usr/bin/env python3
"""Validate LEX-NET-032 canonical interpretation and parser-differential contract."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

TRANCHE = "LEX-NET-032"
PROFILE_ID = "lex-net-json-interpretation-v1"
PARENT = "94d9869fd82c8708af3e08b95306586ef0d7b4fb"
EXPECTED_PATHS = [
    ".github/workflows/lex-net-032.yml",
    "docs/lex-net/LEX_NET_CANONICAL_INTERPRETATION_V1.md",
    "docs/lex-net/lex_net_032_manifest.json",
    "scripts/validate_lex_net_032.py",
]
MANIFEST_PATH = Path("docs/lex-net/lex_net_032_manifest.json")
DOC_PATH = Path("docs/lex-net/LEX_NET_CANONICAL_INTERPRETATION_V1.md")


class DuplicateKeyError(ValueError):
    pass


class NumericDomainError(ValueError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def reject_float(token: str) -> Any:
    raise NumericDomainError(f"floating-point JSON number forbidden: {token}")


def reject_constant(token: str) -> Any:
    raise NumericDomainError(f"non-finite JSON number forbidden: {token}")


def object_pairs_no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise DuplicateKeyError(key)
        out[key] = value
    return out


def check_integer_domain(value: Any) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, int):
        if value < -(2**63) or value > 2**63 - 1:
            raise NumericDomainError(f"signed-64 overflow: {value}")
        return
    if isinstance(value, list):
        for item in value:
            check_integer_domain(item)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("non-string JSON object key")
            check_integer_domain(item)
        return
    raise NumericDomainError(f"unsupported numeric/value domain: {type(value).__name__}")


def fixture_bytes(fixture: dict[str, Any]) -> bytes:
    has_json = "raw_json" in fixture
    has_b64 = "raw_base64" in fixture
    if has_json == has_b64:
        raise ValueError(f"{fixture.get('id')}: exactly one raw source encoding required")
    if has_json:
        return fixture["raw_json"].encode("utf-8")
    return base64.b64decode(fixture["raw_base64"], validate=True)


def result(disposition: str, reason: str, **extra: Any) -> dict[str, Any]:
    return {
        "disposition": disposition,
        "reason": reason,
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
        **extra,
    }


def interpret(fixture: dict[str, Any]) -> dict[str, Any]:
    if fixture.get("profile_id") != PROFILE_ID:
        return result("ProfileUnsupported", "exact interpretation profile is not supported")

    try:
        raw = fixture_bytes(fixture)
    except Exception as exc:
        return result("MalformedEncoding", f"invalid frozen source encoding: {exc}")

    source_sha = sha256(raw)
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
        return result("MalformedEncoding", "source bytes are not valid UTF-8", source_sha256=source_sha)

    try:
        parsed = json.loads(
            text,
            object_pairs_hook=object_pairs_no_duplicates,
            parse_float=reject_float,
            parse_constant=reject_constant,
        )
    except DuplicateKeyError as exc:
        return result("DuplicateKeyRejected", f"duplicate object key: {exc}", source_sha256=source_sha)
    except NumericDomainError as exc:
        return result("NumericDomainViolation", str(exc), source_sha256=source_sha)
    except (json.JSONDecodeError, ValueError) as exc:
        return result("MalformedEncoding", f"JSON parse failed: {exc}", source_sha256=source_sha)

    if not isinstance(parsed, dict):
        return result("MalformedEncoding", "top-level JSON value must be an object", source_sha256=source_sha)

    try:
        check_integer_domain(parsed)
    except NumericDomainError as exc:
        return result("NumericDomainViolation", str(exc), source_sha256=source_sha)

    critical = parsed.get("critical", [])
    if not isinstance(critical, list) or any(not isinstance(x, str) for x in critical) or len(set(critical)) != len(critical):
        return result("CriticalFieldUnsupported", "critical must be an array of unique strings", source_sha256=source_sha)
    if critical:
        return result("CriticalFieldUnsupported", "v1 supports no critical extensions", source_sha256=source_sha)

    try:
        canonical_text = json.dumps(
            parsed,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        canonical = canonical_text.encode("utf-8", errors="strict")
    except UnicodeEncodeError:
        return result("UnicodeViolation", "decoded projection contains invalid Unicode scalar data", source_sha256=source_sha)

    projection_sha = sha256(canonical)
    claimed = fixture.get("claimed_projection_sha256")
    if claimed is not None and claimed != projection_sha:
        return result(
            "CanonicalizationMismatch",
            "caller-supplied projection commitment does not match independent interpretation",
            source_sha256=source_sha,
            canonical_sha256=projection_sha,
        )

    return result(
        "InterpretationEstablished",
        "exact source bytes interpreted under frozen v1 profile",
        source_sha256=source_sha,
        canonical_sha256=projection_sha,
        projection_sha256=projection_sha,
        canonical_text=canonical_text,
        interpretation_profile=PROFILE_ID,
    )


def load_manifest() -> dict[str, Any]:
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def git(*args: str) -> str:
    proc = subprocess.run(["git", *args], check=True, text=True, capture_output=True)
    return proc.stdout.strip()


def validate_scope() -> None:
    manifest = load_manifest()
    if manifest.get("qualified_parent") != PARENT:
        raise SystemExit("manifest qualified parent drift")
    if manifest.get("expected_paths") != EXPECTED_PATHS:
        raise SystemExit("manifest path census drift")

    head = git("rev-parse", "HEAD")
    parent = git("rev-parse", "HEAD^")
    if parent != PARENT:
        raise SystemExit(f"wrong parent: {parent}")
    commit_count = int(git("rev-list", "--count", f"{PARENT}..HEAD"))
    if commit_count != 1:
        raise SystemExit(f"expected one authored commit, got {commit_count}")
    changed = git("diff", "--name-only", f"{PARENT}..HEAD").splitlines()
    if changed != sorted(EXPECTED_PATHS):
        raise SystemExit(f"exact path set mismatch: {changed}")

    print(json.dumps({
        "tranche": TRANCHE,
        "head": head,
        "parent": parent,
        "commit_count": commit_count,
        "paths": changed,
        "scope_result": "PASS",
    }, sort_keys=True))


def validate_contract_shape(manifest: dict[str, Any]) -> None:
    if manifest.get("tranche") != TRANCHE or manifest.get("issue") != 1254:
        raise SystemExit("manifest tranche/issue drift")
    if manifest.get("profile_id") != "lex-net-032-canonical-interpretation-v1":
        raise SystemExit("manifest profile drift")
    profile = manifest.get("interpretation_profile", {})
    expected = {
        "id": PROFILE_ID,
        "version": 1,
        "encoding": "UTF-8-strict",
        "top_level": "object",
        "duplicate_key_policy": "reject",
        "float_policy": "reject",
        "integer_domain": "signed-64",
        "unicode_normalization": "none",
        "object_key_order": "representation-only-canonical-sort",
        "array_order": "preserve",
        "critical_extension_policy": "reject-nonempty",
        "unknown_noncritical_policy": "preserve",
        "canonicalizer": "synthetic-python-json-sort-keys-v1",
        "canonicalizer_external_standard_claim": False,
        "commitment": "sha256",
    }
    if profile != expected:
        raise SystemExit("interpretation profile drift")
    if len(manifest.get("fixtures", [])) != 16:
        raise SystemExit("fixture census drift")


def validate_semantic() -> None:
    manifest = load_manifest()
    validate_contract_shape(manifest)
    fixtures = manifest["fixtures"]
    seen: set[str] = set()
    outputs: dict[str, dict[str, Any]] = {}

    for fixture in fixtures:
        fixture_id = fixture.get("id")
        if not isinstance(fixture_id, str) or fixture_id in seen:
            raise SystemExit("fixture ids must be unique strings")
        seen.add(fixture_id)
        out = interpret(fixture)
        outputs[fixture_id] = out
        if out["disposition"] != fixture.get("expected"):
            raise SystemExit(
                f"{fixture_id}: expected {fixture.get('expected')} got {out['disposition']} ({out['reason']})"
            )
        if out["grants_local_authority"] or out["grants_external_effect_authority"]:
            raise SystemExit(f"{fixture_id}: authority escalation")
        expected_canonical = fixture.get("expected_canonical")
        if expected_canonical is not None and out.get("canonical_text") != expected_canonical:
            raise SystemExit(f"{fixture_id}: canonical text mismatch")

    duplicate = next(f for f in fixtures if f["id"] == "duplicate_authority_rejected")
    naive = json.loads(duplicate["raw_json"])
    if naive.get(duplicate["naive_last_wins_field"]) != duplicate["naive_last_wins_value"]:
        raise SystemExit("naive last-wins demonstration drift")
    if outputs[duplicate["id"]]["disposition"] != "DuplicateKeyRejected":
        raise SystemExit("strict duplicate rejection drift")

    uc = outputs["unicode_composed_preserved"]["projection_sha256"]
    ud = outputs["unicode_decomposed_preserved"]["projection_sha256"]
    if uc == ud:
        raise SystemExit("composed/decomposed Unicode unexpectedly collapsed")

    ab = outputs["array_order_ab"]["projection_sha256"]
    ba = outputs["array_order_ba"]["projection_sha256"]
    if ab == ba:
        raise SystemExit("array order unexpectedly collapsed")

    doc = DOC_PATH.read_text(encoding="utf-8").casefold()
    for phrase in [
        "signature-valid bytes != parser agreement != canonical-data agreement != semantic agreement != local recognition",
        "grants_local_authority = false",
        "grants_external_effect_authority = false",
        "does not claim rfc 8785",
        "canonicalization does not establish factual truth",
        "canonicalization does not establish local recognition",
    ]:
        if phrase not in doc:
            raise SystemExit(f"normative phrase missing: {phrase}")

    dispositions = sorted({out["disposition"] for out in outputs.values()})
    print(json.dumps({
        "tranche": TRANCHE,
        "fixture_count": len(fixtures),
        "observed_dispositions": dispositions,
        "semantic_result": "PASS",
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
    }, sort_keys=True))


def self_test() -> None:
    cases = [
        ({"profile_id": PROFILE_ID, "raw_json": "{\"x\":1,\"x\":2}"}, "DuplicateKeyRejected"),
        ({"profile_id": PROFILE_ID, "raw_json": "{\"x\":1.5}"}, "NumericDomainViolation"),
        ({"profile_id": PROFILE_ID, "raw_json": "{\"x\":9223372036854775808}"}, "NumericDomainViolation"),
        ({"profile_id": PROFILE_ID, "raw_json": "{\"x\":\"\\ud800\"}"}, "UnicodeViolation"),
        ({"profile_id": "other", "raw_json": "{\"x\":1}"}, "ProfileUnsupported"),
        ({"profile_id": PROFILE_ID, "raw_json": "{\"x\":1}", "claimed_projection_sha256": "0"*64}, "CanonicalizationMismatch"),
    ]
    for fixture, expected in cases:
        actual = interpret(fixture)["disposition"]
        if actual != expected:
            raise SystemExit(f"self-test expected {expected}, got {actual}")
    print("LEX-NET-032 self-test PASS")


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scope", action="store_true")
    group.add_argument("--semantic", action="store_true")
    group.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.scope:
        validate_scope()
    elif args.semantic:
        validate_semantic()
    else:
        self_test()


if __name__ == "__main__":
    main()
