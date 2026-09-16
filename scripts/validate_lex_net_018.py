#!/usr/bin/env python3
"""Validate LEX-NET-018 foreign-evidence quarantine contract."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "docs/lex-net/lex_net_018_manifest.json"
DOC_PATH = ROOT / "docs/lex-net/LEX_NET_FOREIGN_EVIDENCE_QUARANTINE_V1.md"
EXPECTED_PARENT = "d5040517f7787541df6ebf9c33805e9a68df5bec"
EXPECTED_MODES = {
    ".github/workflows/lex-net-018.yml": "100644",
    "docs/lex-net/LEX_NET_FOREIGN_EVIDENCE_QUARANTINE_V1.md": "100644",
    "docs/lex-net/lex_net_018_manifest.json": "100644",
    "scripts/validate_lex_net_018.py": "100755",
}


def die(msg: str) -> None:
    raise SystemExit(f"LEX-NET-018 FAIL: {msg}")


def run_git(*args: str, text: bool = True) -> str | bytes:
    cp = subprocess.run(
        ["git", *args], cwd=ROOT, check=False, capture_output=True,
        text=text,
    )
    if cp.returncode != 0:
        stderr = cp.stderr.strip() if text else cp.stderr.decode("utf-8", "replace").strip()
        die(f"git {' '.join(args)} failed: {stderr}")
    return cp.stdout


def normalized_prose(s: str) -> str:
    s = s.replace("`", "")
    return " ".join(s.casefold().split())


def load_manifest() -> dict:
    try:
        return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    except Exception as exc:
        die(f"cannot load manifest: {exc}")


def evaluate(inp: dict) -> dict:
    base = {
        "projection": {},
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
        "foreign_origin": True,
    }

    if inp.get("attempt_unrelated_authoritative_index"):
        return {**base, "disposition": "PromotionRejected", "reason": "AuthoritativeIndexBypassDenied"}

    state = inp.get("quarantine_state")
    recognition = inp.get("recognition_disposition")

    if recognition is None:
        return {**base, "disposition": "Quarantined", "reason": "NoPositiveRecognition"}

    if state == "RecognitionExpiredOrRevoked":
        return {**base, "disposition": "PromotionRejected", "reason": "RecognitionExpiredOrRevoked"}

    if recognition in {"RecognitionRejected", "Rejected"}:
        return {**base, "disposition": "PromotionRejected", "reason": "RecognitionRejected"}
    if recognition in {"RecognitionIndeterminate", "Indeterminate", "NeedsAdditionalEvidence"}:
        return {**base, "disposition": "PromotionIndeterminate", "reason": "RecognitionIndeterminate"}
    if recognition != "RecognizedEvidence":
        return {**base, "disposition": "PromotionUnsupported", "reason": "RecognitionDispositionUnsupported"}

    source = inp.get("source_commitment")
    recognized_source = inp.get("recognition_source_commitment")
    if not source or recognized_source != source:
        return {**base, "disposition": "PromotionRejected", "reason": "SourceBindingMismatch"}

    if inp.get("purpose") != inp.get("requested_purpose"):
        return {**base, "disposition": "PromotionRejected", "reason": "PurposeMismatch"}

    if "translation_receipt_commitment" in inp or "expected_translation_receipt_commitment" in inp:
        if inp.get("translation_receipt_commitment") != inp.get("expected_translation_receipt_commitment"):
            return {**base, "disposition": "PromotionIndeterminate", "reason": "TranslationBindingMismatch"}

    claims = inp.get("recognized_claims")
    if not isinstance(claims, list) or not claims:
        return {**base, "disposition": "PromotionIndeterminate", "reason": "NoRecognizedClaimProjection"}
    if len(claims) != len(set(claims)) or any(not isinstance(x, str) or not x for x in claims):
        return {**base, "disposition": "PromotionUnsupported", "reason": "InvalidRecognizedClaimSet"}

    source_claims = inp.get("source_claims")
    if not isinstance(source_claims, dict):
        return {**base, "disposition": "PromotionIndeterminate", "reason": "SourceClaimsMissing"}
    missing = [c for c in claims if c not in source_claims]
    if missing:
        return {**base, "disposition": "PromotionIndeterminate", "reason": "RecognizedClaimMissingFromSource"}

    projection = {c: source_claims[c] for c in claims}
    return {
        **base,
        "projection": projection,
        "disposition": "PromotedRecognizedProjection",
        "reason": "BoundedProjectionPromoted",
    }


def validate_semantics() -> None:
    manifest = load_manifest()
    doc = DOC_PATH.read_text(encoding="utf-8")
    prose = normalized_prose(doc)

    if manifest.get("schema") != "lex-net-018-manifest-v1":
        die("wrong schema")
    if manifest.get("tranche") != "LEX-NET-018" or manifest.get("issue") != 1113:
        die("wrong tranche/issue binding")
    if manifest.get("qualified_parent") != EXPECTED_PARENT:
        die("wrong qualified parent in manifest")

    theorem = "received foreign evidence != local canonical fact != locally recognized evidence != local authority"
    if manifest.get("governing_theorem") != theorem:
        die("governing theorem mismatch")
    if theorem.casefold() not in prose:
        die("governing theorem absent from normative doc")

    expected_dispositions = [
        "Quarantined",
        "PromotedRecognizedProjection",
        "PromotionRejected",
        "PromotionIndeterminate",
        "PromotionUnsupported",
    ]
    if manifest.get("dispositions") != expected_dispositions:
        die("disposition set/order drift")

    required_phrases = [
        "foreign evidence stays foreign",
        "grants_local_authority = false",
        "grants_external_effect_authority = false",
        "recognition for purpose a cannot be replayed for purpose b",
        "auditability does not require permanent plaintext retention",
        "does not itself mint authority",
    ]
    for phrase in required_phrases:
        if normalized_prose(phrase) not in prose:
            die(f"required normative phrase absent: {phrase}")

    for nonclaim in manifest.get("required_nonclaims", []):
        if normalized_prose(nonclaim) not in prose:
            die(f"nonclaim absent from doc: {nonclaim}")

    fixtures = manifest.get("fixtures")
    if not isinstance(fixtures, list) or len(fixtures) != 12:
        die("expected exactly 12 golden fixtures")
    ids = [f.get("id") for f in fixtures]
    if len(ids) != len(set(ids)) or any(not x for x in ids):
        die("fixture ids must be unique and non-empty")

    positive_seen = False
    negative_seen = False
    for fixture in fixtures:
        actual = evaluate(fixture.get("input", {}))
        expected = fixture.get("expect", {})
        if actual["disposition"] != expected.get("disposition"):
            die(f"fixture {fixture['id']} disposition: expected {expected.get('disposition')} got {actual['disposition']}")
        if actual["projection"] != expected.get("projection", {}):
            die(f"fixture {fixture['id']} projection mismatch")
        for flag in ("grants_local_authority", "grants_external_effect_authority"):
            if actual[flag] is not False:
                die(f"fixture {fixture['id']} escalated {flag}")
            if flag in expected and expected[flag] is not False:
                die(f"fixture {fixture['id']} expected data illegally requests authority")
        if actual["disposition"] == "PromotedRecognizedProjection":
            positive_seen = True
        else:
            negative_seen = True

    if not positive_seen or not negative_seen:
        die("corpus must include positive and non-positive paths")

    authority_fixture = next(f for f in fixtures if f["id"] == "authority_field_not_promoted")
    result = evaluate(authority_fixture["input"])
    if "authority" in result["projection"]:
        die("foreign authority field leaked into recognized projection")

    print(json.dumps({
        "tranche": "LEX-NET-018",
        "semantic_result": "PASS",
        "fixture_count": len(fixtures),
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
    }, sort_keys=True))


def validate_scope() -> None:
    manifest = load_manifest()
    event_path = os.environ.get("GITHUB_EVENT_PATH")
    if not event_path:
        die("GITHUB_EVENT_PATH missing for scope validation")
    try:
        event = json.loads(Path(event_path).read_text(encoding="utf-8"))
        pr = event["pull_request"]
        event_head = pr["head"]["sha"]
        event_base = pr["base"]["sha"]
    except Exception as exc:
        die(f"malformed pull-request event: {exc}")

    head = str(run_git("rev-parse", "HEAD")).strip()
    parent = str(run_git("rev-parse", "HEAD^" )).strip()
    if head != event_head:
        die(f"checked-out head {head} != authored PR head {event_head}")
    if parent != EXPECTED_PARENT:
        die(f"parent {parent} != qualified parent {EXPECTED_PARENT}")
    if event_base != EXPECTED_PARENT:
        die(f"PR base {event_base} != qualified parent {EXPECTED_PARENT}")

    count = int(str(run_git("rev-list", "--count", f"{EXPECTED_PARENT}..{head}")).strip())
    if count != 1:
        die(f"expected exactly one authored commit, got {count}")

    raw = run_git("diff", "--name-only", "-z", EXPECTED_PARENT, head, text=False)
    assert isinstance(raw, bytes)
    paths = [p.decode("utf-8", "surrogateescape") for p in raw.split(b"\0") if p]
    expected_paths = manifest.get("exact_changed_paths")
    if sorted(paths) != sorted(expected_paths):
        die(f"changed path set mismatch: {paths!r}")

    for path, expected_mode in EXPECTED_MODES.items():
        out = str(run_git("ls-tree", head, "--", path)).strip()
        if not out:
            die(f"missing expected path in tree: {path}")
        mode = out.split(None, 1)[0]
        if mode != expected_mode:
            die(f"mode mismatch for {path}: expected {expected_mode} got {mode}")

    status = run_git("status", "--porcelain=v1", "-z", text=False)
    if status:
        die("working tree not clean during scope validation")

    print(json.dumps({
        "tranche": "LEX-NET-018",
        "scope_result": "PASS",
        "head": head,
        "parent": parent,
        "commit_count": count,
        "paths": sorted(paths),
    }, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", action="store_true")
    parser.add_argument("--semantic", action="store_true")
    args = parser.parse_args()
    if not args.scope and not args.semantic:
        args.semantic = True
    if args.scope:
        validate_scope()
    if args.semantic:
        validate_semantics()


if __name__ == "__main__":
    main()
