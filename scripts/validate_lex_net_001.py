#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/lex-net/LEX_NET_LOCAL_RECOGNITION_V1.md"
MANIFEST = ROOT / "docs/lex-net/lex_net_001_manifest.json"

EXPECTED_IDS = [f"LR-{i:03d}" for i in range(1, 11)]
EXPECTED_DISPOSITIONS = [
    "RecognizedEvidence",
    "NeedsAdditionalEvidence",
    "Unsupported",
    "Rejected",
    "Indeterminate",
]
EXPECTED_FIXTURE_IDS = [
    "recognized-minimal",
    "unrecognized-issuer",
    "unsupported-profile-version",
    "expired-evidence",
    "unknown-currentness",
    "missing-supplementary-evidence",
    "caller-policy-substitution",
    "wrong-purpose",
    "claim-projection-does-not-promote-capability",
    "transitive-recognition-refused",
    "policy-unavailable",
]
REQUIRED_DOC_PHRASES = [
    "Recognized evidence is still evidence.",
    "Recognition is a technical local policy result.",
    "A recognition receipt is not a reusable capability token.",
    "A recognizes B",
    "no ambient wall clock",
]


def fail(msg: str) -> None:
    raise SystemExit(f"LEX-NET-001 validation failed: {msg}")


def evaluate(policy: dict, fixture: dict) -> dict:
    # This is intentionally pure and deterministic over the frozen fixture.
    if fixture.get("policy_available") is not True:
        return result("Indeterminate", "local-policy-unavailable")

    if fixture.get("policy_selection") != "local":
        return result("Rejected", "local-policy-selection-required")

    if fixture.get("recognition_path") != "direct":
        return result("Rejected", "direct-recognition-path-required")

    supported = policy.get("supported_profiles", {})
    versions = supported.get(fixture.get("profile_id"))
    if not isinstance(versions, list) or fixture.get("profile_version") not in versions:
        return result("Unsupported", "profile-or-version-unsupported")

    if fixture.get("issuer") not in policy.get("recognized_issuers", []):
        return result("Rejected", "issuer-not-recognized")

    currentness = fixture.get("currentness")
    if currentness == "unknown":
        return result("Indeterminate", "currentness-unestablished")
    if currentness == "expired":
        return result("Rejected", "evidence-expired")
    if currentness != "current":
        return result("Indeterminate", "currentness-unestablished")

    if fixture.get("purpose") != policy.get("purpose") or fixture.get("resource") != policy.get("resource"):
        return result("Rejected", "purpose-or-resource-not-permitted")

    if fixture.get("supplementary_present") is not True:
        return result("NeedsAdditionalEvidence", "required-supplementary-evidence-missing")

    allowed = set(policy.get("allowed_claims", []))
    present = set(fixture.get("present_claims", []))
    requested = set(fixture.get("requested_claims", []))
    recognized = sorted(allowed & present & requested)
    if not recognized:
        return result("Rejected", "recognized-claim-projection-empty")

    return result("RecognizedEvidence", "recognized-projection", recognized)


def result(disposition: str, reason: str, claims=None) -> dict:
    return {
        "disposition": disposition,
        "reason": reason,
        "recognized_claims": sorted(claims or []),
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
    }


def main() -> None:
    if not DOC.is_file() or not MANIFEST.is_file():
        fail("required profile/manifest file missing")

    doc = DOC.read_text(encoding="utf-8")
    doc_folded = doc.casefold()
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    if data.get("profile") != "mycelix-lex-net-local-recognition-v1":
        fail("unexpected profile id")
    if data.get("qualified_parent_head") != "497182225c76fe4db6531422f7476dcef6b25365":
        fail("qualified parent head changed")
    if data.get("dispositions") != EXPECTED_DISPOSITIONS:
        fail("disposition census/order changed")

    invariants = data.get("invariants")
    if not isinstance(invariants, list):
        fail("invariants must be a list")
    ids = [item.get("id") for item in invariants]
    if ids != EXPECTED_IDS:
        fail(f"invariant order/census mismatch: {ids}")
    names = [item.get("name") for item in invariants]
    if len(names) != len(set(names)) or any(not isinstance(n, str) or not n for n in names):
        fail("invariant names must be unique non-empty strings")

    policy = data.get("reference_policy")
    if not isinstance(policy, dict):
        fail("reference policy missing")
    if policy.get("policy_id") != "local-recognition-test-policy" or policy.get("policy_version") != "1.0.0":
        fail("reference policy identity changed")

    fixtures = data.get("fixtures")
    if not isinstance(fixtures, list):
        fail("fixtures must be a list")
    fixture_ids = [fixture.get("id") for fixture in fixtures]
    if fixture_ids != EXPECTED_FIXTURE_IDS:
        fail(f"fixture census/order mismatch: {fixture_ids}")

    observed_dispositions = set()
    for fixture in fixtures:
        expected = fixture.get("expected")
        if not isinstance(expected, dict):
            fail(f"fixture {fixture.get('id')} missing expected result")
        actual = evaluate(policy, fixture)
        observed_dispositions.add(actual["disposition"])
        comparable_actual = {
            "disposition": actual["disposition"],
            "reason": actual["reason"],
            "recognized_claims": actual["recognized_claims"],
        }
        if comparable_actual != expected:
            fail(
                f"fixture {fixture.get('id')} mismatch: "
                f"expected={expected!r} actual={comparable_actual!r}"
            )
        if actual["grants_local_authority"] or actual["grants_external_effect_authority"]:
            fail(f"fixture {fixture.get('id')} illegally grants authority/effect")

    if observed_dispositions != set(EXPECTED_DISPOSITIONS):
        fail(f"fixture corpus does not exercise all dispositions: {sorted(observed_dispositions)}")

    nonclaims = data.get("mandatory_nonclaims")
    if not isinstance(nonclaims, list) or len(nonclaims) != 10:
        fail("mandatory nonclaim census changed")
    for claim in nonclaims:
        if claim.casefold() not in doc_folded:
            fail(f"mandatory nonclaim absent from profile: {claim}")

    for phrase in REQUIRED_DOC_PHRASES:
        if phrase.casefold() not in doc_folded:
            fail(f"required profile phrase missing: {phrase}")

    headings = [
        line.split(" — ", 1)[0].strip("# ")
        for line in doc.splitlines()
        if line.startswith("## LR-")
    ]
    if headings != EXPECTED_IDS:
        fail(f"profile invariant heading order/census mismatch: {headings}")

    forbidden = [
        "foreign authenticated evidence -> local authority",
        "recognition grants authority",
        "recognized evidence is legally valid",
        "recognized evidence is legally enforceable",
        "indeterminate is recognized",
    ]
    for phrase in forbidden:
        if phrase.casefold() in doc_folded:
            fail(f"forbidden positive claim present: {phrase}")

    print(json.dumps({
        "profile": data["profile"],
        "qualified_parent_head": data["qualified_parent_head"],
        "invariants": len(invariants),
        "fixtures": len(fixtures),
        "dispositions_exercised": sorted(observed_dispositions),
        "status": "PASS",
        "grants_local_authority": False,
        "grants_external_effect_authority": False,
        "network_access_required": False,
        "ambient_clock_required": False,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
