#!/usr/bin/env python3
"""Independent controls for parsed AAT-style issuer/JWK thumbprint linkage.

The reference computes RFC 7638 required-member JWK thumbprints independently.
No JWT signature, JWS signing input, parent par_hash, or proof-of-possession is
validated by this test.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delegation_chain_key_linkage as checker  # noqa: E402

SCHEMA = "mycelix.delegation-chain-key-link.v1"
PREFIX = "urn:ietf:params:oauth:jwk-thumbprint:sha-256:"
PRIVATE_FIELDS = {"d", "p", "q", "dp", "dq", "qi", "oth", "k"}
X_VALUES = [
    "11qYAYKxCrfVS_7TyWQHOg7hcvPapiMlrwIaaPcHURo",
    "rAl9xvTDAeUADPnIWlGpFHtGg4Y8OqcQE5N4XYNdLPs",
    "11qYAYKxCrfVS_7TyWQHOg7hcvPapiMlrwIaaPcHURo",
    "rAl9xvTDAeUADPnIWlGpFHtGg4Y8OqcQE5N4XYNdLPs",
]


def independent_uri(jwk: Any) -> str:
    if not isinstance(jwk, dict) or jwk.get("kty") != "OKP" or jwk.get("crv") != "Ed25519":
        raise ValueError("unsupported public-key profile")
    if PRIVATE_FIELDS.intersection(jwk):
        raise ValueError("private material present")
    x = jwk.get("x")
    if not isinstance(x, str) or not re.fullmatch(r"[A-Za-z0-9_-]{43}", x):
        raise ValueError("invalid x")
    decoded = base64.urlsafe_b64decode(x + "=")
    if len(decoded) != 32:
        raise ValueError("wrong Ed25519 public-key length")
    if base64.urlsafe_b64encode(decoded).rstrip(b"=").decode("ascii") != x:
        raise ValueError("noncanonical x encoding")
    required = {"crv": "Ed25519", "kty": "OKP", "x": x}
    canonical = json.dumps(required, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False).encode("utf-8")
    digest = hashlib.sha256(canonical).digest()
    encoded = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")
    return PREFIX + encoded


def key(index: int) -> dict[str, Any]:
    return {"kty": "OKP", "crv": "Ed25519", "x": X_VALUES[index]}


def fixture() -> dict[str, Any]:
    public_keys = [key(i) for i in range(4)]
    hops: list[dict[str, Any]] = []
    for index, pub in enumerate(public_keys):
        claims = {
            "jti": f"chain-test-{index}",
            "iss": "https://issuer.example.test" if index == 0 else independent_uri(public_keys[index - 1]),
            "cnf": {"jwk": copy.deepcopy(pub)},
        }
        hops.append({"id": f"hop-{index}", "claims": claims, "payload": {"fixture": "v1"}})
    return {"schema": SCHEMA, "hops": hops}


def index_by_id(raw: dict[str, Any]) -> dict[str, int]:
    return {hop["id"]: i for i, hop in enumerate(raw["hops"]) if isinstance(hop, dict)}


def independent_findings(raw: dict[str, Any]) -> set[tuple[str, int | None]]:
    hops = raw.get("hops")
    if not isinstance(hops, list) or not hops:
        return {("malformed-chain", None)}
    if len(hops) > 9:
        return {("implementation-chain-depth-exceeded", None)}
    findings: set[tuple[str, int | None]] = set()
    for index, hop in enumerate(hops):
        if not isinstance(hop, dict) or not isinstance(hop.get("claims"), dict):
            findings.add(("hop-claims-malformed", index))
            continue
        claims = hop["claims"]
        hop_id = hop.get("id")
        issuer = claims.get("iss")
        if not isinstance(issuer, str) or not issuer:
            findings.add(("issuer-missing", index))
        cnf = claims.get("cnf")
        jwk = cnf.get("jwk") if isinstance(cnf, dict) else None
        try:
            independent_uri(jwk)
        except ValueError:
            findings.add(("unsupported-or-invalid-holder-key", index))
        if index == 0:
            if isinstance(issuer, str) and issuer.startswith(PREFIX):
                findings.add(("root-issuer-must-be-trust-anchor-uri", index))
            continue
        parent = hops[index - 1]
        parent_claims = parent.get("claims") if isinstance(parent, dict) else None
        parent_cnf = parent_claims.get("cnf") if isinstance(parent_claims, dict) else None
        parent_jwk = parent_cnf.get("jwk") if isinstance(parent_cnf, dict) else None
        try:
            expected = independent_uri(parent_jwk)
        except ValueError:
            findings.add(("parent-holder-key-unverifiable", index))
            continue
        if issuer != expected:
            findings.add(("issuer-thumbprint-mismatch", index))
    return findings


def audit(raw: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any] | None:
    if raw.get("schema") != SCHEMA:
        return None
    ids = index_by_id(raw)
    observed_findings: set[tuple[str, int | None]] = set()
    for item in observed.get("findings", []):
        hop_id = item.get("hop_id")
        hop_index = ids.get(hop_id) if hop_id is not None else item.get("hop_index")
        observed_findings.add((str(item.get("code")), hop_index))
    expected = independent_findings(raw)
    if observed_findings != expected:
        return {
            "kind": "finding-set-disagrees-with-independent-replay",
            "expected": sorted((code, -1 if index is None else index) for code, index in expected),
            "observed": sorted((code, -1 if index is None else index) for code, index in observed_findings),
        }
    if len(raw["hops"]) > 9:
        expected_status = "UNSUPPORTED_OR_UNDECIDABLE"
    else:
        expected_status = "KEY_LINKAGE_CLAIMS_PASS" if not expected else "INVALID_CHAIN"
    if observed.get("status") != expected_status:
        return {"kind": "status-disagrees-with-independent-replay",
                "expected": expected_status, "observed": observed.get("status")}
    return None


def bad_cases() -> dict[str, dict[str, Any]]:
    cases: dict[str, dict[str, Any]] = {}
    raw = fixture()
    raw["hops"][2]["claims"]["iss"] = PREFIX + "A" * 43
    cases["wrong-derived-issuer"] = raw

    raw = fixture()
    raw["hops"][0]["claims"]["iss"] = independent_uri(key(0))
    cases["root-thumbprint-issuer"] = raw

    raw = fixture()
    raw["hops"][1]["claims"]["cnf"]["jwk"]["d"] = "private-material"
    cases["private-key-in-cnf"] = raw

    raw = fixture()
    raw["hops"][2]["claims"]["cnf"]["jwk"]["crv"] = "P-256"
    cases["unsupported-key-profile"] = raw

    raw = fixture()
    raw["hops"][2]["claims"]["cnf"]["jwk"]["x"] = "B" * 43
    cases["noncanonical-coordinate-encoding"] = raw

    raw = fixture()
    del raw["hops"][2]["claims"]["iss"]
    cases["missing-derived-issuer"] = raw

    raw = fixture()
    raw["hops"] = raw["hops"] * 3
    cases["over-depth-chain"] = raw

    raw = fixture()
    raw["schema"] = "future-v2"
    cases["unsupported-schema"] = raw
    return cases


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.delegation-chain-key-linkage-differential-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "controls": [],
        "mutations": [],
    }
    try:
        require(checker.THUMBPRINT_URI_PREFIX == PREFIX, "thumbprint URI prefix drifted")
        valid = fixture()
        observed = checker.evaluate_key_chain(valid)
        mismatch = audit(valid, observed)
        require(mismatch is None, "valid key-link fixture disagrees with independent replay: " + str(mismatch))
        receipt["controls"].append({
            "id": "valid-four-hop-key-link",
            "status": observed["status"],
            "independent_replay": "PASS",
            "hops": len(valid["hops"]),
        })

        for name, raw in bad_cases().items():
            observed = checker.evaluate_key_chain(raw)
            mismatch = audit(raw, observed)
            require(mismatch is None, name + ": independent replay mismatch: " + str(mismatch))
            if raw.get("schema") == SCHEMA:
                require(observed["status"] == "INVALID_CHAIN",
                        name + ": invalid chain was not rejected")
            else:
                require(observed["status"] == "UNSUPPORTED_OR_UNDECIDABLE",
                        name + ": unknown schema did not fail closed")
            receipt["controls"].append({
                "id": name,
                "status": observed["status"],
                "independent_replay": "PASS",
                "finding_codes": sorted(item["code"] for item in observed.get("findings", [])),
            })

        # Inject three missing-check regressions; the independent evaluator must
        # detect the incomplete finding set even if the status were preserved.
        mutations = (
            ("issuer-link-check-omitted", "wrong-derived-issuer", "issuer-thumbprint-mismatch"),
            ("private-jwk-rejection-omitted", "private-key-in-cnf", "unsupported-or-invalid-holder-key"),
            ("root-issuer-shape-check-omitted", "root-thumbprint-issuer",
             "root-issuer-must-be-trust-anchor-uri"),
        )
        original = checker.evaluate_key_chain
        cases = bad_cases()
        for mutation_id, case_id, removed_code in mutations:
            raw = cases[case_id]
            def mutant(candidate: dict[str, Any], original=original,
                       removed_code=removed_code) -> dict[str, Any]:
                result = copy.deepcopy(original(candidate))
                result["findings"] = [item for item in result.get("findings", [])
                                      if item.get("code") != removed_code]
                if not result["findings"]:
                    result["status"] = "KEY_LINKAGE_CLAIMS_PASS"
                return result
            checker.evaluate_key_chain = mutant
            try:
                observed = checker.evaluate_key_chain(raw)
                mismatch = audit(raw, observed)
            finally:
                checker.evaluate_key_chain = original
            require(mismatch is not None and mismatch["kind"] == "finding-set-disagrees-with-independent-replay",
                    mutation_id + ": omitted check escaped independent detection")
            receipt["mutations"].append({
                "id": mutation_id, "removed_finding": removed_code,
                "detected": True, "mismatch_kind": mismatch["kind"],
            })

        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        checker_path = HERE / "delegation_chain_key_linkage.py"
        receipt["checker_sha256"] = hashlib.sha256(checker_path.read_bytes()).hexdigest()
        receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "positive_controls": 1,
            "adversarial_controls": len(receipt["controls"]) - 1,
            "checker_mutants": len(receipt["mutations"]),
            "checker_mutants_detected": sum(row["detected"] for row in receipt["mutations"]),
            "independent_thumbprint": "RFC7638_REQUIRED_MEMBERS_SHA256",
            "profile": "OKP/Ed25519 only",
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION KEY LINKAGE PASS: 1 positive + 8 adversarial controls")
        print("KEY-LINKAGE MUTATION SENSITIVITY PASS: 3 of 3 omitted checks detected")
        print("QUALIFICATION NOT CLAIMED: no JWS signature, par_hash, trust-anchor, or PoP verification")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION KEY LINKAGE FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
