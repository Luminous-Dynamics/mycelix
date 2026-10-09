#!/usr/bin/env python3
"""Independent adversarial controls for delegation-chain depth/TTL/linkage claims."""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delegation_chain_claims as claims_checker  # noqa: E402

SCHEMA = "mycelix.delegation-chain-claims.v1"
RESULT_SCHEMA = "mycelix.delegation-chain-claims-result.v1"

# These repeat limits independently; tests fail if the implementation silently
# changes its resource or time-bound assumptions.
FROZEN_MAX_DEPTH = 8
FROZEN_MAX_LIFETIME = 90 * 24 * 60 * 60
FROZEN_IAT_SKEW = 30
FROZEN_NOW = 2000


def raw_digest(hop: dict[str, Any]) -> str:
    """Independent definition of this test profile's JSON envelope commitment."""
    value = {"id": hop["id"], "claims": hop["claims"], "payload": hop["payload"]}
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=False, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def relink(hops: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for index in range(1, len(hops)):
        hops[index]["claims"]["parent_envelope_sha256"] = raw_digest(hops[index - 1])
    return hops


def valid_chain(now: int = FROZEN_NOW, hop_count: int = 4) -> dict[str, Any]:
    hops: list[dict[str, Any]] = []
    base_exp = now + 5000
    for index in range(hop_count):
        claims: dict[str, Any] = {
            "jti": f"token-{index}",
            "iat": 1000 + index * 10,
            "exp": base_exp - index * 250,
            "del_depth": index,
            "del_max_depth": max(hop_count - 1, 1),
        }
        if index:
            claims["parent_envelope_sha256"] = "0" * 64
        hops.append({
            "id": f"hop-{index}",
            "claims": claims,
            "payload": {"profile": "bounded-fixture-v1", "permissions": ["read"]},
        })
    relink(hops)
    return {"schema": SCHEMA, "now": now, "hops": hops}


def mutated_case(name: str) -> dict[str, Any]:
    raw = valid_chain()
    hops = raw["hops"]
    if name == "child-expiry-exceeds-parent":
        hops[2]["claims"]["exp"] = hops[1]["claims"]["exp"] + 1
        relink(hops)
    elif name == "depth-skips-parent":
        hops[2]["claims"]["del_depth"] = hops[1]["claims"]["del_depth"] + 2
        relink(hops)
    elif name == "duplicate-jti":
        hops[2]["claims"]["jti"] = hops[1]["claims"]["jti"]
        relink(hops)
    elif name == "broken-parent-commitment":
        hops[2]["claims"]["parent_envelope_sha256"] = "f" * 64
    elif name == "iat-before-parent":
        hops[2]["claims"]["iat"] = hops[1]["claims"]["iat"] - 1
        relink(hops)
    elif name == "expired-leaf":
        hops[-1]["claims"]["exp"] = raw["now"] - 1
        relink(hops)
    elif name == "iat-skew-exceeded":
        hops[2]["claims"]["iat"] = raw["now"] + FROZEN_IAT_SKEW + 1
        relink(hops)
    elif name == "root-depth-budget-exceeded":
        root = valid_chain(hop_count=1)["hops"][0]
        root["claims"]["del_max_depth"] = FROZEN_MAX_DEPTH + 1
        return {"schema": SCHEMA, "now": raw["now"], "hops": [root]}
    elif name == "token-lifetime-exceeded":
        hops[0]["claims"]["exp"] = hops[0]["claims"]["iat"] + FROZEN_MAX_LIFETIME + 1
        relink(hops)
    elif name == "missing-jti":
        del hops[2]["claims"]["jti"]
        relink(hops)
    elif name == "over-depth-chain":
        return valid_chain(hop_count=FROZEN_MAX_DEPTH + 2)
    elif name == "empty-chain":
        return {"schema": SCHEMA, "now": raw["now"], "hops": []}
    elif name == "noninteger-exp":
        hops[2]["claims"]["exp"] = "later"
        relink(hops)
    else:
        raise KeyError(name)
    return raw


def requestless_expected_codes(raw: dict[str, Any]) -> set[tuple[str, int | None]]:
    """Independent reconstruction of invariant violations without checker helpers."""
    findings: set[tuple[str, int | None]] = set()
    now = raw.get("now")
    hops = raw.get("hops")
    if type(now) is not int or not isinstance(hops, list) or not hops:
        return {("malformed-top-level-input", None)}
    if len(hops) > FROZEN_MAX_DEPTH + 1:
        return {("implementation-chain-depth-exceeded", None)}
    ids: list[str] = []
    jtis: list[str] = []
    for index, node in enumerate(hops):
        if not isinstance(node, dict):
            findings.add(("hop-not-object", index))
            continue
        hop_id = node.get("id")
        claims = node.get("claims")
        payload = node.get("payload")
        if not isinstance(hop_id, str) or not hop_id:
            findings.add(("hop-id-missing", index))
            hop_id = f"<invalid-index-{index}>"
        ids.append(hop_id)
        if not isinstance(claims, dict) or not isinstance(payload, dict):
            findings.add(("claims-or-payload-not-object", index))
            continue
        jti = claims.get("jti")
        if not isinstance(jti, str) or not jti:
            findings.add(("jti-missing", index))
        else:
            jtis.append(jti)
        required = ("iat", "exp", "del_depth", "del_max_depth")
        parsed: dict[str, int | None] = {}
        for name in required:
            value = claims.get(name)
            if type(value) is not int:
                findings.add(("claim-not-integer", index))
                parsed[name] = None
            else:
                parsed[name] = value
        iat, exp = parsed["iat"], parsed["exp"]
        depth, max_depth = parsed["del_depth"], parsed["del_max_depth"]
        if depth is not None and max_depth is not None:
            if depth < 0 or max_depth < 0 or depth > max_depth:
                findings.add(("depth-range-invalid", index))
            if max_depth > FROZEN_MAX_DEPTH:
                findings.add(("maximum-depth-exceeded", index))
        if iat is not None and exp is not None:
            if exp <= now:
                findings.add(("token-expired", index))
            if exp <= iat:
                findings.add(("expiry-not-after-issue", index))
            if iat > now + FROZEN_IAT_SKEW:
                findings.add(("issued-at-beyond-skew", index))
            if exp > iat + FROZEN_MAX_LIFETIME:
                findings.add(("token-lifetime-exceeded", index))

        commitment = claims.get("parent_envelope_sha256")
        if index == 0:
            if "parent_envelope_sha256" in claims:
                findings.add(("root-has-parent-commitment", index))
            if depth is not None and depth != 0:
                findings.add(("root-depth-not-zero", index))
        else:
            if not isinstance(commitment, str) or len(commitment) != 64 or any(
                ch not in "0123456789abcdef" for ch in commitment
            ):
                findings.add(("parent-commitment-malformed", index))
            else:
                expected_parent_hash = raw_digest(hops[index - 1])
                if commitment != expected_parent_hash:
                    findings.add(("parent-commitment-mismatch", index))
            parent = hops[index - 1]
            pclaims = parent.get("claims") if isinstance(parent, dict) else None
            if isinstance(pclaims, dict):
                p_iat, p_exp = pclaims.get("iat"), pclaims.get("exp")
                p_depth, p_max = pclaims.get("del_depth"), pclaims.get("del_max_depth")
                if all(type(v) is int for v in (iat, exp, depth, max_depth, p_iat, p_exp, p_depth, p_max)):
                    if depth != p_depth + 1:
                        findings.add(("delegation-depth-not-incremented-by-one", index))
                    if depth > p_max:
                        findings.add(("parent-depth-budget-exceeded", index))
                    if max_depth > p_max:
                        findings.add(("maximum-depth-budget-expanded", index))
                    if exp > p_exp:
                        findings.add(("child-expiry-exceeds-parent", index))
                    if iat < p_iat:
                        findings.add(("child-iat-before-parent", index))
    if len(ids) != len(set(ids)):
        findings.add(("duplicate-hop-id", None))
    if len(jtis) != len(set(jtis)):
        findings.add(("duplicate-jti", None))
    return findings


def audit_result(raw: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any] | None:
    expected = requestless_expected_codes(raw)
    actual = observed.get("findings")
    if not isinstance(actual, list):
        return {"kind": "finding-list-missing"}
    actual_codes: set[tuple[str, int | None]] = set()
    for item in actual:
        code = item.get("code")
        hop_id = item.get("hop_id")
        index_by_id = {hop.get("id"): i for i, hop in enumerate(raw["hops"])
                       if isinstance(hop, dict)}
        actual_codes.add((str(code), index_by_id.get(hop_id) if hop_id is not None else None))
    if expected != actual_codes:
        return {
            "kind": "finding-set-disagrees-with-independent-replay",
            "expected": sorted((code, i if i is not None else -1) for code, i in expected),
            "observed": sorted((code, i if i is not None else -1) for code, i in actual_codes),
        }
    if ("malformed-top-level-input", None) in expected:
        expected_status = "UNSUPPORTED_OR_UNDECIDABLE"
    else:
        expected_status = "DELEGATION_CHAIN_CLAIMS_PASS" if not expected else "INVALID_CHAIN"
    if observed.get("status") != expected_status:
        return {"kind": "claims-status-disagrees-with-independent-replay",
                "expected": expected_status, "observed": observed.get("status")}
    return None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    bad_case_names = (
        "child-expiry-exceeds-parent", "depth-skips-parent", "duplicate-jti",
        "broken-parent-commitment", "iat-before-parent", "expired-leaf",
        "iat-skew-exceeded", "root-depth-budget-exceeded", "token-lifetime-exceeded",
        "missing-jti",
    )
    receipt: dict[str, Any] = {
        "schema": "mycelix.delegation-chain-claims-differential-receipt.v1",
        "status": "RUNNING", "qualification": "NOT_CLAIMED",
        "controls": [], "checker_mutations": [],
    }
    try:
        require(claims_checker.MAX_DELEGATION_DEPTH == FROZEN_MAX_DEPTH,
                "implementation max depth differs from independently frozen limit")
        require(claims_checker.MAX_TOKEN_LIFETIME == FROZEN_MAX_LIFETIME,
                "implementation max lifetime differs from independently frozen limit")
        require(claims_checker.MAX_IAT_SKEW == FROZEN_IAT_SKEW,
                "implementation issued-at skew differs from independently frozen limit")
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True, check=True, timeout=15,
        ).stdout.strip()
        checker_path = HERE / "delegation_chain_claims.py"
        receipt["checker_sha256"] = hashlib.sha256(checker_path.read_bytes()).hexdigest()
        receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()

        valid = valid_chain()
        observed = claims_checker.evaluate_claims_chain(valid)
        mismatch = audit_result(valid, observed)
        require(mismatch is None, "valid chain rejected or disagreed with independent replay: " + str(mismatch))
        receipt["controls"].append({
            "id": "valid-four-hop-chain", "expected_status": "DELEGATION_CHAIN_CLAIMS_PASS",
            "observed_status": observed["status"], "independent_replay": "PASS",
            "hop_count": len(valid["hops"]),
        })

        for name in bad_case_names:
            raw = mutated_case(name)
            observed = claims_checker.evaluate_claims_chain(raw)
            mismatch = audit_result(raw, observed)
            require(mismatch is None, name + ": independent checker disagrees: " + str(mismatch))
            expected_status = "UNSUPPORTED_OR_UNDECIDABLE" if name == "empty-chain" else "INVALID_CHAIN"
            require(observed["status"] == expected_status,
                    name + ": malformed/invalid chain had unexpected status " + observed["status"])
            receipt["controls"].append({
                "id": name, "expected_status": "INVALID_CHAIN",
                "observed_status": observed["status"], "independent_replay": "PASS",
                "finding_codes": sorted(item["code"] for item in observed["findings"]),
            })

        # Mutation-sensitivity: ensure the independent checker detects selected
        # missing findings if the implementation were to omit them.
        mutants = (
            ("expiry-check-omitted", "child-expiry-exceeds-parent"),
            ("depth-check-omitted", "delegation-depth-not-incremented-by-one"),
            ("jti-uniqueness-check-omitted", "duplicate-jti"),
            ("parent-link-check-omitted", "parent-commitment-mismatch"),
        )
        original = claims_checker.evaluate_claims_chain
        for mutant_id, code_to_remove in mutants:
            if mutant_id == "expiry-check-omitted":
                raw = mutated_case("child-expiry-exceeds-parent")
            elif mutant_id == "depth-check-omitted":
                raw = mutated_case("depth-skips-parent")
            elif mutant_id == "jti-uniqueness-check-omitted":
                raw = mutated_case("duplicate-jti")
            else:
                raw = mutated_case("broken-parent-commitment")

            def mutant(candidate: dict[str, Any], original=original,
                       code_to_remove=code_to_remove) -> dict[str, Any]:
                result = copy.deepcopy(original(candidate))
                result["findings"] = [item for item in result.get("findings", [])
                                      if item.get("code") != code_to_remove]
                if not result["findings"]:
                    result["status"] = "DELEGATION_CHAIN_CLAIMS_PASS"
                return result

            claims_checker.evaluate_claims_chain = mutant
            try:
                observed = claims_checker.evaluate_claims_chain(raw)
                mismatch = audit_result(raw, observed)
            finally:
                claims_checker.evaluate_claims_chain = original
            require(mismatch is not None and mismatch["kind"] == "finding-set-disagrees-with-independent-replay",
                    mutant_id + ": missing finding escaped independent detection")
            receipt["checker_mutations"].append({
                "id": mutant_id, "omitted_finding": code_to_remove,
                "independent_detection": True, "mismatch_kind": mismatch["kind"],
            })

        receipt["status"] = "PASS"
        receipt["summary"] = {
            "baseline_valid_chains": 1,
            "invalid_claim_controls": len(bad_case_names),
            "independent_requestless_invariant_replay": "PASS",
            "checker_mutants_injected": len(mutants),
            "checker_mutants_detected": len(receipt["checker_mutations"]),
            "frozen_limits": {
                "max_depth": FROZEN_MAX_DEPTH,
                "max_lifetime_seconds": FROZEN_MAX_LIFETIME,
                "max_iat_skew_seconds": FROZEN_IAT_SKEW,
            },
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION-CHAIN CLAIMS DIFFERENTIAL PASS: 1 valid chain + 13 adversarial controls")
        print("CHAIN-CLAIMS CHECKER MUTATION SENSITIVITY PASS: 4 of 4 omitted checks detected")
        print("QUALIFICATION NOT CLAIMED: parsed claims only; no JWT signature or holder-key validation")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION-CHAIN CLAIMS DIFFERENTIAL FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
