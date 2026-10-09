#!/usr/bin/env python3
"""Bounded, deterministic delegation-chain claim invariant checker.

This verifies parsed fixture claims and a custom canonical parent-envelope
commitment. It does NOT parse or verify JWTs, signatures, issuer trust, holder
keys, or proof-of-possession, and its commitment format is NOT the AAT par_hash
wire encoding.
"""
from __future__ import annotations

import hashlib
import json
import re
from typing import Any

CLAIMS_SCHEMA = "mycelix.delegation-chain-claims.v1"
RESULT_SCHEMA = "mycelix.delegation-chain-claims-result.v1"

# Frozen candidate resource limits. Tests independently repeat these values.
MAX_DELEGATION_DEPTH = 8
MAX_TOKEN_LIFETIME = 90 * 24 * 60 * 60
MAX_IAT_SKEW = 30
_SHA256_HEX = re.compile(r"^[0-9a-f]{64}$")


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def envelope_digest(hop: dict[str, Any]) -> str:
    """Hash the exact normalized fixture envelope, including all supplied claims."""
    normalized = {
        "id": hop["id"],
        "claims": hop["claims"],
        "payload": hop["payload"],
    }
    return hashlib.sha256(canonical_bytes(normalized)).hexdigest()


def _int_claim(claims: dict[str, Any], name: str, hop_id: str,
               findings: list[dict[str, Any]]) -> int | None:
    value = claims.get(name)
    if type(value) is not int:
        findings.append({"code": "claim-not-integer", "hop_id": hop_id, "claim": name})
        return None
    return value


def evaluate_claims_chain(raw: dict[str, Any]) -> dict[str, Any]:
    if raw.get("schema") != CLAIMS_SCHEMA:
        return {
            "schema": RESULT_SCHEMA,
            "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "findings": [{"code": "unsupported-schema"}],
            "qualification": "NOT_CLAIMED",
        }
    now = raw.get("now")
    hops = raw.get("hops")
    if type(now) is not int or not isinstance(hops, list) or not hops:
        return {
            "schema": RESULT_SCHEMA,
            "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "findings": [{"code": "malformed-top-level-input"}],
            "qualification": "NOT_CLAIMED",
        }
    if len(hops) > MAX_DELEGATION_DEPTH + 1:
        return {
            "schema": RESULT_SCHEMA,
            "status": "INVALID_CHAIN",
            "hop_count": len(hops),
            "findings": [{"code": "implementation-chain-depth-exceeded",
                          "maximum_depth": MAX_DELEGATION_DEPTH}],
            "qualification": "NOT_CLAIMED",
        }

    findings: list[dict[str, Any]] = []
    if not hops:
        findings.append({"code": "empty-chain"})
    ids: list[str] = []
    jtis: list[str] = []
    envelopes_valid = True

    for index, hop in enumerate(hops):
        if not isinstance(hop, dict):
            findings.append({"code": "hop-not-object", "hop_index": index})
            envelopes_valid = False
            continue
        hop_id = hop.get("id")
        claims = hop.get("claims")
        payload = hop.get("payload")
        if not isinstance(hop_id, str) or not hop_id:
            findings.append({"code": "hop-id-missing", "hop_index": index})
            envelopes_valid = False
            hop_id = f"<invalid-index-{index}>"
        ids.append(hop_id)
        if not isinstance(claims, dict) or not isinstance(payload, dict):
            findings.append({"code": "claims-or-payload-not-object", "hop_id": hop_id})
            envelopes_valid = False
            continue

        jti = claims.get("jti")
        if not isinstance(jti, str) or not jti:
            findings.append({"code": "jti-missing", "hop_id": hop_id})
        else:
            jtis.append(jti)

        iat = _int_claim(claims, "iat", hop_id, findings)
        exp = _int_claim(claims, "exp", hop_id, findings)
        del_depth = _int_claim(claims, "del_depth", hop_id, findings)
        del_max_depth = _int_claim(claims, "del_max_depth", hop_id, findings)

        if del_depth is not None and del_max_depth is not None:
            if del_depth < 0 or del_max_depth < 0 or del_depth > del_max_depth:
                findings.append({"code": "depth-range-invalid", "hop_id": hop_id,
                                 "del_depth": del_depth, "del_max_depth": del_max_depth})
            if del_max_depth > MAX_DELEGATION_DEPTH:
                findings.append({"code": "maximum-depth-exceeded", "hop_id": hop_id,
                                 "del_max_depth": del_max_depth,
                                 "limit": MAX_DELEGATION_DEPTH})

        if iat is not None and exp is not None:
            if exp <= now:
                findings.append({"code": "token-expired", "hop_id": hop_id, "exp": exp, "now": now})
            if exp <= iat:
                findings.append({"code": "expiry-not-after-issue", "hop_id": hop_id,
                                 "iat": iat, "exp": exp})
            if iat > now + MAX_IAT_SKEW:
                findings.append({"code": "issued-at-beyond-skew", "hop_id": hop_id,
                                 "iat": iat, "now": now, "maximum_skew": MAX_IAT_SKEW})
            if exp > iat + MAX_TOKEN_LIFETIME:
                findings.append({"code": "token-lifetime-exceeded", "hop_id": hop_id,
                                 "iat": iat, "exp": exp, "maximum_lifetime": MAX_TOKEN_LIFETIME})

        commitment = claims.get("parent_envelope_sha256")
        if index == 0:
            if "parent_envelope_sha256" in claims:
                findings.append({"code": "root-has-parent-commitment", "hop_id": hop_id})
            if del_depth is not None and del_depth != 0:
                findings.append({"code": "root-depth-not-zero", "hop_id": hop_id,
                                 "del_depth": del_depth})
        else:
            if not isinstance(commitment, str) or not _SHA256_HEX.fullmatch(commitment):
                findings.append({"code": "parent-commitment-malformed", "hop_id": hop_id})
            else:
                parent = hops[index - 1]
                if not isinstance(parent, dict) or not isinstance(parent.get("claims"), dict) \
                        or not isinstance(parent.get("payload"), dict):
                    findings.append({"code": "parent-envelope-unavailable", "hop_id": hop_id})
                else:
                    try:
                        expected_digest = envelope_digest(parent)
                        if commitment != expected_digest:
                            findings.append({"code": "parent-commitment-mismatch",
                                             "hop_id": hop_id, "parent_id": parent.get("id"),
                                             "expected_sha256": expected_digest,
                                             "observed_sha256": commitment})
                    except (TypeError, ValueError):
                        findings.append({"code": "parent-envelope-not-canonicalizable",
                                         "hop_id": hop_id, "parent_id": parent.get("id")})

            parent = hops[index - 1]
            if isinstance(parent, dict) and isinstance(parent.get("claims"), dict):
                p_claims = parent["claims"]
                p_iat = p_claims.get("iat")
                p_exp = p_claims.get("exp")
                p_depth = p_claims.get("del_depth")
                p_max = p_claims.get("del_max_depth")
                if all(type(value) is int for value in (iat, exp, del_depth, del_max_depth,
                                                        p_iat, p_exp, p_depth, p_max)):
                    if del_depth != p_depth + 1:
                        findings.append({"code": "delegation-depth-not-incremented-by-one",
                                         "hop_id": hop_id, "parent_id": parent.get("id"),
                                         "parent_depth": p_depth, "child_depth": del_depth})
                    if del_depth > p_max:
                        findings.append({"code": "parent-depth-budget-exceeded",
                                         "hop_id": hop_id, "parent_id": parent.get("id"),
                                         "parent_max_depth": p_max, "child_depth": del_depth})
                    if del_max_depth > p_max:
                        findings.append({"code": "maximum-depth-budget-expanded",
                                         "hop_id": hop_id, "parent_id": parent.get("id"),
                                         "parent_max_depth": p_max, "child_max_depth": del_max_depth})
                    if exp > p_exp:
                        findings.append({"code": "child-expiry-exceeds-parent",
                                         "hop_id": hop_id, "parent_id": parent.get("id"),
                                         "parent_exp": p_exp, "child_exp": exp})
                    if iat < p_iat:
                        findings.append({"code": "child-iat-before-parent",
                                         "hop_id": hop_id, "parent_id": parent.get("id"),
                                         "parent_iat": p_iat, "child_iat": iat})

    if len(ids) != len(set(ids)):
        findings.append({"code": "duplicate-hop-id"})
    if len(jtis) != len(set(jtis)):
        findings.append({"code": "duplicate-jti"})

    # Validate envelopes with an independent deterministic digest recomputation
    # even when policy claim checks already produced errors.
    digests: list[str] = []
    if envelopes_valid:
        for hop in hops:
            try:
                digests.append(envelope_digest(hop))
            except (TypeError, ValueError):
                findings.append({"code": "envelope-not-canonicalizable"})
                break

    status = "DELEGATION_CHAIN_CLAIMS_PASS" if not findings else "INVALID_CHAIN"
    return {
        "schema": RESULT_SCHEMA,
        "status": status,
        "hop_count": len(hops),
        "findings": findings,
        "envelope_sha256": digests,
        "limits": {
            "max_delegation_depth": MAX_DELEGATION_DEPTH,
            "max_token_lifetime_seconds": MAX_TOKEN_LIFETIME,
            "max_iat_skew_seconds": MAX_IAT_SKEW,
        },
        "qualification": "NOT_CLAIMED",
        "scope": (
            "parsed-claim temporal/depth/ID/fixture-commitment checks only; "
            "custom parent envelope digest is not AAT par_hash and confers no authenticity; "
            "JWT parsing, signatures, issuer trust, and holder proof are not implemented"
        ),
    }
