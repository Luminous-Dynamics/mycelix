#!/usr/bin/env python3
"""Bounded AAT-style issuer/holder-key linkage checks over parsed JWK claims.

Implements only the Ed25519 OKP JWK-thumbprint profile documented in RFC 7638
and the RFC 9278 thumbprint-URI form. It does not parse compact JWTs, verify JWS
signatures, verify par_hash, validate issuer trust anchors, or verify PoP.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
from typing import Any

KEY_LINK_SCHEMA = "mycelix.delegation-chain-key-link.v1"
RESULT_SCHEMA = "mycelix.delegation-chain-key-link-result.v1"
THUMBPRINT_URI_PREFIX = "urn:ietf:params:oauth:jwk-thumbprint:sha-256:"
_B64URL_RE = re.compile(r"^[A-Za-z0-9_-]{43}$")
_PRIVATE_FIELDS = {"d", "p", "q", "dp", "dq", "qi", "oth", "k"}


class UnsupportedKey(ValueError):
    pass


def canonical_thumbprint_jwk(jwk: dict[str, Any]) -> bytes:
    """Return RFC 7638 required-member JSON for the deliberately narrow profile."""
    if not isinstance(jwk, dict):
        raise UnsupportedKey("cnf.jwk must be an object")
    if jwk.get("kty") != "OKP" or jwk.get("crv") != "Ed25519":
        raise UnsupportedKey("only public OKP/Ed25519 JWKs are admitted by this profile")
    if _PRIVATE_FIELDS.intersection(jwk):
        raise UnsupportedKey("private key material is forbidden in cnf.jwk")
    x = jwk.get("x")
    if not isinstance(x, str) or not _B64URL_RE.fullmatch(x):
        raise UnsupportedKey("Ed25519 x must be a 43-character base64url value")
    try:
        decoded = base64.urlsafe_b64decode(x + "=")
    except (ValueError, base64.binascii.Error) as error:
        raise UnsupportedKey("invalid Ed25519 x encoding") from error
    if len(decoded) != 32:
        raise UnsupportedKey("Ed25519 x must encode exactly 32 bytes")
    canonical_x = base64.urlsafe_b64encode(decoded).rstrip(b"=").decode("ascii")
    if canonical_x != x:
        raise UnsupportedKey("Ed25519 x must use canonical unpadded base64url encoding")
    required_members = {"crv": "Ed25519", "kty": "OKP", "x": x}
    return json.dumps(required_members, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def jwk_thumbprint_uri(jwk: dict[str, Any]) -> str:
    thumbprint = hashlib.sha256(canonical_thumbprint_jwk(jwk)).digest()
    encoded = base64.urlsafe_b64encode(thumbprint).rstrip(b"=").decode("ascii")
    return THUMBPRINT_URI_PREFIX + encoded


def evaluate_key_chain(raw: dict[str, Any]) -> dict[str, Any]:
    if raw.get("schema") != KEY_LINK_SCHEMA:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "unsupported-schema"}],
                "qualification": "NOT_CLAIMED"}
    hops = raw.get("hops")
    if not isinstance(hops, list) or not hops:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "malformed-chain"}],
                "qualification": "NOT_CLAIMED"}
    if len(hops) > 9:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "implementation-chain-depth-exceeded", "limit": 8}],
                "qualification": "NOT_CLAIMED"}

    findings: list[dict[str, Any]] = []
    links: list[dict[str, Any]] = []
    for index, hop in enumerate(hops):
        if not isinstance(hop, dict) or not isinstance(hop.get("claims"), dict):
            findings.append({"code": "hop-claims-malformed", "hop_index": index})
            continue
        claims = hop["claims"]
        hop_id = hop.get("id", f"<index-{index}>")
        iss = claims.get("iss")
        cnf = claims.get("cnf")
        jwk = cnf.get("jwk") if isinstance(cnf, dict) else None
        try:
            current_uri = jwk_thumbprint_uri(jwk)
        except UnsupportedKey as error:
            findings.append({"code": "unsupported-or-invalid-holder-key", "hop_id": hop_id,
                             "reason": str(error)})
            current_uri = None

        if not isinstance(iss, str) or not iss:
            findings.append({"code": "issuer-missing", "hop_id": hop_id})
        if index == 0:
            if isinstance(iss, str) and iss.startswith(THUMBPRINT_URI_PREFIX):
                findings.append({"code": "root-issuer-must-be-trust-anchor-uri", "hop_id": hop_id})
            links.append({"hop_id": hop_id, "role": "root",
                          "issuer": iss if isinstance(iss, str) else None,
                          "holder_thumbprint_uri": current_uri})
            continue

        parent = hops[index - 1]
        parent_claims = parent.get("claims") if isinstance(parent, dict) else None
        parent_cnf = parent_claims.get("cnf") if isinstance(parent_claims, dict) else None
        parent_jwk = parent_cnf.get("jwk") if isinstance(parent_cnf, dict) else None
        try:
            expected_issuer = jwk_thumbprint_uri(parent_jwk)
        except UnsupportedKey as error:
            findings.append({"code": "parent-holder-key-unverifiable", "hop_id": hop_id,
                             "parent_id": parent.get("id") if isinstance(parent, dict) else None,
                             "reason": str(error)})
            expected_issuer = None

        if expected_issuer is not None and iss != expected_issuer:
            findings.append({"code": "issuer-thumbprint-mismatch", "hop_id": hop_id,
                             "parent_id": parent.get("id") if isinstance(parent, dict) else None,
                             "expected_issuer": expected_issuer,
                             "observed_issuer": iss})
        links.append({"hop_id": hop_id, "role": "derived",
                      "issuer": iss if isinstance(iss, str) else None,
                      "expected_parent_key_issuer": expected_issuer,
                      "holder_thumbprint_uri": current_uri})

    status = "KEY_LINKAGE_CLAIMS_PASS" if not findings else "INVALID_CHAIN"
    return {
        "schema": RESULT_SCHEMA, "status": status,
        "hop_count": len(hops), "links": links, "findings": findings,
        "profile": "OKP/Ed25519 only; RFC 7638 thumbprint required members and RFC 9278 URI",
        "qualification": "NOT_CLAIMED",
        "scope": (
            "parsed-claim holder-key linkage only; no signature verification, trust anchor "
            "validation, par_hash verification, JWT parsing, or proof-of-possession"
        ),
    }
