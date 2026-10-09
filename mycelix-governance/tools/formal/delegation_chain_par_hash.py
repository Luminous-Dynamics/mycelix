#!/usr/bin/env python3
"""Narrow parent JWS signing-input hash-link checker for parsed AAT fixtures.

This verifies par_hash = BASE64URL(SHA-256(parent JWS Signing Input)) over the
exact ASCII signing-input string supplied by the caller. It does not parse or
authenticate compact JWS/JWT, verify signatures, or verify issuer-key linkage.
"""
from __future__ import annotations

import base64
import hashlib
import re
from typing import Any

PAR_HASH_SCHEMA = "mycelix.delegation-chain-par-hash.v1"
RESULT_SCHEMA = "mycelix.delegation-chain-par-hash-result.v1"
_B64URL_SEGMENT = re.compile(r"^[A-Za-z0-9_-]+$")
MAX_TOKEN_COUNT = 9
MAX_SIGNING_INPUT_BYTES = 64 * 1024
MAX_CHAIN_SIGNING_INPUT_BYTES = 256 * 1024


def _is_canonical_segment(value: Any) -> bool:
    if not isinstance(value, str) or not value or not _B64URL_SEGMENT.fullmatch(value):
        return False
    try:
        decoded = base64.urlsafe_b64decode(value + "=" * ((4 - len(value) % 4) % 4))
    except (ValueError, base64.binascii.Error):
        return False
    return base64.urlsafe_b64encode(decoded).rstrip(b"=").decode("ascii") == value


def _is_canonical_sha256(value: Any) -> bool:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]{43}", value):
        return False
    try:
        decoded = base64.urlsafe_b64decode(value + "=")
    except (ValueError, base64.binascii.Error):
        return False
    return len(decoded) == 32 and base64.urlsafe_b64encode(decoded).rstrip(b"=").decode("ascii") == value


def signing_input_bytes(value: Any) -> bytes:
    if not isinstance(value, str):
        raise ValueError("JWS signing input must be an ASCII string")
    try:
        encoded = value.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("JWS signing input must be ASCII") from error
    if len(encoded) > MAX_SIGNING_INPUT_BYTES:
        raise ValueError(f"JWS signing input exceeds {MAX_SIGNING_INPUT_BYTES} bytes")
    parts = value.split(".")
    if len(parts) != 2 or not all(_is_canonical_segment(part) for part in parts):
        raise ValueError("JWS signing input must be canonical BASE64URL(header).BASE64URL(payload)")
    return encoded


def parent_signing_input_hash(signing_input: str) -> str:
    digest = hashlib.sha256(signing_input_bytes(signing_input)).digest()
    return base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")


def evaluate_par_hash_chain(raw: dict[str, Any]) -> dict[str, Any]:
    if raw.get("schema") != PAR_HASH_SCHEMA:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "unsupported-schema"}], "qualification": "NOT_CLAIMED"}
    hops = raw.get("hops")
    if not isinstance(hops, list) or not hops:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "malformed-chain"}], "qualification": "NOT_CLAIMED"}
    if len(hops) > MAX_TOKEN_COUNT:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "hop_count": len(hops),
                "findings": [{"code": "implementation-chain-depth-exceeded",
                              "max_token_count": MAX_TOKEN_COUNT}],
                "qualification": "NOT_CLAIMED"}

    total_input_bytes = sum(
        len(hop["signing_input"].encode("utf-8"))
        for hop in hops
        if isinstance(hop, dict) and isinstance(hop.get("signing_input"), str)
    )
    if total_input_bytes > MAX_CHAIN_SIGNING_INPUT_BYTES:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "hop_count": len(hops), "total_signing_input_bytes": total_input_bytes,
                "findings": [{"code": "chain-signing-input-size-exceeded",
                              "limit": MAX_CHAIN_SIGNING_INPUT_BYTES}],
                "qualification": "NOT_CLAIMED"}

    findings: list[dict[str, Any]] = []
    links: list[dict[str, Any]] = []
    ids: list[str] = []
    for index, hop in enumerate(hops):
        if not isinstance(hop, dict) or not isinstance(hop.get("claims"), dict):
            findings.append({"code": "hop-claims-malformed", "hop_index": index})
            continue
        hop_id = hop.get("id")
        claims = hop["claims"]
        if not isinstance(hop_id, str) or not hop_id:
            findings.append({"code": "hop-id-missing", "hop_index": index})
            hop_id = f"<index-{index}>"
        ids.append(hop_id)
        try:
            own_signing_input = hop.get("signing_input")
            # Validate syntax/canonical segments even though no signature is verified.
            signing_input_bytes(own_signing_input)
        except ValueError as error:
            findings.append({"code": "signing-input-invalid", "hop_id": hop_id,
                             "reason": str(error)})

        if index == 0:
            if "par_hash" in claims:
                findings.append({"code": "root-par-hash-must-be-absent", "hop_id": hop_id})
            links.append({"hop_id": hop_id, "role": "root",
                          "parent_signing_input_sha256_b64url": None,
                          "observed_par_hash": claims.get("par_hash")})
            continue

        parent = hops[index - 1]
        parent_id = parent.get("id") if isinstance(parent, dict) else None
        parent_input = parent.get("signing_input") if isinstance(parent, dict) else None
        observed = claims.get("par_hash")
        try:
            expected = parent_signing_input_hash(parent_input)
        except ValueError as error:
            findings.append({"code": "parent-signing-input-unverifiable",
                             "hop_id": hop_id, "parent_id": parent_id,
                             "reason": str(error)})
            expected = None
        if not isinstance(observed, str) or not observed:
            findings.append({"code": "par-hash-missing", "hop_id": hop_id,
                             "parent_id": parent_id})
        elif not _is_canonical_sha256(observed):
            findings.append({"code": "par-hash-malformed", "hop_id": hop_id,
                             "parent_id": parent_id})
        elif expected is not None and observed != expected:
            findings.append({"code": "par-hash-mismatch", "hop_id": hop_id,
                             "parent_id": parent_id, "expected": expected,
                             "observed": observed})
        links.append({"hop_id": hop_id, "role": "derived",
                      "parent_id": parent_id, "parent_signing_input_sha256_b64url": expected,
                      "observed_par_hash": observed})

    if len(ids) != len(set(ids)):
        findings.append({"code": "duplicate-hop-id"})
    status = "PARENT_SIGNING_INPUT_LINKAGE_PASS" if not findings else "INVALID_CHAIN"
    return {
        "schema": RESULT_SCHEMA, "status": status, "hop_count": len(hops),
        "links": links, "findings": findings,
        "qualification": "NOT_CLAIMED",
        "scope": (
            "provided JWS signing-input string/hash linkage only; compact JWS parsing, "
            "signature verification, root trust anchors, and proof-of-possession are not implemented"
        ),
    }
