#!/usr/bin/env python3
"""Bounded Ed25519 compact-JWS AAT chain verification candidate.

Unlike the neighboring parsed-fixture checkers, this module verifies actual
three-segment compact JWS token signatures using OpenSSL's Ed25519 primitive.
It checks root trust-anchor signature, child signatures under parent cnf.jwk,
issuer thumbprint linkage, par_hash over the exact parent signing input,
depth/TTL/jti/size invariants, and AAT entry cardinality.

It deliberately does not evaluate the full RFC 9396 authorization_details
constraint lattice, invocation arguments, or leaf proof-of-possession. It is
a research candidate and is not qualified for production enforcement.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
import subprocess
import tempfile
from json.decoder import scanstring
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import aat_capability_subsumption as capability  # noqa: E402

SCHEMA = "mycelix.compact-jws-aat-chain.v1"
RESULT_SCHEMA = "mycelix.compact-jws-aat-chain-result.v1"

MAX_DELEGATION_DEPTH = 8
MAX_TOKEN_COUNT = MAX_DELEGATION_DEPTH + 1
MAX_TOKEN_SIZE_BYTES = 64 * 1024
MAX_STACK_SIZE_BYTES = 256 * 1024
MAX_TOKEN_LIFETIME_SECONDS = 90 * 24 * 60 * 60
MAX_IAT_SKEW_SECONDS = 30
MAX_TRUST_ANCHORS = 16
THUMBPRINT_URI_PREFIX = "urn:ietf:params:oauth:jwk-thumbprint:sha-256:"
_B64URL = re.compile(r"^[A-Za-z0-9_-]+$")
_B64URL_43 = re.compile(r"^[A-Za-z0-9_-]{43}$")
_ED25519_SPKI_PREFIX = bytes.fromhex("302a300506032b6570032100")
_PRIVATE_JWK_FIELDS = {"d", "p", "q", "dp", "dq", "qi", "oth", "k"}


class VerificationError(ValueError):
    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def b64url_decode_canonical(value: Any, code: str = "base64url-invalid") -> bytes:
    if not isinstance(value, str) or not value or not _B64URL.fullmatch(value):
        raise VerificationError(code, "expected non-empty unpadded base64url")
    try:
        decoded = base64.urlsafe_b64decode(value + "=" * ((4 - len(value) % 4) % 4))
    except (ValueError, base64.binascii.Error) as error:
        raise VerificationError(code, "invalid base64url encoding") from error
    if base64.urlsafe_b64encode(decoded).rstrip(b"=").decode("ascii") != value:
        raise VerificationError(code, "noncanonical base64url encoding")
    return decoded


def b64url_encode(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def json_no_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise VerificationError("json-duplicate-member", f"duplicate JSON member {key!r}")
        result[key] = value
    return result


def parse_json_object(raw: bytes, code: str) -> dict[str, Any]:
    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=json_no_duplicate_keys,
                           parse_constant=lambda value: (_ for _ in ()).throw(
                               VerificationError("json-invalid-constant", f"invalid JSON constant {value}")))
    except VerificationError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError, ValueError) as error:
        raise VerificationError(code, "invalid or excessively nested UTF-8 JSON object") from error

    if not isinstance(value, dict):
        raise VerificationError(code, "JWT header/payload must be a JSON object")
    return value


def validate_jwk_signature_usage(jwk: Any) -> None:
    """Reject JWK metadata that conflicts with this Ed25519 signature profile."""
    if not isinstance(jwk, dict):
        raise VerificationError("public-key-invalid", "public JWK must be an object")
    if "use" in jwk and jwk["use"] != "sig":
        raise VerificationError("jwk-use-invalid", "JWK use, when present, must be sig")
    if "alg" in jwk and jwk["alg"] != "EdDSA":
        raise VerificationError("jwk-algorithm-mismatch",
                                "JWK alg, when present, must match the EdDSA token profile")
    if "key_ops" in jwk:
        operations = jwk["key_ops"]
        if (not isinstance(operations, list)
                or any(not isinstance(operation, str) or not operation for operation in operations)
                or len(operations) != len(set(operations))
                or set(operations) != {"verify"}):
            raise VerificationError(
                "jwk-key-ops-invalid",
                "Ed25519 public verification JWK key_ops, when present, must be exactly ['verify']",
            )


def public_ed25519_bytes(jwk: Any) -> bytes:
    if not isinstance(jwk, dict):
        raise VerificationError("public-key-invalid", "public JWK must be an object")
    if jwk.get("kty") != "OKP" or jwk.get("crv") != "Ed25519":
        raise VerificationError("algorithm-key-mismatch", "only public OKP/Ed25519 keys are supported")
    if _PRIVATE_JWK_FIELDS.intersection(jwk):
        raise VerificationError("private-key-material-present", "public JWK contains private key members")
    validate_jwk_signature_usage(jwk)
    raw = b64url_decode_canonical(jwk.get("x"), "jwk-coordinate-invalid")
    if len(raw) != 32:
        raise VerificationError("jwk-coordinate-invalid", "Ed25519 x must encode exactly 32 bytes")
    return raw


def jwk_thumbprint_uri(jwk: dict[str, Any]) -> str:
    public_ed25519_bytes(jwk)
    required_members = {"crv": "Ed25519", "kty": "OKP", "x": jwk["x"]}
    canonical = json.dumps(required_members, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False).encode("utf-8")
    return THUMBPRINT_URI_PREFIX + b64url_encode(hashlib.sha256(canonical).digest())


_JSON_NUMBER = re.compile(r"-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?")


def _skip_json_space(text: str, index: int) -> int:
    while index < len(text) and text[index] in " \t\r\n":
        index += 1
    return index


def _scan_json_string(text: str, index: int) -> tuple[str, int]:
    if index >= len(text) or text[index] != '"':
        raise VerificationError("jti-preparse-invalid", "expected JSON string while scanning jti")
    try:
        value, end = scanstring(text, index + 1, strict=True)
    except (ValueError, UnicodeError) as error:
        raise VerificationError("jti-preparse-invalid", "invalid JSON string while scanning jti") from error
    return value, end


def _skip_json_value(text: str, index: int, depth: int = 0) -> int:
    """Validate and skip one JSON value without materializing arbitrary nested values."""
    index = _skip_json_space(text, index)
    if depth > 64:
        raise VerificationError("jti-preparse-depth-exceeded",
                                "pre-signature jti scan exceeds the profile's JSON nesting bound")
    if index >= len(text):
        raise VerificationError("jti-preparse-invalid", "missing JSON value")
    char = text[index]
    if char == '"':
        _, end = _scan_json_string(text, index)
        return end
    if char == "{":
        index = _skip_json_space(text, index + 1)
        if index < len(text) and text[index] == "}":
            return index + 1
        while True:
            _, index = _scan_json_string(text, index)
            index = _skip_json_space(text, index)
            if index >= len(text) or text[index] != ":":
                raise VerificationError("jti-preparse-invalid", "object key lacks a colon")
            index = _skip_json_value(text, index + 1, depth + 1)
            index = _skip_json_space(text, index)
            if index >= len(text):
                raise VerificationError("jti-preparse-invalid", "unterminated JSON object")
            if text[index] == "}":
                return index + 1
            if text[index] != ",":
                raise VerificationError("jti-preparse-invalid", "expected comma in JSON object")
            index = _skip_json_space(text, index + 1)
    if char == "[":
        index = _skip_json_space(text, index + 1)
        if index < len(text) and text[index] == "]":
            return index + 1
        while True:
            index = _skip_json_value(text, index, depth + 1)
            index = _skip_json_space(text, index)
            if index >= len(text):
                raise VerificationError("jti-preparse-invalid", "unterminated JSON array")
            if text[index] == "]":
                return index + 1
            if text[index] != ",":
                raise VerificationError("jti-preparse-invalid", "expected comma in JSON array")
            index = _skip_json_space(text, index + 1)
    for literal in ("true", "false", "null"):
        if text.startswith(literal, index):
            end = index + len(literal)
            if end == len(text) or text[end] in " \t\r\n,]}":
                return end
            raise VerificationError("jti-preparse-invalid", "invalid JSON literal")
    number_match = _JSON_NUMBER.match(text, index)
    if number_match is not None:
        end = number_match.end()
        if end == len(text) or text[end] in " \t\r\n,]}":
            return end
    raise VerificationError("jti-preparse-invalid", "invalid JSON value during pre-signature scan")


def extract_untrusted_jti(payload_bytes: bytes, hop_index: int) -> str:
    """Extract only top-level jti before signature verification, for cycle rejection.

    Returned values remain untrusted. The caller may use this result only to
    reject duplicate token identifiers; authorization claims are parsed after
    the corresponding token signature has verified.
    """
    try:
        text = payload_bytes.decode("utf-8", errors="strict")
    except UnicodeDecodeError as error:
        raise VerificationError("jti-preparse-invalid",
                                f"token[{hop_index}] payload is not UTF-8 JSON") from error
    index = _skip_json_space(text, 0)
    if index >= len(text) or text[index] != "{":
        raise VerificationError("jti-preparse-invalid",
                                f"token[{hop_index}] payload must be a JSON object")
    index = _skip_json_space(text, index + 1)
    found = False
    jti: str | None = None
    if index < len(text) and text[index] == "}":
        raise VerificationError("jti-preparse-invalid",
                                f"token[{hop_index}] payload is missing a non-empty jti")
    while True:
        key, index = _scan_json_string(text, index)
        index = _skip_json_space(text, index)
        if index >= len(text) or text[index] != ":":
            raise VerificationError("jti-preparse-invalid",
                                    f"token[{hop_index}] has an invalid JSON object member")
        index = _skip_json_space(text, index + 1)
        if key == "jti":
            if found:
                raise VerificationError("jti-preparse-duplicate",
                                        f"token[{hop_index}] has duplicate top-level jti members")
            found = True
            value, index = _scan_json_string(text, index)
            if not value:
                raise VerificationError("jti-preparse-invalid",
                                        f"token[{hop_index}] jti must be a non-empty string")
            if any(0xD800 <= ord(char) <= 0xDFFF for char in value):
                raise VerificationError("jti-preparse-invalid",
                                        f"token[{hop_index}] jti contains a surrogate code point")
            jti = value
        else:
            index = _skip_json_value(text, index, depth=1)
        index = _skip_json_space(text, index)
        if index >= len(text):
            raise VerificationError("jti-preparse-invalid",
                                    f"token[{hop_index}] payload object is unterminated")
        if text[index] == "}":
            index = _skip_json_space(text, index + 1)
            if index != len(text):
                raise VerificationError("jti-preparse-invalid",
                                        f"token[{hop_index}] payload has trailing JSON data")
            break
        if text[index] != ",":
            raise VerificationError("jti-preparse-invalid",
                                    f"token[{hop_index}] expected comma between JSON members")
        index = _skip_json_space(text, index + 1)
    if not found or not isinstance(jti, str) or not jti:
        raise VerificationError("jti-preparse-invalid",
                                f"token[{hop_index}] payload is missing a non-empty string jti")
    return jti


def split_compact_jws(token: Any) -> tuple[str, str, str, bytes, bytes, bytes]:
    if not isinstance(token, str) or not token:
        raise VerificationError("compact-token-malformed", "token must be a non-empty string")
    try:
        token_bytes = token.encode("ascii")
    except UnicodeEncodeError as error:
        raise VerificationError("compact-token-malformed", "compact JWS must be ASCII") from error
    if len(token_bytes) > MAX_TOKEN_SIZE_BYTES:
        raise VerificationError("token-size-exceeded", f"token exceeds {MAX_TOKEN_SIZE_BYTES} bytes")
    parts = token.split(".")
    if len(parts) != 3 or any(not part for part in parts):
        raise VerificationError("compact-token-malformed", "compact JWS requires exactly three non-empty segments")
    header_bytes = b64url_decode_canonical(parts[0], "protected-header-invalid")
    payload_bytes = b64url_decode_canonical(parts[1], "payload-segment-invalid")
    signature_bytes = b64url_decode_canonical(parts[2], "signature-segment-invalid")
    if len(signature_bytes) != 64:
        raise VerificationError("signature-length-invalid", "Ed25519 JWS signature must be 64 bytes")
    return parts[0], parts[1], parts[2], header_bytes, payload_bytes, signature_bytes


def parse_and_validate_header(header_bytes: bytes) -> dict[str, Any]:
    header = parse_json_object(header_bytes, "protected-header-invalid")
    if header.get("alg") != "EdDSA":
        raise VerificationError("algorithm-not-allowed", "only EdDSA/Ed25519 is allowed")
    token_type = header.get("typ")
    if not isinstance(token_type, str) or token_type.lower() not in {"aat+jwt", "application/aat+jwt"}:
        raise VerificationError("token-type-invalid", "JWT type header is required by this profile")
    if "b64" in header:
        raise VerificationError("b64-header-not-allowed",
                                "this profile requires the default JWS payload encoding; b64 must be absent")
    if "crit" in header:
        crit = header.get("crit")
        if not isinstance(crit, list) or not crit:
            raise VerificationError("critical-header-unsupported", "critical JWS headers must be a non-empty array")
        raise VerificationError("critical-header-unsupported",
                                "this profile does not implement any critical JWS header extensions")
    return header


def verify_ed25519_signature(signing_input: bytes, signature: bytes, jwk: dict[str, Any],
                             openssl_binary: str = "openssl") -> None:
    public_raw = public_ed25519_bytes(jwk)
    der = _ED25519_SPKI_PREFIX + public_raw
    with tempfile.TemporaryDirectory(prefix="mycelix-ed25519-verify-") as temporary:
        root = Path(temporary)
        pub_path = root / "public.der"
        input_path = root / "signing-input.bin"
        sig_path = root / "signature.bin"
        pub_path.write_bytes(der)
        input_path.write_bytes(signing_input)
        sig_path.write_bytes(signature)
        try:
            completed = subprocess.run(
                [openssl_binary, "pkeyutl", "-verify", "-pubin", "-inkey", str(pub_path),
                 "-keyform", "DER", "-rawin", "-in", str(input_path), "-sigfile", str(sig_path)],
                check=False, capture_output=True, timeout=3,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise VerificationError("signature-verifier-unavailable", "OpenSSL Ed25519 verifier unavailable or timed out") from error
        if completed.returncode != 0:
            raise VerificationError("signature-invalid", "Ed25519 JWS signature verification failed")


def _safe_uri(value: Any) -> bool:
    if not isinstance(value, str) or not value or any(ord(ch) < 0x20 for ch in value):
        return False
    try:
        parsed = urlsplit(value)
    except ValueError:
        return False
    return bool(parsed.scheme) and not any(ch.isspace() for ch in value)


def _int_claim(claims: dict[str, Any], name: str, hop_index: int) -> int:
    value = claims.get(name)
    if type(value) is not int:
        raise VerificationError("claim-not-integer", f"token[{hop_index}] {name} must be an integer")
    return value


def _validate_common_claims(claims: dict[str, Any], hop_index: int, now: int) -> tuple[int, int, int, int, str, dict[str, Any]]:
    jti = claims.get("jti")
    if not isinstance(jti, str) or not jti:
        raise VerificationError("jti-invalid", f"token[{hop_index}] jti must be a non-empty string")
    iat = _int_claim(claims, "iat", hop_index)
    exp = _int_claim(claims, "exp", hop_index)
    depth = _int_claim(claims, "del_depth", hop_index)
    max_depth = _int_claim(claims, "del_max_depth", hop_index)
    if depth < 0 or max_depth < 0 or depth > max_depth:
        raise VerificationError("depth-range-invalid", f"token[{hop_index}] delegation depth range is invalid")
    if max_depth > MAX_DELEGATION_DEPTH:
        raise VerificationError("maximum-depth-exceeded", f"token[{hop_index}] exceeds configured delegation depth")
    if exp <= now:
        raise VerificationError("token-expired", f"token[{hop_index}] is expired")
    if exp <= iat:
        raise VerificationError("expiry-not-after-issue", f"token[{hop_index}] exp must be after iat")
    if iat > now + MAX_IAT_SKEW_SECONDS:
        raise VerificationError("issued-at-beyond-skew", f"token[{hop_index}] iat exceeds allowed future skew")
    if exp > iat + MAX_TOKEN_LIFETIME_SECONDS:
        raise VerificationError("token-lifetime-exceeded", f"token[{hop_index}] exceeds lifetime limit")
    cnf = claims.get("cnf")
    jwk = cnf.get("jwk") if isinstance(cnf, dict) else None
    public_ed25519_bytes(jwk)
    auth_details = claims.get("authorization_details")
    if not isinstance(auth_details, list):
        raise VerificationError("authorization-details-invalid", f"token[{hop_index}] authorization_details must be an array")
    return iat, exp, depth, max_depth, jti, jwk


def verify_issuer_thumbprint(issuer: Any, parent_jwk: dict[str, Any]) -> None:
    expected = jwk_thumbprint_uri(parent_jwk)
    if not isinstance(issuer, str) or issuer != expected:
        raise VerificationError("issuer-thumbprint-mismatch",
                                "derived iss does not identify the parent's holder key")


def verify_parent_par_hash(claims: dict[str, Any], parent_token: str) -> None:
    expected = b64url_encode(hashlib.sha256(_token_signing_input(parent_token)).digest())
    if claims.get("par_hash") != expected:
        raise VerificationError("par-hash-mismatch",
                                "par_hash does not bind the exact parent JWS signing input")


def _aat_entries(auth_details: list[Any]) -> list[dict[str, Any]]:
    return [item for item in auth_details if isinstance(item, dict)
            and item.get("type") == "attenuating_agent_token"]


def _parse_authenticated_token(token: str, hop_index: int) -> dict[str, Any]:
    _, _, _, _, payload_bytes, _ = split_compact_jws(token)
    return parse_json_object(payload_bytes, f"payload-json-invalid-token-{hop_index}")


def _token_signing_input(token: str) -> bytes:
    parts = token.split(".")
    return (parts[0] + "." + parts[1]).encode("ascii")


def evaluate_compact_chain(raw: dict[str, Any], trusted_anchors: list[dict[str, Any]],
                           openssl_binary: str = "openssl", *,
                           trusted_now: int | None = None) -> dict[str, Any]:
    """Verify a chain using a separately supplied trusted enforcement clock.

    The raw input's "now" value is retained only for fixture/schema compatibility;
    it is never used as authority for expiration or issued-at checks.
    """
    if not isinstance(raw, dict):
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "top-level-input-not-object"}], "qualification": "NOT_CLAIMED"}
    if raw.get("schema") != SCHEMA:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "unsupported-schema"}], "qualification": "NOT_CLAIMED"}
    if set(raw) != {"schema", "now", "chain"}:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "unexpected-chain-input-field"}], "qualification": "NOT_CLAIMED"}
    if type(trusted_now) is not int or not 0 <= trusted_now <= (1 << 53) - 1:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "trusted-clock-invalid",
                              "detail": "trusted_now must be a non-negative safe integer Unix timestamp"}],
                "qualification": "NOT_CLAIMED"}
    now = trusted_now
    chain = raw.get("chain")
    anchors = trusted_anchors
    if not isinstance(chain, list) or not chain or not isinstance(anchors, list) or not anchors:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "malformed-top-level-input"}], "qualification": "NOT_CLAIMED"}
    if len(chain) > MAX_TOKEN_COUNT:
        return {"schema": RESULT_SCHEMA, "status": "INVALID_CHAIN",
                "findings": [{"code": "chain-depth-exceeded", "limit": MAX_DELEGATION_DEPTH}],
                "qualification": "NOT_CLAIMED"}
    if len(anchors) > MAX_TRUST_ANCHORS:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "trust-anchor-count-exceeded", "limit": MAX_TRUST_ANCHORS}],
                "qualification": "NOT_CLAIMED"}
    if not all(isinstance(anchor, dict) and _safe_uri(anchor.get("issuer_uri"))
               and isinstance(anchor.get("jwk"), dict) for anchor in anchors):
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "trust-anchor-malformed"}], "qualification": "NOT_CLAIMED"}
    if len(set(anchor["issuer_uri"] for anchor in anchors)) != len(anchors):
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "findings": [{"code": "duplicate-trust-anchor-issuer"}], "qualification": "NOT_CLAIMED"}
    try:
        total_bytes = sum(len(token.encode("ascii")) if isinstance(token, str) else MAX_TOKEN_SIZE_BYTES + 1
                          for token in chain)
    except UnicodeEncodeError:
        total_bytes = MAX_STACK_SIZE_BYTES + 1
    if total_bytes > MAX_STACK_SIZE_BYTES:
        return {"schema": RESULT_SCHEMA, "status": "INVALID_CHAIN",
                "findings": [{"code": "stack-size-exceeded", "limit": MAX_STACK_SIZE_BYTES}],
                "qualification": "NOT_CLAIMED"}

    # Only protected headers are parsed before each token's signature; payload
    # claims are deserialized only after a successful signature check.
    parsed_wires: list[dict[str, Any]] = []
    try:
        for index, token in enumerate(chain):
            h64, p64, s64, header_bytes, payload_bytes, signature = split_compact_jws(token)
            header = parse_and_validate_header(header_bytes)
            preverified_jti = extract_untrusted_jti(payload_bytes, index)
            parsed_wires.append({
                "token": token, "header": header,
                "signing_input": (h64 + "." + p64).encode("ascii"),
                "signature": signature, "index": index,
                "preverified_jti": preverified_jti,
            })

        # AAT chain verification permits only this one claim to be extracted
        # before signatures; use it solely to reject token-identifier cycles.
        # Every value remains untrusted until its token's signature verifies.
        preverified_jtis = [wire["preverified_jti"] for wire in parsed_wires]
        if len(preverified_jtis) != len(set(preverified_jtis)):
            raise VerificationError("duplicate-jti", "presented chain reuses an untrusted token jti")

        root_wire = parsed_wires[0]
        root_valid_anchors: list[dict[str, Any]] = []
        for anchor in anchors:
            try:
                verify_ed25519_signature(root_wire["signing_input"], root_wire["signature"],
                                         anchor["jwk"], openssl_binary=openssl_binary)
                root_valid_anchors.append(anchor)
            except VerificationError as error:
                if error.code not in {"signature-invalid"}:
                    raise
        if len(root_valid_anchors) != 1:
            raise VerificationError("root-trust-anchor-signature-invalid",
                                    "root signature must verify under exactly one configured trust anchor")
        root_claims = _parse_authenticated_token(root_wire["token"], 0)
        root_anchor = root_valid_anchors[0]
        if root_claims.get("iss") != root_anchor["issuer_uri"]:
            raise VerificationError("root-issuer-trust-anchor-mismatch",
                                    "root iss does not match the issuer URI assigned to its trust anchor")
        if not _safe_uri(root_claims.get("iss")):
            raise VerificationError("root-issuer-invalid", "root iss must be a URI")
        if root_claims.get("del_depth") != 0:
            raise VerificationError("root-depth-not-zero", "root del_depth must be zero")
        if "par_hash" in root_claims:
            raise VerificationError("root-par-hash-present", "root par_hash must be absent")
        root_iat, root_exp, root_depth, root_max, root_jti, root_holder_jwk = _validate_common_claims(
            root_claims, 0, now
        )
        if root_jti != root_wire["preverified_jti"]:
            raise VerificationError("jti-preparse-mismatch",
                                    "authenticated root jti differs from pre-signature extraction")
        try:
            root_tools = capability.validate_authorization_details(
                root_claims["authorization_details"], require_one=True, label="root"
            )
        except capability.CapabilityError as error:
            code = "root-aat-entry-count-invalid" if error.code == "aat-entry-count-invalid" else error.code
            raise VerificationError(code, error.detail) from error
        claims_by_index = [root_claims]
        jtis = {root_jti}
        previous_iat, previous_exp, previous_depth, previous_max = root_iat, root_exp, root_depth, root_max
        previous_holder_jwk = root_holder_jwk
        previous_tools = root_tools
        previous_token = root_wire["token"]

        for index in range(1, len(parsed_wires)):
            wire = parsed_wires[index]
            verify_ed25519_signature(wire["signing_input"], wire["signature"],
                                     previous_holder_jwk, openssl_binary=openssl_binary)
            claims = _parse_authenticated_token(wire["token"], index)
            iat, exp, depth, max_depth, jti, holder_jwk = _validate_common_claims(claims, index, now)
            if jti != wire["preverified_jti"]:
                raise VerificationError("jti-preparse-mismatch",
                                        f"token[{index}] authenticated jti differs from pre-signature extraction")
            if jti in jtis:
                raise VerificationError("duplicate-jti", f"token[{index}] reuses jti from an earlier chain token")
            jtis.add(jti)
            verify_issuer_thumbprint(claims.get("iss"), previous_holder_jwk)
            verify_parent_par_hash(claims, previous_token)
            if depth != previous_depth + 1:
                raise VerificationError("delegation-depth-not-incremented-by-one",
                                        f"token[{index}] del_depth must equal parent depth plus one")
            if depth > previous_max or depth > MAX_DELEGATION_DEPTH:
                raise VerificationError("parent-depth-budget-exceeded",
                                        f"token[{index}] exceeds the parent or implementation depth ceiling")
            if max_depth > previous_max:
                raise VerificationError("maximum-depth-budget-expanded",
                                        f"token[{index}] del_max_depth exceeds its parent's ceiling")
            if exp > previous_exp:
                raise VerificationError("child-expiry-exceeds-parent",
                                        f"token[{index}] expiry exceeds parent expiry")
            if iat < previous_iat:
                raise VerificationError("child-iat-before-parent",
                                        f"token[{index}] iat precedes parent iat")
            try:
                child_tools = capability.validate_authorization_details(
                    claims["authorization_details"], require_one=False, label=f"token[{index}]"
                )
                capability.check_capability_attenuation(previous_tools, child_tools)
            except capability.CapabilityError as error:
                code = "child-aat-entry-count-invalid" if error.code == "aat-entry-count-invalid" else error.code
                raise VerificationError(code, f"token[{index}]: {error.detail}") from error
            claims_by_index.append(claims)
            previous_iat, previous_exp, previous_depth, previous_max = iat, exp, depth, max_depth
            previous_holder_jwk = holder_jwk
            previous_tools = child_tools
            previous_token = wire["token"]

        if len(chain) != previous_depth + 1:
            raise VerificationError("chain-length-depth-mismatch",
                                    "chain token count must equal leaf del_depth plus one")
        leaf_entries = _aat_entries(claims_by_index[-1]["authorization_details"])
        if len(leaf_entries) != 1:
            raise VerificationError("leaf-aat-entry-count-invalid",
                                    "leaf authorization_details must contain exactly one AAT entry")

        return {
            "schema": RESULT_SCHEMA,
            "status": "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
            "token_count": len(chain),
            "root_issuer": root_anchor["issuer_uri"],
            "leaf_jti": claims_by_index[-1]["jti"],
            "verified_invariants": [
                "configured-root-trust-anchor-signature",
                "all-token-ed25519-jws-signatures",
                "issuer-thumbprint-linkage",
                "exact-parent-jws-signing-input-par_hash",
                "unique-jti",
                "depth-and-maximum-depth-monotonicity",
                "expiry-and-iat-monotonicity",
                "per-token-and-chain-size-bounds",
                "root-and-leaf-aat-entry-cardinality",
                "core-tool-and-argument-constraint-attenuation",
            ],
            "qualification": "NOT_CLAIMED",
            "scope": (
                "research candidate verifies compact Ed25519 JWS signatures, selected AAT chain "
                "invariants, and the bounded core capability/constraint subsumption relation; "
                "invocation argument checking, revocation/replay database, and leaf proof-of-possession "
                "are not implemented"
            ),
        }
    except VerificationError as error:
        return {
            "schema": RESULT_SCHEMA,
            "status": "INVALID_CHAIN",
            "findings": [{"code": error.code, "detail": error.detail}],
            "qualification": "NOT_CLAIMED",
        }
