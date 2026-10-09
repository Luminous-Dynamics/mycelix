#!/usr/bin/env python3
"""Invocation-bound Ed25519 proof-of-possession for a verified AAT chain.

This wrapper verifies the compact AAT chain first, then verifies a separate
compact PoP JWT under the authenticated leaf cnf.jwk. It checks leaf jti,
exact tool name, canonical hta/args equality, timestamp window, optional
deployment audience, and atomically consumes PoP jti in a SQLite replay store.

To avoid claiming full RFC 8785 support without a dependency, this candidate
accepts a deliberately narrow canonical JSON subset: string keys and values
are limited to the Unicode Basic Multilingual Plane, integers must be exactly
representable as IEEE-754 safe integers, floats are rejected, and standard
JSON booleans/null/arrays/objects are admitted. Unsupported values fail closed.
This is not a production authorization/dispatch adapter.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sqlite3
import stat
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aat_capability_subsumption as capability  # noqa: E402
import delegation_chain_compact_jws as chain_verifier  # noqa: E402


POP_RESULT_SCHEMA = "mycelix.aat-invocation-pop-result.v1"
POP_CLOCK_TOLERANCE_SECONDS = 30
MAX_POP_TOKEN_BYTES = 64 * 1024
MAX_POP_CLAIMS_BYTES = 64 * 1024
MAX_SAFE_INTEGER = (1 << 53) - 1
MAX_REPLAY_SCOPE_BYTES = 256
MAX_POP_JTI_BYTES = 256


class PopVerificationError(ValueError):
    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def _check_jcs_profile(value: Any, path: str = "$") -> None:
    """Validate the canonicalization subset before using Python JSON encoding."""
    if value is None or type(value) is bool:
        return
    if type(value) is int:
        if abs(value) > MAX_SAFE_INTEGER:
            raise PopVerificationError("jcs-number-outside-profile",
                                       f"{path}: integers must be safe IEEE-754 integers")
        return
    if type(value) is float:
        raise PopVerificationError("jcs-float-outside-profile",
                                   f"{path}: floating point values are outside this verifier profile")
    if isinstance(value, str):
        if any(ord(ch) > 0xFFFF or 0xD800 <= ord(ch) <= 0xDFFF for ch in value):
            raise PopVerificationError("jcs-unicode-outside-profile",
                                       f"{path}: non-BMP or surrogate Unicode is outside this verifier profile")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _check_jcs_profile(item, f"{path}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise PopVerificationError("jcs-object-key-invalid",
                                           f"{path}: JSON object keys must be strings")
            if any(ord(ch) > 0xFFFF or 0xD800 <= ord(ch) <= 0xDFFF for ch in key):
                raise PopVerificationError("jcs-unicode-outside-profile",
                                           f"{path}: non-BMP or surrogate object key")
            _check_jcs_profile(item, f"{path}.{key}")
        return
    raise PopVerificationError("jcs-value-outside-profile",
                               f"{path}: value type is outside the supported canonical JSON profile")


def canonical_json_profile(value: Any) -> bytes:
    """Canonical bytes for the explicitly documented restricted JSON subset."""
    _check_jcs_profile(value)
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=False, allow_nan=False).encode("utf-8")
    except (TypeError, ValueError, UnicodeEncodeError, RecursionError) as error:
        raise PopVerificationError("jcs-canonicalization-failed",
                                   "value cannot be canonicalized by the bounded JSON profile") from error


def _pop_header(header_bytes: bytes) -> dict[str, Any]:
    header = chain_verifier.parse_json_object(header_bytes, "pop-protected-header-invalid")
    if header.get("alg") != "EdDSA":
        raise PopVerificationError("pop-algorithm-not-allowed",
                                   "this PoP profile permits EdDSA/Ed25519 only")
    if "b64" in header:
        raise PopVerificationError("pop-b64-header-not-allowed",
                                   "this PoP profile requires the default encoded JWS payload")
    if "crit" in header:
        raise PopVerificationError("pop-critical-header-unsupported",
                                   "critical JWS headers are not implemented by this profile")
    token_type = header.get("typ")
    if token_type is not None and token_type not in {"pop+jwt", "application/pop+jwt"}:
        raise PopVerificationError("pop-token-type-invalid",
                                   "if typ is present it must be pop+jwt or application/pop+jwt")
    return header


def verify_pop_signature(signing_input: bytes, signature: bytes, leaf_jwk: dict[str, Any],
                         openssl_binary: str = "openssl") -> None:
    try:
        chain_verifier.verify_ed25519_signature(signing_input, signature, leaf_jwk,
                                                 openssl_binary=openssl_binary)
    except chain_verifier.VerificationError as error:
        raise PopVerificationError("pop-" + error.code, error.detail) from error


def verify_pop_payload_canonical(payload_bytes: bytes, claims: dict[str, Any]) -> None:
    if payload_bytes != canonical_json_profile(claims):
        raise PopVerificationError("pop-payload-not-canonical",
                                   "signed PoP payload is not canonical for the supported JSON profile")


def verify_pop_invocation_binding(claims: dict[str, Any], leaf_jti: str, tool: Any, args: Any,
                                  now: int, expected_audience: str | None) -> None:
    if claims["aat_id"] != leaf_jti:
        raise PopVerificationError("pop-leaf-id-mismatch",
                                   "PoP aat_id must exactly equal the verified leaf token jti")
    if claims["aat_tool"] != tool:
        raise PopVerificationError("pop-tool-mismatch",
                                   "PoP aat_tool must exactly equal this invocation's tool identifier")
    if canonical_json_profile(claims["hta"]) != canonical_json_profile(args):
        raise PopVerificationError("pop-arguments-mismatch",
                                   "PoP hta does not canonically equal the actual invocation arguments")
    if expected_audience is not None:
        if not isinstance(expected_audience, str) or not expected_audience:
            raise PopVerificationError("pop-audience-policy-invalid",
                                       "configured expected_audience must be a non-empty string")
        if claims.get("aat_aud") != expected_audience:
            raise PopVerificationError("pop-audience-mismatch",
                                       "PoP aat_aud must identify the configured enforcement audience")
    elif "aat_aud" in claims:
        raise PopVerificationError("pop-audience-policy-unconfigured",
                                   "PoP contains aat_aud but no expected audience is configured")
    if abs(claims["iat"] - now) > POP_CLOCK_TOLERANCE_SECONDS:
        raise PopVerificationError("pop-iat-outside-window",
                                   "PoP iat falls outside the configured ±30-second acceptance window")


def _decode_pop(pop_token: Any, leaf_jwk: dict[str, Any],
                openssl_binary: str) -> dict[str, Any]:
    if not isinstance(pop_token, str) or not pop_token:
        raise PopVerificationError("pop-token-malformed", "PoP token must be a non-empty compact JWS")
    try:
        encoded_token = pop_token.encode("ascii")
    except UnicodeEncodeError as error:
        raise PopVerificationError("pop-token-malformed", "compact PoP token must be ASCII") from error
    if len(encoded_token) > MAX_POP_TOKEN_BYTES:
        raise PopVerificationError("pop-token-size-exceeded", "compact PoP token exceeds the size limit")

    try:
        h64, p64, _s64, header_bytes, payload_bytes, signature = chain_verifier.split_compact_jws(pop_token)
        _pop_header(header_bytes)
        signing_input = (h64 + "." + p64).encode("ascii")
        verify_pop_signature(signing_input, signature, leaf_jwk, openssl_binary=openssl_binary)
        if len(payload_bytes) > MAX_POP_CLAIMS_BYTES:
            raise PopVerificationError("pop-claims-size-exceeded", "PoP claims exceed the size limit")
        claims = chain_verifier.parse_json_object(payload_bytes, "pop-claims-invalid")
    except PopVerificationError:
        raise
    except chain_verifier.VerificationError as error:
        raise PopVerificationError("pop-" + error.code, error.detail) from error

    verify_pop_payload_canonical(payload_bytes, claims)
    required = {"jti", "iat", "aat_id", "aat_tool", "hta"}
    allowed = required | {"aat_aud"}
    missing = sorted(required - set(claims))
    unexpected = sorted(set(claims) - allowed)
    if missing:
        raise PopVerificationError("pop-required-claim-missing", f"missing PoP claim(s): {missing}")
    if unexpected:
        raise PopVerificationError("pop-claim-unsupported", f"unsupported PoP claim(s): {unexpected}")
    if not isinstance(claims["jti"], str) or not claims["jti"]:
        raise PopVerificationError("pop-jti-invalid", "PoP jti must be a non-empty string")
    if type(claims["iat"]) is not int:
        raise PopVerificationError("pop-iat-invalid", "PoP iat must be an integer NumericDate in this profile")
    if not isinstance(claims["aat_id"], str) or not claims["aat_id"]:
        raise PopVerificationError("pop-aat-id-invalid", "aat_id must be a non-empty string")
    if not isinstance(claims["aat_tool"], str) or not claims["aat_tool"]:
        raise PopVerificationError("pop-aat-tool-invalid", "aat_tool must be a non-empty string")
    if not isinstance(claims["hta"], dict):
        raise PopVerificationError("pop-hta-invalid", "hta must be an object")
    if "aat_aud" in claims and (not isinstance(claims["aat_aud"], str) or not claims["aat_aud"]):
        raise PopVerificationError("pop-audience-invalid", "aat_aud, when present, must be a non-empty string")
    return claims


def _prepare_replay_database(database_path: Path) -> None:
    """Open/create the replay file without following symlinks; force mode 0600."""
    database_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        parent_info = database_path.parent.stat()
        if not stat.S_ISDIR(parent_info.st_mode):
            raise PopVerificationError("pop-replay-store-unavailable",
                                       "replay store parent is not a directory")
        if parent_info.st_mode & 0o022:
            raise PopVerificationError(
                "pop-replay-store-unavailable",
                "replay store parent must not be group/world writable",
            )

        nofollow = getattr(os, "O_NOFOLLOW", None)
        if nofollow is None:
            raise PopVerificationError("pop-replay-store-unavailable",
                                       "platform cannot guarantee no-follow replay-store opens")
        close_on_exec = getattr(os, "O_CLOEXEC", 0)
        try:
            descriptor = os.open(
                str(database_path),
                os.O_RDWR | os.O_CREAT | os.O_EXCL | nofollow | close_on_exec,
                0o600,
            )
        except FileExistsError:
            descriptor = os.open(
                str(database_path),
                os.O_RDWR | nofollow | close_on_exec,
            )
        try:
            file_info = os.fstat(descriptor)
            if not stat.S_ISREG(file_info.st_mode):
                raise PopVerificationError(
                    "pop-replay-store-unavailable",
                    "replay store must be a regular file, not a symlink or special file",
                )
            os.fchmod(descriptor, 0o600)
            secured_info = os.fstat(descriptor)
            if stat.S_IMODE(secured_info.st_mode) != 0o600:
                raise PopVerificationError(
                    "pop-replay-store-unavailable",
                    "replay store permissions could not be secured to owner-only",
                )
        finally:
            os.close(descriptor)
    except PopVerificationError:
        raise
    except OSError as error:
        raise PopVerificationError(
            "pop-replay-store-unavailable",
            "replay store path could not be secured; invocation must fail closed",
        ) from error


def _consume_pop_jti(database_path: Path, scope: str, jti: str, iat: int,
                     now: int, tolerance: int) -> None:
    """Consume a proof jti once within a shared SQLite transaction."""
    if not isinstance(scope, str) or not scope:
        raise PopVerificationError("pop-replay-scope-invalid",
                                   "replay_scope must identify the enforcement-point replay domain")
    try:
        if len(scope.encode("utf-8")) > MAX_REPLAY_SCOPE_BYTES:
            raise PopVerificationError("pop-replay-scope-invalid",
                                       "replay_scope exceeds the configured byte limit")
    except UnicodeEncodeError as error:
        raise PopVerificationError("pop-replay-scope-invalid",
                                   "replay_scope must be valid Unicode") from error
    if not isinstance(jti, str) or not jti or len(jti.encode("utf-8")) > MAX_POP_JTI_BYTES:
        raise PopVerificationError("pop-jti-invalid",
                                   "PoP jti must be non-empty and within the configured byte limit")
    if not isinstance(database_path, Path):
        raise PopVerificationError("pop-replay-store-unavailable",
                                   "replay_database must be an explicitly configured pathlib.Path")
    _prepare_replay_database(database_path)
    try:
        connection = sqlite3.connect(str(database_path), timeout=5.0, isolation_level=None)
        try:
            connection.execute("PRAGMA busy_timeout=5000")
            connection.execute(
                "CREATE TABLE IF NOT EXISTS aat_pop_replay ("
                "scope TEXT NOT NULL, jti_sha256 TEXT NOT NULL, iat INTEGER NOT NULL, "
                "PRIMARY KEY(scope, jti_sha256))"
            )
            connection.execute("BEGIN IMMEDIATE")
            # Keep consumed JTIs durably: deleting old entries would allow the
            # same identifier to be reused later, contrary to the one-time proof
            # profile. Retention must be managed without making old JTIs reusable.
            jti_digest = hashlib.sha256(jti.encode("utf-8")).hexdigest()
            connection.execute(
                "INSERT INTO aat_pop_replay(scope, jti_sha256, iat) VALUES (?, ?, ?)",
                (scope, jti_digest, iat),
            )
            connection.execute("COMMIT")
        except sqlite3.IntegrityError as error:
            try:
                connection.execute("ROLLBACK")
            except sqlite3.Error:
                pass
            raise PopVerificationError("pop-jti-replay",
                                       "PoP jti has already been consumed in this replay scope") from error
        except sqlite3.Error as error:
            try:
                connection.execute("ROLLBACK")
            except sqlite3.Error:
                pass
            raise PopVerificationError("pop-replay-store-unavailable",
                                       "replay store failed; invocation must fail closed") from error
        finally:
            connection.close()
    except PopVerificationError:
        raise
    except (OSError, sqlite3.Error) as error:
        raise PopVerificationError("pop-replay-store-unavailable",
                                   "replay store could not be opened; invocation must fail closed") from error


def verify_chain_invocation(raw_chain: dict[str, Any],
                            trusted_anchors: list[dict[str, Any]],
                            tool: Any,
                            args: Any,
                            pop_token: Any,
                            replay_database: Path,
                            replay_scope: str,
                            *,
                            trusted_now: int,
                            expected_audience: str | None = None,
                            openssl_binary: str = "openssl") -> dict[str, Any]:
    """Verify chain, leaf capability/invocation, PoP, and stateful one-time jti.

    trusted_now MUST come from the enforcement point's trusted clock. The
    embedded raw_chain['now'] is caller-controlled fixture/input data and is
    overwritten before the compact-chain verifier is called.
    """
    if type(trusted_now) is not int or not 0 <= trusted_now <= MAX_SAFE_INTEGER:
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "findings": [{"code": "trusted-clock-invalid",
                          "detail": "trusted_now must be a non-negative safe integer Unix timestamp"}],
            "qualification": "NOT_CLAIMED",
        }
    if not isinstance(raw_chain, dict):
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "findings": [{"code": "chain-input-invalid"}],
            "qualification": "NOT_CLAIMED",
        }
    chain_input = dict(raw_chain)
    chain_input["now"] = trusted_now
    chain_result = chain_verifier.evaluate_compact_chain(
        chain_input, trusted_anchors, openssl_binary=openssl_binary, trusted_now=trusted_now
    )
    if chain_result.get("status") != "COMPACT_JWS_CRYPTO_LINKAGE_PASS":
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": chain_result.get("status", "UNSUPPORTED_OR_UNDECIDABLE"),
            "chain_result": chain_result,
            "qualification": "NOT_CLAIMED",
        }

    try:
        chain = raw_chain.get("chain")
        if not isinstance(chain, list) or not chain:
            raise PopVerificationError("verified-chain-material-unavailable",
                                       "verified chain token material is unavailable")
        leaf_token = chain[-1]
        _h, _p, _s, _header, _untrusted_payload, _sig = chain_verifier.split_compact_jws(leaf_token)
        # This parse is deliberately after successful full-chain signature verification.
        leaf_claims = chain_verifier._parse_authenticated_token(leaf_token, len(chain) - 1)
        leaf_jti = leaf_claims["jti"]
        leaf_jwk = leaf_claims["cnf"]["jwk"]
        leaf_tools = capability.validate_authorization_details(
            leaf_claims["authorization_details"], require_one=True, label="verified-leaf"
        )
        try:
            capability.validate_invocation(leaf_tools, tool, args)
        except capability.CapabilityError as error:
            raise PopVerificationError(error.code, error.detail) from error

        claims = _decode_pop(pop_token, leaf_jwk, openssl_binary)
        now = trusted_now
        verify_pop_invocation_binding(claims, leaf_jti, tool, args, now, expected_audience)

        _consume_pop_jti(replay_database, replay_scope, claims["jti"], claims["iat"],
                         now, POP_CLOCK_TOLERANCE_SECONDS)
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
            "leaf_jti": leaf_jti,
            "pop_jti": claims["jti"],
            "tool": tool,
            "replay_scope": replay_scope,
            "replay_jti_consumed": True,
            "verified_invariants": [
                "trusted-clock-injected-into-chain-and-pop-checks",
                "full-compact-aat-chain-verified-first",
                "leaf-capability-and-invocation-constraints",
                "leaf-cnf-jwk-ed25519-pop-signature",
                "aat_id-bound-to-verified-leaf-jti",
                "aat_tool-equals-invocation",
                "hta-canonical-equality-with-invocation-arguments",
                "pop-iat-within-acceptance-window",
                "optional-deployment-audience-enforced",
                "sqlite-transactional-one-time-pop-jti",
            ],
            "qualification": "NOT_CLAIMED",
            "scope": (
                "bounded invocation and PoP verifier candidate; uses a restricted canonical JSON "
                "subset rather than full RFC 8785; no actual tool dispatch, business-side-effect rollback, "
                "or cross-host replay-store deployment validation"
            ),
        }
    except PopVerificationError as error:
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": "INVOCATION_DENIED",
            "findings": [{"code": error.code, "detail": error.detail}],
            "qualification": "NOT_CLAIMED",
        }
    except (KeyError, TypeError, ValueError, RecursionError) as error:
        return {
            "schema": POP_RESULT_SCHEMA,
            "status": "INVOCATION_DENIED",
            "findings": [{"code": "verified-leaf-material-invalid", "detail": str(error)}],
            "qualification": "NOT_CLAIMED",
        }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--help-scope", action="store_true",
                        help="print the verification boundary without accepting invocation data")
    args = parser.parse_args()
    if args.help_scope:
        print("This module is a library candidate; call verify_chain_invocation() from an enforcement adapter.")
    else:
        parser.error("This module has no CLI authorization interface; use the typed Python function.")
