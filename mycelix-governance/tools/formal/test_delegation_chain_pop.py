#!/usr/bin/env python3
"""Signed AAT PoP invocation tests with independent replay and mutation controls.

The AAT chain fixtures and PoP JWTs are signed using temporary Ed25519 keys via
OpenSSL. Tests require the complete AAT chain to verify before PoP is evaluated.
The tested PoP canonicalization is explicitly the implementation's restricted
JCS-compatible subset, not the full RFC 8785 value domain.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
from concurrent.futures import ThreadPoolExecutor
import json
import os
import shutil
import sqlite3
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delegation_chain_compact_jws as chain_verifier  # noqa: E402
import delegation_chain_pop as pop_verifier  # noqa: E402
import test_delegation_chain_compact_jws as aat_fixtures  # noqa: E402

POP_SCHEMA = "mycelix.aat-invocation-pop.v1"
NOW = aat_fixtures.NOW
ISSUER = aat_fixtures.ISSUER
AUDIENCE = "https://tools.example.test"
TOOL = "read_file"
ARGS = {"path": "/public.txt"}
EXPECTED_MUTATIONS = (
    "pop-signature-check-omitted",
    "pop-binding-check-omitted",
    "pop-canonical-payload-check-omitted",
    "pop-replay-consumption-omitted",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def b64u(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def compact_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def run_openssl(args: list[str], timeout: int = 5) -> bytes:
    result = subprocess.run(args, check=False, capture_output=True, timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError("OpenSSL command failed: " + result.stderr.decode("utf-8", errors="replace"))
    return result.stdout


def make_signed_token(private_path: Path, header: dict[str, Any], payload: bytes,
                      directory: Path, label: str) -> str:
    directory.mkdir(parents=True, exist_ok=True)
    header_segment = b64u(compact_json(header))
    payload_segment = b64u(payload)
    signing_input = (header_segment + "." + payload_segment).encode("ascii")
    input_path = directory / (label + ".signing-input")
    signature_path = directory / (label + ".signature")
    input_path.write_bytes(signing_input)
    run_openssl([
        "openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(private_path),
        "-in", str(input_path), "-out", str(signature_path),
    ])
    return header_segment + "." + payload_segment + "." + b64u(signature_path.read_bytes())


def make_chain(directory: Path, empty_constraint_map: bool = False) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    directory.mkdir(parents=True, exist_ok=True)
    openssl = shutil.which("openssl")
    if openssl is None:
        raise RuntimeError("OpenSSL executable is required for Ed25519 proof fixtures")
    keys = [aat_fixtures.generate_keypair(directory, index + 100) for index in range(4)]
    jwks = [item["public_jwk"] for item in keys]
    chain: list[str] = []
    previous_signing_input: str | None = None
    root_iat = NOW - 5
    root_exp = NOW + 3600

    for index in range(3):
        constraints: dict[str, Any] = {} if empty_constraint_map else {
            "path": {"constraint_type": "exact", "value": "/public.txt"}
        }
        claims: dict[str, Any] = {
            "jti": f"aat-leaf-chain-{index}",
            "iss": ISSUER if index == 0 else aat_fixtures.thumbprint_uri(jwks[index]),
            "iat": root_iat + index,
            "exp": root_exp - index * 100,
            "del_depth": index,
            "del_max_depth": 2,
            "cnf": {"jwk": copy.deepcopy(jwks[index + 1])},
            "authorization_details": [{
                "type": "attenuating_agent_token",
                "tools": {TOOL: constraints},
            }],
        }
        if index:
            assert previous_signing_input is not None
            claims["par_hash"] = b64u(hashlib.sha256(previous_signing_input.encode("ascii")).digest())
        token = aat_fixtures.sign_token(
            keys[index]["private_path"],
            {"alg": "EdDSA", "typ": "aat+jwt", "kid": f"pop-aat-{index}"},
            claims,
            directory,
            f"aat-{index}",
        )
        chain.append(token)
        parts = token.split(".")
        previous_signing_input = parts[0] + "." + parts[1]

    raw = {"schema": chain_verifier.SCHEMA, "now": NOW, "chain": chain}
    anchors = [{"issuer_uri": ISSUER, "jwk": copy.deepcopy(jwks[0])}]
    return raw, anchors, keys


def make_pop(directory: Path, private_path: Path, leaf_jti: str, *,
             jti: str = "pop-unique-0001", iat: int = NOW,
             aat_id: str | None = None, aat_tool: str = TOOL,
             hta: dict[str, Any] | None = None, audience: Any = AUDIENCE,
             extra_claims: dict[str, Any] | None = None,
             canonical: bool = True, header: dict[str, Any] | None = None) -> str:
    claims: dict[str, Any] = {
        "jti": jti,
        "iat": iat,
        "aat_id": leaf_jti if aat_id is None else aat_id,
        "aat_tool": aat_tool,
        "hta": copy.deepcopy(ARGS if hta is None else hta),
    }
    if audience is not None:
        claims["aat_aud"] = audience
    if extra_claims:
        claims.update(copy.deepcopy(extra_claims))
    if canonical:
        payload = pop_verifier.canonical_json_profile(claims)
    else:
        # Valid JSON signed as-is, but intentionally not in canonical member order.
        payload = json.dumps(claims, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    protected = {"alg": "EdDSA", "typ": "pop+jwt"}
    if header:
        protected.update(copy.deepcopy(header))
    return make_signed_token(private_path, protected, payload, directory, "proof-" + jti)


def invoke(raw: dict[str, Any], anchors: list[dict[str, Any]], pop_token: str,
           db_path: Path, *, tool: Any = TOOL, args: Any = ARGS,
           expected_audience: str | None = AUDIENCE,
           trusted_now: int | None = None) -> dict[str, Any]:
    enforcement_now = NOW if trusted_now is None else trusted_now
    return pop_verifier.verify_chain_invocation(
        raw, anchors, tool, args, pop_token,
        replay_database=db_path, replay_scope="tools.example.test",
        trusted_now=enforcement_now, expected_audience=expected_audience,
        openssl_binary=shutil.which("openssl") or "openssl",
    )


def expected_code(result: dict[str, Any]) -> str | None:
    findings = result.get("findings", [])
    return findings[0].get("code") if findings else None


def cases(directory: Path) -> dict[str, tuple[dict[str, Any], list[dict[str, Any]], str, dict[str, Any], Any, Any, str | None, str]]:
    out = {}
    # Each negative case has its own keys and replay database, so its outcome
    # is about the intended invalid condition rather than a prior consumed jti.
    raw, anchors, keys = make_chain(directory)
    leaf_jti = "aat-leaf-chain-2"
    token = make_pop(directory, keys[3]["private_path"], leaf_jti, jti="pop-bad-signature")
    parts = token.split(".")
    last = "A" if parts[2][0] != "A" else "B"
    out["tampered-pop-signature"] = (
        raw, anchors, parts[0] + "." + parts[1] + "." + last + parts[2][1:],
        {}, TOOL, ARGS, AUDIENCE, "pop-signature-invalid",
    )

    builders: list[tuple[str, dict[str, Any]]] = [
        ("wrong-aat-id", {"aat_id": "other-leaf"}),
        ("wrong-tool-claim", {"aat_tool": "delete_file"}),
        ("wrong-hta", {"hta": {"path": "/private.txt"}}),
        ("iat-too-old", {"iat": NOW - 31}),
        ("iat-too-future", {"iat": NOW + 31}),
        ("missing-audience", {"audience": None}),
        ("wrong-audience", {"audience": "https://another.example.test"}),
        ("noncanonical-payload", {"canonical": False}),
        ("unsupported-extra-claim", {"extra_claims": {"unreviewed": "value"}}),
        ("empty-pop-jti", {"jti": ""}),
        ("wrong-pop-header-alg", {"header": {"alg": "none"}}),
        ("wrong-pop-header-type", {"header": {"typ": "aat+jwt"}}),
    ]
    for index, (name, values) in enumerate(builders):
        raw, anchors, keys = make_chain(directory / name)
        pop_args = {
            "jti": f"pop-negative-{index}",
            "aat_id": values.get("aat_id"),
            "aat_tool": values.get("aat_tool", TOOL),
            "hta": values.get("hta", ARGS),
            "iat": values.get("iat", NOW),
            "audience": values.get("audience", AUDIENCE),
            "extra_claims": values.get("extra_claims"),
            "canonical": values.get("canonical", True),
            "header": values.get("header"),
        }
        token = make_pop(directory / name, keys[3]["private_path"], leaf_jti, **pop_args)
        code = {
            "wrong-aat-id": "pop-leaf-id-mismatch",
            "wrong-tool-claim": "pop-tool-mismatch",
            "wrong-hta": "pop-arguments-mismatch",
            "iat-too-old": "pop-iat-outside-window",
            "iat-too-future": "pop-iat-outside-window",
            "missing-audience": "pop-audience-mismatch",
            "wrong-audience": "pop-audience-mismatch",
            "noncanonical-payload": "pop-payload-not-canonical",
            "unsupported-extra-claim": "pop-claim-unsupported",
            "empty-pop-jti": "pop-jti-invalid",
            "wrong-pop-header-alg": "pop-algorithm-not-allowed",
            "wrong-pop-header-type": "pop-token-type-invalid",
        }[name]
        out[name] = (raw, anchors, token, {}, TOOL, ARGS, AUDIENCE, code)

    raw, anchors, keys = make_chain(directory / "unauthorized-tool")
    token = make_pop(directory / "unauthorized-tool", keys[3]["private_path"], leaf_jti,
                     jti="pop-unauthorized-tool", aat_tool="delete_file", hta={})
    out["unauthorized-invocation-tool"] = (
        raw, anchors, token, {}, "delete_file", {}, AUDIENCE, "tool-not-authorized",
    )

    raw, anchors, keys = make_chain(directory / "constraint-violation")
    token = make_pop(directory / "constraint-violation", keys[3]["private_path"], leaf_jti,
                     jti="pop-constraint-violation", hta={"path": "/private.txt"})
    out["leaf-capability-constraint-violation"] = (
        raw, anchors, token, {}, TOOL, {"path": "/private.txt"}, AUDIENCE,
        "invocation-constraint-failed",
    )

    raw, anchors, keys = make_chain(directory / "invalid-chain")
    broken = raw["chain"][1]
    parts = broken.split(".")
    raw["chain"][1] = parts[0] + "." + parts[1] + "." + ("A" if parts[2][0] != "A" else "B") + parts[2][1:]
    token = make_pop(directory / "invalid-chain", keys[3]["private_path"], leaf_jti,
                     jti="pop-chain-invalid")
    out["unverified-aat-chain"] = (
        raw, anchors, token, {}, TOOL, ARGS, AUDIENCE, "INVALID_CHAIN",
    )

    # A present audience without configured enforcement policy is rejected.
    raw, anchors, keys = make_chain(directory / "audience-policy-unconfigured")
    token = make_pop(directory / "audience-policy-unconfigured", keys[3]["private_path"], leaf_jti,
                     jti="pop-audience-unconfigured", audience=AUDIENCE)
    out["audience-policy-unconfigured"] = (
        raw, anchors, token, {}, TOOL, ARGS, None, "pop-audience-policy-unconfigured",
    )

    # A JCS-subset violation is tested on an otherwise unconstrained tool.
    raw, anchors, keys = make_chain(directory / "unsupported-float", empty_constraint_map=True)
    float_args = {"measure": 1.5}
    token = make_pop(directory / "unsupported-float", keys[3]["private_path"], leaf_jti,
                     jti="pop-float", hta=float_args, canonical=False)
    out["restricted-jcs-float"] = (
        raw, anchors, token, {}, TOOL, float_args, AUDIENCE, "jcs-float-outside-profile",
    )
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.aat-invocation-pop-differential-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "controls": [],
        "mutations": [],
    }
    try:
        openssl = shutil.which("openssl")
        require(openssl is not None, "OpenSSL is required for signed Ed25519 AAT/PoP fixtures")
        with tempfile.TemporaryDirectory(prefix="mycelix-pop-test-") as temporary:
            root = Path(temporary)
            raw, anchors, keys = make_chain(root / "valid-positive")
            token = make_pop(root / "valid-positive", keys[3]["private_path"],
                             "aat-leaf-chain-2", jti="pop-valid-0001")
            database = root / "valid-positive" / "replay.sqlite3"
            result = invoke(raw, anchors, token, database)
            require(result.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                    "valid signed PoP invocation rejected: " + json.dumps(result, sort_keys=True))
            receipt["controls"].append({
                "id": "valid-constrained-invocation",
                "status": result["status"],
                "independent_fixture": "AAT chain and PoP signed with OpenSSL Ed25519",
                "replay_jti_consumed": result.get("replay_jti_consumed") is True,
            })

            with sqlite3.connect(str(database)) as connection:
                stored_columns = {row[1] for row in connection.execute("PRAGMA table_info(aat_pop_replay)")}
                stored_rows = connection.execute("SELECT scope, jti_sha256, iat FROM aat_pop_replay").fetchall()
            require("jti_sha256" in stored_columns and "jti" not in stored_columns,
                    "replay store schema persists a raw jti field instead of its digest")
            require(len(stored_rows) == 1 and stored_rows[0][1] == hashlib.sha256(
                b"pop-valid-0001"
            ).hexdigest(), "replay store did not persist the expected one-way jti digest")
            require("pop-valid-0001" not in repr(stored_rows),
                    "replay store exposed the raw PoP jti")

            require(stat.S_IMODE(database.stat().st_mode) == 0o600,
                    "replay database must be owner-only (mode 0600)")
            # Replay the exact same proof in the same store: second use MUST deny.
            replay = invoke(raw, anchors, token, database)
            require(replay.get("status") == "INVOCATION_DENIED"
                    and expected_code(replay) == "pop-jti-replay",
                    "same PoP jti was not atomically rejected on second use")
            receipt["controls"].append({
                "id": "one-time-pop-jti-replay-rejected",
                "status": replay["status"], "finding": expected_code(replay),
            })

            # Reissue a fresh, correctly signed PoP at a later time while
            # reusing the already-consumed jti. The AATs remain unexpired at
            # NOW+1000. A retention policy that deleted the old replay row
            # would wrongly accept this newly signed proof.
            later_raw = copy.deepcopy(raw)
            later_raw["now"] = NOW + 1000
            later_proof = make_pop(
                root / "valid-positive", keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-valid-0001", iat=NOW + 1000,
            )
            later_replay = invoke(later_raw, anchors, later_proof, database, trusted_now=NOW + 1000)
            require(later_replay.get("status") == "INVOCATION_DENIED"
                    and expected_code(later_replay) == "pop-jti-replay",
                    "fresh proof reused a consumed jti after the original clock window")
            receipt["controls"].append({
                "id": "fresh-pop-cannot-reuse-old-jti",
                "status": later_replay["status"],
                "finding": expected_code(later_replay),
                "fresh_iat": NOW + 1000,
                "same_jti_rejected": True,
            })

            concurrent_proof = make_pop(
                root / "valid-positive", keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-concurrent-0001",
            )
            concurrent_db = root / "concurrent-replay.sqlite3"
            with ThreadPoolExecutor(max_workers=2) as executor:
                concurrent_results = list(executor.map(
                    lambda _: invoke(raw, anchors, concurrent_proof, concurrent_db),
                    (0, 1),
                ))
            pass_count = sum(
                item.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS"
                for item in concurrent_results
            )
            replay_denial_count = sum(
                item.get("status") == "INVOCATION_DENIED"
                and expected_code(item) == "pop-jti-replay"
                for item in concurrent_results
            )
            require(pass_count == 1 and replay_denial_count == 1,
                    "concurrent same-jti submission must yield exactly one pass and one replay denial: " +
                    json.dumps(concurrent_results, sort_keys=True))
            receipt["controls"].append({
                "id": "concurrent-pop-jti-race",
                "status": "ONE_PASS_ONE_REPLAY_DENIED",
                "pass_count": pass_count,
                "replay_denial_count": replay_denial_count,
                "same_database": True,
                "independent_connections": True,
            })

            unavailable_proof = make_pop(
                root / "valid-positive", keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-store-unavailable",
            )
            unavailable_database = root / "replay-database-is-directory"
            unavailable_database.mkdir()
            unavailable = invoke(raw, anchors, unavailable_proof, unavailable_database)
            require(unavailable.get("status") == "INVOCATION_DENIED"
                    and expected_code(unavailable) == "pop-replay-store-unavailable",
                    "an unavailable replay store must fail closed: " +
                    json.dumps(unavailable, sort_keys=True))
            receipt["controls"].append({
                "id": "replay-store-unavailable-fails-closed",
                "status": unavailable["status"],
                "finding": expected_code(unavailable),
                "fail_closed": True,
            })

            unsafe_parent = root / "world-writable-replay-parent"
            unsafe_parent.mkdir()
            unsafe_parent.chmod(0o777)
            unsafe_proof = make_pop(
                root / "valid-positive", keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-unsafe-parent",
            )
            unsafe_result = invoke(
                raw, anchors, unsafe_proof, unsafe_parent / "replay.sqlite3",
            )
            unsafe_parent.chmod(0o700)
            require(unsafe_result.get("status") == "INVOCATION_DENIED"
                    and expected_code(unsafe_result) == "pop-replay-store-unavailable",
                    "group/world-writable replay-store parent must fail closed: " +
                    json.dumps(unsafe_result, sort_keys=True))
            receipt["controls"].append({
                "id": "replay-store-unsafe-parent-fails-closed",
                "status": unsafe_result["status"],
                "finding": expected_code(unsafe_result),
                "fail_closed": True,
            })

            symlink_path = root / "replay-store-symlink.sqlite3"
            os.symlink(database, symlink_path)
            symlink_proof = make_pop(
                root / "valid-positive", keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-symlink-store",
            )
            symlink_result = invoke(raw, anchors, symlink_proof, symlink_path)
            require(symlink_result.get("status") == "INVOCATION_DENIED"
                    and expected_code(symlink_result) == "pop-replay-store-unavailable",
                    "a symlinked replay-store path must fail closed: " +
                    json.dumps(symlink_result, sort_keys=True))
            receipt["controls"].append({
                "id": "replay-store-symlink-fails-closed",
                "status": symlink_result["status"],
                "finding": expected_code(symlink_result),
                "fail_closed": True,
            })

            clock_raw, clock_anchors, clock_keys = make_chain(root / "untrusted-clock")
            # The caller-supplied fixture clock makes the chain claims look
            # valid; the proof itself is freshly signed for the trusted current
            # time. The trusted enforcement clock is after token expiry.
            clock_raw["now"] = NOW - 30
            clock_proof = make_pop(
                root / "untrusted-clock", clock_keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-forged-clock", iat=NOW + 3500,
            )
            clock_result = invoke(
                clock_raw, clock_anchors, clock_proof,
                root / "untrusted-clock" / "clock-replay.sqlite3",
                trusted_now=NOW + 3500,
            )
            require(clock_result.get("status") == "INVALID_CHAIN",
                    "caller-supplied time bypassed the trusted enforcement clock: " +
                    json.dumps(clock_result, sort_keys=True))
            chain_findings = clock_result.get("chain_result", {}).get("findings", [])
            require(any(item.get("code") == "token-expired" for item in chain_findings),
                    "trusted-clock expiry rejection lacks the expected token-expired finding")
            receipt["controls"].append({
                "id": "caller-supplied-clock-cannot-resurrect-expired-chain",
                "status": clock_result["status"],
                "trusted_now": NOW + 3500,
                "untrusted_bundle_now": NOW - 30,
                "expired_chain_rejected": True,
            })

            # Optional-audience profile with neither expected nor presented aud.
            raw, anchors, keys = make_chain(root / "optional-audience")
            token = make_pop(root / "optional-audience", keys[3]["private_path"],
                             "aat-leaf-chain-2", jti="pop-no-audience", audience=None)
            result = invoke(raw, anchors, token, root / "optional-audience" / "replay.sqlite3",
                            expected_audience=None)
            require(result.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                    "optional no-audience invocation rejected: " + json.dumps(result, sort_keys=True))
            receipt["controls"].append({
                "id": "audience-optional-when-unconfigured-and-absent",
                "status": result["status"],
            })

            bad_cases = cases(root / "negative-cases")
            for name, (chain, trust, proof, _unused, tool, actual_args, expected_aud, expected) in bad_cases.items():
                db = root / "replay" / (name + ".sqlite3")
                result = invoke(chain, trust, proof, db, tool=tool, args=actual_args,
                                expected_audience=expected_aud)
                if expected == "INVALID_CHAIN":
                    require(result.get("status") == "INVALID_CHAIN",
                            name + ": unverified chain did not prevent PoP evaluation")
                    finding = result.get("chain_result", {}).get("findings", [{}])[0].get("code")
                    require(finding == "signature-invalid",
                            name + ": invalid chain failed for unexpected reason: " + str(finding))
                else:
                    require(result.get("status") == "INVOCATION_DENIED",
                            name + ": expected INVOCATION_DENIED, got " +
                            json.dumps(result, sort_keys=True))
                    finding = expected_code(result)
                    require(finding == expected,
                            name + ": expected " + expected + ", got " + str(finding))
                receipt["controls"].append({
                    "id": name, "status": result["status"],
                    "finding": finding, "independent_expected": expected,
                })

            # Each mutation must make a known-invalid invocation appear to pass.
            mutant_cases = (
                ("pop-signature-check-omitted", "tampered-pop-signature", "verify_pop_signature"),
                ("pop-binding-check-omitted", "wrong-aat-id", "verify_pop_invocation_binding"),
                ("pop-canonical-payload-check-omitted", "noncanonical-payload", "verify_pop_payload_canonical"),
            )
            for mutant_id, case_id, function_name in mutant_cases:
                chain, trust, proof, _unused, tool, actual_args, expected_aud, _expected = bad_cases[case_id]
                original = getattr(pop_verifier, function_name)
                try:
                    setattr(pop_verifier, function_name, lambda *unused_args, **unused_kwargs: None)
                    result = invoke(chain, trust, proof, root / "mutant-replay" / (mutant_id + ".sqlite3"),
                                    tool=tool, args=actual_args, expected_audience=expected_aud)
                finally:
                    setattr(pop_verifier, function_name, original)
                require(result.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                        mutant_id + ": missing-check mutant did not accept known-invalid proof: " +
                        json.dumps(result, sort_keys=True))
                receipt["mutations"].append({
                    "id": mutant_id, "rejected_if_unmutated": True,
                    "mutant_acceptance_observed": True,
                })

            # A replay mutant must permit duplicate invocation; the independent
            # expected control is the second use of the same valid proof.
            raw, anchors, keys = make_chain(root / "replay-mutant")
            proof = make_pop(root / "replay-mutant", keys[3]["private_path"],
                             "aat-leaf-chain-2", jti="pop-replay-mutant")
            database = root / "replay-mutant" / "replay.sqlite3"
            original_consume = pop_verifier._consume_pop_jti
            try:
                result_first = invoke(raw, anchors, proof, database)
                require(result_first.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                        "replay mutation baseline failed")
                pop_verifier._consume_pop_jti = lambda *unused_args, **unused_kwargs: None
                result_second = invoke(raw, anchors, proof, database)
            finally:
                pop_verifier._consume_pop_jti = original_consume
            require(result_second.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                    "replay-store mutation did not accept duplicate proof")
            receipt["mutations"].append({
                "id": "pop-replay-consumption-omitted",
                "rejected_if_unmutated": True, "mutant_acceptance_observed": True,
            })

            # Mutant: ignore the trusted clock injected by verify_chain_invocation
            # and restore the untrusted bundle time just before chain validation.
            # The PoP itself has a fresh iat at trusted_now, so without the chain
            # expiry check this would accept an expired leaf.
            clock_raw, clock_anchors, clock_keys = make_chain(root / "trusted-clock-mutant")
            clock_raw["now"] = NOW - 30
            clock_proof = make_pop(
                root / "trusted-clock-mutant", clock_keys[3]["private_path"],
                "aat-leaf-chain-2", jti="pop-clock-mutant", iat=NOW + 3500,
            )
            original_clock_check = chain_verifier.evaluate_compact_chain

            def ignore_trusted_clock(candidate: dict[str, Any], anchors: list[dict[str, Any]],
                                     openssl_binary: str = "openssl") -> dict[str, Any]:
                mutated = dict(candidate)
                mutated["now"] = clock_raw["now"]
                return original_clock_check(mutated, anchors, openssl_binary=openssl_binary)

            try:
                chain_verifier.evaluate_compact_chain = ignore_trusted_clock
                clock_mutant_result = invoke(
                    clock_raw, clock_anchors, clock_proof,
                    root / "trusted-clock-mutant" / "replay.sqlite3",
                    trusted_now=NOW + 3500,
                )
            finally:
                chain_verifier.evaluate_compact_chain = original_clock_check
            require(clock_mutant_result.get("status") == "INVOCATION_POP_VERIFIED_CANDIDATE_PASS",
                    "clock mutation did not accept the expired-chain fixture: " +
                    json.dumps(clock_mutant_result, sort_keys=True))
            receipt["mutations"].append({
                "id": "pop-trusted-clock-check-omitted",
                "rejected_if_unmutated": True,
                "mutant_acceptance_observed": True,
            })

        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        checker_path = HERE / "delegation_chain_pop.py"
        fixture_path = HERE / "test_delegation_chain_compact_jws.py"
        receipt["checker_sha256"] = hashlib.sha256(checker_path.read_bytes()).hexdigest()
        receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
        receipt["aat_fixture_sha256"] = hashlib.sha256(fixture_path.read_bytes()).hexdigest()
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "positive_controls": 2,
            "negative_controls": len(receipt["controls"]) - 2,
            "replay_reuse_rejected": True,
            "replay_jti_consumed": True,
            "mutants_detected": len(receipt["mutations"]),
            "verified_pop_mutants_detected": len(receipt["mutations"]),
            "qualification": "NOT_CLAIMED",
            "canonicalization_profile": "BMP unicode + safe integers; no floats",
            "side_effecting_invocations_require_stateful_replay": True,
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AAT PoP INVOCATION CHECK PASS: signed leaf-bound proof, capability, argument, time, audience and replay controls")
        print("AAT PoP MUTATION SENSITIVITY PASS: 5 of 5 omitted checks detected")
        print("QUALIFICATION NOT CLAIMED: restricted JCS subset, deployment replay storage, and tool dispatch need further validation")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AAT PoP INVOCATION CHECK FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
