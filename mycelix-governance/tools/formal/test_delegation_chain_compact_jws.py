#!/usr/bin/env python3
"""Adversarial controls for the compact Ed25519 JWS chain verifier.

Fixtures are real compact JWS-shaped tokens signed using OpenSSL Ed25519 keys
generated at test time. This harness does not equate signature validation with
full AAT enforcement: authorization_details subsumption and leaf PoP remain
explicitly out of scope.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delegation_chain_compact_jws as verifier  # noqa: E402

SCHEMA = "mycelix.compact-jws-aat-chain.v1"
ISSUER = "https://issuer.example.test"
NOW = 1_700_000_000
FROZEN_DEPTH = 8
FROZEN_TOKEN_SIZE = 64 * 1024
FROZEN_STACK_SIZE = 256 * 1024


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_openssl(args: list[str], *, timeout: int = 5) -> bytes:
    completed = subprocess.run(args, capture_output=True, timeout=timeout, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            "OpenSSL command failed: " + " ".join(args[:3]) + " :: " +
            completed.stderr.decode("utf-8", errors="replace")[:500]
        )
    return completed.stdout


def b64u(raw: bytes) -> str:
    return base64.urlsafe_b64encode(raw).rstrip(b"=").decode("ascii")


def compact_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def generate_keypair(directory: Path, index: int) -> dict[str, Any]:
    private_path = directory / f"private-{index}.pem"
    public_der_path = directory / f"public-{index}.der"
    run_openssl(["openssl", "genpkey", "-algorithm", "Ed25519", "-out", str(private_path)])
    run_openssl(["openssl", "pkey", "-in", str(private_path), "-pubout",
                 "-outform", "DER", "-out", str(public_der_path)])
    der = public_der_path.read_bytes()
    require(len(der) >= 32, "OpenSSL produced unexpectedly short Ed25519 SPKI DER")
    raw_public = der[-32:]
    public_jwk = {"kty": "OKP", "crv": "Ed25519", "x": b64u(raw_public)}
    return {"private_path": private_path, "public_jwk": public_jwk}


def thumbprint_uri(jwk: dict[str, Any]) -> str:
    canonical = compact_json({"crv": "Ed25519", "kty": "OKP", "x": jwk["x"]})
    return verifier.THUMBPRINT_URI_PREFIX + b64u(hashlib.sha256(canonical).digest())


def encode_segment(raw: bytes) -> str:
    return b64u(raw)


def signing_input(header: dict[str, Any], claims: dict[str, Any]) -> str:
    return encode_segment(compact_json(header)) + "." + encode_segment(compact_json(claims))


def sign_token(private_path: Path, header: dict[str, Any], claims: dict[str, Any],
               directory: Path, token_name: str, raw_payload: bytes | None = None) -> str:
    header_segment = encode_segment(compact_json(header))
    payload_segment = encode_segment(raw_payload if raw_payload is not None else compact_json(claims))
    si = (header_segment + "." + payload_segment).encode("ascii")
    input_path = directory / f"{token_name}.input"
    sig_path = directory / f"{token_name}.sig"
    input_path.write_bytes(si)
    run_openssl(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(private_path),
                 "-in", str(input_path), "-out", str(sig_path)])
    return header_segment + "." + payload_segment + "." + encode_segment(sig_path.read_bytes())


def build_chain(directory: Path, case: str = "valid-four-token-chain", count: int = 4) -> dict[str, Any]:
    keys = [generate_keypair(directory, i) for i in range(count + 1)]
    holder_jwks = [key["public_jwk"] for key in keys]
    if case == "valid-compatible-key-metadata-chain":
        for index, jwk in enumerate(holder_jwks):
            jwk.update({
                "use": "sig",
                "alg": "EdDSA",
                "key_ops": ["verify"],
                "kid": f"holder-{index}",
                "x-deployment-note": "unknown JWK metadata must be ignored",
            })
    chain: list[str] = []
    parent_signing_input: str | None = None
    root_iat = NOW - 200
    root_exp = NOW + 10_000

    for index in range(count):
        header: dict[str, Any] = {"alg": "EdDSA", "typ": "aat+jwt", "kid": f"fixture-{index}"}
        tool_map: dict[str, Any] = {"read_file": {}}
        if case == "child-adds-tool" and index == 1:
            tool_map["delete_file"] = {}
        elif case == "constraint-expansion":
            tool_map = {
                "read_file": {
                    "path": {
                        "constraint_type": "one_of",
                        "values": ["/public.txt"] if index == 0 else ["/public.txt", "/private.txt"],
                    }
                }
            }
        elif case == "argument-key-added" and index in {0, 1}:
            tool_map = {
                "read_file": {"path": {"constraint_type": "exact", "value": "/public.txt"}}
            }
            if index == 1:
                tool_map["read_file"]["admin"] = {"constraint_type": "wildcard"}
        elif case == "unknown-constraint-type" and index == 0:
            tool_map = {
                "read_file": {"path": {"constraint_type": "regex", "pattern": ".*"}}
            }
        claims: dict[str, Any] = {
            "jti": f"token-{index}",
            "iss": ISSUER if index == 0 else thumbprint_uri(holder_jwks[index]),
            "iat": root_iat + index * 10,
            "exp": root_exp - index * 100,
            "del_depth": index,
            "del_max_depth": 3,
            "cnf": {"jwk": copy.deepcopy(holder_jwks[index + 1])},
            "authorization_details": [{
                "type": "attenuating_agent_token",
                "tools": tool_map,
            }],
        }
        if index:
            claims["par_hash"] = b64u(hashlib.sha256(parent_signing_input.encode("ascii")).digest())

        # Negative cases alter one invariant while keeping the affected token's
        # signature valid whenever the point of the test is a claim-level check.
        if case == "wrong-root-issuer" and index == 0:
            claims["iss"] = "https://attacker.example.test"
        elif case == "wrong-par-hash" and index == 1:
            claims["par_hash"] = b64u(hashlib.sha256(b"not-the-parent-signing-input").digest())
        elif case == "wrong-derived-issuer" and index == 1:
            claims["iss"] = ISSUER
        elif case == "child-expiry-exceeds-parent" and index == 1:
            claims["exp"] = root_exp + 1
        elif case == "depth-skips-parent" and index == 2:
            claims["del_depth"] = index + 1
        elif case == "maximum-depth-expands" and index == 1:
            claims["del_max_depth"] = 4
        elif case == "duplicate-jti" and index == 2:
            claims["jti"] = "token-0"
        elif case == "private-holder-key" and index == 0:
            claims["cnf"]["jwk"]["d"] = "private-material-must-not-appear"
        elif case == "jwk-use-encryption" and index == 0:
            claims["cnf"]["jwk"]["use"] = "enc"
        elif case == "jwk-algorithm-mismatch" and index == 0:
            claims["cnf"]["jwk"]["alg"] = "ES256"
        elif case == "jwk-key-ops-missing-verify" and index == 0:
            claims["cnf"]["jwk"]["key_ops"] = ["sign"]
        elif case == "jwk-key-ops-duplicate" and index == 0:
            claims["cnf"]["jwk"]["key_ops"] = ["verify", "verify"]
        elif case == "jwk-key-ops-wrong-type" and index == 0:
            claims["cnf"]["jwk"]["key_ops"] = "verify"
        elif case == "jwk-key-ops-unrelated-operation" and index == 0:
            claims["cnf"]["jwk"]["key_ops"] = ["verify", "encrypt"]
        elif case == "valid-four-token-chain" and index == 0:
            raw_payload = compact_json(claims).replace(b'"jti":"token-0"', b'"jti":"token-0","unrecognized_extension":1e999')
        elif case == "duplicate-payload-member" and index == 0:
            raw_payload = compact_json(claims).replace(b'"jti":"token-0"', b'"jti":"token-x","jti":"token-0"')
        elif case == "duplicate-nonjti-payload-member" and index == 0:
            duplicate_iat = str(claims["iat"]).encode("ascii")
            raw_payload = compact_json(claims).replace(
                b'"jti":"token-0"', b'"jti":"token-0","iat":' + duplicate_iat
            )
        else:
            raw_payload = None

        signer_index = index
        if case == "wrong-child-signing-key" and index == 1:
            signer_index = 2
        if case == "none-algorithm" and index == 1:
            header["alg"] = "none"
        elif case == "b64-header-present" and index == 1:
            header["b64"] = True
        elif case == "critical-header-present" and index == 1:
            header["crit"] = ["unsupported-extension"]
        elif case == "wrong-token-type" and index == 1:
            header["typ"] = "JWT"
        elif case == "unhashable-header-typ" and index == 1:
            header["typ"] = []

        if case == "parent-token-reassociation" and index == 0:
            header["kid"] = "different-token-instance"

        token = sign_token(keys[signer_index]["private_path"], header, claims, directory,
                           f"{case}-{index}", raw_payload=raw_payload)
        if case == "tampered-signature" and index == 2:
            h, p, sig = token.split(".")
            b = "A" if sig[0] != "A" else "B"
            token = h + "." + p + "." + b + sig[1:]
        if case == "malformed-compact-token" and index == 2:
            token = token + ".extra"
        if case == "oversized-token" and index == 2:
            token = "A" * (FROZEN_TOKEN_SIZE + 1)
        chain.append(token)
        parent_signing_input = token.split(".")[0] + "." + token.split(".")[1]

    if case == "parent-token-reassociation":
        # Token 0 was re-signed with a different protected header. Rebuild only
        # token 1 using the old par_hash to leave a valid child signature but an
        # invalid parent-token-instance commitment.
        old_token = chain[1]
        parts = old_token.split(".")
        claims = json.loads(base64.urlsafe_b64decode(parts[1] + "=" * ((4-len(parts[1])%4)%4)))
        original_root_header = {"alg": "EdDSA", "typ": "aat+jwt", "kid": "fixture-0"}
        original_parent_input = (
            encode_segment(compact_json(original_root_header)) + "." + chain[0].split(".")[1]
        )
        claims["par_hash"] = b64u(hashlib.sha256(original_parent_input.encode("ascii")).digest())
        chain[1] = sign_token(keys[1]["private_path"], {"alg": "EdDSA", "typ": "aat+jwt", "kid": "fixture-1"},
                              claims, directory, f"{case}-relinked-child")

    anchors = [{"issuer_uri": ISSUER, "jwk": copy.deepcopy(keys[0]["public_jwk"])}]
    if case == "wrong-root-trust-anchor":
        wrong = generate_keypair(directory, 20)
        anchors = [{"issuer_uri": ISSUER, "jwk": copy.deepcopy(wrong["public_jwk"])}]
    if case == "duplicate-trust-anchor-issuer":
        anchors.append(copy.deepcopy(anchors[0]))
    if case == "unknown-schema":
        schema = "future-unknown-schema"
    else:
        schema = SCHEMA
    if case == "malformed-top-level":
        chain = []
    if case == "oversized-stack":
        chain = [("A" * (FROZEN_STACK_SIZE // 2)) for _ in range(3)]
    return {"schema": schema, "now": NOW, "trust_anchors": anchors, "chain": chain}


def cases(directory: Path) -> dict[str, dict[str, Any]]:
    names = (
        "tampered-signature",
        "wrong-root-trust-anchor",
        "wrong-root-issuer",
        "wrong-child-signing-key",
        "wrong-par-hash",
        "wrong-derived-issuer",
        "child-expiry-exceeds-parent",
        "depth-skips-parent",
        "maximum-depth-expands",
        "duplicate-jti",
        "private-holder-key",
        "jwk-use-encryption",
        "jwk-algorithm-mismatch",
        "jwk-key-ops-missing-verify",
        "jwk-key-ops-duplicate",
        "jwk-key-ops-wrong-type",
        "jwk-key-ops-unrelated-operation",
        "duplicate-payload-member",
        "duplicate-nonjti-payload-member",
        "none-algorithm",
        "b64-header-present",
        "critical-header-present",
        "wrong-token-type",
        "child-adds-tool",
        "constraint-expansion",
        "argument-key-added",
        "unknown-constraint-type",
        "unhashable-header-typ",
        "malformed-compact-token",
        "oversized-token",
        "parent-token-reassociation",
        "duplicate-trust-anchor-issuer",
        "oversized-stack",
        "malformed-top-level",
        "unknown-schema",
    )
    return {name: build_chain(directory, name) for name in names}


def invoke_fixture(raw: dict[str, Any], openssl: str,
                   trusted_anchors: list[dict[str, Any]] | None = None,
                   trusted_now: int = NOW) -> dict[str, Any]:
    # Exclude any trust configuration from the token-chain input. Trusted keys
    # and the enforcement clock are separate, explicit inputs.
    anchors = trusted_anchors if trusted_anchors is not None else raw.get("trust_anchors")
    token_input = {key: raw[key] for key in ("schema", "now", "chain") if key in raw}
    return verifier.evaluate_compact_chain(token_input, anchors,
                                           openssl_binary=openssl, trusted_now=trusted_now)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.compact-jws-aat-chain-differential-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "controls": [],
        "mutations": [],
    }
    try:
        openssl = shutil.which("openssl")
        require(openssl is not None, "OpenSSL executable is required for Ed25519 signature fixtures")
        require(verifier.MAX_DELEGATION_DEPTH == FROZEN_DEPTH, "delegation depth limit drifted")
        require(verifier.MAX_TOKEN_SIZE_BYTES == FROZEN_TOKEN_SIZE, "per-token size limit drifted")
        require(verifier.MAX_STACK_SIZE_BYTES == FROZEN_STACK_SIZE, "stack size limit drifted")
        with tempfile.TemporaryDirectory(prefix="mycelix-compact-jws-test-") as temporary:
            root = Path(temporary)
            valid = build_chain(root, "valid-four-token-chain")
            observed = invoke_fixture(valid, openssl)
            require(observed.get("status") == "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
                    "valid signed chain rejected: " + json.dumps(observed, sort_keys=True))
            receipt["controls"].append({
                "id": "valid-four-token-chain", "status": observed["status"],
                "independent_fixture": "OpenSSL Ed25519 signatures verified",
                "token_count": len(valid["chain"]),
            })
            valid_single = build_chain(root, "valid-single-token-chain", count=1)
            observed_single = invoke_fixture(valid_single, openssl)
            require(observed_single.get("status") == "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
                    "valid root-as-leaf single-token chain rejected: " +
                    json.dumps(observed_single, sort_keys=True))
            receipt["controls"].append({
                "id": "valid-single-token-chain", "status": observed_single["status"],
                "independent_fixture": "OpenSSL root signature verified; root also treated as leaf",
                "token_count": len(valid_single["chain"]),
            })

            valid_metadata = build_chain(root, "valid-compatible-key-metadata-chain")
            observed_metadata = invoke_fixture(valid_metadata, openssl)
            require(observed_metadata.get("status") == "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
                    "consistent JWK usage metadata or unknown optional JWK member rejected: " +
                    json.dumps(observed_metadata, sort_keys=True))
            receipt["controls"].append({
                "id": "valid-compatible-key-metadata-chain",
                "status": observed_metadata["status"],
                "independent_fixture": "OpenSSL Ed25519; use=sig, alg=EdDSA, key_ops=verify",
                "token_count": len(valid_metadata["chain"]),
            })

            no_clock = verifier.evaluate_compact_chain(
                {"schema": valid["schema"], "now": valid["now"], "chain": valid["chain"]},
                valid["trust_anchors"], openssl_binary=openssl,
            )
            require(no_clock.get("status") == "UNSUPPORTED_OR_UNDECIDABLE"
                    and no_clock.get("findings", [{}])[0].get("code") == "trusted-clock-invalid",
                    "compact chain verifier accepted an embedded time without a trusted clock")
            receipt["controls"].append({
                "id": "missing-trusted-clock-rejected",
                "status": no_clock["status"],
                "finding": no_clock.get("findings", [{}])[0].get("code"),
            })

            forged_clock_input = {
                "schema": valid["schema"], "now": NOW - 30, "chain": valid["chain"],
            }
            forged_clock_result = invoke_fixture(
                forged_clock_input, openssl, valid["trust_anchors"], trusted_now=NOW + 11000,
            )
            require(forged_clock_result.get("status") == "INVALID_CHAIN"
                    and any(item.get("code") == "token-expired"
                            for item in forged_clock_result.get("findings", [])),
                    "compact chain verifier let bundle time resurrect an expired chain: " +
                    json.dumps(forged_clock_result, sort_keys=True))
            receipt["controls"].append({
                "id": "caller-supplied-clock-cannot-resurrect-expired-chain",
                "status": forged_clock_result["status"],
                "trusted_now": NOW + 11000,
                "untrusted_bundle_now": NOW - 30,
                "expiry_rejected": True,
            })
            # Reject attempts to smuggle trust roots in the same object as
            # untrusted token-chain data. Real trust configuration stays outside.
            wrong_anchor = generate_keypair(root, 30)
            malicious_input = {
                "schema": valid["schema"],
                "now": valid["now"],
                "chain": valid["chain"],
                "trust_anchors": [{"issuer_uri": ISSUER, "jwk": wrong_anchor["public_jwk"]}],
            }
            rejected_embedded_config = verifier.evaluate_compact_chain(
                malicious_input, valid["trust_anchors"], openssl_binary=openssl, trusted_now=NOW
            )
            require(rejected_embedded_config.get("status") == "UNSUPPORTED_OR_UNDECIDABLE",
                    "chain input was allowed to carry its own trust-anchor configuration")
            require(rejected_embedded_config.get("findings", [{}])[0].get("code") == "unexpected-chain-input-field",
                    "embedded trust-anchor input rejected for the wrong reason")
            receipt["controls"].append({
                "id": "embedded-trust-anchor-field-rejected",
                "status": rejected_embedded_config["status"],
                "finding": rejected_embedded_config.get("findings", [{}])[0].get("code"),
            })
            non_object = verifier.evaluate_compact_chain(
                ["not", "an", "object"], valid["trust_anchors"], openssl_binary=openssl, trusted_now=NOW
            )
            require(non_object.get("status") == "UNSUPPORTED_OR_UNDECIDABLE",
                    "non-object chain request did not fail closed")
            require(non_object.get("findings", [{}])[0].get("code") == "top-level-input-not-object",
                    "non-object chain request failed for the wrong reason")
            receipt["controls"].append({
                "id": "non-object-chain-input-rejected",
                "status": non_object["status"],
                "finding": non_object.get("findings", [{}])[0].get("code"),
            })
            bad_cases = cases(root)
            for name, raw in bad_cases.items():
                if name in {"duplicate-jti", "duplicate-payload-member"}:
                    # The verifier must reject a repeated untrusted token ID
                    # before doing any expensive public-key signature operation.
                    original_verify = verifier.verify_ed25519_signature
                    signature_calls = 0

                    def forbidden_signature_call(*args: Any, **kwargs: Any) -> None:
                        nonlocal signature_calls
                        signature_calls += 1
                        raise AssertionError("signature verification ran before duplicate-jti rejection")

                    try:
                        verifier.verify_ed25519_signature = forbidden_signature_call
                        observed = invoke_fixture(raw, openssl)
                    finally:
                        verifier.verify_ed25519_signature = original_verify
                    require(signature_calls == 0,
                            name + " was not rejected during the pre-signature jti scan")
                else:
                    observed = invoke_fixture(raw, openssl)
                expected = "UNSUPPORTED_OR_UNDECIDABLE" if name in {
                    "duplicate-trust-anchor-issuer", "malformed-top-level", "unknown-schema"
                } else "INVALID_CHAIN"
                require(observed.get("status") == expected,
                        name + ": expected " + expected + ", observed " +
                        json.dumps(observed, sort_keys=True))
                expected_findings = {
                    "tampered-signature": "signature-invalid",
                    "wrong-root-trust-anchor": "root-trust-anchor-signature-invalid",
                    "wrong-root-issuer": "root-issuer-trust-anchor-mismatch",
                    "wrong-child-signing-key": "signature-invalid",
                    "wrong-par-hash": "par-hash-mismatch",
                    "wrong-derived-issuer": "issuer-thumbprint-mismatch",
                    "child-expiry-exceeds-parent": "child-expiry-exceeds-parent",
                    "depth-skips-parent": "delegation-depth-not-incremented-by-one",
                    "maximum-depth-expands": "maximum-depth-budget-expanded",
                    "duplicate-jti": "duplicate-jti",
                    "private-holder-key": "private-key-material-present",
                    "jwk-use-encryption": "jwk-use-invalid",
                    "jwk-algorithm-mismatch": "jwk-algorithm-mismatch",
                    "jwk-key-ops-missing-verify": "jwk-key-ops-invalid",
                    "jwk-key-ops-duplicate": "jwk-key-ops-invalid",
                    "jwk-key-ops-wrong-type": "jwk-key-ops-invalid",
                    "jwk-key-ops-unrelated-operation": "jwk-key-ops-invalid",
                    "duplicate-payload-member": "jti-preparse-duplicate",
                    "duplicate-nonjti-payload-member": "json-duplicate-member",
                    "none-algorithm": "algorithm-not-allowed",
                    "b64-header-present": "b64-header-not-allowed",
                    "critical-header-present": "critical-header-unsupported",
                    "wrong-token-type": "token-type-invalid",
                    "child-adds-tool": "tool-capability-expanded",
                    "constraint-expansion": "argument-constraint-expanded",
                    "argument-key-added": "argument-shape-changed",
                    "unknown-constraint-type": "constraint-type-unsupported",
                    "unhashable-header-typ": "token-type-invalid",
                    "malformed-compact-token": "compact-token-malformed",
                    "oversized-token": "token-size-exceeded",
                    "parent-token-reassociation": "par-hash-mismatch",
                    "duplicate-trust-anchor-issuer": "duplicate-trust-anchor-issuer",
                    "oversized-stack": "stack-size-exceeded",
                    "malformed-top-level": "malformed-top-level-input",
                    "unknown-schema": "unsupported-schema",
                }
                observed_finding = observed.get("findings", [{}])[0].get("code")
                require(observed_finding == expected_findings[name],
                        name + ": expected finding " + expected_findings[name] +
                        ", got " + str(observed_finding))
                receipt["controls"].append({
                    "id": name, "status": observed["status"],
                    "finding": observed.get("findings", [{}])[0].get("code"),
                })

            # Mutation-sensitivity controls: omitted security checks must cause
            # a known-bad, cryptographically or semantically invalid fixture
            # to be accepted by the mutated implementation.
            mutations: list[tuple[str, str, str, Callable[..., Any]]] = [
                ("signature-check-omitted", "tampered-signature", "verify_ed25519_signature",
                 lambda *args, **kwargs: None),
                ("issuer-thumbprint-check-omitted", "wrong-derived-issuer", "verify_issuer_thumbprint",
                 lambda issuer, parent_jwk: None),
                ("par-hash-check-omitted", "wrong-par-hash", "verify_parent_par_hash",
                 lambda claims, parent_token: None),
            ]
            for mutation_id, fixture_name, attribute, mutant in mutations:
                original = getattr(verifier, attribute)
                try:
                    setattr(verifier, attribute, mutant)
                    mutated_result = invoke_fixture(bad_cases[fixture_name], openssl)
                finally:
                    setattr(verifier, attribute, original)
                require(mutated_result.get("status") == "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
                        mutation_id + ": omitted-check mutant did not accept the known-bad fixture")
                receipt["mutations"].append({
                    "id": mutation_id,
                    "rejected_if_unmutated": True,
                    "mutant_acceptance_observed": True,
                    "independent_expected_rejection": fixture_name,
                })

            # Omitting all three JWK usage-metadata checks must accept each
            # known-invalid key profile, proving those checks are independently
            # observable rather than merely decorative metadata parsing.
            original_usage_check = verifier.validate_jwk_signature_usage
            bad_metadata_fixtures = (
                "jwk-use-encryption",
                "jwk-algorithm-mismatch",
                "jwk-key-ops-missing-verify",
                "jwk-key-ops-duplicate",
                "jwk-key-ops-wrong-type",
                "jwk-key-ops-unrelated-operation",
            )
            try:
                verifier.validate_jwk_signature_usage = lambda jwk: None
                for fixture_name in bad_metadata_fixtures:
                    mutated_result = invoke_fixture(bad_cases[fixture_name], openssl)
                    require(mutated_result.get("status") == "COMPACT_JWS_CRYPTO_LINKAGE_PASS",
                            "omitted JWK metadata mutant did not accept known-bad fixture " +
                            fixture_name + ": " + json.dumps(mutated_result, sort_keys=True))
            finally:
                verifier.validate_jwk_signature_usage = original_usage_check
            receipt["mutations"].append({
                "id": "jwk-usage-metadata-checks-omitted",
                "rejected_if_unmutated": True,
                "mutant_acceptance_observed": True,
                "independent_expected_rejections": list(bad_metadata_fixtures),
            })

            receipt["source_head"] = subprocess.run(
                ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
                check=True, timeout=15,
            ).stdout.strip()
            checker_path = HERE / "delegation_chain_compact_jws.py"
            receipt["checker_sha256"] = hashlib.sha256(checker_path.read_bytes()).hexdigest()
            receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
            receipt["status"] = "PASS"
            receipt["summary"] = {
                "positive_controls": 3,
                "negative_controls": sum(row["id"] not in {"valid-four-token-chain", "valid-single-token-chain", "valid-compatible-key-metadata-chain"} for row in receipt["controls"]),
                "signatures_verified": sum(row["token_count"] for row in receipt["controls"] if row["id"] in {"valid-four-token-chain", "valid-single-token-chain", "valid-compatible-key-metadata-chain"}),
                "mutants_detected": len(receipt["mutations"]),
                "signature_and_linkage_mutants_detected": len(receipt["mutations"]),
                "root_anchor_checks": True,
                "child_signature_checks": True,
                "issuer_thumbprint_and_par_hash_controls": True,
                "qualification": "NOT_CLAIMED",
            }
            args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
            print(f"COMPACT JWS CRYPTO-LINK CHECK PASS: 3 signed positives + {receipt['summary']['negative_controls']} negative controls")
            print("COMPACT JWS MUTATION SENSITIVITY PASS: 4 of 4 omitted checks accepted known-bad fixtures")
            print("QUALIFICATION NOT CLAIMED: capability subsumption and leaf proof-of-possession are not implemented")
            return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("COMPACT JWS CHAIN CHECK FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
