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


def build_chain(directory: Path, case: str = "valid-four-token-chain") -> dict[str, Any]:
    keys = [generate_keypair(directory, i) for i in range(5)]
    holder_jwks = [key["public_jwk"] for key in keys]
    chain: list[str] = []
    parent_signing_input: str | None = None
    root_iat = NOW - 200
    root_exp = NOW + 10_000

    for index in range(4):
        header: dict[str, Any] = {"alg": "EdDSA", "typ": "JWT", "kid": f"fixture-{index}"}
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
                "tools": {"read_file": {}},
            }],
        }
        if index:
            claims["par_hash"] = b64u(hashlib.sha256(parent_signing_input.encode("ascii")).digest())

        # Negative cases alter one invariant while keeping the affected token's
        # signature valid whenever the point of the test is a claim-level check.
        if case == "wrong-par-hash" and index == 1:
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
        elif case == "duplicate-payload-member" and index == 0:
            raw_payload = compact_json(claims).replace(b'"jti":"token-0"', b'"jti":"token-x","jti":"token-0"')
        else:
            raw_payload = None

        signer_index = index
        if case == "wrong-child-signing-key" and index == 1:
            signer_index = 2
        if case == "none-algorithm" and index == 1:
            header["alg"] = "none"

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
        original_root_header = {"alg": "EdDSA", "typ": "JWT", "kid": "fixture-0"}
        original_parent_input = (
            encode_segment(compact_json(original_root_header)) + "." + chain[0].split(".")[1]
        )
        claims["par_hash"] = b64u(hashlib.sha256(original_parent_input.encode("ascii")).digest())
        chain[1] = sign_token(keys[1]["private_path"], {"alg": "EdDSA", "typ": "JWT", "kid": "fixture-1"},
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
        "wrong-child-signing-key",
        "wrong-par-hash",
        "wrong-derived-issuer",
        "child-expiry-exceeds-parent",
        "depth-skips-parent",
        "maximum-depth-expands",
        "duplicate-jti",
        "private-holder-key",
        "duplicate-payload-member",
        "none-algorithm",
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
                   trusted_anchors: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    # Exclude any trust configuration from the token-chain input. Trusted keys
    # are supplied through a distinct caller-controlled parameter.
    anchors = trusted_anchors if trusted_anchors is not None else raw.get("trust_anchors")
    token_input = {key: raw[key] for key in ("schema", "now", "chain") if key in raw}
    return verifier.evaluate_compact_chain(token_input, anchors, openssl_binary=openssl)


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
            require(observed.get("status") == "COMPACT_JWS_CHAIN_VERIFIED",
                    "valid signed chain rejected: " + json.dumps(observed, sort_keys=True))
            receipt["controls"].append({
                "id": "valid-four-token-chain", "status": observed["status"],
                "independent_fixture": "OpenSSL Ed25519 signatures verified",
                "token_count": len(valid["chain"]),
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
                malicious_input, valid["trust_anchors"], openssl_binary=openssl
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
            bad_cases = cases(root)
            for name, raw in bad_cases.items():
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
                    "wrong-child-signing-key": "signature-invalid",
                    "wrong-par-hash": "par-hash-mismatch",
                    "wrong-derived-issuer": "issuer-thumbprint-mismatch",
                    "child-expiry-exceeds-parent": "child-expiry-exceeds-parent",
                    "depth-skips-parent": "delegation-depth-not-incremented-by-one",
                    "maximum-depth-expands": "maximum-depth-budget-expanded",
                    "duplicate-jti": "duplicate-jti",
                    "private-holder-key": "private-key-material-present",
                    "duplicate-payload-member": "json-duplicate-member",
                    "none-algorithm": "algorithm-not-allowed",
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
                require(mutated_result.get("status") == "COMPACT_JWS_CHAIN_VERIFIED",
                        mutation_id + ": omitted-check mutant did not accept the known-bad fixture")
                receipt["mutations"].append({
                    "id": mutation_id,
                    "rejected_if_unmutated": True,
                    "mutant_acceptance_observed": True,
                    "independent_expected_rejection": fixture_name,
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
                "positive_controls": 1,
                "negative_controls": len(receipt["controls"]) - 1,
                "signatures_verified": 4,
                "mutants_detected": len(receipt["mutations"]),
                "signature_and_linkage_mutants_detected": len(receipt["mutations"]),
                "root_anchor_checks": True,
                "child_signature_checks": True,
                "issuer_thumbprint_and_par_hash_controls": True,
                "qualification": "NOT_CLAIMED",
            }
            args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
            print(f"COMPACT JWS CHAIN CHECK PASS: 1 signed positive + {len(receipt['controls']) - 1} negative controls")
            print("COMPACT JWS MUTATION SENSITIVITY PASS: 3 of 3 omitted checks accepted known-bad fixtures")
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
