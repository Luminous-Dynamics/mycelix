#!/usr/bin/env python3
"""Execute EK X.509 path validation under a frozen, explicit OpenSSL policy."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-path-validation.v0.1"
PROFILE_ID = "mycelix.security.tpm.ek-cert-path-validation"
PROFILE_VERSION = "0.1.0"


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        c in "0123456789abcdef" for c in value
    )


def b64(value: bytes) -> str:
    return base64.b64encode(value).decode("ascii")


def unb64(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be base64")
    try:
        return base64.b64decode(value, validate=True)
    except Exception as exc:
        raise ValueError(f"{field} invalid base64: {exc}") from exc


def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out: dict[str, Any] = {
        "profile_id": PROFILE_ID,
        "profile_version": PROFILE_VERSION,
        "verifier_id": VERIFIER_ID,
        "state": state,
        "reason": reason,
    }
    if details is not None:
        out["details"] = details
    out["content_sha256"] = canonical_hash(
        {key: value for key, value in out.items() if key != "content_sha256"}
    )
    return out


def expected_input_binding(manifest: dict[str, Any]) -> str:
    return canonical_hash(
        {
            "session_id": manifest["session_id"],
            "tpm_identity_digest": manifest["tpm_identity_digest"],
            "leaf_certificate_sha256": manifest["leaf_certificate_sha256"],
            "intermediate_certificate_sha256": manifest["intermediate_certificate_sha256"],
            "trust_anchor_root_sha256": manifest["trust_anchor_root_sha256"],
            "crl_bundle_pem_sha256": manifest["crl_bundle_pem_sha256"],
            "verification_time_unix": manifest["verification_time_unix"],
            "policy_argv": [
                "openssl",
                "verify",
                "-CAfile",
                "root.pem",
                "-untrusted",
                "intermediate.pem",
                "-CRLfile",
                "crl-bundle.pem",
                "-crl_check_all",
                "-attime",
                str(manifest["verification_time_unix"]),
                "leaf.pem",
            ],
        }
    )


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id",
        "profile_version",
        "verification_mode",
        "claim_ceiling",
        "session_id",
        "tpm_identity_digest",
        "leaf_certificate_der_base64",
        "leaf_certificate_sha256",
        "intermediate_certificate_der_base64",
        "intermediate_certificate_sha256",
        "trust_anchor_root_der_base64",
        "trust_anchor_root_sha256",
        "crl_bundle_pem_base64",
        "crl_bundle_pem_sha256",
        "verification_time_unix",
        "execution_binding_sha256",
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})
    if manifest["profile_id"] != PROFILE_ID:
        return result("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != PROFILE_VERSION:
        return result("DENY", "profile-version-mismatch")
    if manifest["verification_mode"] not in {
        "ReferenceModelOnly",
        "OfflineBundle",
        "LiveVerifierSession",
    }:
        return result("DENY", "verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")
    if not isinstance(manifest["session_id"], str) or not manifest["session_id"]:
        return result("DENY", "session-id-invalid")
    if not isinstance(manifest["verification_time_unix"], int) or manifest["verification_time_unix"] < 0:
        return result("DENY", "verification-time-invalid")
    for field in (
        "tpm_identity_digest",
        "leaf_certificate_sha256",
        "intermediate_certificate_sha256",
        "trust_anchor_root_sha256",
        "crl_bundle_pem_sha256",
        "execution_binding_sha256",
    ):
        if not valid_hash(manifest[field]):
            return result("DENY", "digest-invalid", {"field": field})

    try:
        leaf = unb64(manifest["leaf_certificate_der_base64"], "leaf_certificate_der_base64")
        intermediate = unb64(
            manifest["intermediate_certificate_der_base64"],
            "intermediate_certificate_der_base64",
        )
        root = unb64(manifest["trust_anchor_root_der_base64"], "trust_anchor_root_der_base64")
        crl_bundle = unb64(manifest["crl_bundle_pem_base64"], "crl_bundle_pem_base64")
    except ValueError as exc:
        return result("DENY", "certificate-input-invalid", {"error": str(exc)})

    for raw, field in (
        (leaf, "leaf_certificate_sha256"),
        (intermediate, "intermediate_certificate_sha256"),
        (root, "trust_anchor_root_sha256"),
        (crl_bundle, "crl_bundle_pem_sha256"),
    ):
        if hashlib.sha256(raw).hexdigest() != manifest[field]:
            return result("DENY", "digest-mismatch", {"field": field})

    policy_argv = [
        "openssl",
        "verify",
        "-CAfile",
        "root.pem",
        "-untrusted",
        "intermediate.pem",
        "-CRLfile",
        "crl-bundle.pem",
        "-crl_check_all",
        "-attime",
        str(manifest["verification_time_unix"]),
        "leaf.pem",
    ]
    expected_binding = expected_input_binding(manifest)
    if manifest["execution_binding_sha256"] != expected_binding:
        return result("DENY", "execution-binding-mismatch")

    openssl = shutil.which("openssl")
    if not openssl:
        return result("INDETERMINATE", "openssl-unavailable")

    with tempfile.TemporaryDirectory(prefix="mycelix-ek-path-") as td:
        work = Path(td)
        # Encode the exact supplied DER bytes directly in PEM. OpenSSL therefore
        # parses the same DER object whose digest was bound above, rather than a
        # potentially re-serialized certificate emitted by an intermediate step.
        for name, raw in (
            ("leaf", leaf),
            ("intermediate", intermediate),
            ("root", root),
        ):
            der = work / f"{name}.der"
            pem = work / f"{name}.pem"
            der.write_bytes(raw)
            pem.write_bytes(
                b"-----BEGIN CERTIFICATE-----\n"
                + base64.b64encode(raw)
                + b"\n-----END CERTIFICATE-----\n"
            )
            pem_lines = pem.read_bytes().splitlines()
            if len(pem_lines) != 3 or base64.b64decode(pem_lines[1], validate=True) != raw:
                return result("DENY", f"{name}-pem-der-binding-failed")

        (work / "crl-bundle.pem").write_bytes(crl_bundle)


def self_test() -> int:
    root = Path(__file__).resolve().parents[2]
    generator = root / "scripts/security/generate_mycelix_ek_chain_fixtures_v0_1.py"
    recipe = root / "docs/security/fixtures/ek-chain-policy-v0.1/fixture-recipe-v0.1.json"
    if not generator.is_file() or not recipe.is_file():
        print("EK path-validation generator provenance: FAIL")
        return 1

    with tempfile.TemporaryDirectory(prefix="mycelix-ek-path-fixtures-") as td:
        fixture = Path(td)
        proc = subprocess.run(
            [sys.executable, str(generator), "--recipe", str(recipe), "--output-dir", str(fixture), "--check"],
            cwd=fixture, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        if proc.returncode != 0:
            print("EK path-validation deterministic fixture generation: FAIL")
            print(proc.stderr or proc.stdout)
            return 1

        paths = {
            "leaf": fixture / "leaf.der",
            "intermediate": fixture / "intermediate.der",
            "root": fixture / "root.der",
            "crl": fixture / "crl-bundle.pem",
        }
        if not all(path.is_file() for path in paths.values()):
            print("EK path-validation generated fixture set: FAIL")
            return 1
        values = {name: path.read_bytes() for name, path in paths.items()}

        manifest = {
            "profile_id": PROFILE_ID,
            "profile_version": PROFILE_VERSION,
            "verification_mode": "ReferenceModelOnly",
            "claim_ceiling": "ReferenceModelOnly",
            "session_id": "ek-path-self-test",
            "tpm_identity_digest": "55" * 32,
            "leaf_certificate_der_base64": b64(values["leaf"]),
            "leaf_certificate_sha256": hashlib.sha256(values["leaf"]).hexdigest(),
            "intermediate_certificate_der_base64": b64(values["intermediate"]),
            "intermediate_certificate_sha256": hashlib.sha256(values["intermediate"]).hexdigest(),
            "trust_anchor_root_der_base64": b64(values["root"]),
            "trust_anchor_root_sha256": hashlib.sha256(values["root"]).hexdigest(),
            "crl_bundle_pem_base64": b64(values["crl"]),
            "crl_bundle_pem_sha256": hashlib.sha256(values["crl"]).hexdigest(),
            "verification_time_unix": 1791158400,
        }
        manifest["execution_binding_sha256"] = expected_input_binding(manifest)
        observed = verify(manifest)
        if observed["state"] not in {"PASS", "DENY", "INDETERMINATE"}:
            print("canonical path-validation state: FAIL")
            return 1

        tampered = dict(manifest)
        tampered["execution_binding_sha256"] = "aa" * 32
        if verify(tampered)["state"] != "DENY":
            print("execution-binding substitution: FAIL")
            return 1

        tampered = dict(manifest)
        tampered["leaf_certificate_sha256"] = "bb" * 32
        if verify(tampered)["state"] != "DENY":
            print("certificate digest substitution: FAIL")
            return 1

        tampered = dict(manifest)
        tampered["verification_time_unix"] += 1
        if verify(tampered)["state"] != "DENY":
            print("verification-time binding substitution: FAIL")
            return 1

        if observed["state"] == "PASS":
            print("EK certificate path-validation semantic corpus: PASS")
            print("explicit OpenSSL full-chain CRL policy: PASS")
            print("exact DER-to-OpenSSL PEM binding: PASS")
            return 0
        if observed["state"] == "INDETERMINATE":
            print("EK certificate path-validation semantic corpus: INDETERMINATE")
            return 2
        print("EK certificate path-validation semantic corpus: FAIL")
        print(observed["reason"])
        return 1

def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--verify", metavar="MANIFEST")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.output:
        parser.error("--output is required with --verify")
    manifest_path = Path(args.verify).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit("manifest must be a JSON object")
    verified = verify(manifest)
    output = {
        **verified,
        "input_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
    }
    output["content_sha256"] = canonical_hash(
        {key: value for key, value in output.items() if key != "content_sha256"}
    )
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "DENY": 1, "INDETERMINATE": 2}[verified["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
