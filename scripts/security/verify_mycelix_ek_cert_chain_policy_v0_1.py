#!/usr/bin/env python3
"""Verify an EK X.509 certificate path under an explicit reference policy."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import shutil
import sys
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-chain-policy.v0.1"
SPKI_VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
TEMPLATE_VERIFIER_ID = "mycelix.tpm.ek-template-appraisal.v0.1"
TEMPLATE_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_template_appraisal_v0_1.py")
EK_CERT_EKU_OID = "2.23.133.8.1"
REFERENCE_ROOT_SOURCE_TAG = "synthetic-ek-root-fixture-v0.1"
REFERENCE_ROOT_SHA256 = "bf027c125d7641b37bb1e8e71eab5f480cc3b925eb33660b818b251faf5fa6bc"


def reference_root_source_hash(root_sha256: str) -> str:
    return canonical_hash(
        {
            "source_tag": REFERENCE_ROOT_SOURCE_TAG,
            "root_sha256": root_sha256,
        }
    )


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


REFERENCE_ROOT_SOURCE_SHA256 = hashlib.sha256(
    json.dumps(
        {
            "source_tag": REFERENCE_ROOT_SOURCE_TAG,
            "root_sha256": REFERENCE_ROOT_SHA256,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        c in "0123456789abcdef" for c in value
    )


def b64(value: bytes) -> str:
    import base64
    return base64.b64encode(value).decode("ascii")


def unb64(value: Any, field: str) -> bytes:
    import base64
    if not isinstance(value, str):
        raise ValueError(f"{field} must be base64")
    try:
        return base64.b64decode(value, validate=True)
    except Exception as exc:
        raise ValueError(f"{field} invalid base64: {exc}") from exc


def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
    )


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_template_verifier(binding: dict[str, Any]) -> dict[str, Any]:
    verifier_input = binding.get("verifier_input")
    if not isinstance(verifier_input, dict):
        return result("DENY", "ek-template-verifier-input-invalid")
    if not TEMPLATE_VERIFIER_SCRIPT.is_file():
        return result("DENY", "ek-template-verifier-missing")
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-template-") as td:
        work = Path(td)
        input_path = work / "template-input.json"
        output_path = work / "template-output.json"
        input_path.write_text(
            json.dumps(verifier_input, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(TEMPLATE_VERIFIER_SCRIPT),
                "--verify",
                str(input_path),
                "--output",
                str(output_path),
            ],
            cwd=work,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode not in (0, 1, 2):
            return result("DENY", "ek-template-verifier-execution-error", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "ek-template-verifier-produced-no-output")
        try:
            output = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "ek-template-verifier-output-invalid", {"error": str(exc)})
        if output.get("verifier_id") != TEMPLATE_VERIFIER_ID:
            return result("DENY", "ek-template-verifier-id-mismatch")
        expected_input_sha = sha256_file(input_path)
        if output.get("input_sha256") != expected_input_sha:
            return result("DENY", "ek-template-verifier-input-digest-mismatch")
        if binding.get("input_sha256") != expected_input_sha:
            return result("DENY", "ek-template-binding-input-digest-mismatch")
        expected_output_sha = sha256_file(output_path)
        if binding.get("output_sha256") != expected_output_sha:
            return result("DENY", "ek-template-verifier-output-digest-mismatch")
        return output


def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details is not None:
        out["details"] = details
    return out


def x509_text(der: bytes, work: Path, name: str) -> str:
    path = work / f"{name}.der"
    path.write_bytes(der)
    proc = run(["openssl", "x509", "-inform", "DER", "-in", str(path), "-noout", "-text"], work)
    if proc.returncode != 0:
        raise ValueError(f"openssl x509 parse failed: {proc.stderr.strip()}")
    return proc.stdout


def x509_scalar(der: bytes, work: Path, name: str, flag: str) -> str:
    path = work / f"{name}-scalar.der"
    path.write_bytes(der)
    proc = run(["openssl", "x509", "-inform", "DER", "-in", str(path), "-noout", flag], work)
    if proc.returncode != 0:
        raise ValueError(f"openssl x509 {flag} failed: {proc.stderr.strip()}")
    line = proc.stdout.strip()
    return line.split("=", 1)[1].strip() if "=" in line else line


def crl_scalar(der: bytes, work: Path, name: str, flag: str) -> str:
    path = work / f"{name}-crl.der"
    path.write_bytes(der)
    proc = run(["openssl", "crl", "-inform", "DER", "-in", str(path), "-noout", "-nameopt", "RFC2253", flag], work)
    if proc.returncode != 0:
        raise ValueError(f"openssl crl {flag} failed: {proc.stderr.strip()}")
    line = proc.stdout.strip()
    return line.split("=", 1)[1].strip() if "=" in line else line


def extension(text: str, name: str) -> tuple[bool, str | None]:
    lines = text.splitlines()
    wanted = f"X509v3 {name}"
    for index, line in enumerate(lines):
        if wanted.lower() in line.lower():
            critical = "critical" in line.lower()
            for value_line in lines[index + 1 : index + 4]:
                stripped = value_line.strip()
                if stripped and not stripped.startswith("X509v3 "):
                    return critical, stripped
    return False, None


def parse_key_id(value: str | None) -> str:
    if not value:
        return ""
    lower = value.lower()
    if "keyid:" in lower:
        value = value[lower.index("keyid:") + len("keyid:") :]
    value = value.replace(":", "").replace(" ", "")
    return value.lower()


def verify_aki_ski(leaf_text: str, intermediate_text: str) -> bool:
    _aki_critical, aki = extension(leaf_text, "Authority Key Identifier")
    _ski_critical, ski = extension(intermediate_text, "Subject Key Identifier")
    return bool(parse_key_id(aki) and parse_key_id(ski) and parse_key_id(aki) == parse_key_id(ski))


def verify_chain(
    leaf: bytes,
    intermediate: bytes,
    root: bytes,
    crl: bytes,
    attime: int,
    work: Path,
) -> tuple[bool, str]:
    paths = {
        "leaf.der": leaf,
        "inter.der": intermediate,
        "root.der": root,
        "crl.der": crl,
    }
    for name, data in paths.items():
        (work / name).write_bytes(data)

    conversions = [
        ["openssl", "x509", "-inform", "DER", "-in", str(work / "leaf.der"), "-out", str(work / "leaf.pem")],
        ["openssl", "x509", "-inform", "DER", "-in", str(work / "inter.der"), "-out", str(work / "inter.pem")],
        ["openssl", "x509", "-inform", "DER", "-in", str(work / "root.der"), "-out", str(work / "root.pem")],
        ["openssl", "crl", "-inform", "DER", "-in", str(work / "crl.der"), "-out", str(work / "crl.pem")],
    ]
    for command in conversions:
        proc = run(command, work)
        if proc.returncode != 0:
            return False, proc.stderr.strip()

    proc = run(
        [
            "openssl", "verify",
            "-CAfile", str(work / "root.pem"),
            "-untrusted", str(work / "inter.pem"),
            "-x509_strict",
            "-check_ss_sig",
            "-crl_check",
            "-CRLfile", str(work / "crl.pem"),
            "-attime", str(attime),
            str(work / "leaf.pem"),
        ],
        work,
    )
    return proc.returncode == 0, (proc.stdout + proc.stderr).strip()


def leaf_profile_ok(text: str) -> tuple[bool, dict[str, Any]]:
    version_ok = "Version: 3 (0x2)" in text
    basic_critical, basic = extension(text, "Basic Constraints")
    usage_critical, usage = extension(text, "Key Usage")
    eku_critical, eku = extension(text, "Extended Key Usage")
    aki_critical, aki = extension(text, "Authority Key Identifier")
    profile = {
        "version_3": version_ok,
        "basic_constraints_critical": basic_critical,
        "basic_constraints": basic,
        "key_usage_critical": usage_critical,
        "key_usage": usage,
        "extended_key_usage": eku,
        "extended_key_usage_critical": eku_critical,
        "authority_key_identifier": aki,
        "authority_key_identifier_critical": aki_critical,
    }
    eku_ok = eku is None or EK_CERT_EKU_OID in eku or "Endorsement Key Certificate" in eku
    ok = (
        version_ok
        and basic_critical
        and basic is not None
        and basic.upper() == "CA:FALSE"
        and usage_critical
        and usage is not None
        and "Key Encipherment" in usage
        and eku_ok
        and parse_key_id(aki) != ""
    )
    return ok, profile


def validate_template_binding(manifest: dict[str, Any]) -> dict[str, Any]:
    template = manifest.get("ek_template_binding")
    if not isinstance(template, dict):
        return result("DENY", "ek-template-binding-invalid")
    for field in (
        "state", "verifier_id", "input_sha256", "output_sha256",
        "public_wire_sha256", "source_sha256", "template_id", "verifier_input",
    ):
        if field not in template:
            return result("DENY", "ek-template-binding-field-missing", {"field": field})
    if template["verifier_id"] != TEMPLATE_VERIFIER_ID:
        return result("DENY", "ek-template-verifier-id-mismatch")
    if template["state"] == "INDETERMINATE":
        return result("INDETERMINATE", "ek-template-binding-indeterminate")
    if template["state"] != "PASS":
        return result("DENY", "ek-template-binding-not-pass")
    for field in ("input_sha256", "output_sha256", "public_wire_sha256", "source_sha256"):
        if not valid_hash(template[field]):
            return result("DENY", "ek-template-binding-digest-invalid", {"field": field})
    if template["source_sha256"] != sha256_file(TEMPLATE_VERIFIER_SCRIPT):
        return result("DENY", "ek-template-verifier-source-mismatch")
    if template["template_id"] != "L-1":
        return result("DENY", "ek-template-id-mismatch")
    generated_template = run_template_verifier(template)
    if generated_template.get("verifier_id") != TEMPLATE_VERIFIER_ID:
        return generated_template
    if generated_template.get("state") != "PASS":
        return result("DENY", "ek-template-reexecution-not-pass")
    details = generated_template.get("details")
    if not isinstance(details, dict):
        return result("DENY", "ek-template-result-details-missing")
    if details.get("public_wire_sha256") != template["public_wire_sha256"]:
        return result("DENY", "ek-template-public-wire-result-mismatch")
    if template["public_wire_sha256"] != manifest["ek_public_wire_sha256"]:
        return result("DENY", "ek-template-public-wire-digest-mismatch")
    generated_input = template["verifier_input"]
    if generated_input.get("public_wire_sha256") != manifest["ek_public_wire_sha256"]:
        return result("DENY", "ek-template-input-public-wire-mismatch")
    return generated_template


def session_binding(
    manifest: dict[str, Any],
    leaf_sha: str,
    intermediate_sha: str,
    root_sha: str,
    crl_sha: str,
) -> str:
    rev = manifest["revocation"]
    spki = manifest["spki_binding"]
    template = manifest["ek_template_binding"]
    return canonical_hash(
        {
            "verification_mode": manifest["verification_mode"],
            "session_id": manifest["session_id"],
            "tpm_identity_digest": manifest["tpm_identity_digest"],
            "ek_public_wire_sha256": manifest["ek_public_wire_sha256"],
            "leaf_certificate_sha256": leaf_sha,
            "intermediate_certificate_sha256": intermediate_sha,
            "trust_anchor_root_sha256": root_sha,
            "trust_anchor_source_sha256": manifest["trust_anchor_source_sha256"],
            "trust_anchor_state": manifest["trust_anchor_state"],
            "verification_time_unix": manifest["verification_time_unix"],
            "revocation_state": rev["state"],
            "revocation_method": rev.get("method"),
            "revocation_crl_sha256": crl_sha,
            "spki_state": spki.get("state"),
            "spki_certificate_sha256": spki.get("certificate_sha256"),
            "spki_ek_public_wire_sha256": spki.get("ek_public_wire_sha256"),
            "template_state": template.get("state"),
            "template_input_sha256": template.get("input_sha256"),
            "template_output_sha256": template.get("output_sha256"),
            "template_public_wire_sha256": template.get("public_wire_sha256"),
            "template_source_sha256": template.get("source_sha256"),
            "template_id": template.get("template_id"),
        }
    )

def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id", "profile_version", "verification_mode", "claim_ceiling",
        "session_id", "tpm_identity_digest", "ek_public_wire_sha256",
        "leaf_certificate_der_base64", "leaf_certificate_sha256",
        "intermediate_certificate_der_base64", "intermediate_certificate_sha256",
        "trust_anchor_root_der_base64", "trust_anchor_root_sha256",
        "trust_anchor_state", "trust_anchor_source_sha256",
        "verification_time_unix", "revocation", "spki_binding", "ek_template_binding",
        "session_binding_sha256",
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})
    if manifest["profile_id"] != "mycelix.security.tpm.ek-cert-chain-policy":
        return result("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY", "profile-version-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly", "OfflineBundle", "LiveVerifierSession"}:
        return result("DENY", "verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")
    if not isinstance(manifest["session_id"], str) or not manifest["session_id"]:
        return result("DENY", "session-id-invalid")
    for field in ("tpm_identity_digest", "ek_public_wire_sha256", "leaf_certificate_sha256", "intermediate_certificate_sha256",
                  "trust_anchor_root_sha256", "trust_anchor_source_sha256", "session_binding_sha256"):
        if not valid_hash(manifest[field]):
            return result("DENY", "digest-invalid", {"field": field})

    try:
        leaf = unb64(manifest["leaf_certificate_der_base64"], "leaf_certificate_der_base64")
        intermediate = unb64(manifest["intermediate_certificate_der_base64"], "intermediate_certificate_der_base64")
        root = unb64(manifest["trust_anchor_root_der_base64"], "trust_anchor_root_der_base64")
        rev = manifest["revocation"]
        if not isinstance(rev, dict):
            return result("DENY", "revocation-object-invalid")
        crl = unb64(rev.get("crl_der_base64", ""), "revocation.crl_der_base64")
    except ValueError as exc:
        return result("DENY", "certificate-input-invalid", {"error": str(exc)})

    for raw, field in (
        (leaf, "leaf_certificate_sha256"),
        (intermediate, "intermediate_certificate_sha256"),
        (root, "trust_anchor_root_sha256"),
    ):
        if hashlib.sha256(raw).hexdigest() != manifest[field]:
            return result("DENY", "digest-mismatch", {"field": field})

    if manifest["trust_anchor_state"] == "DENY":
        return result("DENY", "trust-anchor-denied")
    if manifest["trust_anchor_state"] == "INDETERMINATE":
        return result("INDETERMINATE", "trust-anchor-indeterminate")
    if manifest["trust_anchor_source_sha256"] != reference_root_source_hash(manifest["trust_anchor_root_sha256"]):
        return result("DENY", "trust-anchor-source-not-authorized-for-root")

    if manifest["verification_time_unix"] < 0:
        return result("DENY", "verification-time-invalid")
    if rev.get("state") == "DENY":
        return result("DENY", "ek-certificate-revoked")
    if rev.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "ek-certificate-revocation-indeterminate")
    if rev.get("state") != "PASS":
        return result("DENY", "revocation-state-invalid")
    if rev.get("method") != "issuer-crl":
        return result("DENY", "revocation-method-invalid")
    if not valid_hash(rev.get("crl_der_sha256")):
        return result("DENY", "revocation-crl-digest-invalid")
    if hashlib.sha256(crl).hexdigest() != rev["crl_der_sha256"]:
        return result("DENY", "revocation-crl-digest-mismatch")

    spki = manifest["spki_binding"]
    if not isinstance(spki, dict):
        return result("DENY", "spki-binding-invalid")
    if spki.get("verifier_id") != SPKI_VERIFIER_ID:
        return result("DENY", "spki-verifier-id-mismatch")
    if spki.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "spki-binding-indeterminate")
    if spki.get("state") != "PASS":
        return result("DENY", "spki-binding-not-pass")
    if not valid_hash(spki.get("certificate_sha256")) or not valid_hash(spki.get("ek_public_wire_sha256")):
        return result("DENY", "spki-binding-digest-invalid")
    if spki["certificate_sha256"] != manifest["leaf_certificate_sha256"]:
        return result("DENY", "spki-certificate-digest-mismatch")
    if spki["ek_public_wire_sha256"] != manifest["ek_public_wire_sha256"]:
        return result("DENY", "spki-ek-public-wire-digest-mismatch")

    template_result = validate_template_binding(manifest)
    if template_result.get("state") != "PASS":
        return template_result

    expected_session_binding = session_binding(
        manifest,
        manifest["leaf_certificate_sha256"],
        manifest["intermediate_certificate_sha256"],
        manifest["trust_anchor_root_sha256"],
        rev["crl_der_sha256"],
    )
    if expected_session_binding != manifest["session_binding_sha256"]:
        return result("DENY", "session-binding-mismatch")

    if not shutil.which("openssl"):
        return result("INDETERMINATE", "openssl-unavailable")

    with tempfile.TemporaryDirectory(prefix="mycelix-ek-chain-") as td:
        work = Path(td)
        try:
            chain_ok, chain_detail = verify_chain(
                leaf, intermediate, root, crl, manifest["verification_time_unix"], work
            )
            leaf_text = x509_text(leaf, work, "leaf-profile")
            intermediate_text = x509_text(intermediate, work, "intermediate-profile")
            serial = int(x509_scalar(leaf, work, "leaf", "-serial"), 16)
            subject = x509_scalar(leaf, work, "leaf-subject", "-subject")
            issuer = x509_scalar(leaf, work, "leaf-issuer", "-issuer")
            intermediate_subject = x509_scalar(intermediate, work, "intermediate-subject", "-subject")
            crl_issuer = crl_scalar(crl, work, "issuer", "-issuer")
            openssl_version = run(["openssl", "version"], work).stdout.strip()
        except (ValueError, OSError) as exc:
            return result("DENY", "openssl-parse-error", {"error": str(exc)})

    if not chain_ok:
        return result("DENY", "certificate-path-validation-failed", {"openssl": chain_detail})

    profile_ok, profile = leaf_profile_ok(leaf_text)
    profile["serial_positive"] = serial > 0
    profile["subject"] = subject
    profile["issuer"] = issuer
    profile["openssl_version"] = openssl_version
    if serial <= 0:
        return result("DENY", "leaf-serial-invalid", profile)
    if not profile_ok:
        return result("DENY", "ek-leaf-profile-requirements-failed", profile)
    if issuer != intermediate_subject:
        return result(
            "DENY",
            "leaf-issuer-does-not-match-intermediate-subject",
            {**profile, "intermediate_subject": intermediate_subject},
        )
    if crl_issuer != intermediate_subject:
        return result(
            "DENY",
            "crl-issuer-does-not-match-intermediate-subject",
            {**profile, "crl_issuer": crl_issuer, "intermediate_subject": intermediate_subject},
        )
    if not verify_aki_ski(leaf_text, intermediate_text):
        return result("DENY", "authority-key-identifier-does-not-match-intermediate-ski", profile)

    if manifest["verification_mode"] != "ReferenceModelOnly":
        return result("INDETERMINATE", "live-origin-not-authorized-by-reference-model", profile)

    return result(
        "PASS",
        "ek-certificate-chain-and-profile-policy-verified",
        {
            **profile,
            "trust_anchor_sha256": manifest["trust_anchor_root_sha256"],
            "trust_anchor_source_sha256": manifest["trust_anchor_source_sha256"],
            "verification_time_unix": manifest["verification_time_unix"],
            "revocation_state": rev["state"],
            "revocation_crl_sha256": rev["crl_der_sha256"],
            "spki_certificate_sha256": spki["certificate_sha256"],
            "spki_ek_public_wire_sha256": spki["ek_public_wire_sha256"],
        },
    )


def openssl_fixture(work: Path) -> dict[str, Any]:
    root_key = work / "root.key"
    inter_key = work / "inter.key"
    leaf_key = work / "leaf.key"
    root_pem = work / "root.pem"
    inter_pem = work / "inter.pem"
    leaf_pem = work / "leaf.pem"
    leaf_bad_usage_pem = work / "leaf-bad-usage.pem"
    leaf_bad_eku_pem = work / "leaf-bad-eku.pem"

    run(["openssl", "genrsa", "-traditional", "-out", str(root_key), "2048"], work)
    p = run([
        "openssl", "req", "-new", "-x509", "-sha256", "-days", "3650",
        "-key", str(root_key), "-subj", "/CN=Mycelix Synthetic EK Root CA",
        "-addext", "basicConstraints=critical,CA:true,pathlen:1",
        "-addext", "keyUsage=critical,keyCertSign,cRLSign",
        "-addext", "subjectKeyIdentifier=hash",
        "-out", str(root_pem),
    ], work)
    if p.returncode != 0:
        raise RuntimeError(p.stderr)

    run(["openssl", "genrsa", "-traditional", "-out", str(inter_key), "2048"], work)
    p = run(["openssl", "req", "-new", "-sha256", "-key", str(inter_key),
             "-subj", "/CN=Mycelix Synthetic EK Issuing CA", "-out", str(work / "inter.csr")], work)
    if p.returncode != 0:
        raise RuntimeError(p.stderr)
    (work / "inter.ext").write_text(
        "basicConstraints=critical,CA:true,pathlen:0\n"
        "keyUsage=critical,keyCertSign,cRLSign\n"
        "subjectKeyIdentifier=hash\n"
        "authorityKeyIdentifier=keyid,issuer\n",
        encoding="utf-8",
    )
    p = run(["openssl", "x509", "-req", "-sha256", "-days", "2555",
             "-in", str(work / "inter.csr"), "-CA", str(root_pem), "-CAkey", str(root_key),
             "-CAcreateserial", "-extfile", str(work / "inter.ext"), "-out", str(inter_pem)], work)
    if p.returncode != 0:
        raise RuntimeError(p.stderr)

    run(["openssl", "genrsa", "-traditional", "-out", str(leaf_key), "2048"], work)
    p = run(["openssl", "req", "-new", "-sha256", "-key", str(leaf_key),
             "-subj", "/CN=Mycelix Synthetic EK/O=Mycelix Reference Lab/OU=EK",
             "-out", str(work / "leaf.csr")], work)
    if p.returncode != 0:
        raise RuntimeError(p.stderr)
    (work / "leaf.ext").write_text(
        "basicConstraints=critical,CA:false\n"
        "keyUsage=critical,keyEncipherment\n"
        f"extendedKeyUsage=OID.{EK_CERT_EKU_OID}\n"
        "subjectKeyIdentifier=hash\n"
        "authorityKeyIdentifier=keyid,issuer\n",
        encoding="utf-8",
    )
    (work / "bad-usage.ext").write_text(
        "basicConstraints=critical,CA:false\n"
        "keyUsage=critical,digitalSignature\n"
        f"extendedKeyUsage=OID.{EK_CERT_EKU_OID}\n"
        "subjectKeyIdentifier=hash\n"
        "authorityKeyIdentifier=keyid,issuer\n",
        encoding="utf-8",
    )
    (work / "bad-eku.ext").write_text(
        "basicConstraints=critical,CA:false\n"
        "keyUsage=critical,keyEncipherment\n"
        "extendedKeyUsage=clientAuth\n"
        "subjectKeyIdentifier=hash\n"
        "authorityKeyIdentifier=keyid,issuer\n",
        encoding="utf-8",
    )
    for ext_path, out_path in (
        ("leaf.ext", leaf_pem),
        ("bad-usage.ext", leaf_bad_usage_pem),
        ("bad-eku.ext", leaf_bad_eku_pem),
    ):
        p = run([
            "openssl", "x509", "-req", "-sha256", "-days", "3650",
            "-in", str(work / "leaf.csr"), "-CA", str(inter_pem), "-CAkey", str(inter_key),
            "-CAcreateserial", "-extfile", str(work / ext_path), "-out", str(out_path),
        ], work)
        if p.returncode != 0:
            raise RuntimeError(p.stderr)

    ca_dir = work / "ca"
    ca_dir.mkdir()
    (ca_dir / "index.txt").write_text("", encoding="utf-8")
    (ca_dir / "serial").write_text("1000\n", encoding="utf-8")
    (ca_dir / "crlnumber").write_text("1000\n", encoding="utf-8")
    (ca_dir / "openssl.cnf").write_text(
        "[ca]\ndefault_ca=ca_default\n"
        "[ca_default]\n"
        f"database={ca_dir / 'index.txt'}\n"
        f"private_key={inter_key}\n"
        f"certificate={inter_pem}\n"
        f"serial={ca_dir / 'serial'}\n"
        f"crlnumber={ca_dir / 'crlnumber'}\n"
        f"crl={ca_dir / 'crl.pem'}\n"
        "default_crl_days=30\ndefault_md=sha256\npolicy=policy_any\n"
        "[policy_any]\ncommonName=supplied\n",
        encoding="utf-8",
    )
    p = run(["openssl", "ca", "-config", str(ca_dir / "openssl.cnf"), "-gencrl",
             "-out", str(ca_dir / "crl.pem"), "-batch"], work)
    if p.returncode != 0:
        raise RuntimeError(p.stderr)

    def der(path: Path) -> bytes:
        out = path.with_suffix(".der")
        cmd = ["openssl", "x509", "-in", str(path), "-outform", "DER", "-out", str(out)]
        if path.name == "crl.pem":
            cmd = ["openssl", "crl", "-in", str(path), "-outform", "DER", "-out", str(out)]
        p = run(cmd, work)
        if p.returncode != 0:
            raise RuntimeError(p.stderr)
        return out.read_bytes()

    root = der(root_pem)
    intermediate = der(inter_pem)
    leaf = der(leaf_pem)
    bad_usage = der(leaf_bad_usage_pem)
    bad_eku = der(leaf_bad_eku_pem)
    crl = der(ca_dir / "crl.pem")

    texts = {}
    for name, data in (("leaf", leaf), ("bad-usage", bad_usage), ("bad-eku", bad_eku)):
        texts[name] = x509_text(data, work, f"fixture-{name}")

    start = int(time.time()) - 60
    return {
        "root": root,
        "intermediate": intermediate,
        "leaf": leaf,
        "bad_usage": bad_usage,
        "bad_eku": bad_eku,
        "crl": crl,
        "attime": start + 120,
    }


def make_manifest(fx: dict[str, Any]) -> dict[str, Any]:
    leaf_sha = hashlib.sha256(fx["leaf"]).hexdigest()
    inter_sha = hashlib.sha256(fx["intermediate"]).hexdigest()
    root_sha = hashlib.sha256(fx["root"]).hexdigest()
    crl_sha = hashlib.sha256(fx["crl"]).hexdigest()
    m = {
        "profile_id": "mycelix.security.tpm.ek-cert-chain-policy",
        "profile_version": "0.1.0",
        "verification_mode": "ReferenceModelOnly",
        "claim_ceiling": "ReferenceModelOnly",
        "session_id": "ek-chain-self-test",
        "tpm_identity_digest": "44" * 32,
        "ek_public_wire_sha256": hashlib.sha256(
            bytes.fromhex("0001000b000300b2")
            + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
            + bytes.fromhex("00060080004300100800")
            + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
        ).hexdigest(),
        "leaf_certificate_der_base64": b64(fx["leaf"]),
        "leaf_certificate_sha256": leaf_sha,
        "intermediate_certificate_der_base64": b64(fx["intermediate"]),
        "intermediate_certificate_sha256": inter_sha,
        "trust_anchor_root_der_base64": b64(fx["root"]),
        "trust_anchor_root_sha256": root_sha,
        "trust_anchor_state": "PASS",
        "trust_anchor_source_sha256": reference_root_source_hash(root_sha),
        "verification_time_unix": fx["attime"],
        "revocation": {
            "state": "PASS",
            "method": "issuer-crl",
            "crl_der_base64": b64(fx["crl"]),
            "crl_der_sha256": crl_sha,
        },
        "ek_template_binding": {
            "state": "PASS",
            "verifier_id": TEMPLATE_VERIFIER_ID,
            "input_sha256": "",
            "output_sha256": "",
            "public_wire_sha256": hashlib.sha256(
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
            ).hexdigest(),
            "source_sha256": sha256_file(TEMPLATE_VERIFIER_SCRIPT),
            "template_id": "L-1",
            "verifier_input": {
                "profile_id": "mycelix.security.tpm.ek-template-appraisal",
                "profile_version": "0.1.0",
                "verification_mode": "ReferenceModelOnly",
                "claim_ceiling": "ReferenceModelOnly",
                "object_role": "EK",
                "public_format": "TPMT_PUBLIC",
                "public_wire_hex": (
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
            ).hex(),
                "public_wire_sha256": hashlib.sha256(
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
            ).hexdigest(),
                "name_hex": (SHA256_ALG_ID + hashlib.sha256(
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
            ).digest()).hex(),
                "qualified_name_hex": (SHA256_ALG_ID + hashlib.sha256(b"template-qname" + (
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + bytes(256)
            )).digest()).hex(),
                "creation_provenance": {
                    "state": "PASS",
                    "tool_id": "tpm2_createek",
                    "hierarchy": "TPM_RH_ENDORSEMENT",
                    "template_mode": "default-low-range",
                    "transcript_sha256": "ee" * 32
                }
            }
        },
        "spki_binding": {
            "state": "PASS",
            "verifier_id": SPKI_VERIFIER_ID,
            "certificate_sha256": leaf_sha,
            "ek_public_wire_sha256": m["ek_public_wire_sha256"],
        },
    }
    m["session_binding_sha256"] = session_binding(m, leaf_sha, inter_sha, root_sha, crl_sha)
    return m


def refresh_template_binding(m: dict[str, Any]) -> None:
    binding = m["ek_template_binding"]
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-template-refresh-") as td:
        work = Path(td)
        ip = work / "input.json"
        op = work / "output.json"
        ip.write_text(json.dumps(binding["verifier_input"], indent=2, sort_keys=True) + "\n", encoding="utf-8")
        subprocess.run(
            [sys.executable, str(TEMPLATE_VERIFIER_SCRIPT), "--verify", str(ip), "--output", str(op)],
            cwd=work, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        binding["input_sha256"] = hashlib.sha256(ip.read_bytes()).hexdigest()
        binding["output_sha256"] = hashlib.sha256(op.read_bytes()).hexdigest()
        output=json.loads(op.read_text(encoding="utf-8"))
        binding["public_wire_sha256"] = output["details"]["public_wire_sha256"]

def mutate_trust_anchor_source(m: dict[str, Any]) -> None:
    m["trust_anchor_source_sha256"] = "66" * 32
    m["session_binding_sha256"] = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_der_sha256"],
    )


def mutate_trust_anchor_state_upgrade(m: dict[str, Any]) -> None:
    m["trust_anchor_state"] = "DENY"
    bound = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_der_sha256"],
    )
    m["trust_anchor_state"] = "PASS"
    m["session_binding_sha256"] = bound


def mutate_verification_mode_upgrade(m: dict[str, Any]) -> None:
    m["verification_mode"] = "OfflineBundle"
    bound = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_der_sha256"],
    )
    m["verification_mode"] = "ReferenceModelOnly"
    m["session_binding_sha256"] = bound


def mutate_leaf(m: dict[str, Any], leaf: bytes) -> None:
    leaf_sha = hashlib.sha256(leaf).hexdigest()
    m["leaf_certificate_der_base64"] = b64(leaf)
    m["leaf_certificate_sha256"] = leaf_sha
    m["spki_binding"]["certificate_sha256"] = leaf_sha
    m["session_binding_sha256"] = session_binding(
        m, leaf_sha, m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"], m["revocation"]["crl_der_sha256"]
    )


def self_test() -> int:
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-chain-fixture-") as td:
        fx = openssl_fixture(Path(td))
        base = make_manifest(fx)
        refresh_template_binding(base)
        base["session_binding_sha256"] = session_binding(
            base,
            base["leaf_certificate_sha256"],
            base["intermediate_certificate_sha256"],
            base["trust_anchor_root_sha256"],
            base["revocation"]["crl_der_sha256"],
        )
        cases = [
            ("canonical-valid", "PASS", lambda x: None),
            ("root-substitution", "DENY", lambda x: x.update({
                "trust_anchor_root_der_base64": x["intermediate_certificate_der_base64"],
                "trust_anchor_root_sha256": x["intermediate_certificate_sha256"],
            })),
            ("intermediate-substitution", "DENY", lambda x: x.update({
                "intermediate_certificate_der_base64": x["trust_anchor_root_der_base64"],
                "intermediate_certificate_sha256": x["trust_anchor_root_sha256"],
            })),
            ("leaf-byte-substitution", "DENY", lambda x: mutate_leaf(x, fx["leaf"][:-1] + bytes([fx["leaf"][-1] ^ 1]))),
            ("expired-reference-time", "DENY", lambda x: x.update({"verification_time_unix": int(time.time()) + 20 * 365 * 24 * 3600})),
            ("not-yet-valid-reference-time", "DENY", lambda x: x.update({"verification_time_unix": 0})),
            ("key-usage-profile-mismatch", "DENY", lambda x: mutate_leaf(x, fx["bad_usage"])),
            ("eku-profile-mismatch", "DENY", lambda x: mutate_leaf(x, fx["bad_eku"])),
            ("trust-anchor-source-substitution", "DENY", lambda x: mutate_trust_anchor_source(x)),
            ("template-verifier-substitution", "DENY", lambda x: x["ek_template_binding"].update({"verifier_id": "other-verifier"})),
            ("template-source-substitution", "DENY", lambda x: x["ek_template_binding"].update({"source_sha256": "12" * 32})),
            ("template-input-substitution", "DENY", lambda x: x["ek_template_binding"].update({"input_sha256": "14" * 32})),
            ("template-wire-substitution", "DENY", lambda x: x["ek_template_binding"].update({"public_wire_sha256": "13" * 32})),
            ("template-indeterminate", "INDETERMINATE", lambda x: x["ek_template_binding"].update({"state": "INDETERMINATE"})),
            ("trust-anchor-state-substitution", "DENY", lambda x: x.update({"trust_anchor_state": "DENY"})),
            ("revocation-deny", "DENY", lambda x: x["revocation"].update({"state": "DENY"})),
            ("revocation-indeterminate", "INDETERMINATE", lambda x: x["revocation"].update({"state": "INDETERMINATE"})),
            ("spki-certificate-substitution", "DENY", lambda x: x["spki_binding"].update({"certificate_sha256": "77" * 32})),
            ("spki-indeterminate", "INDETERMINATE", lambda x: x["spki_binding"].update({"state": "INDETERMINATE"})),
            ("spki-ek-public-digest-substitution", "DENY", lambda x: x["spki_binding"].update({"ek_public_wire_sha256": "77" * 32})),
            ("verification-time-binding-substitution", "DENY", lambda x: x.update({"verification_time_unix": x["verification_time_unix"] + 3600})),
            ("revocation-state-binding-substitution", "DENY", lambda x: x["revocation"].update({"state": "PASS"})),
            ("verification-mode-substitution", "DENY", lambda x: x.update({"verification_mode": "OfflineBundle"})),
            ("session-binding-substitution", "DENY", lambda x: x.update({"session_id": "attacker"})),
        ]
        for name, expected, mutate in cases:
            candidate = copy.deepcopy(base)
            try:
                mutate(candidate)
            except Exception as exc:
                print(f"{name}: mutation setup failed: {exc}")
                return 1
            observed = verify(candidate)
            if observed["state"] != expected:
                print(
                    f"{name}: FAIL expected={expected} got={observed['state']} "
                    f"reason={observed['reason']}"
                )
                return 1
        permuted = json.loads(json.dumps(base, sort_keys=True))
        if verify(permuted)["state"] != "PASS":
            print("key-order-permutation: FAIL")
            return 1

    print("EK certificate chain policy semantic corpus: PASS")
    print("22 adversarial mutations plus canonical and key-order controls: PASS")
    print("synthetic trust anchor is explicitly reference-only")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--verify", metavar="MANIFEST")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.verify:
        parser.error("--verify is required")
    path = Path(args.verify).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit("manifest must be a JSON object")
    verified = verify(manifest)
    out = {
        "profile_id": "mycelix.security.tpm.ek-cert-chain-policy",
        "profile_version": "0.1.0",
        "verifier_id": VERIFIER_ID,
        "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        **verified,
    }
    out["content_sha256"] = canonical_hash(
        {k: v for k, v in out.items() if k != "content_sha256"}
    )
    rendered = json.dumps(out, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "DENY": 1, "INDETERMINATE": 2}[verified["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
