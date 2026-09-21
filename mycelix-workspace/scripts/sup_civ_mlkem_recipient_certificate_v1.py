#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import tempfile


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def run(*args: str, input_bytes: bytes | None = None) -> bytes:
    return subprocess.run(
        list(args),
        input=input_bytes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    ).stdout


def clean_ski(text: str) -> str:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 2:
        raise AssertionError(f"unexpected SKI output: {text!r}")
    return lines[-1].replace(":", "").lower()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=pathlib.Path, required=True)
    args = ap.parse_args()

    m = json.loads(args.manifest.read_text(encoding="utf-8"))
    assert m["schema"] == "sup-civ-000d1c2b2b0-mlkem-recipient-certificate-v1"
    assert m["structural_parent"] == "27852e1eadc831fc3957758769031ef8855cd62f"
    assert m["recipient_spki"]["private_key_committed"] is False
    assert m["recipient_identifier"]["authorization_proven"] is False

    recipient_path = pathlib.Path(m["recipient_certificate"]["path"])
    ca_path = pathlib.Path(m["ca_certificate"]["path"])
    recipient = recipient_path.read_bytes()
    ca = ca_path.read_bytes()

    assert len(recipient) == m["recipient_certificate"]["length"]
    assert sha256_hex(recipient) == m["recipient_certificate"]["sha256"]
    assert len(ca) == m["ca_certificate"]["length"]
    assert sha256_hex(ca) == m["ca_certificate"]["sha256"]

    with tempfile.TemporaryDirectory(prefix="sup-civ-c2b2b0-") as td:
        td = pathlib.Path(td)
        recipient_pem = td / "recipient.pem"
        ca_pem = td / "ca.pem"
        spki_der = td / "recipient-spki.der"

        recipient_pem.write_bytes(run("openssl", "x509", "-inform", "DER", "-in", str(recipient_path), "-outform", "PEM"))
        ca_pem.write_bytes(run("openssl", "x509", "-inform", "DER", "-in", str(ca_path), "-outform", "PEM"))

        verify = run("openssl", "verify", "-CAfile", str(ca_pem), str(recipient_pem)).decode().strip()
        if not verify.endswith(": OK"):
            raise AssertionError(f"certificate chain verification failed: {verify}")

        meta = run(
            "openssl", "x509", "-in", str(recipient_pem), "-noout",
            "-nameopt", "RFC2253", "-subject", "-issuer", "-serial", "-dates"
        ).decode()
        expected_lines = {
            f"subject={m['recipient_certificate']['subject_rfc2253']}",
            f"issuer={m['recipient_certificate']['issuer_rfc2253']}",
            f"serial={m['recipient_certificate']['serial_hex']}",
            f"notBefore={m['recipient_certificate']['not_before']}",
            f"notAfter={m['recipient_certificate']['not_after']}",
        }
        got_lines = {line.strip() for line in meta.splitlines() if line.strip()}
        missing = expected_lines - got_lines
        if missing:
            raise AssertionError(f"certificate metadata drift: missing {sorted(missing)} from {sorted(got_lines)}")

        text = run("openssl", "x509", "-in", str(recipient_pem), "-noout", "-text").decode()
        if "Public Key Algorithm: ML-KEM-768" not in text:
            raise AssertionError("recipient certificate SPKI is not ML-KEM-768")

        pub_pem = run("openssl", "x509", "-in", str(recipient_pem), "-pubkey", "-noout")
        spki = run("openssl", "pkey", "-pubin", "-outform", "DER", input_bytes=pub_pem)
        spki_der.write_bytes(spki)
        if sha256_hex(spki) != m["recipient_spki"]["sha256"]:
            raise AssertionError("recipient SPKI does not match frozen C2A1 public-key fixture")

        asn1 = run("openssl", "asn1parse", "-inform", "DER", "-in", str(spki_der), "-i").decode()
        if "ML-KEM-768" not in asn1 and "2.16.840.1.101.3.4.4.2" not in asn1:
            raise AssertionError("SPKI ASN.1 does not identify ML-KEM-768")

        ski_out = run("openssl", "x509", "-in", str(recipient_pem), "-noout", "-ext", "subjectKeyIdentifier").decode()
        ski = clean_ski(ski_out)
        if ski != m["recipient_identifier"]["hex"]:
            raise AssertionError("certificate SKI does not match frozen B2A RID")
        if ski != m["extensions"]["subject_key_identifier"]["hex"]:
            raise AssertionError("certificate SKI does not match manifest extension")

        ku = run("openssl", "x509", "-in", str(recipient_pem), "-noout", "-ext", "keyUsage").decode()
        ku_lines = [line.strip() for line in ku.splitlines() if line.strip()]
        if not ku_lines or ku_lines[0] != "X509v3 Key Usage: critical":
            raise AssertionError(f"keyUsage must be critical: {ku!r}")
        if ku_lines[1:] != m["extensions"]["key_usage"]["only"]:
            raise AssertionError(f"keyUsage must contain only Key Encipherment: {ku_lines!r}")

        bc = run("openssl", "x509", "-in", str(recipient_pem), "-noout", "-ext", "basicConstraints").decode()
        bc_lines = [line.strip() for line in bc.splitlines() if line.strip()]
        if not bc_lines or bc_lines[0] != "X509v3 Basic Constraints: critical":
            raise AssertionError(f"basicConstraints must be critical: {bc!r}")
        if bc_lines[1:] != [m["extensions"]["basic_constraints"]["value"]]:
            raise AssertionError(f"basicConstraints drift: {bc_lines!r}")

        print(f"openssl-version={run('openssl','version').decode().strip()}")
        print(f"recipient-cert-sha256={sha256_hex(recipient)}")
        print(f"ca-cert-sha256={sha256_hex(ca)}")
        print(f"recipient-spki-sha256={sha256_hex(spki)}")
        print(f"recipient-ski={ski}")
        print("SUP-CIV-000D1C2B2B0 PASS: exact ML-KEM recipient certificate/RID fixture verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
