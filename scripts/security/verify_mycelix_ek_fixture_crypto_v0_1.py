#!/usr/bin/env python3
"""Independent exact-byte RSA/SHA-256 witness for the synthetic EK corpus."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-fixture-crypto.v0.1"
PROFILE_ID = "mycelix.security.tpm.ek-fixture-crypto"
PROFILE_VERSION = "0.1.0"
RSA_OID = "1.2.840.113549.1.1.1"
SHA256_RSA_OID = "1.2.840.113549.1.1.11"
SHA256_DI = bytes.fromhex("3031300d060960864801650304020105000420")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()


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
    out["content_sha256"] = canonical_hash({k: v for k, v in out.items() if k != "content_sha256"})
    return out


def tlv(data: bytes, pos: int = 0) -> tuple[int, bytes, bytes, int]:
    start = pos
    if pos >= len(data):
        raise ValueError("DER truncated")
    tag = data[pos]
    pos += 1
    if pos >= len(data):
        raise ValueError("DER length truncated")
    first = data[pos]
    pos += 1
    if first & 0x80:
        n = first & 0x7f
        if n == 0 or n > 4 or pos + n > len(data) or data[pos] == 0:
            raise ValueError("DER non-canonical length")
        length = int.from_bytes(data[pos:pos+n], "big")
        if length < 128:
            raise ValueError("DER long-form length for short value")
        pos += n
    else:
        length = first
    end = pos + length
    if end > len(data):
        raise ValueError("DER value truncated")
    return tag, data[pos:end], data[start:end], end


def children(content: bytes) -> list[tuple[int, bytes, bytes]]:
    out = []
    p = 0
    while p < len(content):
        tag, val, raw, nxt = tlv(content, p)
        out.append((tag, val, raw))
        p = nxt
    if p != len(content):
        raise ValueError("DER trailing bytes")
    return out


def integer(content: bytes, name: str, positive: bool = False) -> int:
    if not content or content[0] & 0x80:
        raise ValueError(f"{name} INTEGER invalid")
    if len(content) > 1 and content[0] == 0 and not (content[1] & 0x80):
        raise ValueError(f"{name} INTEGER non-canonical")
    value = int.from_bytes(content, "big")
    if positive and value <= 0:
        raise ValueError(f"{name} INTEGER not positive")
    return value


def oid(content: bytes) -> str:
    if not content:
        raise ValueError("empty OID")
    arcs = []
    value = 0
    first = True
    first_subidentifier = True
    for b in content:
        if first_subidentifier and b == 0x80:
            raise ValueError("OID has non-canonical leading zero subidentifier")
        value = (value << 7) | (b & 0x7f)
        if not (b & 0x80):
            if first:
                if value < 40:
                    arcs = [0, value]
                elif value < 80:
                    arcs = [1, value - 40]
                else:
                    arcs = [2, value - 80]
                first_subidentifier = False
                first = False
            else:
                arcs.append(value)
            value = 0
    if first or value:
        raise ValueError("unterminated OID")
    return ".".join(map(str, arcs))


def alg_oid(raw: bytes) -> str:
    tag, content, _raw, end = tlv(raw)
    if tag != 0x30 or end != len(raw):
        raise ValueError("AlgorithmIdentifier malformed")
    parts = children(content)
    if len(parts) != 2 or parts[0][0] != 0x06 or parts[1][0] != 0x05 or parts[1][1] != b"":
        raise ValueError("AlgorithmIdentifier parameters are not explicit NULL")
    return oid(parts[0][1])


def rsa_spki(raw: bytes) -> tuple[int, int]:
    tag, content, _raw, end = tlv(raw)
    if tag != 0x30 or end != len(raw):
        raise ValueError("SPKI malformed")
    parts = children(content)
    if len(parts) != 2 or parts[0][0] != 0x30 or parts[1][0] != 0x03:
        raise ValueError("SPKI structure invalid")
    if alg_oid(parts[0][2]) != RSA_OID:
        raise ValueError("SPKI is not rsaEncryption")
    bits = parts[1][1]
    if not bits or bits[0] != 0:
        raise ValueError("SPKI BIT STRING has non-zero unused bits")
    tag, rsa_content, _raw, end = tlv(bits[1:])
    if tag != 0x30 or end != len(bits[1:]):
        raise ValueError("RSA public key malformed")
    parts = children(rsa_content)
    if len(parts) != 2:
        raise ValueError("RSA public key requires n,e")
    n = integer(parts[0][1], "modulus", True)
    e = integer(parts[1][1], "exponent", True)
    if n.bit_length() != 2048 or n % 2 == 0 or e % 2 == 0 or e < 3:
        raise ValueError("issuer RSA key violates fixture constraints")
    return n, e


def signed_object(der: bytes, kind: str) -> dict[str, Any]:
    tag, content, _raw, end = tlv(der)
    if tag != 0x30 or end != len(der):
        raise ValueError(f"{kind} outer SEQUENCE malformed")
    parts = children(content)
    if len(parts) != 3 or any(parts[i][0] != [0x30, 0x30, 0x03][i] for i in range(3)):
        raise ValueError(f"{kind} signed structure invalid")
    tbs = parts[0][2]
    alg = alg_oid(parts[1][2])
    sig_bits = parts[2][1]
    if not sig_bits or sig_bits[0] != 0:
        raise ValueError(f"{kind} signature BIT STRING has unused bits")
    signature = sig_bits[1:]
    if alg != SHA256_RSA_OID:
        raise ValueError(f"{kind} signature algorithm unsupported")
    ttag, tcontent, _traw, tend = tlv(tbs)
    if ttag != 0x30 or tend != len(tbs):
        raise ValueError(f"{kind} TBS malformed")
    fields = children(tcontent)
    cursor = 0
    if kind == "certificate":
        if not fields or fields[0][0] != 0xA0:
            raise ValueError("certificate is not explicit X.509 v3")
        v = children(fields[0][1])
        if len(v) != 1 or v[0][0] != 0x02 or integer(v[0][1], "version") != 2:
            raise ValueError("certificate is not X.509 v3")
        cursor = 1
        serial = integer(fields[cursor][1], "serial", True)
        cursor += 1
        inner_alg = fields[cursor][2]
        if alg_oid(inner_alg) != SHA256_RSA_OID or inner_alg != parts[1][2]:
            raise ValueError("certificate signatureAlgorithm mismatch")
        cursor += 1
        issuer = fields[cursor][2]
        cursor += 1
        validity = fields[cursor][1]
        if len(children(validity)) != 2:
            raise ValueError("certificate validity malformed")
        cursor += 1
        subject = fields[cursor][2]
        cursor += 1
        spki = fields[cursor][2]
        n, e = rsa_spki(spki)
        return {
            "object_sha256": hashlib.sha256(der).hexdigest(),
            "tbs_sha256": hashlib.sha256(tbs).hexdigest(),
            "signature_sha256": hashlib.sha256(signature).hexdigest(),
            "signature_algorithm_oid": alg,
            "signature": signature,
            "tbs": tbs,
            "issuer": issuer,
            "subject": subject,
            "spki": spki,
            "n": n,
            "e": e,
            "serial": serial,
        }

    cursor = 0
    if fields and fields[0][0] == 0x02:
        if integer(fields[0][1], "CRL version") != 1:
            raise ValueError("CRL is not v2")
        cursor = 1
    inner_alg = fields[cursor][2]
    if alg_oid(inner_alg) != SHA256_RSA_OID or inner_alg != parts[1][2]:
        raise ValueError("CRL signatureAlgorithm mismatch")
    issuer = fields[cursor + 1][2]
    return {
        "object_sha256": hashlib.sha256(der).hexdigest(),
        "tbs_sha256": hashlib.sha256(tbs).hexdigest(),
        "signature_sha256": hashlib.sha256(signature).hexdigest(),
        "signature_algorithm_oid": alg,
        "signature": signature,
        "tbs": tbs,
        "issuer": issuer,
    }


def verify_signature(tbs: bytes, signature: bytes, issuer_n: int, issuer_e: int) -> None:
    width = (issuer_n.bit_length() + 7) // 8
    if len(signature) != width:
        raise ValueError("signature width mismatch")
    s = int.from_bytes(signature, "big")
    if s >= issuer_n:
        raise ValueError("signature representative out of range")
    encoded = pow(s, issuer_e, issuer_n).to_bytes(width, "big")
    digest = SHA256_DI + hashlib.sha256(tbs).digest()
    if not encoded.startswith(b"\\x00\\x01"):
        raise ValueError("PKCS#1 v1.5 header invalid")
    zero = encoded.find(b"\\x00", 2)
    if zero < 10 or any(b != 0xFF for b in encoded[2:zero]) or encoded[zero+1:] != digest:
        raise ValueError("RSA/SHA-256 signature invalid")


def crl_blocks(bundle: bytes) -> list[bytes]:
    begin = b"-----BEGIN X509 CRL-----"
    end = b"-----END X509 CRL-----"
    result_blocks = []
    cursor = 0
    while cursor < len(bundle):
        start = bundle.find(begin, cursor)
        if start < 0:
            if bundle[cursor:].strip():
                raise ValueError("CRL bundle contains non-CRL bytes")
            break
        if bundle[cursor:start].strip():
            raise ValueError("CRL bundle has bytes between PEM blocks")
        finish = bundle.find(end, start + len(begin))
        if finish < 0:
            raise ValueError("CRL END marker missing")
        finish += len(end)
        block = bundle[start:finish] + b"\\n"
        encoded = b"".join(block[len(begin):-len(end)].split())
        der = base64.b64decode(encoded, validate=True)
        # Preserve exact PEM identity while parsing exact DER payload.
        result_blocks.append((block, der))
        cursor = finish
    if len(result_blocks) != 2:
        raise ValueError("expected exactly two CRLs")
    return result_blocks


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    try:
        leaf = base64.b64decode(manifest["leaf_certificate_der_base64"], validate=True)
        intermediate = base64.b64decode(manifest["intermediate_certificate_der_base64"], validate=True)
        root = base64.b64decode(manifest["trust_anchor_root_der_base64"], validate=True)
        crl_bundle = base64.b64decode(manifest["crl_bundle_pem_base64"], validate=True)
        for data, field in (
            (leaf, "leaf_certificate_sha256"),
            (intermediate, "intermediate_certificate_sha256"),
            (root, "trust_anchor_root_sha256"),
            (crl_bundle, "crl_bundle_pem_sha256"),
        ):
            if hashlib.sha256(data).hexdigest() != manifest[field]:
                return result("DENY", "input-digest-mismatch", {"field": field})
        l = signed_object(leaf, "certificate")
        i = signed_object(intermediate, "certificate")
        r = signed_object(root, "certificate")
        if l["issuer"] != i["subject"] or i["issuer"] != r["subject"] or r["issuer"] != r["subject"]:
            return result("DENY", "name-binding-mismatch")
        verify_signature(l["tbs"], l["signature"], i["n"], i["e"])
        verify_signature(i["tbs"], i["signature"], r["n"], r["e"])
        verify_signature(r["tbs"], r["signature"], r["n"], r["e"])
        crls = []
        for pem, der in crl_blocks(crl_bundle):
            parsed = signed_object(der, "crl")
            crls.append((pem, parsed))
        crl_results: dict[str, Any] = {}
        for pem, crl in crls:
            if crl["issuer"] == r["subject"]:
                label, issuer = "root", r
            elif crl["issuer"] == i["subject"]:
                label, issuer = "intermediate", i
            else:
                return result("DENY", "crl-issuer-mismatch")
            if label in crl_results:
                return result("DENY", "duplicate-crl-issuer")
            verify_signature(crl["tbs"], crl["signature"], issuer["n"], issuer["e"])
            crl_results[label] = {
                "object_sha256": crl["object_sha256"],
                "pem_block_sha256": hashlib.sha256(pem).hexdigest(),
                "tbs_sha256": crl["tbs_sha256"],
                "signature_sha256": crl["signature_sha256"],
                "signature_algorithm_oid": crl["signature_algorithm_oid"],
                "issuer_name_sha256": hashlib.sha256(crl["issuer"]).hexdigest(),
                "issuer_object_sha256": issuer["object_sha256"],
            }
        if set(crl_results) != {"root", "intermediate"}:
            return result("DENY", "crl-set-incomplete")
        details = {
            "exact_input_objects": {
                "leaf_certificate_sha256": l["object_sha256"],
                "intermediate_certificate_sha256": i["object_sha256"],
                "trust_anchor_root_sha256": r["object_sha256"],
                "crl_bundle_pem_sha256": hashlib.sha256(crl_bundle).hexdigest(),
            },
            "certificate_signatures": {
                "leaf": {
                    "object_sha256": l["object_sha256"], "tbs_sha256": l["tbs_sha256"],
                    "signature_sha256": l["signature_sha256"], "issuer_object_sha256": i["object_sha256"],
                    "issuer_spki_sha256": hashlib.sha256(i["spki"]).hexdigest(),
                    "signature_verification": "PASS",
                },
                "intermediate": {
                    "object_sha256": i["object_sha256"], "tbs_sha256": i["tbs_sha256"],
                    "signature_sha256": i["signature_sha256"], "issuer_object_sha256": r["object_sha256"],
                    "issuer_spki_sha256": hashlib.sha256(r["spki"]).hexdigest(),
                    "signature_verification": "PASS",
                },
                "root": {
                    "object_sha256": r["object_sha256"], "tbs_sha256": r["tbs_sha256"],
                    "signature_sha256": r["signature_sha256"], "issuer_object_sha256": r["object_sha256"],
                    "issuer_spki_sha256": hashlib.sha256(r["spki"]).hexdigest(),
                    "signature_verification": "PASS",
                },
            },
            "crl_signatures": crl_results,
            "exact_relationships": {
                "leaf_to_intermediate_subject_exact": True,
                "intermediate_to_root_subject_exact": True,
                "root_self_issued_exact": True,
                "root_crl_issuer_exact": True,
                "intermediate_crl_issuer_exact": True,
            },
        }
        return result("PASS", "all-exact-signatures-verified", details)
    except (KeyError, ValueError, TypeError) as exc:
        return result("DENY", "cryptographic-parse-or-verification-failed", {"error": str(exc)})


def self_test() -> int:
    root = Path(__file__).resolve().parents[2]
    generator = root / "scripts/security/generate_mycelix_ek_chain_fixtures_v0_1.py"
    recipe = root / "docs/security/fixtures/ek-chain-policy-v0.1/fixture-recipe-v0.1.json"
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-crypto-") as td:
        out = Path(td)
        proc = subprocess.run([sys.executable, str(generator), "--recipe", str(recipe), "--output-dir", str(out), "--check"], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        if proc.returncode:
            print("fixture generation: FAIL")
            print(proc.stderr or proc.stdout)
            return 1
        m = {
            "leaf_certificate_der_base64": base64.b64encode((out/"leaf.der").read_bytes()).decode(),
            "leaf_certificate_sha256": hashlib.sha256((out/"leaf.der").read_bytes()).hexdigest(),
            "intermediate_certificate_der_base64": base64.b64encode((out/"intermediate.der").read_bytes()).decode(),
            "intermediate_certificate_sha256": hashlib.sha256((out/"intermediate.der").read_bytes()).hexdigest(),
            "trust_anchor_root_der_base64": base64.b64encode((out/"root.der").read_bytes()).decode(),
            "trust_anchor_root_sha256": hashlib.sha256((out/"root.der").read_bytes()).hexdigest(),
            "crl_bundle_pem_base64": base64.b64encode((out/"crl-bundle.pem").read_bytes()).decode(),
            "crl_bundle_pem_sha256": hashlib.sha256((out/"crl-bundle.pem").read_bytes()).hexdigest(),
        }
        good = verify(m)
        if good["state"] != "PASS":
            print("exact synthetic EK cryptographic witness: FAIL")
            print(good)
            return 1
        bad = dict(m)
        tampered = bytearray(base64.b64decode(m["leaf_certificate_der_base64"]))
        tampered[-1] ^= 1
        bad["leaf_certificate_der_base64"] = base64.b64encode(tampered).decode()
        bad["leaf_certificate_sha256"] = hashlib.sha256(tampered).hexdigest()
        if verify(bad)["state"] != "DENY":
            print("tampered leaf rejection: FAIL")
            return 1
    print("independent EK certificate/CRL cryptographic witness: PASS")
    print("exact DER/TBS/signature binding: PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--verify")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.output:
        parser.error("--output is required with --verify")
    path = Path(args.verify).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    out = verify(manifest)
    out["input_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    out["content_sha256"] = canonical_hash({k: v for k, v in out.items() if k != "content_sha256"})
    Path(args.output).write_text(json.dumps(out, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"PASS": 0, "DENY": 1, "INDETERMINATE": 2}[out["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
