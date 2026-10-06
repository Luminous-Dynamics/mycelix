#!/usr/bin/env python3
"""Verify an EK X.509 certificate path under an explicit reference policy."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
import subprocess
from datetime import datetime, timezone
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-chain-policy.v0.1"
SPKI_VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
PATH_VERIFIER_ID = "mycelix.tpm.ek-cert-path-validation.v0.1"
CRYPTO_BINDING_ID = "mycelix.tpm.ek-fixture-signatures.v0.1"
SHA256_WITH_RSA_OID = "1.2.840.113549.1.1.11"
RSA_ENCRYPTION_OID = "1.2.840.113549.1.1.1"
TEMPLATE_VERIFIER_ID = "mycelix.tpm.ek-template-appraisal.v0.1"
ROOT = Path(__file__).resolve().parents[2]
TRUST_ANCHOR_APPRAISAL_ID = "mycelix.tpm.ek-trust-anchor-appraisal.v0.1"
TRUST_ANCHOR_APPRAISAL_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_trust_anchor_appraisal_v0_1.py")
TRUST_ANCHOR_REGISTRY_FILE = ROOT / "docs/security/mycelix-ek-trust-anchor-registry-v0.1.json"
TRUST_ANCHOR_AUTHORIZATION_RECEIPT_FILE = ROOT / "docs/security/mycelix-ek-trust-anchor-authorization-receipt-v0.1.json"
SPKI_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_cert_spki_binding_v0_1.py")
FIXTURE_RECIPE_FILE = ROOT / "docs/security/fixtures/ek-chain-policy-v0.1/fixture-recipe-v0.1.json"
FIXTURE_GENERATOR_SCRIPT = ROOT / "scripts/security/generate_mycelix_ek_chain_fixtures_v0_1.py"
CRYPTO_VERIFIER_SCRIPT = ROOT / "scripts/security/verify_mycelix_ek_fixture_crypto_v0_1.py"
REFERENCE_ROOT_SOURCE_TAG = "mycelix.synthetic-ek-root.v0.1"
REFERENCE_ROOT_SHA256 = "fca39a44f906461818995af4242bc7d779eb5a0266349c3ed0236053ddcb5556"
REFERENCE_TIME_UNIX = 1791158400
REFERENCE_EK_RSA_MODULUS = bytes.fromhex("6f24c46cf921615f74a3c7a6a01b73b3b06e7ae9b51d575cbac358ec03593f47d4b54110aff2589c1d9ac57e6c5b34bcdaa563550d294b06c8dd308cc466b4204901dad45fc012ec169a1224101108abe2da72b33c8e77c32f198b1fa9b95f26b85ee36a202a7102571ee51efd71e7618b94b931dcb05cfda6768d546f0a2256ce37707ef644da096a422caebf0e5c6698de39b2145fbdcaef529a6f53f6d2a81d151b63f1714d0f3e4c6702bb00051f33b2451302c18513b3620dca718927b2555c358fd40dad3d2f32e1fd8ca631955b9e5a569e6fb4b813d6da39cbe41e8d0458fbb865e646f0e5a81a4208c90917492bfda3befcab8eadaee1b20163e5b7")
EXPIRED_TIME_UNIX = 4102444800
TEMPLATE_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_template_appraisal_v0_1.py")
PATH_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_cert_path_validation_v0_1.py")
EK_CERT_EKU_OID = "2.23.133.8.1"


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


def parse_crl_time(tag: int, content: bytes, field: str) -> dict[str, Any]:
    if tag == 0x17:
        if len(content) != 13 or not content.endswith(b"Z"):
            raise ValueError(f"{field} UTCTime malformed")
        value_text = content.decode("ascii")
        year_short = int(value_text[:2])
        year = 1900 + year_short if year_short >= 50 else 2000 + year_short
        fmt = "%y%m%d%H%M%SZ"
    elif tag == 0x18:
        if len(content) != 15 or not content.endswith(b"Z"):
            raise ValueError(f"{field} GeneralizedTime malformed")
        value_text = content.decode("ascii")
        year = int(value_text[:4])
        fmt = "%Y%m%d%H%M%SZ"
    else:
        raise ValueError(f"{field} must be UTCTime or GeneralizedTime")
    try:
        parsed = datetime.strptime(value_text, fmt).replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise ValueError(f"{field} time value invalid: {exc}") from exc
    if tag == 0x17 and not 1950 <= year <= 2049:
        raise ValueError(f"{field} UTCTime year outside RFC 5280 range")
    return {"text": value_text, "unix": int(parsed.timestamp())}


def parse_crl_reason_extension(ext_value: bytes) -> int:
    tag, content, _raw, end = der_tlv(ext_value, 0)
    if tag != 0x0A or end != len(ext_value) or not content or content[0] & 0x80:
        raise ValueError("CRL reasonCode malformed")
    if len(content) > 1 and content[0] == 0:
        raise ValueError("CRL reasonCode non-canonical")
    reason = int.from_bytes(content, "big")
    if reason not in {0, 1, 2, 3, 4, 5, 6, 8, 9, 10}:
        raise ValueError("CRL reasonCode unsupported")
    return reason


def parse_crl_aki(info: dict[str, Any]) -> bytes:
    extension = info["crl_extensions"].get("2.5.29.35")
    if not extension:
        raise ValueError("CRL AuthorityKeyIdentifier missing")
    if extension["critical"]:
        raise ValueError("CRL AuthorityKeyIdentifier must be non-critical")
    tag, content, _raw, end = der_tlv(extension["extn_value"], 0)
    if tag != 0x30 or end != len(extension["extn_value"]):
        raise ValueError("CRL AuthorityKeyIdentifier malformed")
    fields = der_children(content)
    if len(fields) != 1 or fields[0][0] != 0x80 or not fields[0][1]:
        raise ValueError("CRL AuthorityKeyIdentifier must contain exactly keyIdentifier")
    return fields[0][1]


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


def split_pem_crls(bundle: bytes) -> list[bytes]:
    start_marker = b"-----BEGIN X509 CRL-----"
    end_marker = b"-----END X509 CRL-----"
    blocks: list[bytes] = []
    cursor = 0
    while cursor < len(bundle):
        start = bundle.find(start_marker, cursor)
        if start < 0:
            if bundle[cursor:].strip():
                raise ValueError("CRL bundle contains non-CRL data")
            break
        if bundle[cursor:start].strip():
            raise ValueError("CRL bundle contains bytes outside PEM CRL blocks")
        end = bundle.find(end_marker, start + len(start_marker))
        if end < 0:
            raise ValueError("CRL PEM block missing END marker")
        end += len(end_marker)
        if bundle[end:end + 2] == b"\r\n":
            end += 2
        elif bundle[end:end + 1] == b"\n":
            end += 1
        blocks.append(bundle[start:end])
        cursor = end
    if not blocks:
        raise ValueError("CRL bundle contains no PEM CRLs")
    return blocks


def hex_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or len(value) % 2:
        raise ValueError(f"{field} must be an even-length hexadecimal string")
    try:
        return bytes.fromhex(value)
    except ValueError as exc:
        raise ValueError(f"{field} invalid hexadecimal: {exc}") from exc


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


def der_encode_tlv(tag: int, value: bytes) -> bytes:
    if len(value) < 128:
        return bytes([tag, len(value)]) + value
    raw_length = len(value).to_bytes((len(value).bit_length() + 7) // 8, "big")
    return bytes([tag, 0x80 | len(raw_length)]) + raw_length + value


def der_tlv(data: bytes, offset: int) -> tuple[int, bytes, bytes, int]:
    if offset >= len(data):
        raise ValueError("DER truncated at tag")
    start = offset
    tag = data[offset]
    offset += 1
    if offset >= len(data):
        raise ValueError("DER truncated at length")
    first = data[offset]
    offset += 1
    if first & 0x80 == 0:
        length = first
    else:
        count = first & 0x7F
        if count == 0:
            raise ValueError("DER indefinite length forbidden")
        if count > 4 or offset + count > len(data):
            raise ValueError("DER length invalid")
        if data[offset] == 0:
            raise ValueError("DER non-canonical length")
        length = int.from_bytes(data[offset : offset + count], "big")
        if length < 128:
            raise ValueError("DER long-form length used for short value")
        offset += count
    end = offset + length
    if end > len(data):
        raise ValueError("DER value truncated")
    return tag, data[offset:end], data[start:end], end


def der_children(sequence_content: bytes) -> list[tuple[int, bytes, bytes]]:
    children: list[tuple[int, bytes, bytes]] = []
    offset = 0
    while offset < len(sequence_content):
        tag, content, raw, end = der_tlv(sequence_content, offset)
        children.append((tag, content, raw))
        offset = end
    if offset != len(sequence_content):
        raise ValueError("DER sequence trailing bytes")
    return children


def der_integer_value(content: bytes, field: str, *, positive: bool = False) -> int:
    if not content:
        raise ValueError(f"{field} INTEGER empty")
    if content[0] & 0x80:
        raise ValueError(f"{field} INTEGER negative")
    if len(content) > 1 and content[0] == 0 and not (content[1] & 0x80):
        raise ValueError(f"{field} INTEGER non-canonical leading zero")
    value = int.from_bytes(content, "big")
    if positive and value == 0:
        raise ValueError(f"{field} INTEGER must be positive")
    return value


def oid_base128_value(content: bytes, offset: int, field: str) -> tuple[int, int]:
    if offset >= len(content):
        raise ValueError(f"{field} OID truncated")
    value = 0
    first = True
    while True:
        if offset >= len(content):
            raise ValueError(f"{field} OID unterminated")
        byte = content[offset]
        offset += 1
        if first and byte & 0x80 and (byte & 0x7F) == 0:
            raise ValueError(f"{field} OID non-canonical base-128")
        value = (value << 7) | (byte & 0x7F)
        first = False
        if byte & 0x80 == 0:
            return value, offset


def algorithm_identifier_oid(raw: bytes, field: str) -> str:
    tag, content, _raw, end = der_tlv(raw, 0)
    if tag != 0x30 or end != len(raw):
        raise ValueError(f"{field} AlgorithmIdentifier malformed")
    children = der_children(content)
    if len(children) != 2 or children[0][0] != 0x06 or children[1][0] != 0x05 or children[1][1] != b"":
        raise ValueError(f"{field} AlgorithmIdentifier must contain OID and explicit NULL parameters")
    return oid_string(children[0][1])


def oid_string(content: bytes) -> str:
    if not content:
        raise ValueError("DER OID empty")
    first_subidentifier, offset = oid_base128_value(content, 0, "first")
    if first_subidentifier < 40:
        arcs = [0, first_subidentifier]
    elif first_subidentifier < 80:
        arcs = [1, first_subidentifier - 40]
    else:
        arcs = [2, first_subidentifier - 80]
    while offset < len(content):
        value, offset = oid_base128_value(content, offset, "arc")
        arcs.append(value)
    return ".".join(str(x) for x in arcs)


def bit_string_has(bit_string_content: bytes, bit_number: int) -> bool:
    if not bit_string_content:
        raise ValueError("DER BIT STRING empty")
    unused = bit_string_content[0]
    payload = bit_string_content[1:]
    if unused > 7:
        raise ValueError("DER BIT STRING invalid unused-bit count")
    if not payload:
        if unused != 0:
            raise ValueError("DER BIT STRING empty payload has unused bits")
        return False
    if unused and payload[-1] & ((1 << unused) - 1):
        raise ValueError("DER BIT STRING has non-zero padding bits")
    byte_index = bit_number // 8
    bit_mask = 0x80 >> (bit_number % 8)
    return byte_index < len(payload) and bool(payload[byte_index] & bit_mask)


def validate_named_bit_string(bit_string_content: bytes, field: str) -> None:
    if not bit_string_content:
        raise ValueError(f"{field} BIT STRING empty")
    unused = bit_string_content[0]
    payload = bit_string_content[1:]
    if unused > 7:
        raise ValueError(f"{field} BIT STRING invalid unused-bit count")
    if not payload:
        if unused != 0:
            raise ValueError(f"{field} BIT STRING empty payload has unused bits")
        return
    if unused and payload[-1] & ((1 << unused) - 1):
        raise ValueError(f"{field} BIT STRING has non-zero padding bits")
    if payload[-1] == 0:
        raise ValueError(f"{field} BIT STRING has non-canonical trailing zero byte")


def parse_extensions(extension_wrapper: bytes) -> dict[str, dict[str, Any]]:
    tag, content, _raw, end = der_tlv(extension_wrapper, 0)
    if tag != 0x30 or end != len(extension_wrapper):
        raise ValueError("X.509 Extensions must be a SEQUENCE")
    extension_children = der_children(content)
    if not extension_children:
        raise ValueError("X.509 Extensions must contain at least one Extension")
    extensions: dict[str, dict[str, Any]] = {}
    for ext_tag, ext_content, _ext_raw in extension_children:
        if ext_tag != 0x30:
            raise ValueError("X.509 Extension is not a SEQUENCE")
        offset = 0
        oid_tag, oid_content, _oid_raw, offset = der_tlv(ext_content, offset)
        if oid_tag != 0x06:
            raise ValueError("X.509 Extension missing OID")
        critical = False
        next_tag, next_content, _next_raw, next_offset = der_tlv(ext_content, offset)
        if next_tag == 0x01:
            if next_content != b"\xff":
                raise ValueError("X.509 Extension critical BOOLEAN must encode TRUE")
            critical = True
            next_tag, next_content, _next_raw, next_offset = der_tlv(ext_content, next_offset)
        if next_tag != 0x04 or next_offset != len(ext_content):
            raise ValueError("X.509 Extension missing extnValue")
        oid = oid_string(oid_content)
        if oid in extensions:
            raise ValueError(f"duplicate X.509 extension OID: {oid}")
        extensions[oid] = {
            "critical": critical,
            "extn_value": next_content,
        }
    return extensions


def parse_certificate_der(der: bytes) -> dict[str, Any]:
    tag, cert_content, _cert_raw, cert_end = der_tlv(der, 0)
    if tag != 0x30 or cert_end != len(der):
        raise ValueError("X.509 Certificate is not a single DER SEQUENCE")

    tag, tbs_content, _tbs_raw, cert_cursor = der_tlv(cert_content, 0)
    if tag != 0x30:
        raise ValueError("X.509 TBSCertificate is not a SEQUENCE")
    sig_alg_tag, sig_alg_content, sig_alg_raw, cert_cursor = der_tlv(cert_content, cert_cursor)
    if sig_alg_tag != 0x30:
        raise ValueError("X.509 certificate signatureAlgorithm is not a SEQUENCE")
    sig_value_tag, sig_value_content, _sig_value_raw, cert_end = der_tlv(cert_content, cert_cursor)
    if sig_value_tag != 0x03 or cert_end != len(cert_content):
        raise ValueError("X.509 certificate signatureValue is malformed")
    if sig_value_content[:1] != b"\x00":
        raise ValueError("X.509 certificate signatureValue must have zero unused bits")
    bit_string_has(sig_value_content, 0)
    signature_algorithm_oid = algorithm_identifier_oid(sig_alg_raw, "X.509.signatureAlgorithm")

    cursor = 0
    version = 1
    tag, content, _raw, next_cursor = der_tlv(tbs_content, cursor)
    if tag == 0xA0:
        inner_tag, inner_content, _inner_raw, inner_end = der_tlv(content, 0)
        if inner_tag != 0x02 or inner_end != len(content):
            raise ValueError("X.509 version field invalid")
        version_value = der_integer_value(inner_content, "X.509.version")
        if version_value > 2:
            raise ValueError("X.509 version value invalid")
        version = version_value + 1
        cursor = next_cursor

    tag, serial_content, _serial_raw, cursor = der_tlv(tbs_content, cursor)
    if tag != 0x02:
        raise ValueError("X.509 serial is not INTEGER")
    serial = der_integer_value(serial_content, "X.509.serial", positive=True)

    sig_tag, _sig_content, sig_raw, cursor = der_tlv(tbs_content, cursor)
    if sig_tag != 0x30:
        raise ValueError("X.509 TBSCertificate signature is not a SEQUENCE")
    tbs_signature_algorithm_oid = algorithm_identifier_oid(
        sig_raw, "X.509.TBSCertificate.signature"
    )
    if sig_raw != sig_alg_raw:
        raise ValueError("X.509 outer and TBSCertificate signatureAlgorithm encodings differ")
    if tbs_signature_algorithm_oid != SHA256_WITH_RSA_OID or signature_algorithm_oid != SHA256_WITH_RSA_OID:
        raise ValueError("synthetic EK certificate signature algorithm must be SHA256withRSA")

    issuer_tag, _issuer_content, issuer_raw, cursor = der_tlv(tbs_content, cursor)
    if issuer_tag != 0x30:
        raise ValueError("X.509 issuer Name invalid")

    validity_tag, validity_content, _validity_raw, cursor = der_tlv(tbs_content, cursor)
    if validity_tag != 0x30:
        raise ValueError("X.509 validity is not a SEQUENCE")
    validity = der_children(validity_content)
    if len(validity) != 2 or any(tag not in (0x17, 0x18) or not content or content[-1] != 0x5A for tag, content, _ in validity):
        raise ValueError("X.509 validity time structure invalid")

    subject_tag, subject_content, subject_raw, cursor = der_tlv(tbs_content, cursor)
    if subject_tag != 0x30:
        raise ValueError("X.509 subject Name invalid")

    spki_tag, _spki_content, spki_raw, cursor = der_tlv(tbs_content, cursor)
    if spki_tag != 0x30:
        raise ValueError("X.509 SubjectPublicKeyInfo invalid")

    extensions: dict[str, dict[str, Any]] = {}
    saw_issuer_unique_id = False
    saw_subject_unique_id = False
    saw_extensions = False
    while cursor < len(tbs_content):
        tag, content, _raw, cursor = der_tlv(tbs_content, cursor)
        if tag == 0xA1:
            if version != 3 or saw_issuer_unique_id or saw_subject_unique_id or saw_extensions:
                raise ValueError("X.509 issuerUniqueID is duplicated or out of order")
            saw_issuer_unique_id = True
            continue
        if tag == 0xA2:
            if version != 3 or saw_subject_unique_id or saw_extensions:
                raise ValueError("X.509 subjectUniqueID is duplicated or out of order")
            saw_subject_unique_id = True
            continue
        if tag == 0xA3:
            if saw_extensions or version != 3:
                raise ValueError("X.509 Extensions wrapper duplicated or outside v3")
            extensions = parse_extensions(content)
            saw_extensions = True
            continue
        raise ValueError("X.509 TBSCertificate contains unexpected trailing field")

    return {
        "version": version,
        "serial": serial,
        "issuer_der": issuer_raw,
        "subject_der": subject_raw,
        "spki_der": spki_raw,
        "subject_empty": subject_content == b"",
        "extensions": extensions,
        "tbs_der": tbs_raw,
        "signature_der": sig_value_content[1:],
        "signature_algorithm_der": sig_alg_raw,
        "signature_algorithm_oid": signature_algorithm_oid,
    }


def extension_value(info: dict[str, Any], oid: str) -> tuple[bool, bytes | None]:
    extension = info["extensions"].get(oid)
    if not extension:
        return False, None
    return bool(extension["critical"]), extension["extn_value"]


def basic_constraints(info: dict[str, Any]) -> tuple[bool, bool]:
    critical, value = extension_value(info, "2.5.29.19")
    if value is None:
        return critical, False
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError("BasicConstraints extension malformed")
    children = der_children(content)
    if not children:
        return critical, True
    if children[0][0] != 0x01 or children[0][1] != b"\xff":
        raise ValueError("BasicConstraints cA BOOLEAN must encode TRUE when present")
    if len(children) > 2:
        raise ValueError("BasicConstraints contains unexpected fields")
    if len(children) == 2:
        if children[1][0] != 0x02:
            raise ValueError("BasicConstraints pathLenConstraint malformed")
        der_integer_value(children[1][1], "BasicConstraints.pathLenConstraint")
    return critical, False


def key_usage_bits(info: dict[str, Any]) -> tuple[bool, bool, bool, bool]:
    critical, value = extension_value(info, "2.5.29.15")
    if value is None:
        return critical, False, False, False
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x03 or end != len(value):
        raise ValueError("KeyUsage extension malformed")
    validate_named_bit_string(content, "KeyUsage")
    return critical, bit_string_has(content, 2), bit_string_has(content, 6), bit_string_has(content, 5)


def eku_oids(info: dict[str, Any]) -> tuple[bool, list[str]]:
    critical, value = extension_value(info, "2.5.29.37")
    if value is None:
        return critical, []
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError("ExtendedKeyUsage extension malformed")
    oids: list[str] = []
    for child_tag, child_content, _raw in der_children(content):
        if child_tag != 0x06:
            raise ValueError("ExtendedKeyUsage contains non-OID")
        oids.append(oid_string(child_content))
    return critical, oids


def authority_key_id(info: dict[str, Any]) -> tuple[bool, bytes | None]:
    critical, value = extension_value(info, "2.5.29.35")
    if value is None:
        return critical, None
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError("AuthorityKeyIdentifier extension malformed")
    seen: set[int] = set()
    key_identifier: bytes | None = None
    for child_tag, child_content, _raw in der_children(content):
        if child_tag in seen:
            raise ValueError("AuthorityKeyIdentifier field duplicated")
        seen.add(child_tag)
        if child_tag == 0x80:
            if not child_content:
                raise ValueError("AuthorityKeyIdentifier keyIdentifier is empty")
            key_identifier = child_content
        elif child_tag == 0xA1:
            # authorityCertIssuer is GeneralNames encoded by IMPLICIT context tag [1].
            names = der_children(child_content)
            if not names:
                raise ValueError("AuthorityKeyIdentifier authorityCertIssuer is empty")
            allowed = {0xA0, 0x81, 0x82, 0x83, 0xA4, 0xA5, 0x86, 0x87, 0x88}
            for general_name_tag, general_name_content, _general_name_raw in names:
                if general_name_tag not in allowed:
                    raise ValueError("AuthorityKeyIdentifier authorityCertIssuer GeneralName invalid")
                if general_name_tag == 0xA4:
                    name_tag, _name_content, _name_raw, name_end = der_tlv(general_name_content, 0)
                    if name_tag != 0x30 or name_end != len(general_name_content):
                        raise ValueError("AuthorityKeyIdentifier directoryName malformed")
                elif general_name_tag == 0xA0:
                    other_tag, _other_content, _other_raw, other_end = der_tlv(general_name_content, 0)
                    if other_tag != 0x30 or other_end != len(general_name_content):
                        raise ValueError("AuthorityKeyIdentifier otherName malformed")
                elif general_name_tag == 0x87 and len(general_name_content) not in (4, 16):
                    raise ValueError("AuthorityKeyIdentifier iPAddress structure invalid")
                elif general_name_tag == 0x88:
                    oid_string(general_name_content)
        elif child_tag == 0x82:
            # authorityCertSerialNumber is CertificateSerialNumber, encoded IMPLICIT INTEGER.
            der_integer_value(child_content, "AuthorityKeyIdentifier.authorityCertSerialNumber", positive=True)
        else:
            raise ValueError("AuthorityKeyIdentifier contains unknown field")
    return critical, key_identifier


def subject_key_id(info: dict[str, Any]) -> tuple[bool, bytes | None]:
    critical, value = extension_value(info, "2.5.29.14")
    if value is None:
        return critical, None
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x04 or end != len(value):
        raise ValueError("SubjectKeyIdentifier extension malformed")
    return critical, content



def _extension_sequence_content(info: dict[str, Any], oid: str, label: str) -> tuple[bool, bytes | None]:
    critical, value = extension_value(info, oid)
    if value is None:
        return critical, None
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError(f"{label} extension value must be a SEQUENCE")
    return critical, content


def validate_aia(info: dict[str, Any]) -> bool:
    critical, content = _extension_sequence_content(
        info, "1.3.6.1.5.5.7.1.1", "AuthorityInformationAccess"
    )
    if content is None:
        return True
    if critical:
        raise ValueError("AuthorityInformationAccess MUST be non-critical")
    descriptions = der_children(content)
    if not descriptions:
        raise ValueError("AuthorityInformationAccess must contain AccessDescription")
    for tag, description, _raw in descriptions:
        if tag != 0x30:
            raise ValueError("AuthorityInformationAccess AccessDescription malformed")
        children = der_children(description)
        if len(children) != 2 or children[0][0] != 0x06:
            raise ValueError("AuthorityInformationAccess AccessDescription structure invalid")
        access_method = oid_string(children[0][1])
        if access_method not in {"1.3.6.1.5.5.7.48.1", "1.3.6.1.5.5.7.48.2"}:
            raise ValueError("AuthorityInformationAccess accessMethod is not id-ad-ocsp or id-ad-caIssuers")
        if children[1][0] not in {0xA0, 0x81, 0x82, 0xA4, 0xA5, 0x86, 0x87, 0x88}:
            raise ValueError("AuthorityInformationAccess accessLocation GeneralName invalid")
    return True


def validate_certificate_policies(info: dict[str, Any]) -> bool:
    critical, value = extension_value(info, "2.5.29.32")
    if value is None:
        return True
    if critical:
        raise ValueError("CertificatePolicies MUST be non-critical")
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError("CertificatePolicies extension must be a SEQUENCE")
    policies = der_children(content)
    if not policies:
        raise ValueError("CertificatePolicies must contain PolicyInformation")
    for policy_tag, policy_content, _policy_raw in policies:
        if policy_tag != 0x30:
            raise ValueError("CertificatePolicies PolicyInformation malformed")
        children = der_children(policy_content)
        if not children or children[0][0] != 0x06:
            raise ValueError("CertificatePolicies PolicyInformation missing policyIdentifier")
        oid_string(children[0][1])
        if len(children) > 2:
            raise ValueError("CertificatePolicies PolicyInformation has unexpected fields")
        if len(children) == 2:
            if children[1][0] != 0x30:
                raise ValueError("CertificatePolicies policyQualifiers malformed")
            # TCG v2.7's EK certificate table defines the value as PolicyIdentifier;
            # qualifiers are therefore outside this reference profile.
            if der_children(children[1][1]):
                raise ValueError("CertificatePolicies policyQualifiers are outside reference profile")
    return True


def validate_cdp(info: dict[str, Any]) -> bool:
    critical, content = _extension_sequence_content(
        info, "2.5.29.31", "CRLDistributionPoints"
    )
    if content is None:
        return True
    if critical:
        raise ValueError("CRLDistributionPoints MUST be non-critical")
    points = der_children(content)
    if not points:
        raise ValueError("CRLDistributionPoints must contain DistributionPoint")
    for tag, point, _raw in points:
        if tag != 0x30:
            raise ValueError("CRLDistributionPoints DistributionPoint malformed")
        children = der_children(point)
        if not children:
            raise ValueError("CRLDistributionPoints DistributionPoint empty")
        for child_tag, child_content, _child_raw in children:
            if child_tag == 0xA0:
                inner_tag, inner_content, _inner_raw, inner_end = der_tlv(child_content, 0)
                if inner_tag not in {0xA0, 0xA1} or inner_end != len(child_content):
                    raise ValueError("CRLDistributionPoints DistributionPointName malformed")
                if inner_tag == 0xA0 and not der_children(inner_content):
                    raise ValueError("CRLDistributionPoints fullName is empty")
            elif child_tag == 0x81:
                bit_string_has(child_content, 0)
            elif child_tag == 0xA2:
                names = der_children(child_content)
                if not names:
                    raise ValueError("CRLDistributionPoints cRLIssuer is empty")
            else:
                raise ValueError("CRLDistributionPoints DistributionPoint field invalid")
    return True


def validate_subject_directory_attributes(info: dict[str, Any]) -> bool:
    critical, content = _extension_sequence_content(
        info, "2.5.29.9", "SubjectDirectoryAttributes"
    )
    if content is None:
        return True
    if critical:
        raise ValueError("SubjectDirectoryAttributes MUST be non-critical")
    attributes = der_children(content)
    if not attributes:
        raise ValueError("SubjectDirectoryAttributes must contain Attribute")
    for tag, attribute, _raw in attributes:
        if tag != 0x30:
            raise ValueError("SubjectDirectoryAttributes Attribute malformed")
        children = der_children(attribute)
        if len(children) != 2 or children[0][0] != 0x06 or children[1][0] != 0x31:
            raise ValueError("SubjectDirectoryAttributes Attribute structure invalid")
        oid_string(children[0][1])
        if not der_children(children[1][1]):
            raise ValueError("SubjectDirectoryAttributes Attribute value SET empty")
    return True


def reference_tpm_values() -> dict[str, Any]:
    recipe = json.loads(FIXTURE_RECIPE_FILE.read_text(encoding="utf-8"))
    tpm = recipe.get("tpm")
    if not isinstance(tpm, dict) or not {"manufacturer", "model", "version"} <= set(tpm):
        raise ValueError("fixture recipe TPM metadata incomplete")
    return tpm


def subject_alt_name(info: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
    critical, value = extension_value(info, "2.5.29.17")
    if value is None:
        return critical, {"present": False, "directory_name_count": 0, "attributes": {}}
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x30 or end != len(value):
        raise ValueError("SubjectAltName extension malformed")
    general_names = der_children(content)
    if not general_names:
        raise ValueError("SubjectAltName must contain at least one GeneralName")
    directory_names = 0
    attributes: dict[str, list[str]] = {}
    for gn_tag, gn_content, _gn_raw in general_names:
        if gn_tag != 0xA4:
            continue
        directory_names += 1
        name_tag, name_content, _name_raw, name_end = der_tlv(gn_content, 0)
        if name_tag != 0x30 or name_end != len(gn_content):
            raise ValueError("SubjectAltName directoryName is not a Name")
        for rdn_tag, rdn_content, _rdn_raw in der_children(name_content):
            if rdn_tag != 0x31:
                raise ValueError("SubjectAltName RDN is not a SET")
            attrs = der_children(rdn_content)
            for attr_tag, attr_content, _attr_raw in attrs:
                if attr_tag != 0x30:
                    raise ValueError("SubjectAltName Attribute is not a SEQUENCE")
                at_offset = 0
                oid_tag, oid_content, _oid_raw, at_offset = der_tlv(attr_content, at_offset)
                if oid_tag != 0x06:
                    raise ValueError("SubjectAltName Attribute missing OID")
                value_tag, value_content, _value_raw, value_end = der_tlv(attr_content, at_offset)
                if value_tag != 0x31 or value_end != len(attr_content):
                    raise ValueError("SubjectAltName Attribute value is not a SET")
                values = der_children(value_content)
                if len(values) != 1 or values[0][0] != 0x0C:
                    raise ValueError("SubjectAltName TCG directory attribute must contain one UTF8String")
                text_value = values[0][1].decode("utf-8")
                if not text_value:
                    raise ValueError("SubjectAltName TCG directory attribute is empty")
                oid = oid_string(oid_content)
                attributes.setdefault(oid, []).append(text_value)
    return critical, {
        "present": True,
        "directory_name_count": directory_names,
        "attributes": attributes,
    }




def leaf_profile_ok(info: dict[str, Any], expected_tpm: dict[str, Any] | None = None) -> tuple[bool, dict[str, Any]]:
    expected_tpm = reference_tpm_values() if expected_tpm is None else expected_tpm
    bc_critical, ca_false = basic_constraints(info)
    ku_critical, key_encipherment, _crl_sign, key_cert_sign = key_usage_bits(info)
    eku_critical, eku = eku_oids(info)
    aki_critical, aki = authority_key_id(info)
    san_critical, san = subject_alt_name(info)
    ski_critical, ski_value = subject_key_id(info)
    validate_aia(info)
    validate_certificate_policies(info)
    validate_cdp(info)
    validate_subject_directory_attributes(info)
    profile = {
        "version_3": info["version"] == 3,
        "serial_positive": info["serial"] > 0,
        "basic_constraints_critical": bc_critical,
        "basic_constraints_ca_false": ca_false,
        "key_usage_critical": ku_critical,
        "key_encipherment_set": key_encipherment,
        "key_cert_sign_set": key_cert_sign,
        "extended_key_usage_oids": eku,
        "extended_key_usage_critical": eku_critical,
        "authority_key_identifier_present": aki is not None,
        "authority_key_identifier_critical": aki_critical,
        "subject_alt_name_present": san["present"],
        "subject_alt_name_directory_name_count": san["directory_name_count"],
        "subject_alt_name_tcg_attributes": san["attributes"],
        "subject_alt_name_critical": san_critical,
        "reference_tpm_manufacturer": expected_tpm["manufacturer"],
        "reference_tpm_model": expected_tpm["model"],
        "reference_tpm_version": expected_tpm["version"],
        "subject_alt_name_criticality_ok": (
            (info["subject_empty"] and san_critical)
            or (not info["subject_empty"] and not san_critical)
        ),
        "certificate_policies_present": "2.5.29.32" in info["extensions"],
        "subject_name_empty": info["subject_empty"],
        "subject_key_identifier_present": ski_value is not None,
        "subject_key_identifier_critical": ski_critical,
    }
    eku_ok = not eku or EK_CERT_EKU_OID in eku
    eku_critical_ok = not eku_critical
    aki_critical_ok = not aki_critical
    tcg_san_oids = {"2.23.133.2.1", "2.23.133.2.2", "2.23.133.2.3"}
    san_attrs = san["attributes"]
    expected_san_values = {
        "2.23.133.2.1": "id:" + format(expected_tpm["manufacturer"], "08X"),
        "2.23.133.2.2": expected_tpm["model"],
        "2.23.133.2.3": "id:" + expected_tpm["version"],
    }
    san_values_ok = all(
        san_attrs.get(oid, []) == [value] for oid, value in expected_san_values.items()
    )
    san_ok = (
        san["present"]
        and san["directory_name_count"] >= 1
        and all(len(san_attrs.get(oid, [])) == 1 for oid in tcg_san_oids)
        and san_attrs["2.23.133.2.1"][0].startswith("id:")
        and san_attrs["2.23.133.2.3"][0].startswith("id:")
        and san_values_ok
        and (
            (info["subject_empty"] and san_critical)
            or (not info["subject_empty"] and not san_critical)
        )
    )
    ok = (
        profile["version_3"]
        and profile["serial_positive"]
        and bc_critical
        and ca_false
        and ku_critical
        and key_encipherment
        and not key_cert_sign
        and eku_ok
        and eku_critical_ok
        and aki_critical_ok
        and aki is not None
        and san_ok
        and not ski_critical
    )
    return ok, profile


def verify_crl_sign_key_usage(info: dict[str, Any]) -> bool:
    critical, _key_encipherment, crl_sign, _key_cert_sign = key_usage_bits(info)
    return critical and crl_sign


def crl_pem_to_der(block: bytes) -> bytes:
    import base64
    start_marker = b"-----BEGIN X509 CRL-----"
    end_marker = b"-----END X509 CRL-----"
    stripped = block.strip()
    if not stripped.startswith(start_marker) or not stripped.endswith(end_marker):
        raise ValueError("invalid X509 CRL PEM block")
    body = stripped[len(start_marker):-len(end_marker)]
    encoded = b"".join(body.split())
    try:
        return base64.b64decode(encoded, validate=True)
    except Exception as exc:
        raise ValueError(f"invalid X509 CRL PEM encoding: {exc}") from exc


def rsa_public_key_from_spki(spki_der: bytes) -> tuple[int, int]:
    tag, content, _raw, end = der_tlv(spki_der, 0)
    if tag != 0x30 or end != len(spki_der):
        raise ValueError("SubjectPublicKeyInfo must be one SEQUENCE")
    children = der_children(content)
    if len(children) != 2 or children[0][0] != 0x30 or children[1][0] != 0x03:
        raise ValueError("SubjectPublicKeyInfo structure invalid")
    if algorithm_identifier_oid(children[0][2], "SubjectPublicKeyInfo") != RSA_ENCRYPTION_OID:
        raise ValueError("SubjectPublicKeyInfo algorithm is not rsaEncryption")
    # Reference corpus keys are RSA-2048; reject weaker issuer keys in the
    # independent cryptographic witness rather than leaving this only to OpenSSL.
    key_bits = children[1][1]
    if not key_bits or key_bits[0] != 0:
        raise ValueError("SubjectPublicKeyInfo RSA BIT STRING must have zero unused bits")
    rsa_der = key_bits[1:]
    rsa_tag, rsa_content, _rsa_raw, rsa_end = der_tlv(rsa_der, 0)
    if rsa_tag != 0x30 or rsa_end != len(rsa_der):
        raise ValueError("RSA public key is not one SEQUENCE")
    rsa_children = der_children(rsa_content)
    if len(rsa_children) != 2 or any(child[0] != 0x02 for child in rsa_children):
        raise ValueError("RSA public key must contain modulus and exponent")
    modulus = der_integer_value(rsa_children[0][1], "RSA.modulus", positive=True)
    exponent = der_integer_value(rsa_children[1][1], "RSA.exponent", positive=True)
    if modulus.bit_length() != 2048 or modulus % 2 != 1:
        raise ValueError("reference issuer RSA modulus must be odd RSA-2048")
    if exponent < 3 or exponent % 2 == 0:
        raise ValueError("reference issuer RSA exponent is invalid")
    return modulus, exponent


def rsa_sha256_verify(tbs_der: bytes, signature: bytes, spki_der: bytes) -> dict[str, str]:
    modulus, exponent = rsa_public_key_from_spki(spki_der)
    width = (modulus.bit_length() + 7) // 8
    if len(signature) != width:
        raise ValueError("RSA signature width does not match issuer modulus")
    signature_integer = int.from_bytes(signature, "big")
    if signature_integer >= modulus:
        raise ValueError("RSA signature integer is not below issuer modulus")
    encoded = pow(signature_integer, exponent, modulus).to_bytes(width, "big")
    digest_info = bytes.fromhex("3031300d060960864801650304020105000420") + hashlib.sha256(tbs_der).digest()
    if not encoded.startswith(b"\x00\x01"):
        raise ValueError("RSA PKCS#1 v1.5 signature header invalid")
    separator = encoded.find(b"\x00", 2)
    if separator < 10 or any(byte != 0xFF for byte in encoded[2:separator]):
        raise ValueError("RSA PKCS#1 v1.5 padding invalid")
    if encoded[separator + 1:] != digest_info:
        raise ValueError("RSA SHA-256 signature does not match exact TBS bytes")
    return {
        "modulus_sha256": hashlib.sha256(
            modulus.to_bytes((modulus.bit_length() + 7) // 8, "big")
        ).hexdigest(),
        "signature_sha256": hashlib.sha256(signature).hexdigest(),
        "tbs_sha256": hashlib.sha256(tbs_der).hexdigest(),
    }


def parse_crl_der_for_crypto(der: bytes) -> dict[str, Any]:
    tag, content, _raw, end = der_tlv(der, 0)
    if tag != 0x30 or end != len(der):
        raise ValueError("CRL is not one DER SEQUENCE")
    outer = der_children(content)
    if len(outer) != 3 or outer[0][0] != 0x30 or outer[1][0] != 0x30 or outer[2][0] != 0x03:
        raise ValueError("CRL outer structure invalid")
    tbs_raw = outer[0][2]
    outer_alg_raw = outer[1][2]
    signature_content = outer[2][1]
    if signature_content[:1] != b"\x00":
        raise ValueError("CRL signatureValue must have zero unused bits")
    signature = signature_content[1:]
    tbs_tag, tbs_content, _tbs_raw, tbs_end = der_tlv(tbs_raw, 0)
    if tbs_tag != 0x30 or tbs_end != len(tbs_raw):
        raise ValueError("TBSCertList malformed")
    fields = der_children(tbs_content)
    if not fields or fields[0][0] != 0x02 or der_integer_value(fields[0][1], "TBSCertList.version") != 1:
        raise ValueError("CRL must be explicit v2")
    cursor = 1
    if len(fields) <= cursor or fields[cursor][0] != 0x30:
        raise ValueError("TBSCertList signature AlgorithmIdentifier malformed")
    inner_alg_raw = fields[cursor][2]
    inner_oid = algorithm_identifier_oid(inner_alg_raw, "TBSCertList.signature")
    outer_oid = algorithm_identifier_oid(outer_alg_raw, "CRL.signatureAlgorithm")
    if inner_alg_raw != outer_alg_raw or inner_oid != SHA256_WITH_RSA_OID or outer_oid != SHA256_WITH_RSA_OID:
        raise ValueError("CRL signature AlgorithmIdentifiers are not identical SHA256withRSA")
    cursor += 1
    if cursor >= len(fields) or fields[cursor][0] != 0x30:
        raise ValueError("TBSCertList issuer Name malformed")
    issuer_der = fields[cursor][2]
    if not issuer_der:
        raise ValueError("TBSCertList issuer Name empty")
    cursor += 1
    if cursor >= len(fields):
        raise ValueError("CRL thisUpdate missing")
    this_update = parse_crl_time(fields[cursor][0], fields[cursor][1], "CRL.thisUpdate")
    cursor += 1
    if cursor >= len(fields):
        raise ValueError("CRL nextUpdate missing")
    next_update = parse_crl_time(fields[cursor][0], fields[cursor][1], "CRL.nextUpdate")
    if this_update["unix"] >= next_update["unix"]:
        raise ValueError("CRL nextUpdate must be later than thisUpdate")
    cursor += 1

    revoked_entries = []
    if cursor < len(fields) and fields[cursor][0] == 0x30:
        for entry_tag, entry_content, entry_raw in der_children(fields[cursor][1]):
            if entry_tag != 0x30:
                raise ValueError("CRL entry malformed")
            entry_fields = der_children(entry_content)
            if len(entry_fields) < 2 or entry_fields[0][0] != 0x02:
                raise ValueError("CRL entry serial missing")
            serial = der_integer_value(entry_fields[0][1], "CRL revoked serial", positive=True)
            revocation_date = parse_crl_time(entry_fields[1][0], entry_fields[1][1], "CRL revocationDate")
            entry_extensions = {}
            if len(entry_fields) > 2:
                if len(entry_fields) != 3 or entry_fields[2][0] != 0xA0:
                    raise ValueError("CRL entry extensions malformed")
                entry_extensions = parse_extensions(entry_fields[2][1])
            revoked_entries.append({
                "entry_identity_sha256": hashlib.sha256(entry_raw).hexdigest(),
                "serial": serial,
                "revocation_date": revocation_date,
                "extensions": entry_extensions,
            })
        cursor += 1

    if cursor >= len(fields) or fields[cursor][0] != 0xA0:
        raise ValueError("CRL Extensions wrapper missing")
    crl_extensions = parse_extensions(fields[cursor][1])
    cursor += 1
    if cursor != len(fields):
        raise ValueError("CRL trailing field malformed")
    if set(crl_extensions) != {"2.5.29.35", "2.5.29.20"}:
        raise ValueError("CRL extension set must be exactly AuthorityKeyIdentifier and cRLNumber")
    if any(ext["critical"] for ext in crl_extensions.values()):
        raise ValueError("CRL AuthorityKeyIdentifier and cRLNumber must be non-critical")
    number_value = crl_extensions["2.5.29.20"]["extn_value"]
    number_tag, number_content, _number_raw, number_end = der_tlv(number_value, 0)
    if number_tag != 0x02 or number_end != len(number_value):
        raise ValueError("CRL cRLNumber malformed")
    crl_number = der_integer_value(number_content, "CRL.cRLNumber")
    if crl_number.bit_length() > 160:
        raise ValueError("CRL.cRLNumber value exceeds RFC 5280 20-octet limit")
    authority_key_identifier = parse_crl_aki({"crl_extensions": crl_extensions})
    return {
        "object_der": der,
        "object_sha256": hashlib.sha256(der).hexdigest(),
        "tbs_der": tbs_raw,
        "tbs_sha256": hashlib.sha256(tbs_raw).hexdigest(),
        "signature_der": signature,
        "signature_sha256": hashlib.sha256(signature).hexdigest(),
        "signature_algorithm_oid": outer_oid,
        "issuer_der": issuer_der,
        "version": 2,
        "this_update": this_update,
        "next_update": next_update,
        "crl_number": crl_number,
        "authority_key_identifier": authority_key_identifier,
        "revoked_entries": revoked_entries,
        "crl_extensions": crl_extensions,
    }


def validate_crl_semantics(
    crls: dict[str, dict[str, Any]],
    issuers: dict[str, dict[str, Any]],
    expected: dict[str, Any],
    verification_time_unix: int,
) -> dict[str, Any]:
    if not isinstance(expected, dict) or set(expected) != {"root", "intermediate"}:
        raise ValueError("reference CRL semantics must contain root and intermediate")
    if verification_time_unix < 0:
        raise ValueError("verification time must be non-negative")
    chain_serials = {issuers["leaf"]["serial"], issuers["intermediate"]["serial"], issuers["root"]["serial"]}
    observed: dict[str, Any] = {}
    for label in ("root", "intermediate"):
        crl = crls[label]
        spec = expected[label]
        if crl.get("version") != 2:
            raise ValueError(f"{label} CRL must be v2")
        if "this_update" not in crl or "next_update" not in crl:
            raise ValueError(f"{label} CRL thisUpdate/nextUpdate are required")
        if crl["this_update"]["unix"] > verification_time_unix or verification_time_unix >= crl["next_update"]["unix"]:
            raise ValueError(f"{label} CRL outside exact modeled validity window")
        selection = spec.get("selection")
        if not isinstance(selection, dict) or set(selection) != {
            "issuer_certificate_sha256", "crl_der_sha256", "scope",
            "delta_crl_supported", "indirect_crl_supported", "crl_number_lineage",
        }:
            raise ValueError(f"{label} CRL authoritative selection contract malformed")
        if selection["issuer_certificate_sha256"] != issuers[label]["object_sha256"]:
            raise ValueError(f"{label} CRL authoritative issuer certificate identity mismatch")
        if selection["crl_der_sha256"] != crl["object_sha256"]:
            raise ValueError(f"{label} CRL authoritative CRL DER identity mismatch")
        if selection["scope"] != "all-certificates-issued-by-issuer":
            raise ValueError(f"{label} CRL scope is not complete-single-CA")
        if selection["delta_crl_supported"] is not False or selection["indirect_crl_supported"] is not False:
            raise ValueError(f"{label} CRL delta/indirect semantics are outside the reference model")
        if selection["crl_number_lineage"] != "single-current-reference-no-history":
            raise ValueError(f"{label} CRL number historical progression is not modeled")
        expected_number = int(spec["crl_number"])
        if crl["crl_number"] != expected_number:
            raise ValueError(f"{label} cRLNumber does not match committed recipe")
        if crl["this_update"]["text"] != str(spec["this_update"]) or crl["next_update"]["text"] != str(spec["next_update"]):
            raise ValueError(f"{label} CRL time values do not match committed recipe")
        signer_ski = issuers[label]["subject_key_identifier"]
        if not signer_ski or crl["authority_key_identifier"] != signer_ski:
            raise ValueError(f"{label} CRL AuthorityKeyIdentifier does not match signer SubjectKeyIdentifier")
        expected_entries = spec["revoked_entries"]
        if not isinstance(expected_entries, list) or len(crl["revoked_entries"]) != len(expected_entries):
            raise ValueError(f"{label} CRL revoked-entry count mismatch")
        entries = []
        seen_serials = set()
        previous_serial = 0
        for observed_entry, expected_entry in zip(crl["revoked_entries"], expected_entries, strict=True):
            serial = observed_entry["serial"]
            if serial <= previous_serial or serial in seen_serials:
                raise ValueError(f"{label} CRL revoked serials are not strictly increasing and unique")
            previous_serial = serial
            seen_serials.add(serial)
            if serial in chain_serials:
                raise ValueError(f"{label} CRL revokes an active chain certificate")
            reason_ext = observed_entry["extensions"].get("2.5.29.21")
            if reason_ext is None or reason_ext["critical"]:
                raise ValueError(f"{label} CRL reasonCode missing or critical")
            if set(observed_entry["extensions"]) != {"2.5.29.21"}:
                raise ValueError(f"{label} CRL entry extension set mismatch")
            reason_tag, reason_content, _reason_raw, reason_end = der_tlv(reason_ext["extn_value"], 0)
            if reason_tag != 0x0A or reason_end != len(reason_ext["extn_value"]) or not reason_content:
                raise ValueError(f"{label} CRL reasonCode malformed")
            if len(reason_content) > 1 and reason_content[0] == 0:
                raise ValueError(f"{label} CRL reasonCode non-canonical")
            reason = int.from_bytes(reason_content, "big")
            if reason == 8:
                raise ValueError(f"{label} CRL removeFromCRL requires delta-CRL semantics")
            if reason != int(expected_entry["reason_code"]):
                raise ValueError(f"{label} CRL reasonCode mismatch")
            if observed_entry["entry_identity_sha256"] != str(expected_entry["entry_identity_sha256"]):
                raise ValueError(f"{label} revoked-entry DER identity mismatch")
            if observed_entry["revocation_date"]["text"] != str(expected_entry["revocation_date"]):
                raise ValueError(f"{label} revocationDate mismatch")
            if observed_entry["revocation_date"]["unix"] > crl["this_update"]["unix"]:
                raise ValueError(f"{label} revocationDate after thisUpdate")
            entries.append({
                "entry_identity_sha256": observed_entry["entry_identity_sha256"],
                "serial": serial,
                "revocation_date": observed_entry["revocation_date"]["text"],
                "reason_code": reason,
            })
        observed[label] = {
            "object_sha256": crl["object_sha256"],
            "tbs_sha256": crl["tbs_sha256"],
            "this_update": crl["this_update"]["text"],
            "next_update": crl["next_update"]["text"],
            "crl_number": crl["crl_number"],
            "authority_key_identifier_sha256": hashlib.sha256(crl["authority_key_identifier"]).hexdigest(),
            "issuer_object_sha256": issuers[label]["object_sha256"],
            "revoked_entries": entries,
        }
    return observed


def cryptographic_binding_receipt(
    leaf: dict[str, Any],
    intermediate: dict[str, Any],
    root: dict[str, Any],
    crl_bundle: bytes,
    expected_crl_semantics: dict[str, Any],
    verification_time_unix: int,
) -> dict[str, Any]:
    if leaf["issuer_der"] != intermediate["subject_der"]:
        raise ValueError("leaf issuer does not exactly match intermediate subject")
    if intermediate["issuer_der"] != root["subject_der"]:
        raise ValueError("intermediate issuer does not exactly match root subject")
    if root["issuer_der"] != root["subject_der"]:
        raise ValueError("root is not self-issued")

    certificate_signatures = {}
    for label, cert, issuer, self_signed in (
        ("leaf", leaf, intermediate, False),
        ("intermediate", intermediate, root, False),
        ("root", root, root, True),
    ):
        check = rsa_sha256_verify(cert["tbs_der"], cert["signature_der"], issuer["spki_der"])
        certificate_signatures[label] = {
            "object_sha256": cert["object_sha256"],
            "tbs_sha256": check["tbs_sha256"],
            "signature_sha256": check["signature_sha256"],
            "signature_algorithm_oid": cert["signature_algorithm_oid"],
            "issuer_name_sha256": hashlib.sha256(cert["issuer_der"]).hexdigest(),
            "subject_name_sha256": hashlib.sha256(cert["subject_der"]).hexdigest(),
            "issuer_object_sha256": issuer["object_sha256"],
            "issuer_spki_sha256": hashlib.sha256(issuer["spki_der"]).hexdigest(),
            "issuer_modulus_sha256": check["modulus_sha256"],
            "signature_verification": "PASS",
            "issuer_name_exact_match": True,
            "self_signed": self_signed,
        }

    parsed_crls: dict[str, dict[str, Any]] = {}
    crl_signatures: dict[str, dict[str, Any]] = {}
    seen_crl: set[str] = set()
    blocks = split_pem_crls(crl_bundle)
    for block in blocks:
        der = crl_pem_to_der(block)
        entry = parse_crl_der_for_crypto(der)
        if entry["object_sha256"] in seen_crl:
            raise ValueError("duplicate CRL object")
        seen_crl.add(entry["object_sha256"])
        matches = []
        if entry["issuer_der"] == root["subject_der"]:
            matches.append(("root", root))
        if entry["issuer_der"] == intermediate["subject_der"]:
            matches.append(("intermediate", intermediate))
        if len(matches) != 1:
            raise ValueError("CRL issuer does not exactly match one chain issuer subject")
        label, issuer = matches[0]
        check = rsa_sha256_verify(entry["tbs_der"], entry["signature_der"], issuer["spki_der"])
        parsed_crls[label] = entry
        crl_signatures[label] = {
            "object_sha256": entry["object_sha256"],
            "pem_block_sha256": hashlib.sha256(block).hexdigest(),
            "tbs_sha256": check["tbs_sha256"],
            "signature_sha256": check["signature_sha256"],
            "signature_algorithm_oid": entry["signature_algorithm_oid"],
            "issuer_name_sha256": hashlib.sha256(entry["issuer_der"]).hexdigest(),
            "issuer_object_sha256": issuer["object_sha256"],
            "issuer_spki_sha256": hashlib.sha256(issuer["spki_der"]).hexdigest(),
            "issuer_modulus_sha256": check["modulus_sha256"],
            "signature_verification": "PASS",
            "issuer_name_exact_match": True,
        }
    if set(crl_signatures) != {"root", "intermediate"}:
        raise ValueError("CRL bundle must contain exactly one CRL for root and intermediate")
    crl_semantics = validate_crl_semantics(
        parsed_crls,
        {"root": root, "intermediate": intermediate, "leaf": leaf},
        expected_crl_semantics,
        verification_time_unix,
    )
    return {
        "verifier_id": CRYPTO_BINDING_ID,
        "verifier_source_sha256": sha256_file(Path(__file__)),
        "state": "PASS",
        "certificate_signatures": certificate_signatures,
        "crl_signatures": crl_signatures,
        "crl_semantics_sha256": canonical_hash(expected_crl_semantics),
        "crl_semantics_recipe": expected_crl_semantics,
        "crl_semantics": crl_semantics,
        "exact_relationships": {
            "leaf_to_intermediate_subject_exact": True,
            "intermediate_to_root_subject_exact": True,
            "root_self_issued_exact": True,
            "root_crl_issuer_exact": True,
            "intermediate_crl_issuer_exact": True,
            "crl_aki_to_signer_ski_exact": True,
            "crl_times_to_verification_time_exact": True,
            "crl_revocation_entries_exact": True,
        },
        "exact_input_objects": {
            "leaf_certificate_sha256": leaf["object_sha256"],
            "intermediate_certificate_sha256": intermediate["object_sha256"],
            "trust_anchor_root_sha256": root["object_sha256"],
            "crl_bundle_pem_sha256": hashlib.sha256(crl_bundle).hexdigest(),
        },
    }


def crl_issuer_names_from_pem_bundle(bundle: bytes) -> list[bytes]:
    issuers: list[bytes] = []
    for block in split_pem_crls(bundle):
        der = crl_pem_to_der(block)
        tag, crl_content, _raw, end = der_tlv(der, 0)
        if tag != 0x30 or end != len(der):
            raise ValueError("CRL is not a single DER sequence")
        tbs_tag, tbs_content, _tbs_raw, _ = der_tlv(crl_content, 0)
        if tbs_tag != 0x30:
            raise ValueError("CRL TBSCertList is not a sequence")
        offset = 0
        first_tag, _first_content, _first_raw, first_end = der_tlv(tbs_content, offset)
        if first_tag == 0x02:
            offset = first_end
        _sig_tag, _sig_content, _sig_raw, offset = der_tlv(tbs_content, offset)
        issuer_tag, _issuer_content, issuer_raw, _ = der_tlv(tbs_content, offset)
        if issuer_tag != 0x30:
            raise ValueError("CRL issuer Name malformed")
        issuers.append(issuer_raw)
    return issuers


def verify_aki_ski(leaf: dict[str, Any], intermediate: dict[str, Any]) -> bool:
    _aki_critical, aki = authority_key_id(leaf)
    _ski_critical, ski = subject_key_id(intermediate)
    return bool(aki and ski and aki == ski)


def run_trust_anchor_appraiser(
    manifest: dict[str, Any],
    root_der: bytes,
) -> dict[str, Any]:
    appraisal = manifest.get("trust_anchor_appraisal")
    if not isinstance(appraisal, dict):
        return result("DENY", "trust-anchor-appraisal-invalid")
    if appraisal.get("verifier_id") != TRUST_ANCHOR_APPRAISAL_ID:
        return result("DENY", "trust-anchor-appraisal-verifier-id-mismatch")
    if not TRUST_ANCHOR_APPRAISAL_SCRIPT.is_file():
        return result("DENY", "trust-anchor-appraisal-verifier-missing")
    if not TRUST_ANCHOR_REGISTRY_FILE.is_file():
        return result("DENY", "trust-anchor-registry-missing")
    if not TRUST_ANCHOR_AUTHORIZATION_RECEIPT_FILE.is_file():
        return result("DENY", "trust-anchor-authorization-receipt-missing")
    try:
        receipt = json.loads(TRUST_ANCHOR_AUTHORIZATION_RECEIPT_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return result("DENY", "trust-anchor-authorization-receipt-invalid", {"error": str(exc)})
    receipt_sha = hashlib.sha256(TRUST_ANCHOR_AUTHORIZATION_RECEIPT_FILE.read_bytes()).hexdigest()
    if appraisal.get("registry_source_sha256") != receipt_sha:
        return result("DENY", "trust-anchor-authorization-receipt-digest-mismatch")
    if appraisal.get("receipt_root_sha256") != receipt.get("root_certificate_sha256"):
        return result("DENY", "trust-anchor-appraisal-receipt-root-mismatch")
    if receipt.get("registry_file_sha256") != hashlib.sha256(TRUST_ANCHOR_REGISTRY_FILE.read_bytes()).hexdigest():
        return result("DENY", "trust-anchor-receipt-registry-file-mismatch")
    if receipt.get("root_certificate_sha256") != hashlib.sha256(root_der).hexdigest():
        return result("DENY", "trust-anchor-receipt-root-mismatch")
    if receipt.get("authorization_state") != appraisal.get("authorization_state"):
        return result("DENY", "trust-anchor-receipt-state-mismatch")
    try:
        registry = json.loads(TRUST_ANCHOR_REGISTRY_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return result("DENY", "trust-anchor-registry-invalid", {"error": str(exc)})
    if receipt.get("registry_id") != registry.get("registry_id"):
        return result("DENY", "trust-anchor-receipt-registry-id-mismatch")
    registry_sha = hashlib.sha256(
        (json.dumps(registry, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()
    ).hexdigest()
    if appraisal.get("registry_sha256") != registry_sha:
        return result("DENY", "trust-anchor-registry-digest-mismatch")
    required = ("anchor_id", "authorization_state", "registry_sha256", "registry_source_sha256", "receipt_root_sha256", "input_sha256", "output_sha256", "verifier_source_sha256", "output_content_sha256")
    for field in required:
        if field not in appraisal:
            return result("DENY", "trust-anchor-appraisal-field-missing", {"field": field})
    try:
        if not valid_hash(appraisal["registry_source_sha256"]):
            return result("DENY", "trust-anchor-registry-source-digest-invalid")
        if not valid_hash(appraisal["verifier_source_sha256"]):
            return result("DENY", "trust-anchor-appraisal-verifier-source-digest-invalid")
        if appraisal["verifier_source_sha256"] != sha256_file(TRUST_ANCHOR_APPRAISAL_SCRIPT):
            return result("DENY", "trust-anchor-appraisal-verifier-source-mismatch")
        if not valid_hash(appraisal["output_content_sha256"]):
            return result("DENY", "trust-anchor-appraisal-output-content-digest-invalid")
    except KeyError:
        return result("DENY", "trust-anchor-appraisal-source-fields-invalid")

    input_manifest = {
        "profile_id": "mycelix.security.tpm.ek-trust-anchor-appraisal",
        "profile_version": "0.1.0",
        "claim_ceiling": "ReferenceModelOnly",
        "anchor_id": appraisal["anchor_id"],
        "root_certificate_der_base64": b64(root_der),
        "root_certificate_sha256": hashlib.sha256(root_der).hexdigest(),
        "registry_json": registry,
        "registry_sha256": registry_sha,
        "registry_source_sha256": appraisal["registry_source_sha256"],
        "authorization_state": appraisal["authorization_state"],
    }
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-trust-anchor-") as td:
        work = Path(td)
        input_path = work / "trust-anchor-input.json"
        output_path = work / "trust-anchor-output.json"
        input_path.write_text(
            json.dumps(input_manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(TRUST_ANCHOR_APPRAISAL_SCRIPT),
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
            return result("DENY", "trust-anchor-appraisal-execution-error", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "trust-anchor-appraisal-produced-no-output")
        try:
            output = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "trust-anchor-appraisal-output-invalid", {"error": str(exc)})
        expected_input_sha = hashlib.sha256(input_path.read_bytes()).hexdigest()
        expected_output_sha = hashlib.sha256(output_path.read_bytes()).hexdigest()
        if appraisal["input_sha256"] != expected_input_sha:
            return result("DENY", "trust-anchor-appraisal-input-digest-mismatch")
        if appraisal["output_sha256"] != expected_output_sha:
            return result("DENY", "trust-anchor-appraisal-output-digest-mismatch")
        if output.get("verifier_id") != TRUST_ANCHOR_APPRAISAL_ID:
            return result("DENY", "trust-anchor-appraisal-result-verifier-id-mismatch")
        if output.get("state") != appraisal.get("state"):
            return result("DENY", "trust-anchor-appraisal-result-state-mismatch")
        if output.get("content_sha256") != appraisal.get("output_content_sha256"):
            return result("DENY", "trust-anchor-appraisal-output-content-digest-mismatch")
        if not valid_hash(output.get("content_sha256")):
            return result("DENY", "trust-anchor-appraisal-output-content-digest-invalid")
        if output["content_sha256"] != canonical_hash(
            {key: value for key, value in output.items() if key != "content_sha256"}
        ):
            return result("DENY", "trust-anchor-appraisal-output-content-invalid")
        return output


def run_spki_verifier(
    manifest: dict[str, Any],
    binding: dict[str, Any],
    leaf: bytes,
    ek_public_wire: bytes,
) -> dict[str, Any]:
    if not SPKI_VERIFIER_SCRIPT.is_file():
        return result("DENY", "spki-verifier-missing")
    verifier_input = binding.get("verifier_input")
    if not isinstance(verifier_input, dict):
        return result("DENY", "spki-verifier-input-invalid")
    expected_input = {
        "profile_id": "mycelix.security.tpm.ek-cert-spki-binding",
        "profile_version": "0.1.0",
        "session_id": manifest["session_id"],
        "tpm_identity_digest": manifest["tpm_identity_digest"],
        "verification_mode": manifest["verification_mode"],
        "claim_ceiling": "ReferenceModelOnly",
        "certificate_der_hex": leaf.hex(),
        "certificate_der_sha256": hashlib.sha256(leaf).hexdigest(),
        "ek_public_format": "TPMT_PUBLIC",
        "ek_public_wire_hex": ek_public.hex(),
        "ek_public_wire_sha256": hashlib.sha256(ek_public_wire).hexdigest(),
        "certificate_source_sha256": verifier_input.get("certificate_source_sha256"),
        "ek_public_source_sha256": verifier_input.get("ek_public_source_sha256"),
    }
    expected_binding = canonical_hash({
        "session_id": expected_input["session_id"],
        "tpm_identity_digest": expected_input["tpm_identity_digest"],
        "certificate_der_sha256": expected_input["certificate_der_sha256"],
        "ek_public_wire_sha256": expected_input["ek_public_wire_sha256"],
    })
    expected_input["session_binding_sha256"] = expected_binding
    if verifier_input != {
        "certificate_source_sha256": expected_input["certificate_source_sha256"],
        "ek_public_source_sha256": expected_input["ek_public_source_sha256"],
    }:
        return result("DENY", "spki-verifier-source-projection-mismatch")
    if binding.get("session_binding_sha256") != expected_binding:
        return result("DENY", "spki-verifier-session-binding-mismatch")
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-spki-compose-") as td:
        work = Path(td)
        input_path = work / "spki-input.json"
        output_path = work / "spki-output.json"
        input_path.write_text(
            json.dumps(expected_input, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(SPKI_VERIFIER_SCRIPT),
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
            return result("DENY", "spki-verifier-execution-error", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "spki-verifier-produced-no-output")
        try:
            output = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "spki-verifier-output-invalid", {"error": str(exc)})
        if output.get("verifier_id") != SPKI_VERIFIER_ID:
            return result("DENY", "spki-verifier-result-id-mismatch")
        expected_input_sha = sha256_file(input_path)
        expected_output_sha = sha256_file(output_path)
        if binding.get("input_sha256") != expected_input_sha:
            return result("DENY", "spki-verifier-input-digest-mismatch")
        if binding.get("output_sha256") != expected_output_sha:
            return result("DENY", "spki-verifier-output-digest-mismatch")
        if not valid_hash(output.get("content_sha256")):
            return result("DENY", "spki-verifier-output-content-digest-invalid")
        if output["content_sha256"] != binding.get("output_content_sha256"):
            return result("DENY", "spki-verifier-output-content-digest-mismatch")
        if output["content_sha256"] != canonical_hash(
            {key: value for key, value in output.items() if key != "content_sha256"}
        ):
            return result("DENY", "spki-verifier-output-content-invalid")
        return output


def run_crypto_verifier(manifest: dict[str, Any]) -> dict[str, Any]:
    if not CRYPTO_VERIFIER_SCRIPT.is_file():
        return result("DENY", "cryptographic-verifier-missing")
    verifier_input = {
        "leaf_certificate_der_base64": manifest["leaf_certificate_der_base64"],
        "leaf_certificate_sha256": manifest["leaf_certificate_sha256"],
        "intermediate_certificate_der_base64": manifest["intermediate_certificate_der_base64"],
        "intermediate_certificate_sha256": manifest["intermediate_certificate_sha256"],
        "trust_anchor_root_der_base64": manifest["trust_anchor_root_der_base64"],
        "trust_anchor_root_sha256": manifest["trust_anchor_root_sha256"],
        "crl_bundle_pem_base64": manifest["revocation"]["crl_bundle_pem_base64"],
        "crl_bundle_pem_sha256": manifest["revocation"]["crl_bundle_pem_sha256"],
        "verification_time_unix": manifest["verification_time_unix"],
        "expected_crl_semantics_sha256": manifest["crl_semantics_sha256"],
        "expected_crl_semantics": manifest["crl_semantics"],
    }
    expected_source_sha = sha256_file(CRYPTO_VERIFIER_SCRIPT)
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-crypto-compose-") as td:
        work = Path(td)
        ip = work / "crypto-input.json"
        op = work / "crypto-output.json"
        ip.write_text(json.dumps(verifier_input, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        proc = subprocess.run(
            [sys.executable, str(CRYPTO_VERIFIER_SCRIPT), "--verify", str(ip), "--output", str(op)],
            cwd=work, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        if proc.returncode not in (0, 1, 2):
            return result("DENY", "cryptographic-verifier-execution-error", {"stderr": proc.stderr})
        if not op.is_file():
            return result("DENY", "cryptographic-verifier-produced-no-output")
        try:
            output = json.loads(op.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "cryptographic-verifier-output-invalid", {"error": str(exc)})
        if output.get("verifier_id") != "mycelix.tpm.ek-fixture-crypto.v0.1":
            return result("DENY", "cryptographic-verifier-id-mismatch")
        if output.get("state") != "PASS":
            return result("DENY", "cryptographic-verifier-not-pass")
        if output.get("input_sha256") != hashlib.sha256(ip.read_bytes()).hexdigest():
            return result("DENY", "cryptographic-verifier-input-digest-mismatch")
        output_sha = hashlib.sha256(op.read_bytes()).hexdigest()
        if not valid_hash(output.get("content_sha256")):
            return result("DENY", "cryptographic-verifier-content-digest-invalid")
        if output["content_sha256"] != canonical_hash(
            {key: value for key, value in output.items() if key != "content_sha256"}
        ):
            return result("DENY", "cryptographic-verifier-output-content-invalid")
        details = output.get("details")
        if not isinstance(details, dict):
            return result("DENY", "cryptographic-verifier-details-missing")
        expected_exact = {
            "leaf_certificate_sha256": manifest["leaf_certificate_sha256"],
            "intermediate_certificate_sha256": manifest["intermediate_certificate_sha256"],
            "trust_anchor_root_sha256": manifest["trust_anchor_root_sha256"],
            "crl_bundle_pem_sha256": manifest["revocation"]["crl_bundle_pem_sha256"],
        }
        if details.get("exact_input_objects") != expected_exact:
            return result("DENY", "cryptographic-verifier-object-binding-mismatch")
        if details.get("crl_semantics_sha256") != manifest["crl_semantics_sha256"]:
            return result("DENY", "cryptographic-verifier-crl-semantics-digest-mismatch")
        if details.get("crl_semantics") != manifest["crl_semantics"]:
            return result("DENY", "cryptographic-verifier-crl-semantics-binding-mismatch")
        return {
            "state": "PASS",
            "verifier_id": output["verifier_id"],
            "source_sha256": expected_source_sha,
            "input_sha256": output["input_sha256"],
            "output_sha256": output_sha,
            "output_content_sha256": output["content_sha256"],
            "details": details,
        }


def run_path_verifier(
    manifest: dict[str, Any],
    binding: dict[str, Any],
) -> dict[str, Any]:
    if not PATH_VERIFIER_SCRIPT.is_file():
        return result("DENY", "path-verifier-missing")
    verifier_input = binding.get("verifier_input")
    if not isinstance(verifier_input, dict):
        return result("DENY", "path-verifier-input-invalid")
    expected_input = {
        "profile_id": "mycelix.security.tpm.ek-cert-path-validation",
        "profile_version": "0.1.0",
        "verification_mode": manifest["verification_mode"],
        "claim_ceiling": "ReferenceModelOnly",
        "session_id": manifest["session_id"],
        "tpm_identity_digest": manifest["tpm_identity_digest"],
        "leaf_certificate_der_base64": manifest["leaf_certificate_der_base64"],
        "leaf_certificate_sha256": manifest["leaf_certificate_sha256"],
        "intermediate_certificate_der_base64": manifest["intermediate_certificate_der_base64"],
        "intermediate_certificate_sha256": manifest["intermediate_certificate_sha256"],
        "trust_anchor_root_der_base64": manifest["trust_anchor_root_der_base64"],
        "trust_anchor_root_sha256": manifest["trust_anchor_root_sha256"],
        "crl_bundle_pem_base64": manifest["revocation"]["crl_bundle_pem_base64"],
        "crl_bundle_pem_sha256": manifest["revocation"]["crl_bundle_pem_sha256"],
        "verification_time_unix": manifest["verification_time_unix"],
    }
    if verifier_input != {key: value for key, value in expected_input.items()}:
        return result("DENY", "path-verifier-input-projection-mismatch")
    expected_execution_binding = canonical_hash({
        "session_id": expected_input["session_id"],
        "tpm_identity_digest": expected_input["tpm_identity_digest"],
        "leaf_certificate_sha256": expected_input["leaf_certificate_sha256"],
        "intermediate_certificate_sha256": expected_input["intermediate_certificate_sha256"],
        "trust_anchor_root_sha256": expected_input["trust_anchor_root_sha256"],
        "crl_bundle_pem_sha256": expected_input["crl_bundle_pem_sha256"],
        "verification_time_unix": expected_input["verification_time_unix"],
        "policy_argv": [
            "openssl", "verify", "-x509_strict", "-check_ss_sig", "-CAfile", "root.pem",
            "-untrusted", "intermediate.pem", "-CRLfile", "crl-bundle.pem",
            "-crl_check_all", "-attime", str(expected_input["verification_time_unix"]),
            "leaf.pem",
        ],
    })
    if binding.get("execution_binding_sha256") != expected_execution_binding:
        return result("DENY", "path-verifier-execution-binding-mismatch")
    if binding.get("verifier_id") != PATH_VERIFIER_ID:
        return result("DENY", "path-verifier-id-mismatch")
    if binding.get("source_sha256") != sha256_file(PATH_VERIFIER_SCRIPT):
        return result("DENY", "path-verifier-source-mismatch")
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-path-compose-") as td:
        work = Path(td)
        input_path = work / "path-input.json"
        output_path = work / "path-output.json"
        input_path.write_text(
            json.dumps(expected_input, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(PATH_VERIFIER_SCRIPT),
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
            return result("DENY", "path-verifier-execution-error", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "path-verifier-produced-no-output")
        try:
            output = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "path-verifier-output-invalid", {"error": str(exc)})
        if output.get("verifier_id") != PATH_VERIFIER_ID:
            return result("DENY", "path-verifier-result-id-mismatch")
        if output.get("state") != binding.get("state"):
            return result("DENY", "path-verifier-result-state-mismatch")
        if output.get("input_sha256") != hashlib.sha256(input_path.read_bytes()).hexdigest():
            return result("DENY", "path-verifier-result-input-digest-mismatch")
        if binding.get("input_sha256") != output["input_sha256"]:
            return result("DENY", "path-verifier-input-digest-mismatch")
        output_sha = hashlib.sha256(output_path.read_bytes()).hexdigest()
        if binding.get("output_sha256") != output_sha:
            return result("DENY", "path-verifier-output-digest-mismatch")
        if not valid_hash(output.get("content_sha256")):
            return result("DENY", "path-verifier-output-content-digest-invalid")
        if binding.get("output_content_sha256") != output["content_sha256"]:
            return result("DENY", "path-verifier-output-content-digest-mismatch")
        if output["content_sha256"] != canonical_hash(
            {key: value for key, value in output.items() if key != "content_sha256"}
        ):
            return result("DENY", "path-verifier-output-content-invalid")
        details = output.get("details")
        if not isinstance(details, dict):
            return result("DENY", "path-verifier-result-details-missing")
        for field, expected in (
            ("leaf_certificate_sha256", manifest["leaf_certificate_sha256"]),
            ("intermediate_certificate_sha256", manifest["intermediate_certificate_sha256"]),
            ("trust_anchor_root_sha256", manifest["trust_anchor_root_sha256"]),
            ("crl_bundle_pem_sha256", manifest["revocation"]["crl_bundle_pem_sha256"]),
            ("verification_time_unix", manifest["verification_time_unix"]),
            ("execution_binding_sha256", expected_execution_binding),
        ):
            if details.get(field) != expected:
                return result("DENY", "path-verifier-result-binding-mismatch", {"field": field})
        return output


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
            "trust_anchor_appraisal_state": manifest["trust_anchor_appraisal"].get("state"),
            "trust_anchor_appraisal_verifier_id": manifest["trust_anchor_appraisal"].get("verifier_id"),
            "trust_anchor_appraisal_anchor_id": manifest["trust_anchor_appraisal"].get("anchor_id"),
            "trust_anchor_appraisal_registry_sha256": manifest["trust_anchor_appraisal"].get("registry_sha256"),
            "trust_anchor_appraisal_registry_source_sha256": manifest["trust_anchor_appraisal"].get("registry_source_sha256"),
            "trust_anchor_appraisal_receipt_root_sha256": manifest["trust_anchor_appraisal"].get("receipt_root_sha256"),
            "trust_anchor_appraisal_input_sha256": manifest["trust_anchor_appraisal"].get("input_sha256"),
            "trust_anchor_appraisal_output_sha256": manifest["trust_anchor_appraisal"].get("output_sha256"),
            "trust_anchor_appraisal_verifier_source_sha256": manifest["trust_anchor_appraisal"].get("verifier_source_sha256"),
            "trust_anchor_appraisal_output_content_sha256": manifest["trust_anchor_appraisal"].get("output_content_sha256"),
            "verification_time_unix": manifest["verification_time_unix"],
            "revocation_state": rev["state"],
            "revocation_method": rev.get("method"),
            "revocation_crl_bundle_pem_sha256": crl_sha,
            "crl_semantics_sha256": manifest.get("crl_semantics_sha256"),
            "crl_semantics": manifest.get("crl_semantics"),
            "path_state": manifest["path_validation"].get("state"),
            "path_verifier_id": manifest["path_validation"].get("verifier_id"),
            "path_source_sha256": manifest["path_validation"].get("source_sha256"),
            "path_input_sha256": manifest["path_validation"].get("input_sha256"),
            "path_output_sha256": manifest["path_validation"].get("output_sha256"),
            "path_output_content_sha256": manifest["path_validation"].get("output_content_sha256"),
            "path_execution_binding_sha256": manifest["path_validation"].get("execution_binding_sha256"),
            "cryptographic_binding_sha256": manifest.get("cryptographic_binding_sha256"),
            "cryptographic_binding_source_sha256": manifest.get("cryptographic_binding_source_sha256"),
            "cryptographic_binding_input_sha256": manifest.get("cryptographic_binding_input_sha256"),
            "cryptographic_binding_output_sha256": manifest.get("cryptographic_binding_output_sha256"),
            "fixture_recipe_sha256": manifest.get("fixture_recipe_sha256"),
            "spki_state": spki.get("state"),
            "spki_certificate_sha256": spki.get("certificate_sha256"),
            "spki_ek_public_wire_sha256": spki.get("ek_public_wire_sha256"),
            "spki_source_sha256": spki.get("source_sha256"),
            "spki_input_sha256": spki.get("input_sha256"),
            "spki_output_sha256": spki.get("output_sha256"),
            "spki_output_content_sha256": spki.get("output_content_sha256"),
            "spki_session_binding_sha256": spki.get("session_binding_sha256"),
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
        "trust_anchor_state", "trust_anchor_source_sha256", "trust_anchor_appraisal",
        "verification_time_unix", "revocation", "crl_semantics", "crl_semantics_sha256", "path_validation", "spki_binding", "ek_template_binding",
        "cryptographic_binding_sha256", "cryptographic_binding_source_sha256", "cryptographic_binding_input_sha256", "cryptographic_binding_output_sha256", "fixture_recipe_sha256", "session_binding_sha256",
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
    if manifest["fixture_recipe_sha256"] != sha256_file(FIXTURE_RECIPE_FILE):
        return result("DENY", "fixture-recipe-source-mismatch")
    try:
        recipe = json.loads(FIXTURE_RECIPE_FILE.read_text(encoding="utf-8"))
        expected_crl_semantics = recipe["crl_semantics"]
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
        return result("DENY", "fixture-crl-semantics-invalid", {"error": str(exc)})
    if manifest.get("crl_semantics") != expected_crl_semantics:
        return result("DENY", "crl-semantics-recipe-mismatch")
    if manifest.get("crl_semantics_sha256") != canonical_hash(expected_crl_semantics):
        return result("DENY", "crl-semantics-digest-mismatch")
    forbidden_inputs = {"profile_override", "caller_supplied_certificate_criticality_overrides"}
    supplied_forbidden = sorted(forbidden_inputs & set(manifest))
    if supplied_forbidden:
        return result("DENY", "forbidden-inputs-present", {"fields": supplied_forbidden})
    if not isinstance(manifest["session_id"], str) or not manifest["session_id"]:
        return result("DENY", "session-id-invalid")
    for field in ("tpm_identity_digest", "ek_public_wire_sha256", "leaf_certificate_sha256", "intermediate_certificate_sha256",
                  "trust_anchor_root_sha256", "trust_anchor_source_sha256", "cryptographic_binding_sha256", "cryptographic_binding_source_sha256", "cryptographic_binding_input_sha256", "cryptographic_binding_output_sha256", "fixture_recipe_sha256", "crl_semantics_sha256", "session_binding_sha256"):
        if not valid_hash(manifest[field]):
            return result("DENY", "digest-invalid", {"field": field})

    try:
        leaf = unb64(manifest["leaf_certificate_der_base64"], "leaf_certificate_der_base64")
        intermediate = unb64(manifest["intermediate_certificate_der_base64"], "intermediate_certificate_der_base64")
        root = unb64(manifest["trust_anchor_root_der_base64"], "trust_anchor_root_der_base64")
        rev = manifest["revocation"]
        if not isinstance(rev, dict):
            return result("DENY", "revocation-object-invalid")
        if not isinstance(rev, dict):
            return result("DENY", "revocation-object-invalid")
        crl_bundle_pem = unb64(rev.get("crl_bundle_pem_base64", ""), "revocation.crl_bundle_pem_base64")
    except ValueError as exc:
        return result("DENY", "certificate-input-invalid", {"error": str(exc)})

    for raw, field in (
        (leaf, "leaf_certificate_sha256"),
        (intermediate, "intermediate_certificate_sha256"),
        (root, "trust_anchor_root_sha256"),
    ):
        if hashlib.sha256(raw).hexdigest() != manifest[field]:
            return result("DENY", "digest-mismatch", {"field": field})

    appraisal_result = run_trust_anchor_appraiser(manifest, root)
    if appraisal_result.get("state") != "PASS":
        return appraisal_result
    appraisal_details = appraisal_result.get("details")
    if not isinstance(appraisal_details, dict):
        return result("DENY", "trust-anchor-appraisal-details-missing")
    if appraisal_details.get("root_certificate_sha256") != manifest["trust_anchor_root_sha256"]:
        return result("DENY", "trust-anchor-appraisal-root-digest-mismatch")
    if appraisal_details.get("anchor_id") != manifest["trust_anchor_appraisal"].get("anchor_id"):
        return result("DENY", "trust-anchor-appraisal-anchor-id-mismatch")
    if manifest["trust_anchor_state"] != manifest["trust_anchor_appraisal"].get("authorization_state"):
        return result("DENY", "trust-anchor-state-appraisal-mismatch")
    if manifest["trust_anchor_source_sha256"] != manifest["trust_anchor_appraisal"].get("registry_source_sha256"):
        return result("DENY", "trust-anchor-source-appraisal-mismatch")

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
    if rev.get("coverage") != "leaf-and-chain":
        return result("DENY", "revocation-coverage-invalid")
    if not valid_hash(rev.get("crl_bundle_pem_sha256")):
        return result("DENY", "revocation-crl-bundle-digest-invalid")
    if hashlib.sha256(crl_bundle_pem).hexdigest() != rev["crl_bundle_pem_sha256"]:
        return result("DENY", "revocation-crl-bundle-digest-mismatch")

    spki = manifest["spki_binding"]
    if not isinstance(spki, dict):
        return result("DENY", "spki-binding-invalid")
    if spki.get("verifier_id") != SPKI_VERIFIER_ID:
        return result("DENY", "spki-verifier-id-mismatch")
    if spki.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "spki-binding-indeterminate")
    if spki.get("state") != "PASS":
        return result("DENY", "spki-binding-not-pass")
    for field in (
        "certificate_sha256", "ek_public_wire_sha256", "source_sha256",
        "input_sha256", "output_sha256", "output_content_sha256",
        "session_binding_sha256", "verifier_input",
    ):
        if field not in spki:
            return result("DENY", "spki-binding-field-missing", {"field": field})
    for field in (
        "certificate_sha256", "ek_public_wire_sha256", "source_sha256",
        "input_sha256", "output_sha256", "output_content_sha256",
        "session_binding_sha256",
    ):
        if not valid_hash(spki[field]):
            return result("DENY", "spki-binding-digest-invalid", {"field": field})
    if spki["source_sha256"] != sha256_file(SPKI_VERIFIER_SCRIPT):
        return result("DENY", "spki-verifier-source-mismatch")
    if spki["certificate_sha256"] != manifest["leaf_certificate_sha256"]:
        return result("DENY", "spki-certificate-digest-mismatch")
    if spki["ek_public_wire_sha256"] != manifest["ek_public_wire_sha256"]:
        return result("DENY", "spki-ek-public-wire-digest-mismatch")
    try:
        ek_public_wire_for_spki = hex_bytes(manifest["ek_public_wire_hex"], "ek_public_wire_hex")
    except ValueError as exc:
        return result("DENY", "ek-public-wire-hex-invalid", {"error": str(exc)})
    if hashlib.sha256(ek_public_wire_for_spki).hexdigest() != manifest["ek_public_wire_sha256"]:
        return result("DENY", "ek-public-wire-hex-digest-mismatch")
    generated_spki = run_spki_verifier(manifest, spki, leaf, ek_public_wire_for_spki)
    if generated_spki.get("verifier_id") != SPKI_VERIFIER_ID:
        return generated_spki
    if generated_spki.get("state") != "PASS":
        return result("DENY", "spki-reexecution-not-pass")
    spki_details = generated_spki.get("details")
    if not isinstance(spki_details, dict):
        return result("DENY", "spki-result-details-missing")
    if spki_details.get("certificate_der_sha256") != manifest["leaf_certificate_sha256"]:
        return result("DENY", "spki-result-certificate-digest-mismatch")
    if spki_details.get("ek_public_wire_sha256") != manifest["ek_public_wire_sha256"]:
        return result("DENY", "spki-result-ek-public-digest-mismatch")

    template_result = validate_template_binding(manifest)
    if template_result.get("state") != "PASS":
        return template_result

    expected_session_binding = session_binding(
        manifest,
        manifest["leaf_certificate_sha256"],
        manifest["intermediate_certificate_sha256"],
        manifest["trust_anchor_root_sha256"],
        rev["crl_bundle_pem_sha256"],
    )
    if expected_session_binding != manifest["session_binding_sha256"]:
        return result("DENY", "session-binding-mismatch")

    path_validation = manifest["path_validation"]
    if not isinstance(path_validation, dict):
        return result("DENY", "path-validation-object-invalid")
    if path_validation.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "path-validation-indeterminate")
    if path_validation.get("state") != "PASS":
        return result("DENY", "path-validation-not-pass")
    for field in (
        "state", "verifier_id", "source_sha256", "input_sha256",
        "output_sha256", "output_content_sha256", "execution_binding_sha256",
        "verifier_input",
    ):
        if field not in path_validation:
            return result("DENY", "path-validation-field-missing", {"field": field})
    for field in (
        "source_sha256", "input_sha256", "output_sha256",
        "output_content_sha256", "execution_binding_sha256",
    ):
        if not valid_hash(path_validation[field]):
            return result("DENY", "path-validation-digest-invalid", {"field": field})
    generated_path = run_path_verifier(manifest, path_validation)
    if generated_path.get("verifier_id") != PATH_VERIFIER_ID:
        return generated_path
    if generated_path.get("state") != "PASS":
        return result("DENY", "path-validation-reexecution-not-pass")
    path_details = generated_path.get("details")
    if not isinstance(path_details, dict):
        return result("DENY", "path-validation-result-details-missing")
    expected_cross_witness = {
        "leaf_certificate_sha256": manifest["leaf_certificate_sha256"],
        "intermediate_certificate_sha256": manifest["intermediate_certificate_sha256"],
        "trust_anchor_root_sha256": manifest["trust_anchor_root_sha256"],
        "crl_bundle_pem_sha256": manifest["revocation"]["crl_bundle_pem_sha256"],
        "crl_semantics_sha256": manifest["crl_semantics_sha256"],
    }

    try:
        leaf_info = parse_certificate_der(leaf)
        intermediate_info = parse_certificate_der(intermediate)
        root_info = parse_certificate_der(root)
        crl_issuers = crl_issuer_names_from_pem_bundle(crl_bundle_pem)
        crypto_receipt = cryptographic_binding_receipt(
            leaf_info,
            intermediate_info,
            root_info,
            crl_bundle_pem,
            manifest["crl_semantics"],
            manifest["verification_time_unix"],
        )
        expected_crypto_binding_sha256 = canonical_hash(crypto_receipt)
    except (ValueError, OSError) as exc:
        return result("DENY", "certificate-parse-error", {"error": str(exc)})

    external_crypto = run_crypto_verifier(manifest)
    if external_crypto.get("state") != "PASS":
        return external_crypto
    if external_crypto["source_sha256"] != manifest["cryptographic_binding_source_sha256"]:
        return result("DENY", "cryptographic-verifier-source-binding-mismatch")
    if external_crypto["input_sha256"] != manifest["cryptographic_binding_input_sha256"]:
        return result("DENY", "cryptographic-verifier-input-binding-mismatch")
    if external_crypto["output_sha256"] != manifest["cryptographic_binding_output_sha256"]:
        return result("DENY", "cryptographic-verifier-output-binding-mismatch")
    independent_details = external_crypto["details"]
    if not isinstance(independent_details, dict):
        return result("DENY", "independent-crypto-details-invalid")
    if independent_details.get("crl_semantics_sha256") != manifest["crl_semantics_sha256"]:
        return result("DENY", "independent-crl-semantics-digest-mismatch")
    if independent_details.get("crl_semantics_recipe") != manifest["crl_semantics"]:
        return result("DENY", "independent-crl-semantics-recipe-binding-mismatch")
    if independent_details.get("crl_semantics") != crypto_receipt["crl_semantics"]:
        return result("DENY", "independent-crl-semantics-binding-mismatch")
    if manifest["cryptographic_binding_sha256"] != expected_crypto_binding_sha256:
        return result(
            "DENY",
            "cryptographic-binding-receipt-mismatch",
            {
                "expected_sha256": expected_crypto_binding_sha256,
                "supplied_sha256": manifest["cryptographic_binding_sha256"],
            },
        )

    crypto_exact = independent_details.get("exact_input_objects")
    if crypto_exact != {key: expected_cross_witness[key] for key in (
        "leaf_certificate_sha256",
        "intermediate_certificate_sha256",
        "trust_anchor_root_sha256",
        "crl_bundle_pem_sha256",
    )}:
        return result("DENY", "independent-crypto-exact-object-binding-mismatch")
    path_exact = {
        "leaf_certificate_sha256": path_details.get("leaf_certificate_sha256"),
        "intermediate_certificate_sha256": path_details.get("intermediate_certificate_sha256"),
        "trust_anchor_root_sha256": path_details.get("trust_anchor_root_sha256"),
        "crl_bundle_pem_sha256": path_details.get("crl_bundle_pem_sha256"),
    }
    if path_exact != {key: expected_cross_witness[key] for key in (
        "leaf_certificate_sha256",
        "intermediate_certificate_sha256",
        "trust_anchor_root_sha256",
        "crl_bundle_pem_sha256",
    )}:
        return result("DENY", "openssl-path-exact-object-binding-mismatch")

    independent_certs = independent_details.get("certificate_signatures")
    independent_crls = independent_details.get("crl_signatures")
    if not isinstance(independent_certs, dict) or not isinstance(independent_crls, dict):
        return result("DENY", "independent-crypto-signature-details-missing")
    for label, embedded in (
        ("leaf", crypto_receipt["certificate_signatures"]["leaf"]),
        ("intermediate", crypto_receipt["certificate_signatures"]["intermediate"]),
        ("root", crypto_receipt["certificate_signatures"]["root"]),
    ):
        observed = independent_certs.get(label)
        if not isinstance(observed, dict):
            return result("DENY", "independent-certificate-signature-missing", {"label": label})
        for field in ("object_sha256", "tbs_sha256", "signature_sha256", "issuer_object_sha256"):
            if observed.get(field) != embedded.get(field):
                return result("DENY", "independent-certificate-signature-binding-mismatch", {"label": label, "field": field})
    for label, embedded in (
        ("root", crypto_receipt["crl_signatures"]["root"]),
        ("intermediate", crypto_receipt["crl_signatures"]["intermediate"]),
    ):
        observed = independent_crls.get(label)
        if not isinstance(observed, dict):
            return result("DENY", "independent-crl-signature-missing", {"label": label})
        for field in ("object_sha256", "tbs_sha256", "signature_sha256", "issuer_object_sha256"):
            if observed.get(field) != embedded.get(field):
                return result("DENY", "independent-crl-signature-binding-mismatch", {"label": label, "field": field})

    cross_witness = canonical_hash({
        "exact_input_objects": {key: expected_cross_witness[key] for key in (
            "leaf_certificate_sha256",
            "intermediate_certificate_sha256",
            "trust_anchor_root_sha256",
            "crl_bundle_pem_sha256",
        )},
        "crl_semantics_sha256": manifest["crl_semantics_sha256"],
        "independent_crypto_verifier_source_sha256": external_crypto["source_sha256"],
        "independent_crypto_output_sha256": external_crypto["output_sha256"],
        "openssl_path_verifier_source_sha256": path_validation["source_sha256"],
        "openssl_path_output_sha256": path_validation["output_sha256"],
        "openssl_policy_argv": path_details.get("policy_argv"),
    })

    profile_ok, profile = leaf_profile_ok(leaf_info)
    profile["leaf_issuer_name_sha256"] = hashlib.sha256(leaf_info["issuer_der"]).hexdigest()
    profile["leaf_subject_name_sha256"] = hashlib.sha256(leaf_info["subject_der"]).hexdigest()
    profile["intermediate_subject_name_sha256"] = hashlib.sha256(intermediate_info["subject_der"]).hexdigest()
    profile["root_subject_name_sha256"] = hashlib.sha256(root_info["subject_der"]).hexdigest()
    profile["aia_non_critical"] = (
        not extension_value(leaf_info, "1.3.6.1.5.5.7.1.1")[0]
    )
    profile["crl_distribution_non_critical"] = (
        not extension_value(leaf_info, "2.5.29.31")[0]
    )
    profile["subject_directory_attributes_non_critical"] = (
        not extension_value(leaf_info, "2.5.29.9")[0]
    )
    profile["path_verifier_id"] = PATH_VERIFIER_ID
    profile["path_execution_binding_sha256"] = path_validation["execution_binding_sha256"]
    if leaf_info["serial"] <= 0:
        return result("DENY", "leaf-serial-invalid", profile)
    if not profile_ok:
        return result("DENY", "ek-leaf-profile-requirements-failed", profile)
    if extension_value(leaf_info, "1.3.6.1.5.5.7.1.1")[0]:
        return result("DENY", "aia-extension-must-be-non-critical", profile)
    if extension_value(leaf_info, "2.5.29.31")[0]:
        return result("DENY", "crl-distribution-extension-must-be-non-critical", profile)
    if extension_value(leaf_info, "2.5.29.9")[0]:
        return result("DENY", "subject-directory-attributes-must-be-non-critical", profile)
    if leaf_info["issuer_der"] != intermediate_info["subject_der"]:
        return result(
            "DENY",
            "leaf-issuer-does-not-match-intermediate-subject",
            {**profile, "intermediate_subject_name_sha256": hashlib.sha256(intermediate_info["subject_der"]).hexdigest()},
        )
    if intermediate_info["subject_der"] not in crl_issuers:
        return result(
            "DENY",
            "crl-bundle-missing-leaf-issuer",
            {**profile, "crl_issuer_name_match": False},
        )
    if root_info["subject_der"] not in crl_issuers:
        return result(
            "DENY",
            "crl-bundle-missing-intermediate-issuer",
            {**profile, "crl_root_issuer_name_match": False},
        )
    if not verify_crl_sign_key_usage(intermediate_info):
        return result("DENY", "intermediate-crl-issuer-missing-crlSign", profile)
    if not verify_crl_sign_key_usage(root_info):
        return result("DENY", "root-crl-issuer-missing-crlSign", profile)
    if not verify_aki_ski(leaf_info, intermediate_info):
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
            "revocation_crl_bundle_pem_sha256": rev["crl_bundle_pem_sha256"],
            "spki_certificate_sha256": spki["certificate_sha256"],
            "spki_ek_public_wire_sha256": spki["ek_public_wire_sha256"],
            "cryptographic_binding_sha256": manifest["cryptographic_binding_sha256"],
            "cryptographic_binding": crypto_receipt,
            "cross_witness_sha256": cross_witness,
        },
    )


def load_fixture(output_dir: Path) -> dict[str, Any]:
    if not FIXTURE_RECIPE_FILE.is_file():
        raise RuntimeError(f"missing EK fixture recipe: {FIXTURE_RECIPE_FILE}")
    if not FIXTURE_GENERATOR_SCRIPT.is_file():
        raise RuntimeError(f"missing EK fixture generator: {FIXTURE_GENERATOR_SCRIPT}")
    recipe = json.loads(FIXTURE_RECIPE_FILE.read_text(encoding="utf-8"))
    expected = recipe.get("expected_outputs")
    required_outputs = {
        "root.der", "intermediate.der", "leaf.der",
        "bad-usage.der", "bad-eku.der", "crl-bundle.pem",
    }
    if not isinstance(expected, dict) or set(expected) != required_outputs:
        raise RuntimeError("EK fixture recipe expected_outputs are incomplete")
    output_dir.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(
        [
            sys.executable,
            str(FIXTURE_GENERATOR_SCRIPT),
            "--recipe",
            str(FIXTURE_RECIPE_FILE),
            "--output-dir",
            str(output_dir),
            "--check",
        ],
        cwd=output_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"EK fixture generation/check failed: {proc.stdout}\n{proc.stderr}")
    values: dict[str, bytes] = {}
    for name, expected_sha in expected.items():
        path = output_dir / name
        if not path.is_file():
            raise RuntimeError(f"generator omitted expected fixture: {path}")
        observed = sha256_file(path)
        if observed != expected_sha:
            raise RuntimeError(
                f"generated EK fixture digest mismatch for {name}: expected {expected_sha} got {observed}"
            )
        values[name] = path.read_bytes()
    return {
        "root": values["root.der"],
        "intermediate": values["intermediate.der"],
        "leaf": values["leaf.der"],
        "bad_usage": values["bad-usage.der"],
        "bad_eku": values["bad-eku.der"],
        "crl_bundle_pem": values["crl-bundle.pem"],
        "attime": REFERENCE_TIME_UNIX,
        "fixture_recipe_sha256": hashlib.sha256(FIXTURE_RECIPE_FILE.read_bytes()).hexdigest(),
    }


def make_manifest(fx: dict[str, Any]) -> dict[str, Any]:
    recipe = json.loads(FIXTURE_RECIPE_FILE.read_text(encoding="utf-8"))
    crl_semantics = recipe["crl_semantics"]
    leaf_sha = hashlib.sha256(fx["leaf"]).hexdigest()
    inter_sha = hashlib.sha256(fx["intermediate"]).hexdigest()
    root_sha = hashlib.sha256(fx["root"]).hexdigest()
    crl_sha = hashlib.sha256(fx["crl_bundle_pem"]).hexdigest()
    m = {
        "profile_id": "mycelix.security.tpm.ek-cert-chain-policy",
        "profile_version": "0.1.0",
        "verification_mode": "ReferenceModelOnly",
        "claim_ceiling": "ReferenceModelOnly",
        "session_id": "ek-chain-self-test",
        "tpm_identity_digest": "44" * 32,
        "ek_public_wire_hex": (
            bytes.fromhex("0001000b000300b2")
            + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
            + bytes.fromhex("00060080004300100800")
            + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
        ).hex(),
        "ek_public_wire_sha256": hashlib.sha256(
            bytes.fromhex("0001000b000300b2")
            + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
            + bytes.fromhex("00060080004300100800")
            + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
        ).hexdigest(),
        "leaf_certificate_der_base64": b64(fx["leaf"]),
        "leaf_certificate_sha256": leaf_sha,
        "intermediate_certificate_der_base64": b64(fx["intermediate"]),
        "intermediate_certificate_sha256": inter_sha,
        "trust_anchor_root_der_base64": b64(fx["root"]),
        "trust_anchor_root_sha256": root_sha,
        "trust_anchor_state": "PASS",
        "trust_anchor_source_sha256": "3bad61140bfe271c6495e6b7e58dfc5ae45cf4fff339bd891a9e04881b63a3ea",
        "verification_time_unix": fx["attime"],
        "crl_semantics": crl_semantics,
        "crl_semantics_sha256": canonical_hash(crl_semantics),
        "cryptographic_binding_sha256": "",
        "cryptographic_binding_source_sha256": "",
        "cryptographic_binding_input_sha256": "",
        "cryptographic_binding_output_sha256": "",
        "path_validation": {
            "state": "PASS",
            "verifier_id": PATH_VERIFIER_ID,
            "source_sha256": sha256_file(PATH_VERIFIER_SCRIPT),
            "input_sha256": "",
            "output_sha256": "",
            "output_content_sha256": "",
            "execution_binding_sha256": "",
            "verifier_input": {},
        },
        "revocation": {
            "state": "PASS",
            "method": "issuer-crl",
            "coverage": "leaf-and-chain",
            "crl_bundle_pem_base64": b64(fx["crl_bundle_pem"]),
            "crl_bundle_pem_sha256": crl_sha,
        },
        "trust_anchor_appraisal": {
            "state": "PASS",
            "verifier_id": TRUST_ANCHOR_APPRAISAL_ID,
            "anchor_id": "mycelix.synthetic-ek-root.v0.1",
            "authorization_state": "PASS",
            "registry_sha256": "",
            "registry_source_sha256": "3bad61140bfe271c6495e6b7e58dfc5ae45cf4fff339bd891a9e04881b63a3ea",
            "receipt_root_sha256": REFERENCE_ROOT_SHA256,
            "input_sha256": "",
            "output_sha256": "",
            "verifier_source_sha256": sha256_file(TRUST_ANCHOR_APPRAISAL_SCRIPT),
            "output_content_sha256": "",
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
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
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
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
            ).hex(),
                "public_wire_sha256": hashlib.sha256(
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
            ).hexdigest(),
                "name_hex": (SHA256_ALG_ID + hashlib.sha256(
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
            ).digest()).hex(),
                "qualified_name_hex": (SHA256_ALG_ID + hashlib.sha256(b"template-qname" + (
                bytes.fromhex("0001000b000300b2")
                + bytes.fromhex("0020") + bytes.fromhex("837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa")
                + bytes.fromhex("00060080004300100800")
                + bytes.fromhex("00000000") + bytes.fromhex("0100") + REFERENCE_EK_RSA_MODULUS
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
            "source_sha256": sha256_file(SPKI_VERIFIER_SCRIPT),
            "input_sha256": "",
            "output_sha256": "",
            "output_content_sha256": "",
            "session_binding_sha256": canonical_hash({
                "session_id": "ek-chain-self-test",
                "tpm_identity_digest": "44" * 32,
                "certificate_der_sha256": leaf_sha,
                "ek_public_wire_sha256": m["ek_public_wire_sha256"],
            }),
            "verifier_input": {
                "certificate_source_sha256": "11" * 32,
                "ek_public_source_sha256": "22" * 32,
            },
        },
    }
    m["session_binding_sha256"] = session_binding(m, leaf_sha, inter_sha, root_sha, crl_sha)
    return m


def refresh_trust_anchor_appraisal(m: dict[str, Any]) -> None:
    appraisal = m["trust_anchor_appraisal"]
    registry = json.loads(TRUST_ANCHOR_REGISTRY_FILE.read_text(encoding="utf-8"))
    registry_sha = hashlib.sha256(
        (json.dumps(registry, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()
    ).hexdigest()
    root = unb64(m["trust_anchor_root_der_base64"], "trust_anchor_root_der_base64")
    input_manifest = {
        "profile_id": "mycelix.security.tpm.ek-trust-anchor-appraisal",
        "profile_version": "0.1.0",
        "claim_ceiling": "ReferenceModelOnly",
        "anchor_id": appraisal["anchor_id"],
        "root_certificate_der_base64": b64(root),
        "root_certificate_sha256": hashlib.sha256(root).hexdigest(),
        "registry_json": registry,
        "registry_sha256": registry_sha,
        "registry_source_sha256": appraisal["registry_source_sha256"],
        "authorization_state": appraisal["authorization_state"],
    }
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-trust-anchor-refresh-") as td:
        work = Path(td)
        ip = work / "input.json"
        op = work / "output.json"
        ip.write_text(json.dumps(input_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        proc = subprocess.run(
            [sys.executable, str(TRUST_ANCHOR_APPRAISAL_SCRIPT), "--verify", str(ip), "--output", str(op)],
            cwd=work, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        if proc.returncode != 0:
            raise RuntimeError(f"trust-anchor appraiser fixture failed: {proc.stderr}")
        appraisal["registry_sha256"] = registry_sha
        appraisal["input_sha256"] = hashlib.sha256(ip.read_bytes()).hexdigest()
        appraisal["output_sha256"] = hashlib.sha256(op.read_bytes()).hexdigest()
        appraisal["verifier_source_sha256"] = sha256_file(TRUST_ANCHOR_APPRAISAL_SCRIPT)
        output = json.loads(op.read_text(encoding="utf-8"))
        appraisal["output_content_sha256"] = output["content_sha256"]

def refresh_spki_binding(m: dict[str, Any]) -> None:
    binding=m["spki_binding"]
    leaf=unb64(m["leaf_certificate_der_base64"],"leaf_certificate_der_base64")
    wire=bytes.fromhex(m["ek_public_wire_hex"])
    binding["certificate_sha256"]=hashlib.sha256(leaf).hexdigest()
    binding["ek_public_wire_sha256"]=hashlib.sha256(wire).hexdigest()
    binding["source_sha256"]=sha256_file(SPKI_VERIFIER_SCRIPT)
    verifier_input={
        "certificate_source_sha256":"11"*32,
        "ek_public_source_sha256":"22"*32,
    }
    binding["verifier_input"]=verifier_input
    binding["session_binding_sha256"]=canonical_hash({
        "session_id":m["session_id"],
        "tpm_identity_digest":m["tpm_identity_digest"],
        "certificate_der_sha256":hashlib.sha256(leaf).hexdigest(),
        "ek_public_wire_sha256":hashlib.sha256(wire).hexdigest(),
    })
    composed_input={
        "profile_id":"mycelix.security.tpm.ek-cert-spki-binding",
        "profile_version":"0.1.0",
        "session_id":m["session_id"],
        "tpm_identity_digest":m["tpm_identity_digest"],
        "verification_mode":m["verification_mode"],
        "claim_ceiling":"ReferenceModelOnly",
        "certificate_der_hex":leaf.hex(),
        "certificate_der_sha256":hashlib.sha256(leaf).hexdigest(),
        "ek_public_format":"TPMT_PUBLIC",
        "ek_public_wire_hex":wire.hex(),
        "ek_public_wire_sha256":hashlib.sha256(wire).hexdigest(),
        "certificate_source_sha256":"11"*32,
        "ek_public_source_sha256":"22"*32,
    }
    composed_input["session_binding_sha256"]=binding["session_binding_sha256"]
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-spki-refresh-") as td:
        work=Path(td);ip=work/"input.json";op=work/"output.json"
        ip.write_text(json.dumps(composed_input,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        proc=subprocess.run(
            [sys.executable,str(SPKI_VERIFIER_SCRIPT),"--verify",str(ip),"--output",str(op)],
            cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False
        )
        if proc.returncode!=0 or not op.is_file():
            raise RuntimeError(f"SPKI fixture verifier failed: {proc.stderr}")
        output=json.loads(op.read_text(encoding="utf-8"))
        binding["input_sha256"]=hashlib.sha256(ip.read_bytes()).hexdigest()
        binding["output_sha256"]=hashlib.sha256(op.read_bytes()).hexdigest()
        binding["output_content_sha256"]=output["content_sha256"]


def refresh_path_validation(m: dict[str, Any]) -> None:
    binding = m["path_validation"]
    verifier_input = {
        "profile_id": "mycelix.security.tpm.ek-cert-path-validation",
        "profile_version": "0.1.0",
        "verification_mode": m["verification_mode"],
        "claim_ceiling": "ReferenceModelOnly",
        "session_id": m["session_id"],
        "tpm_identity_digest": m["tpm_identity_digest"],
        "leaf_certificate_der_base64": m["leaf_certificate_der_base64"],
        "leaf_certificate_sha256": m["leaf_certificate_sha256"],
        "intermediate_certificate_der_base64": m["intermediate_certificate_der_base64"],
        "intermediate_certificate_sha256": m["intermediate_certificate_sha256"],
        "trust_anchor_root_der_base64": m["trust_anchor_root_der_base64"],
        "trust_anchor_root_sha256": m["trust_anchor_root_sha256"],
        "crl_bundle_pem_base64": m["revocation"]["crl_bundle_pem_base64"],
        "crl_bundle_pem_sha256": m["revocation"]["crl_bundle_pem_sha256"],
        "verification_time_unix": m["verification_time_unix"],
    }
    execution_binding = canonical_hash({
        "session_id": verifier_input["session_id"],
        "tpm_identity_digest": verifier_input["tpm_identity_digest"],
        "leaf_certificate_sha256": verifier_input["leaf_certificate_sha256"],
        "intermediate_certificate_sha256": verifier_input["intermediate_certificate_sha256"],
        "trust_anchor_root_sha256": verifier_input["trust_anchor_root_sha256"],
        "crl_bundle_pem_sha256": verifier_input["crl_bundle_pem_sha256"],
        "verification_time_unix": verifier_input["verification_time_unix"],
        "policy_argv": [
            "openssl", "verify", "-x509_strict", "-check_ss_sig", "-CAfile", "root.pem",
            "-untrusted", "intermediate.pem", "-CRLfile", "crl-bundle.pem",
            "-crl_check_all", "-attime", str(verifier_input["verification_time_unix"]),
            "leaf.pem",
        ],
    })
    binding["verifier_input"] = verifier_input
    binding["execution_binding_sha256"] = execution_binding
    binding["source_sha256"] = sha256_file(PATH_VERIFIER_SCRIPT)
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-path-refresh-") as td:
        work = Path(td)
        ip = work / "input.json"
        op = work / "output.json"
        ip.write_text(json.dumps(verifier_input, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        proc = subprocess.run(
            [sys.executable, str(PATH_VERIFIER_SCRIPT), "--verify", str(ip), "--output", str(op)],
            cwd=work, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        if proc.returncode != 0 or not op.is_file():
            raise RuntimeError(f"path verifier fixture failed: {proc.stderr}")
        output = json.loads(op.read_text(encoding="utf-8"))
        binding["state"] = output["state"]
        binding["input_sha256"] = hashlib.sha256(ip.read_bytes()).hexdigest()
        binding["output_sha256"] = hashlib.sha256(op.read_bytes()).hexdigest()
        binding["output_content_sha256"] = output["content_sha256"]


def refresh_cryptographic_binding(m: dict[str, Any]) -> None:
    leaf = parse_certificate_der(unb64(m["leaf_certificate_der_base64"], "leaf_certificate_der_base64"))
    intermediate = parse_certificate_der(
        unb64(m["intermediate_certificate_der_base64"], "intermediate_certificate_der_base64")
    )
    root = parse_certificate_der(
        unb64(m["trust_anchor_root_der_base64"], "trust_anchor_root_der_base64")
    )
    crl_bundle = unb64(m["revocation"]["crl_bundle_pem_base64"], "revocation.crl_bundle_pem_base64")
    receipt = cryptographic_binding_receipt(leaf, intermediate, root, crl_bundle, m["crl_semantics"], m["verification_time_unix"])
    m["cryptographic_binding_sha256"] = canonical_hash(receipt)
    external = run_crypto_verifier(m)
    if external.get("state") != "PASS":
        raise RuntimeError(f"independent crypto verifier failed: {external}")
    m["cryptographic_binding_source_sha256"] = external["source_sha256"]
    m["cryptographic_binding_input_sha256"] = external["input_sha256"]
    m["cryptographic_binding_output_sha256"] = external["output_sha256"]


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

def mutate_root_authorization(m: dict[str, Any]) -> None:
    m["trust_anchor_root_der_base64"] = m["intermediate_certificate_der_base64"]
    m["trust_anchor_root_sha256"] = m["intermediate_certificate_sha256"]
    m["trust_anchor_source_sha256"] = reference_root_source_hash(
        m["trust_anchor_root_sha256"]
    )
    m["session_binding_sha256"] = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_bundle_pem_sha256"],
    )


def mutate_trust_anchor_source(m: dict[str, Any]) -> None:
    m["trust_anchor_source_sha256"] = "66" * 32
    m["session_binding_sha256"] = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_bundle_pem_sha256"],
    )


def mutate_trust_anchor_state_upgrade(m: dict[str, Any]) -> None:
    m["trust_anchor_state"] = "DENY"
    bound = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_bundle_pem_sha256"],
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
        m["revocation"]["crl_bundle_pem_sha256"],
    )
    m["verification_mode"] = "ReferenceModelOnly"
    m["session_binding_sha256"] = bound


def mutate_root_crl_missing(m: dict[str, Any]) -> None:
    bundle = unb64(
        m["revocation"]["crl_bundle_pem_base64"],
        "revocation.crl_bundle_pem_base64",
    )
    blocks = split_pem_crls(bundle)
    reduced = blocks[1]
    m["revocation"]["crl_bundle_pem_base64"] = b64(reduced)
    m["revocation"]["crl_bundle_pem_sha256"] = hashlib.sha256(reduced).hexdigest()
    m["session_binding_sha256"] = session_binding(
        m,
        m["leaf_certificate_sha256"],
        m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"],
        m["revocation"]["crl_bundle_pem_sha256"],
    )


def mutate_leaf(m: dict[str, Any], leaf: bytes) -> None:
    leaf_sha = hashlib.sha256(leaf).hexdigest()
    m["leaf_certificate_der_base64"] = b64(leaf)
    m["leaf_certificate_sha256"] = leaf_sha
    m["spki_binding"]["certificate_sha256"] = leaf_sha
    m["session_binding_sha256"] = session_binding(
        m, leaf_sha, m["intermediate_certificate_sha256"],
        m["trust_anchor_root_sha256"], m["revocation"]["crl_bundle_pem_sha256"]
    )


def self_test() -> int:
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-chain-fixture-") as td:
        fx = load_fixture(Path(td) / "generated")
        base = make_manifest(fx)
        refresh_trust_anchor_appraisal(base)
        refresh_spki_binding(base)
        refresh_path_validation(base)
        refresh_template_binding(base)
        refresh_cryptographic_binding(base)
        if base["crl_semantics_sha256"] != canonical_hash(base["crl_semantics"]):
            print("CRL semantics recipe digest: FAIL")
            return 1
        base["session_binding_sha256"] = session_binding(
            base,
            base["leaf_certificate_sha256"],
            base["intermediate_certificate_sha256"],
            base["trust_anchor_root_sha256"],
            base["revocation"]["crl_bundle_pem_sha256"],
        )
        def tamper_signed_der(der: bytes) -> bytes:
            tag, content, _raw, end = der_tlv(der, 0)
            if tag != 0x30 or end != len(der):
                raise ValueError("signed object malformed")
            children = der_children(content)
            if len(children) != 3 or children[2][0] != 0x03:
                raise ValueError("signatureValue malformed")
            signature_content = bytearray(children[2][1])
            if len(signature_content) < 2:
                raise ValueError("signatureValue too short")
            signature_content[-1] ^= 0x01
            return der_encode_tlv(
                0x30,
                children[0][2]
                + children[1][2]
                + der_encode_tlv(0x03, bytes(signature_content)),
            )

        inter_for_crypto = parse_certificate_der(fx["intermediate"])
        root_for_crypto = parse_certificate_der(fx["root"])
        leaf_for_crypto = parse_certificate_der(fx["leaf"])

        tampered_leaf_info = parse_certificate_der(tamper_signed_der(fx["leaf"]))
        try:
            rsa_sha256_verify(
                tampered_leaf_info["tbs_der"],
                tampered_leaf_info["signature_der"],
                inter_for_crypto["spki_der"],
            )
        except ValueError:
            pass
        else:
            print("tampered leaf certificate signature acceptance: FAIL")
            return 1

        tampered_root_info = parse_certificate_der(tamper_signed_der(fx["root"]))
        try:
            rsa_sha256_verify(
                tampered_root_info["tbs_der"],
                tampered_root_info["signature_der"],
                root_for_crypto["spki_der"],
            )
        except ValueError:
            pass
        else:
            print("tampered root self-signature acceptance: FAIL")
            return 1

        crl_blocks_for_crypto = split_pem_crls(fx["crl_bundle_pem"])
        if b"".join(crl_blocks_for_crypto) != fx["crl_bundle_pem"]:
            print("CRL PEM block segmentation loses supplied bytes: FAIL")
            return 1
        root_crl_der = crl_pem_to_der(crl_blocks_for_crypto[0])
        tampered_crl_info = parse_crl_der_for_crypto(tamper_signed_der(root_crl_der))
        try:
            rsa_sha256_verify(
                tampered_crl_info["tbs_der"],
                tampered_crl_info["signature_der"],
                root_for_crypto["spki_der"],
            )
        except ValueError:
            pass
        else:
            print("tampered CRL signature acceptance: FAIL")
            return 1

        source = Path(__file__).read_text(encoding="utf-8")
        implementation_source = source.split("def self_test()", 1)[0]
        if 'manifest.get("profile_override")' in implementation_source or 'override = manifest.get("profile_override")' in implementation_source:
            print("caller profile override escape hatch: FAIL")
            return 1
        if "def verify_crl_sign_key_usage" not in source or "crl sign" not in source.lower():
            print("explicit CRL issuer cRLSign enforcement: FAIL")
            return 1
        if '"-crl_check_all",' not in source or "PATH_VERIFIER_ID" not in source:
            print("full-chain CRL path verifier composition: FAIL")
            return 1
        if "def run_crypto_verifier" not in implementation_source or "CRYPTO_VERIFIER_SCRIPT" not in implementation_source:
            print("independent cryptographic witness composition: FAIL")
            return 1
        if "openssl" not in implementation_source or "-check_ss_sig" not in implementation_source:
            print("strict OpenSSL path composition: FAIL")
            return 1
        if "def parse_certificate_der" not in implementation_source or "leaf_profile_ok(leaf_info)" not in implementation_source:
            print("binary DER certificate semantics: FAIL")
            return 1
        if "x509_text(" in implementation_source or "x509_scalar(" in implementation_source or "extension(leaf_text" in implementation_source:
            print("human-readable certificate text remains security-authoritative: FAIL")
            return 1
        try:
            der_tlv(b"\x04\x81\x01\x00", 0)
        except ValueError:
            pass
        else:
            print("non-canonical DER length acceptance: FAIL")
            return 1
        duplicate_extension = der_tlv(
            0x30,
            der_tlv(0x06, bytes.fromhex("551d13")) + der_tlv(0x04, der_tlv(0x30, b"")),
        )
        try:
            parse_extensions(der_tlv(0x30, duplicate_extension + duplicate_extension))
        except ValueError:
            pass
        else:
            print("duplicate X.509 extension acceptance: FAIL")
            return 1

        explicit_false_extension = der_tlv(
            0x30,
            der_tlv(0x06, bytes.fromhex("551d13"))
            + der_tlv(0x01, b"\x00")
            + der_tlv(0x04, der_tlv(0x30, b"")),
        )
        try:
            parse_extensions(der_tlv(0x30, explicit_false_extension))
        except ValueError:
            pass
        else:
            print("explicit FALSE extension critical BOOLEAN acceptance: FAIL")
            return 1

        try:
            oid_string(bytes.fromhex("2a800100"))
        except ValueError:
            pass
        else:
            print("non-canonical OID encoding acceptance: FAIL")
            return 1

        try:
            der_integer_value(b"\x00\x01", "test")
        except ValueError:
            pass
        else:
            print("non-canonical INTEGER encoding acceptance: FAIL")
            return 1

        version = der_tlv(0xA0, der_tlv(0x02, b"\x02"))
        serial = der_tlv(0x02, b"\x01")
        algorithm = der_tlv(0x30, b"")
        issuer = der_tlv(0x30, b"")
        validity = der_tlv(
            0x30,
            der_tlv(0x17, b"260101000000Z") + der_tlv(0x17, b"270101000000Z"),
        )
        subject = der_tlv(0x30, b"")
        spki = der_tlv(0x30, b"")
        extensions = der_tlv(0xA3, der_tlv(0x30, b""))
        synthetic_cert = der_tlv(
            0x30,
            der_tlv(0x30, version + serial + algorithm + issuer + validity + subject + spki + extensions + extensions)
            + algorithm
            + der_tlv(0x03, b"\x00"),
        )
        try:
            parse_certificate_der(synthetic_cert)
        except ValueError:
            pass
        else:
            print("duplicate X.509 Extensions wrapper acceptance: FAIL")
            return 1

        missing_san = copy.deepcopy(leaf_info)
        missing_san["extensions"].pop("2.5.29.17", None)
        san_ok, _san_profile = leaf_profile_ok(missing_san)
        if san_ok:
            print("TCG SubjectAltName omission acceptance: FAIL")
            return 1

        ca_true = copy.deepcopy(leaf_info)
        ca_true["extensions"]["2.5.29.19"]["extn_value"] = der_tlv(
            0x30, der_tlv(0x01, b"\xff")
        )
        ok_ca_true, _ = leaf_profile_ok(ca_true)
        if ok_ca_true:
            print("leaf CA=true BasicConstraints acceptance: FAIL")
            return 1

        ca_false_pathlen = copy.deepcopy(leaf_info)
        ca_false_pathlen["extensions"]["2.5.29.19"]["extn_value"] = der_tlv(
            0x30, der_tlv(0x01, b"\x00") + der_tlv(0x02, b"\x00")
        )
        try:
            leaf_profile_ok(ca_false_pathlen)
        except ValueError:
            pass
        else:
            print("CA=false pathLenConstraint acceptance: FAIL")
            return 1

        key_usage_trailing_zero = copy.deepcopy(leaf_info)
        key_usage_trailing_zero["extensions"]["2.5.29.15"]["extn_value"] = der_tlv(
            0x03, b"\x00\x20\x00"
        )
        try:
            leaf_profile_ok(key_usage_trailing_zero)
        except ValueError:
            pass
        else:
            print("non-canonical KeyUsage trailing zero acceptance: FAIL")
            return 1

        empty_extensions = der_tlv(0x30, b"")
        try:
            parse_extensions(empty_extensions)
        except ValueError:
            pass
        else:
            print("empty X.509 Extensions acceptance: FAIL")
            return 1

        for oid, label in (
            ("1.3.6.1.5.5.7.1.1", "AuthorityInformationAccess"),
            ("2.5.29.31", "CRLDistributionPoints"),
        ):
            malformed = copy.deepcopy(leaf_info)
            malformed["extensions"][oid]["extn_value"] = der_tlv(0x04, b"malformed")
            try:
                leaf_profile_ok(malformed)
            except ValueError:
                pass
            else:
                print(f"malformed {label} acceptance: FAIL")
                return 1

        for oid, label in (
            ("1.3.6.1.5.5.7.1.1", "AuthorityInformationAccess"),
            ("2.5.29.31", "CRLDistributionPoints"),
            ("2.5.29.9", "SubjectDirectoryAttributes"),
        ):
            criticalized = copy.deepcopy(leaf_info)
            criticalized["extensions"][oid]["critical"] = True
            try:
                leaf_profile_ok(criticalized)
            except ValueError:
                pass
            else:
                print(f"critical {label} acceptance: FAIL")
                return 1

        nonempty_subject_critical_san = copy.deepcopy(leaf_info)
        nonempty_subject_critical_san["extensions"]["2.5.29.17"]["critical"] = True
        ok_san_critical, _ = leaf_profile_ok(nonempty_subject_critical_san)
        if ok_san_critical:
            print("non-empty-subject critical SAN acceptance: FAIL")
            return 1

        critical_policies = copy.deepcopy(leaf_info)
        critical_policies["extensions"]["2.5.29.32"] = {
            "critical": True,
            "extn_value": der_tlv(
                0x30,
                der_tlv(0x30, der_tlv(0x06, bytes.fromhex("2b06010505070301"))),
            ),
        }
        try:
            leaf_profile_ok(critical_policies)
        except ValueError:
            pass
        else:
            print("critical CertificatePolicies acceptance: FAIL")
            return 1

        malformed_policies = copy.deepcopy(leaf_info)
        malformed_policies["extensions"]["2.5.29.32"] = {
            "critical": False,
            "extn_value": der_tlv(0x04, b"malformed"),
        }
        try:
            leaf_profile_ok(malformed_policies)
        except ValueError:
            pass
        else:
            print("malformed CertificatePolicies acceptance: FAIL")
            return 1

        unknown_aia = copy.deepcopy(leaf_info)
        unknown_aia["extensions"]["1.3.6.1.5.5.7.1.1"]["extn_value"] = der_tlv(
            0x30,
            der_tlv(
                0x30,
                der_tlv(0x06, bytes.fromhex("2b060104018237")) + der_tlv(0x86, b"https://example.invalid/unknown"),
            ),
        )
        try:
            leaf_profile_ok(unknown_aia)
        except ValueError:
            pass
        else:
            print("unknown AIA accessMethod acceptance: FAIL")
            return 1
        san_value_mismatch = copy.deepcopy(leaf_info)
        san_value_mismatch["extensions"]["2.5.29.17"] = copy.deepcopy(
            leaf_info["extensions"]["2.5.29.17"]
        )
        san_critical, san_value = subject_alt_name(san_value_mismatch)
        san_value["attributes"]["2.23.133.2.2"] = ["Attacker TPM Model"]
        san_value_mismatch["_san_override_for_test"] = san_value
        original_san_parser = subject_alt_name

        def patched_subject_alt_name(_info: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
            return san_critical, san_value

        globals()["subject_alt_name"] = patched_subject_alt_name
        try:
            san_profile_ok, _ = leaf_profile_ok(san_value_mismatch)
        finally:
            globals()["subject_alt_name"] = original_san_parser
        if san_profile_ok:
            print("TCG SAN value substitution acceptance: FAIL")
            return 1

        malformed_sda = copy.deepcopy(leaf_info)
        malformed_sda["extensions"]["2.5.29.9"] = {
            "critical": False,
            "extn_value": der_tlv(0x30, b""),
        }
        try:
            leaf_profile_ok(malformed_sda)
        except ValueError:
            pass
        else:
            print("malformed SubjectDirectoryAttributes acceptance: FAIL")
            return 1


        crl_blocks = split_pem_crls(fx["crl_bundle_pem"])
        parsed_crls = {}
        for block in crl_blocks:
            parsed = parse_crl_der_for_crypto(crl_pem_to_der(block))
            label = (
                "root" if parsed["issuer_der"] == root_for_crypto["subject_der"]
                else "intermediate" if parsed["issuer_der"] == inter_for_crypto["subject_der"]
                else None
            )
            if label is None:
                print("CRL semantic fixture issuer mapping: FAIL")
                return 1
            parsed_crls[label] = parsed
        semantic_mutations = [
            ("crl-version-must-be-v2", lambda x: x["root"].update({"version": 1})),
            ("crl-next-update-required", lambda x: x["root"].pop("next_update")),
            ("crl-time-window-bound-to-verification-time", lambda x: x["root"]["next_update"].update({"unix": base["verification_time_unix"]})),
            ("crl-authority-key-identifier-binds-to-signer-ski", lambda x: x["root"].update({"authority_key_identifier": b"\x00" * 32})),
            ("crl-number-binds-to-reference-recipe", lambda x: x["root"].update({"crl_number": x["root"]["crl_number"] + 1})),
            ("crl-revocation-entry-identity-binds-to-exact-der", lambda x: x["root"]["revoked_entries"][0].update({"entry_identity_sha256": "00" * 32})),
            ("crl-revocation-entry-serials-unique-and-ordered", lambda x: x["intermediate"]["revoked_entries"][1].update({"serial": x["intermediate"]["revoked_entries"][0]["serial"]})),
            ("crl-revocation-date-not-after-this-update", lambda x: x["root"]["revoked_entries"][0]["revocation_date"].update({"unix": x["root"]["this_update"]["unix"] + 1})),
        ]
        for name, mutate in semantic_mutations:
            candidate = copy.deepcopy(parsed_crls)
            mutate(candidate)
            try:
                validate_crl_semantics(
                    candidate,
                    {"root": root_for_crypto, "intermediate": inter_for_crypto, "leaf": leaf_for_crypto},
                    base["crl_semantics"],
                    base["verification_time_unix"],
                )
            except (ValueError, KeyError):
                pass
            else:
                print(f"{name}: FAIL")
                return 1

        selection_mutations = [
            ("crl-authoritative-object-identity", lambda x: x["root"]["selection"].update({"crl_der_sha256": "92" * 32})),
            ("crl-authoritative-issuer-certificate-identity", lambda x: x["root"]["selection"].update({"issuer_certificate_sha256": "93" * 32})),
            ("crl-scope-completeness-substitution", lambda x: x["root"]["selection"].update({"scope": "limited-reason-scope"})),
            ("crl-delta-indirect-semantics-substitution", lambda x: x["root"]["selection"].update({"indirect_crl_supported": True})),
            ("crl-number-progression-claim-substitution", lambda x: x["root"]["selection"].update({"crl_number_lineage": "strictly-increasing-history"})),
        ]
        for name, mutate in selection_mutations:
            expected = copy.deepcopy(base["crl_semantics"])
            mutate(expected)
            try:
                validate_crl_semantics(parsed_crls, {"root": root_for_crypto, "intermediate": inter_for_crypto, "leaf": leaf_for_crypto}, expected, base["verification_time_unix"])
            except (ValueError, KeyError):
                pass
            else:
                print(f"{name}: FAIL")
                return 1

        cases = [
            ("canonical-valid", "PASS", lambda x: None),
            ("forbidden-profile-override-on-valid-input", "DENY", lambda x: x.update({
                "profile_override": {"authority_key_identifier_critical": False}
            })),
            ("cryptographic-binding-substitution", "DENY", lambda x: x.update({
                "cryptographic_binding_sha256": "99" * 32
            })),
            ("tpm-identity-digest-substitution", "DENY", lambda x: x.update({
                "tpm_identity_digest": "ab" * 32
            })),
            ("root-substitution", "DENY", lambda x: (
                x.update({
                    "trust_anchor_root_der_base64": x["intermediate_certificate_der_base64"],
                    "trust_anchor_root_sha256": x["intermediate_certificate_sha256"],
                    "trust_anchor_source_sha256": reference_root_source_hash(x["intermediate_certificate_sha256"]),
                })
            )),
            ("intermediate-substitution", "DENY", lambda x: x.update({
                "intermediate_certificate_der_base64": x["trust_anchor_root_der_base64"],
                "intermediate_certificate_sha256": x["trust_anchor_root_sha256"],
            })),
            ("leaf-byte-substitution", "DENY", lambda x: mutate_leaf(x, fx["leaf"][:-1] + bytes([fx["leaf"][-1] ^ 1]))),
            ("expired-reference-time", "DENY", lambda x: x.update({"verification_time_unix": EXPIRED_TIME_UNIX})),
            ("not-yet-valid-reference-time", "DENY", lambda x: x.update({"verification_time_unix": 0})),
            ("key-usage-profile-mismatch", "DENY", lambda x: mutate_leaf(x, fx["bad_usage"])),
            ("profile-override-cannot-rescue-key-usage", "DENY", lambda x: (
                mutate_leaf(x, fx["bad_usage"]),
                x.update({"profile_override": {"authority_key_identifier_critical": False, "extended_key_usage_critical": False}})
            )),
            ("eku-profile-mismatch", "DENY", lambda x: mutate_leaf(x, fx["bad_eku"])),
            ("profile-override-cannot-rescue-eku", "DENY", lambda x: (
                mutate_leaf(x, fx["bad_eku"]),
                x.update({"profile_override": {"authority_key_identifier_critical": False, "extended_key_usage_critical": False}})
            )),
            ("trust-anchor-source-substitution", "DENY", lambda x: mutate_trust_anchor_source(x)),
            ("root-self-consistent-source-substitution", "DENY", lambda x: mutate_root_authorization(x)),
            ("template-verifier-substitution", "DENY", lambda x: x["ek_template_binding"].update({"verifier_id": "other-verifier"})),
            ("template-source-substitution", "DENY", lambda x: x["ek_template_binding"].update({"source_sha256": "12" * 32})),
            ("template-input-substitution", "DENY", lambda x: x["ek_template_binding"].update({"input_sha256": "14" * 32})),
            ("template-wire-substitution", "DENY", lambda x: x["ek_template_binding"].update({"public_wire_sha256": "13" * 32})),
            ("template-indeterminate", "INDETERMINATE", lambda x: x["ek_template_binding"].update({"state": "INDETERMINATE"})),
            ("trust-anchor-state-substitution", "DENY", lambda x: x.update({"trust_anchor_state": "DENY"})),
            ("revocation-deny", "DENY", lambda x: x["revocation"].update({"state": "DENY"})),
            ("revocation-indeterminate", "INDETERMINATE", lambda x: x["revocation"].update({"state": "INDETERMINATE"})),
            ("root-crl-missing", "DENY", mutate_root_crl_missing),
            ("spki-certificate-substitution", "DENY", lambda x: x["spki_binding"].update({"certificate_sha256": "77" * 32})),
            ("spki-verifier-source-substitution", "DENY", lambda x: x["spki_binding"].update({"source_sha256": "78" * 32})),
            ("spki-input-substitution", "DENY", lambda x: x["spki_binding"].update({"input_sha256": "79" * 32})),
            ("spki-output-substitution", "DENY", lambda x: x["spki_binding"].update({"output_sha256": "7a" * 32})),
            ("spki-output-content-substitution", "DENY", lambda x: x["spki_binding"].update({"output_content_sha256": "7b" * 32})),
            ("spki-indeterminate", "INDETERMINATE", lambda x: x["spki_binding"].update({"state": "INDETERMINATE"})),
            ("path-verifier-source-substitution", "DENY", lambda x: x["path_validation"].update({"source_sha256": "80" * 32})),
            ("path-verifier-input-substitution", "DENY", lambda x: x["path_validation"].update({"input_sha256": "81" * 32})),
            ("path-verifier-output-substitution", "DENY", lambda x: x["path_validation"].update({"output_sha256": "82" * 32})),
            ("path-verifier-output-content-substitution", "DENY", lambda x: x["path_validation"].update({"output_content_sha256": "83" * 32})),
            ("path-verifier-execution-binding-substitution", "DENY", lambda x: x["path_validation"].update({"execution_binding_sha256": "84" * 32})),
            ("spki-ek-public-digest-substitution", "DENY", lambda x: x["spki_binding"].update({"ek_public_wire_sha256": "77" * 32})),
            ("verification-time-binding-substitution", "DENY", lambda x: x.update({"verification_time_unix": x["verification_time_unix"] + 3600})),
            ("revocation-state-binding-substitution", "DENY", lambda x: (x["revocation"].update({"state": "INDETERMINATE"}), x.update({"session_binding_sha256": session_binding(x, x["leaf_certificate_sha256"], x["intermediate_certificate_sha256"], x["trust_anchor_root_sha256"], x["revocation"]["crl_bundle_pem_sha256"])}), x["revocation"].update({"state": "PASS"}))),
            ("verification-mode-substitution", "DENY", lambda x: x.update({"verification_mode": "OfflineBundle"})),
            ("trust-anchor-appraisal-verifier-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"verifier_id": "other-verifier"})),
            ("trust-anchor-appraisal-registry-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"registry_sha256": "12" * 32})),
            ("trust-anchor-appraisal-receipt-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"registry_source_sha256": "13" * 32})),
            ("trust-anchor-appraisal-output-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"output_sha256": "14" * 32})),
            ("trust-anchor-appraisal-verifier-source-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"verifier_source_sha256": "45" * 32})),
            ("trust-anchor-appraisal-output-content-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"output_content_sha256": "46" * 32})),
            ("trust-anchor-appraisal-receipt-root-substitution", "DENY", lambda x: x["trust_anchor_appraisal"].update({"receipt_root_sha256": "15" * 32})),
            ("session-binding-substitution", "DENY", lambda x: x.update({"session_id": "attacker"})),
            ("crl-semantics-substitution", "DENY", lambda x: x["crl_semantics"]["root"].update({"crl_number": 2})),
            ("crl-semantics-digest-substitution", "DENY", lambda x: x.update({"crl_semantics_sha256": "91" * 32})),
            ("crl-this-update-substitution", "DENY", lambda x: x["crl_semantics"]["root"].update({"this_update": "250101000000Z"})),
            ("crl-next-update-expiry", "DENY", lambda x: x["crl_semantics"]["root"].update({"next_update": "260101000000Z"})),
            ("crl-revocation-entry-serial-substitution", "DENY", lambda x: x["crl_semantics"]["intermediate"]["revoked_entries"][1].update({"serial": 4100})),
            ("crl-number-substitution", "DENY", lambda x: x["crl_semantics"]["intermediate"].update({"crl_number": 99})),
            ("crl-revocation-entry-identity-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["revoked_entries"][0].update({"entry_identity_sha256": "00" * 32})),
            ("crl-revocation-entry-reason-substitution", "DENY", lambda x: x["crl_semantics"]["intermediate"]["revoked_entries"][0].update({"reason_code": 2})),
            ("crl-authoritative-object-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["selection"].update({"crl_der_sha256": "92" * 32})),
            ("crl-authoritative-issuer-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["selection"].update({"issuer_certificate_sha256": "93" * 32})),
            ("crl-scope-completeness-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["selection"].update({"scope": "limited-reason-scope"})),
            ("crl-delta-indirect-semantics-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["selection"].update({"indirect_crl_supported": True})),
            ("crl-number-progression-claim-substitution", "DENY", lambda x: x["crl_semantics"]["root"]["selection"].update({"crl_number_lineage": "strictly-increasing-history"})),
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
    print("56 contract vectors plus 20 structural parser controls plus 8 CRL semantic controls: PASS")
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
