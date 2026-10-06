#!/usr/bin/env python3
"""Verify an EK X.509 certificate path under an explicit reference policy."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
import subprocess
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-chain-policy.v0.1"
SPKI_VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
PATH_VERIFIER_ID = "mycelix.tpm.ek-cert-path-validation.v0.1"
TEMPLATE_VERIFIER_ID = "mycelix.tpm.ek-template-appraisal.v0.1"
ROOT = Path(__file__).resolve().parents[2]
TRUST_ANCHOR_APPRAISAL_ID = "mycelix.tpm.ek-trust-anchor-appraisal.v0.1"
TRUST_ANCHOR_APPRAISAL_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_trust_anchor_appraisal_v0_1.py")
TRUST_ANCHOR_REGISTRY_FILE = ROOT / "docs/security/mycelix-ek-trust-anchor-registry-v0.1.json"
TRUST_ANCHOR_AUTHORIZATION_RECEIPT_FILE = ROOT / "docs/security/mycelix-ek-trust-anchor-authorization-receipt-v0.1.json"
SPKI_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ek_cert_spki_binding_v0_1.py")
FIXTURE_DIR = ROOT / "docs/security/fixtures/ek-chain-policy-v0.1"
REFERENCE_ROOT_SOURCE_TAG = "mycelix.synthetic-ek-root.v0.1"
REFERENCE_ROOT_SHA256 = "f9dbfd812b4772854cf32096bca60947ea62164835299e1839bc44c003e46fab"
REFERENCE_TIME_UNIX = 1791158400
REFERENCE_EK_RSA_MODULUS = bytes.fromhex("d019fb7bdf2679af2d0bf1d5438dae19f73cad698d171a5e3292506bcd05d8b85b7753f35b9c174105fab4613ccc54a67f43fa97c6ef9330a101897b39c3d0f730ee8001b0b53511846de96731104c242781a1fedee583b72c1205a8ace27bcea878ca22c3be355abd7989a3b8e0f9a384a0b6f3e3a8d8bfc35048d93fc07cf45957a52083ed2a49ce01016dcbbb66d4a39569de30285318f4f5a9ff364a80858cf16e84ee4182de3a282c972b6545ee7aa62d48202043cd006e2a5c84c575499b750226ad19c0a3faea19eb0b813a5e31907fb541ea11e3d3a05a39b120ba3b944dd87da4cb89c3e548d0a05e7b538b5e97c1ecd34290d0b0d2e413e369e657")
EXPIRED_TIME_UNIX = 4102444800
FIXTURE_HASHES = {
    "root.der": REFERENCE_ROOT_SHA256,
    "intermediate.der": "859a9f31a543940927bfc8484a33101710cc67e7467493472b456192fad677a3",
    "leaf.der": "7448f84d763c2bee017ff33e18f1679b73f6db3f96bcfe2793c4b607b20a048b",
    "bad-usage.der": "526a43a655e5a46d7e9176281a96820c4d15d932d0a18be507300d11d8598e9f",
    "bad-eku.der": "45c32aac3237eabe9cecdc38cb9d49c359e52d9d68deacf62aee8d30c44ab62c",
    "crl-bundle.pem": "6e0e1ea27ae4b5933d40e54368a8618a7aee98b45011962029a131c11b323abb",
}
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
        blocks.append(bundle[start:end] + b"\\n")
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


def oid_string(content: bytes) -> str:
    if not content:
        raise ValueError("DER OID empty")
    first = content[0]
    first_arc = min(first // 40, 2)
    second_arc = first - (40 * first_arc)
    arcs = [first_arc, second_arc]
    value = 0
    have = False
    for byte in content[1:]:
        have = True
        value = (value << 7) | (byte & 0x7F)
        if byte & 0x80 == 0:
            arcs.append(value)
            value = 0
            have = False
    if have:
        raise ValueError("DER OID unterminated")
    return ".".join(str(x) for x in arcs)


def bit_string_has(bit_string_content: bytes, bit_number: int) -> bool:
    if not bit_string_content:
        raise ValueError("DER BIT STRING empty")
    unused = bit_string_content[0]
    payload = bit_string_content[1:]
    if unused > 7:
        raise ValueError("DER BIT STRING invalid unused-bit count")
    byte_index = bit_number // 8
    bit_mask = 0x80 >> (bit_number % 8)
    return byte_index < len(payload) and bool(payload[byte_index] & bit_mask)


def parse_extensions(extension_wrapper: bytes) -> dict[str, dict[str, Any]]:
    tag, content, _raw, end = der_tlv(extension_wrapper, 0)
    if tag != 0x30 or end != len(extension_wrapper):
        raise ValueError("X.509 Extensions must be a SEQUENCE")
    extensions: dict[str, dict[str, Any]] = {}
    for ext_tag, ext_content, _ext_raw in der_children(content):
        if ext_tag != 0x30:
            raise ValueError("X.509 Extension is not a SEQUENCE")
        offset = 0
        oid_tag, oid_content, _oid_raw, offset = der_tlv(ext_content, offset)
        if oid_tag != 0x06:
            raise ValueError("X.509 Extension missing OID")
        critical = False
        next_tag, next_content, _next_raw, next_offset = der_tlv(ext_content, offset)
        if next_tag == 0x01:
            if len(next_content) != 1 or next_content not in (b"\x00", b"\xff"):
                raise ValueError("X.509 Extension critical BOOLEAN invalid")
            critical = next_content != b"\x00"
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
    tag, tbs_content, _tbs_raw, tbs_end = der_tlv(cert_content, 0)
    if tag != 0x30:
        raise ValueError("X.509 TBSCertificate is not a SEQUENCE")
    cursor = 0
    version = 1
    tag, content, raw, next_cursor = der_tlv(tbs_content, cursor)
    if tag == 0xA0:
        inner_tag, inner_content, _inner_raw, inner_end = der_tlv(content, 0)
        if inner_tag != 0x02 or inner_end != len(content):
            raise ValueError("X.509 version field invalid")
        version = int.from_bytes(inner_content, "big") + 1
        cursor = next_cursor
    else:
        cursor = 0
    tag, serial_content, _serial_raw, cursor = der_tlv(tbs_content, cursor)
    if tag != 0x02 or not serial_content:
        raise ValueError("X.509 serial invalid")
    serial = int.from_bytes(serial_content, "big")
    _tag, _sig_content, _sig_raw, cursor = der_tlv(tbs_content, cursor)
    issuer_tag, issuer_content, issuer_raw, cursor = der_tlv(tbs_content, cursor)
    if issuer_tag not in (0x30, 0xA0, 0xA1, 0xA2, 0xA3):
        raise ValueError("X.509 issuer Name invalid")
    _tag, _validity_content, _validity_raw, cursor = der_tlv(tbs_content, cursor)
    subject_tag, _subject_content, subject_raw, cursor = der_tlv(tbs_content, cursor)
    if subject_tag not in (0x30, 0xA0, 0xA1, 0xA2, 0xA3):
        raise ValueError("X.509 subject Name invalid")
    _tag, _spki_content, spki_raw, cursor = der_tlv(tbs_content, cursor)

    extensions: dict[str, dict[str, Any]] = {}
    while cursor < len(tbs_content):
        tag, content, raw, cursor = der_tlv(tbs_content, cursor)
        if tag == 0xA3:
            extensions = parse_extensions(content)
    if cursor != len(tbs_content):
        raise ValueError("X.509 TBSCertificate trailing bytes")
    return {
        "version": version,
        "serial": serial,
        "issuer_der": issuer_raw,
        "subject_der": subject_raw,
        "spki_der": spki_raw,
        "extensions": extensions,
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
    if children[0][0] != 0x01 or len(children[0][1]) != 1:
        raise ValueError("BasicConstraints missing cA BOOLEAN")
    ca_false = children[0][1] == b"\x00"
    if len(children) > 1:
        # RFC 5280 forbids pathLenConstraint when cA is FALSE.
        if ca_false:
            raise ValueError("BasicConstraints pathLenConstraint present with CA=false")
        if children[1][0] != 0x02:
            raise ValueError("BasicConstraints pathLenConstraint malformed")
    return critical, ca_false


def key_usage_bits(info: dict[str, Any]) -> tuple[bool, bool, bool, bool]:
    critical, value = extension_value(info, "2.5.29.15")
    if value is None:
        return critical, False, False
    tag, content, _raw, end = der_tlv(value, 0)
    if tag != 0x03 or end != len(value):
        raise ValueError("KeyUsage extension malformed")
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
    seen = set()
    key_identifier: bytes | None = None
    for child_tag, child_content, _raw in der_children(content):
        if child_tag in seen:
            raise ValueError("AuthorityKeyIdentifier field duplicated")
        seen.add(child_tag)
        if child_tag == 0x80:
            key_identifier = child_content
        elif child_tag == 0xA1:
            # authorityCertIssuer is GeneralNames.
            gn_tag, _gn_content, _gn_raw, gn_end = der_tlv(child_content, 0)
            if gn_tag != 0x30 or gn_end != len(child_content):
                raise ValueError("AuthorityKeyIdentifier authorityCertIssuer malformed")
        elif child_tag == 0x82:
            # authorityCertSerialNumber is INTEGER.
            serial_tag, serial_content, _serial_raw, serial_end = der_tlv(child_content, 0)
            if serial_tag != 0x02 or not serial_content or serial_end != len(child_content):
                raise ValueError("AuthorityKeyIdentifier authorityCertSerialNumber malformed")
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


def leaf_profile_ok(info: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
    bc_critical, ca_false = basic_constraints(info)
    ku_critical, key_encipherment, _crl_sign, key_cert_sign = key_usage_bits(info)
    eku_critical, eku = eku_oids(info)
    aki_critical, aki = authority_key_id(info)
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
    }
    eku_ok = not eku or EK_CERT_EKU_OID in eku
    eku_critical_ok = not eku_critical
    aki_critical_ok = not aki_critical
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
            "openssl", "verify", "-CAfile", "root.pem",
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
            "path_state": manifest["path_validation"].get("state"),
            "path_verifier_id": manifest["path_validation"].get("verifier_id"),
            "path_source_sha256": manifest["path_validation"].get("source_sha256"),
            "path_input_sha256": manifest["path_validation"].get("input_sha256"),
            "path_output_sha256": manifest["path_validation"].get("output_sha256"),
            "path_output_content_sha256": manifest["path_validation"].get("output_content_sha256"),
            "path_execution_binding_sha256": manifest["path_validation"].get("execution_binding_sha256"),
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
        "verification_time_unix", "revocation", "path_validation", "spki_binding", "ek_template_binding",
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

    try:
        leaf_info = parse_certificate_der(leaf)
        intermediate_info = parse_certificate_der(intermediate)
        root_info = parse_certificate_der(root)
        crl_issuers = crl_issuer_names_from_pem_bundle(crl_bundle_pem)
    except (ValueError, OSError) as exc:
        return result("DENY", "certificate-parse-error", {"error": str(exc)})

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
        },
    )


def load_fixture() -> dict[str, Any]:
    if not FIXTURE_DIR.is_dir():
        raise RuntimeError(f"missing frozen EK certificate fixture directory: {FIXTURE_DIR}")
    for name, expected in FIXTURE_HASHES.items():
        path = FIXTURE_DIR / name
        if not path.is_file():
            raise RuntimeError(f"missing frozen EK certificate fixture: {path}")
        observed = sha256_file(path)
        if observed != expected:
            raise RuntimeError(
                f"frozen EK fixture digest mismatch for {name}: expected {expected} got {observed}"
            )
    return {
        "root": (FIXTURE_DIR / "root.der").read_bytes(),
        "intermediate": (FIXTURE_DIR / "intermediate.der").read_bytes(),
        "leaf": (FIXTURE_DIR / "leaf.der").read_bytes(),
        "bad_usage": (FIXTURE_DIR / "bad-usage.der").read_bytes(),
        "bad_eku": (FIXTURE_DIR / "bad-eku.der").read_bytes(),
        "crl_bundle_pem": (FIXTURE_DIR / "crl-bundle.pem").read_bytes(),
        "attime": REFERENCE_TIME_UNIX,
    }



def make_manifest(fx: dict[str, Any]) -> dict[str, Any]:
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
        "trust_anchor_source_sha256": "9bd58a822f05138a4b4b41438452be414a8475911e9a02e9dbf4527f9884c591",
        "verification_time_unix": fx["attime"],
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
            "registry_source_sha256": "9bd58a822f05138a4b4b41438452be414a8475911e9a02e9dbf4527f9884c591",
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
        "profile_override": {
            "authority_key_identifier_critical": False,
            "extended_key_usage_critical": False
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
            "openssl", "verify", "-CAfile", "root.pem",
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
        fx = load_fixture()
        base = make_manifest(fx)
        refresh_trust_anchor_appraisal(base)
        refresh_spki_binding(base)
        refresh_path_validation(base)
        refresh_template_binding(base)
        base["session_binding_sha256"] = session_binding(
            base,
            base["leaf_certificate_sha256"],
            base["intermediate_certificate_sha256"],
            base["trust_anchor_root_sha256"],
            base["revocation"]["crl_bundle_pem_sha256"],
        )
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


        cases = [
            ("canonical-valid", "PASS", lambda x: None),
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
    print("43 adversarial mutations plus canonical and key-order control: PASS")
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
