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
from datetime import datetime, timezone
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


def parse_extensions(wrapper: bytes) -> dict[str, dict[str, Any]]:
    tag, content, _raw, end = tlv(wrapper)
    if tag != 0x30 or end != len(wrapper):
        raise ValueError("X.509 Extensions wrapper malformed")
    children_list = children(content)
    if not children_list:
        raise ValueError("X.509 Extensions wrapper empty")
    out: dict[str, dict[str, Any]] = {}
    for ext_tag, ext_content, _ext_raw in children_list:
        if ext_tag != 0x30:
            raise ValueError("X.509 Extension malformed")
        offset = 0
        oid_tag, oid_content, _oid_raw, offset = tlv(ext_content, offset)
        if oid_tag != 0x06:
            raise ValueError("X.509 Extension missing OID")
        critical = False
        next_tag, next_content, _next_raw, next_offset = tlv(ext_content, offset)
        if next_tag == 0x01:
            if next_content != b"\xff":
                raise ValueError("X.509 Extension critical BOOLEAN must encode TRUE")
            critical = True
            next_tag, next_content, _next_raw, next_offset = tlv(ext_content, next_offset)
        if next_tag != 0x04 or next_offset != len(ext_content):
            raise ValueError("X.509 Extension missing extnValue")
        key = oid(oid_content)
        if key in out:
            raise ValueError(f"duplicate X.509 extension OID: {key}")
        out[key] = {"critical": critical, "extn_value": next_content}
    return out


def parse_time(tag: int, content: bytes, field: str) -> dict[str, Any]:
    if tag == 0x17:
        if len(content) != 13 or not content.endswith(b"Z"):
            raise ValueError(f"{field} UTCTime malformed")
        text_value = content.decode("ascii")
        fmt = "%y%m%d%H%M%SZ"
        year_prefix = int(text_value[:2])
        year = 1900 + year_prefix if year_prefix >= 50 else 2000 + year_prefix
    elif tag == 0x18:
        if len(content) != 15 or not content.endswith(b"Z"):
            raise ValueError(f"{field} GeneralizedTime malformed")
        text_value = content.decode("ascii")
        fmt = "%Y%m%d%H%M%SZ"
        year = int(text_value[:4])
    else:
        raise ValueError(f"{field} must be UTCTime or GeneralizedTime")
    try:
        value = datetime.strptime(text_value, fmt).replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise ValueError(f"{field} time value invalid: {exc}") from exc
    if tag == 0x17 and not (1950 <= year <= 2049):
        raise ValueError(f"{field} UTCTime year outside RFC 5280 range")
    return {
        "text": text_value,
        "unix": int(value.timestamp()),
        "der_sha256": hashlib.sha256(bytes([tag]) + len(content).to_bytes(1, "big") + content).hexdigest(),
    }


def parse_crl_authority_key_identifier(ext_value: bytes) -> bytes:
    tag, content, _raw, end = tlv(ext_value)
    if tag != 0x30 or end != len(ext_value):
        raise ValueError("CRL AuthorityKeyIdentifier malformed")
    fields = children(content)
    if len(fields) != 1 or fields[0][0] != 0x80 or not fields[0][1]:
        raise ValueError("CRL AuthorityKeyIdentifier must contain exactly one non-empty keyIdentifier")
    return fields[0][1]


def parse_reason_code(ext_value: bytes) -> int:
    tag, content, _raw, end = tlv(ext_value)
    if tag != 0x0A or end != len(ext_value) or not content or content[0] & 0x80:
        raise ValueError("CRL reasonCode malformed")
    if len(content) > 1 and content[0] == 0:
        raise ValueError("CRL reasonCode non-canonical")
    value = int.from_bytes(content, "big")
    if value not in {0, 1, 2, 3, 4, 5, 6, 8, 9, 10}:
        raise ValueError("CRL reasonCode unsupported")
    return value


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
        version = children(fields[0][1])
        if len(version) != 1 or version[0][0] != 0x02 or integer(version[0][1], "version") != 2:
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
        validity = children(fields[cursor][1])
        if len(validity) != 2:
            raise ValueError("certificate validity malformed")
        parse_time(validity[0][0], validity[0][1], "certificate.notBefore")
        parse_time(validity[1][0], validity[1][1], "certificate.notAfter")
        cursor += 1
        subject = fields[cursor][2]
        cursor += 1
        spki = fields[cursor][2]
        n, e = rsa_spki(spki)
        cursor += 1
        extensions: dict[str, dict[str, Any]] = {}
        saw_extensions = False
        while cursor < len(fields):
            tag_value, content_value, _raw_value = fields[cursor]
            if tag_value == 0xA1 or tag_value == 0xA2:
                if saw_extensions:
                    raise ValueError("certificate unique-ID field appears after Extensions")
                cursor += 1
                continue
            if tag_value == 0xA3:
                if saw_extensions:
                    raise ValueError("certificate Extensions wrapper duplicated")
                extensions = parse_extensions(content_value)
                saw_extensions = True
                cursor += 1
                continue
            raise ValueError("certificate trailing field malformed")
        ski_value = extensions.get("2.5.29.14")
        subject_key_identifier = None
        if ski_value is not None:
            if ski_value["critical"]:
                raise ValueError("certificate SubjectKeyIdentifier must be non-critical")
            ski_tag, ski_content, _ski_raw, ski_end = tlv(ski_value["extn_value"])
            if ski_tag != 0x04 or ski_end != len(ski_value["extn_value"]) or not ski_content:
                raise ValueError("certificate SubjectKeyIdentifier malformed")
            subject_key_identifier = ski_content
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
            "subject_key_identifier": subject_key_identifier,
            "extensions": extensions,
        }

    if not fields or fields[0][0] != 0x02 or integer(fields[0][1], "CRL version") != 1:
        raise ValueError("CRL must be explicit v2")
    cursor = 1
    inner_alg = fields[cursor][2]
    if alg_oid(inner_alg) != SHA256_RSA_OID or inner_alg != parts[1][2]:
        raise ValueError("CRL signatureAlgorithm mismatch")
    cursor += 1
    if cursor >= len(fields) or fields[cursor][0] != 0x30 or not fields[cursor][1]:
        raise ValueError("CRL issuer Name must be non-empty")
    issuer = fields[cursor][2]
    cursor += 1
    if cursor >= len(fields):
        raise ValueError("CRL thisUpdate missing")
    this_update = parse_time(fields[cursor][0], fields[cursor][1], "CRL.thisUpdate")
    cursor += 1
    if cursor >= len(fields):
        raise ValueError("CRL nextUpdate missing")
    next_update = parse_time(fields[cursor][0], fields[cursor][1], "CRL.nextUpdate")
    cursor += 1
    if this_update["unix"] >= next_update["unix"]:
        raise ValueError("CRL nextUpdate must be later than thisUpdate")

    revoked_entries = []
    if cursor < len(fields) and fields[cursor][0] == 0x30:
        for entry_tag, entry_content, entry_raw in children(fields[cursor][1]):
            if entry_tag != 0x30:
                raise ValueError("CRL entry malformed")
            entry_fields = children(entry_content)
            if len(entry_fields) < 2 or entry_fields[0][0] != 0x02:
                raise ValueError("CRL entry serial missing")
            serial = integer(entry_fields[0][1], "CRL revoked serial", True)
            rev_time = parse_time(entry_fields[1][0], entry_fields[1][1], "CRL revocationDate")
            entry_extensions: dict[str, dict[str, Any]] = {}
            if len(entry_fields) > 2:
                if len(entry_fields) != 3 or entry_fields[2][0] != 0xA0:
                    raise ValueError("CRL entry extensions malformed")
                entry_extensions = parse_extensions(entry_fields[2][1])
            revoked_entries.append({
                "entry_identity_sha256": hashlib.sha256(entry_raw).hexdigest(),
                "serial": serial,
                "revocation_date": rev_time,
                "extensions": entry_extensions,
            })
        cursor += 1

    if cursor >= len(fields) or fields[cursor][0] != 0xA0:
        raise ValueError("CRL Extensions wrapper missing")
    crl_extensions = parse_extensions(fields[cursor][1])
    cursor += 1
    if cursor != len(fields):
        raise ValueError("CRL trailing field malformed")

    return {
        "object_sha256": hashlib.sha256(der).hexdigest(),
        "tbs_sha256": hashlib.sha256(tbs).hexdigest(),
        "signature_sha256": hashlib.sha256(signature).hexdigest(),
        "signature_algorithm_oid": alg,
        "signature": signature,
        "tbs": tbs,
        "issuer": issuer,
        "version": 2,
        "this_update": this_update,
        "next_update": next_update,
        "revoked_entries": revoked_entries,
        "crl_extensions": crl_extensions,
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
    if not encoded.startswith(b"\x00\x01"):
        raise ValueError("PKCS#1 v1.5 header invalid")
    zero = encoded.find(b"\x00", 2)
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
        if bundle[finish:finish + 2] == b"\r\n":
            finish += 2
        elif bundle[finish:finish + 1] == b"\n":
            finish += 1
        block = bundle[start:finish]
        stripped = block.strip()
        encoded = b"".join(stripped[len(begin):-len(end)].split())
        der = base64.b64decode(encoded, validate=True)
        # Preserve exact PEM identity while parsing exact DER payload.
        result_blocks.append((block, der))
        cursor = finish
    if len(result_blocks) != 2:
        raise ValueError("expected exactly two CRLs")
    return result_blocks


def expected_crl_semantics_check(
    crls: dict[str, dict[str, Any]],
    issuers: dict[str, dict[str, Any]],
    expected: dict[str, Any],
    verification_time_unix: int,
) -> None:
    if verification_time_unix < 0:
        raise ValueError("verification time must be non-negative")
    if not isinstance(expected, dict) or set(expected) != {"root", "intermediate"}:
        raise ValueError("CRL semantics recipe must define root and intermediate")
    chain_serials = {
        issuers["root"].get("serial"),
        issuers["intermediate"].get("serial"),
        issuers["leaf"].get("serial") if "leaf" in issuers else None,
    }
    for label, crl in crls.items():
        spec = expected.get(label)
        if not isinstance(spec, dict):
            raise ValueError(f"CRL semantics missing {label}")
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
        if crl["version"] != 2:
            raise ValueError(f"{label} CRL version mismatch")
        if "this_update" not in crl or "next_update" not in crl:
            raise ValueError(f"{label} CRL thisUpdate/nextUpdate are required")
        if crl["this_update"]["unix"] > verification_time_unix or verification_time_unix >= crl["next_update"]["unix"]:
            raise ValueError(f"{label} CRL is outside its modeled validity window")
        extensions = crl["crl_extensions"]
        if set(extensions) != {"2.5.29.35", "2.5.29.20"}:
            raise ValueError(f"{label} CRL extension set mismatch")
        if any(extensions[key]["critical"] for key in extensions):
            raise ValueError(f"{label} CRL required extensions must be non-critical")
        aki = parse_crl_authority_key_identifier(extensions["2.5.29.35"]["extn_value"])
        signer_ski = issuers[label]["subject_key_identifier"]
        if not signer_ski or aki != signer_ski:
            raise ValueError(f"{label} CRL AuthorityKeyIdentifier does not match signer SubjectKeyIdentifier")
        number_tag, number_content, _raw, number_end = tlv(extensions["2.5.29.20"]["extn_value"])
        if number_tag != 0x02 or number_end != len(extensions["2.5.29.20"]["extn_value"]):
            raise ValueError(f"{label} CRL number malformed")
        number = integer(number_content, f"{label}.cRLNumber")
        if number.bit_length() > 160:
            raise ValueError("CRL.cRLNumber value exceeds RFC 5280 20-octet limit")
        if number != int(spec["crl_number"]):
            raise ValueError(f"{label} cRLNumber mismatch")
        if crl["this_update"]["text"] != str(spec["this_update"]) or crl["next_update"]["text"] != str(spec["next_update"]):
            raise ValueError(f"{label} CRL time encoding/value mismatch")
        entries = crl["revoked_entries"]
        expected_entries = spec["revoked_entries"]
        if not isinstance(expected_entries, list) or len(entries) != len(expected_entries):
            raise ValueError(f"{label} CRL revoked-entry count mismatch")
        observed_serials = []
        for observed, exp in zip(entries, expected_entries, strict=True):
            if observed["serial"] != int(exp["serial"]):
                raise ValueError(f"{label} revoked serial mismatch")
            if observed["entry_identity_sha256"] != str(exp["entry_identity_sha256"]):
                raise ValueError(f"{label} revoked-entry DER identity mismatch")
            if observed["revocation_date"]["text"] != str(exp["revocation_date"]):
                raise ValueError(f"{label} revocationDate mismatch")
            if observed["revocation_date"]["unix"] > crl["this_update"]["unix"]:
                raise ValueError(f"{label} revocationDate is after thisUpdate")
            reason_ext = observed["extensions"].get("2.5.29.21")
            if reason_ext is None or reason_ext["critical"]:
                raise ValueError(f"{label} CRL entry reasonCode missing or critical")
            if set(observed["extensions"]) != {"2.5.29.21"}:
                raise ValueError(f"{label} CRL entry extension set mismatch")
            reason = parse_reason_code(reason_ext["extn_value"])
            if reason == 8:
                raise ValueError(f"{label} CRL removeFromCRL requires delta-CRL semantics")
            if reason != int(exp["reason_code"]):
                raise ValueError(f"{label} CRL reasonCode mismatch")
            observed_serials.append(observed["serial"])
        if observed_serials != sorted(set(observed_serials)):
            raise ValueError(f"{label} CRL revoked serials must be strictly increasing")
        if any(serial in chain_serials for serial in observed_serials):
            raise ValueError(f"{label} CRL revokes a certificate in the active chain")
    root_number = int(expected["root"]["crl_number"])
    intermediate_number = int(expected["intermediate"]["crl_number"])
    if root_number < 0 or intermediate_number < 0:
        raise ValueError("CRL number cannot be negative")


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    try:
        expected_recipe_semantics = manifest["expected_crl_semantics"]
        if not isinstance(expected_recipe_semantics, dict):
            return result("DENY", "crl-semantics-recipe-projection-invalid")
        expected_semantics_sha = canonical_hash(expected_recipe_semantics)
        if manifest.get("expected_crl_semantics_sha256") != expected_semantics_sha:
            return result("DENY", "crl-semantics-recipe-digest-mismatch")
        if manifest.get("verification_time_unix") is None:
            return result("DENY", "verification-time-missing")
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
        blocks = crl_blocks(crl_bundle)
        if b"".join(block for block, _der in blocks) != crl_bundle:
            return result("DENY", "crl-pem-block-segmentation-mismatch")
        crls = []
        for pem, der in blocks:
            parsed = signed_object(der, "crl")
            crls.append((pem, parsed))
        crl_map: dict[str, dict[str, Any]] = {}
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
            crl_map[label] = crl
            entries = []
            for entry in crl["revoked_entries"]:
                entry_extensions = entry["extensions"]
                reason = parse_reason_code(entry_extensions["2.5.29.21"]["extn_value"])
                entries.append({
                    "entry_identity_sha256": entry["entry_identity_sha256"],
                    "serial": entry["serial"],
                    "revocation_date": entry["revocation_date"]["text"],
                    "reason_code": reason,
                })
            aki = parse_crl_authority_key_identifier(
                crl["crl_extensions"]["2.5.29.35"]["extn_value"]
            )
            number_tag, number_content, _number_raw, number_end = tlv(
                crl["crl_extensions"]["2.5.29.20"]["extn_value"]
            )
            if number_tag != 0x02 or number_end != len(crl["crl_extensions"]["2.5.29.20"]["extn_value"]):
                return result("DENY", "crl-number-malformed")
            crl_results[label] = {
                "object_sha256": crl["object_sha256"],
                "pem_block_sha256": hashlib.sha256(pem).hexdigest(),
                "tbs_sha256": crl["tbs_sha256"],
                "signature_sha256": crl["signature_sha256"],
                "signature_algorithm_oid": crl["signature_algorithm_oid"],
                "issuer_name_sha256": hashlib.sha256(crl["issuer"]).hexdigest(),
                "issuer_object_sha256": issuer["object_sha256"],
                "issuer_ski_sha256": hashlib.sha256(issuer["subject_key_identifier"]).hexdigest(),
                "authority_key_identifier_sha256": hashlib.sha256(aki).hexdigest(),
                "crl_number": integer(number_content, f"{label}.cRLNumber"),
                "this_update": crl["this_update"],
                "next_update": crl["next_update"],
                "revoked_entries": entries,
            }
        if set(crl_results) != {"root", "intermediate"}:
            return result("DENY", "crl-set-incomplete")
        try:
            expected_crl_semantics_check(
                crl_map,
                {"root": r, "intermediate": i, "leaf": l},
                expected_recipe_semantics,
                int(manifest["verification_time_unix"]),
            )
            crl_semantics_sha256 = expected_semantics_sha
        except (KeyError, ValueError, TypeError) as exc:
            return result("DENY", "crl-semantics-verification-failed", {"error": str(exc)})
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
            "crl_semantics_sha256": crl_semantics_sha256,
            "crl_semantics_recipe": expected_crl_semantics,
            "crl_semantics": {
                label: {
                    "object_sha256": crl_results[label]["object_sha256"],
                    "tbs_sha256": crl_results[label]["tbs_sha256"],
                    "this_update": crl_results[label]["this_update"]["text"],
                    "next_update": crl_results[label]["next_update"]["text"],
                    "crl_number": crl_results[label]["crl_number"],
                    "authority_key_identifier_sha256": crl_results[label]["authority_key_identifier_sha256"],
                    "revoked_entries": crl_results[label]["revoked_entries"],
                }
                for label in ("root", "intermediate")
            },
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
            "verification_time_unix": 1791158400,
            "expected_crl_semantics": json.loads((recipe).read_text(encoding="utf-8"))["crl_semantics"],
            "expected_crl_semantics_sha256": canonical_hash(
                json.loads((recipe).read_text(encoding="utf-8"))["crl_semantics"]
            ),
        }
        implementation_source = Path(__file__).read_text(encoding="utf-8").split("def self_test()", 1)[0]
        if "fixture-recipe-v0.1.json" in implementation_source:
            print("independent witness rereads fixture recipe from disk: FAIL")
            return 1
        if "expected_crl_semantics" not in implementation_source:
            print("independent witness lacks supplied CRL recipe projection: FAIL")
            return 1
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

        selection_cases = [
            ("authoritative CRL object", lambda x: x["root"]["selection"].update({"crl_der_sha256": "92" * 32})),
            ("authoritative issuer certificate", lambda x: x["root"]["selection"].update({"issuer_certificate_sha256": "93" * 32})),
            ("complete direct-issuer scope", lambda x: x["root"]["selection"].update({"scope": "limited-reason-scope"})),
            ("indirect CRL support", lambda x: x["root"]["selection"].update({"indirect_crl_supported": True})),
            ("historical CRL-number progression", lambda x: x["root"]["selection"].update({"crl_number_lineage": "strictly-increasing-history"})),
        ]
        for label, mutate in selection_cases:
            candidate = json.loads(json.dumps(m["expected_crl_semantics"]))
            mutate(candidate)
            bad_selection = dict(m)
            bad_selection["expected_crl_semantics"] = candidate
            bad_selection["expected_crl_semantics_sha256"] = canonical_hash(candidate)
            if verify(bad_selection)["state"] != "DENY":
                print(f"{label} mutation acceptance: FAIL")
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
