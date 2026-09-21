#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import tempfile
from dataclasses import dataclass


class DerError(ValueError):
    pass


def der_len(n: int) -> bytes:
    if n < 0:
        raise DerError("negative-length")
    if n < 128:
        return bytes([n])
    raw = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(raw)]) + raw


def tlv(tag: int, content: bytes) -> bytes:
    return bytes([tag]) + der_len(len(content)) + content


def oid(*arcs: int) -> bytes:
    if len(arcs) < 2 or not 0 <= arcs[0] <= 2 or not 0 <= arcs[1] <= 39:
        raise DerError("bad-oid")
    out = bytearray([40 * arcs[0] + arcs[1]])
    for value in arcs[2:]:
        if value < 0:
            raise DerError("bad-oid-arc")
        groups = [value & 0x7F]
        value >>= 7
        while value:
            groups.append(value & 0x7F)
            value >>= 7
        groups.reverse()
        for i, group in enumerate(groups):
            out.append(group | (0x80 if i < len(groups) - 1 else 0))
    return tlv(0x06, bytes(out))


def algid(arcs: tuple[int, ...]) -> bytes:
    # All three B2A AlgorithmIdentifier parameter fields are normatively absent.
    return tlv(0x30, oid(*arcs))


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


@dataclass(frozen=True)
class Item:
    tag: int
    content: bytes
    end: int
    encoded: bytes


def read_tlv(data: bytes, off: int) -> Item:
    start = off
    if off >= len(data):
        raise DerError("truncated-tag")
    tag = data[off]
    off += 1
    if off >= len(data):
        raise DerError("truncated-length")
    first = data[off]
    off += 1
    if first < 0x80:
        length = first
    else:
        count = first & 0x7F
        if count == 0 or count > 8 or off + count > len(data):
            raise DerError("invalid-length")
        raw = data[off : off + count]
        off += count
        if raw[0] == 0:
            raise DerError("noncanonical-length")
        length = int.from_bytes(raw, "big")
        if length < 128:
            raise DerError("noncanonical-long-length")
    end = off + length
    if end > len(data):
        raise DerError("truncated-content")
    return Item(tag, data[off:end], end, data[start:end])


def build_kri(m: dict, kemct: bytes) -> bytes:
    rid = bytes.fromhex(m["recipient_identifier"]["hex"])
    ukm = bytes.fromhex(m["ukm"]["hex"])
    encrypted_key = bytes.fromhex(m["encrypted_key"]["hex"])

    kem_alg = algid((2, 16, 840, 1, 101, 3, 4, 4, 2))
    kdf_alg = algid((1, 2, 840, 113549, 1, 9, 16, 3, 28))
    wrap_alg = algid((2, 16, 840, 1, 101, 3, 4, 1, 45))

    if kem_alg.hex() != m["kem"]["algorithm_identifier_der_hex"]:
        raise AssertionError("ML-KEM-768 AlgorithmIdentifier drift")
    if kdf_alg.hex() != m["kdf"]["algorithm_identifier_der_hex"]:
        raise AssertionError("HKDF-SHA256 AlgorithmIdentifier drift")
    if wrap_alg.hex() != m["wrap"]["algorithm_identifier_der_hex"]:
        raise AssertionError("AES-256-WRAP AlgorithmIdentifier drift")

    body = b"".join(
        [
            tlv(0x02, b"\x00"),
            tlv(0x80, rid),
            kem_alg,
            tlv(0x04, kemct),
            kdf_alg,
            tlv(0x02, bytes([m["kek_length"]])),
            tlv(0xA0, tlv(0x04, ukm)),
            wrap_alg,
            tlv(0x04, encrypted_key),
        ]
    )
    return tlv(0x30, body)


def verify_structure(kri: bytes, m: dict, kemct: bytes) -> None:
    top = read_tlv(kri, 0)
    if top.tag != 0x30 or top.end != len(kri):
        raise AssertionError("KEMRecipientInfo must be one exact SEQUENCE")

    p = top.content
    off = 0
    fields: list[Item] = []
    while off < len(p):
        item = read_tlv(p, off)
        fields.append(item)
        off = item.end
    if len(fields) != 9:
        raise AssertionError(f"expected 9 KEMRecipientInfo fields, got {len(fields)}")

    version, rid, kem, kemct_item, kdf, keklen, ukm, wrap, encrypted = fields
    if version.tag != 0x02 or version.content != b"\x00":
        raise AssertionError("version must be INTEGER 0")
    if rid.tag != 0x80 or rid.content.hex() != m["recipient_identifier"]["hex"]:
        raise AssertionError("test subjectKeyIdentifier encoding drift")
    if kem.encoded.hex() != m["kem"]["algorithm_identifier_der_hex"]:
        raise AssertionError("KEM AlgorithmIdentifier mismatch")
    if kemct_item.tag != 0x04 or kemct_item.content != kemct:
        raise AssertionError("KEM ciphertext mismatch")
    if kdf.encoded.hex() != m["kdf"]["algorithm_identifier_der_hex"]:
        raise AssertionError("KDF AlgorithmIdentifier mismatch")
    if keklen.tag != 0x02 or keklen.content != b"\x20":
        raise AssertionError("kekLength must be INTEGER 32")
    expected_ukm = tlv(0x04, bytes.fromhex(m["ukm"]["hex"]))
    if ukm.tag != 0xA0 or ukm.content != expected_ukm:
        raise AssertionError("UKM must be [0] EXPLICIT OCTET STRING")
    if wrap.encoded.hex() != m["wrap"]["algorithm_identifier_der_hex"]:
        raise AssertionError("wrap AlgorithmIdentifier mismatch")
    if encrypted.tag != 0x04 or encrypted.content.hex() != m["encrypted_key"]["hex"]:
        raise AssertionError("encryptedKey mismatch")


def require_any(parsed: str, labels: tuple[str, ...], what: str) -> None:
    if not any(label in parsed for label in labels):
        raise AssertionError(f"OpenSSL ASN.1 parse missing {what}: {labels!r}\n{parsed}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    args = parser.parse_args()

    m = json.loads(args.manifest.read_text(encoding="utf-8"))
    if m["schema"] != "sup-civ-000d1c2b2a-kemrecipientinfo-der-v1":
        raise AssertionError("wrong manifest schema")

    kemct_path = pathlib.Path(m["kem_ciphertext"]["fixture_path"])
    kemct = kemct_path.read_bytes()
    if len(kemct) != m["kem_ciphertext"]["length"]:
        raise AssertionError("KEM ciphertext length mismatch")
    if sha256_hex(kemct) != m["kem_ciphertext"]["sha256"]:
        raise AssertionError("KEM ciphertext digest mismatch")

    if m["recipient_identifier"]["certificate_binding_proven"] is not False:
        raise AssertionError("B2A may not claim certificate binding")
    if m["recipient_identifier"]["authorization_proven"] is not False:
        raise AssertionError("B2A may not claim recipient authorization")

    kri = build_kri(m, kemct)
    verify_structure(kri, m, kemct)
    if len(kri) != m["expected_kemrecipientinfo"]["length"]:
        raise AssertionError("KEMRecipientInfo length mismatch")
    if sha256_hex(kri) != m["expected_kemrecipientinfo"]["sha256"]:
        raise AssertionError("KEMRecipientInfo digest mismatch")

    with tempfile.TemporaryDirectory(prefix="sup-civ-c2b2a-") as td:
        path = pathlib.Path(td) / "kemrecipientinfo.der"
        path.write_bytes(kri)
        version = subprocess.run(
            ["openssl", "version"], check=True, text=True, stdout=subprocess.PIPE
        ).stdout.strip()
        print(f"openssl-version={version}")
        parsed = subprocess.run(
            ["openssl", "asn1parse", "-inform", "DER", "-in", str(path), "-i"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout
        require_any(parsed, ("ML-KEM-768", "2.16.840.1.101.3.4.4.2"), "ML-KEM-768 OID")
        require_any(parsed, ("1.2.840.113549.1.9.16.3.28",), "HKDF-SHA256 OID")
        require_any(parsed, ("id-aes256-wrap", "2.16.840.1.101.3.4.1.45"), "AES-256-WRAP OID")
        require_any(parsed, ("cont [ 0 ]",), "context-specific [0] fields")

    print(f"kemrecipientinfo-length={len(kri)}")
    print(f"kemrecipientinfo-sha256={sha256_hex(kri)}")
    print("SUP-CIV-000D1C2B2A PASS: exact KEMRecipientInfo DER syntax vector reproduced")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
