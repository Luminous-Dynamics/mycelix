#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import pathlib
import subprocess
import tempfile


def der_len(n: int) -> bytes:
    if n < 0:
        raise ValueError("negative length")
    if n < 128:
        return bytes([n])
    raw = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(raw)]) + raw


def tlv(tag: int, content: bytes) -> bytes:
    return bytes([tag]) + der_len(len(content)) + content


def oid(*arcs: int) -> bytes:
    if len(arcs) < 2 or not 0 <= arcs[0] <= 2 or not 0 <= arcs[1] <= 39:
        raise ValueError("bad oid")
    out = bytearray([40 * arcs[0] + arcs[1]])
    for value in arcs[2:]:
        if value < 0:
            raise ValueError("bad oid arc")
        chunks = [value & 0x7F]
        value >>= 7
        while value:
            chunks.append(0x80 | (value & 0x7F))
            value >>= 7
        out.extend(reversed(chunks))
    return tlv(0x06, bytes(out))


def hkdf_sha256(ikm: bytes, info: bytes, length: int) -> bytes:
    # RFC 5869 / RFC 9936: absent salt is the zero-length string. For HKDF
    # extraction, this is equivalent to HashLen zero octets.
    salt = b"\x00" * hashlib.sha256().digest_size
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    t = b""
    okm = bytearray()
    counter = 1
    while len(okm) < length:
        t = hmac.new(prk, t + info + bytes([counter]), hashlib.sha256).digest()
        okm.extend(t)
        counter += 1
    return bytes(okm[:length])


def build_other_info(ukm: bytes) -> bytes:
    # id-aes256-wrap = 2.16.840.1.101.3.4.1.45; parameters absent.
    wrap_alg = tlv(0x30, oid(2, 16, 840, 1, 101, 3, 4, 1, 45))
    kek_length = tlv(0x02, b"\x20")
    ukm_explicit = tlv(0xA0, tlv(0x04, ukm))
    return tlv(0x30, wrap_alg + kek_length + ukm_explicit)


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def run(manifest_path: pathlib.Path) -> None:
    m = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert m["schema"] == "sup-civ-000d1c2b1-rfc9629-kdf-wrap-v1"

    ukm = bytes.fromhex(m["ukm"]["hex"])
    assert len(ukm) == m["ukm"]["length"]
    assert sha256_hex(ukm) == m["ukm"]["sha256"]

    other_info = build_other_info(ukm)
    assert len(other_info) == m["cmsori_for_kem_other_info"]["length"]
    assert other_info.hex() == m["cmsori_for_kem_other_info"]["der_hex"]
    assert sha256_hex(other_info) == m["cmsori_for_kem_other_info"]["sha256"]

    shared = bytes.fromhex(m["shared_secret_fixture"]["hex"])
    assert len(shared) == 32
    assert sha256_hex(shared) == m["shared_secret_fixture"]["sha256"]

    kek = hkdf_sha256(shared, other_info, m["algorithms"]["kek_length"])
    assert kek.hex() == m["derived_kek"]["hex"]
    assert sha256_hex(kek) == m["derived_kek"]["sha256"]

    dek = bytes.fromhex(m["test_dek"]["hex"])
    assert len(dek) == m["test_dek"]["length"]
    assert sha256_hex(dek) == m["test_dek"]["sha256"]

    with tempfile.TemporaryDirectory(prefix="sup-civ-c2b1-") as td:
        root = pathlib.Path(td)
        info_path = root / "other-info.der"
        dek_path = root / "dek.bin"
        wrapped_path = root / "wrapped.bin"
        unwrapped_path = root / "unwrapped.bin"
        info_path.write_bytes(other_info)
        dek_path.write_bytes(dek)

        version = subprocess.run(
            ["openssl", "version"], check=True, text=True, stdout=subprocess.PIPE
        ).stdout.strip()
        print(f"openssl-version={version}")

        parsed = subprocess.run(
            ["openssl", "asn1parse", "-inform", "DER", "-in", str(info_path), "-i"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout
        if "id-aes256-wrap" not in parsed or "INTEGER" not in parsed:
            raise AssertionError(parsed)

        iv = "A6A6A6A6A6A6A6A6"
        subprocess.run(
            [
                "openssl",
                "enc",
                "-id-aes256-wrap",
                "-K",
                kek.hex(),
                "-iv",
                iv,
                "-in",
                str(dek_path),
                "-out",
                str(wrapped_path),
            ],
            check=True,
        )
        wrapped = wrapped_path.read_bytes()
        assert len(wrapped) == m["wrapped_dek"]["length"]
        assert wrapped.hex() == m["wrapped_dek"]["hex"]
        assert sha256_hex(wrapped) == m["wrapped_dek"]["sha256"]

        subprocess.run(
            [
                "openssl",
                "enc",
                "-d",
                "-id-aes256-wrap",
                "-K",
                kek.hex(),
                "-iv",
                iv,
                "-in",
                str(wrapped_path),
                "-out",
                str(unwrapped_path),
            ],
            check=True,
        )
        assert unwrapped_path.read_bytes() == dek

    print("SUP-CIV-000D1C2B1 PASS: RFC9629 OtherInfo/HKDF/AES-KW vector reproduced")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    args = parser.parse_args()
    run(args.manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
