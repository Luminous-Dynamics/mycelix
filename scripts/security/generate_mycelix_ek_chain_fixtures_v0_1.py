#!/usr/bin/env python3
"""Generate deterministic synthetic EK certificate/CRL fixtures from a committed recipe."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path

SHA256_WITH_RSA = bytes.fromhex("300d06092a864886f70d01010b0500")
RSA_ENCRYPTION = bytes.fromhex("300d06092a864886f70d0101010500")
MR_BASES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101)


def der_len(value: int) -> bytes:
    if value < 128:
        return bytes([value])
    raw = value.to_bytes((value.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(raw)]) + raw


def tlv(tag: int, value: bytes) -> bytes:
    return bytes([tag]) + der_len(len(value)) + value


def seq(*values: bytes) -> bytes:
    return tlv(0x30, b"".join(values))


def set_of(*values: bytes) -> bytes:
    return tlv(0x31, b"".join(sorted(values)))


def integer(value: int) -> bytes:
    if value < 0:
        raise ValueError("negative INTEGER unsupported")
    raw = value.to_bytes(max(1, (value.bit_length() + 7) // 8), "big")
    if raw[0] & 0x80:
        raw = b"\x00" + raw
    return tlv(0x02, raw)


def oid(dotted: str) -> bytes:
    arcs = [int(part) for part in dotted.split(".")]
    if len(arcs) < 2 or arcs[0] not in (0, 1, 2):
        raise ValueError(f"invalid OID {dotted}")
    if arcs[0] < 2 and arcs[1] > 39:
        raise ValueError(f"invalid OID {dotted}")
    out = bytes([40 * arcs[0] + arcs[1]])
    for arc in arcs[2:]:
        if arc < 0:
            raise ValueError(f"invalid OID {dotted}")
        parts = [arc & 0x7F]
        arc >>= 7
        while arc:
            parts.append(0x80 | (arc & 0x7F))
            arc >>= 7
        out += bytes(reversed(parts))
    return tlv(0x06, out)


def utf8(value: str) -> bytes:
    return tlv(0x0C, value.encode("utf-8"))


def octet(value: bytes) -> bytes:
    return tlv(0x04, value)


def bit_string(value: bytes, unused_bits: int = 0) -> bytes:
    return tlv(0x03, bytes([unused_bits]) + value)


def explicit(tag_number: int, value: bytes) -> bytes:
    return tlv(0xA0 + tag_number, value)


def utc_time(value: str) -> bytes:
    return tlv(0x17, value.encode("ascii"))


def name(common_name: str) -> bytes:
    return seq(set_of(seq(oid("2.5.4.3"), utf8(common_name))))


def spki(n: int, e: int) -> bytes:
    return seq(RSA_ENCRYPTION, bit_string(seq(integer(n), integer(e))))


def ski(n: int, e: int) -> bytes:
    return hashlib.sha1(seq(integer(n), integer(e))).digest()


def extension(oid_value: str, extension_value: bytes, *, critical: bool = False) -> bytes:
    critical_field = [tlv(0x01, b"\xFF")] if critical else []
    return seq(oid(oid_value), *critical_field, octet(extension_value))


def san_value(manufacturer: int, model: str, version: str) -> bytes:
    return seq(
        explicit(
            4,
            seq(
                set_of(seq(oid("2.23.133.2.1"), utf8(f"id:{manufacturer:08X}"))),
                set_of(seq(oid("2.23.133.2.2"), utf8(model))),
                set_of(seq(oid("2.23.133.2.3"), utf8(f"id:{version}"))),
            ),
        )
    )


def aia_value() -> bytes:
    return seq(
        seq(oid("1.3.6.1.5.5.7.48.2"), tlv(0x86, b"https://example.invalid/ek-intermediate.cer")),
        seq(oid("1.3.6.1.5.5.7.48.1"), tlv(0x86, b"https://example.invalid/ocsp")),
    )


def cdp_value() -> bytes:
    full_name = tlv(0xA0, tlv(0x86, b"https://example.invalid/ek.crl"))
    return seq(seq(explicit(0, full_name)))


def key_usage_value(bits: int) -> bytes:
    if bits == 0:
        return bit_string(b"\x00", 7)
    highest = bits.bit_length()
    width = (highest + 7) // 8
    raw = bytearray(width)
    for bit in range(highest):
        if bits & (1 << bit):
            raw[bit // 8] |= 1 << (7 - (bit % 8))
    return bit_string(bytes(raw), width * 8 - highest)


def basic_constraints(*, ca: bool, path_length: int | None = None) -> bytes:
    if not ca and path_length is not None:
        raise ValueError("pathLenConstraint forbidden with CA=FALSE")
    if not ca:
        return seq()
    values = [tlv(0x01, b"\xFF")]
    if path_length is not None:
        values.append(integer(path_length))
    return seq(*values)


def deterministic_bytes(seed: str, label: str, counter: int, length: int) -> bytes:
    return hashlib.shake_256(f"{seed}|{label}|{counter}".encode("utf-8")).digest(length)


def is_probable_prime(value: int) -> bool:
    if value < 2:
        return False
    for prime in (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37):
        if value % prime == 0:
            return value == prime
    d = value - 1
    s = 0
    while d % 2 == 0:
        d //= 2
        s += 1
    for base in MR_BASES:
        if base >= value - 2:
            continue
        x = pow(base, d, value)
        if x in (1, value - 1):
            continue
        for _ in range(s - 1):
            x = pow(x, 2, value)
            if x == value - 1:
                break
        else:
            return False
    return True


def derive_prime(seed: str, label: str, bits: int = 1024) -> int:
    for counter in range(100_000):
        candidate = int.from_bytes(
            deterministic_bytes(seed, label, counter, bits // 8),
            "big",
        )
        candidate |= (1 << (bits - 1)) | 1
        if is_probable_prime(candidate):
            return candidate
    raise RuntimeError("deterministic prime search exhausted")


def derive_key(seed: str) -> dict[str, int]:
    p = derive_prime(seed, "p")
    q = derive_prime(seed, "q")
    if p == q:
        raise RuntimeError("deterministic RSA key produced equal primes")
    e = 65537
    n = p * q
    phi = (p - 1) * (q - 1)
    if math.gcd(e, phi) != 1:
        raise RuntimeError("RSA exponent is not coprime to phi")
    d = pow(e, -1, phi)
    return {"n": n, "e": e, "d": d}


def rsa_sign(tbs: bytes, n: int, d: int) -> bytes:
    digest_info = bytes.fromhex("3031300d060960864801650304020105000420") + hashlib.sha256(tbs).digest()
    width = (n.bit_length() + 7) // 8
    encoded = b"\x00\x01" + b"\xFF" * (width - len(digest_info) - 3) + b"\x00" + digest_info
    return pow(int.from_bytes(encoded, "big"), d, n).to_bytes(width, "big")


def certificate(
    serial: int,
    subject: str,
    issuer: str,
    key: dict[str, int],
    issuer_key: dict[str, int] | None = None,
    *,
    ca: bool = False,
    path_length: int | None = None,
    bad_usage: bool = False,
    bad_eku: bool = False,
    tpm: dict | None = None,
) -> bytes:
    issuer_ski = ski(issuer_key["n"], issuer_key["e"]) if issuer_key else None
    tbs_parts = [
        explicit(0, integer(2)),
        integer(serial),
        SHA256_WITH_RSA,
        name(issuer),
        seq(utc_time("260101000000Z"), utc_time("270101000000Z")),
        name(subject),
        spki(key["n"], key["e"]),
    ]
    extensions = [extension("2.5.29.19", basic_constraints(ca=ca, path_length=path_length), critical=True)]
    if ca:
        extensions.append(extension("2.5.29.15", key_usage_value((1 << 5) | (1 << 6)), critical=True))
    else:
        if tpm is None or issuer_ski is None:
            raise ValueError("leaf TPM metadata and issuer SKI are required")
        extensions.extend(
            [
                extension("2.5.29.15", key_usage_value(1 if bad_usage else (1 << 2)), critical=True),
                extension("2.5.29.17", san_value(tpm["manufacturer"], tpm["model"], tpm["version"])),
                extension(
                    "2.5.29.37",
                    seq(oid("1.3.6.1.5.5.7.3.3")) if bad_eku else seq(oid("2.23.133.8.1")),
                ),
                extension("2.5.29.35", seq(tlv(0x80, issuer_ski))),
                extension("1.3.6.1.5.5.7.1.1", aia_value()),
                extension("2.5.29.31", cdp_value()),
            ]
        )
    extensions.append(extension("2.5.29.14", octet(ski(key["n"], key["e"]))))
    tbs_parts.append(explicit(3, seq(*extensions)))
    tbs = seq(*tbs_parts)
    signer = issuer_key or key
    return seq(tbs, SHA256_WITH_RSA, bit_string(rsa_sign(tbs, signer["n"], signer["d"])))


def crl_entry(serial: int, revocation_date: str, reason_code: int) -> bytes:
    if serial <= 0:
        raise ValueError("CRL revoked serial must be positive")
    if reason_code not in {0, 1, 2, 3, 4, 5, 6, 8, 9, 10}:
        raise ValueError("unsupported RFC 5280 CRLReason")
    if reason_code == 8:
        raise ValueError("removeFromCRL requires unsupported delta CRL semantics")
    reason_extension = extension(
        "2.5.29.21",
        tlv(0x0A, bytes([reason_code])),
    )
    return seq(
        integer(serial),
        utc_time(revocation_date),
        seq(reason_extension),
    )


def crl(
    issuer: str,
    signer: dict[str, int],
    *,
    crl_number: int,
    this_update: str,
    next_update: str,
    revoked_entries: list[dict],
) -> bytes:
    if crl_number < 0:
        raise ValueError("CRL number must be non-negative")
    if max(1, (crl_number.bit_length() + 7) // 8) > 20:
        raise ValueError("CRL number exceeds RFC 5280 20-octet limit")
    crl_extensions = seq(
        extension("2.5.29.35", seq(tlv(0x80, ski(signer["n"], signer["e"])))),
        extension("2.5.29.20", integer(crl_number)),
    )
    tbs_parts = [
        integer(1),
        SHA256_WITH_RSA,
        name(issuer),
        utc_time(this_update),
        utc_time(next_update),
    ]
    if revoked_entries:
        serials = [int(entry["serial"]) for entry in revoked_entries]
        if serials != sorted(set(serials)):
            raise ValueError("CRL revoked serials must be strictly increasing")
        tbs_parts.append(
            seq(
                *[
                    crl_entry(
                        int(entry["serial"]),
                        str(entry["revocation_date"]),
                        int(entry["reason_code"]),
                    )
                    for entry in revoked_entries
                ]
            )
        )
    tbs_parts.append(explicit(0, crl_extensions))
    tbs = seq(*tbs_parts)
    return seq(tbs, SHA256_WITH_RSA, bit_string(rsa_sign(tbs, signer["n"], signer["d"])))


def generate(recipe: dict, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    keys = {name: derive_key(seed) for name, seed in recipe["key_seeds"].items()}
    tpm = recipe["tpm"]
    serials = recipe["serials"]
    root = keys["root"]
    intermediate = keys["intermediate"]
    leaf = keys["leaf"]

    files = {
        "root.der": certificate(
            0x9000,
            "Mycelix Synthetic EK Root",
            "Mycelix Synthetic EK Root",
            root,
            ca=True,
            path_length=1,
        ),
        "intermediate.der": certificate(
            0x1000,
            "Mycelix Synthetic EK CA",
            "Mycelix Synthetic EK Root",
            intermediate,
            root,
            ca=True,
            path_length=0,
        ),
        "leaf.der": certificate(
            serials["leaf"],
            "Mycelix Synthetic EK",
            "Mycelix Synthetic EK CA",
            leaf,
            intermediate,
            tpm=tpm,
        ),
        "bad-usage.der": certificate(
            serials["leaf_bad_usage"],
            "Mycelix Synthetic EK BadUsage",
            "Mycelix Synthetic EK CA",
            leaf,
            intermediate,
            bad_usage=True,
            tpm=tpm,
        ),
        "bad-eku.der": certificate(
            serials["leaf_bad_eku"],
            "Mycelix Synthetic EK BadEKU",
            "Mycelix Synthetic EK CA",
            leaf,
            intermediate,
            bad_eku=True,
            tpm=tpm,
        ),
    }

    crl_specs = recipe.get("crl_semantics")
    if not isinstance(crl_specs, dict):
        raise ValueError("recipe is missing crl_semantics")
    applicability = recipe.get("crl_applicability")
    expected_applicability_keys = {
        "profile",
        "locator_authority",
        "certificate_sha256",
        "selected_crl_issuer_certificate_sha256",
        "selected_crl_der_sha256",
        "selected_crl_scope",
        "distribution_point",
    }
    if not isinstance(applicability, dict) or set(applicability) != expected_applicability_keys:
        raise ValueError("recipe is missing or has malformed crl_applicability")
    if applicability["profile"] != "direct-issuer-complete-crl-v0.1":
        raise ValueError("CRL applicability profile is unsupported")
    if applicability["locator_authority"] != "non-authoritative":
        raise ValueError("CRL distribution locator must remain non-authoritative")
    if applicability["certificate_sha256"] != hashlib.sha256(files["leaf.der"]).hexdigest():
        raise ValueError("CRL applicability certificate hash does not match generated leaf")
    if applicability["selected_crl_issuer_certificate_sha256"] != hashlib.sha256(files["intermediate.der"]).hexdigest():
        raise ValueError("CRL applicability issuer hash does not match generated intermediate")
    if applicability["selected_crl_scope"] != "all-certificates-issued-by-issuer":
        raise ValueError("CRL applicability selected scope is not complete-single-CA")
    dp = applicability["distribution_point"]
    expected_dp_keys = {
        "count", "name_form", "general_name_count", "general_name_type",
        "uri_sha256", "reasons_present", "crl_issuer_present",
    }
    if not isinstance(dp, dict) or set(dp) != expected_dp_keys:
        raise ValueError("CRL applicability distribution-point metadata malformed")
    if dp["count"] != 1 or dp["name_form"] != "fullName" or dp["general_name_count"] != 1:
        raise ValueError("CRL applicability distribution-point cardinality/form mismatch")
    if dp["general_name_type"] != "uniformResourceIdentifier":
        raise ValueError("CRL applicability GeneralName type mismatch")
    if dp["uri_sha256"] != hashlib.sha256(b"https://example.invalid/ek.crl").hexdigest():
        raise ValueError("CRL applicability URI digest does not match fixture generator")
    if dp["reasons_present"] is not False or dp["crl_issuer_present"] is not False:
        raise ValueError("CRL applicability reason/cRLIssuer semantics are outside reference model")
    crl_bundle = b""
    for issuer_name, key_name in (
        ("Mycelix Synthetic EK Root", "root"),
        ("Mycelix Synthetic EK CA", "intermediate"),
    ):
        spec = crl_specs.get(key_name)
        if not isinstance(spec, dict):
            raise ValueError(f"missing CRL semantics for {key_name}")
        selection = spec.get("selection")
        expected_issuer_hash = hashlib.sha256(files[f"{key_name}.der"]).hexdigest()
        expected_crl_keys = {
            "issuer_certificate_sha256", "crl_der_sha256", "scope",
            "delta_crl_supported", "indirect_crl_supported", "crl_number_lineage",
        }
        if not isinstance(selection, dict) or set(selection) != expected_crl_keys:
            raise ValueError(f"missing or malformed CRL selection metadata for {key_name}")
        if selection["issuer_certificate_sha256"] != expected_issuer_hash:
            raise ValueError(f"{key_name} CRL selection issuer certificate hash does not match generated issuer")
        if selection["scope"] != "all-certificates-issued-by-issuer":
            raise ValueError(f"{key_name} CRL selection scope is not complete-single-CA")
        if selection["delta_crl_supported"] is not False or selection["indirect_crl_supported"] is not False:
            raise ValueError(f"{key_name} CRL selection enables unsupported delta/indirect semantics")
        if selection["crl_number_lineage"] != "single-current-reference-no-history":
            raise ValueError(f"{key_name} CRL historical number lineage is outside reference model")
        der = crl(
            issuer_name,
            keys[key_name],
            crl_number=int(spec["crl_number"]),
            this_update=str(spec["this_update"]),
            next_update=str(spec["next_update"]),
            revoked_entries=list(spec["revoked_entries"]),
        )
        if selection["crl_der_sha256"] != hashlib.sha256(der).hexdigest():
            raise ValueError(f"{key_name} CRL selection DER hash does not match generated CRL")
        crl_bundle += (
            b"-----BEGIN X509 CRL-----\n"
            + base64.b64encode(der)
            + b"\n-----END X509 CRL-----\n"
        )
    files["crl-bundle.pem"] = crl_bundle

    for name, content in files.items():
        (output_dir / name).write_bytes(content)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--recipe", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    recipe_path = Path(args.recipe)
    recipe = json.loads(recipe_path.read_text(encoding="utf-8"))
    output_dir = Path(args.output_dir)
    generate(recipe, output_dir)
    if args.check:
        ok = True
        for name, expected in sorted(recipe["expected_outputs"].items()):
            path = output_dir / name
            actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
            print(f"{name}: {actual}")
            ok = ok and actual == expected
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
