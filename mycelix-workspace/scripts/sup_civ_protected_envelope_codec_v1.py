#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

SCHEMA_V1 = 1
MAX_RECIPIENTS = 64
MAX_WRAPPED_DEK_LEN = 4096
MAX_METADATA_BYTES = 512 * 1024
MAX_CIPHERTEXT_LEN = (1 << 63) - 1

MAGIC_AAD = b"mycelix/protected-envelope/payload-aad/v1\x00"
MAGIC_WRAP = b"mycelix/protected-envelope/recipient-wrap/v1\x00"
MAGIC_FULL = b"mycelix/protected-envelope/full/v1\x00"

ZERO32 = b"\x00" * 32


class CodecError(ValueError):
    pass


def _u16(n: int) -> bytes:
    if not 0 <= n <= 0xFFFF:
        raise CodecError("u16-range")
    return struct.pack(">H", n)


def _u64(n: int) -> bytes:
    if not 0 <= n <= 0xFFFFFFFFFFFFFFFF:
        raise CodecError("u64-range")
    return struct.pack(">Q", n)


def _fixed32(hex_value: str, field: str) -> bytes:
    try:
        raw = bytes.fromhex(hex_value)
    except ValueError as exc:
        raise CodecError(f"{field}-hex") from exc
    if len(raw) != 32:
        raise CodecError(f"{field}-length")
    if raw == ZERO32:
        raise CodecError(f"{field}-zero")
    return raw


def _profile(n: int, field: str) -> bytes:
    if not isinstance(n, int) or n <= 0 or n > 0xFFFF:
        raise CodecError(f"{field}-profile")
    return _u16(n)


def _version(n: int, field: str) -> bytes:
    if not isinstance(n, int) or n <= 0:
        raise CodecError(f"{field}-version")
    return _u64(n)


def _recipient_key(r: dict[str, Any]) -> tuple[bytes, bytes, int, int]:
    return (
        _fixed32(r["recipient_subject_commitment"], "recipient-subject"),
        _fixed32(r["recipient_key_identity_commitment"], "recipient-key"),
        int(r["recipient_key_profile_id"]),
        int(r["wrap_profile_id"]),
    )


def _validate_recipients(recipients: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    items = [dict(r) for r in recipients]
    if not 1 <= len(items) <= MAX_RECIPIENTS:
        raise CodecError("recipient-count")

    seen_subjects: set[bytes] = set()
    seen_keys: set[bytes] = set()
    for r in items:
        subject, key, key_profile, wrap_profile = _recipient_key(r)
        if subject in seen_subjects:
            raise CodecError("duplicate-recipient-subject")
        if key in seen_keys:
            raise CodecError("duplicate-recipient-key")
        seen_subjects.add(subject)
        seen_keys.add(key)
        _profile(key_profile, "recipient-key")
        _profile(wrap_profile, "wrap")
        try:
            wrapped = bytes.fromhex(r["wrapped_dek_hex"])
        except ValueError as exc:
            raise CodecError("wrapped-dek-hex") from exc
        if not 1 <= len(wrapped) <= MAX_WRAPPED_DEK_LEN:
            raise CodecError("wrapped-dek-length")

    items.sort(key=_recipient_key)
    return items


def encode_payload_aad(obj: dict[str, Any]) -> bytes:
    return b"".join([
        MAGIC_AAD,
        _u16(SCHEMA_V1),
        _profile(int(obj["aead_profile_id"]), "aead"),
        _fixed32(obj["payload_subject_commitment"], "payload-subject"),
        _version(int(obj["payload_version"]), "payload"),
        _version(int(obj["key_epoch"]), "key-epoch"),
    ])


def encode_recipient_wrap_context(obj: dict[str, Any], recipient: dict[str, Any]) -> bytes:
    return b"".join([
        MAGIC_WRAP,
        _u16(SCHEMA_V1),
        _profile(int(obj["aead_profile_id"]), "aead"),
        _profile(int(recipient["recipient_key_profile_id"]), "recipient-key"),
        _profile(int(recipient["wrap_profile_id"]), "wrap"),
        _fixed32(obj["payload_subject_commitment"], "payload-subject"),
        _version(int(obj["payload_version"]), "payload"),
        _version(int(obj["key_epoch"]), "key-epoch"),
        _fixed32(recipient["recipient_subject_commitment"], "recipient-subject"),
        _fixed32(recipient["recipient_key_identity_commitment"], "recipient-key"),
    ])


def encode_envelope(obj: dict[str, Any]) -> bytes:
    recipients = _validate_recipients(obj["recipients"])
    ct_len = int(obj["ciphertext_len"])
    if not 1 <= ct_len <= MAX_CIPHERTEXT_LEN:
        raise CodecError("ciphertext-length")

    prev = obj.get("previous_envelope_commitment")
    if prev is None:
        prev_part = b"\x00"
    else:
        prev_part = b"\x01" + _fixed32(prev, "previous-envelope")

    out = bytearray()
    out += MAGIC_FULL
    out += _u16(SCHEMA_V1)
    out += _profile(int(obj["commitment_profile_id"]), "commitment")
    out += _profile(int(obj["aead_profile_id"]), "aead")
    out += _fixed32(obj["payload_subject_commitment"], "payload-subject")
    out += _version(int(obj["payload_version"]), "payload")
    out += _version(int(obj["envelope_revision"]), "envelope")
    out += _version(int(obj["key_epoch"]), "key-epoch")
    out += _fixed32(obj["ciphertext_commitment"], "ciphertext")
    out += _u64(ct_len)
    out += prev_part
    out += _u16(len(recipients))

    for r in recipients:
        wrapped = bytes.fromhex(r["wrapped_dek_hex"])
        out += _fixed32(r["recipient_subject_commitment"], "recipient-subject")
        out += _fixed32(r["recipient_key_identity_commitment"], "recipient-key")
        out += _profile(int(r["recipient_key_profile_id"]), "recipient-key")
        out += _profile(int(r["wrap_profile_id"]), "wrap")
        out += _u16(len(wrapped))
        out += wrapped

    if len(out) > MAX_METADATA_BYTES:
        raise CodecError("metadata-too-large")
    return bytes(out)


@dataclass
class Reader:
    data: bytes
    off: int = 0

    def take(self, n: int, err: str = "truncated") -> bytes:
        if n < 0 or self.off + n > len(self.data):
            raise CodecError(err)
        part = self.data[self.off:self.off+n]
        self.off += n
        return part

    def u16(self) -> int:
        return struct.unpack(">H", self.take(2))[0]

    def u64(self) -> int:
        return struct.unpack(">Q", self.take(8))[0]

    def fixed32(self, field: str) -> bytes:
        raw = self.take(32)
        if raw == ZERO32:
            raise CodecError(f"{field}-zero")
        return raw


def decode_envelope(raw: bytes) -> dict[str, Any]:
    if len(raw) > MAX_METADATA_BYTES:
        raise CodecError("metadata-too-large")
    r = Reader(raw)
    if r.take(len(MAGIC_FULL), "bad-magic") != MAGIC_FULL:
        raise CodecError("bad-magic")
    if r.u16() != SCHEMA_V1:
        raise CodecError("unsupported-schema")
    commitment_profile = r.u16()
    aead_profile = r.u16()
    if commitment_profile == 0:
        raise CodecError("commitment-profile")
    if aead_profile == 0:
        raise CodecError("aead-profile")
    payload_subject = r.fixed32("payload-subject")
    payload_version = r.u64()
    envelope_revision = r.u64()
    key_epoch = r.u64()
    if payload_version == 0:
        raise CodecError("payload-version")
    if envelope_revision == 0:
        raise CodecError("envelope-version")
    if key_epoch == 0:
        raise CodecError("key-epoch-version")
    ciphertext_commitment = r.fixed32("ciphertext")
    ciphertext_len = r.u64()
    if not 1 <= ciphertext_len <= MAX_CIPHERTEXT_LEN:
        raise CodecError("ciphertext-length")

    prev_tag = r.take(1)[0]
    if prev_tag == 0:
        prev = None
    elif prev_tag == 1:
        prev = r.fixed32("previous-envelope")
    else:
        raise CodecError("previous-envelope-tag")

    count = r.u16()
    if not 1 <= count <= MAX_RECIPIENTS:
        raise CodecError("recipient-count")

    recipients = []
    seen_subjects: set[bytes] = set()
    seen_keys: set[bytes] = set()
    last_key: tuple[bytes, bytes, int, int] | None = None
    for _ in range(count):
        subject = r.fixed32("recipient-subject")
        key = r.fixed32("recipient-key")
        key_profile = r.u16()
        wrap_profile = r.u16()
        if key_profile == 0:
            raise CodecError("recipient-key-profile")
        if wrap_profile == 0:
            raise CodecError("wrap-profile")
        wrapped_len = r.u16()
        if not 1 <= wrapped_len <= MAX_WRAPPED_DEK_LEN:
            raise CodecError("wrapped-dek-length")
        wrapped = r.take(wrapped_len)

        if subject in seen_subjects:
            raise CodecError("duplicate-recipient-subject")
        if key in seen_keys:
            raise CodecError("duplicate-recipient-key")
        seen_subjects.add(subject)
        seen_keys.add(key)

        cur = (subject, key, key_profile, wrap_profile)
        if last_key is not None and cur <= last_key:
            raise CodecError("noncanonical-recipient-order")
        last_key = cur

        recipients.append({
            "recipient_subject_commitment": subject.hex(),
            "recipient_key_identity_commitment": key.hex(),
            "recipient_key_profile_id": key_profile,
            "wrap_profile_id": wrap_profile,
            "wrapped_dek_hex": wrapped.hex(),
        })

    if r.off != len(raw):
        raise CodecError("trailing-bytes")

    return {
        "schema_version": SCHEMA_V1,
        "commitment_profile_id": commitment_profile,
        "aead_profile_id": aead_profile,
        "payload_subject_commitment": payload_subject.hex(),
        "payload_version": payload_version,
        "envelope_revision": envelope_revision,
        "key_epoch": key_epoch,
        "ciphertext_commitment": ciphertext_commitment.hex(),
        "ciphertext_len": ciphertext_len,
        "previous_envelope_commitment": None if prev is None else prev.hex(),
        "recipients": recipients,
    }


def _repeat32(byte_value: int) -> str:
    if not 1 <= int(byte_value) <= 255:
        raise AssertionError("fixture byte must be 1..255")
    return (bytes([int(byte_value)]) * 32).hex()


def materialize_fixture(fixtures: dict[str, Any], name: str) -> dict[str, Any]:
    spec = dict(fixtures[name])
    if "derive" in spec:
        base = materialize_fixture(fixtures, spec.pop("derive"))
        reverse = bool(spec.pop("reverse_recipients", False))
        for k, v in spec.items():
            if k == "ciphertext_byte":
                base["ciphertext_commitment"] = _repeat32(v)
            elif k == "previous_byte":
                base["previous_envelope_commitment"] = None if v is None else _repeat32(v)
            else:
                base[k] = v
        if reverse:
            base["recipients"] = list(reversed(base["recipients"]))
        return base

    obj = {
        "commitment_profile_id": spec["commitment_profile_id"],
        "aead_profile_id": spec["aead_profile_id"],
        "payload_subject_commitment": _repeat32(spec["payload_subject_byte"]),
        "payload_version": spec["payload_version"],
        "envelope_revision": spec["envelope_revision"],
        "key_epoch": spec["key_epoch"],
        "ciphertext_commitment": _repeat32(spec["ciphertext_byte"]),
        "ciphertext_len": spec["ciphertext_len"],
        "previous_envelope_commitment": (
            None if spec["previous_byte"] is None else _repeat32(spec["previous_byte"])
        ),
        "recipients": [],
    }
    for r in spec["recipients"]:
        obj["recipients"].append({
            "recipient_subject_commitment": _repeat32(r["subject_byte"]),
            "recipient_key_identity_commitment": _repeat32(r["key_byte"]),
            "recipient_key_profile_id": r["key_profile"],
            "wrap_profile_id": r["wrap_profile"],
            "wrapped_dek_hex": (bytes([r["wrapped_byte"]]) * r["wrapped_len"]).hex(),
        })
    return obj


def mutate_negative(raw: bytes, mutation: str) -> bytes:
    x = bytearray(raw)

    def overwrite(offset: int, data: bytes) -> bytes:
        y = bytearray(x)
        y[offset:offset+len(data)] = data
        return bytes(y)

    if mutation == "bad_magic":
        return overwrite(0, b"X")
    if mutation == "unsupported_schema":
        return overwrite(35, struct.pack(">H", 2))
    if mutation == "zero_commitment_profile":
        return overwrite(37, b"\x00\x00")
    if mutation == "zero_aead_profile":
        return overwrite(39, b"\x00\x00")
    if mutation == "zero_payload_subject":
        return overwrite(41, b"\x00" * 32)
    if mutation == "zero_payload_version":
        return overwrite(73, b"\x00" * 8)
    if mutation == "zero_envelope_revision":
        return overwrite(81, b"\x00" * 8)
    if mutation == "zero_key_epoch":
        return overwrite(89, b"\x00" * 8)
    if mutation == "zero_ciphertext_commitment":
        return overwrite(97, b"\x00" * 32)
    if mutation == "zero_ciphertext_length":
        return overwrite(129, b"\x00" * 8)
    if mutation == "invalid_previous_tag":
        return overwrite(137, b"\x02")
    if mutation == "zero_recipient_count":
        return overwrite(138, b"\x00\x00")
    if mutation == "truncated_recipient":
        return bytes(x[:-5])
    if mutation == "duplicate_recipient_subject":
        return overwrite(258, bytes(x[140:172]))
    if mutation == "duplicate_recipient_key":
        return overwrite(290, bytes(x[172:204]))
    if mutation == "noncanonical_recipient_order":
        first = bytes(x[140:258])
        second = bytes(x[258:])
        return bytes(x[:140]) + second + first
    if mutation == "zero_recipient_key_profile":
        return overwrite(204, b"\x00\x00")
    if mutation == "zero_wrap_profile":
        return overwrite(206, b"\x00\x00")
    if mutation == "zero_wrapped_dek_length":
        return overwrite(208, b"\x00\x00")
    if mutation == "wrapped_dek_length_overflow":
        return overwrite(208, struct.pack(">H", MAX_WRAPPED_DEK_LEN + 1))
    if mutation == "trailing_bytes":
        return bytes(x) + b"\x00"
    if mutation == "metadata_too_large":
        target = MAX_METADATA_BYTES + 1
        return bytes(x) + (b"\x00" * (target - len(x)))
    raise AssertionError(f"unknown mutation {mutation}")


def _assert_exact_vector(vec: dict[str, Any], fixtures: dict[str, Any]) -> None:
    obj = materialize_fixture(fixtures, vec["fixture"])
    kind = vec["kind"]
    if kind == "payload_aad":
        actual = encode_payload_aad(obj)
    elif kind == "recipient_wrap_context":
        recipient = obj["recipients"][int(vec["recipient_index"])]
        actual = encode_recipient_wrap_context(obj, recipient)
    elif kind == "full_envelope":
        actual = encode_envelope(obj)
        decoded = decode_envelope(actual)
        recoded = encode_envelope(decoded)
        if recoded != actual:
            raise AssertionError(f"{vec['id']}: decode/re-encode mismatch")
    else:
        raise AssertionError(f"unknown vector kind {kind}")

    expected = bytes.fromhex(vec["expected_hex"])
    if actual != expected:
        raise AssertionError(f"{vec['id']}: byte mismatch")
    if hashlib.sha256(actual).hexdigest() != vec["fixture_sha256"]:
        raise AssertionError(f"{vec['id']}: fixture sha256 mismatch")


def execute(vectors: dict[str, Any]) -> dict[str, Any]:
    if vectors["schema"] != "sup-civ-000d1b-vectors-v1":
        raise AssertionError("wrong vector schema")
    fixtures = vectors["fixtures"]
    positives = vectors["positive_vectors"]
    negatives = vectors["negative_vectors"]

    for vec in positives:
        _assert_exact_vector(vec, fixtures)

    positive_by_id = {v["id"]: v for v in positives}
    for vec in negatives:
        source = positive_by_id[vec["base"]]
        raw = bytes.fromhex(source["expected_hex"])
        raw = mutate_negative(raw, vec["mutation"])
        try:
            decode_envelope(raw)
        except CodecError as exc:
            if str(exc) != vec["expected_error"]:
                raise AssertionError(
                    f"{vec['id']}: expected {vec['expected_error']!r}, got {str(exc)!r}"
                ) from exc
        else:
            raise AssertionError(f"{vec['id']}: expected rejection")

    return {
        "schema": "sup-civ-000d1b-reference-receipt-v1",
        "vector_schema": vectors["schema"],
        "positive_count": len(positives),
        "negative_count": len(negatives),
        "canonical_bytes_authority": "normative-v1-transcript",
        "fixture_sha256_authority": "fixture-integrity-only-not-runtime-envelope-id",
        "all_vectors_passed": True,
        "vector_file_sha256": vectors["_file_sha256"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vectors", required=True)
    parser.add_argument("--receipt")
    args = parser.parse_args()

    path = Path(args.vectors)
    raw = path.read_bytes()
    vectors = json.loads(raw)
    vectors["_file_sha256"] = hashlib.sha256(raw).hexdigest()
    receipt = execute(vectors)
    text = json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n"
    if args.receipt:
        Path(args.receipt).write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
