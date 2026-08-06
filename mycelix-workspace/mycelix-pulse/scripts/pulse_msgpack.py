#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Small, dependency-free MessagePack codec for Holochain bundle inspection.

The decoder intentionally supports only the data types used by Holochain bundle
manifests/resources and rejects extension values, duplicate map keys, excessive
nesting, oversized containers, and trailing bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any


class MessagePackError(ValueError):
    pass


@dataclass
class Limits:
    max_depth: int = 32
    max_container_items: int = 4096
    max_blob_bytes: int = 256 * 1024 * 1024


class Decoder:
    def __init__(self, payload: bytes, limits: Limits | None = None):
        self.payload = memoryview(payload)
        self.offset = 0
        self.limits = limits or Limits()

    def _read(self, length: int) -> memoryview:
        if length < 0 or self.offset + length > len(self.payload):
            raise MessagePackError("truncated MessagePack payload")
        value = self.payload[self.offset:self.offset + length]
        self.offset += length
        return value

    def _uint(self, length: int) -> int:
        return int.from_bytes(self._read(length), "big", signed=False)

    def _int(self, length: int) -> int:
        return int.from_bytes(self._read(length), "big", signed=True)

    def _blob_length(self, length: int) -> int:
        if length > self.limits.max_blob_bytes:
            raise MessagePackError("MessagePack string/binary value exceeds size limit")
        return length

    def _container_length(self, length: int) -> int:
        if length > self.limits.max_container_items:
            raise MessagePackError("MessagePack container exceeds item limit")
        return length

    def decode_one(self, depth: int = 0) -> Any:
        if depth > self.limits.max_depth:
            raise MessagePackError("MessagePack nesting exceeds depth limit")
        prefix = self._uint(1)

        if prefix <= 0x7F:
            return prefix
        if prefix >= 0xE0:
            return prefix - 256
        if 0xA0 <= prefix <= 0xBF:
            return bytes(self._read(self._blob_length(prefix & 0x1F))).decode("utf-8")
        if 0x90 <= prefix <= 0x9F:
            return [self.decode_one(depth + 1) for _ in range(self._container_length(prefix & 0x0F))]
        if 0x80 <= prefix <= 0x8F:
            return self._decode_map(prefix & 0x0F, depth)

        if prefix == 0xC0:
            return None
        if prefix == 0xC2:
            return False
        if prefix == 0xC3:
            return True
        if prefix == 0xC4:
            return bytes(self._read(self._blob_length(self._uint(1))))
        if prefix == 0xC5:
            return bytes(self._read(self._blob_length(self._uint(2))))
        if prefix == 0xC6:
            return bytes(self._read(self._blob_length(self._uint(4))))
        if prefix == 0xCA:
            return struct.unpack(">f", self._read(4))[0]
        if prefix == 0xCB:
            return struct.unpack(">d", self._read(8))[0]
        if prefix == 0xCC:
            return self._uint(1)
        if prefix == 0xCD:
            return self._uint(2)
        if prefix == 0xCE:
            return self._uint(4)
        if prefix == 0xCF:
            return self._uint(8)
        if prefix == 0xD0:
            return self._int(1)
        if prefix == 0xD1:
            return self._int(2)
        if prefix == 0xD2:
            return self._int(4)
        if prefix == 0xD3:
            return self._int(8)
        if prefix == 0xD9:
            return bytes(self._read(self._blob_length(self._uint(1)))).decode("utf-8")
        if prefix == 0xDA:
            return bytes(self._read(self._blob_length(self._uint(2)))).decode("utf-8")
        if prefix == 0xDB:
            return bytes(self._read(self._blob_length(self._uint(4)))).decode("utf-8")
        if prefix == 0xDC:
            return [self.decode_one(depth + 1) for _ in range(self._container_length(self._uint(2)))]
        if prefix == 0xDD:
            return [self.decode_one(depth + 1) for _ in range(self._container_length(self._uint(4)))]
        if prefix == 0xDE:
            return self._decode_map(self._uint(2), depth)
        if prefix == 0xDF:
            return self._decode_map(self._uint(4), depth)
        raise MessagePackError(f"unsupported MessagePack prefix 0x{prefix:02x}")

    def _decode_map(self, length: int, depth: int) -> dict[Any, Any]:
        result: dict[Any, Any] = {}
        for _ in range(self._container_length(length)):
            key = self.decode_one(depth + 1)
            if not isinstance(key, (str, int, bytes, bool, type(None))):
                raise MessagePackError("unsupported MessagePack map key type")
            if key in result:
                raise MessagePackError("duplicate MessagePack map key")
            result[key] = self.decode_one(depth + 1)
        return result


def unpack(payload: bytes, limits: Limits | None = None) -> Any:
    decoder = Decoder(payload, limits)
    value = decoder.decode_one()
    if decoder.offset != len(decoder.payload):
        raise MessagePackError("trailing bytes after MessagePack value")
    return value


def _encode_length(prefix_small: int, small_limit: int, code16: int, code32: int, length: int) -> bytes:
    if length < small_limit:
        return bytes([prefix_small | length])
    if length <= 0xFFFF:
        return bytes([code16]) + length.to_bytes(2, "big")
    if length <= 0xFFFFFFFF:
        return bytes([code32]) + length.to_bytes(4, "big")
    raise MessagePackError("container is too large to encode")


def pack(value: Any) -> bytes:
    """Encode the safe subset used by verifier self-tests."""
    if value is None:
        return b"\xc0"
    if value is False:
        return b"\xc2"
    if value is True:
        return b"\xc3"
    if isinstance(value, int):
        if 0 <= value <= 0x7F:
            return bytes([value])
        if -32 <= value < 0:
            return bytes([value & 0xFF])
        if 0 <= value <= 0xFF:
            return b"\xcc" + value.to_bytes(1, "big")
        if 0 <= value <= 0xFFFF:
            return b"\xcd" + value.to_bytes(2, "big")
        if 0 <= value <= 0xFFFFFFFF:
            return b"\xce" + value.to_bytes(4, "big")
        if 0 <= value <= 0xFFFFFFFFFFFFFFFF:
            return b"\xcf" + value.to_bytes(8, "big")
        if -0x80 <= value < 0:
            return b"\xd0" + value.to_bytes(1, "big", signed=True)
        if -0x8000 <= value < 0:
            return b"\xd1" + value.to_bytes(2, "big", signed=True)
        if -0x80000000 <= value < 0:
            return b"\xd2" + value.to_bytes(4, "big", signed=True)
        return b"\xd3" + value.to_bytes(8, "big", signed=True)
    if isinstance(value, str):
        payload = value.encode("utf-8")
        length = len(payload)
        if length < 32:
            return bytes([0xA0 | length]) + payload
        if length <= 0xFF:
            return b"\xd9" + length.to_bytes(1, "big") + payload
        if length <= 0xFFFF:
            return b"\xda" + length.to_bytes(2, "big") + payload
        return b"\xdb" + length.to_bytes(4, "big") + payload
    if isinstance(value, (bytes, bytearray, memoryview)):
        payload = bytes(value)
        length = len(payload)
        if length <= 0xFF:
            return b"\xc4" + length.to_bytes(1, "big") + payload
        if length <= 0xFFFF:
            return b"\xc5" + length.to_bytes(2, "big") + payload
        return b"\xc6" + length.to_bytes(4, "big") + payload
    if isinstance(value, (list, tuple)):
        return _encode_length(0x90, 16, 0xDC, 0xDD, len(value)) + b"".join(pack(item) for item in value)
    if isinstance(value, dict):
        return _encode_length(0x80, 16, 0xDE, 0xDF, len(value)) + b"".join(
            pack(key) + pack(item) for key, item in value.items()
        )
    raise MessagePackError(f"unsupported value type: {type(value).__name__}")
