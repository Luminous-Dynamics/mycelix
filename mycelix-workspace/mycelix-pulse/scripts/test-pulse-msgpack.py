#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
from pulse_msgpack import Limits, MessagePackError, pack, unpack

fixture = {
    "manifest": {"name": "pulse", "roles": [{"name": "main", "deferred": False}]},
    "resources": {"dna": b"\x00\x01\x02"},
    "numbers": [0, 127, 128, 65536, -1, -129],
    "none": None,
}
assert unpack(pack(fixture)) == fixture

for payload, message in [
    (pack(1) + b"\x00", "trailing bytes"),
    (b"\x82\xa1a\x01\xa1a\x02", "duplicate"),
    (b"\xc7\x00\x00", "unsupported"),
]:
    try:
        unpack(payload)
    except MessagePackError as error:
        assert message in str(error)
    else:
        raise AssertionError(f"expected decoder rejection containing {message!r}")

try:
    unpack(pack([1, 2]), Limits(max_container_items=1))
except MessagePackError as error:
    assert "item limit" in str(error)
else:
    raise AssertionError("container limit was not enforced")

print("Pulse MessagePack codec self-test passed.")
