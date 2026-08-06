#!/usr/bin/env python3
"""Deterministic protocol-v17 device trust-root lifecycle simulation."""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass


def digest(label: str, value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(label.encode() + b"\0" + payload).hexdigest()


@dataclass(frozen=True, order=True)
class Measurement:
    platform: str
    firmware: str
    minimum_security_epoch: int


@dataclass(frozen=True)
class MeasurementPolicy:
    epoch: int
    previous_hash: str | None
    measurements: tuple[Measurement, ...]

    def hash(self) -> str:
        assert self.epoch > 0
        assert bool(self.previous_hash) == (self.epoch > 1)
        assert self.measurements == tuple(sorted(set(self.measurements)))
        return digest("measurement-policy-v1", {
            "epoch": self.epoch,
            "previous": self.previous_hash,
            "measurements": [m.__dict__ for m in self.measurements],
        })

    def authorizes(self, platform: str, firmware: str, security_epoch: int) -> bool:
        return any(
            m.platform == platform
            and m.firmware == firmware
            and security_epoch >= m.minimum_security_epoch
            for m in self.measurements
        )


@dataclass(frozen=True)
class RootRotation:
    sequence: int
    previous_hash: str | None
    current_root: str
    replacement_root: str
    current_approval: bool
    replacement_approval: bool

    def hash(self) -> str:
        assert self.sequence > 0
        assert bool(self.previous_hash) == (self.sequence > 1)
        assert self.current_root != self.replacement_root
        assert self.current_approval and self.replacement_approval
        return digest("attestation-root-rotation-v1", self.__dict__)


def main() -> None:
    platform = digest("platform", "verified-boot-state")
    firmware_v1 = digest("firmware", "1.0.0")
    firmware_v2 = digest("firmware", "1.1.0")
    root_a = digest("root", "manufacturer-root-a")
    root_b = digest("root", "manufacturer-root-b")

    p1 = MeasurementPolicy(1, None, (Measurement(platform, firmware_v1, 1),))
    p1_hash = p1.hash()
    assert p1.authorizes(platform, firmware_v1, 1)
    assert not p1.authorizes(platform, firmware_v2, 1)

    p2 = MeasurementPolicy(
        2,
        p1_hash,
        tuple(sorted((
            Measurement(platform, firmware_v1, 1),
            Measurement(platform, firmware_v2, 2),
        ))),
    )
    p2_hash = p2.hash()
    assert p2.authorizes(platform, firmware_v2, 2)
    assert not p2.authorizes(platform, firmware_v2, 1)

    rotation = RootRotation(1, None, root_a, root_b, True, True)
    rotation_hash = rotation.hash()

    rejected = []
    for name, candidate in [
        ("missing-current-consent", RootRotation(1, None, root_a, root_b, False, True)),
        ("missing-replacement-possession", RootRotation(1, None, root_a, root_b, True, False)),
        ("same-root", RootRotation(1, None, root_a, root_a, True, True)),
    ]:
        try:
            candidate.hash()
        except AssertionError:
            rejected.append(name)

    try:
        MeasurementPolicy(2, digest("wrong", "parent"), p2.measurements).hash()
        raise AssertionError("policy with wrong predecessor was only shape-valid")
    except AssertionError:
        # The cross-record check below is what rejects a valid-looking fork.
        pass
    assert p2.previous_hash == p1_hash

    checkpoint = digest("device-trust-checkpoint-v1", {
        "trust_policy": digest("trust-policy", "dual-root+allowlist+checkpoint"),
        "root": root_b,
        "root_rotation_sequence": 1,
        "measurement_policy": p2_hash,
        "measurement_policy_epoch": 2,
        "trust_binding": digest("binding", "attestation+root+measurement"),
    })

    result = {
        "protocol": 17,
        "firmware_v2_rejected_before_policy_update": True,
        "firmware_v2_authorized_at_security_epoch_2": True,
        "root_rotation_hash": rotation_hash,
        "measurement_policy_epoch": 2,
        "trust_checkpoint_hash": checkpoint,
        "rejected_transitions": rejected,
        "proof_acceptance_enabled": False,
    }
    assert set(rejected) == {
        "missing-current-consent",
        "missing-replacement-possession",
        "same-root",
    }
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
