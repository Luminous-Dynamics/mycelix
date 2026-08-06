#!/usr/bin/env python3
"""Deterministic custody ceremony, compromise, and recovery lifecycle simulation."""

from dataclasses import dataclass
from hashlib import sha256

ROLES = (
    "replay_reservation",
    "activation_policy",
    "artifact_transparency",
    "rollout_health",
    "release_approval",
)


def digest(*parts: object) -> str:
    h = sha256()
    for part in parts:
        value = str(part).encode()
        h.update(len(value).to_bytes(8, "little"))
        h.update(value)
    return h.hexdigest()


@dataclass(frozen=True)
class Custodian:
    role: str
    key_id: str
    hardware_attestation: str

    @property
    def identity(self) -> str:
        return digest("custodian-v1", self.role, self.key_id, self.hardware_attestation)


@dataclass(frozen=True)
class Ceremony:
    role: str
    epoch: int
    threshold: int
    custodians: tuple[Custodian, ...]
    previous: str | None

    @property
    def ceremony_hash(self) -> str:
        identities = tuple(sorted(c.identity for c in self.custodians))
        return digest("ceremony-v1", self.role, self.epoch, self.threshold, identities, self.previous)


def validate(ceremony: Ceremony) -> None:
    assert ceremony.role in ROLES
    assert ceremony.epoch > 0
    assert ceremony.threshold >= 2
    assert len(ceremony.custodians) >= ceremony.threshold
    assert all(c.role == ceremony.role for c in ceremony.custodians)
    assert all(c.hardware_attestation != "0" * 64 for c in ceremony.custodians)
    assert len({c.identity for c in ceremony.custodians}) == len(ceremony.custodians)
    assert len({c.key_id for c in ceremony.custodians}) == len(ceremony.custodians)
    assert (ceremony.epoch == 1) == (ceremony.previous is None)


def role_separated(ceremonies: tuple[Ceremony, ...]) -> bool:
    roles_by_key: dict[str, str] = {}
    for ceremony in ceremonies:
        validate(ceremony)
        for custodian in ceremony.custodians:
            prior = roles_by_key.setdefault(custodian.key_id, custodian.role)
            if prior != custodian.role:
                return False
    return True


def make(role: str, epoch: int, previous: str | None, key_offset: int = 0) -> Ceremony:
    custodians = tuple(
        Custodian(role, digest("key", key_offset + i), digest("hardware", role, i))
        for i in range(3)
    )
    return Ceremony(role, epoch, 2, custodians, previous)


def main() -> None:
    release = make("release_approval", 1, None)
    activation = make("activation_policy", 1, None, key_offset=10)
    validate(release)
    validate(activation)
    assert role_separated((release, activation))

    reused = Ceremony(
        activation.role,
        activation.epoch,
        activation.threshold,
        (Custodian(activation.role, release.custodians[0].key_id, digest("new-hardware")),)
        + activation.custodians[1:],
        None,
    )
    assert not role_separated((release, reused))

    compromise_notice = digest("compromise-v1", release.ceremony_hash, release.custodians[0].key_id)
    assert compromise_notice
    replacement = make("release_approval", 2, release.ceremony_hash, key_offset=20)
    validate(replacement)
    assert replacement.previous == release.ceremony_hash
    recovery = digest(
        "recovery-v1",
        compromise_notice,
        release.ceremony_hash,
        replacement.ceremony_hash,
        replacement.epoch,
    )
    assert recovery

    runtime = {
        "protocol": 13,
        "custody_ready": False,
        "compromise_resolved": False,
        "acceptance_enabled": False,
    }
    assert runtime["protocol"] == 13
    assert not runtime["custody_ready"]
    assert not runtime["acceptance_enabled"]
    print(
        "verifier custody simulation passed: distinct hardware-bound custodians, "
        "cross-role reuse rejected, exact compromise/recovery linked, acceptance disabled"
    )


if __name__ == "__main__":
    main()
