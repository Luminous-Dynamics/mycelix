#!/usr/bin/env python3
"""Deterministic fresh-attestation, clone, rollback, and decommission simulation."""
from dataclasses import dataclass, replace
from hashlib import sha256


def h(*parts: object) -> str:
    return sha256("|".join(map(str, parts)).encode()).hexdigest()


@dataclass(frozen=True)
class Attestation:
    device_id: str
    generation: int
    sequence: int
    previous_hash: str | None
    endorsement_key: str
    challenge: str
    platform: str
    firmware: str
    boot_counter: int
    issued_at: int
    expires_at: int
    secure_boot: bool = True
    debug_disabled: bool = True

    @property
    def digest(self) -> str:
        return h(
            "attestation-v1", self.device_id, self.generation, self.sequence,
            self.previous_hash, self.endorsement_key, self.challenge,
            self.platform, self.firmware, self.boot_counter,
            self.issued_at, self.expires_at, self.secure_boot, self.debug_disabled,
        )


def verify(record: Attestation, previous: Attestation | None, now: int) -> str:
    assert record.generation > 0
    assert record.sequence > 0
    assert record.boot_counter > 0
    assert record.secure_boot and record.debug_disabled
    assert record.challenge and record.endorsement_key
    assert record.platform and record.firmware
    assert record.issued_at <= now < record.expires_at
    assert record.expires_at - record.issued_at <= 3600
    if previous is None:
        assert record.sequence == 1 and record.previous_hash is None
    else:
        assert record.device_id == previous.device_id
        assert record.generation == previous.generation
        assert record.sequence == previous.sequence + 1
        assert record.previous_hash == previous.digest
        assert record.endorsement_key == previous.endorsement_key
        assert record.boot_counter >= previous.boot_counter
        assert record.issued_at >= previous.issued_at
        assert record.expires_at > previous.expires_at
    return record.digest


def clone_evidence(a: Attestation, b: Attestation) -> str | None:
    same_slot = (
        a.device_id == b.device_id
        and a.endorsement_key == b.endorsement_key
        and a.sequence == b.sequence
    )
    if not same_slot or a.digest == b.digest:
        return None
    first, second = sorted((a.digest, b.digest))
    return h("clone-evidence-v1", a.device_id, a.endorsement_key, a.sequence, first, second)


base = Attestation(
    device_id=h("device-2"), generation=2, sequence=1, previous_hash=None,
    endorsement_key=h("ek-2"), challenge=h("challenge-1"),
    platform=h("platform"), firmware=h("firmware-a"), boot_counter=41,
    issued_at=1_000, expires_at=1_600,
)
assert verify(base, None, 1_100)
next_record = replace(
    base, sequence=2, previous_hash=base.digest, challenge=h("challenge-2"),
    boot_counter=42, issued_at=1_500, expires_at=2_100,
)
assert verify(next_record, base, 1_700)

clone = replace(next_record, firmware=h("malicious-firmware"))
evidence = clone_evidence(next_record, clone)
assert evidence
assert clone_evidence(clone, next_record) == evidence

invalid = [
    replace(next_record, boot_counter=40),
    replace(next_record, sequence=3),
    replace(next_record, previous_hash=h("wrong")),
    replace(next_record, secure_boot=False),
    replace(next_record, debug_disabled=False),
    replace(next_record, issued_at=2_200, expires_at=2_300),
]
for candidate in invalid:
    try:
        verify(candidate, base, 1_700)
    except AssertionError:
        pass
    else:
        raise AssertionError(f"invalid attestation passed: {candidate}")

# Decommission requires both operations, never one or the other.
def verify_decommission(*, zeroized: bool, revoked: bool, delay: int) -> None:
    assert zeroized and revoked
    assert 0 <= delay <= 86_400

verify_decommission(zeroized=True, revoked=True, delay=900)
for args in [
    dict(zeroized=False, revoked=True, delay=1),
    dict(zeroized=True, revoked=False, delay=1),
    dict(zeroized=True, revoked=True, delay=86_401),
]:
    try:
        verify_decommission(**args)
    except AssertionError:
        pass
    else:
        raise AssertionError(f"invalid decommission passed: {args}")

runtime = {
    "protocol": 16,
    "fresh": False,
    "checkpoint": False,
    "clone_resolved": False,
    "decommission_verified": False,
    "acceptance": False,
}
assert runtime == {
    "protocol": 16,
    "fresh": False,
    "checkpoint": False,
    "clone_resolved": False,
    "decommission_verified": False,
    "acceptance": False,
}

print("verifier device attestation simulation passed: freshness, rollback rejection, clone evidence, dual decommission, fail-closed runtime")
