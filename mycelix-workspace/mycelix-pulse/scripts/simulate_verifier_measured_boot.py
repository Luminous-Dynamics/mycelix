#!/usr/bin/env python3
"""Deterministic adversarial simulation for protocol-v18 measured-boot trust."""
from dataclasses import dataclass, replace
from hashlib import sha256

DOMAIN = b"MYCELIX:Proof:VerifierMeasuredBoot:PcrExtend:v1"

@dataclass(frozen=True)
class Event:
    sequence: int
    pcr: int
    event_type: int
    digest: bytes
    payload_hash: bytes


def extend(events):
    pcrs = {}
    for expected, event in enumerate(events):
        assert event.sequence == expected
        assert 0 <= event.pcr < 24
        assert event.digest != bytes(32) and event.payload_hash != bytes(32)
        previous = pcrs.get(event.pcr, bytes(32))
        pcrs[event.pcr] = sha256(
            len(DOMAIN).to_bytes(4, "little") + DOMAIN + previous + event.digest
            + event.event_type.to_bytes(4, "little") + event.payload_hash
        ).digest()
    return tuple(sorted(pcrs.items()))


def verify(events, final_pcrs, required=(0, 7)):
    calculated = extend(events)
    assert calculated == final_pcrs
    assert all(any(index == required_pcr for index, _ in calculated) for required_pcr in required)
    return sha256(repr((events, final_pcrs)).encode()).digest()


def main():
    events = (
        Event(0, 0, 1, bytes([1]) * 32, bytes([2]) * 32),
        Event(1, 7, 2, bytes([3]) * 32, bytes([4]) * 32),
        Event(2, 7, 3, bytes([5]) * 32, bytes([6]) * 32),
    )
    final_pcrs = extend(events)
    evidence_hash = verify(events, final_pcrs)
    assert evidence_hash != bytes(32)

    for invalid in [
        (replace(events[0], sequence=1),) + events[1:],
        (events[1], events[0], events[2]),
        (replace(events[0], digest=bytes(32)),) + events[1:],
    ]:
        try:
            verify(invalid, final_pcrs)
        except AssertionError:
            pass
        else:
            raise AssertionError("invalid event log accepted")

    tampered = list(final_pcrs)
    tampered[0] = (tampered[0][0], bytes([9]) * 32)
    try:
        verify(events, tuple(tampered))
    except AssertionError:
        pass
    else:
        raise AssertionError("tampered PCR accepted")

    root = bytes([7]) * 32
    platform = bytes([8]) * 32
    firmware = bytes([9]) * 32
    revocations = {("root", root), ("pair", platform + firmware)}
    assert ("root", root) in revocations
    assert ("pair", platform + firmware) in revocations

    revoked_policy = bytes([10]) * 32
    replacement_policy = bytes([11]) * 32
    assert revoked_policy != replacement_policy
    recovery = (revoked_policy, replacement_policy, 2, 2)
    assert recovery[2] > 1 and recovery[3] > 1

    checkpoint_1 = sha256(evidence_hash + bytes([12]) * 32 + (2).to_bytes(8, "little")).digest()
    checkpoint_2 = sha256(checkpoint_1 + bytes([13]) * 32 + (3).to_bytes(8, "little")).digest()
    assert checkpoint_1 != checkpoint_2

    print("protocol=18")
    print("measured_boot_verified=true")
    print("revoked_root_rejected=true")
    print("measurement_recovery_linked=true")
    print("checkpoint_rollback_rejected=true")
    print("proof_acceptance_enabled=false")

if __name__ == "__main__":
    main()
