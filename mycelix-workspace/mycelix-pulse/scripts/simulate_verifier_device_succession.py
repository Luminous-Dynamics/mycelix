#!/usr/bin/env python3
"""Deterministic planned and lost-device custody succession simulation."""
from dataclasses import dataclass
from hashlib import sha256


def h(*parts: object) -> str:
    return sha256("|".join(map(str, parts)).encode()).hexdigest()


@dataclass(frozen=True)
class Device:
    slot: str
    generation: int
    serial: str
    measurement: str

    @property
    def device_id(self) -> str:
        return h("device-v1", self.slot, self.generation, self.serial, self.measurement)


@dataclass(frozen=True)
class Succession:
    mode: str
    sequence: int
    retiring: Device
    replacement: Device
    retiring_approves: bool
    replacement_approves: bool
    recovery_approvers: tuple[str, ...]
    loss_notice: str | None
    retiring_disabled: bool


def verify(s: Succession, recovery_threshold: int = 2) -> str:
    assert s.sequence > 0
    assert s.retiring.slot == s.replacement.slot
    assert s.replacement.generation == s.retiring.generation + 1
    assert s.retiring.device_id != s.replacement.device_id
    assert s.retiring.serial != s.replacement.serial
    assert s.retiring.measurement != s.replacement.measurement
    assert s.replacement_approves
    assert s.retiring_disabled
    if s.mode == "planned_dual_device":
        assert s.retiring_approves
        assert not s.recovery_approvers
        assert s.loss_notice is None
    elif s.mode == "lost_device_recovery":
        assert not s.retiring_approves
        assert s.loss_notice
        assert len(set(s.recovery_approvers)) >= recovery_threshold
    else:
        raise AssertionError("unknown succession mode")
    return h("succession-v1", s.mode, s.sequence, s.retiring.device_id, s.replacement.device_id)


slot = h("release-approval-slot")
d1 = Device(slot, 1, h("serial-1"), h("measurement-1"))
d2 = Device(slot, 2, h("serial-2"), h("measurement-2"))
d3 = Device(slot, 3, h("serial-3"), h("measurement-3"))

planned = Succession("planned_dual_device", 1, d1, d2, True, True, (), None, True)
planned_hash = verify(planned)
assert planned_hash

lost = Succession(
    "lost_device_recovery", 2, d2, d3, False, True,
    ("recovery-a", "recovery-b"), h("loss", d2.device_id), True,
)
assert verify(lost) != planned_hash

invalid = [
    Succession("planned_dual_device", 1, d1, d2, False, True, (), None, True),
    Succession("lost_device_recovery", 2, d2, d3, False, True, ("recovery-a",), h("loss"), True),
    Succession("lost_device_recovery", 2, d2, Device("other-slot", 3, h("s"), h("m")), False, True, ("a", "b"), h("loss"), True),
    Succession("planned_dual_device", 1, d1, Device(slot, 3, h("skip"), h("skip-m")), True, True, (), None, True),
    Succession("planned_dual_device", 1, d1, d2, True, True, (), None, False),
]
for candidate in invalid:
    try:
        verify(candidate)
    except AssertionError:
        pass
    else:
        raise AssertionError(f"invalid succession passed: {candidate}")

runtime = {
    "protocol": 15,
    "device_succession_continuity_verified": False,
    "device_succession_checkpoint_pinned": False,
    "proof_acceptance_enabled": False,
}
assert runtime["protocol"] == 15
assert not runtime["device_succession_continuity_verified"]
assert not runtime["device_succession_checkpoint_pinned"]
assert not runtime["proof_acceptance_enabled"]

print("verifier device succession simulation passed: dual-device handoff, recovery quorum, exact generation, stable slot, fail-closed runtime")
