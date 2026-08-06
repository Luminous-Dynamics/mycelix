#!/usr/bin/env python3
"""Deterministic model of retained telemetry, rollback drills, and release quorum."""
from __future__ import annotations
from dataclasses import dataclass, replace
import hashlib
import json

MIN_RETENTION = 7 * 24 * 60 * 60
MAX_GAP = 60
MIN_RECOVERY_REQUESTS = 16
MAX_REPORT_AGE = 90 * 24 * 60 * 60


def h(*parts: object) -> str:
    digest = hashlib.sha256()
    for part in parts:
        encoded = str(part).encode()
        digest.update(len(encoded).to_bytes(8, "little"))
        digest.update(encoded)
    return digest.hexdigest()


@dataclass(frozen=True)
class Window:
    start: int
    end: int
    state_epoch: int
    executions: int
    mismatches: int


def retention_checkpoint(windows: list[Window]) -> str:
    if len(windows) < 2:
        raise ValueError("insufficient windows")
    for previous, current in zip(windows, windows[1:]):
        if current.start < previous.end or current.start - previous.end > MAX_GAP:
            raise ValueError("gap or overlap")
        if current.state_epoch < previous.state_epoch:
            raise ValueError("state epoch rollback")
    if windows[-1].end - windows[0].start < MIN_RETENTION:
        raise ValueError("retention horizon insufficient")
    return h("retention-v1", *(window for window in windows))


@dataclass(frozen=True)
class Drill:
    trigger_injected: int
    trigger_detected: int
    acceptance_disabled: int
    rollback_started: int
    predecessor_restored: int
    recovery_verified: int
    completed: int
    acceptance_events_after_trigger: int
    successful_recovery_requests: int
    failed_recovery_requests: int
    final_stage: str


def drill_passes(drill: Drill, now: int) -> bool:
    monotonic = (
        drill.trigger_injected <= drill.trigger_detected <= drill.acceptance_disabled
        <= drill.rollback_started <= drill.predecessor_restored
        <= drill.recovery_verified <= drill.completed
    )
    return (
        monotonic
        and drill.trigger_detected - drill.trigger_injected <= 30
        and drill.acceptance_disabled - drill.trigger_detected <= 30
        and drill.predecessor_restored - drill.rollback_started <= 120
        and drill.recovery_verified - drill.predecessor_restored <= 120
        and drill.acceptance_events_after_trigger == 0
        and drill.successful_recovery_requests >= MIN_RECOVERY_REQUESTS
        and drill.failed_recovery_requests == 0
        and drill.final_stage == "rolled_back"
        and now - drill.completed <= MAX_REPORT_AGE
    )


def release_quorum(authorities: list[str], threshold: int, release_hash: str) -> str:
    if threshold < 1 or len(authorities) < threshold or len(set(authorities)) != len(authorities):
        raise ValueError("invalid release quorum")
    return h("release-quorum-v1", release_hash, *sorted(authorities))


windows = [
    Window(10_000, 10_000 + MIN_RETENTION // 2, 3, 500, 0),
    Window(10_000 + MIN_RETENTION // 2, 10_000 + MIN_RETENTION, 4, 500, 0),
]
retention_hash = retention_checkpoint(windows)

try:
    retention_checkpoint([windows[0], replace(windows[1], start=windows[0].end + MAX_GAP + 1)])
except ValueError:
    retention_gap_rejected = True
else:
    retention_gap_rejected = False
assert retention_gap_rejected

try:
    retention_checkpoint([windows[0], replace(windows[1], state_epoch=2)])
except ValueError:
    retention_rollback_rejected = True
else:
    retention_rollback_rejected = False
assert retention_rollback_rejected

healthy_drill = Drill(1_000, 1_010, 1_020, 1_025, 1_060, 1_080, 1_090, 0, 16, 0, "rolled_back")
assert drill_passes(healthy_drill, 1_100)
assert not drill_passes(replace(healthy_drill, acceptance_events_after_trigger=1), 1_100)
assert not drill_passes(healthy_drill, healthy_drill.completed + MAX_REPORT_AGE + 1)

authorities = ["release-a", "release-b"]
release_hash = h("release-v1", retention_hash, h(healthy_drill))
quorum_hash = release_quorum(authorities, 2, release_hash)
try:
    release_quorum(["release-a", "release-a"], 2, release_hash)
except ValueError:
    duplicate_authority_rejected = True
else:
    duplicate_authority_rejected = False
assert duplicate_authority_rejected

result = {
    "protocol": 12,
    "retention_checkpoint_hash": retention_hash,
    "retention_gap_rejected": retention_gap_rejected,
    "retention_epoch_rollback_rejected": retention_rollback_rejected,
    "rollback_drill_passed": True,
    "continued_acceptance_rejected": True,
    "stale_drill_rejected": True,
    "release_quorum_hash": quorum_hash,
    "duplicate_release_authority_rejected": duplicate_authority_rejected,
    "acceptance_enabled": False,
}
print(json.dumps(result, sort_keys=True))
