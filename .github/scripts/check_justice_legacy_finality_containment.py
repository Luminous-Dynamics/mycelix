#!/usr/bin/env python3
"""Fail closed if legacy Justice finality/disposition fields regain authority."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
INTEGRITY = ROOT / "mycelix-workspace/mycelix-civic/zomes/justice-arbitration/integrity/src/lib.rs"
COORDINATOR = ROOT / "mycelix-workspace/mycelix-civic/zomes/justice-arbitration/coordinator/src/lib.rs"

integrity = INTEGRITY.read_text()
coordinator = COORDINATOR.read_text()
integrity_prod = integrity.split("#[cfg(test)]", 1)[0]
coordinator_prod = coordinator.split("#[cfg(test)]", 1)[0]


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing containment invariant: {label}")


def require_count(text: str, needle: str, expected: int, label: str) -> None:
    actual = text.count(needle)
    if actual != expected:
        raise SystemExit(
            f"containment invariant changed: {label}: expected {expected}, found {actual}"
        )


def function_body(text: str, function_name: str) -> str:
    pattern = re.compile(
        rf"pub fn {re.escape(function_name)}\b.*?\n\}}\n(?=#\[derive|///|#\[hdk_extern\]|$)",
        re.S,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"could not isolate function: {function_name}")
    return match.group(0)


# Creation paths normalize caller-controlled legacy projection fields.
require(
    coordinator_prod,
    "pub fn render_decision(mut decision: Decision)",
    "render_decision owns normalization",
)
require(
    coordinator_prod,
    "decision.finalized = false;",
    "Decision.finalized normalized false before commit",
)
require(
    coordinator_prod,
    "appeal.status = AppealStatus::Filed;",
    "Appeal.status normalized to Filed before commit",
)

# Production coordinator code may not consult these legacy fields for positive truth.
require_count(
    coordinator_prod,
    "decision.finalized",
    1,
    "coordinator Decision.finalized is normalization-only",
)
require_count(
    coordinator_prod,
    "appeal.status",
    1,
    "coordinator Appeal.status is normalization-only",
)
for forbidden in (
    "AppealStatus::UnderReview",
    "AppealStatus::Granted",
    "AppealStatus::Denied",
    "AppealStatus::Remanded",
    "AppealStatus::Resolved",
):
    if forbidden in coordinator_prod:
        raise SystemExit(
            f"legacy appellate disposition regained coordinator authority: {forbidden}"
        )

# Integrity accepts only the inert legacy representation.
require(
    integrity_prod,
    "if decision.finalized {",
    "integrity rejects Decision.finalized=true",
)
require(
    integrity_prod,
    "Decision.finalized must be false; legacy finality is projection-only",
    "Decision finalized rejection reason",
)
require(
    integrity_prod,
    "if appeal.status != AppealStatus::Filed {",
    "integrity rejects non-Filed Appeal.status",
)
require(
    integrity_prod,
    "Appeal.status must be Filed; appellate disposition requires a separate authority-qualified record",
    "Appeal status rejection reason",
)
require_count(
    integrity_prod,
    "decision.finalized",
    1,
    "integrity Decision.finalized is rejection-only",
)
require_count(
    integrity_prod,
    "appeal.status",
    1,
    "integrity Appeal.status is rejection-only",
)

# DHT validation, not just coordinator convention, makes legacy records append-only.
require(
    integrity_prod,
    "Decision and Appeal legacy records are append-only; updates are forbidden",
    "RegisterUpdate rejects legacy Decision/Appeal",
)
require(
    integrity_prod,
    "Decision and Appeal legacy records are append-only; deletes are forbidden",
    "RegisterDelete rejects legacy Decision/Appeal",
)
require(
    integrity_prod,
    "Decision entries are append-only; direct updates cannot establish finality",
    "StoreEntry UpdateEntry rejects Decision",
)
require(
    integrity_prod,
    "Appeal filings are append-only; disposition requires a separate authority-qualified record",
    "StoreEntry UpdateEntry rejects Appeal",
)

# Public legacy mutation APIs remain present only as fail-closed compatibility surfaces.
for function_name in ("update_appeal_status", "finalize_decision"):
    body = function_body(coordinator_prod, function_name)
    require(body, "Err(wasm_error!", f"{function_name} fails closed")
    if "update_entry(" in body or "create_entry(" in body:
        raise SystemExit(f"{function_name} regained a DHT write path")

print("justice legacy finality containment contract: PASS")
