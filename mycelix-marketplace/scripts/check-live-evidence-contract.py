#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HELPER = (ROOT / "frontend-leptos/bridge/test/evidence.ts").read_text()
RUNNER = (ROOT / "scripts/run-live-promotion.sh").read_text()
VERIFY = (ROOT / "scripts/verify-live-evidence.py").read_text()
WORKFLOW = (ROOT / ".github/workflows/live-evidence.yml").read_text()
RUNTIME_TEST = (ROOT / "frontend-leptos/bridge/test/runtime-capabilities.test.ts").read_text()
VERIFIER_TEST = (ROOT / "scripts/test-live-evidence-verifier.py").read_text()
SCENARIOS = {
    name: (ROOT / f"frontend-leptos/bridge/test/conductor-{name}.ts").read_text()
    for name in ["lifecycle", "arbitration", "settlement", "network"]
}

errors: list[str] = []
for needle in [
    "fixture: false",
    "MARKETPLACE_SOURCE_REVISION",
    "MARKETPLACE_HAPP_SHA256",
    "MARKETPLACE_DNA_SHA256",
    "MARKETPLACE_CLIENT_VERSION",
    "requireActiveRoles",
]:
    if needle not in HELPER:
        errors.append(f"live evidence helper missing {needle}")
for name, text in SCENARIOS.items():
    multiline = f'emitLiveEvidence(\n      "{name}"'
    inline = f'emitLiveEvidence("{name}"'
    if multiline not in text and inline not in text:
        errors.append(f"{name} scenario does not emit a structured receipt")
if '[ROLE, "finance"]' not in SCENARIOS["settlement"]:
    errors.append("settlement scenario does not require an active Finance role")
for needle in [
    "fixture evidence cannot promote a release",
    "holochain_client_version",
    "marketplace_status_after_settlement",
    "keep dispute UI gated",
]:
    if needle not in VERIFY:
        errors.append(f"promotion verifier missing {needle}")
for needle in ["sha256sum", "test:conductor-lifecycle", "verify-live-evidence.py"]:
    if needle not in RUNNER:
        errors.append(f"live promotion runner missing {needle}")
for needle in ["CellType.Provisioned", "CellType.Cloned", "configuredRoles", "activeRoles"]:
    if needle not in RUNTIME_TEST:
        errors.append(f"runtime capability test missing {needle}")
for needle in ["promoted_with_limitations", "fixture evidence cannot promote"]:
    if needle not in VERIFIER_TEST:
        errors.append(f"evidence verifier test missing {needle}")
for needle in ["workflow_dispatch", "self-hosted", "run-live-promotion.sh"]:
    if needle not in WORKFLOW:
        errors.append(f"live evidence workflow missing {needle}")

for needle in [
    "partition_divergence_observed",
    "safe_terminal_dominance_observed",
    "unsafe_conflict_observed",
]:
    if needle not in SCENARIOS["network"]:
        errors.append(f"network scenario missing {needle}")
if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))
print("live evidence contract passed: receipts bind source, artifacts, roles, and scenarios")
