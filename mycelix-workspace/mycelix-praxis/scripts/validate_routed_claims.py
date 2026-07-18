#!/usr/bin/env python3
"""Fail closed when routed trust-sensitive concepts drift into product claims."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/leptos/src/app.rs"
VERIFY = ROOT / "apps/leptos/src/pages/verify.rs"
EMPLOYER = ROOT / "apps/leptos/src/pages/employer.rs"
GOVERNANCE = ROOT / "apps/leptos/src/pages/governance.rs"
CAREERS = ROOT / "apps/leptos/src/pages/careers.rs"
DASHBOARD = ROOT / "apps/leptos/src/pages/dashboard.rs"
LEDGER = ROOT / "apps/leptos/src/ledger.rs"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def main() -> int:
    errors: list[str] = []
    app = read(APP)
    verification = read(VERIFY)
    employer = read(EMPLOYER)
    governance = read(GOVERNANCE)
    careers = read(CAREERS)
    dashboard = read(DASHBOARD)
    ledger = read(LEDGER)

    for route in (
        'path!("/verify")',
        'path!("/verify/:id")',
        'path!("/employer")',
        'path!("/governance")',
        'path!("/careers")',
    ):
        if route not in app:
            errors.append(f"expected routed trust-sensitive surface {route!r}")

    verification_required = {
        "Verification is unavailable in this build and fails closed.",
        "This reference has not been verified.",
        "No signature, expiry, ownership, or revocation check has been run.",
        "Proof fields, a Holochain action hash, or standards-compatible formatting are not by themselves verification.",
    }
    verification_forbidden = {
        "hash.len() >= 8",
        "set_verification_result.set(Some(true))",
        "Credential Verified",
        "ACTIVE & VERIFIED",
        "Holochain Verifiable",
        "W3C DID Compliant",
        "Verified Global Alignments",
        "spawn_local",
        "gloo_timers",
    }
    for phrase in sorted(verification_required):
        if phrase not in verification:
            errors.append(f"verification routes do not disclose {phrase!r}")
    for phrase in sorted(verification_forbidden):
        if phrase in verification:
            errors.append(f"verification routes still contain fabricated behavior {phrase!r}")

    employer_required = {
        "mode.get() == AppMode::Demo",
        "Demo-only talent concept",
        "Every candidate, identifier, score, skill, and proof badge below is fictional.",
        "No employer-search coordinator contract is connected.",
        "Concept badge — no ZK proof checked",
    }
    employer_forbidden = {
        "did:mycelix:",
        "ZK-Mastery Verified",
        "2 found in range",
        "Searching Local Swarm Mesh",
        "spawn_local",
        "gloo_timers",
    }
    for phrase in sorted(employer_required):
        if phrase not in employer:
            errors.append(f"employer concept does not enforce {phrase!r}")
    for phrase in sorted(employer_forbidden):
        if phrase in employer:
            errors.append(f"employer concept still claims {phrase!r}")
    if employer.count("disabled") < 6:
        errors.append("employer Demo concept does not disable every illustrative control")

    governance_required = {
        "mode.get() == AppMode::Demo",
        "Demo-only governance concept",
        "Every control is disabled.",
        "No proposals, votes, stakes, or payouts are claimed.",
        "No influence, payout, or reputation change is calculated.",
    }
    governance_forbidden = {
        "Quadratic Staking Active",
        "Active Proposals",
        "Commit Vote",
        "Submit Prediction",
        "High-conviction minority payouts active",
        "set_voting_active",
    }
    for phrase in sorted(governance_required):
        if phrase not in governance:
            errors.append(f"governance concept does not enforce {phrase!r}")
    for phrase in sorted(governance_forbidden):
        if phrase in governance:
            errors.append(f"governance concept still claims {phrase!r}")
    if governance.count("disabled") < 6:
        errors.append("governance Demo concept does not disable every illustrative control")

    careers_required = {
        "Local planning data — not a credential",
        "Nothing on this page establishes certification or eligibility.",
        "LOCAL DRAFT PROFESSIONAL SUMMARY",
        "This draft is not a credential, certification, transcript, CLR document, DID claim, or verification result.",
        "Mapped prerequisites recorded locally",
        "Transcript unavailable",
    }
    careers_forbidden = {
        "VERIFIABLE SOVEREIGN PROFESSIONAL SUMMARY",
        "CLR 2.0 Compliant",
        "did:mycelix:praxis-alpha-student",
        "Local contribution verified by community consensus",
        "VERIFICATION GATEWAY",
        '"Certified"',
        '"Ready to Certify"',
        "[Hash:",
        "Request Legacy Transcript",
    }
    for phrase in sorted(careers_required):
        if phrase not in careers:
            errors.append(f"career planner does not disclose {phrase!r}")
    for phrase in sorted(careers_forbidden):
        if phrase in careers:
            errors.append(f"career planner still claims {phrase!r}")
    if "disabled title=\"No transcript issuer is connected\"" not in careers:
        errors.append("career planner transcript control is not disabled")

    dashboard_required = {
        "mode.get() != AppMode::Demo",
        "Demo-only concept simulation",
        "Values and statuses are fictional examples.",
        "Topics with browser-local progress",
        "Demo only — no balance, credit, grant, or verification",
        "They have not been submitted to or validated by a conductor and create no TEND entitlement.",
        "Claim unavailable",
    }
    dashboard_forbidden = {
        "Verified via local Swarm Mesh",
        "Connect & Claim TEND",
        "TEND verified and claimed",
        "mints verified TEND",
        "GoSovereignPrompt",
        "on_connect_claim",
        "validate_pwa_import",
    }
    for phrase in sorted(dashboard_required):
        if phrase not in dashboard:
            errors.append(f"dashboard does not enforce {phrase!r}")
    for phrase in sorted(dashboard_forbidden):
        if phrase in dashboard:
            errors.append(f"dashboard still claims or executes {phrase!r}")

    try:
        demo_gate = dashboard.index("mode.get() != AppMode::Demo")
        demo_disclosure = dashboard.index("Demo-only concept simulation", demo_gate)
        ledger_mount = dashboard.index("<StewardshipLedger />", demo_disclosure)
        tend_mount = dashboard.index("<PendingTendCard />", demo_disclosure)
    except ValueError:
        errors.append("dashboard Demo gate, disclosure, or concept mounts are missing")
    else:
        if not demo_gate < demo_disclosure < ledger_mount < tend_mount:
            errors.append("dashboard economic concepts are not below the Demo disclosure")

    ledger_required = {
        "Demo-only fictional ledger",
        "All balances, reputation scores, merchants, projects, payouts, compliance states, infrastructure, and network statuses below are illustrative.",
        "No account, payment rail, registry, device, or conductor is connected.",
        "Example TEND Credits",
    }
    for phrase in sorted(ledger_required):
        if phrase not in ledger:
            errors.append(f"fictional ledger does not disclose {phrase!r}")
    if ledger.count("<button") != ledger.count("disabled title=\"Illustrative control only\""):
        errors.append("fictional ledger contains an enabled or unlabeled control")

    if errors:
        print("Routed claim validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(
        "Routed claims valid: verification fails closed; employer and governance "
        "concepts are Demo-only; careers export is a local draft; dashboard "
        "economics remain disclosed, disabled fiction."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
