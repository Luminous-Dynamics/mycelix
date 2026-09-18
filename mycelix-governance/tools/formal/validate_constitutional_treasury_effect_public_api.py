#!/usr/bin/env python3
"""Static public-surface audit for MYC-CONST-003D1D-F0.

This companion validator proves that the raw transition kernel is not the crate
entrypoint and that external consumers must pass through the guarded facade.
It complements, rather than replaces, the source/profile validator.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CRATE = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider"
MANIFEST = CRATE / "Cargo.toml"
FACADE = CRATE / "src/public.rs"
KERNEL = CRATE / "src/lib.rs"
HISTORY_TEST = CRATE / "tests/reconciliation_history.rs"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    manifest = MANIFEST.read_text()
    facade = FACADE.read_text()
    kernel = KERNEL.read_text()
    history_test = HISTORY_TEST.read_text()

    require('[lib]\npath = "src/public.rs"' in manifest, "crate must enter through guarded public.rs")
    require('mod kernel {\n    include!("lib.rs");\n}' in facade, "raw kernel must remain a private module")
    require("pub mod kernel" not in facade, "kernel module must not be public")

    require("pub struct EffectRecord {\n    inner: kernel::EffectRecord," in facade, "EffectRecord must wrap private kernel state")
    require("pub struct TreasuryEffectRegistry {\n    inner: kernel::TreasuryEffectRegistry," in facade, "registry must wrap private kernel state")
    require("pub fn record_mut" not in facade, "public mutable raw-record escape hatch is forbidden")
    require("pub inner:" not in facade, "facade inner kernel state must remain private")

    require("fn historical_contradiction(" in facade, "history contradiction guard missing")
    require(
        "SuccessContradictsNoEffect" in facade and "NoEffectContradictsSuccess" in facade,
        "both reconciliation contradiction directions must be represented",
    )
    require(
        "prior.covers_through_attempt_ordinal\n                        >= observation.covers_through_attempt_ordinal" in facade,
        "late success must conflict with prior no-effect evidence covering that attempt horizon",
    )
    require(
        "prior.covers_through_attempt_ordinal\n                        <= observation.covers_through_attempt_ordinal" in facade,
        "later no-effect evidence must conflict with already observed success in its horizon",
    )

    require("apply_observation_guarded" in facade, "facade observation path must be guarded")
    require(
        "let record = self.inner.record_mut(execution_id)?;\n        apply_observation_guarded(record, observation)" in facade,
        "registry observation application must route through guarded history checks",
    )

    require(
        "late_success_cannot_overwrite_prior_no_effect_reconciliation" in history_test,
        "external adversarial reconciliation-history test missing",
    )
    require(
        "ObservationDecision::IntegrityConflict" in history_test
        and "EffectState::IntegrityHalted" in history_test,
        "late contradictory receipt must be required to halt",
    )

    # Positive control: the private kernel still contains the original focused
    # transition model and can retain its unit corpus; the facade is a guard, not
    # a second independent effect model.
    require("pub struct EffectRecord" in kernel, "private kernel EffectRecord unexpectedly missing")
    require("pub struct TreasuryEffectRegistry" in kernel, "private kernel registry unexpectedly missing")

    print("MYC-CONST-003D1D-F0 public facade validator: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
