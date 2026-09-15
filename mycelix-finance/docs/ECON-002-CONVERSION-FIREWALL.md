# ECON-002 — Conversion Firewall

**Status:** Draft policy kernel; not yet connected to Finance runtime zomes.

## Purpose

ECON-002 turns ECON-001's lane separation into executable policy. The kernel classifies a proposed economic effect as `Allowed`, `RequiresIndependentReview`, or `Forbidden`.

## Encoded rules

Allowed directly:

- SAP movement within SAP;
- TEND mutual-credit movement within TEND;
- SAP demurrage under applicable SAP policy;
- same-lane normalization.

Forbidden:

- direct conversion between different SAP, TEND, and MYCEL lanes;
- direct transfer of MYCEL;
- TEND or MYCEL demurrage;
- automatic SAP activity to MYCEL standing;
- automatic TEND activity to MYCEL standing;
- automatic fundamental civic authority from any lane balance or score.

Requires independent review:

- using an economic event as evidence for a bounded domain credential;
- using MYCEL/domain evidence for bounded role eligibility.

The intended path is:

`economic event -> reviewable evidence -> independent qualification -> bounded credential`

not an automatic balance-to-standing conversion.

## Why a separate dependency-free crate

Current Finance still contains legacy cross-lane couplings. ECON-002 freezes and tests the target policy before later remediation changes touch large Holochain zomes. `mycelix-finance/policy` holds no ledger state and has no authority to move value or issue credentials.

## Qualification target

Run:

- `cargo fmt --manifest-path mycelix-finance/policy/Cargo.toml -- --check`
- `cargo test --manifest-path mycelix-finance/policy/Cargo.toml --locked`
- `cargo clippy --manifest-path mycelix-finance/policy/Cargo.toml --all-targets --locked -- -D warnings`

The tests cover all cross-lane pairs, MYCEL non-transferability, SAP-only demurrage, standing separation, civic-power separation, and independent-review requirements.

## Non-goals

This tranche does not change current fee tiers, staking, TEND quality behavior, Holochain zomes, parent Finance Cargo.lock, or current balances. It does not claim runtime conformance.
