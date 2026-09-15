# ECON-004 — Exact TEND Reciprocity Kernel

**Status:** Draft policy model. Canonical TEND runtime zomes are not modified by this tranche.

## Purpose

ECON-004 makes TEND's accounting semantics exact before runtime migration:

- TEND is signed zero-sum mutual credit;
- `1 TEND = 1 hour`;
- ledger arithmetic uses integer minutes, not floating-point hours;
- credit expansion is bounded by the declared canonical ±120 TEND emergency ceiling;
- care and gifts do not create recipient TEND debt;
- canonical TEND has no demurrage;
- TEND is not automatically cash-convertible to SAP.

## Exact unit

The protocol-facing value remains one TEND per hour. Internally, this policy uses:

`1 TEND = 60 TEND-minutes`

This permits exact 5-, 15-, 30-, or 45-minute exchanges without rounding a fractional service into a whole TEND or losing it entirely.

## Conservation

A reciprocity exchange of `m` minutes produces exactly:

- provider: `+m`
- receiver: `-m`

The bilateral total before and after must be identical. No pre-mining or unilateral positive-balance creation is represented by the kernel.

## Credit bounds

The canonical tiers remain:

- Normal ±40 TEND;
- Elevated ±60 TEND;
- High ±80 TEND;
- Emergency ±120 TEND.

The policy rejects a requested limit above ±120 TEND. Current runtime behavior that can double a tier during hibernation is therefore migration debt rather than part of the canonical target.

## Relationship integrity

Only a voluntary reciprocity relationship creates TEND liability. Care and gifts do not.

This makes the rule explicit:

**need is not debt.**

Care Commons may compensate caregivers or providers through a separately governed funding mechanism without posting a negative TEND balance to the person who needed care.

## No demurrage / no automatic cash-out

Canonical TEND uses bounded credit, reciprocity, clearing, and conservation rather than timed decay. The ECON-002 firewall therefore classifies TEND demurrage as forbidden.

Likewise, an individual's TEND balance is not automatically exchangeable for SAP. Federation settlement, if used later, must be represented as an explicit institutional clearing claim/contract rather than a hidden user-level TEND→SAP exchange rate.

## Current runtime migration observations

The present TEND coordinator still contains floating-point hour fields, rounded balance updates in some paths, an adaptive-demurrage API, and a hibernation path that can expand limits beyond the canonical 120-TEND tier ceiling. ECON-004 does not modify those runtime bytes yet; it defines the exact migration target first.

## Qualification target

Target commands:

- `cargo fmt --manifest-path mycelix-finance/tend-policy/Cargo.toml -- --check`
- `cargo test --manifest-path mycelix-finance/tend-policy/Cargo.toml --locked`
- `cargo clippy --manifest-path mycelix-finance/tend-policy/Cargo.toml --all-targets --locked -- -D warnings`

Tests cover minute precision, exact zero-sum conservation, bounded credit, non-debt care/gift semantics, no TEND demurrage, and no automatic TEND→SAP cash-out.

## Non-goals

This tranche does not yet define federation netting prices, replace the Holochain TEND schema, migrate historical balances, decide dispute policy, or grant any new authority.
