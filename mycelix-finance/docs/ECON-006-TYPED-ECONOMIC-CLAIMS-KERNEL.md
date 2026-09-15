# ECON-006 — Typed Economic Claims Kernel

**Status:** Draft policy model. No existing Finance ledger or contract runtime is modified by this tranche.

## Purpose

ECON-006 supplies the expressiveness that prevents token proliferation.

Equity, debt, bonds, mortgages, insurance, escrow, revenue sharing, subscriptions, grants, pensions, infrastructure rights, capacity reservations, and stewardship can be represented as typed claims instead of new foundational currencies.

A claim records who issued it, who holds it, what it concerns, which rights/duties it conveys, how it may be transferred, what performance is owed, its evidence, and its lifecycle.

## Performance is separate from instrument identity

A claim may specify:

- SAP settlement;
- a TEND reciprocity obligation measured in canonical minutes;
- a non-monetary quantity;
- an external denomination;
- or no payment performance at all.

That settlement choice does not transform the claim itself into SAP, TEND, MYCEL, or another foundational currency.

External denominations remain external claim terms; declaring a ZAR-, USD-, or other externally denominated obligation does not create a new Mycelix currency.

## Rights and duties

The initial kernel can represent use, income, transfer, exclusion, and stewardship duties. It rejects contradictory transfer semantics and requires a stewardship-right claim to actually carry a stewardship duty.

This is deliberately a generic substrate. Later domain modules can place stricter rules around securities, insurance, housing, infrastructure, pensions, public concessions, or regulated credit.

## Lifecycle and contestability

Claims carry creation/expiry information and may become challenged or cancelled without erasing the historical record. A challenged claim is not treated as simply active.

## Constitutional boundary

A contractual/property claim may define economic rights according to its terms, but a claim cannot manufacture fundamental civic sovereignty or rank human worth.

Typed claims also do not bypass the ECON-002 conversion firewall. A TEND reciprocity obligation does not become SAP; a SAP-settled claim does not alter MYCEL.

## Why this matters for token minimization

The test corpus demonstrates that the major financial instruments above fit the same claim model while `creates_foundational_currency` remains false.

That makes the economic architecture:

**three foundational lanes + many typed claims**

instead of one token for each business or civic concept.

## Qualification target

Target commands:

- `cargo fmt --manifest-path mycelix-finance/claims-policy/Cargo.toml -- --check`
- `cargo test --manifest-path mycelix-finance/claims-policy/Cargo.toml --locked`
- `cargo clippy --manifest-path mycelix-finance/claims-policy/Cargo.toml --all-targets --locked -- -D warnings`

Tests cover major instrument representation, SAP/TEND lane isolation, external denomination isolation, stewardship duties, transfer contradictions, challenge/expiry state, civic-authority prohibition, and reuse of canonical SAP/TEND units.

## Non-goals

This tranche does not decide securities-law treatment, insurance regulation, enforceability in any jurisdiction, accounting standards, tax treatment, custody architecture, market pricing, or automated liquidation. Those require domain-specific policy and legal review.
