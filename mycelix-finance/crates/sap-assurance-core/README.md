# SAP Assurance Core

A dependency-free Rust policy kernel for the proposed SAP-ZAR reserve perimeter.

## Scope

- Reserve asset eligibility is explicit and class-based.
- Haircuts and encumbrances use checked integer arithmetic.
- Productive assets, loan receivables, commercial-bank deposits and self-issued claims do not count as immediately liquid reserves in the default profile.
- Duplicate canonical asset IDs are rejected within a reserve snapshot.
- New issuance is checked against post-issuance liabilities at a 100% haircut-adjusted reserve floor.
- SAP outstanding supply is derived as total issued minus total burned.
- Idempotency binds a one-time key digest to an exact request digest.
- A redemption with unknown payout outcome cannot be resubmitted or burned until external reconciliation resolves it.

## Run

From mycelix-finance/: cargo test -p sap-assurance-core

## Explicit limitations

This is a pure reference kernel, not a Holochain zome integration, bank adapter, custody attestation, legal opinion or production reserve system. A caller must authenticate custody evidence, verify signatures and current authority, canonicalize/hash the exact request, and enforce these checks in the integrity/authoritative transaction path. The 90-day short-term sovereign maturity limit is a proposed configurable default, not a regulatory classification.

The tests include a deterministic one-million-case arithmetic/property loop. This code is not qualified until Cargo tests run on the exact PR head and results are reproduced.
