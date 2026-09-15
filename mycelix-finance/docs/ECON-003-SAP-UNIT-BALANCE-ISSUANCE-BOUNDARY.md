# ECON-003 — SAP Unit / Balance / Issuance Boundary

**Status:** Draft policy model. No money-moving Finance zome is changed by this tranche.

## Purpose

ECON-003 separates four concepts that must not be collapsed:

1. **SAP denomination** — what one SAP means as an accounting unit;
2. **SAP balance treatment** — whether an existing balance is eligible for demurrage or an exemption;
3. **SAP issuance** — an explicit event that creates new SAP under a declared authority/evidence basis;
4. **SAP settlement against a typed claim** — transferring/redeeming SAP for a right, asset, or service claim without silently mutating TEND or MYCEL.

## Canonical denomination

The current wire/runtime scale is retained:

`1 SAP = 1,000,000 micro-SAP`

Demurrage is a treatment of eligible balances. It does not redefine the denomination. A contract denominated as 100 SAP remains denominated as 100 SAP regardless of balance decay elsewhere.

## Balance treatment

ECON-003 models three semantic classes:

- ordinary balances — may be demurrage eligible under ratified policy;
- commons reserves — not assumed demurrage eligible by this policy model;
- explicitly protected/exempt tranches — not assumed demurrage eligible while the exemption is valid.

This tranche does not set or change the actual demurrage rate, exemptions, or reserve policy.

## Issuance is not checkout

Ordinary checkout/payment is classified as movement of existing SAP. It is not a mint event.

Issuing new SAP requires a separate positive issuance request with:

- an explicit issuance basis;
- an authority reference;
- an evidence reference;
- and, for external-value settlement, explicit conversion intent separate from ordinary checkout.

The initial basis vocabulary mirrors current Finance concepts without claiming they are sufficient or correctly collateralized: verified collateral, ratified governance authorization, explicit external-value settlement, and bootstrap distribution.

Passing the policy validator does **not** prove signatures, collateral value, legal authority, reserve sufficiency, settlement finality, or oracle truth. Those remain adapter/evidence responsibilities.

## Claims are not currencies

SAP may settle against typed claims such as energy, agricultural goods, housing use, restoration obligations, or services. That creates a claim/right/obligation; it does not create another foundational currency.

In particular, a SAP-denominated service-hours claim is **not** SAP→TEND conversion. TEND balances remain untouched unless the parties separately enter a voluntary TEND reciprocity event.

Likewise, settlement against a claim does not automatically change MYCEL.

## Explicit non-claims

ECON-003 does not yet establish:

- SAP price stability;
- reserve adequacy or a reserve ratio;
- redemption liquidity;
- collateral valuation correctness;
- oracle robustness;
- legal redemption rights;
- bank-run resistance;
- runtime conformance of existing mint/bridge/redemption zomes.

These require separate stress tests and qualified runtime wiring.

## Qualification target

`mycelix-sap-policy` is an independent local policy workspace with a path dependency only on the qualified ECON-002 policy surface.

Target commands:

- `cargo fmt --manifest-path mycelix-finance/sap-policy/Cargo.toml -- --check`
- `cargo test --manifest-path mycelix-finance/sap-policy/Cargo.toml --locked`
- `cargo clippy --manifest-path mycelix-finance/sap-policy/Cargo.toml --all-targets --locked -- -D warnings`

Tests prove denomination invariance, balance/unit separation, explicit issuance envelopes, external-conversion intent separation, ordinary-checkout non-mint semantics, and typed-claim isolation from TEND/MYCEL.
