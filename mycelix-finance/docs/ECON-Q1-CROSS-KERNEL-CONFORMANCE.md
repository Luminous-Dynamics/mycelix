# ECON-Q1 — Cross-Kernel Economic Conformance Capsule

Status: qualification/conformance surface only

Parent subject: ECON-006

This capsule adds no new economic instrument, monetary parameter, governance power, or runtime mutation. It composes the ECON-002 through ECON-006 policy kernels and tests that their boundaries remain true when the kernels are used together.

## Proposition

If ECON-002..006 behave according to their public policy APIs, composition does not create a hidden path by which:

- SAP becomes TEND or MYCEL;
- TEND becomes SAP or MYCEL;
- MYCEL becomes money, economic privilege, or fundamental civic power;
- a financial claim becomes a new foundational currency;
- care or gifts become reciprocity debt;
- ordinary SAP checkout becomes issuance;
- external settlement silently becomes SAP issuance;
- a credential grants a role without an independent bounded review step.

The capsule therefore tests system-level boundary preservation rather than another layer of local rules.

## Cross-kernel adversarial chains

### Economic activity -> standing -> authority

The strongest adversarial chain in Q1 deliberately composes otherwise legitimate steps:

1. an ordinary SAP transfer is allowed inside the SAP lane;
2. the economic event may be submitted as evidence;
3. a valid domain MYCEL credential may exist;
4. that credential may support a bounded-role review;
5. the chain must still fail to produce automatic SAP fee privilege or fundamental civic authority.

This prevents authority laundering through individually valid abstractions.

### SAP -> service claim -> TEND

A SAP-denominated service claim is a contractual/service claim. It does not mutate the TEND ledger. If participants separately create a TEND reciprocity exchange, that is a distinct voluntary operation with its own zero-sum accounting.

### Typed financial claims -> currency proliferation

Equity, debt, bonds, mortgages, insurance, escrow, revenue shares, stewardship rights, subscriptions, grants, pensions, infrastructure rights, and capacity reservations are instantiated through the claims kernel. Each remains a typed claim and reports `creates_foundational_currency = false`.

## Exact invariants exercised

Q1 checks:

- all six direct cross-lane conversion directions fail closed;
- SAP checkout is transfer of existing SAP, not issuance;
- external-value SAP issuance requires explicit conversion intent;
- TEND sub-hour sequences preserve exact integer-minute zero-sum accounting;
- TEND emergency credit remains bounded at 120 TEND;
- canonical TEND demurrage remains forbidden;
- care and gifts create no TEND repayment liability;
- SAP/TEND economic events enter MYCEL only as reviewable evidence;
- matching MYCEL credentials support review but never self-grant roles;
- challenged, revoked, and expired credentials fail closed;
- valid MYCEL cannot automatically create SAP fee discounts, TEND credit expansion, or payouts;
- typed claim performance touches at most its declared lane;
- claim instruments do not mint new foundational currencies;
- SAP and TEND canonical units are reused rather than redefined;
- balances, credentials, and claims cannot create fundamental civic power;
- a composed SAP -> evidence -> MYCEL -> role path still terminates at independent review.

## Execution target

The capsule is dependency-light and uses only local path dependencies.

```bash
cargo fmt --manifest-path mycelix-finance/econ-conformance/Cargo.toml -- --check
cargo test --manifest-path mycelix-finance/econ-conformance/Cargo.toml --locked
cargo clippy --manifest-path mycelix-finance/econ-conformance/Cargo.toml --all-targets --locked -- -D warnings
```

## Evidence boundary

An executed PASS would establish only the cross-kernel propositions encoded by this capsule for the exact tested commits.

It would not establish:

- current Holochain runtime conformance;
- reserve adequacy or SAP price stability;
- real-world TEND adoption or credit safety;
- correctness of credential issuers or evidence;
- legal enforceability or regulatory classification of claims;
- fairness of fees, demurrage rates, credit limits, or allocation policy;
- resistance to every economic or governance attack.

Runtime migration remains a separate qualification problem.

## Freeze discipline

ECON-Q1 should remain a single atomic child commit over the frozen ECON-006 head. If a policy parent changes, Q1 must be regenerated against the new exact parent rather than silently mixing lineages.
