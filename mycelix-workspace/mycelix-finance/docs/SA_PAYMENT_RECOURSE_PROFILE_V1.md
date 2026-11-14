# South African Payment Recourse Profile v1

**Status:** synthetic reference and qualification contract; not legal advice, a consumer guarantee, or a provider integration.

## Purpose

This profile separates the existence of a payment from the availability and result of a remedy. It makes participant-specific recovery procedures, institutional authority, evidence, time windows, fees and actual returned value explicit. It composes with the South African payments profile and inclusion/outcomes benchmark without pretending that one generic `reversible` flag describes every instrument.

## Fundamental distinction

`recall requested != acknowledged != funds located != funds held != return authorised != funds returned`

Only an independently evidenced and reconciled return event increases the returned amount. A hold, trace, request, best-efforts response, voluntary promise or provider callback is not a return.

## Source-specific findings

The 4 November 2026 source snapshot records:
- FNB says completed PayShap payments cannot be reversed; assistance may be best-efforts without a guarantee.
- Nedbank describes a best-efforts recovery path for its mistaken-payment scenario, requiring the recipient to voluntarily return funds.
- CPMI's February 2026 FPS interlinking report recommends inter-institution recovery paths, clear responsibilities, and harmonised dispute mechanisms that can adapt to local rules.
- SARB's PSMB transition distinguishes regulator/authorisation/standards responsibilities from clearing-house operations.

These sources do **not** establish a universal rule for all PayShap participants, all legal claims, all fraud types, or other payment instruments. Profile effective dates are explicitly unknown where the public product source does not state them.

## Remedy taxonomy

Keep these distinct: mistaken beneficiary, authorised push-payment scam, unauthorised account takeover, duplicate debit, incorrect amount, timeout with possible debit, merchant non-delivery, debit-order/mandate dispute, card chargeback, internal ledger refund and cross-border trace/recovery.

Never import chargeback, debit-order reversal or wallet refund semantics into a PayShap credit-transfer profile without exact source authority.

## Data contract and state machine

Each profile binds jurisdiction, instrument, rail, participant, product, channel, observation date, effective-date status, remedy type, availability, responsible authority, time-window knowledge, source references and notes.

Each synthetic case records:
- claimed amount and currency;
- source profile;
- append-only, sequenced events;
- actor and authority state;
- evidence types;
- amount actually returned and reconciliation state;
- case disposition and expected qualification decision.

A return event must have a positive amount, valid actor authority, an independent return receipt, an independent ledger reconciliation and a unique return reference. Returns cannot exceed the claim amount. Later returned funds are a successor event; the original payment is not rewritten.

## Benchmark gates

- 0 requests or acknowledgements promoted to returned value.
- 0 funds-located or funds-held events promoted to returned value.
- 0 unverified, unreconciled or duplicate returns counted.
- 0 return amounts greater than the original claim.
- 0 cases closed without an explicit disposition.
- 0 unsupported participant rules inherited from a rail.
- 100% profile/source references resolved; dates and unknown windows preserved.
- 100% deterministic replay of frozen evidence and case events.
- 100% returned-value accounting in integer minor units.

Operational comparisons should measure report-to-acknowledgement time, time to first substantive response, time to disposition, amount-weighted recovery among eligible cases, residual loss, fees, customer effort, appeal/escalation rates and subgroup outcomes. Keep unresolved cases in the denominator. Do not collapse all measures into one score.

## Industry comparison

Initial comparison is synthetic and effects-disabled. A later authorised study can compare matched case sets across incumbent bank procedures, conventional case-management tools and a Mycelix evidence/coordination workflow.

The product goal is not to compel a bank or recipient to return funds. It is to reduce lost cases, duplicated evidence requests, misrouted remedies, false promises and time-to-correct-disposition while preserving authority and audit trails.

## Claim ceiling

This corpus proves only that the stated schema and reference evaluator can reject seeded semantic errors. It establishes no live recovery capability, legal entitlement, provider guarantee, customer savings or real-money safety. Real cases require current terms, source evidence, the responsible institution and qualified legal advice.
