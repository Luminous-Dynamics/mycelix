# FIN-ECO-006 v1 — Evidence-bound accounting journal reference kernel

**Owner:** [FIN-ECO-006 / #1123](https://github.com/Luminous-Dynamics/mycelix/issues/1123)  
**Exact arithmetic dependency:** FIN-ECO-001 / #919  
**Qualified source dependency:** FIN-ECO-002 settlement-finality / #929  
**Claim ceiling:** deterministic, in-memory, single-asset-unit accounting projection reference. It is not a production ledger, complete book, audit opinion, GAAP/IFRS implementation, tax engine, valuation service, FX oracle or action-authority system.

## Authority direction

The kernel deliberately enforces a one-way boundary:

`qualified economic fact -> pinned accounting projection policy -> balanced journal entry`

A balanced entry does not prove that the economic action was authorized, that the provider observation was true, or that settlement is final. The first source adapter accepts only the constructor-controlled `QualifiedSettlement` from FIN-ECO-002. Raw provider acknowledgements, agent claims and free-form event tags cannot be submitted as qualified sources.

Accounting projection never authorizes a payment, transfer, reservation or capacity change.

## Versioned profile model

`AccountingLedgerProfile` commits:
- ledger ID/revision, reporting entity and book domain;
- asset-unit profiles, including exact `AssetId`, atomic decimal scale, issuer, network and fungibility domain;
- account taxonomy revisions, reporting entity, normal balance and permitted unit profiles.

`AccountingProjectionPolicy` commits:
- exact ledger profile digest;
- allowlisted settlement finality profile ID/revision/digest;
- semantic posting rules binding each posting role to one account and normal debit/credit side;
- whether the semantic role may be reversed, and posting-count bounds.

The account rule prevents a caller from producing numerically balanced but semantically arbitrary postings. The rule's normal side must agree with the account's profile. Reversal postings must use the opposite side and an explicitly reversal-enabled semantic rule.

## Exact journal construction

`qualify_journal_entry`:
1. validates the ledger profile and policy;
2. binds their canonical SHA-256 commitments;
3. verifies the source's exact settlement-profile revision against the policy allowlist;
4. checks reporting entity, period, effective time, account revision and asset-unit allowlists;
5. checks semantic role -> account -> debit/credit side against the pinned posting rule;
6. rejects zero, cross-asset, mixed unit-profile, duplicate semantic, over-bound or malformed postings;
7. sums debit and credit atomic units with checked integer arithmetic and requires exact equality;
8. requires both totals to equal the qualified source amount;
9. canonicalizes posting order and emits immutable bytes binding source event, evidence frontier, entity, period, time, policy/profile commitments and all postings.

No floating-point arithmetic or implicit FX conversion exists. v1 intentionally supports one asset-unit profile per entry. Cross-asset settlement and FX require a separately qualified conversion profile, not a numeric equality across different units.

## Append-only book semantics

`JournalBook` is a pure in-memory reference book:
- appends constructor-controlled qualified entries only;
- uses a source-application key to reject a conflicting second projection of the same source event;
- treats an identical re-submission as an idempotent replay;
- permits explicit compensating entries only when the policy allows them and the original qualified settlement profile says `MayReverse`;
- requires a reversal to target an existing original (not another reversal), the same ledger/entity and a non-earlier effective time;
- matches account, unit profile, opposite side and semantic role, and prevents cumulative partial reversals from exceeding the original posting.

The original entry is never modified. A reversal is a new economic-source-bound journal entry. A profile declaring terminal settlement is not silently made reversible by accounting policy.

This reference book is not a durable event store. Database transactions, concurrent source uniqueness, multi-process idempotency, checkpoint recovery, period closing and reconciliation are separate work.

## Current source adapter and tests

The v1 source adapter binds a FIN-ECO-002 `QualifiedSettlement` to:
- exact source-event ID;
- settlement subject and execution attempt;
- rail/network and finality profile ID/revision/digest;
- exact asset amount;
- evidence-frontier digest and qualification time;
- reversal model and all qualified operation/evidence identifiers.

The test fixture exercises exact qualified settlement projection, debit/credit mismatch, source amount mismatch, canonical posting permutation, semantic tampering, reporting-entity and account revision mismatch, invalid unit profile, cross-asset postings, zero/duplicate lines, unsupported settlement-profile revisions, idempotency/conflicting re-projection, partial reversal and over-reversal, missing reversal target, policy/ledger digest mismatch, profile commitment mutation, semantic posting-rule mismatch, wrong normal side, disabled reversal semantics, source terminality and reversal event-time ordering.

The exact-head workflow runs locked `cargo fmt`, `cargo check`, all crate tests and warnings-denied Clippy with preflight/postflight subject SHA and clean-tree assertions.

## Deliberate non-goals / next proofs

Not implemented or claimed:
- GAAP, IFRS, IPSAS or jurisdiction-specific statutory compliance;
- chart-of-accounts correctness, materiality, accrual recognition, tax treatment or audit opinion;
- asset ownership, fair value, price discovery, impairment, FX conversion or solvency;
- complete source acquisition, omitted-event detection, period completeness or close;
- durable persistence, multi-writer serializable append, distributed idempotency or recovery;
- automatic posting for encumbrance reservations or capacity holds;
- authorization of any economic action.

The next stage should be an independent verifier that reconstructs canonical bytes, verifies each source receipt against its owning FIN-ECO profile, checks source uniqueness across a durable append-only store, and produces separate completeness/reconciliation reports. Only then should real budget, transfer or economic workflows consume this journal projection.
