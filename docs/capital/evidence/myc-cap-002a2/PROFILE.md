# MYC-CAP-002A2 — Return Envelope financial checkpoint currentness v1

Status: executable candidate over exact Return Envelope subject; local preflight only until exact-head hosted qualification succeeds.

## Purpose

Separate the financial state reconstructed at one exact Return Envelope event-chain tip from proof that this exact checkpoint is the currently designated financial checkpoint.

```text
valid Return Envelope checkpoint
!= latest designated financial checkpoint

CLAIM_ACTIVE at supplied tip
!= CLAIM_ACTIVE now

financial currentness
!= payment authority
```

## Exact qualified input

Return Envelope subject:

`270b852e0ac744dfca3a2cb966bf53fce78f2ab9`

The subject is backed by successful MYC-CAP-002A1 portable exact-subject replay.

Canonical Return Envelope receipt semantic SHA-256:

`60b7217980c615eab453eae25b74b27e75fc67cb336ea60d140f089341d50565`

Financial profile SHA-256:

`6929b9e089e3f7b272713f6ff8587bf429594a51a9d67d67d0d4468ba72b52e7`

Event-history SHA-256:

`b1e75101137bcecc60daba64aa0d232398b78d65e4e946723bd25e5ab23184b7`

Event-chain tip SHA-256:

`f766650cbc8233f016ae3482f46708f395264cbc0baeb869ad7c09dd662d240d`

## Frozen currentness profile

Profile semantic SHA-256:

`f795dd36475028c44418e8bf45d2268dcb1cf459526992bb7d043b5377603383`

Designation semantic SHA-256:

`2b17df5581778c5c957ff561fbbe42b554505973a07e4025db5ad4c973d53761`

Registry:

`registry:return-envelope-currentness`

## States

`CURRENT | STALE | PENDING | REVOKED`

Precedence:

`REVOKED > PENDING > STALE > CURRENT`.

Closed currentness-event vocabulary:

`SupersedeCheckpoint | PendingReconciliation | MaterialInvalidation | RevokeEvidence`.

No local wall-clock time creates authority.

## Canonical checkpoint

The frozen example echoes, without upgrading:

```text
financial_state = CLAIM_ACTIVE
remaining_claim_units = 73_000_000
reserve_balance_units = 5_000_000
reserve_compliant = true
unit = ZAR-cent
```

These values describe the qualified checkpoint. They are not a live debt statement and do not authorize payment.

## Positive theorem

A positive result requires the exact qualified Return Envelope subject and canonical receipt/profile/history/tip plus an ACTIVE designation naming those exact commitments and no stronger invalidation event.

Then only:

`financial_checkpoint_currentness_state = CURRENT`.

Currentness is orthogonal to financial outcome. A future separately qualified checkpoint could legitimately be `RETURN_ENVELOPE_SATISFIED + CURRENT`.

## Canonical commitments

Case file SHA-256:

`979cde6c9e2282bf39dd518d3adb5218b427c0945fb6881c58f5b763e08ee9f3`

Receipt file SHA-256:

`3b6ca6a3f2bfc37c3ecf412724af9013e5b2cbff47bfdf14028d71047144c7f6`

Receipt semantic SHA-256:

`034c0dcd6c7168687a41b2f01d25f646fb6969d2d27296382486d71d200b461e`

Empty currentness-event history semantic SHA-256:

`4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945`

## Local preflight

The stdlib suite passes **31/31** locally, covering exact subject/receipt/profile/history/tip binding, unit/project substitution, stale/pending/revoked precedence, event-chain integrity, handback/title contamination, authority ceilings and deterministic replay.

## Authority boundary

Every receipt fixes:

```text
payment_authority_established = false
accounting_compliance_established = false
tax_compliance_established = false
securities_law_compliance_established = false
legal_title_transition_established = false
handover_accepted = false
uses_local_wall_clock = false
```

## H5 boundary

H5 must require both A2 `CURRENT` and C1 `CURRENT` for the exact financial/holder pair. A2 currentness alone does not establish current holder state or payment eligibility.

## Nonclaims

Even hosted PASS would not establish payment entitlement, accounting/tax/securities compliance, audited financial statements, solvency, valuation, legal title, handback, democratic legitimacy, external authority authenticity, or the currentness of any later financial checkpoint not designated by this exact lineage.
