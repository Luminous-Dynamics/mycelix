# AC-049 — Finalization Certificate Ledger

## Purpose

AC-049 turns a fresh, clean finalization assessment into a durable terminal
record without mutating or deleting the underlying evidence.

The certificate retains the exact lifecycle revision, active scope fingerprint,
and AC-048 evidence snapshot fingerprint that justified closure.

## Terminal semantics

The reference ledger is single-shot for one action:

- no certificate can be issued from a stale assessment;
- no certificate can be issued from a non-Ready assessment;
- once a certificate exists, a second terminal certificate is rejected;
- the ledger remains append-only and retains the original certificate;
- verification re-runs the finalization gate against current evidence.

This is intentionally modeled as a reference close-out policy, not as a claim
that every jurisdiction or procurement regime permits only one administrative
certificate.

## Verification

A certificate is not trusted solely because its fields look valid.

Verification checks that:

- the certificate is actually recorded in the ledger;
- its action identity matches the current lifecycle;
- the lifecycle revision and scope binding still match;
- the stored evidence snapshot fingerprint equals the freshly recomputed
  AC-048 snapshot;
- the current finalization assessment is still Ready.

Thus the certificate can become stale as a current truth if relevant evidence
changes. The historical certificate itself is never rewritten.

## Evidence chain

AC-035 binds action lifecycle history.

AC-036 fingerprints economic action scope.

AC-039 binds execution receipts to lifecycle authorization.

AC-041 reconciles execution against explicit constraints.

AC-045 establishes clean finalization eligibility.

AC-046 fingerprints and semantically recomputes reconciliation evidence.

AC-047 makes impact integrity action-scoped.

AC-048 binds finalization to a deterministic evidence snapshot.

AC-049 records that fresh result as a durable terminal certificate.

The resulting chain is:

**authorization → execution → reconciliation → integrity → fresh assessment → terminal certificate**

## Security boundary

AC-049 is not a signature scheme, identity proof, or governance authorization
protocol by itself. The authority reference is explicit evidence, while actual
authorization and cryptographic signing remain integration responsibilities.

The design follows the same broad replay-resistance principle used in modern
digital-identity guidance: a previously valid assertion should not silently
become a valid assertion for a materially different transaction state. NIST
describes freshness/timeliness data such as nonces as mechanisms for preventing
replay. AC-048/049 apply a ledger-state analogue without introducing arbitrary
timeouts.

## Non-goals

AC-049 does not:

- define legal finalization procedures;
- assign a universal expiration period;
- erase failed execution or reconciliation evidence;
- make a hash equivalent to a signature;
- assert that measurements are truthful merely because they are fingerprinted.

## Tests

The reference tests cover:

- fresh Ready assessment issuing exactly one terminal certificate;
- stale assessment rejection;
- second-certificate rejection;
- verification after issuance;
- verification failure after current action evidence changes;
- unrecorded certificate rejection.

