# AC-046 — Execution Reconciliation Fingerprint Binding

## Purpose

AC-046 strengthens the execution-to-reconciliation boundary by making a
reconciliation record bind to the exact receipt and exact execution constraint
that were observed.

An execution ID or constraint ID is an identity handle. It is not, by itself,
proof that the referenced object's contents have remained unchanged.

AC-046 therefore records SHA-256 fingerprints for both artifacts and requires
the finalization gate to verify those fingerprints against the currently
supplied evidence.

## Invariants

### Exact receipt binding

Every persisted reconciliation contains an execution fingerprint derived from
the complete validated execution receipt.

Changing any receipt field changes its fingerprint under the reference
canonicalization scheme.

### Exact constraint binding

Every persisted reconciliation contains a constraint fingerprint derived from
the complete validated execution constraint.

Changing the expected kind, quantity, unit, authorization reference, evidence,
or other serialized field therefore changes the fingerprint.

### Authorization remains semantic

At finalization, the receipt must also match the required constraint's:

- action reference;
- lifecycle revision;
- scope ID;
- scope fingerprint.

Fingerprints do not replace these explicit semantic checks.

### Result is not trusted merely because it is hashed

A reconciliation can contain a valid pair of fingerprints and still carry a
false conformance result.

AC-046 therefore recomputes the exact reference conformance result from the
receipt and constraint and rejects a stored result that differs from that
calculation.

This prevents a forged `Conformant` label from becoming authoritative merely
because its surrounding hashes are internally consistent.

### Execution ledger structure

The execution ledger validates:

- non-empty action identity;
- valid receipt envelopes;
- unique execution IDs;
- receipt action identity matching the ledger.

This prevents ambiguous receipt lookup caused by malformed persisted history.

## Decision effects

The finalization assessment distinguishes:

- `BlockedByMissingExecutionEvidence` when a reconciliation references no
  corresponding execution receipt;
- `BlockedByEvidenceBinding` when a fingerprint or explicit receipt
  authorization binding disagrees;
- `BlockedBySemanticMismatch` when the stored conformance result differs from
  exact recomputation;
- `BlockedByNonConformance` when a correctly bound reconciliation proves an
  actual under/over/unit/kind/missing-quantity result.

The distinctions preserve diagnostic evidence instead of collapsing every failure
into a generic block.

## Canonicalization boundary

The reference implementation uses serde JSON serialization with an explicit
domain/version prefix before SHA-256.

This is deterministic for the same Rust/serde representation, but it should not
be described as a universal cross-language canonical JSON standard. A future
interoperability profile can replace or supplement this with an explicitly
specified canonical encoding without weakening the binding invariant.

## Relationship to AC-045

AC-045 established that finalization requires:

- a Completed lifecycle;
- explicit completion control;
- complete required reconciliation;
- clean integrity state;
- resolved impact exposure.

AC-046 makes the reconciliation evidence itself cryptographically and
semantically self-consistent at that boundary.

The resulting chain is:

**scope → lifecycle revision → execution receipt → execution constraint →
reconciliation → finalization assessment**

Each transition retains enough identity and provenance to detect substitution,
staleness, or result forgery.

## Non-goals

AC-046 does not:

- define procurement law;
- create universal execution tolerances;
- assign economic value to non-market substrate;
- prove that the impact-discovery process is omniscient;
- make a cryptographic hash equivalent to an external signature or governance
  authorization.

## Tests

The reference tests cover:

- receipt fingerprint sensitivity;
- constraint fingerprint sensitivity;
- malformed reconciliation fingerprint structure;
- malformed duplicate execution history;
- missing execution evidence at finalization;
- receipt fingerprint mismatch;
- constraint fingerprint mismatch;
- explicit receipt authorization mismatch;
- forged conformance result despite matching fingerprints.

