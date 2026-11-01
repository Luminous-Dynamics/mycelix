# GCPN Evidence Graph Lifecycle Engine v0.1 — Stage B

**Status:** Rust implementation authored; compile/test/CI and external cryptographic/CAP adapter checks remain outstanding.  
**Parent issue:** #5646.  
**Specification PR:** #5647.  
**Implementation package:** `benchmarks/gcpn-evidence-graph-lifecycle`.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented source core

The package contains a first typed in-memory evidence graph with:

- distinct `NodeKind` types for source observation, forecast, authority assertion, CAP message, delivery receipt, accessible rendering, comprehension evidence, action evidence, outcome evaluation, activity and agent;
- typed relations for derivation, support, contradiction, supersession, revocation, reference, delivery, rendering, assessment, action evidence and evaluation;
- relation endpoint checks and forbidden-cycle detection for derivation/supersession/revocation;
- exact duplicate node idempotency and preservation of conflicting node identity payloads;
- valid-time and transaction-time fields on records and edges;
- deterministic transaction-time `snapshot_as_of(T)` reconstruction;
- visibility of dangling references in historical snapshots when an edge arrived before its endpoint;
- informational forward-reference counts when an endpoint record arrives after its relation;
- interval, endpoint, relation-type and forbidden-cycle diagnostics.

This is a deliberately small in-memory engine, not yet a persistent event-log format or production provenance store.

## 2. Bitemporal semantics in code

Nodes carry:
- optional valid/event time start and end;
- transaction/knowledge `recorded_at`;
- optional source-declared creation time, retained separately;
- optional payload digest and explicit digest-verification status.

Edges carry:
- relation valid/effective time;
- transaction/knowledge `recorded_at`.

`snapshot_as_of(T)` includes only nodes and edges recorded by T. It retains the original valid/effective timestamps. Later-arriving records and revocations therefore appear in the current graph but not in earlier transaction-time snapshots.

The Rust tests include:
- a source created/effective at time 100 but recorded at 300 is absent from the time-200 snapshot;
- a revocation with effective time 150 but recorded at 300 is absent from the time-200 snapshot and visible in the time-400 snapshot with both clocks preserved.

The implementation currently uses synthetic integer ticks. A production adapter must parse and preserve full CAP/RFC3339 time semantics and trusted transaction-time provenance.

## 3. CAP lifecycle core

The CAP sidecar validator consumes already-parsed synthetic messages. It checks:
- stable extended identity `sender,identifier,sent`;
- exact duplicate idempotency;
- conflicting identity diagnostics and preservation of conflicting payload objects;
- required references for Update and Cancel;
- exact references to earlier messages in the append-ordered fixture;
- chronology and expiration of referenced messages.

Expiry is reported separately from core CAP structural conformance. The base evaluator checks that Update/Cancel references resolve to earlier messages; an optional stricter active-reference profile additionally requires the referenced message to remain unexpired. That stricter profile is an explicit GCPN policy choice, not a claim that CAP 1.2's core syntax imposes one universal expiry rule.
- externally supplied digest-verification state;
- receipts bound to exact extended message identity and exact declared digest.

It does not parse CAP XML, validate an XSD, interpret every CAP element, or calculate cryptographic digests. CAP message times are synthetic integer ticks. Those responsibilities must be implemented by explicit adapters and checked against the OASIS/ITU standard before any real interoperability claim.

CAP predecessor payload bytes are held immutably in each message object; adding an Update or Cancel does not mutate the predecessor.

## 4. Cryptographic boundary

The code deliberately does not implement SHA-256. `DigestStatus::VerifiedByExternalAdapter` means the caller supplied a verification result from a separate cryptographic implementation. `FixtureOracleAccepted` exists only for the frozen synthetic examples and tests; it is not a cryptographic result and must never be used for production acceptance.

A payload digest by itself does not prove authenticity, authority or physical truth. The future adapter must bind a declared algorithm to exact wire bytes, retain the original bytes and verify signatures/keys under an explicit policy. FNV-1a remains only a non-cryptographic fixture checksum.

## 5. Authored tests

Seventeen Rust tests are included:

1. historical as-of snapshot excludes late arrivals;
2. later-recorded revocation is not backdated into earlier knowledge;
3. exact duplicate node is idempotent and counted;
4. conflicting node identity is preserved and rejected;
5. dangling and wrongly typed relations are distinct diagnostics;
6. forbidden supersession cycle is detected;
7. CAP Update with exact prior reference is accepted;
8. Update without reference is rejected;
9. unknown, non-prior and expired references are distinguished;
10. conflicting CAP identity is reported and payload retained;
11. exact CAP duplicate is idempotent but visible;
12. receipt must match exact message digest;
13. unverified digest is not accepted as a graph pass;
14. invalid valid-time interval fails;
15. a forward reference is dangling in a historical snapshot and resolves when the late endpoint arrives;
16. a receipt cannot select one of multiple conflicting CAP payloads sharing an extended identity;
17. a receipt referencing an ambiguous CAP extended identity is rejected rather than bound to the last map entry.

These are authored source tests, not executed results.

## 6. Deliberate limitations and next steps

Not implemented in this increment:
- real CAP XML parsing, XSD/schema validation, full dateTime parsing and canonical wire-format adapter;
- SHA-256 calculation, detached-signature verification, credential/key revocation validation or trust-store policy;
- durable append-only event-log serialization, persistence and replay from disk;
- full graph schema/version migrations and actor/authority scope policy;
- explicit privacy projection/redaction engine;
- source-independence derivation across shared upstream providers;
- every mutation family and frozen JSON/CSV fixture from #5646;
- property-based fuzzing and independent graph-oracle implementation;
- operational or national interoperability testing.

Do not infer that a graph is trusted because `GraphReport::is_conformant` is true. That method only covers the implemented structural checks and digest states supplied by the caller.

## 7. Validation status

- Isolated Cargo manifest and Rust source are on the implementation branch.
- Source-level review checks the bitemporal and CAP invariants described above.
- No Cargo build, Rust test execution, CLI run, XML validation, cryptographic implementation check or CI pass is claimed because this authoring environment has no Cargo or rustc.
- No live system or national CAP profile was used.

## 8. Safety and non-claims

Synthetic civilian warning provenance and audit only. No live alert dispatch, operational warning policy, recipient tracking, emergency resource control, physical hazard verification, weapons, targeting or military command. Cryptographic integrity is not physical truth; provenance completeness is not legal authorization; graph conformance is not evidence of lives saved.
