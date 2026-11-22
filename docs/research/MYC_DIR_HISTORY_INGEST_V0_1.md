# Mycelix Directory Fabric: authenticated history evidence ingest v0.1

**Profile:** MYC-DIR-HISTORY-INGEST-0.1  
**Status:** source-present reference adapter and tests; not compiled or runtime-qualified  
**Tracking:** [MYC-DIR-HISTORY-INGEST-001 / #6501](https://github.com/Luminous-Dynamics/mycelix/issues/6501)  
**Composes with:** [history-oracle PR #6500](https://github.com/Luminous-Dynamics/mycelix/pull/6500), [owner epoch/route admission PR #6492](https://github.com/Luminous-Dynamics/mycelix/pull/6492)

## 1. The trust boundary

The history oracle must not accept caller assertions that a bundle or owner stream is complete. A partial set of individually valid records can still omit the one effect that disproves a safety claim.

The first oracle slice exposed `HistoryEvidence.event_set_complete` and `complete_owner_streams` as fields that the caller had to set. That made them explicit trust-boundary assertions. This increment closes that API bypass: those fields are now `pub(crate)`, and external callers cannot construct a `HistoryEvidence` directly. The only public construction path is `ingest_history(RawHistoryBundle, EvidenceAuthenticator)`.

The ingest verdict is three-valued:

- **READY:** source manifests, exact event coverage, owner streams, inventory closure and any global owner-ack set have all passed independent verification. Only READY yields a `HistoryEvidence`.
- **VIOLATION:** a source signature, scope, owner, inventory, event coverage or receipt binding is contradictory or invalid.
- **INCONCLUSIVE:** a source is missing, a manifest event is absent, a verifier/key is unavailable, the capture interval is truncated or completeness cannot be established.

INCONCLUSIVE is never a pass. A concrete violation takes precedence if a bundle contains both invalid and incomplete evidence.

## 2. Source stream manifest

Each source manifest binds:
- source ID and stream ID;
- optional effect-owner ID (coordinator streams use no owner);
- exact realm/resource/action scope digest;
- event schema version and source/build digest;
- first and last source cursor / capture watermark;
- the exact expected event ID set and event count;
- canonical event-set digest;
- source receipt/signature commitment.

Manifests are independently authenticated. The verifier receives both the manifest and the exact covered event records in canonical event-ID order. It must recompute the event-set commitment over the records and verify that the signed cursor interval is complete according to that source's semantics. Verifying the manifest signature without verifying the contents and completeness of its event set is insufficient.

The reference type uses a `BTreeSet` of event IDs, so duplicate raw IDs must be rejected by the manifest decoder before construction; the declared event count is checked against the set size as a second control.

The ingest layer rejects:
- missing or duplicate manifest identities;
- scope, owner or source substitution;
- event IDs declared but missing from the bundle;
- events not covered by any manifest;
- an event claimed by more than one source manifest;
- zero or malformed commitments;
- owner streams that do not cover the complete required owner set.

## 3. Independent authenticator interface

`EvidenceAuthenticator` is an explicit dependency supplied by the qualification/deployment layer. It must:
1. verify each signed source manifest and its exact covered event-set commitment;
2. independently verify each inventory closure and exact route-to-owner mapping;
3. verify the global owner-ack-set commitment against the exact canonical owner receipts, not a count or a caller-provided list;
4. authenticate source/build identity and the declared capture-completeness policy.

Each method returns `Verified`, `Invalid` or `Unavailable`. Invalid evidence becomes VIOLATION. An unavailable trust root, key service, proof verifier or source is INCONCLUSIVE. The adapter has no permissive fallback.

Cryptographic primitives, trust-root storage, issuer authorization, canonical hashing and signature verification are not implemented in this reference module. Their absence must not be hidden behind a boolean flag. Production adapters must use actual authenticated evidence.

## 4. Exact owner and inventory coverage

For every `InventoryClosed` event, ingest derives the owner set and checks that the values of its route-to-owner map project to the exact declared owner set. It authenticates the closure proof using the external verifier. It then requires an authenticated complete stream manifest for each required owner.

The global barrier's `owner_ack_event_ids` are resolved to actual `OwnerAcknowledged` event records. A missing record is INCONCLUSIVE. A wrong event type is VIOLATION. The authenticator receives the exact global barrier event and resolved acknowledgement records and must recompute the canonical sorted owner-ack-set commitment and compare it to the global receipt.

An owner acknowledgement must never be inferred from a broker delivery, message ACK, publisher confirm, source-manifest count or coordinator status log.

## 5. Rust implementation

Adds `tools/myc-dir-fabric-history-oracle/src/ingest.rs`:
- `SourceStreamManifest`, `RawHistoryBundle` and `EvidenceAuthenticator`;
- `VerificationStatus::{Verified, Invalid, Unavailable}`;
- `IngestVerdict::{Ready, Violation, Inconclusive}`;
- structured finding codes and an `IngestReport`;
- `ingest_history`, which derives exact event/owner coverage and constructs `HistoryEvidence` only on READY.

The `HistoryEvidence` completeness fields in `src/lib.rs` are no longer externally writable. Callers can inspect the exact scope, event count and verified owner IDs through read-only getters.

There are 14 authored ingest tests covering:
- authenticated complete source manifests;
- missing required owner stream;
- invalid and unavailable source verifier;
- declared event missing from bundle;
- overlapping/duplicated event coverage;
- owner and scope substitution;
- invalid inventory-closure proof;
- unavailable and invalid global owner-ack-set verifier;
- missing referenced owner-ack event;
- uncovered event and duplicate manifest identity.

Tests are authored, not compiled or executed.

## 6. Benchmark and comparison

The companion corpus `MYC_DIR_HISTORY_INGEST_V0_1.json` contains known-READY, known-VIOLATION and known-INCONCLUSIVE fixtures. The hard correctness requirement is zero false READY on any invalid or incomplete input. The checker must preserve INCONCLUSIVE during source or verifier outages.

Benchmark event sizes: 100, 1k, 100k, 1m and 10m; owner streams: 1, 4, 16, 64 and 256; at least ten independent open-loop repetitions per configuration. Measure source signature/proof verification, exact event coverage, canonical owner-ack recomputation, peak memory, ingest throughput and end-to-end p50/p95/p99 latency. Preserve raw evidence, manifests, verifier results, build digests and failed histories.

This is not a claim that Mycelix has invented evidence provenance or signed logs. It is the explicit bridge between the domain-specific history oracle and independently authenticated source records. The generic checker comparisons remain Porcupine and Jepsen; the ingest layer adds the source provenance and completeness contract needed before those histories can be trusted.

## 7. Explicit limitations

- No production signature verifier, trust-root store or proof system is implemented here.
- The authenticity/completeness guarantee depends on the `EvidenceAuthenticator` implementation.
- The adapter does not prove that a source's claimed cursor interval is complete; it delegates this to the source-specific verifier.
- The adapter does not read PostgreSQL WAL, Kafka offsets, JetStream sequence records or broker logs itself.
- The 14 tests are authored but not executed.
- No runtime, safety, performance or SOTA result is claimed.


## 8. Canonical event-set serialization

The ingest module now defines versioned canonical bytes for every typed EventKind and a deterministic event-set encoding:
- event identity, scope, owner, owner-local sequence and sorted causal parents are encoded;
- each event kind has a stable numeric tag and all of its typed fields are included;
- set/map fields are encoded in sorted order;
- the event set is sorted by event ID and each event is length-prefixed;
- duplicate event IDs are rejected before the event-set commitment can be computed.

The source authenticator must hash `canonical_event_set_bytes(covered_events)` and compare it with the signed manifest's event-set digest. The digest algorithm, signature and trust-root verification remain external. The canonical byte contract is versioned so the producers and verifier cannot silently choose different encodings.

The ingest crate now has 17 authored tests, including event-set ordering independence, duplicate event rejection and payload substitution detection. These tests remain uncompiled and unexecuted.
