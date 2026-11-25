# SAP owner-scoped causal graph projection v1

**Status:** effects-disabled reference verifier and deterministic partial-order projection. This is not a Holochain adapter, journal, authority protocol, payment execution path, settlement proof, or production concurrency guarantee.

## Why this exists

The SAP saga has two different execution boundaries:

- The sender-owned local payment invocation commits its local debit, recipient credit, payment/receipt records and Treasury outbox intent as one same-source-chain write set.
- Treasury fee delivery is a separate owner effect with its own stable identity and retry/reconciliation lifecycle.

A replay model must not turn those owner-local records into a fictional, globally ordered journal. The existing FIN-SAGA-STATE-SPACE-001 requirements explicitly call for owner-local frontiers and explicit causal references. This increment makes that requirement executable.

## Model

A v1 event carries:

- `owner_id`: an explicit owner namespace;
- `event_id`: an owner-native identifier, unique only within that owner namespace;
- `owner_sequence`: zero-based sequence within that owner's chain;
- `owner_predecessor`: the exact prior event on the same owner chain, absent only for sequence zero;
- `caused_by`: zero or more explicit references to events that causally precede this event.

An event reference is the pair `(owner_id, event_id)`. The model does not presume that two owners share a global counter, global timestamp, or globally unique local event IDs.

The v1 verifier requires each supplied owner history to be a complete prefix beginning at sequence zero. A future snapshot profile may accept truncated histories only when it is anchored to separately authenticated owner-frontier subjects. This implementation rejects gaps rather than guessing or filling them.

## Projection algorithm

1. Validate bounded owner/event identifiers and reject duplicate owner-scoped event references.
2. Group events by owner and validate contiguous owner-local sequence numbers.
3. Require each non-genesis event to point to the exact preceding event in its own owner chain.
4. Resolve every `caused_by` reference; reject missing, duplicate and self-references.
5. Combine owner-predecessor and explicit-causality edges. Duplicate representation of the same edge is harmless and is counted once.
6. Run deterministic Kahn topological layering. Each layer is the ready antichain for the current graph.
7. Reject a cycle. Produce independent per-owner frontiers and a canonical, domain-separated SHA-256 graph digest.

There is no timestamp, ingestion-time or lexical-order edge. Sorting is used only to make the report and canonical digest deterministic; it never adds a causal relationship.

### Interpreting layers

Two events in the same topological layer are **incomparable in the supplied graph**. This does not prove that they occurred simultaneously, nor does it prove that no unobserved causal relation exists. It means only that the provided owner-predecessor and explicit cause edges do not order them.

A layer is therefore not a distributed clock, consensus order, or settlement sequence. It is a deterministic representation of the partial order justified by the supplied references.

## Canonical digest

Domain: `mycelix:sap-causal-graph:v1\0`

Encoding:

- schema version: u16 big-endian;
- event count: u32 big-endian;
- events sorted by owner ID, owner-local sequence, then event ID;
- owner ID and event ID: u32 byte length followed by UTF-8 bytes;
- owner sequence: u64 big-endian;
- predecessor: one-byte presence tag, then owner/event reference if present;
- caused-by count: u32 big-endian, followed by references sorted by owner ID and event ID.

The shared Rust/Go known-answer fixture is `fixtures/sap-payment-causal-graph-v1.kat`. It pins one four-event graph with independent Payments and Treasury genesis events, a Payments successor and a Hearth event explicitly caused by both owner events.

The digest identifies canonical graph content only. It is not a signature, source-chain authorship proof, authorization grant, trusted timestamp, or evidence that the referenced events really occurred.

## Adversarial coverage

Rust and Go independently cover:

- independent owner effects remain in one incomparable layer;
- explicit cross-owner causality adds only the declared dependency;
- independent owner frontiers are preserved;
- input permutation does not alter digest, layers or frontiers;
- shared canonical digest known-answer vector;
- owner-sequence gaps;
- wrong owner predecessor;
- missing causal references;
- cross-owner causal cycles;
- duplicate owner-scoped event references;
- duplicate causal references;
- empty input.

The verifier also bounds the graph at 100,000 events, 64 causal references per event and 1,024 UTF-8 bytes per owner/event component.

## Complexity and benchmark plan

The graph projection uses adjacency lists and Kahn's algorithm. Its graph traversal is (O(V+E)), in addition to deterministic sorting and canonical serialization. It is not yet a measured performance claim.

The next qualification should report the following sizes on a frozen generated corpus:

| Events | Required measurements |
|---:|---|
| 1, 10, 100 | p50/p95/p99 projection latency, digest latency |
| 1,000, 10,000 | throughput, peak memory, bytes per event/edge |
| 100,000 | bounded completion, peak memory, deterministic report digest |

For every size, include chain-heavy, star-causality, wide-antichain, mixed-owner and near-cycle graphs. Keep valid graphs and hostile graphs separate. Publish raw samples, compiler/runtime versions, CPU and memory limits, and exact source SHA. Compare Rust and Go on identical canonical inputs; do not compare a microbenchmark to full Holochain or PostgreSQL performance.

Hard gates (targets, not achieved production results):

- zero accepted owner sequence gaps;
- zero missing-reference or cycle acceptance;
- zero event identity collisions within an owner namespace;
- zero added ordering edges not justified by the input;
- 100% input-permutation determinism for digest and projection;
- exact Rust/Go agreement on the shared KAT and all frozen disposition cases.

## Relationship to industry models

- Temporal durable execution reconstructs workflow state from durable event history and recommends idempotent Activities because activity attempts may be retried. The causal graph here is not a workflow engine and does not replace Temporal's execution history or retry policy.
- Transactional outbox patterns address the local database/message dual-write boundary but retain duplicate-delivery handling requirements. This graph does not remove the need for idempotent destination effects and reconciliation.
- Jepsen-style history checking is the right later comparison for a live distributed runtime: record actual invocation/completion histories under partitions, crashes and retries, then check them against the declared consistency model. This finite graph verifier is only an offline reference checker.
- OCEL 2.0 is a useful process-mining exchange model for multi-object events and object-to-object relationships. A future exporter could emit the validated event/object relationships for analysis, but OCEL conformance must remain separate from Mycelix authority, evidence and finality claims.

## Claim ceiling and next boundary

This v1 module validates a complete, caller-supplied graph. It does not:

- authenticate owner IDs or event references;
- prove Holochain action authorship, source-chain membership or DHT completeness;
- establish rollback resistance, fork choice, Byzantine safety or global uniqueness;
- prove an explicit causal reference is truthful;
- infer external bank settlement, legal discharge, physical delivery or beneficiary acceptance;
- recover truncated histories without an authenticated base frontier;
- measure production latency, throughput, availability or operating cost.

The next runtime increment should bind each node reference to a verified Holochain action, bind each owner frontier to the native signed action and authorization scope, and test partial DHT visibility, restarts, fork observations and stale frontiers. This model is a structural building block for that work—not its substitute.
