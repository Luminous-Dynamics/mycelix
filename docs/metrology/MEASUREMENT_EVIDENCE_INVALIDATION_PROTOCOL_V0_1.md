# Measurement evidence dependency and invalidation protocol v0.1

Parent: Mycelix #6042. This is a measurement-specific composition of #5805 DAG semantics and #5149 provenance correction/revocation patterns. It is not a new generic ledger.

## Core theorem

A measurement-derived claim is current only while every exact required upstream evidence dependency remains current for the declared purpose. Corrections, revocations, expiry, supersession, rights changes and configuration revisions invalidate only affected descendants.

## Data model

- A node is a versioned evidence object with stable ID, node kind, subject ID, revision, content digest and state.
- An edge points `dependency -> dependent` and binds the exact dependency revision and digest.
- An invalidation event binds the exact target node ID, revision and digest and names the change kind.
- The evaluator is read-only. It canonicalizes and classifies; it does not issue certificates, modify samples, operate instruments or authorize a device.

## Deterministic semantics

1. Exact duplicate node records are idempotent; the same node ID with changed content is a conflict.
2. Duplicate, self and dangling edges are invalid.
3. Parent revision/digest mismatch invalidates the graph input as a conflict; it is not interpreted as an older valid parent.
4. The dependency graph must be acyclic.
5. Canonical topological order uses lexicographically smallest currently-ready node first, independent of input node/edge array order.
6. A node starts with its source state; an event may mark its exact target corrected/revoked/expired/superseded.
7. Any dependent node whose required parent is not current becomes `invalidated`; this propagates transitively. The source node retains its own `unknown`/`revoked`/`expired`/`superseded`/`corrected` state. `invalidated_nodes` lists only dependent descendants, not the source node.
8. Independent branches remain current.
9. A calibration update invalidates dependent measurements/decisions only; a purpose-profile update can invalidate a claim while the raw sample remains current; an assay mismatch/revocation does not rewrite the raw measurement.
10. A graph error yields `INVALID_GRAPH` and no topological order or currentness claim.
11. `CURRENT` means every graph node remains current. `PARTIALLY_INVALIDATED` means at least one node is non-current while at least one node remains current, including a revoked/expired source leaf with no dependants. `NO_CURRENT_EVIDENCE` means no graph node is current. These labels do not upgrade any source state.

## Corpus

32 frozen synthetic cases cover current graphs, calibration revocation/expiry, firmware and installation changes, purpose-profile correction, independent-assay revocation, sample supersession, unknown and already superseded parents, multiple independent branches, parent digest/revision conflicts, dangling/self/duplicate edges, conflicting/idempotent duplicate nodes, two-/three-node cycles, duplicate event IDs/targets, absent event targets, stale event revisions/digests, uncertainty/method corrections, rights revocation and a graph with no current evidence.

## Claim ceiling

Passing this corpus establishes deterministic graph and invalidation semantics only. It does not authenticate certificates, establish actual calibration, prove physical accuracy, or qualify a service/building outcome.
