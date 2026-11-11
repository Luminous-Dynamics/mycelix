# Measurement evidence dependency and invalidation oracle

This package implements the read-only graph semantics for Mycelix #6042. It reuses the measurement source records and owner contracts; it does not create a new ledger or mutate source evidence.

## Invariants

- Each node binds a stable ID, node kind, subject, revision, content digest and source state.
- Every edge is directed dependency → dependent and binds the exact parent revision and digest.
- Identical duplicate nodes are idempotent; conflicting duplicate IDs, duplicate/self/dangling edges and parent-binding mismatches are rejected.
- The graph is a DAG. Canonical topological order selects the lexicographically smallest currently-ready node, independent of input node/edge order.
- Invalidation events bind one exact target node/revision/digest. Duplicate event IDs, duplicate targets, missing targets and stale target bindings are rejected.
- A non-current source state invalidates transitive dependants; independent branches remain current. Historical source bytes are not rewritten.
- The evaluator only classifies graph state. It does not authenticate the source evidence or authorize measurements/device effects.

## Frozen corpus

32 synthetic cases cover currentness, calibration/firmware/installation/purpose/assay/sample/uncertainty/method/rights changes, independent branches, edge/node/event integrity, cycles, canonical ordering and the no-current-evidence boundary. The Rust oracle pins every case ID, disposition, error code and a stable FNV-1a digest of the complete expected result, so changing node states or invalidation sets together with the fixture cannot silently redefine the known answer.

## Claim ceiling

Passing the corpus establishes deterministic dependency and affected-descendant invalidation semantics only. It does not authenticate certificates, establish calibration status, prove physical measurement accuracy, compute uncertainty or qualify a building/service outcome.
