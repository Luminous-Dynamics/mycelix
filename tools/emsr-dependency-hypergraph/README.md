# EMSR dependency hypergraph reference evaluator 0.1

Status: **research prototype; not yet independently qualified**.

This isolated Rust crate is the first executable reference for Mycelix #5923. It models typed, time-qualified dependency hyperedges and derives transitive shared ancestors for a declared observation time and evidence frontier.

## Semantics

- Direction is dependent node → dependency node.
- A hyperedge can bind multiple dependent subjects to one dependency node.
- Queries select one dependency domain at a time; power does not imply clock.
- Valid-time intervals are half-open: `[valid_from, valid_until)`; a null bound is explicitly unbounded.
- The inventory snapshot must cover the event time, requested source set and every visited node.
- An assertion recorded after the evidence frontier, or an applicable stale/revoked/conflicted/unqualified assertion, produces an unknown result.
- Qualified common ancestors produce `SHARED_DEPENDENCY`.
- `NO_DECLARED_COMMON_DOMAIN` is permitted only when the declared inventory is complete, current for the query, and closure has no common ancestor.
- Cycles and malformed/dangling/duplicated identities are invalid inputs.
- No output asserts physical or statistical independence.

## Run

```sh
cargo test --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml
cargo run --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --bin emsr-dependency-hypergraph -- evaluate input.json report.json
cargo run --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --bin emsr-dependency-hypergraph -- verify input.json report.json
```

The crate is standalone by design. It does not add a Holochain zome, evidence ledger, trust root, clock service or production graph database.

## Independence boundary

The evaluator and reference verifier use separate traversal implementations (iterative breadth-first closure vs recursive depth-first closure) but share the Rust data contract and cryptographic/serialization libraries. This proves code-path disagreement can be detected in the current crate; it does not prove organizational, language, dependency, custody or hardware independence. A second implementation in a different language/library remains required.

## Scale work

The initial prototype does not claim scale or performance. Follow-on measurements must report p50/p95/p99 runtime and peak memory at declared node, hyperedge and reachability densities, plus exact expected outcomes and false-independent negative controls.
