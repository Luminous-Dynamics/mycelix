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

## Cross-language report verification

A separate Go implementation is provided in `tools/emsr-dependency-hypergraph-go-verifier`. It uses only the Go standard library, parses the exact input/report contracts, recomputes the dependency closure using its own recursive traversal, and checks the input-byte digest, query/snapshot binding, closure, disposition, reason codes, authority and claim ceiling.

Run:

```sh
cd tools/emsr-dependency-hypergraph-go-verifier
go test -race -count=1 ./...
go run . verify input.json report.json
```

The Go test consumes the same versioned public challenge pack as the Rust reference tests. Both implementations are checked against frozen expected outcomes; the Go suite also rejects a stale input digest and authority/claim-ceiling tampering.

This raises the independence boundary from a second Rust traversal to a second language and standard-library implementation. It is still **not organizationally independent**: both implementations and tests are maintained in the same repository, and the public challenge pack is visible. A blind evaluator-owned test set, separately controlled custody, external reference measurement and independent review remain necessary for stronger assurance claims.

## Metamorphic assurance controls

The Rust suite also includes metamorphic tests that do not depend on a single hand-authored expected report:

- Node/hyperedge ordering permutations preserve semantic closure and remain independently verifiable.
- A shared power-domain hyperedge cannot change a clock-domain result.
- The half-open temporal boundary is tested at both `valid_from` (included) and `valid_until` (excluded).
- Marking the inventory incomplete cannot upgrade `NO_DECLARED_COMMON_DOMAIN`.
- A known common ancestor remains visible in the report even when another inventory-completeness predicate forces the overall disposition to `UNKNOWN_DEPENDENCY`.
- Inserting a qualified common ancestor changes the result from no declared common domain to shared dependency; making that edge stale produces unknown, never a clean independent result.

These are metamorphic software properties, not statistical independence or physical fault-injection results.


## Civil-domain consumer profiles

The same graph evaluator is now exercised by a separate civil-domain integration test module, without changing the graph vocabulary or adding a second implementation:

- **UAS:** two RF observation feeds with distinct immediate collectors but one shared collector ancestor (collector).
- **Maritime:** AIS and radar processing paths with distinct immediate pipelines but one shared port-correlation platform (data_pipeline).
- **Civil aviation:** ADS-B and MLAT paths with separate clock domains but a shared airport power bus. A clock query remains NO_DECLARED_COMMON_DOMAIN; a power query returns SHARED_DEPENDENCY.
- **Positive control:** a complete inventory with no common ancestor remains NO_DECLARED_COMMON_DOMAIN; the report claim ceiling explicitly says this is not proof of physical/statistical independence.

Each Rust civil-domain profile also passes the existing separate Rust reference traversal. The Go standard-library verifier has matching profile tests and verifies the same report contract.

## Reproducible scale benchmark

An isolated standard-library benchmark driver is available at `examples/scale_benchmark.rs`. It generates a synthetic star-shaped dependency hypergraph at 10, 100, 1,000, 10,000 and 100,000 sources, with one multi-subject shared-clock hyperedge. For each size it reports input SHA-256/bytes, node/edge counts, evaluator p50/p95/p99/min/max, independent-reference-verifier p50/p95/p99/min/max, disposition, verifier status and Linux process high-water RSS where available.

Run in optimized mode:

```sh
cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark -- /tmp/emsr-hypergraph-scale.json
GCPN_BENCH_REPETITIONS=31 cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark
```

The default is 9 repetitions; the environment variable is clamped to 5–101. Results include OS, architecture, rustc version and debug/release state. The 16 MiB parser limit is not relaxed for the benchmark; oversized synthetic cases are reported as rejected rather than silently excluded or scored.

The driver is a synthetic scale harness, not an observed result. Do not publish p50/p95/p99 or memory numbers until the exact revision, host, toolchain, input digests and complete output JSON are retained. The star graph is intentionally simple; follow it with chain, fan-in/fan-out, dense dependency, unknown-edge, temporal filtering and many-small-components shapes before making general throughput claims.

## Scale and traversal behavior

The main Rust evaluator retains its indexed breadth-first closure; the independent Rust reference traversal now uses a separately built subject index and explicit depth-first event stack. The Go reference verifier uses its own subject index and explicit DFS event stack. Both avoid recursive stack growth and repeated full-hyperedge scans for each visited node.

The integration tests include two 10,000-edge-deep maritime lineage branches converging on a shared ancestor. This is a stack-safety/correctness stress case, not a latency or throughput benchmark. Benchmark runs must still report p50/p95/p99, peak memory, graph shape and input digests at declared source/hyperedge sizes before any performance claim.

Both Rust and Go parsers enforce the same 16 MiB input limit. The Rust CLI's bounded reader and both language-level parsers reject oversize input before full deserialization.

## Parser and byte-boundary hardening

Input and report parsing is strict in both language implementations:
- duplicate JSON object keys are rejected at every nesting level;
- unknown input fields are rejected (Rust uses deny_unknown_fields; Go uses DisallowUnknownFields);
- multiple trailing JSON values are rejected;
- Rust CLI reads use a capped Read::take(max+1) rather than a metadata-then-unbounded-read sequence; Go uses io.LimitReader.

The parser changes remain subject to exact-head workflow results on the integration PR. A passing parser test proves only deterministic interpretation of supplied JSON bytes, not issuer authenticity.

Follow-on integration: [#5997 — civil-domain hypergraph and parser integrity](https://github.com/Luminous-Dynamics/mycelix/issues/5997).