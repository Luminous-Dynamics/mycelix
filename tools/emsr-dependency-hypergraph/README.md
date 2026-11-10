# EMSR dependency hypergraph reference evaluator 0.1

Status: **research prototype; synthetic software-conformance coverage only; no external organizational qualification or production qualification is claimed**.

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

The evaluator and Rust reference verifier use separate traversal implementations (indexed iterative breadth-first closure vs a separately indexed depth-first event stack) but share the Rust data contract and cryptographic/serialization libraries. This proves code-path disagreement can be detected in the current crate; it does not prove organizational, language, dependency, custody or hardware independence. A second implementation in a different language/library remains required.

## Scale work

The topology benchmark below measures only deterministic synthetic graph cases. It is intended to expose shape-dependent runtime and memory behavior, not to establish physical independence, field throughput or operational performance.

## Cross-language report verification

A separate Go implementation is provided in `tools/emsr-dependency-hypergraph-go-verifier`. It uses only the Go standard library, parses the exact input/report contracts, recomputes the dependency closure with its own indexed, explicit depth-first event stack, and checks the input-byte digest, query/snapshot binding, closure, disposition, reason codes, authority and claim ceiling.

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

The standard-library Rust driver at `examples/scale_benchmark.rs` emits a versioned, synthetic-only report. It measures these graph families:

- `star_shared_root`: 10, 100, 1,000, 10,000 and 100,000 sources sharing one dependency hyperedge.
- `deep_shared_chain`: 10, 100, 1,000 and 10,000 sources traversing a 256-edge common lineage.
- `paired_deep_chains_depth_10000`: two independent 10,000-edge dependency lineages converging on one common ancestor. This is a dedicated depth-stress case with two sources, not a 10,000-source case.
- `layered_hierarchy`, `overlap_clock`, `overlap_power`, `overlap_network`, `sparse_independent`, `unknown_dependency`, `stale_dependency`, `unavailable_dependency` and `partial_inventory_unknown`: each measured at 10, 100, 1,000 and 10,000 sources. The final three uncertainty families explicitly distinguish unqualified common-cause assertions from missing dependency metadata in a snapshot whose `complete` flag is false.

Each case records its topology, dependency domain, source/node/hyperedge counts, total subject-reference count, direct source-hyperedge-reference count, average direct hyperedges per queried source, serialized input bytes and SHA-256, expected and actual disposition, verifier status/check count, and Linux high-water RSS when available. The Rust evaluator and separately implemented Rust reference verifier are timed separately. The evaluator timing includes strict input parsing and evaluation; report serialization is outside the timed interval. Each timing record retains the raw per-run samples in measurement order as well as p50/p95/p99, min and max.

The benchmark defaults to 101 measured repetitions after one unmeasured warm-up. `GCPN_BENCH_REPETITIONS` is clamped to 101–501. Reports include the exact commit SHA when available, host OS/architecture, rustc version and build profile. The Go standard-library verifier is not timed by this Rust harness; its test suite separately checks the public challenge pack, civil-domain cases and a matching two-branch 10,000-edge lineage. Do not interpret that as a performance comparison between languages.

Run in optimized mode:

```sh
cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark -- /tmp/emsr-hypergraph-scale.json
GCPN_BENCH_REPETITIONS=101 cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark
```

The production 16 MiB parser limit is unchanged. Oversized synthetic cases are retained in the output with `REJECTED_BY_INPUT_SIZE_LIMIT`; they are not silently omitted or scored. Results are comparable only when exact input digests, source revision, toolchain, build profile and host are retained. Report each topology separately; do not collapse latency, memory and correctness into a composite score.

The benchmark is synthetic software evidence only. It does not establish physical or statistical independence, source authenticity, sensor accuracy, field throughput, certification or operational safety.

## Scale and traversal behavior

The main Rust evaluator retains its indexed breadth-first closure; the independent Rust reference traversal now uses a separately built subject index and explicit depth-first event stack. The Go reference verifier uses its own subject index and explicit DFS event stack. Both avoid recursive stack growth and repeated full-hyperedge scans for each visited node.

The Rust and Go integration tests include two 10,000-edge-deep maritime lineage branches converging on a shared ancestor. The Rust benchmark now times a two-source instance of that shape, with raw evaluator and Rust-reference samples. This remains a synthetic traversal stress case, not physical or field-performance evidence.

Both Rust and Go parsers enforce the same 16 MiB input limit. The Rust CLI's bounded reader and both language-level parsers reject oversize input before full deserialization.

## Parser and byte-boundary hardening

Input and report parsing is strict in both language implementations:
- duplicate JSON object keys are rejected at every nesting level;
- unknown input fields are rejected (Rust uses deny_unknown_fields; Go uses DisallowUnknownFields);
- multiple trailing JSON values are rejected;
- Rust CLI reads use a capped Read::take(max+1) rather than a metadata-then-unbounded-read sequence; Go uses io.LimitReader.

Treat qualification as exact-head evidence: a result applies only to the commit SHA whose Rust and Go workflows actually completed. A later code or benchmark change requires fresh exact-head checks. A passing parser test proves only deterministic interpretation of supplied JSON bytes, not issuer authenticity.

Follow-on integration: [#5997 — civil-domain hypergraph and parser integrity](https://github.com/Luminous-Dynamics/mycelix/issues/5997).