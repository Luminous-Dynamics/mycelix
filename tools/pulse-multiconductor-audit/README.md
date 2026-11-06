# Pulse multi-conductor history auditor

A Rust-first independent structural verifier for the Pulse multi-conductor causal-history format.

## Purpose

The auditor consumes redacted multi-conductor histories without inventing a global clock. Each conductor has its own local_sequence and host_authority_index; those counters are compared only within that conductor. Cross-conductor ordering exists only when an explicit, evidence-referenced causal edge connects two events. A Holochain CellId is agent-specific (DNA hash + AgentPubKey), so different peers in one DNA network are expected to have different cell IDs. Cross-conductor validation and observer evidence are joined by the shared DNA hash and exact integrity-WASM lineage; each reported cell ID is checked against its own conductor and is never required to equal another peer's CellId.

The v2 contract distinguishes:
- host grant issuance/revocation and host call decisions;
- private capability claim acceptance;
- source-chain action authoring;
- receiving-conductor validator acceptance/rejection;
- observer projection state;
- remote signal trust class, exact source-action reference, payload digest, receiving-validator event and integrity-WASM identity;
- caller-visible outcomes and durable domain effects.

## Run

    cargo test --manifest-path tools/pulse-multiconductor-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- --input tools/pulse-multiconductor-audit/examples/safe-history.json

The CLI first runs the borrowed v2 closed-world preflight over the original input bytes, then materializes the owned history used by the current semantic audit. This is deliberate: the preflight rejects duplicate JSON object keys inside raw event payloads, which would otherwise be collapsed by `serde_json::Value` map materialization. The preflight also validates exact event shapes and required scalar/array types. A duplicate-key or shape error fails before the semantic report is produced. The current production auditor still materializes the owned history after this gate; the borrowed path has not yet replaced the full semantic event representation, so the memory-reduction target remains open under #5872. Differential mutation tests compare borrowed and owned event-shape decisions, and a CLI-entrypoint regression proves duplicate payload keys fail closed.
The v2 report binds input bytes by SHA-256 and returns PASS, FAIL or INCOMPLETE. A verified-record signal must resolve an exact source-chain action, payload digest, accepted validator event, validator WASM digest, observer lineage and both causal edges. A missing causal edge or missing authority evidence is INCOMPLETE; a contradictory decision, cycle, false-fresh projection or unauthorized effect is FAIL.

## Synthetic performance benchmark

Build and run one deterministic synthetic benchmark sample:

    cargo run --release --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- \
      --benchmark-events 100000 --output benchmark.json

The accepted event count range is 1 through 1,000,000. The generator creates two peer conductors with independent local sequence counters and unverified remote-signal events. It has no cross-conductor causal edges, because the current purpose is to measure parsing-independent structural audit cost on a controlled safe history.

The benchmark report separates:
- synthetic history build time and verifier audit time;
- event count / conductor count / edge count;
- audit events per second;
- allocator live bytes after history construction and after audit;
- build-phase and audit-phase peak live allocation bytes;
- cumulative requested allocation bytes by phase;
- status and finding count;
- a deterministic generator-configuration commitment;
- an explicit synthetic-only claim ceiling.

The allocator metrics come from a benchmark-only counting global allocator. Atomic accounting adds overhead to allocation/deallocation, so throughput from this instrumented build must not be compared directly with earlier uninstrumented runs. The workflow's /usr/bin/time peak-RSS measurement remains independent and is still captured per process.

The Actions benchmark job runs 30 independent processes at 1k, 10k and 100k events, plus one 1M-event capacity probe. It records peak RSS per process and preserves every raw run plus a summary. The 1M sample is a capacity probe, not a percentile population. This measures the verifier on synthetic remote-signal histories; it does not establish Holochain runtime throughput, DHT convergence performance or comparative SOTA superiority.

A `verified_record` signal must resolve the exact authored action, match its canonical payload SHA-256, resolve an accepted validator event in the same DNA/cell lineage, match the receiving conductor's integrity-WASM digest, and have explicit author→validator→observer causal edges. `authenticated_peer_hint` is bound to the transport-observed sender but remains advisory; `local_only` cannot claim remote sender metadata.

A performance optimization is accepted only if the same negative-control suite still detects foreign lineages, missing validator receipts, digest mismatches, forged signal claims, cycles, false-fresh projections and invalid authorization histories.

## Reachability strategy comparison

The current semantic auditor retains three independently selectable reachability strategies over one immutable, cycle-checked adjacency graph:

- **Uncached DFS** — reference implementation. Performs a fresh graph traversal for each non-shortcut query.
- **Exact-pair memoization** — caches the boolean result for an exact `(from_event_id, to_event_id)` pair, including negative results, with a hard 100,000-pair cap. It is valid only for the same in-memory graph and one audit invocation.
- **Bounded per-source memoization** — after repeated queries from a source, builds that source's complete reachable set if the graph and remaining global 100,000-node budget allow it; otherwise it remains on DFS.

All three strategies preserve the same semantics:
- same-conductor local-sequence order is a shortcut only inside one conductor;
- direct adjacency is an explicit-edge shortcut;
- cross-conductor transitive order requires graph traversal;
- direct relation-type checks remain separate from transitive reachability;
- a topological position is never substituted for a reachability proof;
- the optimized result is differential-checked against uncached DFS on identical history bytes.

Claim-fanout comparison:

    cargo run --release --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- \
      --benchmark-join-operations 1000 --output join-benchmark.json

The report contains the uncached, exact-pair and per-source audit times, cache hits, memory/allocation metrics, and canonical semantic report digests. It fails unless all three outputs have identical semantic digests and PASS status.

## Adversarial DAG reachability microbenchmark

The graph-only benchmark separates reachability-structure cost from full-history parsing, evidence joins and domain semantics. It assigns graph nodes across 2–50 conductor labels with deliberately non-comparable local sequence values. Cross-conductor reachability is determined only by explicit directed edges.

    cargo run --release --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- \
      --benchmark-dag-shape wide_fanout \
      --benchmark-dag-nodes 1000 \
      --benchmark-dag-conductors 10 \
      --benchmark-dag-queries 2048 \
      --output dag-benchmark.json

Supported shapes:
- `long_chain`
- `wide_fanout`
- `wide_fanin`
- `diamond`
- `sparse_dag`
- `bounded_dense_dag` (maximum 512 nodes)

The report records node/edge/conductor/query counts, output-vector digests, elapsed time, cache hit/entry counts and DFS node visits for all three strategies. The result digests must match byte-for-byte as serialized boolean vectors. Unit tests cover every shape at conductor counts 2, 3, 10 and 50; the wide-fanout smoke case requires both cache strategies to achieve hits.

This is a graph-algorithm microbenchmark, not a complete lifecycle-history audit, Holochain runtime measurement, or DHT convergence benchmark. The GitHub workflow runs five process-level samples per shape/conductor combination. Those are scaling samples and report medians/ranges only; they do not report p95/p99. A later full-history verified-record profile must still exercise the author/action/validator/observer joins tracked in #5888.

## Verified-record structural stress profile

Build and run a deterministic synthetic author/validator/observer evidence history:

    cargo run --release --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- \
      --benchmark-verified-signals 10000 \
      --benchmark-verified-conductors 10 \
      --output verified-record-benchmark.json

Each record generates:
1. one source-chain action-authorship event bound to an exact ActionHash and canonical payload digest;
2. one accepted receiving-validator disposition bound to the author conductor, DNA and exact integrity-WASM digest;
3. one observer-side verified-record signal bound to the transport sender, record author, exact ActionHash, payload digest and validator event;
4. two evidence-backed edges: author→validator (`validator_receipt`) and validator→observer (`observer_confirmation`).

The default is three conductors (one author, one validator, one observer). The benchmark accepts 3–50 conductors; additional author conductors spread records deterministically across their independent local sequence spaces. The output runs uncached DFS, exact-pair memoization and per-source reachability over the identical history and requires identical canonical semantic report digests and PASS status.

The CI profile runs 30 independent processes at 1k, 10k and 100k signals with 10 conductors. It separately runs a 10k-signal conductor-scaling stratum at 3 and 50 conductors, and one 1M-signal capacity probe. Percentiles are emitted only for populations with at least 20 completed independent processes. The one-shot capacity result is recorded separately as PASS or INCOMPLETE; it is not used to infer a percentile.

This is synthetic structural evidence-lineage verification. It does not verify real cryptographic signatures, prove the claimed validator executed in a real conductor, establish DHT integration, or prove that the underlying event occurred. A successful audit means only that the generated evidence graph is internally consistent under this verifier's declared semantics.

## Claim ceiling

This is a structural partial-order verifier. It does not authenticate evidence references, validate Holochain signatures, prove that a conductor is honest, establish DHT consensus, or prove unbounded linearizability. A production runtime capture must be separately signed/attested and independently verified.
