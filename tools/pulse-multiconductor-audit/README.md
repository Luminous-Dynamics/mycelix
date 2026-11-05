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

The v2 report binds input bytes by SHA-256 and returns PASS, FAIL or INCOMPLETE. A verified-record signal must resolve an exact source-chain action, payload digest, accepted validator event, validator WASM digest, observer lineage and both causal edges. A missing causal edge or missing authority evidence is INCOMPLETE; a contradictory decision, cycle, false-fresh projection or unauthorized effect is FAIL.

## Synthetic performance benchmark

Build and run one deterministic synthetic benchmark sample:

    cargo run --release --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- \
      --benchmark-events 100000 --output benchmark.json

The accepted event count range is 1 through 1,000,000. The generator creates two peer conductors with independent local sequence counters and unverified remote-signal events. It has no cross-conductor causal edges, because the current purpose is to measure parsing-independent structural audit cost on a controlled safe history.

The benchmark report separates:
- synthetic history build time;
- verifier audit time;
- event count / conductor count / edge count;
- audit events per second;
- status and finding count;
- a deterministic generator-configuration commitment;
- an explicit synthetic-only claim ceiling.

The Actions benchmark job runs 30 independent processes at 1k, 10k and 100k events, plus one 1M-event capacity probe. It records peak RSS per process and preserves every raw run plus a summary. The 1M sample is a capacity probe, not a percentile population. This measures the verifier on synthetic remote-signal histories; it does not establish Holochain runtime throughput, DHT convergence performance or comparative SOTA superiority.

A `verified_record` signal must resolve the exact authored action, match its canonical payload SHA-256, resolve an accepted validator event in the same DNA/cell lineage, match the receiving conductor's integrity-WASM digest, and have explicit author→validator→observer causal edges. `authenticated_peer_hint` is bound to the transport-observed sender but remains advisory; `local_only` cannot claim remote sender metadata.

A performance optimization is accepted only if the same negative-control suite still detects foreign lineages, missing validator receipts, digest mismatches, forged signal claims, cycles, false-fresh projections and invalid authorization histories.

## Claim ceiling

This is a structural partial-order verifier. It does not authenticate evidence references, validate Holochain signatures, prove that a conductor is honest, establish DHT consensus, or prove unbounded linearizability. A production runtime capture must be separately signed/attested and independently verified.
