# Pulse multi-conductor history auditor

A Rust-first independent structural verifier for the Pulse multi-conductor causal-history format.

## Purpose

The auditor consumes redacted multi-conductor histories without inventing a global clock. Each conductor has its own local_sequence and host_authority_index; those counters are compared only within that conductor. Cross-conductor ordering exists only when an explicit, evidence-referenced causal edge connects two events.

The v1 contract distinguishes:
- host grant issuance/revocation and host call decisions;
- private capability claim acceptance;
- source-chain action authoring;
- receiving-conductor validator acceptance/rejection;
- observer projection state;
- remote signal trust class;
- caller-visible outcomes and durable domain effects.

## Run

    cargo test --manifest-path tools/pulse-multiconductor-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-multiconductor-audit/Cargo.toml -- --input tools/pulse-multiconductor-audit/examples/safe-history.json

The report binds the input bytes by SHA-256 and returns PASS, FAIL or INCOMPLETE. A missing causal edge or missing authority evidence is INCOMPLETE; a contradictory decision, cycle, false-fresh projection or unauthorized effect is FAIL.

## Claim ceiling

This is a structural partial-order verifier. It does not authenticate evidence references, validate Holochain signatures, prove that a conductor is honest, establish DHT consensus, or prove unbounded linearizability. A production runtime capture must be separately signed/attested and independently verified.
