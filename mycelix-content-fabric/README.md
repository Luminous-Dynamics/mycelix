# Mycelix Content Fabric hApp

CF-05 is the metadata-only Holochain coordination plane for Content Fabric.

It intentionally does **not** store bulk content, grant transport reads, create leases, make placement decisions, prove failure-domain independence, or settle payments.

## Evidence model

- `ProviderAdvertisementV1` — Holochain-authored service claim with an Ed25519 proof that the author controls the advertised Iroh endpoint key.
- `ContentAvailabilityClaimV1` — opt-in provider claim that a digest is serviceable. Absence means only "not advertised".
- `ReplicaObservationV1` — one observer's signed report about a transfer/verification attempt.
- `ProviderWithdrawalV1` — append-only early retirement of an advertisement.

All application entries and Content Fabric index links are append-only. Current state is derived from historical actions, TTLs, and withdrawals.

## Fresh endpoint binding

Call `get_provider_binding_context(())` immediately before creating an advertisement. It returns the calling Holochain agent and current source-chain head. The Iroh endpoint signs that chain head together with the complete advertised capability body.

`publish_provider_advertisement` must be the next source-chain write. Integrity validation independently requires the signed `binding_prev_action` to equal the advertisement Create action's `prev_action`.

An endpoint proof is therefore single-use at one chain position: replaying an old signed advertisement after the source chain advances fails validation.

## Trust boundaries

Endpoint-key possession is cryptographically verified. Provider failure-domain labels remain self-claims and cannot alone satisfy independence policy. Availability claims are not content-integrity proof. `VerifiedComplete` is an observer statement, not universal truth.

Transport authorization remains local to CF-04's `ReadAuthorizerV1`; discoverability never implies byte access.

## Workspace

```bash
cargo fmt --manifest-path mycelix-content-fabric/Cargo.toml --all -- --check
cargo clippy --manifest-path mycelix-content-fabric/Cargo.toml --workspace --all-targets -- -D warnings
cargo test --manifest-path mycelix-content-fabric/Cargo.toml --workspace --all-targets
cargo check --manifest-path mycelix-content-fabric/Cargo.toml --workspace --target wasm32-unknown-unknown --release
```

Packaging into `.dna`/`.happ` artifacts is intentionally separate from Rust correctness and requires the Holochain CLI.
