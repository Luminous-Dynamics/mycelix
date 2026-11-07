# Pulse Holochain 0.6 → 0.7.1 Migration

**Status: 0.7.1 target selected; this Pulse branch is an evaluation candidate and remains unqualified.** The previous 0.7.0 canary is a migration intermediate, not the target runtime.

## Target matrix

Use one coherent Holochain 0.7.1 generation across the Pulse Rust zomes, Sweettest harness, clients, and Nix environment:

| Component | Required candidate |
|---|---|
| Conductor / `holochain` | `0.7.1`, source tag `holochain-0.7.1` |
| `hc` CLI | `0.7.1` |
| `hdk` | `=0.7.1` |
| `hdi` | `=0.8.1` |
| `holochain_integrity_types`, `holochain_zome_types`, `holo_hash`, `hdk_derive` | `=0.7.1` |
| `holochain_serialized_bytes` | `0.0.57` |
| JavaScript client | `@holochain/client 0.21.1` |
| Rust client | `holochain_client 0.9.1` for the qualified backend/CLI boundary |
| Lair | `0.7.1` |
| Kitsune2 libraries used by Holochain | `0.5.1` |
| Kitsune2 bootstrap service from the inspected Holonix snapshot | `0.5.2` (separate from the Holochain library dependencies) |
| Node.js tooling | `24` |
| Pulse application Rust toolchain | `1.96.1`, via `mycelix-workspace/mycelix-pulse/rust-toolchain.toml` |
| Holonix moving-branch snapshot Rust | `1.95.0` in the inspected `main-0.7` snapshot; this is not the Pulse compiler pin |

The Holochain `holochain-0.7.1` source tag itself declares Rust `1.96.1` in its `rust-toolchain.toml`, and its committed Cargo.lock resolves HDK `0.7.1`, HDI `0.8.1`, Kitsune2 library packages `0.5.1`, and Lair `0.7.1`. Pulse therefore uses a local app toolchain file pinned to `1.96.1`; it must not inherit the shared Mycelix workspace `1.96.0` or mistake the Holonix `1.95.0` snapshot compiler for the release-source toolchain.

Primary migration sources:

- https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- https://github.com/holochain/holochain/tree/holochain-0.7.1
- https://blog.holochain.org/dev-pulse-157-holochain-0-7-1-brings-chain-restore-direct-signal-security/

The compatibility table still lists 0.7.0; track that as upstream documentation debt rather than silently downgrading the selected target. Holochain announced 0.7.1 ready for development on 5 November 2026.

## Why a pin-only upgrade is invalid

Holochain 0.7 changes the integrity action model. `Action` carries `header` plus `data`; payload variants are `ActionData::*`; shared header access uses `author()`, `timestamp()`, `action_seq()`, and `prev_action()`. The validation dispatcher changes from the 0.6 `FlatOp::StoreEntry` / `RegisterUpdate` / `RegisterCreateLink` shape to 0.7 `FlatOp::CreateEntry` / `Update` / `Link(OpLink::CreateLink { .. })` and related variants. This touches every integrity zome, not just Cargo manifests.

Sweettest and packaging also need separate attention. Holochain 0.7 removes tx5/WebRTC; Iroh/QUIC is the transport. Holochain 0.6 conductor databases and DNA hashes are not compatible with 0.7, so existing conductor data cannot be migrated in place. Use a fresh 0.7.1 data root and installation/network identity, or a separately designed application-level export/import process.

The 0.7.0 → 0.7.1 bump is also a protocol boundary, not a harmless patch upgrade:

- 0.7.0 source-chain capability grants are not readable as 0.7.1 grants; re-issue fresh host grants and private claims.
- DirectSignal calls require an explicit `Capability::DirectSignal` grant in 0.7.1.
- 0.7.0 and 0.7.1 direct-signal wire formats are incompatible; mixed-version signalling is unsupported.
- 0.7.1 AppDirect carries sender identity. Bind received notifications to transport provenance and authoritative records; do not trust sender-asserted event payloads.

## Migration sequence

1. Migrate the canonical monorepo source before relying on the standalone export; prove exporter/source parity so the normal sync cannot overwrite the 0.7.1 work.
2. Use the exact 0.7.1 tuple above, including the Pulse-local Rust 1.96.1 toolchain and a regenerated Cargo.lock. Never hand-edit a generated lock.
3. Migrate every active integrity zome's validation dispatcher and typed validation helpers to the 0.7 ActionData, TypedAction and FlatOp forms.
4. Migrate coordinator action/signal decoding and every SignedActionHashed consumer to the header/data model.
5. Update Rust/JavaScript clients, signal decoding, capability-grant re-issuance and DirectSignal grants; test negative controls for wrong caller, wrong secret, wrong scope and stale 0.7.0 claims.
6. Update Nix/CLI locks to an exact release-pinned Holochain source and explicitly separate the Kitsune2 library dependency from the Holonix bootstrap service input. Record the complete flake.lock and binary digests.
7. Build all WASM zomes, pack the DNA/hApp, install into a fresh 0.7.1 conductor, and run the multi-agent Sweettest corpus.
8. Run independent Rust and JavaScript client E2E. Qualify grant/revoke, DirectSignal, restore-from-DHT and the AwaitingRestore/enable_app boundary.
9. Run restart, retry, export/import reconciliation, and adversarial network histories. Keep every runtime receipt bound to the exact source SHA, toolchain, lockfiles, WASM/DNA hashes, conductor identities and test traces.

## Security invariants retained during migration

- A successful 0.7.1 compile does not prove runtime authorization.
- A deleted Holochain host capability grant must cause a retained claim to fail at the conductor.
- An application-level `revoked` flag or DHT projection never substitutes for host-level grant deletion.
- The authorization probe is empty-response and must not fetch or transmit inbox entries merely to test a grant.
- 0.7.0 grant secrets are never imported or reused; issue fresh 0.7.1 CapGrants and private claims.
- Missing DHT frontiers, validators or observers remain UNKNOWN/INCOMPLETE; they are never interpreted as an empty inbox or as successful authorization.
- Unrestricted remote signals are untrusted wake-up hints until their backing records and provenance are verified.
- The restored public source chain does not recreate private entries, CapSecrets, private CapClaims or host capabilities; fresh capability grants are required after restore.
- No /chat activation or second durable Chat store is part of the substrate version migration.

## Current branch boundary

This branch is the source-level 0.7.1 evaluation candidate. It now pins a local Rust 1.96.1 toolchain to match the Holochain 0.7.1 source tag, and its qualification workflow uses the same version. The checked-in Cargo.lock was still resolving HDK 0.7.0 / HDI 0.8.0 when inspected; the existing exact-branch lock-regeneration job must regenerate it from the 0.7.1 manifests before any --locked build can pass.

The 0.7.1 action-model conversion, lock closure, WASM/DNA packaging, fresh-conductor install, multi-conductor runtime corpus and independent client E2E remain blocking. Do not promote this branch until those exact-head gates have actually completed. The separate 0.7.0 PR #4891 remains a migration intermediate, not evidence of 0.7.1 qualification.
