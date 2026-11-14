# Pulse Holochain 0.6 → 0.7 Migration

**Status: active migration; not qualified or merge-ready until the 0.7 compile and runtime gates pass.**

## Target matrix

Use one coherent Holochain 0.7 generation across the Pulse Rust workspace and its Sweettest harness:

- conductor / `holochain`: `0.7.0`
- `hc` CLI: `0.7.0`
- `hdk`: `0.7.0`
- `hdi`: `0.8.0`
- `holochain_integrity_types`, `holochain_zome_types`, `holo_hash`, `hdk_derive`: `0.7.0`
- `holochain_serialized_bytes`: `0.0.57`
- Lair: `0.7.1`
- JavaScript client, where used: `@holochain/client 0.21.0`
- Node.js development/runtime tooling: `24` (per the official 0.7 Holonix migration example)

Primary migration source: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/

## Why a pin-only upgrade is invalid

Holochain 0.7 changes the integrity action model. `Action` now carries `header` plus `data`; payload variants are `ActionData::*`; shared header access uses `author()`, `timestamp()`, `action_seq()`, and `prev_action()`. The validation dispatcher also changes from `FlatOp::StoreEntry` / `RegisterUpdate` / `RegisterCreateLink` to `FlatOp::CreateEntry` / `Update` / `Link(OpLink::CreateLink { .. })` and related variants. This touches every integrity zome, not just the versions in Cargo manifests.

Sweettest and packaging also need separate attention. Holochain 0.7 removes tx5/WebRTC; Iroh/QUIC is the transport. Its databases and DNA hashes are not compatible with 0.6, so existing conductor data cannot be migrated in place: upgrading requires a clean 0.7 data root and a fresh installation/network identity, or a separately designed application-level export/import process.

## Migration sequence

1. Pin workspace and standalone Sweettest dependencies to the same 0.7 generation; eliminate coordinator-local `hdk = "0.6"` drift.
2. Migrate every active integrity zome's `validate` dispatcher and typed validation helpers to the 0.7 `ActionData`, `TypedAction`, and `FlatOp` forms.
3. Migrate coordinator action/signal decoding and every `SignedActionHashed` consumer to the `header`/`data` model.
4. Update the Nix/CLI lock to a release-pinned Holochain 0.7 toolchain; verify no conductor config still contains `signal_url`, `webrtc_config`, or WebRTC transport assumptions.
5. Update Sweettest constructors to `SweetConductor::standard()` / `SweetConductorBatch::standard(n)`; regenerate Cargo lockfiles with the actual 0.7 toolchain, then run format, workspace compile/tests, packed DNA checks, and multi-conductor Sweettests.
6. Run a fresh-conductor two-agent corpus covering capability grant/revocation and V2 qualification. Keep /chat promotion fail-closed until the protocol-level delivery-completeness witness is independently proved.

## Security invariants retained during migration

- A successful 0.7 compile does not prove runtime authorization.
- A deleted Holochain capability grant must cause the same remote capability call to fail as `Unauthorized`.
- The authorization probe uses `mail_messages.capability_probe_v1`, an empty-response endpoint; it must not fetch or transmit inbox entries merely to test the grant.
- No application-level `revoked` flag substitutes for conductor-level revocation.
- No local inbox enumeration, ACK, receipt, or digest substitutes for an authoritative delivery frontier.
- A missing frontier or unavailable entitled sender remains incomplete; it is never interpreted as an empty inbox.
- No `/chat` activation or second durable Chat store is part of the version migration.

## Current branch boundary

This branch begins the version/dependency standardization and adds an explicit compile gate. The 0.7 action-model conversion, regenerated lockfiles, and conductor runtime corpus remain blocking work until independently executed and passing. Do not merge this branch while those gates are failing or absent.
