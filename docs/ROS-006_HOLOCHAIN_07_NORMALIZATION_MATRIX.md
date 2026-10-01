# Holochain 0.7 normalization matrix

Status: migration planning artifact; no 0.7 compatibility claim  
Audit date: 2026-10-01  
Branch: `feat/ros-006-qualification-boundary-v2`

## Purpose

Detailed source-level inventory: [`ROS-006_HOLOCHAIN_07_SOURCE_MIGRATION_INVENTORY.md`](ROS-006_HOLOCHAIN_07_SOURCE_MIGRATION_INVENTORY.md).

This matrix turns the repository's confirmed 0.6-era Holochain surface into an explicit migration contract. It is intentionally separate from ROS-006 qualification semantics.

Holochain's official 0.7 compatibility generation is:

| Component | 0.7 target |
| --- | --- |
| Holochain / conductor | 0.7.0 |
| HDK | 0.7.0 |
| HDI | 0.8.0 |
| Rust client | 0.9.0 |
| JavaScript client | 0.21.0 |
| hc | 0.7.0 |
| hc-scaffold | 0.700.0 |
| hc-spin | 0.700.0 |
| Lair | 0.7.1 |
| Kitsune2 bootstrap | 0.5.0 |

Source: Holochain 0.7 compatibility guidance.

## Active workspace dependency matrix

| Workspace | Current declaration | 0.7 target | Migration state |
| --- | --- | --- | --- |
| `mycelix-workspace/Cargo.toml` | hdk 0.6.1; hdi 0.7.1; holochain 0.6.1 | hdk 0.7.0; hdi 0.8.0; holochain 0.7.0 | pending |
| `mycelix-workspace/Cargo.toml` | holochain_client 0.8.1 | Rust client 0.9.0 | pending |
| `mycelix-workspace/Cargo.toml` | holochain_types/zome_types/integrity_types/holo_hash/state/p2p/keystore/sqlite 0.6.1 | resolve from coherent 0.7 graph; add direct pins only when source requires them | pending |
| `mycelix-workspace/Cargo.toml` | kitsune2 0.4.1; lair_keystore 0.6.3 | coherent 0.7 generation; Lair 0.7.1 | pending |
| `mycelix-civic/Cargo.toml` | hdk 0.6.1; hdi 0.7.1; zome/integrity types 0.6.1 | hdk 0.7.0; hdi 0.8.0; corresponding 0.7 types | pending |
| `mycelix-commons/Cargo.toml` | hdk 0.6.0; hdi 0.7.0; integrity types 0.6.0; holo_hash 0.6 | hdk 0.7.0; hdi 0.8.0; corresponding 0.7 types | pending |

### Serialization pin

The repository currently uses `holochain_serialized_bytes = 0.0.57` in the inspected Holochain workspaces. Holochain's 0.7 guide specifies 0.0.57 when an explicit pin is retained, so this is not a migration blocker by itself.

## Nix / Holonix matrix

Current `mycelix-workspace/flake.nix` points directly at:

- Holonix ref `d21b3543`
- comments refer to a non-present `nix/modules/holochain-versions.nix`

Current committed `flake.lock` resolves:

- Holochain original ref `holochain-0.6.0`
- hc-scaffold original ref `0.600.0-dev.0`
- Lair original ref `v0.6.3`
- Holonix revision `d21b35431e425e615bc05da790987380a84b8280`
- Kitsune2 original ref `v0.3.2`

Required outcome:

1. move the flake input to the repository's chosen 0.7 Holonix generation;
2. regenerate `flake.lock` in an actual 0.7 environment;
3. verify the resulting conductor, CLI, scaffold, Lair and bootstrap components against the compatibility matrix;
4. remove stale version comments rather than creating a second source of truth.

## Source/API migration matrix

| 0.6-shaped surface | 0.7 target | Semantic risk | Required evidence |
| --- | --- | --- | --- |
| `FlatOp::StoreEntry` | `FlatOp::CreateEntry` | validation dispatch | compiler + behavior |
| `FlatOp::StoreRecord` | `FlatOp::CreateRecord` | validation dispatch | compiler + behavior |
| `FlatOp::RegisterUpdate` | `FlatOp::Update` | author/update semantics | compiler + behavior |
| `FlatOp::RegisterDelete` | `FlatOp::Delete(OpDelete { action })` | delete semantics | compiler + behavior |
| `FlatOp::RegisterCreateLink` | `FlatOp::Link(OpLink::CreateLink { .. })` | link authorization | compiler + behavior |
| `FlatOp::RegisterDeleteLink` | `FlatOp::Link(OpLink::DeleteLink { .. })` | link authorization | compiler + behavior |
| `FlatOp::RegisterAgentActivity` | `FlatOp::AgentActivity` | chain validation | compiler + behavior |
| `EntryCreationAction::Create` | `TypedAction<EntryCreationData>` | action identity/entry semantics | compiler + behavior |
| `action.author` | `action.author()` | actor binding | behavior |
| `action.timestamp` | `action.timestamp()` | temporal semantics | behavior |
| crate-root re-exports | prelude / 0.7 module paths | type identity/imports | compiler |
| `must_get_agent_activity` exhaustive match | include 0.7 indeterminate/incomplete variants | deterministic validation | compiler + behavior |
| `block_agent` / `unblock_agent` | remove application calls | authority model | behavior/design review |
| tx5/WebRTC configuration | iroh/QUIC | network operation | host evidence |
| `signal_url` / `webrtc_config` | remove obsolete config | conductor startup | host evidence |

## First compiler fixture

The first source migration target remains:

`mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs`

Reason:

- it is a concrete bridge boundary;
- its dispatcher contains several representative 0.6 `FlatOp` variants;
- it contains explicit author-binding checks;
- its existing tests largely exercise application validation semantics that should survive the representation migration.

The migration must preserve author checks and entry semantics. It must not be reduced to mechanical enum renaming.

## Excluded test workspaces

The unified workspace contains numerous deliberately excluded standalone test workspaces. Existing comments identify several with loose or incompatible Holochain requirements.

These exclusions are **migration inventory**, not proof of compatibility and not permission to delete the exclusions.

Each excluded workspace needs one of:

- migrate to the same 0.7 generation and re-enter the unified graph;
- remain standalone with an explicit supported generation;
- archive/deprecate with evidence if genuinely obsolete.

No exclusion should be silently removed during normalization.

## Lockfile and DNA gates

A 0.7 normalization is not complete when manifests merely say 0.7.

Required gates:

### Gate A — dependency coherence

All active Holochain workspaces resolve one compatible 0.7 generation. Cargo and Nix lockfiles are regenerated from that environment.

### Gate B — compiler migration

Representative integrity and coordinator zomes compile against the new graph. Compiler diagnostics drive the remaining action/re-export migration.

### Gate C — behavioral preservation

Existing application-level validation tests pass. New tests cover migrated action dispatch, author binding, agent-activity variants, and deterministic failure modes.

### Gate D — host evidence

The actual zome/test harness executes under Holochain 0.7.

### Gate E — network identity

The resulting DNA hashes are recorded. Holochain documents that integrity-zome dependency updates break DNA compatibility, so the 0.7 generation must be treated as a distinct network/DHT identity rather than an invisible in-place upgrade.

## ROS-006 boundary

ROS-006 core-types must remain Holochain-version-neutral.

The intended flow remains:

transport → observed dependency → qualification → certificate → Relationship 360 projection

The concrete Holochain adapter belongs after the host workspace is normalized. It must produce observations from actual retrieved records rather than copying requested dependencies.

## Evidence status

- 0.7 compatibility target: confirmed from official Holochain guidance.
- Current active dependency generation: confirmed 0.6-era/mixed from branch manifests.
- Current Nix generation: confirmed 0.6-era from committed lockfile.
- API migration: not yet executed.
- 0.7 lockfiles: not yet regenerated.
- 0.7 compiler evidence: not yet available.
- 0.7 host/conductor evidence: not yet available.
- ROS-006 core qualification hardening: independent of this migration.

## References

- https://developer.holochain.org/resources/compatibility/holochain-0.7/
- https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- https://developer.holochain.org/resources/compatibility/
