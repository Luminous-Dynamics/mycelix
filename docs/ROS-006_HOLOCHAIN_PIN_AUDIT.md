# ROS-006 Holochain dependency pin audit

Audit date: 2026-10-01  
Scope: repository manifests, the committed `mycelix-workspace/flake.lock`, and the committed `mycelix-civic/Cargo.lock` inspected on branch `feat/ros-006-qualification-boundary-v2`.  
Purpose: establish the Holochain 0.7 target without performing an unsafe partial dependency migration inside ROS-006.

Detailed execution matrix: [`ROS-006_HOLOCHAIN_07_NORMALIZATION_MATRIX.md`](ROS-006_HOLOCHAIN_07_NORMALIZATION_MATRIX.md).

## Current state

The repository is **not yet Holochain-0.7 normalized**. The inspected active workspaces still contain a mixture of 0.6-era Holochain dependencies, and the workspace Nix lock is explicitly resolving a Holochain 0.6 generation.

| Location | Current declarations | Required 0.7 target |
| --- | --- | --- |
| mycelix-workspace/Cargo.toml | HDK 0.6.1, HDI 0.7.1, Holochain/types family 0.6.1 | HDK 0.7.0, HDI 0.8.0, Holochain 0.7.0 family |
| mycelix-civic/Cargo.toml | HDK 0.6.1, HDI 0.7.1, zome/integrity types 0.6.1 | HDK 0.7.0, HDI 0.8.0, corresponding 0.7 zome/integrity types |
| mycelix-commons/Cargo.toml | HDK 0.6.0, HDI 0.7.0, integrity types 0.6.0 | HDK 0.7.0, HDI 0.8.0, corresponding 0.7 zome/integrity types |
| crates/mycelix-core-types/Cargo.toml | optional HDI 0.7.1 plus 0.6-era Holochain type dependencies | optional 0.7-generation host integration; default core remains Holochain-independent |
| mycelix-workspace/flake.lock | Holochain source `holochain-0.6.0`; Holonix commit `d21b3543`; Lair `v0.6.3`; hc-scaffold original ref `0.600.0-dev.0` | coherent 0.7-compatible Holonix/tooling lock |

This is a migration inventory, not evidence of a compile failure. The important finding is that the repository's current dependency generation does not match the intended Holochain 0.7 baseline.

## Nix pin integrity finding

The workspace `flake.nix` comments say the Holonix version is pinned in `nix/modules/holochain-versions.nix`, but that file is not present at the inspected branch path. The actual `flake.nix` directly pins Holonix to commit `d21b3543`.

More importantly, the committed `flake.lock` resolves:

- `holochain` original ref: `holochain-0.6.0`;
- `hc-scaffold` original ref: `0.600.0-dev.0`;
- `lair-keystore` original ref: `v0.6.3`;
- Holonix locked revision: `d21b35431e425e615bc05da790987380a84b8280`.

Therefore the current Nix development environment is not merely undocumented or ambiguous: its committed lock graph is concretely anchored to the 0.6 generation.

This should be corrected as part of the dedicated Holochain-0.7 normalization rather than by changing only the prose comment or only the Rust Cargo manifests.

## Canonical 0.7 compatibility target

Holochain's current 0.7 compatibility table recommends:

- Holochain/conductor: 0.7.0
- HDK: 0.7.0
- HDI: 0.8.0
- JavaScript client: 0.21.0
- Rust client: 0.9.0
- hc CLI: 0.7.0
- hc-scaffold: 0.700.0
- hc-spin: 0.700.0
- Lair: 0.7.1

For this repository, "Holochain 0.7 everywhere" means that every **active Holochain application workspace** uses one coherent 0.7 compatibility generation. It does not mean unrelated Rust dependencies should be changed merely because they are in the same repository.

## Migration hazards that must be handled together

The official 0.6 → 0.7 guide identifies several breaking changes:

1. The action model changed. Variant action structs were replaced by a header plus typed data payload. Integrity validation and coordinator signal_action code must be migrated.
2. `holochain_zome_types` and `holochain_integrity_types` re-exports changed; imports that bypass the preludes need review.
3. `must_get_agent_activity` gained additional deterministic-response variants.
4. `block_agent` and `unblock_agent` were removed.
5. tx5/WebRTC transport was removed; iroh/QUIC is the network transport.
6. Conductor configuration fields changed, including removal of `signal_url` and `webrtc_config`.
7. Sweettest/Holochain feature names changed: `sqlite-encrypted` → `encryption`, `wasmer_sys` → `wasmer-sys-cranelift`, and `transport-iroh` is removed.
8. The official guide requires `cargo update` after dependency changes so the lockfile resolves the complete compatible graph.
9. Integrity-zome dependency changes alter the DNA hash, so a 0.7 migration is a network/DHT compatibility event rather than a transparent in-place dependency bump.

## Safe execution boundary

Do **not** mechanically replace the version strings in the current manifests and commit the result without regenerating and validating the corresponding lockfiles. That would create a repository state whose dependency declarations advertise 0.7 while its resolved graph and source code may still be 0.6-shaped.

The migration should therefore be executed as a dedicated Holochain-0.7 normalization effort:

1. Update each active workspace's root Holochain dependency matrix together.
2. Update the workspace Nix/Holonix lock graph to the coherent 0.7 toolchain.
3. Regenerate each workspace lockfile in a real Holochain-0.7/Nix environment.
4. Enumerate and migrate 0.6 API usage from compiler diagnostics rather than guessing.
5. Update conductor/sandbox configuration and test harnesses.
6. Run workspace-specific build/test evidence, including zome host tests where applicable.
7. Record the resulting dependency graph and DNA/network break explicitly.
8. Only then make the 0.7 pins the repository-wide default.

ROS-006 remains correctly Holochain-independent at its qualification core. Its eventual adapter should consume the already-normalized 0.7 host workspace rather than introducing a special dependency island.

## Relationship to ROS-006

The ROS-006 qualification hardening already enforces an important boundary independent of the Holochain migration:

transport → observation → qualification → projection

The adapter is responsible for producing an explicit dependency observation. The core qualification layer verifies the observation against the requested logical identity, dependency kind, and exact record address. A bare legacy `Valid` result is rejected as unattested.

This lets the ROS-006 safety property advance now without coupling its core semantics to the Holochain migration.


## Additional root-workspace finding

A second pass over the active branch shows that the unified `mycelix-workspace/Cargo.toml` has a substantially larger direct Holochain surface than the three standalone application workspaces alone suggest. Its Holochain section directly pins:

- `hdk = 0.6.1`
- `hdi = 0.7.1`
- `holochain = 0.6.1`
- `holochain_client = 0.8.1`
- `holochain_types = 0.6.1`
- `holochain_zome_types = 0.6.1`
- `holo_hash = 0.6.1`
- `holochain_integrity_types = 0.6.1`
- `holochain_state = 0.6.1`
- `holochain_p2p = 0.6.1`
- `holochain_keystore = 0.6.1`
- `holochain_sqlite = 0.6.1`
- `holochain_wasmer_host = 0.0.102`
- `kitsune2 = 0.4.1`
- `lair_keystore = 0.6.3`

This makes the normalization task materially broader than changing only hdk/hdi. The official 0.7 compatibility table places the client at 0.9.0, Lair at 0.7.1, and the core/conductor at 0.7.0. The remaining internal Holochain crates should be allowed to resolve from the coherent 0.7 graph unless a concrete source-level reason requires a direct pin.

The root workspace also contains comments documenting earlier attempts to keep standalone test workspaces containing loose Holochain 0.6 requirements out of the unified resolution. Those exclusions should be treated as migration inventory, not silently removed: each excluded workspace needs an explicit compatibility decision during the 0.7 normalization.

### Consequence for execution

The next Holochain migration should be treated as a **dependency-graph normalization**, not a version-string patch:

1. establish one 0.7 compatibility matrix for each active Holochain workspace;
2. update the root/unified workspace and standalone civic/commons workspaces coherently;
3. update the Holonix/Nix lock graph;
4. regenerate Cargo locks in the actual 0.7 environment;
5. use compiler diagnostics to enumerate the 0.6 action/re-export/API migration;
6. separately inspect excluded standalone test workspaces for stale 0.6 requirements;
7. record which DNAs receive new hashes and therefore represent new networks.

This is consistent with Holochain's documented 0.6→0.7 process and its warning that integrity dependency changes break DNA compatibility.


## Source-level 0.6 API hotspot scan

A targeted read of the active civic/commons integrity zomes confirms that the migration risk is not limited to dependency declarations. Representative files on this branch still contain the Holochain 0.6 action/validation API that the official 0.7 guide identifies as breaking.

Confirmed examples include:

| Workspace / path | Confirmed 0.6-shaped API | 0.7 migration implication |
| --- | --- | --- |
| `mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs` | `FlatOp::StoreEntry`, `FlatOp::RegisterUpdate`, `FlatOp::RegisterDelete`, `FlatOp::RegisterDeleteLink` | migrate validation callback to 0.7 `FlatOp` variants and typed actions |
| `mycelix-civic/zomes/justice-cases/integrity/src/lib.rs` | `StoreEntry`, `RegisterCreateLink`, `RegisterDeleteLink`, `RegisterUpdate`, `RegisterDelete` | same action/FlatOp migration |
| `mycelix-civic/zomes/emergency-incidents/integrity/src/lib.rs` | `StoreEntry`, `RegisterCreateLink`, `RegisterDeleteLink`, `StoreRecord`, `RegisterAgentActivity`, `RegisterUpdate` | same action/FlatOp migration; agent-activity handling must use the 0.7 API |
| `mycelix-commons/zomes/property-registry/integrity/src/lib.rs` | `StoreEntry`, `RegisterCreateLink`, `RegisterDeleteLink`, `StoreRecord`, `RegisterAgentActivity`, `RegisterUpdate`; `EntryCreationAction::Create` | migrate both FlatOp and `EntryCreationAction` usage |
| `mycelix-commons/zomes/property-transfer/integrity/src/lib.rs` | `StoreEntry`, `RegisterCreateLink`, `RegisterDeleteLink`, `StoreRecord`, `RegisterAgentActivity`, `RegisterUpdate`; `EntryCreationAction::Create` | same action/typed-action migration |
| `mycelix-commons/zomes/housing-membership/integrity/src/lib.rs` | `StoreEntry`, `RegisterCreateLink`, `RegisterDeleteLink`, `StoreRecord`, `RegisterAgentActivity`, `RegisterUpdate` | same action/FlatOp migration |

This scan is deliberately a **source audit, not a build result**. It establishes concrete migration hotspots without claiming that every occurrence in the repository has been enumerated or that any 0.7 build currently succeeds.

The pattern is significant: the active civic and commons integrity zomes are structurally dependent on the 0.6 validation callback model. The official 0.7 upgrade guide says the action-model rewrite is the bulk of the upgrade work and specifically replaces legacy `FlatOp` variants such as `StoreEntry`, `StoreRecord`, and `RegisterUpdate` with 0.7 forms such as `CreateEntry`, `CreateRecord`, and `Update`, with typed actions carrying the relevant payload.

### Migration consequence

The safest next implementation unit is therefore **one representative integrity-zome migration in a dedicated 0.7 normalization branch/workspace**, followed by compiler-driven propagation across the remaining zomes. Do not start changing these validation callbacks inside ROS-006 itself: the ROS-006 core is intentionally Holochain-version-neutral, and the migration needs a coherent HDK/HDI/toolchain environment first.

A particularly useful first migration target is `mycelix-civic/zomes/civic-bridge/integrity`, because it is a bridge boundary and its callback is small enough to serve as a representative compiler/migration fixture before applying the same transformations to the larger civic and commons zomes.


## Representative migration worksheet: civic-bridge integrity

The targeted read of `mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs` makes this a useful first 0.7 compiler fixture.

Current validation dispatcher hotspots:

- `FlatOp::StoreEntry(OpEntry::CreateEntry { ... })`
- `FlatOp::StoreEntry(OpEntry::UpdateEntry { ... })`
- `FlatOp::StoreEntry(_)`
- `FlatOp::RegisterUpdate(update)`
- `FlatOp::RegisterDeleteLink { ... }`
- `FlatOp::RegisterDelete(OpDelete { ... })`

The 0.7 guide gives the corresponding structural transformations:

| Current 0.6-shaped code | 0.7 target shape |
| --- | --- |
| `FlatOp::StoreEntry` | `FlatOp::CreateEntry` |
| `FlatOp::RegisterUpdate` | `FlatOp::Update` |
| `FlatOp::RegisterDeleteLink` | `FlatOp::Link(OpLink::DeleteLink { .. })` |
| `FlatOp::RegisterDelete` | `FlatOp::Delete(OpDelete { action })` |
| `action.author` | `action.author()` |
| `action.timestamp` | `action.timestamp()` |
| variant action structs | `TypedAction<D>` / `ActionData` |

The existing civic-bridge logic also performs explicit author checks on update/delete operations. Those checks should be preserved semantically during migration rather than replaced with a generic "compile fix". The 0.7 action accessors are specifically designed to preserve access to common header fields while the action data is split out.

### Test-preservation requirement

The civic-bridge file currently has a large pure validation test surface for:

- civic-domain allowlisting;
- JSON parameter size and validity boundaries;
- event payload size;
- related-hash cardinality;
- optional result/success fields;
- serialization round-trips;
- type aliases;
- validation wrapper behavior.

The 0.7 migration should therefore be treated as a **behavior-preserving type/API migration**. Those tests should remain unchanged wherever they test entry semantics, while new 0.7-specific tests should cover the migrated dispatcher paths and author-binding behavior.

This is preferable to broad test rewrites: Holochain's 0.7 change is an action representation migration, while these entry-validation invariants are application semantics.

### Do-not-do list

For this fixture, do not:

1. mechanically rename `StoreEntry` to `CreateEntry` without changing the surrounding match types;
2. keep constructing `EntryCreationAction::Create` after migrating to `TypedAction<EntryCreationData>`;
3. recreate old action structs merely to satisfy existing helper signatures;
4. weaken author checks because the accessor locations changed;
5. change application entry validation behavior while performing the API migration;
6. mix the migration with ROS-006 qualification semantics.

The official 0.7 guide explicitly describes `EntryCreationAction` → `TypedAction<EntryCreationData>` and the `FlatOp` renames, making these compiler errors expected migration work rather than reasons to weaken validation.

### Migration gate

This fixture should not be declared migrated until all four evidence layers exist:

1. **dependency evidence** — coherent 0.7 Cargo/Nix graph;
2. **compiler evidence** — civic-bridge integrity builds against that graph;
3. **behavior evidence** — existing validation tests pass without semantic weakening;
4. **host evidence** — the actual zome/test harness runs under the 0.7 conductor/toolchain.

Until then, the repository should continue to label the migration as planned/in-progress rather than implying 0.7 compatibility.

## Evidence status

- Holochain 0.7 target: **confirmed from official compatibility guidance**.
- Current Mycelix Cargo dependency generation: **0.6-era / mixed; confirmed from repository manifests**.
- Current Mycelix Nix dependency generation: **0.6-era; confirmed from committed flake.lock**.
- Holochain 0.7 source/API migration: **not yet executed; concrete 0.6-shaped hotspots confirmed by targeted source inspection**.
- Regenerated 0.7 Cargo/Nix lockfiles: **not yet produced**.
- Full 0.7 workspace compile/test evidence: **not yet available**.
- ROS-006 qualification-core hardening: **implemented on PR head**.

## Sources

- Holochain 0.7 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Holochain 0.6 → 0.7 upgrade guide: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain tooling compatibility guidance: https://developer.holochain.org/resources/compatibility/
