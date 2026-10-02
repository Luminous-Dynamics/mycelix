# ROS-006 — Holochain 0.7 Cargo Normalization Gate

**Status:** staged; no 0.7 Cargo build is claimed  
**Branch:** `feat/ros-006-qualification-boundary-v2`

## Purpose

This gate separates the Holochain 0.7 dependency changes that are directly prescribed by the upstream upgrade guide from repository-local runtime dependencies that must be proven by compilation before they are changed.

The repository currently has two coupled Cargo surfaces:

1. `mycelix-workspace/Cargo.toml` — the unified workspace.
2. Standalone `mycelix-civic/Cargo.toml` and `mycelix-commons/Cargo.toml` — independently resolvable hApp workspaces.

The migration must make all three surfaces converge on one Holochain 0.7 generation. A manifest edit is not evidence of compatibility; the 0.7 compiler and generated lock graph are the evidence gates.

## Verified current state

The unified workspace currently declares these Holochain-era direct dependencies:

| Dependency | Current declaration | Migration classification |
| --- | --- | --- |
| `hdk` | `=0.6.1` | **application API — update to 0.7.0** |
| `hdi` | `=0.7.1` | **application API — update to 0.8.0** |
| `holochain` | `=0.6.1` | **runtime/test API — update only with feature/source audit** |
| `holochain_client` | `=0.8.1` | **client API — target 0.9.0 if actually required by 0.7 source** |
| `holochain_types` | `=0.6.1` | **runtime/internal API — compiler-gated** |
| `holochain_zome_types` | `=0.6.1` | **source API — compiler-gated; action model changes apply** |
| `holochain_serialized_bytes` | `=0.0.57` | **already at 0.7-compatible target** |
| `holo_hash` | `=0.6.1` | **source API — compiler-gated** |
| `holochain_integrity_types` | `=0.6.1` | **source API — compiler-gated** |
| `holochain_state` | `=0.6.1` | **runtime/internal API — compiler-gated** |
| `holochain_p2p` | `=0.6.1` | **runtime/internal API — compiler-gated** |
| `holochain_keystore` | `=0.6.1` | **runtime/internal API — compiler-gated** |
| `holochain_sqlite` | `=0.6.1` | **runtime/internal API — likely obsolete in 0.7 graph; do not hand-replace** |
| `holochain_wasmer_host` | `=0.0.102` | **runtime/internal API — compiler/lockfile-gated** |
| `kitsune2` | `=0.4.1` | **runtime/internal API — target 0.5 generation through regenerated graph** |
| `lair_keystore` | `=0.6.3` | **runtime/internal API — target 0.7.1 through regenerated graph** |

Standalone Civic currently declares `hdk = =0.6.1`, `hdi = =0.7.1`, plus explicit 0.6-era zome/hash crates. Standalone Commons declares `hdk = 0.6.0`, `hdi = 0.7.0`, `holochain_integrity_types = 0.6.0`, and `holo_hash = 0.6` with the `hashing` feature.

## Upstream 0.7 target

The official Holochain 0.6 → 0.7 guide prescribes:

- `hdk = =0.7.0`
- `hdi = =0.8.0`
- explicit `holochain_serialized_bytes` at `0.0.57`
- Holochain 0.7's rewritten action model
- iroh/QUIC as the network transport
- no 0.6 → 0.7 database migration

The official compatibility table lists Holochain 0.7.0, HDK 0.7.0, HDI 0.8.0, Kitsune2 0.5.0, Lair 0.7.1, and Rust client 0.9.0 as the compatible generation.

## Normalization order

### Gate C0 — Preserve the current 0.6 evidence

Do not rewrite or delete existing 0.6 lockfiles/fixtures merely to make the tree appear migrated. Existing 0.6 fixtures remain useful as negative/legacy evidence until the 0.7 path is independently proven.

### Gate C1 — Update application-facing HDK/HDI declarations

Update the root dependency authority for:

- `hdk` → `=0.7.0`
- `hdi` → `=0.8.0`

Apply the same target to the standalone Civic and Commons dependency authorities rather than allowing three different HDK/HDI generations.

### Gate C2 — Resolve the runtime dependency surface

Only after C1, regenerate the Cargo graph in the intended 0.7 environment.

Do **not** hand-bump `holochain_state`, `holochain_sqlite`, `holochain_p2p`, `holochain_keystore`, `kitsune2`, or `lair_keystore` one-by-one. Holochain 0.7 changed the runtime/storage graph, and the official upgrade notes explicitly describe removal/replacement of several 0.6-era surfaces.

The regenerated graph is authoritative for transitive runtime versions.

### Gate C3 — Compile a representative integrity zome

Use Civic Bridge as the first compiler fixture because it is already the designated lowest-risk migration surface.

Required evidence:

- real 0.7 HDK/HDI resolution,
- successful WASM compilation,
- successful existing validation tests where executable,
- captured compiler errors/transforms for the action model.

No source file is marked 0.7-compatible merely because textual substitutions were made.

### Gate C4 — Propagate proven source transformations

Once Civic Bridge compiles, apply the proven 0.7 action-model transformations to the remaining integrity zomes and then coordinator `signal_action` implementations.

Known transformation families include:

- `FlatOp::StoreEntry` → `FlatOp::CreateEntry`
- `FlatOp::StoreRecord` → `FlatOp::CreateRecord`
- `FlatOp::RegisterUpdate` → `FlatOp::Update`
- `FlatOp::RegisterDelete` → `FlatOp::Delete(OpDelete { action })`
- link registration variants → `FlatOp::Link(OpLink::...)`
- `EntryCreationAction` → `TypedAction<EntryCreationData>`
- generic action fields → typed-action/header accessors as required by the compiler

### Gate C5 — Runtime/conductor validation

Only after Cargo compilation succeeds:

1. regenerate/validate the Nix lock graph;
2. enter the 0.7 development environment;
3. validate conductor configuration against iroh/QUIC;
4. use a fresh conductor data root;
5. run isolated zome/sweettest validation;
6. run multi-conductor/network validation;
7. record resulting DNA hashes and network identity.

Holochain 0.7 cannot consume the 0.6 database format, and the DNA hash changes across the upgrade. Therefore an existing 0.6 data root must never silently become the 0.7 test root.

## Stop conditions

Stop the migration rather than papering over failures if:

- a generated lock graph still resolves Holochain 0.6 runtime crates in the 0.7 target;
- a standalone workspace resolves a different HDK/HDI generation;
- an internal Holochain crate is missing after regeneration and source usage has not been audited;
- a zome only passes because a stale 0.6 lockfile is being reused;
- a conductor test reuses a pre-existing 0.6 data root;
- a source transformation changes ordinary application fields merely because they are named `author` or `update`.

## Evidence rule

The migration is complete only when the following independent evidence exists:

1. Nix lock graph is regenerated from the 0.7 Holonix input.
2. Unified Cargo graph resolves the intended 0.7 generation.
3. Standalone Civic Cargo graph resolves the same generation.
4. Standalone Commons Cargo graph resolves the same generation.
5. Representative integrity zome compiles under the real 0.7 HDK/HDI.
6. Remaining action-model hotspots compile and pass their executable tests.
7. 0.7 conductor starts with a fresh data root.
8. Network/DNA evidence is captured separately from source migration evidence.

This keeps ROS-006 qualification evidence separate from the underlying Holochain runtime migration and prevents a green textual diff from being mistaken for a proven 0.7 upgrade.
