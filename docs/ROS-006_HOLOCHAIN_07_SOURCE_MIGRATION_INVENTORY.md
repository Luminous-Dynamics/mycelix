# Holochain 0.7 source migration inventory

Status: source-level migration inventory; **not** a 0.7 compatibility claim  
Audit date: 2026-10-01  
Branch: `feat/ros-006-qualification-boundary-v2`

## Purpose

This inventory converts the previously identified 0.6-era Holochain API hotspots into concrete source locations and migration units. It is deliberately separate from ROS-006 qualification semantics.

The official Holochain 0.6 → 0.7 guide identifies the action-model rewrite as the bulk of the upgrade work: `Action` becomes a header plus `ActionData`, `FlatOp` variants are renamed/restructured, and `EntryCreationAction` becomes `TypedAction<EntryCreationData>`.

## Inventory

| Priority | File | Hotspots | Main 0.7 work |
| --- | --- | --- | --- |
| P0 | `mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs` | `FlatOp::StoreEntry` L77/L87/L94; `RegisterUpdate` L95; `RegisterDeleteLink` L110; `RegisterDelete` L117 | Convert dispatcher to 0.7 `FlatOp`; preserve explicit author/link-author checks |
| P1 | `mycelix-civic/zomes/emergency-incidents/integrity/src/lib.rs` | `StoreEntry` L155; `RegisterCreateLink` L175; `RegisterDeleteLink` L235; `StoreRecord` L268; `RegisterAgentActivity` L269; `RegisterUpdate` L270 | Migrate all dispatcher variants; review agent-activity determinism and typed-action access |
| P1 | `mycelix-commons/zomes/property-registry/integrity/src/lib.rs` | `StoreEntry` L143; `RegisterCreateLink` L169; `RegisterDeleteLink` L216; `StoreRecord` L269; `RegisterAgentActivity` L270; `RegisterUpdate` L271; `EntryCreationAction::Create` L146/L149 | Migrate dispatcher plus typed create-action signatures without changing property semantics |
| P1 | `mycelix-commons/zomes/property-transfer/integrity/src/lib.rs` | `StoreEntry` L109; `RegisterCreateLink` L133; `RegisterDeleteLink` L167; `StoreRecord` L174; `RegisterAgentActivity` L175; `RegisterUpdate` L176; `EntryCreationAction::Create` L112/L115 | Same typed-action/FlatOp migration; preserve transfer and escrow validation |
| P1 | `mycelix-commons/zomes/housing-membership/integrity/src/lib.rs` | `StoreEntry` L149; `RegisterCreateLink` L183; `RegisterDeleteLink` L247; `StoreRecord` L254; `RegisterAgentActivity` L255; `RegisterUpdate` L256 | Same dispatcher migration; preserve membership/application invariants |
| P2 | `mycelix-civic/zomes/justice-cases/integrity/src/lib.rs` | `StoreEntry` L768/L846; `RegisterCreateLink` L856; `RegisterDeleteLink` L951; `RegisterUpdate` L980; `RegisterDelete` L972 | Larger validation surface; migrate after representative fixtures establish patterns |

Line numbers are anchors into the audited branch and may move as migration commits land.

## Canonical transformation map

| Current 0.6-shaped source | Holochain 0.7 target |
| --- | --- |
| `FlatOp::StoreEntry` | `FlatOp::CreateEntry` |
| `FlatOp::StoreRecord` | `FlatOp::CreateRecord` |
| `FlatOp::RegisterUpdate` | `FlatOp::Update` |
| `FlatOp::RegisterDelete` | `FlatOp::Delete(OpDelete { action })` |
| `FlatOp::RegisterCreateLink` | `FlatOp::Link(OpLink::CreateLink { .. })` |
| `FlatOp::RegisterDeleteLink` | `FlatOp::Link(OpLink::DeleteLink { .. })` |
| `FlatOp::RegisterAgentActivity` | `FlatOp::AgentActivity` |
| `EntryCreationAction::Create(action)` | `TypedAction<EntryCreationData>` |
| `action.author` | `action.author()` |
| `action.timestamp` | `action.timestamp()` |
| old crate-root re-exports | 0.7 prelude/module paths |

## Migration ordering

### 1. Civic Bridge first

Use `civic-bridge/integrity` as the compiler fixture because it is comparatively compact and contains several representative update/delete authorization paths.

The first migrated fixture must demonstrate:

- create-entry dispatch;
- update dispatch;
- delete dispatch;
- delete-link dispatch;
- exact author binding;
- exact link-author binding;
- unchanged application-level entry validation.

Do not add ROS-006 dependency resolution to this fixture.

### 2. Emergency + Commons small/medium zomes

Apply the proven 0.7 dispatcher pattern to:

- emergency-incidents;
- property-registry;
- property-transfer;
- housing-membership.

These contain the important `StoreRecord` and `RegisterAgentActivity` paths that the smaller civic-bridge fixture does not.

### 3. Justice Cases last

Justice Cases has a substantially larger source/test surface. Treat it as propagation after the 0.7 type/dispatcher pattern has been proven rather than as the first migration experiment.

## Semantic traps

### Author checks

Do not weaken or delete checks merely because the action representation changes. In 0.7, shared author/timestamp fields are accessed through the action/header API. The authorization relationship being checked must remain identical.

### Link deletes

0.7 folds create/delete link validation into `FlatOp::Link(OpLink::...)`. Delete-link validation still needs the original create-link action when checking ownership. Do not replace that relationship with the deleting action's author alone.

### Entry creation

Do not mechanically rename `EntryCreationAction::Create`. The 0.7 type is `TypedAction<EntryCreationData>`; function signatures and conversion sites must be migrated together.

### Agent activity

The 0.7 `must_get_agent_activity` response has additional deterministic failure variants. Exhaustive matches must account for them rather than collapsing them into a generic success/failure interpretation.

### Network/configuration

The source migration is not complete when Rust compiles. Holochain 0.7 removes tx5/WebRTC in favor of iroh/QUIC and removes obsolete conductor fields such as `signal_url` and `webrtc_config`. Host evidence belongs to the dedicated normalization effort.


## Second-pass source findings

A branch-head inspection was performed against `dd04521004d5d416043c5229ed98cecffd24e80f`. This pass confirms that the migration is not limited to the dispatcher names: several integrity zomes also read the pre-0.7 action fields directly.

### Confirmed direct-field hotspots

| File | Confirmed 0.6-shaped access | Migration implication |
| --- | --- | --- |
| `mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs` | `action.author` in update/delete/link-delete checks; `original.action().author()` retained | Move only the current action access to `action.author()`; preserve both sides of the authorization comparison |
| `mycelix-civic/zomes/emergency-incidents/integrity/src/lib.rs` | `action.author` in link/delete/application checks; `update.author` is an application entry field, not an Holochain action field | Do not mechanically rename every `.author`; distinguish Holochain action metadata from application data |
| `mycelix-commons/zomes/property-registry/integrity/src/lib.rs` | `EntryCreationAction::Create`; direct `action.author`; helper constructors/tests using legacy action types | Migrate production signatures and test fixtures together |
| `mycelix-commons/zomes/property-transfer/integrity/src/lib.rs` | `EntryCreationAction::Create`; direct `action.author`; test helper builds legacy `Create` | Treat tests as part of the API migration, not as post-migration cleanup |
| `mycelix-commons/zomes/housing-membership/integrity/src/lib.rs` | direct `action.author` in link/update/delete/application validation | Preserve the distinction between action author and application-level actor fields |

### Important false-positive guard

The branch contains ordinary application fields named `author` and `update.author`. These are not Holochain 0.6 API uses and must not be rewritten. The migration should therefore be performed by typed/contextual transformation rather than a repository-wide text replacement of `.author`.

### Coordinator boundary

The next source pass should explicitly inventory coordinator `signal_action` implementations before any integrity migration is declared complete. Holochain 0.7 changes these matches from `Action::...` to `ActionData::...` over the action's `data` field. This is a separate migration unit from integrity validation and should be tracked independently so that a successful integrity compile cannot be mistaken for a complete zome migration.

### Source-tree duplication remains a migration risk

The branch audit already established byte-identical workspace roots for Civic and Commons. The API inventory must be applied to the actual source-of-truth path and then mirrored/verified where duplicate trees remain. Do not independently hand-edit duplicate trees: divergence here would create a false sense of migration completeness.

### Recommended next implementation slice

The safest next code slice is now narrower than a repository-wide rewrite:

1. establish the 0.7 dependency graph in the real Nix/Cargo environment;
2. migrate `civic-bridge/integrity` as the compiler fixture;
3. compile and run its existing validation tests;
4. capture the exact transformation pattern, including typed-action conversions and link-delete handling;
5. inventory/migrate coordinator `signal_action` separately;
6. propagate only after the fixture is proven.

This keeps ROS-006's qualification boundary untouched and prevents a broad mechanical rewrite from changing application authorization semantics.


## Exact 0.6→0.7 transformation map for the first compiler fixture

The official Holochain 0.7 guide makes the first fixture's required transformations concrete: `FlatOp::StoreEntry` becomes `FlatOp::CreateEntry`; `RegisterUpdate` becomes `Update`; `RegisterDelete` becomes `Delete`; the two link operations become `FlatOp::Link(OpLink::CreateLink|DeleteLink)`; and typed action payloads replace `EntryCreationAction`/legacy action structs. Common metadata is accessed through `author()`/other accessors. citeturn1search0

### civic-bridge: lowest-risk first fixture

The current dispatcher in `mycelix-civic/zomes/civic-bridge/integrity/src/lib.rs` only performs application-entry validation for create/update and author checks for update/delete/link-delete. That makes it a particularly clean compiler fixture: its application semantics can remain byte-for-byte conceptually identical while only the Holochain dispatch representation changes.

Required transformations, once the 0.7 dependency graph is actually available:

- `FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action })` → `FlatOp::CreateEntry(OpEntry::CreateEntry { app_entry, action })` and pass `action.into()` where the validator expects `TypedAction<EntryCreationData>`.
- `FlatOp::StoreEntry(OpEntry::UpdateEntry { ... })` → `FlatOp::Update(OpUpdate::Entry { ... })` / corresponding typed operation shape supplied by HDI 0.8.
- `FlatOp::RegisterUpdate(update)` → `FlatOp::Update(update)`.
- `FlatOp::RegisterDeleteLink { ... }` → `FlatOp::Link(OpLink::DeleteLink { ... })` and use the typed delete-link action's `link_add_address` / author accessor.
- `FlatOp::RegisterDelete(OpDelete { action, .. })` → `FlatOp::Delete(OpDelete { action, .. })`.
- `action.author` → `action.author()` for every Holochain `TypedAction`/action value. Do not touch application structs merely because they contain an `author` field.
- Preserve the existing comparison against `original.action().author()`. This is part of the authorization invariant and is not an incidental API rewrite.

### property-registry / property-transfer: additional typed-action work

These two zomes require more than dispatcher renaming because their create/update helpers explicitly accept `EntryCreationAction`, their test helpers construct `EntryCreationAction::Create`, and they use fields such as `original_action_hash` that the 0.7 guide maps to action-level accessors. Holochain explicitly documents `TypedAction<EntryCreationData>` as the replacement and notes that update/delete/link operations expose their original-action addresses through the typed action. citeturn1search0

Therefore these zomes should follow the proven civic-bridge fixture rather than being migrated independently by search/replace.

### Evidence rule

No source file is being claimed as 0.7-compatible until the real 0.7 HDK/HDI dependency graph compiles it. The official compatibility table currently identifies Holochain/HDK 0.7.0 and HDI 0.8.0 as the compatible release line. citeturn1search2

## Evidence gates

For each migration unit, record evidence separately:

1. **Dependency gate** — coherent Holochain 0.7 Cargo/Nix graph.
2. **Compiler gate** — migrated integrity/coordinator code compiles.
3. **Behavior gate** — existing application semantics remain intact.
4. **Adversarial gate** — author, link-delete, malformed, and deterministic-failure paths are covered.
5. **Host gate** — actual 0.7 zome/test harness executes.
6. **Network gate** — resulting DNA identity/network break is recorded.

Until these gates are satisfied, the repository must continue to describe the work as migration-in-progress rather than Holochain-0.7 compatible.

## Relationship to ROS-006

ROS-006 remains Holochain-version-neutral:

`transport → observed dependency → qualification → certificate → projection`

The Holochain adapter should consume this normalized host boundary later. It must construct observations from records actually retrieved and validated by the host adapter; it must not use `QualificationDependencyObservation::from_dependency` as production evidence.

## Evidence limitations

This document is a source audit. It does **not** claim:

- that the listed zomes compile against Holochain 0.7;
- that Cargo or Nix locks have been regenerated;
- that a 0.7 conductor has executed these zomes;
- that all 0.6-shaped occurrences in the repository have been exhaustively enumerated.

Those claims require actual 0.7 environment evidence.

## References

- Holochain 0.7 compatibility: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Holochain 0.6 → 0.7 upgrade guide: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Existing normalization matrix: `docs/ROS-006_HOLOCHAIN_07_NORMALIZATION_MATRIX.md`
- Existing pin audit: `docs/ROS-006_HOLOCHAIN_PIN_AUDIT.md`
