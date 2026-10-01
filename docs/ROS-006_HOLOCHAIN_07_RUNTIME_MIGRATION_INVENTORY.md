# ROS-006 Holochain 0.7 runtime / host migration inventory

Status: pre-migration runtime inventory; **no conductor, network, lockfile, or source mutation performed**
Audit date: 2026-10-01
Branch: `feat/ros-006-qualification-boundary-v2`

## Purpose

The existing 0.7 normalization work has covered dependency roots and integrity-zome source hotspots. This document closes the remaining host-side planning gap: conductor configuration, sandbox lifecycle, network transport, test tooling, and DNA/network identity consequences.

This is intentionally separate from ROS-006 qualification semantics. ROS-006 should consume a normalized host boundary; it should not become the vehicle for an unrelated conductor migration.

## Official 0.7 constraints

Holochain's current 0.7 compatibility table identifies this generation as:

| Component | 0.7 target |
| --- | --- |
| Holochain core / conductor | 0.7.0 |
| HDK | 0.7.0 |
| HDI | 0.8.0 |
| hc | 0.7.0 |
| hc-scaffold | 0.700.0 |
| hc-spin | 0.700.0 |
| Lair | 0.7.1 |
| Kitsune2 bootstrap | 0.5.0 |
| JS client | 0.21.0 |
| Rust client | 0.9.0 |

The 0.6 -> 0.7 guide identifies three host/runtime changes that must not be treated as ordinary Rust API edits:

1. tx5/WebRTC is removed; iroh over QUIC is the network transport.
2. `signal_url` and `webrtc_config` conductor settings are removed.
3. Holochain 0.7 cannot consume 0.6 databases; the upgrade flow requires clearing old conductor data, and DNA hashes change.

## Audited pre-migration graph (2026-10-01)

A source-level audit of the branch confirms that the runtime migration is still a **multi-workspace graph normalization**, not a single dependency edit:

| Workspace | Current HDK | Current HDI | Observation |
| --- | --- | --- | --- |
| `mycelix-civic` | `=0.6.1` | `=0.7.1` | Exact pins; also carries explicit `holochain_zome_types`, `holochain_integrity_types`, `holo_hash`, and `hdk_derive` 0.6-era pins. |
| `mycelix-commons` | `0.6.0` | `0.7.0` | Independent workspace with a different 0.6-era dependency baseline. |
| `mycelix-workspace` | host aggregation | host aggregation | Direct Holonix input; lock regeneration is required before claiming a 0.7 environment. |

This matters because Holochain's 0.7 guide requires HDK 0.7.0 and HDI 0.8.0, and explicitly calls out additional feature/API changes beyond those two version strings.

### Concrete normalization implication

Do **not** update only `mycelix-civic/Cargo.toml` and call the repository migrated. At minimum, the Civic and Commons workspace roots must converge on the 0.7-compatible graph, their lockfiles must be regenerated, and any direct 0.6-era Holochain type pins/features must be compiler-audited.

The official guide specifically calls out these additional 0.7-sensitive items:

- `holochain_serialized_bytes = 0.0.57` when explicitly pinned;
- `holochain_zome_types`, `holochain_integrity_types`, and `holo_hash` feature changes;
- `holochain` Sweettest dependency feature changes;
- `transport-iroh` removal as an explicit feature;
- `wasmer_sys` → `wasmer-sys-cranelift`;
- `sqlite-encrypted` → `encryption`.

These should be discovered from the actual manifests rather than blindly inserted into every workspace.

## Runtime migration surfaces

### R0 — Holonix / Nix graph

**Current evidence:** the audited branch uses a direct Holonix commit pin in `mycelix-workspace/flake.nix`, while an adjacent comment references a missing `nix/modules/holochain-versions.nix`.

**Required action:** select the actual official 0.7 Holonix revision in the real Nix environment, update the flake input, and regenerate `flake.lock`.

**Acceptance evidence:**
- `flake.lock` records the new Holonix graph;
- `nix develop` enters successfully;
- `holochain --version`, `hc --version`, and other required tools report the intended generation.

Do not invent a Holonix commit hash in documentation before the real lock is regenerated.

### R1 — Conductor configuration

Audit all maintained conductor/config fixtures for:

- `signal_url`;
- `webrtc_config`;
- `request_timeout_s` at the old top level;
- `db_sync_strategy`;
- `chc_url`;
- obsolete transport selectors;
- obsolete WASM backend configuration.

0.7-specific changes documented by Holochain include moving `request_timeout_s` under `network`, renaming `db_sync_strategy` to `db_sync_level`, and removing `chc_url`. The 0.7 guide also documents the new optional `wasm_backend` setting.

**Acceptance evidence:** every maintained conductor fixture is parseable by the 0.7 conductor, with no deprecated 0.6 network fields remaining.

### R2 — Sandbox / data lifecycle

0.7 is a clean host/database generation boundary.

The migration procedure must explicitly distinguish:

- disposable local developer sandboxes;
- checked-in fixture databases, if any;
- persistent deployment data;
- newly generated DNA bundles.

For disposable development state, follow the documented `hc sandbox clean` path rather than trying to make 0.7 consume 0.6 databases.

**Acceptance evidence:** a fresh 0.7 sandbox can create/install/enable the migrated app from an empty data root.

### R3 — Network transport

The runtime baseline is iroh over QUIC.

Audit:

- scripts invoking `hc sandbox`;
- explicit `webrtc` network selections;
- relay/bootstrap configuration;
- local relay configuration;
- firewall assumptions;
- test harness environment variables.

The official guide states that `hc sandbox` no longer offers the `webrtc` network type; `mem` and `quic` remain.

If a local iroh relay is intentionally used over an unencrypted connection, the 0.7 configuration requires the documented `relayAllowPlainText` advanced network setting.

**Acceptance evidence:** a two-conductor or equivalent network fixture starts with the intended QUIC/iroh path and exchanges the test app's traffic.

### R4 — Client / test tooling

The target client generation is:

- JS client 0.21.0;
- Rust client 0.9.0;
- hc-spin 0.700.x;
- Holochain core 0.7.0.

Tryorama is a special case: the official compatibility guidance says it is no longer officially supported from the 0.6.1 line, and the 0.7 upgrade guide points older Tryorama users toward the community-maintained `@holochain-open-dev/tryorama` 0.20.0 package.

**Required action:** inventory actual test manifests before editing package versions. Do not assume a root `package.json` or `ui/package.json` exists merely because the generic upgrade guide mentions those paths.

**Acceptance evidence:** the repository's real test harness, whatever its current implementation, can start a 0.7 conductor and exercise at least one migrated zome.

### R5 — DNA identity / network break

Integrity-zome dependency changes alter DNA compatibility. Holochain's compatibility guidance explicitly states that even dependency updates within a compatible SemVer range change the DNA hash when the dependency belongs to an integrity zome.

Therefore the 0.6 -> 0.7 migration must produce an explicit identity ledger containing:

- old DNA hash;
- new DNA hash;
- app/bundle identifier;
- migration commit;
- conductor generation;
- whether the network is intentionally new;
- whether any deployment data requires a separate migration strategy.

Do not describe a 0.7 rebuild as an in-place network upgrade merely because the application semantics are intended to remain equivalent.

### R6 — WASM build/runtime behavior

The 0.7 guide documents that compiled WASM is cached in the database rather than the old `wasm-cache` directory and is loaded when an app is installed or enabled.

Audit scripts and operational documentation for assumptions about:

- `wasm-cache` directories;
- startup-time compilation;
- manual cache cleanup;
- database layout.

This is runtime hygiene, not a reason to alter ROS-006 semantics.

## Execution order

1. **Inventory actual runtime files** — conductor configs, sandbox scripts, package manifests, CI/test harnesses.
2. **Normalize Holonix/Nix** — update the actual 0.7 input and regenerate locks.
3. **Normalize dependency graph** — regenerate Cargo locks in the same 0.7 environment.
4. **Compile the representative civic-bridge integrity zome.**
5. **Migrate remaining integrity/coordinator action APIs using compiler diagnostics.**
6. **Normalize conductor/test harness configuration.**
7. **Run fresh-sandbox host tests.**
8. **Run multi-conductor/network tests.**
9. **Record new DNA hashes and network identity explicitly.**
10. **Only after host evidence exists, implement the concrete Relationship 360 adapter.**

## Evidence gates

| Gate | Evidence required | Status |
| --- | --- | --- |
| R0 | 0.7 Nix/Holonix lock + shell version evidence | Pending |
| R1 | 0.7 conductor configs parse/start | Pending |
| R2 | fresh sandbox lifecycle | Pending |
| R3 | iroh/QUIC runtime path | Pending |
| R4 | real test harness against 0.7 | Pending |
| R5 | old/new DNA identity ledger | Pending |
| R6 | WASM/runtime assumptions audited | Pending |

No gate above should be marked complete from documentation alone.

## ROS-006 boundary

The runtime migration must not modify:

- qualification dependency semantics;
- attested observation equality;
- certificate schema/digest rules;
- resolver determinism;
- Relationship 360 projection semantics.

The intended boundary remains:

`Holochain host/runtime -> deterministic observed dependency -> version-neutral qualification core -> certificate -> projection`

The host adapter becomes implementable only after the host workspace has a verified 0.7 foundation.

## Evidence limitations

This inventory is a controlled planning/audit artifact. It does **not** claim:

- regenerated 0.7 Cargo locks;
- regenerated Nix locks;
- successful 0.7 compilation;
- successful 0.7 conductor startup;
- successful network connectivity;
- migrated DNA hashes.

Those require execution in the actual 0.7 environment.
