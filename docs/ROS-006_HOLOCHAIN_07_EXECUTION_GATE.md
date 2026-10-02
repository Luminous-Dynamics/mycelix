# ROS-006 Holochain 0.7 normalization execution gate

Audit date: 2026-10-02  
Branch: `feat/ros-006-qualification-boundary-v2`

## Newly confirmed root-workspace constraint

The unified `mycelix-workspace/Cargo.toml` is a much larger migration surface than the standalone Civic/Commons manifests.

Its direct Holochain pins currently include:

- `hdk = "=0.6.1"`
- `hdi = "=0.7.1"`
- `holochain = "=0.6.1"`
- `holochain_client = "=0.8.1"`
- `holochain_types = "=0.6.1"`
- `holochain_zome_types = "=0.6.1"`
- `holochain_serialized_bytes = "=0.0.57"`
- `holo_hash = "=0.6.1"`
- `holochain_integrity_types = "=0.6.1"`
- `holochain_state = "=0.6.1"`
- `holochain_p2p = "=0.6.1"`
- `holochain_keystore = "=0.6.1"`
- `holochain_sqlite = "=0.6.1"`
- `holochain_wasmer_host = "=0.0.102"`
- `kitsune2 = "=0.4.1"`
- `lair_keystore = "=0.6.3"`

This confirms that changing only `hdk` and `hdi` would leave an internally inconsistent direct dependency graph.

## 0.7 normalization rule

The root workspace should first be reduced to the smallest necessary set of direct Holochain dependencies whose versions are intentionally controlled.

Normative targets from Holochain's 0.7 compatibility guidance:

| Component | 0.7 target |
| --- | --- |
| Holochain | 0.7.0 |
| HDK | 0.7.0 |
| HDI | 0.8.0 |
| Holochain Rust client | 0.9.0 |
| Kitsune2 bootstrap | 0.5.0 |
| Lair | 0.7.1 |

The remaining internal Holochain crates should not be independently guessed. Where they are direct dependencies because application code imports them, their exact 0.7-compatible versions should be established from the regenerated 0.7 lock graph and compiler diagnostics.

## Important distinction: direct dependency vs transitive dependency

The migration should avoid preserving 0.6-era internal Holochain crates merely because they happened to be explicitly pinned in the old workspace.

In particular, these are high-risk direct pins:

- `holochain_state`
- `holochain_p2p`
- `holochain_keystore`
- `holochain_sqlite`
- `holochain_wasmer_host`
- `kitsune2`
- `lair_keystore`

If application source does not import them directly, the preferred migration is to let Holochain 0.7's dependency graph provide them rather than inventing new independent version pins.

This is especially important because Holochain 0.7 changed conductor storage internals and the WASM/runtime stack. The official release notes describe the switch away from the old `holochain_sqlite`/`holochain_state` conductor database and the upgrade of Kitsune2, Wasmer, and Lair.

## Standalone-workspace interaction

The unified workspace explicitly includes broad Civic and Commons zome globs while the standalone workspaces independently declare their own Holochain dependency matrices.

Therefore there are two migration surfaces:

1. **Unified workspace resolution** — affects all members selected by `mycelix-workspace/Cargo.toml`.
2. **Standalone Civic/Commons resolution** — each has its own `Cargo.toml`, `Cargo.lock`, and flake.

They must converge on the same 0.7 compatibility generation rather than being migrated independently to subtly different graphs.

The repository also contains explicit workspace exclusions for some standalone/legacy components. Those exclusions must remain intentional during the migration; removing them solely to "make the workspace complete" would expand the migration surface without evidence that those packages belong to the active 0.7 target.

## Safe next execution sequence

1. Generate the 0.7 Nix lock graph from the intended Holonix 0.7 ref.
2. Enter the resulting Nix environment.
3. Update root HDK/HDI and only the direct Holochain crates that source inspection proves are imported.
4. Run Cargo resolution and capture the resulting dependency graph.
5. Compile the smallest representative integrity zome: Civic Bridge.
6. Use its compiler diagnostics to migrate the 0.6 action model.
7. Preserve existing validation tests while adapting only API representation.
8. Propagate proven transformations to the remaining Civic/Commons integrity zomes.
9. Migrate conductor/test configuration from WebRTC/tx5 to iroh/QUIC.
10. Clear/recreate 0.7 conductor data roots; do not reuse 0.6 databases.
11. Record new DNA hashes as a deliberate network-identity break.

The official upgrade guide explicitly requires the Holonix lock update, HDK/HDI update, action-model migration, conductor configuration changes, and clearing 0.6 conductor data.

## Current evidence status

- Upstream 0.7 target: **captured**
- Mycelix 0.6-era Nix lock: **confirmed**
- Mycelix root direct Holochain pins: **confirmed**
- Standalone Civic/Commons 0.6 pins: **confirmed**
- 0.6 action-model hotspots: **confirmed**
- 0.7 Nix regeneration: **not yet executed in this environment**
- 0.7 Cargo lock regeneration: **not yet executed**
- 0.7 compilation: **not claimed**
- 0.7 conductor startup/network evidence: **not claimed**

This gate intentionally keeps ROS-006 qualification semantics independent of the runtime migration. Holochain 0.7 remains an adapter/host concern, while the qualification core stays version-neutral.
