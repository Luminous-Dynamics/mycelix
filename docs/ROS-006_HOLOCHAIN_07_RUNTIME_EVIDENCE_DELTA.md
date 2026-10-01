# ROS-006 Holochain 0.7 runtime evidence delta

Audit date: 2026-10-01
Branch: `feat/ros-006-qualification-boundary-v2`
Status: **repository inspection only; no runtime migration performed**

## Why this delta exists

The runtime inventory identified conductor/network/test surfaces abstractly. Direct inspection of the branch now provides concrete evidence that the repository contains multiple active 0.6-era runtime islands. These are high-value migration anchors because Holochain 0.7 changes conductor configuration, network transport, client tooling, and the action model. The official upgrade guide requires these surfaces to move together rather than treating the change as a version-string bump.

## Confirmed runtime findings

### 1. Production conductor is explicitly 0.6-shaped

File: `mycelix-workspace/deploy/conductor-config.prod.yaml`

Confirmed:

- header states Holochain 0.6.x;
- `network.signal_url` is present;
- `network.relay_url` uses the old WebRTC-era signal endpoint;
- `network.advanced.tx5Transport` is present;
- `db_sync_strategy: Resilient` is present.

These fields cannot simply survive a 0.7 conductor migration. Holochain 0.7 removes tx5/WebRTC and `signal_url`/ `webrtc_config`; `db_sync_strategy` becomes `db_sync_level`. The official guide also moves `request_timeout_s` under `network` and removes `chc_url`.

**Disposition:** migration target, not safe to patch on the ROS-006 qualification branch without the 0.7 dependency/runtime graph.

### 2. Test conductor is also explicitly 0.6-shaped

File: `mycelix-workspace/deploy/conductor-config.test.yaml`

Confirmed:

- header states Holochain 0.6.x;
- `signal_url` and `relay_url` point at the local test service;
- `advanced.tx5Transport` is configured;
- `db_sync_strategy: Fast` is present.

**Disposition:** migrate together with the production configuration during the dedicated runtime normalization.

### 3. SDK Docker conductor is an independent 0.6 network island

File: `mycelix-workspace/sdk-ts/docker/conductor-config.yaml`

Confirmed:

- `network.transport_pool` contains `type: webrtc`;
- `signal_url` is embedded in that transport;
- `bootstrap_service` is used;
- `db_sync_strategy: Fast` is present.

This is especially important because it would otherwise be easy to normalize the primary deploy configs while leaving integration tests on a different Holochain network/configuration generation.

**Disposition:** P0 runtime migration target.

### 4. SDK test/client surface is version-skewed

File: `mycelix-workspace/happs/support-tryorama/tests/package.json`

Confirmed:

- `@holochain/tryorama ^0.17.0`;
- `@holochain/client ^0.18.0`.

File: `mycelix-workspace/happs/lucid/tests/package.json`

Confirmed:

- `@holochain/client ^0.20.0`;
- `@holochain/tryorama ^0.19.0`.

The official 0.7 guide identifies JS client 0.21.0 and points 0.7 Tryorama users toward the community package `@holochain-open-dev/tryorama ^0.20.0`.

**Disposition:** separate client/test migration inventory required; do not globally replace package names without updating imports and test APIs.

### 5. Unified Sweettest workspace is hard-pinned to Holochain 0.6

File: `mycelix-workspace/tests/sweettest/Cargo.toml`

Confirmed:

`holochain = { version = "0.6", features = ["test_utils"] }`

The file also contains an explicit comment explaining that an exact `hdk = "=0.6.1"` dependency elsewhere in the workspace forces the sweettest graph to 0.6.1.

This is a particularly important migration boundary: changing the standalone sweettest crate alone could create an incompatible WASM host-function ABI against existing 0.6-built DNA bundles.

**Disposition:** migrate only after the workspace dependency graph is coherent.

### 6. Production Docker build is independently pinned to 0.6

File: `mycelix-workspace/deploy/Dockerfile.conductor`

Confirmed:

- WASM builder uses `rust:1.85-slim`;
- packer installs `holochain_cli 0.6.0`;
- `HC_VERSION=0.6.0`;
- runtime expects `holochain-0.6.0`;
- runtime expects `lair-keystore-0.6.3`.

The workspace Nix module separately documents Rust 1.96.0 as its current toolchain, so the Docker build is an independent reproducibility surface rather than merely another invocation of the Nix environment.

**Disposition:** P0 deployment reproducibility migration target. Do not assume the Docker path follows the Nix path.

### 7. Nix version source comment is stale/misaligned

File: `mycelix-workspace/flake.nix`

The flake says the Holonix version is controlled by:

`nix/modules/holochain-versions.nix`

but the actual audited flake directly pins:

`github:holochain/holonix/d21b3543`

while importing the shared environment from:

`../nix/modules/holochain-base.nix`

The latter exists and is real; the specifically referenced `holochain-versions.nix` path is not established by the audited branch.

**Disposition:** make the actual version authority explicit during 0.7 normalization. Do not invent a missing version module merely to satisfy the comment.

## New migration priority

| Priority | Surface | Evidence | Reason |
| --- | --- | --- | --- |
| P0 | `deploy/conductor-config.prod.yaml` | 0.6 / tx5 / signal | production runtime |
| P0 | `deploy/conductor-config.test.yaml` | 0.6 / tx5 / signal | test runtime |
| P0 | `sdk-ts/docker/conductor-config.yaml` | WebRTC / old bootstrap | independent integration runtime |
| P0 | `deploy/Dockerfile.conductor` | HC 0.6.0 / Lair 0.6.3 | deployment reproducibility |
| P0 | `tests/sweettest/Cargo.toml` | Holochain 0.6 | host/WASM compatibility |
| P1 | Tryorama/client manifests | 0.17–0.19 / client 0.18–0.20 | JS integration surface |
| P1 | `mycelix-workspace/flake.nix` | direct Holonix pin | Nix source-of-truth ambiguity |

## Important architectural conclusion

The migration surface is now demonstrably larger than the six integrity-zome source files already inventoried.

The actual stack is:

`Nix/Holonix -> Cargo dependency graph -> WASM build -> DNA packaging -> conductor config -> conductor binary -> network transport -> client/test harness`

A partial migration can therefore produce a superficially compiling repository whose WASM, conductor, Docker, and integration-test generations disagree.

The safest normalization strategy is consequently:

**one coherent 0.7 graph + one reproducible host/tooling generation + explicit new DNA/network identity**, followed by the ROS-006 Holochain adapter.

## Evidence limitations

This delta does not claim that any 0.7 migration has been executed.

No claim is made for:

- Cargo lock regeneration;
- Nix lock regeneration;
- 0.7 Docker image build;
- conductor startup;
- Sweettest execution;
- Tryorama execution;
- network connectivity;
- DNA hash generation.

Those remain execution gates in the runtime inventory.
