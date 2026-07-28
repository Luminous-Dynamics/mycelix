# Mycelix

Fractal CivOS — decentralized civic infrastructure on Holochain. **Pre-alpha research project.** Most clusters below build and pass unit tests; substantially fewer have been exercised with real multi-agent (Tryorama/sweettest) integration tests, and only a couple have any deployed UI at all. See the Maturity Matrix before assuming any given cluster is ready to build on.

## What can I actually run today?

The one cluster with a **live, canonical, user-facing application** is **[Pulse](https://mail.mycelix.net)** (decentralized email, `mycelix-workspace/mycelix-pulse/`) — install docs there. **[Praxis](https://praxis.mycelix.net)** (K-to-PhD curriculum platform) is also live. Everything else in this repo is a Holochain hApp with tests but no deployed frontend, a component-showcase scaffold, or research-stage code — check the matrix below, don't assume a cluster is deployment-ready because it "builds" or has tests.

The integration-test story is still being built out: the Tryorama harness and a dedicated integration-test guide are tracked as open work, not finished infrastructure — see open issues in `mycelix-workspace/`.

## Architecture

Mycelix is a unified hApp with 10 roles organized into domain clusters:

| Cluster | Domains | Zomes | Tests |
|---------|---------|-------|-------|
| **commons** | property, housing, care, mutualaid, water, food, transport | 35 | 5,276 |
| **civic** | justice, emergency, media | 16 | 2,273 |
| **hearth** | kinship, gratitude, care, autonomy, decisions, stories, milestones, rhythms | 12 | 1,023 |
| **finance** | payments (SAP), time exchange (TEND), recognition (MYCEL), treasury, staking | 14 | 140+ |
| **governance** | proposals, voting, threshold-signing (DKG), councils, constitution | 7 | 300+ |
| **identity** | DID registry, MFA, trust credentials, verifiable credentials, recovery | 9 | 100+ |
| **personal** | identity vault, health vault, credential wallet | 4 | 20 |
| **attribution** | dependency registry, usage receipts, reciprocity | 3 | 17 |
| **health** | clinical, consent, diagnostics, imaging, monitoring, pharmacy, records | 7 | submodule |

*Test counts above are combined unit+integration figures as previously reported; they are not split into "unit" vs. "real multi-agent" — see the Maturity Matrix immediately below for that distinction, where known.*

## Maturity Matrix

Honest per-cluster status, current as of this repo's own internal tracking (not independently re-run for this table — verify against the cluster's own CI/tests before relying on a cell). "Unit tests" = single-agent `cargo test`; "Multi-agent test" = Tryorama/sweettest with more than one simulated agent, which is what actually exercises DHT/validation logic across peers, not just business logic in isolation.

| Cluster | Builds | Unit tests | Multi-agent test | UI | Deployment status |
|---|---|---|---|---|---|
| **Pulse** (`mycelix-workspace/mycelix-pulse/`) | Yes | Yes | Yes (Sweettest, real ML-DSA sigs, restart-recovery included) | **Live** — mail.mycelix.net | Research alpha, real users possible |
| **Praxis** | Yes | Yes | Partial | **Live** — praxis.mycelix.net, Leptos CSR | Live, but flagship *source* does not currently compile at HEAD — the live site is served from a pinned IPFS snapshot restored around a broken build, not the latest commit |
| **Craft** | Yes | Yes (42) | Not verified this pass | Built (Leptos CSR), not deployed | Built, not live |
| **Music** | Yes | Yes | Not verified this pass | Built, WASM verified | Built, hApp packed, not confirmed live |
| **Governance** | Yes | Yes (44+) | Partial (156+ sweettest) | Built (paired with Finance UI at governance.luminousdynamics.io) | Experimental |
| **Health** | Yes | Yes | Not verified this pass | Built | Data-model research — includes real differential-privacy work, but this is not a clinical deployment |
| **Commons** | Yes | Yes (5,276 combined) | Not verified this pass | Scaffold/reserved, no real UI deployed | Pre-alpha |
| **Civic** | Yes | Yes (2,273 combined) | Not verified this pass | Scaffold only — its `apps/leptos` renders a component showcase, not real justice/emergency/media pages | Pre-alpha |
| **Hearth** | Yes | Yes (1,023 combined) | Not verified this pass | Built | Experimental |
| **Identity** | Yes | Yes (23+) | Yes (100+ sweettest) | None | Pre-alpha, underlies other clusters' auth |
| **Finance** | Yes | Yes | Partial | None dedicated (shares governance.luminousdynamics.io UI) | Not production-ready |
| **Personal, Attribution, Knowledge, DeSci, Marketplace, Supplychain, Space, Energy, Climate, Manufacturing, Position, Core (0TML)** | Yes | Yes (counts vary, see table above / cluster READMEs) | Not verified this pass | None deployed | Pre-alpha research; treat as schema/backend-only until a cluster's own README says otherwise |

**None of this has been independently threat-modeled or externally audited.** "Multi-agent test: Yes" means real Tryorama/sweettest coverage exists for that cluster, not that it's been reviewed by anyone outside this project.

## Structure

```
mycelix/
├── mycelix-commons/       # Commons cluster (property, housing, care, water, food, transport)
├── mycelix-civic/         # Civic cluster (justice, emergency, media)
├── mycelix-hearth/        # Hearth cluster (kinship, gratitude, autonomy)
├── mycelix-finance/       # Finance cluster (SAP/TEND/MYCEL 3-currency system)
├── mycelix-governance/    # Governance cluster (proposals, voting, DKG)
├── mycelix-identity/      # Identity cluster (DID, MFA, trust)
├── mycelix-personal/      # Personal cluster (vaults, wallet)
├── mycelix-attribution/   # Attribution cluster (dependency, receipts)
├── mycelix-health/        # Health cluster (git submodule)
├── mycelix-workspace/     # Orchestration: SDKs, tests, unified hApp, justfile
│   ├── sdk/               # Rust SDK
│   ├── sdk-ts/            # TypeScript SDK
│   ├── sdk-python/        # Python SDK
│   ├── happs/             # hApp definitions + unified hApp YAML
│   └── tests/             # Integration tests (sweettest, tryorama, ecosystem)
└── crates/                # Shared bridge crates
    ├── mycelix-bridge-common/       # Coordinator dispatch, cross-cluster, consciousness gating
    └── mycelix-bridge-entry-types/  # Shared DHT entry types
```

## Prerequisites

- [Nix](https://nixos.org/) with flakes enabled
- Holochain 0.6.0 (provided via Nix)

## Quick Start

```bash
cd mycelix-workspace
nix develop
just            # List all commands
just test       # Run all tests
just build      # Build all clusters
just dev        # Start development conductor
```

## Building Individual Clusters

```bash
just build-commons
just build-civic
just build-finance
just build-governance
just build-hearth
just build-identity
just build-personal
just build-attribution
```

## Testing

```bash
# Unit tests per cluster
cd mycelix-commons && cargo test --workspace
cd mycelix-civic && cargo test --workspace

# Sweettest (integration, requires conductor)
just test-sweettest

# Tryorama E2E
cd mycelix-workspace/tests/ecosystem && npm test
```

## Key Concepts

### Consciousness Gating
4D profile (identity/reputation/community/engagement) determines tier (Observer → Guardian) with progressive vote weights.

### Bridge Protocol
Cross-cluster communication via `CallTargetCell::OtherRole` in the unified hApp. Each cluster has a bridge zome for dispatch.

### 3-Currency System (Finance)
- **SAP** — Sustainable Abundance Points (community currency)
- **TEND** — Time Exchange Notes for Development (time banking)
- **MYCEL** — Mycelial Recognition (reputation/contribution)

## Holochain Version

| Component | Version |
|-----------|---------|
| holochain | 0.6.0 |
| hdk | 0.6.0 |
| hdi | 0.7.0 |

## License

**The root `LICENSE` file (Apache-2.0) does not cover most of this repository.** Every cluster directory checked (commons, civic, hearth, finance, governance, identity, personal, attribution, praxis, music, marketplace, climate, energy, knowledge, desci, core, workspace, manufacturing, position, craft) and the shared `crates/` bridge libraries are independently licensed **AGPL-3.0-or-later**. `mycelix-health` is a separate repository (git submodule) with its own license. `mycelix-lawful-identity` currently has no license specified at all — a real gap, not resolved by this note.

See [`LICENSING.md`](LICENSING.md) for the full per-directory breakdown and why the split exists. Commercial licensing for AGPL-covered clusters is available on request — contact tristan.stoltz@evolvingresonantcocreationism.com.
