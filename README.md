# Mycelix

Fractal CivOS — decentralized civic infrastructure on Holochain.

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

See individual cluster directories for license information.
