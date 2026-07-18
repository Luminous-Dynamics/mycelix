# Mycelix Marketplace

A research-grade, Holochain-based peer-to-peer marketplace exploring evidence-bearing commerce, explicit transaction authority, conflict-aware state reduction, and community arbitration.

> **Status:** pre-alpha. The repository contains substantial backend and frontend work, but production readiness, Byzantine tolerance, legal finality, and globally convergent conflict resolution are not established claims.

## Canonical implementation paths

- `backend/` — eight Holochain zome families and the Marketplace DNA/hApp manifests.
- `frontend-leptos/` — the canonical frontend migration path, using typed Rust contracts and an official-client browser bridge.
- `frontend/` — the older SvelteKit product prototype. Preserve it as a visual and workflow reference while Leptos parity is built; do not treat it as the authoritative backend contract.
- `contracts/` — machine-checked coordinator and security evidence contracts.
- `docs/migration/` — current implementation boundaries and required live evidence.

The coordinator API manifest currently records **8 zomes and 75 extern functions**. Verify it instead of relying on this number:

```sh
python3 scripts/check-zome-contract.py
```

## Current guarded product slice

The Leptos path currently models and statically gates:

1. authenticated conductor connection through the official Holochain client;
2. listing discovery and listing detail;
3. authoritative purchase creation;
4. stable-root transaction recovery;
5. conflict-aware transaction revision reduction;
6. seller confirmation and shipment;
7. buyer delivery confirmation;
8. cancellation by an authorized party;
9. typed, conflict-aware arbitration contracts;
10. Finance settlement projection and evidence-derived reputation;
11. runtime discovery of active/deferred hApp roles and fail-closed feature gates;
12. artifact-bound live promotion receipts for lifecycle, settlement, arbitration, and isolated multi-conductor network profiles.

Development fixtures are opt-in. A failed live connection never silently becomes fixture data.

## Security model

Critical rules belong in integrity zomes rather than cooperative UI or coordinator checks. Current guarded work includes:

- transaction terms derived from the authoritative listing;
- buyer/seller authorship and lifecycle-transition validation;
- explicit concurrent-update conflicts rather than arbitrary winner selection;
- dispute, vote, and result binding to exact revisions and action hashes;
- equal arbitration votes until a score snapshot can be integrity-verified;
- Finance finality represented by an external payment record rather than a locally forgeable Marketplace status;
- immutable fulfillment and arbitration reputation events;
- direct mutable MATL score updates disabled in the guarded path.

The derived reputation ratio is an explainable summary of visible evidence. It is **not** an integrity authorization weight or a demonstrated Byzantine-fault-tolerance threshold.

## Finance dependency

Retry-safe settlement requires the companion Finance patch. Marketplace derives one reference from the stable transaction root and queries Finance for durable status. Marketplace remains `Delivered`; a matching Finance `Completed` record represents economic finality.

The Finance protocol intentionally fails closed when it finds an indeterminate `Processing` record. This provides retry-safe, at-most-once behavior; it is not an atomic rollback guarantee across DHT records and token movement.

## Validate

Run the static, bridge, Rust, and WASM gates from a clean checkout:

```sh
scripts/validate-leptos-migration.sh
```

The script requires Python, Node.js 22, npm, Rust 1.94, and the `wasm32-unknown-unknown` target. It validates:

- backend/API drift;
- typed Leptos calls;
- purchase, lifecycle, arbitration, Finance, and reputation contracts;
- MessagePack wire fixtures;
- browser bridge and conductor-scenario bundles;
- Rust formatting, tests, and WASM checks.

The bundled conductor scenarios are not execution evidence until run against configured conductors. `scripts/run-live-promotion.sh` now emits exclusive, machine-readable receipts bound to the exact source revision, hApp hash, DNA hashes, active role inventories, and official-client version.

## Frontend modes

Default live build:

```sh
cd frontend-leptos
cargo check -p marketplace-web --target wasm32-unknown-unknown
```

Explicit fixture build:

```sh
cd frontend-leptos
cargo check -p marketplace-web --target wasm32-unknown-unknown \
  --no-default-features --features dev-fixtures
```

Settlement-enabled builds additionally require the patched Finance role, runtime confirmation that the role is active, and artifact-bound live evidence:

```sh
cd frontend-leptos
cargo check -p marketplace-web --target wasm32-unknown-unknown \
  --features finance-settlement
```

## Deployment promotion

- The base `backend/happ.yaml` is intentionally not settlement-capable.
- `scripts/build-promotion-artifacts.sh` builds and binds the exact WASMs, DNA, hApp, source revision, and toolchain receipts.
- `scripts/run-disposable-promotion.sh` installs distinct agents into an isolated conductor and runs real signed lifecycle, arbitration, or settlement evidence.
- `scripts/run-network-promotion.sh` launches two independent conductors, proves an unsafe conflict remains explicit after healing, then requires independent buyer/seller approvals and the same branch-preserving bilateral projection on both peers.
- `scripts/seal-promotion-bundle.sh` emits an Ed25519-signed release bundle; `scripts/verify-promotion-bundle.sh` independently revalidates it.
- Settlement requires the patched Finance DNA and an explicit funded-test bootstrap hook.
- See [`docs/migration/REPRODUCIBLE_PROMOTION_V1.md`](docs/migration/REPRODUCIBLE_PROMOTION_V1.md), [`docs/migration/MULTI_CONDUCTOR_NETWORK_EVIDENCE_V1.md`](docs/migration/MULTI_CONDUCTOR_NETWORK_EVIDENCE_V1.md), and [`docs/migration/DEPLOYMENT_PROMOTION_V1.md`](docs/migration/DEPLOYMENT_PROMOTION_V1.md).

## Claims and evidence

See [`docs/CLAIMS_AND_EVIDENCE.md`](docs/CLAIMS_AND_EVIDENCE.md). Historical completion reports remain in the repository as project history, but they are not current release evidence unless explicitly reaffirmed by a reproducible contract and test artifact.

## License

AGPL-3.0-or-later. See [`LICENSE`](LICENSE).
