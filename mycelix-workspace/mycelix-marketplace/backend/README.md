# Mycelix Marketplace backend

Holochain integrity and coordinator zomes for listings, transactions, reputation, arbitration, messaging, notifications, search, and security.

> **Status:** research-grade pre-alpha. Historical phase summaries may describe stronger completion or Byzantine-tolerance claims; they are not the current security statement.

## Current source of truth

- Coordinator interface: `../contracts/zome-api.json`
- Purchase/recovery contract: `../contracts/leptos-vertical-slice-v1.json`
- Conflict-safe lifecycle: `../contracts/leptos-lifecycle-slice-v2.json`
- Arbitration integrity: `../contracts/arbitration-integrity-v1.json`
- Finance/reputation integrity: `../contracts/finance-reputation-integrity-v1.json`
- Deployment promotion: `../contracts/deployment-promotion-v1.json`

Run:

```sh
python3 ../scripts/check-zome-contract.py
python3 ../scripts/check-arbitration-integrity.py
python3 ../scripts/check-finance-reputation-integrity.py
python3 ../scripts/check-deployment-promotion.py
python3 ../scripts/check-live-evidence-contract.py
```

## Important current boundaries

- Transaction and dispute updates are reduced as revision trees. Unsafe concurrent leaves require matching bilateral approvals or a conflict-bound arbitration result; no authority deletes the original branches.
- Guarded arbitration uses equal votes. Reputation-weighted voting is disabled until the integrity zome can validate a bound score snapshot.
- Direct MATL score mutation is disabled in the guarded path.
- Reputation is projected from immutable delivered-transaction and arbitration-result evidence.
- Marketplace settlement does not create a local `Completed` revision. The external Finance record is the economic-finality evidence.
- The base hApp manifest is deliberately non-settling. A release profile must be rendered from exact Marketplace and Finance DNA bundles, discovered as active at runtime, and bound to passing live receipts.

## Historical material

The former Phase 4 backend README is preserved at:

`docs/archived-sessions/BACKEND_README_PHASE4_HISTORICAL.md`

Other phase reports and session summaries are historical planning/evidence artifacts. They should not be quoted as current product claims without a contemporary reproduction run.
