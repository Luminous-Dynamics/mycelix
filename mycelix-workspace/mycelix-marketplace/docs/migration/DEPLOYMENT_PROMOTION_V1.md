# Marketplace deployment promotion contract v1

## Why this exists

Compilation, a packed hApp, and a passing fixture preview are not deployment
evidence. Promotion must bind live scenario results to the exact source revision,
hApp, Marketplace DNA, Finance DNA where relevant, and official-client version.

## Profiles

### Base

Requires an active `marketplace` role and a passing two-agent lifecycle receipt.
This proves listing propagation, purchase, delivery, cancellation, stable roots,
and revision recovery for the tested build.

### Settlement

Requires the Base evidence plus active `finance` roles for both agents and the
settlement receipt. The receipt must prove response-loss recovery returns the
same payment identity, Marketplace remains `Delivered`, and fulfillment
reputation deduplicates to one event.

### Arbitration

Requires the Base evidence plus the three-agent arbitration receipt. Duplicate
votes must be rejected and finalization must be idempotent. Until a concurrent
dispute-head injection test is also recorded, promotion is explicitly
`promoted_with_limitations` and the dispute-creation UI remains gated.

## Running locally or on a controlled evidence runner

Set the conductor URLs/tokens already required by the scenario scripts, then:

```sh
export MARKETPLACE_HAPP_PATH=/path/to/exact.happ
export MARKETPLACE_DNA_PATH=/path/to/exact-marketplace.dna
export FINANCE_DNA_PATH=/path/to/exact-finance.dna
export MARKETPLACE_EVIDENCE_DIR=/tmp/marketplace-evidence
scripts/run-live-promotion.sh settlement
```

The evidence directory must be empty. Receipts are created with exclusive file
creation and cannot overwrite a previous run. `promotion.json` records the
profile result, artifact hashes, evidence hashes, and known limitations.

## Trust boundary

The GitHub workflow is restricted to a labeled self-hosted runner because the
scenarios need conductor Admin API access to authorize ephemeral signing
credentials. Hosted runners must not receive those interfaces, tokens, or funded
Finance identities.
