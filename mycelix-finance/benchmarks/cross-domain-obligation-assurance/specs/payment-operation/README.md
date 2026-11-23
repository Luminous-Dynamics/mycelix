# PCOE effect-owner payment model

This directory contains a bounded TLA+ state model for the effect-owner payment contract.

## Pinned model checker

The reproducible workflow pins the stable TLA+ tools release **v1.7.4** and verifies the `tla2tools.jar` asset against the SHA-1 checksum published in that release's notes:

```text
bee4a54f3ee3d4afc347c3240ec2d9e93b075104  tla2tools.jar
```

TLA+ v1.8.0 was visible as a pre-release on the 23 November 2026 snapshot, so the workflow uses the stable v1.7.4 release rather than a moving pre-release. The tools require Java 11 or newer; CI uses Temurin 17.

## Run both models

From this directory, with the pinned jar available:

```sh
java -jar ./tla2tools.jar -config PCOEEffectOwnerPayment.cfg PCOEEffectOwnerPayment.tla
java -jar ./tla2tools.jar -config PCOEEffectOwnerPayment-mutant.cfg PCOEEffectOwnerPayment.tla
```

The first run must complete without an invariant violation. The second is an intentional negative control and must fail specifically on `NoPartialLocalValueEffect`, because it enables either a debit that commits outside the same-source-chain atomic batch or a duplicate debit after commit.

The state space is finite: retries, durable-outbox scans and unsafe debit mutations are bounded, operation / fee / response / wake-up states are finite, and the safe model disables the mutant actions.

## What the model does and does not prove

The model checks the abstract safety contract:
- at most one claim and journal batch;
- payer debit, beneficiary credit and fee liability appear together;
- Payment / Receipt / outbox projections match the journal batch;
- a committed operation corresponds to a complete batch;
- response delivery cannot precede commit;
- an unknown fee outcome is reconciled through its original outbox identity;
- a lost post-commit wake-up does not erase a Pending outbox item; a bounded durable scanner can rediscover it and restore the dispatch signal;
- expiration of a client cache does not erase the durable operation tombstone.

It does **not** prove that the live Holochain implementation satisfies the model, that a source-chain identity is globally unique across agents or forks, or that Treasury / external rails have the required idempotency. Those require real-agent tests and effect-owner adapter evidence.

**Status as of 23 November 2026:** source-authored and statically reviewed; TLC has not yet been run in this authoring environment. The focused GitHub Actions workflow is the first reproducible TLC qualification attempt. Do not report a model-check pass until the safe configuration completes and the mutant produces the expected counterexample.
