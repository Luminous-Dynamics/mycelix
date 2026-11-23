# PCOE two-owner payment model

This directory contains a bounded TLA+ model for an owner-authenticated, two-sided SAP transfer protocol. The payer's intent/debit, the beneficiary's claim/credit, and Treasury fee delivery are independent effects with distinct source-chain owners.

## Pinned model checker

The reproducible workflow pins the stable TLA+ tools release **v1.7.4** and verifies the `tla2tools.jar` asset against the SHA-1 checksum published in that release's notes:

```text
bee4a54f3ee3d4afc347c3240ec2d9e93b075104  tla2tools.jar
```

TLA+ v1.8.0 was visible as a pre-release on the 23 November 2026 snapshot, so the workflow uses stable v1.7.4 rather than a moving pre-release. CI uses Temurin 17.

## Run both models

From this directory, with the pinned jar available:

```sh
java -jar ./tla2tools.jar -config PCOEEffectOwnerPayment.cfg PCOEEffectOwnerPayment.tla
java -jar ./tla2tools.jar -config PCOEEffectOwnerPayment-mutant.cfg PCOEEffectOwnerPayment.tla
```

The safe model must complete without an invariant violation. The mutant is a deliberate negative control and must fail specifically on `SenderIntentDebitAtomicity`: it permits a debit to commit without the payer's immutable transfer intent, or permits a second debit after the same operation was committed. This is a hypothetical unsafe split-write implementation, not a claim that same-source-chain Holochain writes can commit independently.

The state space is finite: retries, durable-outbox scans, chain-head race observations, recipient-fork observations and unsafe debit mutations are bounded. The model includes:

- sender-owned transfer intent + debit + fee liability / outbox committed on the payer's source chain;
- a separately authored beneficiary claim + credit on the recipient's source chain;
- sender and recipient response-loss / reconciliation states;
- `Settled` only after an exact recipient claim is observed;
- competing recipient successors causing `Conflicted` / frozen spendability rather than last-write-wins;
- same-key retry after a Holochain `chain head moved` response;
- durable Treasury outbox recovery after a lost `post_commit` wake-up;
- a Treasury-owned idempotent effect and receipt;
- durable operation tombstone after client cache expiry.

## Scenario corpus

- `../../pcoe-effect-owner-payment-scenarios.v0.1.json` contains 22 synthetic adversarial cases.
- `../../pcoe-effect-owner-payment-scenarios.v0.1.schema.json` defines the corpus shape.
- The CI workflow additionally checks all IDs are unique and each expected disposition appears in the declared vocabulary.

## What the model does and does not prove

The model checks effect counts and state transitions across a bounded state space. It does **not** calculate actual monetary amounts, fee arithmetic, valuation or full value-conservation arithmetic; those remain obligations for the FIN-SAFE arithmetic kernels and property tests. It also does **not** prove the live Holochain implementation satisfies the model, global uniqueness across alternate agent keys / clones / forks, or bank / payment-rail finality. Those require real-agent Tryorama / Sweettests, authority and fork-resolution proofs, Treasury adapter qualification and independent external evidence.

**Status as of 23 November 2026:** authored and statically audited; this two-owner revision has not yet been TLC-checked. Prior TLC results on the earlier single-batch model are superseded. The pinned GitHub Actions workflow must pass on the exact PR head, with the safe model complete and the unsafe mutant detected, before this revision can claim a model-check pass.