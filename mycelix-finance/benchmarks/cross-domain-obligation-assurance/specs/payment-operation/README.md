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

## Model-to-scenario coverage

`../../pcoe-effect-owner-payment-model-coverage.v0.1.json` maps each of the 22 EOP cases to TLA actions / invariants and names the remaining qualification layer. The current map is 9 bounded-modelled, 6 partially-modelled and 7 not-in-model. The CI workflow checks that its IDs match the scenario corpus. A modelled label means the abstract property exists in the bounded model; it does not mean a real Holochain or adapter test passed.

## What the model does and does not prove

The model checks effect counts and state transitions across a bounded state space. It does **not** calculate actual monetary amounts, fee arithmetic, valuation or full value-conservation arithmetic; those remain obligations for the FIN-SAFE arithmetic kernels and property tests. It also does **not** prove the live Holochain implementation satisfies the model, global uniqueness across alternate agent keys / clones / forks, or bank / payment-rail finality. Those require real-agent Tryorama / Sweettests, authority and fork-resolution proofs, Treasury adapter qualification and independent external evidence.

**Exact-head TLC result (23 November 2026): PASS** on commit `9712134efadbe7da28fe04519c838f13a81cc9bb`, [workflow run #45164659271](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45164659271). The safe model generated 66,120 states, explored 12,105 distinct states at depth 19, and finished with zero states left on queue and no invariant violation. The unsafe mutant produced the expected `SenderIntentDebitAtomicity` counterexample via `Prepare → UnsafeSenderDebitWithoutIntent`, leaving a sender debit without a sender intent. The workflow also passed the 22-scenario corpus gate, pinned TLC checksum check and exact-source-commit assertion.

This pass qualifies the bounded abstract model only; it is not live Holochain, Rust, Tryorama / Sweettest, cross-agent fork, Treasury-adapter or external-rail evidence. Numeric SAP amount / fee arithmetic is outside this model and remains a separate FIN-SAFE obligation.