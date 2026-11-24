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

**Exact-head TLC result (23 November 2026): PASS** on current PR head `5392b34b4b9553d2add00513abc19b85ac55ee9d`, [workflow run #45165934675](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45165934675). The safe model generated 66,120 states, explored 12,105 distinct states at depth 19, and finished with zero states left on queue and no invariant violation. The unsafe mutant produced the expected `SenderIntentDebitAtomicity` counterexample via `Prepare → UnsafeSenderDebitWithoutIntent`, leaving a sender debit without a sender intent.

The same exact-head workflow also passed the 22-case corpus gate, the scenario-to-model coverage gate (9 bounded-modelled, 6 partially-modelled, 7 not-in-model), pinned TLC checksum verification and exact-source-commit assertion. Coverage-map SHA-256: `72edb8afb5d82f9243a632cf011d6d7a57071cc71caed385a2f8a43fb4aa22b4`. Latest retained artifact SHA-256: `b7eef382801d0fa377ff926648d3af13737172f9d909bfacd018e1e6eae2256d`.

This pass qualifies the bounded abstract model only; it is not live Holochain, Rust, Tryorama / Sweettest, cross-agent fork, Treasury-adapter or external-rail evidence. Numeric SAP amount / fee arithmetic is outside this model and remains a separate FIN-SAFE obligation.

## Claim-admission authority submodel (FIN-EFFECT-002)

`PCOEClaimAdmissionAuthority.tla` is a finite quorum-admission submodel intended to be composed with the payment lifecycle above. It does not replace the payment model or assert that a witness service exists in production.

The frozen configuration uses four witnesses, at most one Byzantine, and a three-signature threshold (the (n=3f+1, q=2f+1) baseline). Honest witnesses durably record at most one fingerprint per singleton effect role. An owner effect can commit only after a certificate passes the declared threshold. The model separates certificate state from the owner effect and spendable state.

Run the safe profile and two negative controls:

```sh
java -jar ./tla2tools.jar -config PCOEClaimAdmissionAuthority.cfg PCOEClaimAdmissionAuthority.tla
java -jar ./tla2tools.jar -config PCOEClaimAdmissionAuthority-minority-mutant.cfg PCOEClaimAdmissionAuthority.tla
java -jar ./tla2tools.jar -config PCOEClaimAdmissionAuthority-journal-mutant.cfg PCOEClaimAdmissionAuthority.tla
```

- Safe model: no conflicting certificates or spendable effects; no honest witness equivocation.
- Minority mutant: accepts a certificate with (q-1) signers and must violate `CertificateQuorum`.
- Journal-loss mutant: loses an honest witness's durable vote while leaving its prior signature evidence in circulation; the witness then signs the competing fingerprint and must violate `HonestNonEquivocation`.

The bound (n=4, f=1, q=3) is a finite illustrative instance, not a general proof for every set size or fault model. The quorum-intersection expression is included as a state invariant. This model still assumes authenticated distinct witness enrollment, a correct fault bound, durable journal semantics in the safe profile, correct cryptography, and a safe authority-set transition protocol. It does not model reconfiguration yet, and it is not a Holochain integration, live witness network, or external settlement proof.
