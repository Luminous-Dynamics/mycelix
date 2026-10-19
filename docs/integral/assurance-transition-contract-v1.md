# AssuranceFlowV1 — executable assurance transition contract

## Why this is the next layer

The public Integral architecture already names **Assurance and Validation Flows**, **Viability Envelopes**, **Coordination Envelopes**, FRS evidence references/confidence, and non-executive recommendations. The engineering gap is therefore not to invent those concepts, but to make their boundaries mechanically enforceable and portable. citeturn0search1turn0search2

This contract defines a runtime-neutral transition algebra:

`Observation → Evidence → Verification → Qualification → Certification → Recognition → Decision-use → Authorization → EffectReceipt → Outcome → Correction`

No arrow is implicit. A later state cannot be inferred merely because an earlier state exists.

## Core protections

- **Epistemic:** verification does not silently become qualification; predictions/counterfactuals cannot satisfy observation predicates.
- **Authority:** recommendation, decision, authorization, and effect are separate receipts.
- **Provenance:** foreign recognition preserves foreign origin.
- **Temporal:** stale or superseded evidence remains historical but cannot satisfy current requirements.
- **Anti-oracle:** an evidence reference is not itself independent verification; formal proof is not physical qualification.
- **Reliability:** reliability history is not current availability.
- **Correction:** corrections append lineage; they do not rewrite the historical observation.
- **Federation:** Coordination Envelopes have explicit scope, affected nodes, authority bounds, and dissolution.
- **Reproducibility:** formal closure requires exact source/build identity plus executable and formal evidence.

## Relationship to Integral

The Development Guide describes Assurance and Validation Flows as a federation reciprocity primitive carrying certification results, safety data, provenance proofs, and reliability histories. It also describes Viability and Coordination Envelopes and explicitly states that FRS recommendations are non-executive. citeturn0search1

The White Paper's distributed-production control loop likewise depicts signed node-state summaries, FRS integrity/schema verification, scope detection, threshold evaluation, bounded Coordination Envelopes, local execution, monitoring, and automatic dissolution. citeturn0search2

Accordingly, this artifact is a **neutral engineering refinement**, not a claim that Integral has adopted these exact types or transitions.

## Next implementation tranche

1. Implement these transitions as a small dependency-light Rust state machine.
2. Add negative-first adversarial fixtures for every forbidden promotion.
3. Bind the state machine to the existing COS/ProductiveLoop evidence contracts.
4. Add ASSURE-FV-015..028 formal witnesses, starting with verification≠qualification, qualification≠authorization, origin preservation, bounded coordination scope, correction lineage, and retry idempotency.
5. Add a runtime receipt manifest with source commit, artifact hash, solver version, result digest, and build identity.
6. Expose Symthaea outputs only as analytical artifacts (prediction, anomaly, counterfactual, hypothesis, risk signal) unless an explicit verification/qualification transition occurs.

## Claim ceiling

This is an engineering contract. It does not establish real-world safety, productivity, fairness, ecological outcomes, cryptographic security, or Integral ratification.
