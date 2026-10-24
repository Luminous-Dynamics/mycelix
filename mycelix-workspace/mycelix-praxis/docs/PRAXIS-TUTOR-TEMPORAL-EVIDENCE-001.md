# PRAXIS-TUTOR-TEMPORAL-EVIDENCE-001

**Status:** synthetic contract and evaluator implementation; not yet executed by the Rust toolchain or independently reviewed by educators.

## Why this extension exists

The v1 tutor-history benchmark checks whether a proposed diagnosis/action is backed by cited observations, including contradictory evidence. It does not model the difference between when an event happened and when that record was imported, explicit evidence-validity windows, retractions, or correction history. Those omissions allow a plausible but unsafe evaluator to treat a late-imported record as recent, cite an expired result as current-state support, or use a retracted/superseded source as authoritative.

This extension adds those dimensions without changing the existing v1 wire contract.

## Temporal semantics

Every solver-visible temporal observation carries:

- `occurred_at_unix` plus `occurred_at_uncertainty_seconds`: the centre estimate and maximum absolute uncertainty for when the attempt or assessment occurred.
- `recorded_at_unix` plus `recorded_at_uncertainty_seconds`: the centre estimate and uncertainty for when the record entered the evidence store.
- `occurrence_time_provenance` and `record_time_provenance`: declared source classes, such as assessment server, educator attestation, server receipt, local store, imported-unverified or derived-estimate. These labels are **asserted metadata**, not cryptographic authentication.
- `valid_until_unix`: an explicit, optional deadline for using the record as current-state support. Null means that no expiry has been asserted; it does **not** mean permanent truth or prove current mastery.
- `status`: `active` or `retracted`.
- `supersedes`: provenance links to source records this record replaces. The old record remains in history; a replacement does not silently delete the audit trail.

Each case has a fixed `evaluation_time_unix`. The verifier treats timestamps as closed intervals, not exact points. If the entire interval is after the evaluation cutoff, it is leakage and invalidates the bundle; if the interval straddles the cutoff, the observation is marked ambiguous instead of silently ordered. Unverified/imported or derived timestamps must declare nonzero uncertainty and cannot alone establish temporal proof. Supersession edges must resolve, be acyclic, and have a definitely later record-time interval than the source they replace.

**Important policy choice:** recency is not truth. A recent import does not make an old attempt recent; equal timestamps do not establish an order; a newer attempt in a different task context does not automatically override an older one; and age alone does not expire an observation when no validity deadline is declared. The tutor should preserve contradictory historical evidence and request a fresh, context-matched attempt when the evidence cannot support a current-state claim.

## Fixture families

The public synthetic pack contains nine cases and 18 observations. TE005 includes a record-time uncertainty interval that straddles the evaluation cutoff; TE006 and TE009 exercise unverified/imported or derived timestamp provenance. The pack includes:

| Case | Adversarial property |
|---|---|
| TE001 | Old attempt imported late; event time must remain distinct from ingestion time |
| TE002 | A record with an explicit validity deadline has expired |
| TE003 | A retracted assessment and a later source correction/supersession |
| TE004 | Correctness differs across worked-example and transfer contexts |
| TE005 | Same-context, same-time conflicting attempts without a justified ordering |
| TE006 | An imported historical summary appears newer in storage than a later learner attempt |
| TE007 | Only expired and retracted evidence is present |
| TE008 | Equal timestamps do not break a conflict tie |
| TE009 | A derived estimate has provenance but is not an observed correctness result |

The oracle is a separate file. The candidate receives the observation manifest and policy configuration, never the oracle or mutation matrix. These nine cases are contract probes, not expert-validated diagnoses or evidence of representative real-world performance.

## Executable contract

The Rust module `tutor_temporal_evidence.rs` provides:

- strict Serde DTOs that reject unknown fields and omitted required nullable fields;
- fail-closed temporal, timestamp, identity, provenance-edge, cycle, and oracle-reference validation;
- a temporal eligibility classifier (`eligible`, `expired`, `retracted`, `superseded`, `future_occurrence`, `not_recorded_at_evaluation`);
- a scorecard that separates semantic decision fit from evidence-grounded acceptability;
- counters for expired support, retracted/superseded citations, future/unavailable references, ambiguous and unverified timestamps, missing required contradictions, and critical violations;
- an exact-byte BLAKE3 binding between the input observations and the predictions;
- a 22-entry adversarial and metamorphic matrix linked to 22 named Rust test definitions, including interval/cutoff ambiguity, unverified-import handling, provenance/origin mismatch, and supersession-order uncertainty.

Expired evidence may still be cited to explain historical context, but it cannot support a current-state claim. Retracted, superseded, future, or not-yet-recorded sources are not accepted as evidence citations. Abstention and a correctly targeted evidence request do not incur claim-citation penalties.

## Qualification targets (proposed, not measured)

Before promoting the benchmark beyond synthetic conformance:

1. **Zero temporal leakage:** no accepted result may consume evidence unavailable at the evaluation time.
2. **Zero invalid current-state support:** no accepted claim may rely on expired, retracted, or superseded evidence as current support.
3. **Zero unsupported references and missed required contradictions** in accepted claims.
4. **100% semantic invariance** under observation-row permutation, while the exact-byte digest changes.
5. On an independently adjudicated held-out set, target at least **95% evidence-grounded acceptable decisions**, reported with per-diagnostic-family confidence intervals—not just a pooled score. This is a proposed gate, not a current result.
6. Report unsafe-claim rate, unnecessary abstention, targeted-clarification precision, family-level calibration, latency, and cost separately. Do not combine them into one tunable leaderboard score.

## Comparison plan

Different external benchmarks measure different things and must remain distinct:

- [LongTutor (ACL 2026)](https://aclanthology.org/2026.acl-long.1371/) is a capability comparator for historical evidence acquisition, learner-state diagnosis, and adaptive action. Praxis should report each component separately rather than only overall tutoring text quality.
- [Scale TutorBench](https://labs.scale.com/leaderboard/tutorbench) evaluates rubric-based adaptive explanations, assessment/feedback, active-learning support, and multimodal inputs. It is not a direct measure of durable learning.
- [StudentBench](https://arxiv.org/abs/2609.28470) is a separate real-learner outcome comparator. Its September 2026 preprint reports GRE tutoring results in a specific study; reproduction and scope-matched evaluation are required before drawing broader conclusions.

For an internal ablation, compare B0 (deterministic Praxis evidence/BKT/SRS route) with B1 (identical route plus a pinned optional Symthaea evidence-gap/hypothesis suggestion). Both systems must use the same policy gates, candidate-visible input, independent oracle, and evaluator. Record Git SHA, fixture/oracle/schema/verifier digests, model and prompt/config digests, and run configuration. Any future human-outcome evaluation must independently measure unaided transfer, delayed retention, and displacement/non-harm under an appropriate consent and review process.

## Limits and next qualification gates

This synthetic pack does not prove educational efficacy, generalization, or SOTA performance. The code and tests have been authored but have **not** been compiled or run in this authoring environment; no Rust-test or CI pass is claimed. Static replay and source inspection are separate from executable conformance. Timestamp provenance in this version is declared metadata only; production integrations still need verifiable receipts/signatures or another trusted ingestion path. Keep the associated PR in draft until exact-head Rust compilation/tests, full JSON Schema 2020-12 meta-validation, and independent review of the oracle and scorer are complete.

The larger follow-up in [issue #5481](https://github.com/Luminous-Dynamics/mycelix/issues/5481) remains necessary: an independent generator–verifier pipeline, whole-trajectory and whole-diagnostic-family held-out splits, at least 120 expert-adjudicated trajectories across eight or more diagnostic families with double review and disagreement resolution, and the B0/B1 ablation. Synthetic fixtures must not be promoted to expert-reviewed truth by automation.
