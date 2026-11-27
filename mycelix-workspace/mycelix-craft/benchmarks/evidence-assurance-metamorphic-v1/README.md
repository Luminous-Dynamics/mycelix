# Evidence Assurance Metamorphic Benchmark V1

**Status:** draft protocol; synthetic fixtures and structural validation only.  
**Owner:** [Issue #6775](https://github.com/Luminous-Dynamics/mycelix/issues/6775)  
**Source corpus:** [Water Service Repair Benchmark V1](../water-service-repair-v1/README.md)

## Why this benchmark exists

Static known-answer tests ask whether a system returns the expected answer for a frozen example. Metamorphic testing asks whether relationships between answers remain valid after a controlled transformation of the input. This benchmark uses that second approach to test whether evidence and authority guarantees survive adversarial change.

The goal is not to maximize refusal. It is to preserve safety invariants while keeping justified progress possible. Every adversarial mutation therefore has a negative control describing a legitimate input change that should have a bounded, explainable effect.

## Files

- `manifest.json`: scope, provenance, claim boundaries and counts.
- `seeds.json`: references to synthetic WSR known-answer cases; no operational data is copied.
- `relations.json`: 16 cross-input invariants with seed, mutation and control references.
- `mutations.json`: 30 adversarial operators and 16 negative controls.
- `scorecard.json`: 15 metric definitions with explicit numerators, denominators, targets, gate flags and field-data requirements.
- `comparators.json`: 9 baseline profiles and a matched-comparison protocol, including manual, checklist, single/RAG/multi-agent LLM, Craft, IBM DOORS Next, Jama Connect and Siemens Polarion.
- `validate.sh`: structural and referential-integrity checks only.

Digest semantics are explicit: WSR evidence digest strings are shape-only synthetic placeholders, not cryptographic commitments. An evaluator adapter must recompute commitments from exact fixture bytes and verify the schema before using them.

## Invariant families

1. Evidence monotonicity: removal, downgrade, expiry, revocation, supersession and withdrawal cannot improve validity.
2. Provenance independence: collapsing common-cause groups cannot increase independent evidence count.
3. Scope confinement: new zone, jurisdiction, population, event, edge, artifact revision or use-context requires fresh applicability qualification.
4. Authority preservation: model output, provider assertion, local-review eligibility and draft artifacts do not become consent, observation or authorization.
5. Deterministic replay: conflicting event identity fails closed; host sequence controls ordering; semantically unordered inputs canonicalize identically.
6. Measurement integrity: response, repair, restoration, lab clearance and authority closure remain distinct events and metrics.
7. Non-overblocking controls: valid independent observations, current authority renewals, exact replay and a complete declared alternative may legitimately advance the result.

## Pre-registered evaluation targets (targets, not results)

- **0** false closures on the critical known-answer and mutation corpus.
- **0** false NotAffected transfers across event/artifact/edge/context.
- **0** stale, revoked or cross-context evidence acceptances.
- **0** false independence credit from shared common-cause groups.
- **100%** deterministic replay for identical canonical inputs and host-sequence event streams.
- **100%** detection of the hard-safety / authorization mutation family.
- **At least 95%** detection of non-equivalent high-risk mutants overall.
- Report equivalent mutants separately; do not count them as survived or detected without adjudication.
- Report false holds, false rejections, time-to-review, reviewer minutes, appeal overturns and justified recovery separately. A system that blocks everything does not pass.

Thresholds are proposed qualification gates, not measured Mycelix performance. Freeze corpus and thresholds before evaluating candidate systems. The scorecard deliberately separates hard safety gates from usability and field-outcome measures; there is no weighted aggregate score.

## Comparison design

Run all candidates on the same frozen seed bytes, transformation schedule, evidence access, allowed tools, compute/time budget and adjudication protocol. Compare:
- domain experts working manually;
- structured checklist/template workflow;
- a single LLM;
- retrieval-augmented LLM;
- multi-agent LLM;
- Craft with the independent owner evaluators;
- configured Jama Connect, IBM DOORS Next and Siemens Polarion where their relevant capabilities can be fairly exercised and evidence exported.

Do not claim commercial ALM platforms lack traceability, baselines, workflow or impact analysis. The differentiation hypothesis is narrower: preserve community intent, dissent and consent while independently qualifying claim support, conclusion closure, scoped impact and the path from authorization through execution to observed outcome.

Report precision/recall and false-accept/false-hold rates by risk tier. Stratify by jurisdiction, language, code-switching, evidence source type and consequence class. For South African use, annotations must be authored or adjudicated by fluent domain reviewers in English, Afrikaans, isiZulu, isiXhosa and Sesotho; translated English prompts alone are not a multilingual benchmark.

## Evaluator adapter boundary

Structural validation here does not execute an evaluator. A future adapter must:
1. load one frozen WSR seed and exact digests;
2. apply exactly one registered mutation;
3. call the domain owner's evaluator(s), not a benchmark-local duplicate;
4. capture input, owner versions, evaluator output, decision receipt and replay digest;
5. run the negative control;
6. compare the observed relation with its oracle;
7. preserve exact-head and tool/environment provenance.

Until that exists, this benchmark can claim only protocol definition and structural conformance. It cannot claim operational execution, independent qualification, compliance, service improvement or SOTA.

## Current industry context

IBM DOORS Next, Jama Connect and Siemens Polarion already provide requirements traceability, baselines, change-impact analysis and workflow/audit capabilities. This benchmark should compare against properly configured instances, not strawmen. The research question is whether the complete evidence-governed chain—including consent, exact support, scoped impact, closure and outcome measurements—can be independently replayed with fewer unsafe promotions and acceptable reviewer burden.
