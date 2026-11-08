# GCPN Track A — deterministic event-envelope replay evaluator v0.1

**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Comparison owner:** [GCPN-AWARENESS-INDUSTRY-COMPARISON-001 #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Awareness contract:** [GCPN-CIV-AUTONOMOUS-SYSTEM-SAFETY-001 #5934](https://github.com/Luminous-Dynamics/mycelix/issues/5934)  
**Status:** offline evaluator; CI smoke input is synthetic only  
**Claim ceiling:** common-event-envelope replay metrics only; no physical sensor, field, intent or operational claim

## 1. Why this evaluator exists

The comparison manifest defines Track A, but a manifest verifier does not calculate comparative metrics. This crate adds a deterministic offline evaluator.

It reuses the existing Rust manifest verifiers as local dependencies:
- `gcpn-civil-autonomous-awareness-verifier` validates the exact CAA event taxonomy and its authority/privacy semantics.
- `gcpn-awareness-industry-comparison-verifier` validates the two-track comparison protocol and baseline metadata.

No third copy of those source schemas is introduced. The local Cargo dependencies are intentionally stacked on the civilian-awareness verifier branch while its PR remains open; this comparison/evaluator PR therefore targets that branch until the parent CAA verifier is merged into main.

## 2. Exact input contract

The evaluator consumes three byte streams:
1. the frozen CAA v0.1 case manifest;
2. the frozen awareness-industry comparison manifest;
3. one Track A run document.

The run binds both manifest SHA-256 values, the exact event-frame digest, run identity, generalized site-profile reference, a common per-case operator-review budget, one baseline workflow ID, candidate IDs and each workflow's results for every case.

Every workflow must have:
- a stable workflow ID and declared workflow class;
- a build/SOP reference and configuration digest;
- the same event-frame digest;
- exactly the same 12 case IDs;
- the same per-case review budget;
- one result record for every case, with no duplicate or missing cases.

Allowed workflow classes:
- `rules_baseline`;
- `manual_site_workflow`;
- `evidence_assistant`;
- `vendor_event_adapter`.

An unavailable/unexportable incumbent is not replaced with fabricated data. A future run must mark that comparison incomparable outside this evaluator.

## 3. Per-case output fields

The consumer records only:
- case ID and reported semantic state;
- whether intent classification was explicitly abstained from;
- provenance completeness;
- preservation of source-health/currentness status;
- contradiction preservation for the five designated ambiguity/dependency cases;
- whether human review was required;
- separation of event time from receipt time;
- correction lineage preservation;
- privacy-compliance state and violation count;
- authority-violation count;
- whether a vendor hostile label was promoted to truth;
- duplicate and unsubstantiated alert counts;
- human review minutes.

The strict Serde schema rejects unknown fields, including added detection-rate, location, trajectory, operator, raw-media or effector fields. The run contains no sensor measurements, live tracks, coordinates, bearings, flight parameters, raw audio/video or third-party commands.

## 4. Metrics emitted per workflow

All rates are reported as integer basis points (10,000 = 100%) with explicit numerator and denominator.

- exact semantic-state match rate against the frozen case contract;
- provenance-complete rate;
- source-status-preservation rate;
- contradiction-preservation rate on the designated ambiguity/common-mode cases;
- intent-abstention rate;
- human-review-required rate;
- event/receipt-time separation rate;
- correction-lineage preservation rate;
- privacy-compliance rate and violation total;
- authority-violation and vendor-hostile-label-as-truth totals;
- duplicate and unsubstantiated alert totals;
- total, mean, median and p95 review minutes;
- cases exceeding the common review-time budget.

No sensor precision, recall, probability of detection, range, tracking error, or hostile-intent metric is emitted. Those are outside Track A.

## 5. Paired comparison

Each candidate is compared with the explicitly named baseline using the same 12 cases and review budget.

The report emits:
- relative reduction in review minutes;
- relative reduction in duplicate alerts;
- relative reduction in unsubstantiated alerts;
- non-degradation of exact semantic-state matches;
- non-degradation of provenance completeness;
- non-degradation of contradiction preservation.

If a baseline denominator is zero, the relevant percentage reduction is `null`, not zero or an invented improvement.

The 20% candidate hypothesis is met only when all three workload/alert reductions are at least 20% and semantic, provenance, contradiction, privacy and authority conditions are preserved. It is a falsifiable hypothesis, not a result.

This Track A hypothesis does not refer to physical event recall. Event recall can only be assessed in the separately authorized Track B with independent physical ground truth.

## 6. Hard rejection conditions

A run is rejected if:
- either source manifest digest differs from the compile-time pin;
- either source manifest fails its existing strict verifier;
- the run or nested structures contain unknown/duplicate JSON members;
- case IDs, event-frame digests, build/configuration references or operator budgets are missing or mismatched;
- a workflow omits or duplicates a required case;
- an output state is outside the awareness contract;
- intent classification is not abstained from on these uncertain/degraded cases;
- any privacy or authority violation is recorded;
- a vendor hostile label is used as ground truth.

Performance-quality shortfalls such as wrong state, incomplete provenance or missed contradictions remain visible in the metrics. They are not silently converted into a green result.

## 7. Output and evidence-plane integration

The CLI emits deterministic JSON with:
- SHA-256 of the exact run input bytes;
- exact source manifest digests;
- per-workflow metrics;
- baseline/candidate pairwise comparisons;
- errors and the explicit claim ceiling.

The CLI report is **not signed** and is not a production evidence receipt. Compose later with Mycelix [shared evidence-plane RFC #5451](https://github.com/Luminous-Dynamics/mycelix/issues/5451) for the approved versioned receipt/attestation, source-rights, reviewer and signing semantics. Do not invent a parallel receipt format here.

## 8. Synthetic smoke fixture

`fixtures/track_a_smoke_run_v0_1.json` is labelled `synthetic_calculator_smoke`, uses synthetic workflow IDs/build refs and a synthetic site ref, and exists only to exercise the evaluator formula, parser and CLI in CI.

Its metrics are not observations of Luminous, a vendor, a sensor, a site or a human operator. They must not be quoted as performance results or included in an industry ranking.

## 9. Qualification and non-claims

The crate's tests verify parsing, digest binding, case coverage, hard-gate rejection, rate calculations, percentiles and paired comparison arithmetic. CI success establishes only this evaluator's local deterministic behaviour against frozen synthetic inputs.

It does not establish:
- real vendor adapter compatibility;
- sensor detection, tracking or identification;
- physical event recall or intent;
- legal permissions or CWA conformity;
- privacy compliance of an actual deployment;
- operational safety, threat protection or industry superiority.

A real Track A run requires actual authorized event-envelope exports, frozen build/configuration references, matched human-review budgets, source-rights records, independent case adjudication and the shared evidence-plane receipt. Track B remains separately blocked.
