# GCPN — Track A deterministic replay evaluator v0.1

**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child:** [GCPN-TRACK-A-REPLAY-EVALUATOR-001 #5947](https://github.com/Luminous-Dynamics/mycelix/issues/5947)  
**Comparison protocol:** [#5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Status:** standalone offline evaluator; synthetic reference bundle only  
**Evidence tier:** synthetic fixture  
**Claim ceiling:** deterministic offline Track A event-envelope metrics only

## 1. Purpose and scope

This crate evaluates downstream evidence/review workflows that receive the same frozen, privacy-minimized event envelopes. It does not evaluate a physical radar, RF, optical, acoustic or LiDAR sensor. It is not an aircraft detector, tracker, identification engine, intent classifier, airspace service, robot controller or response authority.

The sealed evaluator bundle contains:
- the exact event-set digest and event identifiers;
- independent adjudication labels held out from evaluated pipelines;
- one explicit disposition per event for each pipeline;
- exact build, configuration and policy digests;
- human review requests, latency and review time;
- source-quality/dependency-inventory summaries;
- privacy/authority hard-gate counters.

The evaluated pipelines must not receive the oracle labels. They process the same event envelopes; the evaluator joins their outputs to the adjudication bundle offline.

This is a metrics payload for the shared evidence-plane workstream [#5451](https://github.com/Luminous-Dynamics/mycelix/issues/5451), not a replacement run-receipt, trust-root, signature or source-stewardship protocol.

## 2. Industry and standards context

**CWA 18150:2024** is a CEN Workshop Agreement titled *Unmanned aircraft systems — Counter UAS — Testing methodology*. The CEN document explicitly says it is a Workshop Agreement and not an official standard developed by CEN and its Members. It describes a scenario-based evaluation methodology for integrated detection, tracking and identification (DTI) solutions, with evaluation plans, test environments, ground truth, performance metrics and reporting. This evaluator references the CWA as a methodological source; it does not reproduce its copyrighted test method, perform a conformity assessment, or claim CWA compliance.

- Official CEN document: https://www.cencenelec.eu/media/CEN-CENELEC/CWAs/ICT/cwa-18150.pdf
- CWA status/reference page: https://www.nen.nl/en/cwa-18150-2024-en-330385

The CWA's operational and physical test method is relevant to a future, separately authorized Track B assessment. The present Track A evaluator answers a narrower question: given identical event envelopes, how well does each downstream workflow preserve evidence and support human review?

Commercial public product descriptions (e.g. Dedrone RF-360, DroneShield DroneSentry-C2 and Sentrycs DTI) are capability claims, not measured performance. They are not encoded as results in the synthetic fixture.

- Dedrone RF-360: https://www.dedrone.com/sensors/rf-360
- DroneShield DroneSentry-C2: https://www.droneshield.com/products-software
- Sentrycs DTI: https://sentrycs.com/our-technology/

## 3. Evaluation pipeline roles

The synthetic reference fixture uses three abstract roles:
- `RULES_ONLY`: deterministic rule/reference pipeline;
- `SITE_WORKFLOW`: illustrative incumbent human/site workflow;
- `EVIDENCE_ASSISTANT`: illustrative evidence-aware advisory workflow.

These are synthetic examples only. They are not vendor adapters or actual vendor, site or Luminous field results. Real Track A runs must use exact participating pipeline build/configuration digests and a predeclared matched review policy.

## 4. Adjudication labels and workflow dispositions

Oracle labels are deliberately limited to:
- `CONFIRMED_RELEVANT`;
- `CONFIRMED_BENIGN`;
- `UNRESOLVED`.

There is no hostile/hostile-intent label.

Each pipeline must emit one explicit outcome per event:
- `REVIEW_REQUIRED`: a review request was sent to a human;
- `NO_REVIEW`: no review was requested;
- `ABSTAIN`: the pipeline abstained from making a binary relevance decision.

Missing output is not interpreted as `NO_REVIEW`; it makes the comparison `INCOMPARABLE`. Duplicate outputs for a case are malformed and `REJECTED`.

For `UNRESOLVED` oracle cases:
- `ABSTAIN` is counted as unresolved abstention;
- `REVIEW_REQUIRED` is counted as a human referral, not a false positive;
- `NO_REVIEW` is reported separately and is not silently treated as a true negative.

For known-label cases, abstentions are excluded from selective-risk calculation, but they still count as missed events for end-to-end recall when the oracle label is `CONFIRMED_RELEVANT`.

## 5. Metric definitions

All metrics are reported per pipeline, with raw numerators/denominators or count fields preserved. The evaluator does not calculate a single composite winner score.

### Event outcomes

- **True positive event:** `CONFIRMED_RELEVANT` with `REVIEW_REQUIRED`.
- **False negative event:** `CONFIRMED_RELEVANT` with `NO_REVIEW` or `ABSTAIN`.
- **False positive event:** `CONFIRMED_BENIGN` with `REVIEW_REQUIRED`.
- **True negative event:** `CONFIRMED_BENIGN` with `NO_REVIEW`.
- **Known-label abstention:** either known label with `ABSTAIN`; excluded from selective coverage/risk, but not removed from full recall accounting.

`precision = TP / (TP + FP)`

`recall = TP / (TP + FN)`

`false_review_fraction = FP / (TP + FP)`

The CWA 18150:2024 also includes false-alarm-rate and probability-of-detection measures for DTI evaluation. Here the denominator is event-level human-review outcomes from a fixed event set, not raw sensor alarms. These Track A metrics must not be presented as physical sensor false-alarm rate, probability of detection or range.

### False-review burden and duplication

- `false_review_requests`: total human review requests attached to confirmed-benign events, including repeated requests.
- `false_review_requests_per_hour`: false review requests divided by declared monitored hours.
- `duplicate_review_requests`: `max(review_requests - 1, 0)` summed over events.
- `duplicate_false_review_requests`: the same duplicate count, limited to confirmed-benign events.

These measures distinguish one false review event from repeated/duplicated workflow messages.

### Selective prediction and uncertainty

- `known_decision_coverage`: non-abstaining known-label cases / all known-label cases.
- `selective_risk`: false positives plus false negatives among non-abstaining known-label cases / non-abstaining known-label cases.
- `unresolved_abstentions`, `unresolved_human_referrals` and `unresolved_no_review` are separate counts.

No unresolved oracle case may carry a binary relevance probability. A supplied `relevance_probability_micros` is scored only for a confirmed-relevant/confirmed-benign oracle label. The Brier score is the mean squared probability error across the subset for which a probability was actually provided; the scored-case count is always shown. Missing probability is not imputed.

The probability is about *event relevance for human review*, not hostile intent, identity, or authorization.

### Human review

- p50/p95 review latency use nearest-rank percentiles over cases with review requests;
- mean human-review seconds per reviewed case;
- review-budget exceedance count, using the same declared per-case budget for all pipelines.

Report the exact review policy and budget digest. Do not compare a fully staffed system with a zero-human-work assumption.

### Evidence provenance and dependency inventory

The evaluator reports source counts, authenticated/calibrated/time-qualified counts, complete/partial/unknown dependency-inventory cases, conflict preservation and purpose/rights verification.

A complete dependency inventory is not proof that sources are independent. A source count is not a count of independent failure domains. The evaluator never promotes sensor quantity, signatures, reputation or peer count into evidence independence.

### Privacy and authority hard gates

A pipeline is blocked if any count is non-zero for:
- vendor hostile label used as ground truth;
- operator-location export;
- exact flight-path export;
- raw audio/video export;
- third-party command;
- active mitigation;
- privacy violation.

If any pipeline crosses a hard gate, the overall comparison is `BLOCKED` and all comparative metrics are withheld. Accuracy, cost or workload cannot compensate for a privacy or authority violation.

## 6. Dispositions

- **COMPARABLE:** same event-set digest, policy digest, case identifiers, observation duration and complete one-to-one case output across pipelines; ground-truth independence is declared.
- **INCOMPARABLE:** event-set/policy/coverage mismatch or ground-truth independence not established; comparative metrics are not emitted.
- **BLOCKED:** any pipeline crosses a privacy/authority boundary, or a controlled replay lacks required review-receipt references.
- **REJECTED:** strict JSON parse failure, unknown/duplicate field, duplicate case/pipeline identity, invalid metric domain, malformed hash, or contradictory per-case accounting.

A case whose oracle label is `UNRESOLVED` does not make the whole comparison incomparable if that uncertainty is explicitly preserved.

## 7. Input and receipt semantics

The input uses strict typed Serde structures with `deny_unknown_fields`. Duplicate JSON object members are rejected by deserialization into typed structs.

For a synthetic fixture:
- `evidence_tier = SYNTHETIC_FIXTURE`;
- `simulation_only = true`;
- a compile-time SHA-256 pin is required by the default CLI.

For a controlled replay:
- `evidence_tier = CONTROLLED_REPLAY`;
- `simulation_only = false`;
- ground-truth adjudication, privacy review and authorization review receipt SHA-256 references are required.

The current evaluator validates that those receipt references are present and correctly formed; it does not verify the reviewer's identity, signature, professional credentials, legal authority or substantive decision. Those checks belong to the shared evidence plane and independent reviewers.

The tool emits an offline metrics report containing the exact input digest. It does not create a digital signature, evidence receipt, identity, trust root or authorization.

## 8. Synthetic reference fixture

The embedded fixture contains eight abstract events and three reference pipelines:
- three confirmed-relevant cases;
- three confirmed-benign cases;
- two unresolved cases.

It contains no sensor waveforms, coordinates, flight paths, operator identity/location, raw audio/video, private communications or tactical geometry. Its outputs are illustrative and cannot be quoted as vendor or real-site performance.

The default CLI evaluates this exact byte-pinned fixture. Passing tests validate evaluator behavior and its finite synthetic input, not physical or operational effectiveness.

## 9. Qualification targets and next statistical step

The current implementation is a deterministic single-bundle metric calculator. It does not calculate confidence intervals, significance tests, paired bootstrap intervals or a 20% superiority claim.

Before claiming superiority, create a preregistered holdout containing independent site/time blocks or other defensible independent units, not merely correlated event rows. Compare paired pipelines on the same eligible cases and operator budget. Use a paired bootstrap or another justified interval method at the independent block/failure-epoch level. Report absolute percentage-point changes and relative changes, sample counts, missingness and uncertainty. A candidate that does not outperform the baseline must be reported as such.

## 10. Non-claims and future gate

This evaluator does not establish:
- physical sensor detection, probability of detection, false alarms, range or tracking;
- correct identification or intent;
- vendor superiority;
- CWA 18150 conformity;
- South African SACAA/ICASA authorization or POPIA compliance;
- professional approval or operational protection.

Track B remains separately blocked pending site/operator authority, SACAA and ICASA applicability, vendor permissions, privacy/insurance review, an independent ground-truth process, and qualified professional/standards review.
