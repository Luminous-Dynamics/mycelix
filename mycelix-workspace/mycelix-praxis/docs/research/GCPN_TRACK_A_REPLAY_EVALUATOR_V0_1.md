# GCPN Track A — common-event-envelope replay evaluator v0.1

**Parent:** [GCPN awareness comparison #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Implementation issue:** [#5955](https://github.com/Luminous-Dynamics/mycelix/issues/5955)  
**Status:** local offline evaluator with a synthetic demonstration fixture  
**Claim ceiling:** synthetic replay metric arithmetic only

## 1. Purpose

The comparison protocol already defined Track A, but a protocol is not an evaluator. This implementation adds an offline Rust CLI that calculates separate evidence-workflow metrics over one frozen event frame and multiple system outputs.

It does **not** measure physical sensor detection, use live vendor feeds, identify intent, track aircraft, or command any device. No vendor data or vendor performance results are included in the demo fixture.

## 2. Input contract

The demo run binds:
- one exact synthetic event frame and one ground-truth denominator;
- two synthetic reference consumers only (rules-only and evidence-aware);
- the same case IDs and review budget for both systems;
- an adjudication basis explicitly named `SYNTHETIC_ORACLE_ONLY`;
- one output per event per system;
- review disposition, probability where truth is determinate, review-ticket ID, human-review minutes and event-decision latency;
- provenance, time-quality, dependency/unknown-preservation, privacy and authority flags.

System identifiers and labels are fixed to the two synthetic demo references. Vendor names cannot be inserted into the synthetic run. Actual vendors are not evaluated by this fixture.

## 3. Dispositions

- `QUEUE_HUMAN_REVIEW`: explicitly requests a human-review ticket.
- `NO_HUMAN_REVIEW`: no review ticket is created.
- `DEFER_TO_HUMAN`: the system abstains from resolving the case and routes it for human adjudication.

For indeterminate ground truth, the required output is a defer-to-human disposition and no probability claim. Unknown is not converted to no-review, hostile intent or a fact.

## 4. Metrics and formulas

The evaluator emits disaggregated fields; it does not produce an overall weighted score.

| Metric | Definition |
|---|---|
| Review-required capture rate | (required cases queued or deferred) / all review-required cases |
| Missed review-required cases | required cases assigned `NO_HUMAN_REVIEW` |
| False review queues | no-review-required cases assigned `QUEUE_HUMAN_REVIEW` |
| Review queue precision | review-required queued cases / all explicit review queues |
| False review queue rate | false queues / all no-review-required cases |
| Decision coverage | determinate cases resolved (queue or no-review) / all determinate cases |
| Selective accuracy/error | correct/incorrect resolved determinate cases / all resolved determinate cases |
| Correct abstentions | indeterminate cases deferred to a human |
| Inappropriate indeterminate resolution | indeterminate cases not deferred |
| Brier score | mean squared error of `p_review_required` on determinate labels only |
| Duplicate review requests | extra unique review-ticket IDs in an adjudicated duplicate incident group |
| Human-review minutes | sum once per unique review-ticket ID |
| Budget overrun | max(0, review minutes minus equal per-system budget) |
| Decision latency p50/p95 | nearest-rank percentiles over event-envelope decision latencies |
| Evidence completeness | fraction with provenance, time quality and dependency/unknown handling flags true |
| Privacy/authority | violation counts; either non-zero causes `HARD_GATE_FAILURE` |

Brier score is not computed for indeterminate cases. Calibration on a 12-case synthetic frame is not evidence of model calibration. Confidence is not authority.

Duplicate groups are adjudicated in the fixture. The evaluator does not infer that two observations belong to the same incident from time, place, track or sensor proximity.

## 5. Hard gates

A validly parsed and matched run can still fail the operational policy gate. If privacy or authority violations are non-zero, the report status is `HARD_GATE_FAILURE`, not `CONFORMS_SYNTHETIC_ONLY`.

Other strict rejections include:
- unknown/duplicate JSON fields;
- changed fixture digest;
- missing, duplicate or unknown output event IDs;
- a vendor-named synthetic system;
- unequal event frame / denominator / budget declaration;
- unsupported truth/disposition;
- out-of-range or missing probabilities;
- duplicate review ticket used across different incident groups or with inconsistent minutes.

## 6. Synthetic demo interpretation

The checked-in fixture has two illustrative synthetic references and twelve synthetic events. Any numeric output from this fixture demonstrates arithmetic and report shape only. It is not:
- a measured result for Mycelix, Symthaea or any vendor;
- evidence that one system is better;
- physical drone/robot detection performance;
- field-validated calibration, latency or operator workload;
- a CWA 18150 conformity result;
- evidence of real-world safety or legal permission.

The fixture deliberately exercises a duplicate observation group, review-required, no-review-required and indeterminate cases so that duplicate-ticket, false-review and abstention metrics have non-zero denominators.

## 7. Next qualification steps

1. Independently review the metric definitions and denominator policy.
2. Connect a real Track A event-envelope adapter only after the shared evidence-plane and event schema owners approve it.
3. Pin adapter code, exact event frame, rights/redactions and adjudication revision.
4. Add independent human adjudication and blinded held-out cases.
5. Compare the same frozen envelopes and equal human-review budget against a site incumbent.
6. Preregister superiority/non-inferiority targets and report uncertainty.
7. Preserve missingness and INCOMPARABLE outcomes; do not silently repair or exclude failures.

Track B physical sensor testing remains separately blocked pending the standards, aviation, radio, vendor, site, privacy, ground-truth, insurance and professional approvals in #5940.
