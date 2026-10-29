# Open-set calibration transport under population shift

**Status:** synthetic research diagnostic; not a credential-format benchmark or a release gate.

## Research question

A validation-locked threshold controls the empirical false-positive identification rate (FPIR) on the calibration distribution. It does not, by itself, guarantee that the same threshold controls future FPIR when the non-mated score distribution, subgroup mix, or long-tail behavior changes. The next benchmark should test transport of the operating point—not merely repeatability on IID splits.

The W3C Threat Model for Decentralized Credentials records linking and surveillance risks across credential, status, registry, and metadata flows; RFC 9901 also makes clear that selective disclosure does not itself remove correlation available from disclosed claims, other sources, or protocol/deployment behavior. This diagnostic narrows those broad risks into one isolated question: what happens to a frozen open-set decision threshold when its score distribution shifts?

## Executable first experiment

Run:

    python3 research/life-privacy-eval/coverage/open_set_threshold_shift.py --self-test

Defaults: seed 990129, 100,000 calibration queries, 100,000 queries per scenario, target FPIR 0.01. The calibration sample alone selects a nearest-rank threshold. That threshold is frozen across all test scenarios.

Each synthetic score represents the attacker's **maximum non-mated candidate score for one query**. It is sampled directly, so this experiment deliberately does not pretend to simulate real credential presentations, individual gallery candidates, a deployed attack, or a calibrated posterior probability.

Scenarios:
1. No-shift IID control.
2. Mean shifts of +0.25 and +0.50 standard deviations.
3. Score dispersion increases of 1.25× and 1.50×.
4. A 1% +3 SD heavy-tail contamination.
5. A 10% newly observed subpopulation with a +0.75 SD shift, reporting its risk separately from the reference stratum.

Each result reports query count, false-identification count, observed FPIR, and Wilson 95% binomial interval. The subgroup scenario reports the aggregate and each stratum. Threshold selection never reads test scores or test labels.

## Local observation — deterministic simulator run

With the default seed and 100,000 calibration/test observations per scenario, the frozen threshold was 2.3386817605903816. Calibration empirical FPIR was 1.000%.

| Synthetic test distribution | Observed FPIR | Wilson 95% interval |
|---|---:|---:|
| No-shift control | 0.963% | 0.904–1.025% |
| Mean shift +0.25 SD | 1.835% | 1.754–1.920% |
| Mean shift +0.50 SD | 3.345% | 3.235–3.458% |
| Scale shift 1.25× | 2.892% | 2.790–2.998% |
| Scale shift 1.50× | 5.972% | 5.827–6.121% |
| 1% heavy-tail contamination | 1.699% | 1.621–1.781% |
| New shifted stratum is 10% of test mix | 1.454% | 1.382–1.530% |

In the mixed-stratum scenario, reference-stratum FPIR was 0.944% (95% interval 0.883–1.009%) while shifted-stratum FPIR was **6.070%** (95% interval 5.618–6.557%). This is an intentionally synthetic failure case demonstrating why aggregate performance can hide a high-risk stratum. These figures validate only the simulator's expected direction under the chosen data-generating process; they are not estimates of real-world credential privacy.

## What this adds to LIFE

The gallery-scaling benchmark varies the size and draw of a fixed candidate frame. This diagnostic varies the **decision-score distribution** while the threshold remains locked. Together they identify two distinct transport failures: candidate-population growth and score/population drift. The next real-format experiment should combine both, with query-subject, candidate-frame, and time/stratum sampling units declared before the experiment.

Minimum next steps for a real study:
- Pin the attack/scorer artifact and permitted observer features, plus the exact training, calibration, and held-out cohort digests.
- Calibrate on earlier time windows or a declared reference population; test on a later, disjoint cohort and a previously unseen stratum.
- Report the overall and worst-stratum FPIR, calibration resolution, exact or otherwise coverage-qualified rare-event bounds, FNIR/utility, abstention rates, and per-stratum sample sizes.
- Include benign drift, adversarially selected tail cases, subgroup composition change, missing metadata, status-service changes, cross-verifier collusion, and candidate-frame size as separate axes.
- Compare a frozen threshold against a predeclared recalibration policy. Recalibration data must be distinct from the final evaluation cohort; every recalibration must generate a new evidence receipt.
- Retain baseline and mitigation results as paired evidence under identical subject splits and allowed observer views.

Do not repair the observed coverage issue by selecting thresholds on the test set, pooling away the worst stratum, or changing the FPIR target after results are seen. If there is insufficient data to constrain a rare-event rate after shift, the correct state is inconclusive.

## Claim ceiling and reference boundary

This experiment reports a synthetic FPIR transport stress test only. It does not measure target-match FNIR, credential-format behavior, task utility, energy, latency, memory, official conformance, differential privacy, or effective anonymity. Wilson intervals are conditional binomial intervals and do not bound population transport error.

NIST's 1:N biometric identity-proofing FPIR and gallery-size rules are useful references for reporting discipline, but they apply to that defined biometric identity-proofing context—not as universal privacy requirements for credentials. The LIFE benchmark must set profile-specific operational targets and not import an unrelated standard as a universal pass criterion.

References:
- W3C Threat Model for Decentralized Credentials, Group Note Draft (8 September 2026): https://www.w3.org/TR/2026/DNOTE-threat-model-decentralized-credentials-20260908/
- RFC 9901: Selective Disclosure for JSON Web Tokens: https://datatracker.ietf.org/doc/rfc9901/
- NIST SP 800-63A-4, Identity Proofing and Enrollment: https://pages.nist.gov/800-63-4/sp800-63a.html
- NIST: Performance evaluation of fingerprint open-set identification algorithms: https://www.nist.gov/publications/performance-evaluation-fingerprint-open-set-identification-algorithms
