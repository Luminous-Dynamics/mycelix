# IM-001A — Public Preregistration Seed

**Status:** experiment-contract draft; no performance result is claimed.
**Question:** can a collaborative defensive-learning protocol improve detection of held-out attack families without centralizing raw client telemetry, while remaining acceptably robust to malicious participants, privacy attacks and false-positive/"autoimmune" regressions?

## Primary corpus

**CSE-CIC-IDS2018** from the Canadian Institute for Cybersecurity / University of New Brunswick.

Record at acquisition time:

- official source URL and citation;
- source/redistribution terms;
- downloaded-file hashes;
- transformation-code revision;
- split/index hashes.

Use CIC-IDS2017 only as an external dataset-shift check after the primary protocol is frozen. CICDDoS2019 is optional for a deliberately scoped DDoS analysis, not automatic scope expansion.

## Phase 0 — calibration only

Use a designated calibration subset to decide:

- feature normalization / compatible feature set;
- number and construction of simulated organizations/clients;
- class-balance treatment;
- named malicious-client regimes;
- numeric decision thresholds for transfer, privacy, false-positive and Byzantine outcomes.

The final attack-family holdout and external-shift evaluation are not inspected here.

## Non-IID organization simulation

Construct client partitions by host/subnet/day or another published deterministic grouping that produces heterogeneous local views. Do not use IID random sharding as the only setting.

## Required comparison arms

1. local-only learning;
2. centralized pooled-data research reference/upper baseline;
3. ordinary collaborative/federated baseline;
4. IM-001 candidate protocol.

This prevents “federated beats nothing” from being counted as evidence.

## Held-out transfer

Freeze at least one complete attack family/sub-family from collaborative training by a deterministic preregistered rule. Do not choose the holdout after seeing which family gives the best result.

## Hostile-participant and privacy regimes

Freeze named regimes before final holdout, including a subset of:

- benign clients;
- malicious-client fractions selected during calibration;
- random / sign-flip Byzantine updates;
- targeted poisoning/backdoor behavior;
- membership-inference or update-leakage attack against the collaboration mechanism.

## Primary outcomes

- held-out-family detection relative to local-only and ordinary federated baselines;
- false-positive / autoimmune regression;
- privacy attack success / leakage metric;
- degradation under named malicious-client regimes;
- clean-task performance cost;
- stability under external dataset shift.

## Freeze point

Before final holdout evaluation, publish or hash-pin:

- train/calibration/holdout indexes;
- model/protocol configuration;
- hostile regimes;
- primary metrics and thresholds;
- random-seed policy;
- analysis revision.

## Interpretation

A successful project does not require a positive result.

- No transfer benefit → reject/weaken the cyber-immunity claim.
- Transfer with unacceptable false positives → report autoimmune failure.
- Privacy leakage or poisoning fragility → report the mechanism unsafe under that regime.
- Benefit that disappears under external dataset shift → constrain the claim to the source environment.

## Non-claims

Public CIC benchmark traffic is not live enterprise/customer telemetry. The experiment tests reproducible research conditions and does not establish production efficacy, deployment safety or universal cyber-immunity.
