# Federated Payment Fraud Signal Interoperability Profile v1

**Status:** synthetic/effects-disabled interoperability contract. This is not a fraud model, a national reporting service, a global blacklist, or a regulatory filing endpoint.

## South African programme context

SARB's 2025 Payments Ecosystem Modernisation position paper proposes that a future National Payments Utility provide baseline fraud protection, supported by participants reporting suspected and actual fraudulent transactions and transaction-level risk scoring across channels and participants. The 9 April 2026 PEM dialogue reported the fraud framework developed with priority items being scoped and insights-platform stakeholder alignment in progress. SARB also lists a Fraud and Intelligence Information Sharing Forum among private-sector PEM participants.

These are programme design/progress facts, not evidence of an operational national scoring API. Mycelix has no basis to claim it is a participant, supplier or endorsed provider.

## Governing distinction

`authenticated participant != legitimate transaction`

`suspected fraud != confirmed fraud`

`risk score != factual finding`

`fraud signal != legal liability`

`local recipient decision != source-status change`

The signal envelope preserves provenance, confidence, freshness, purpose, recipient scope, correction and appeal. A relay may forward a source signal but cannot promote it. A recipient may assess locally without rewriting the source's status.

## Source status and event semantics

Source status is append-only. Only an authorised origin participant may submit or change a source status. Relay and recipient events use `source_status_effect=unchanged`. Confirmation requires independent case evidence; a model score alone is insufficient. Disproof, withdrawal and correction create successor evidence rather than rewriting the previous signal.

A correction may supersede an earlier signal and must be propagated to known recipients. Until all required recipient acknowledgements are present, propagation remains pending; stale copies must not be presented as current.

## Privacy and purpose

- The public synthetic corpus uses pairwise subject references (not unsalted hashes of phone numbers/account numbers).
- Raw names, account numbers, phone numbers, credentials and narrative evidence are excluded from the signal envelope.
- Each signal is scoped to an explicit purpose and recipient audience.
- A recipient request for credit scoring, marketing or unrelated disclosure is not an admissible payment-fraud purpose.
- Do not create a permanent public accusation ledger or universal cross-bank person identifier.

## Risk scores

A numeric score is optional. When present it must carry model/version, calibration ID, cohort, evaluation window, uncertainty and calibration state. A score is a model output, not a source finding or local authority. Do not treat reports with a common origin/case commitment as independent evidence.

## Benchmark gates

- 0 relay/recipient status promotions from suspected to confirmed;
- 0 source-status mutations; corrections/disproofs/withdrawals emit new append-only events;
- 0 source reports from unauthorised or revoked identities shared as current;
- 0 expired or unavailable signals presented as a clean result;
- 0 duplicate/common-origin reports counted as independent evidence;
- 0 uncalibrated or stale scores presented as calibrated risk;
- 0 risk scores used as factual confirmation or external-effect authority;
- 0 raw PII, unsalted stable account/phone hashes or out-of-purpose disclosure;
- 100% corrections, withdrawals, disputes and appeals remain independently auditable;
- deterministic replay of frozen signal and event histories.

Live metrics, only with authorised participant data: signal acknowledgement latency, time to containment, precision/recall at fixed false-positive rates, PR-AUC, calibration (Brier/ECE/reliability curves), false-positive block and appeal reversal rate, false-negative loss, correction propagation/stale-cache rate, N-1 participant generalisation, channel/time-cohort drift, privacy exposure and operator burden. Disclose denominators and case mix; do not publish a single unqualified accuracy score.

## Claim ceiling

The corpus is synthetic. Passing proves only that the evaluator rejects modeled status laundering, scope, provenance, privacy, score and correction failures. It proves no live fraud detection, no false-positive rate, no national service integration, no legal reporting compliance, no production privacy or SARB endorsement.
