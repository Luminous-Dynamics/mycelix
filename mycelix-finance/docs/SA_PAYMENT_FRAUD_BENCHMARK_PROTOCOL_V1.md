# South African Payment Fraud Benchmark Protocol v1

**Status:** source-grounded evaluation protocol and public comparators. No live PSP data, production model or real-money action is involved.

## 1. Why a single fraud score is insufficient

The outcome chain is:

`request → presentation → payer decision → payment instruction → recipient availability → allegation → policy eligibility → recovery/reimbursement → residual loss`

Fraud detection, report volumes, reimbursement eligibility, actual money returned and customer loss have different denominators. Treating them as one "fraud rate" creates misleading comparisons. Treating a VOP/name match as proof of a legitimate request creates claim laundering. Treating all declined requests as "no debt" creates a legal claim Mycelix cannot make.

## 2. Industry and research comparators (observed 4 November 2026)

| Comparator | Published result | What it measures | What it does not establish |
|---|---|---|---|
| UK Open Banking Ltd, June 2026 monitor | 2025: ~1 fraud per 6,000 Open Banking payments vs ~1 per 2,500 across wider UK payments; APP >2/3 of Open Banking fraud cases | Volume-based fraud rate for a provider cohort representing >60% of Open Banking volume | South African PayShap prevalence, RTP request spam, value loss, false-positive rates |
| UK PSR APP dashboard | 7 Oct 2024–31 Mar 2026: 88% (£316m) of in-scope APP scam loss reimbursed; 438,300 claims reported, 301,500 in scope; 82% closed ≤5 business days, 98% ≤35 | Regulatory customer outcomes for UK Faster Payments APP claims | 88% of all reported claims; a South African obligation; every dispute type |
| Pay.UK APP reimbursement policy | In-scope FPS PSPs reimburse within five working days; receiving PSP shares loss cost with sending PSP | Rule/operating mechanism and cost incidence | South African law or consumer entitlement |
| SynSEPA synthetic dataset | ~1.84m synthetic SEPA transactions, 10,000 accounts, 7,112 fraud labels, ~0.39% labelled share | Reproducible synthetic APP stress corpus with four fraud typologies | Real prevalence, production fraud patterns or full device/session-aware detection |
| Harbi 2026 survey | 22 studies; taxonomy spans graph, temporal, edge/federated, class imbalance and auditability; pilot metrics come from telecom CDR | Cross-domain architecture gap and research lead | A payment fraud SOTA result; transferable CDR F1/AUC |

Sources and exact scope are in the machine-readable manifest. There are two important documentation caveats:
- The SynSEPA page reports 1,839,560 rows in its statistics table and 1,849,560 in its footer. We preserve both and mark the row-count discrepancy unresolved.
- SynSEPA discloses no device/session signals and IBANs that fail Mod-97. It may be used as synthetic stress data after integrity review, not as production ground truth.

## 3. Benchmark unit and denominator contract

Report separate populations and frozen denominators:
- requests created, delivered, displayed, accepted, declined, queried, reported;
- payment instructions created, recipient-available/settled payments;
- fraud-alleged, fraud-confirmed, civil-dispute and unresolved cases;
- claims reported, policy-eligible claims, eligible claim value;
- funds requested, located, held, authorised to return, actually returned/reimbursed;
- residual customer and merchant loss.

Keep unresolved/unlabelled observations visible. Report payment-count, account-count, requester-count and amount-weighted rates separately. An eligibility-policy version and the event/label cut-off must accompany every reimbursement rate.

## 4. Evaluation protocol

### Data splits and leakage control

Use a chronological train/calibration/test split plus grouped holdouts by account, requester, merchant, credential/device and fraud campaign. Random-row split alone is prohibited. Freeze the feature-availability timestamp; no post-transaction, post-report, post-adjudication or post-reimbursement fields may enter a pre-payment detector. Keep delayed labels and unknowns explicit. Evaluate cold-start and unseen-payee cohorts.

### Baselines

Compare on identical cohorts and features:
1. deterministic rules-only;
2. calibrated tabular model;
3. temporal sequence model;
4. temporal graph model over account/requester/merchant/payee/credential links;
5. privacy-preserving federated/split-knowledge graph model (only where participant permission exists);
6. human-review/support workflow.

The Harbi telecom CDR pilot is not an appropriate direct baseline for payment metrics; use its five-axis taxonomy as a coverage checklist only.

### Prevalence and adversarial stress

Run prevalence sweeps at 0.5, 1, 2, 5, 10, 39 and 100 basis points (0.005% to 1%). These are controlled stress settings, not estimates of South African incidence. Include:
- APP scams, bank impersonation, romance, investment and invoice/mandate fraud;
- compromised merchant/account credentials;
- first-time/changed payees and confusable names;
- collusive mule/requester graphs;
- abrupt merchant sales spikes and legitimate volume bursts;
- telecom/SIM-swap precursor signals, when legally available;
- drift, adversarial evasion, stale data and label delays;
- low-resource and accessibility channel variation.

### Metrics—never combine into one scalar

Detection quality: PR-AUC, ROC-AUC (with prevalence), precision@fixed review capacity, recall at pre-registered low FPR, amount-weighted recall, Brier score, calibration error.

Operational: p50/p95/p99 decision latency, throughput, queue load, outage/fallback behaviour, feature freshness, time to containment and cost per reviewed case.

Customer/system outcomes: fraud losses per 10,000 recipient-available payments; value loss in basis points; value recovered/reimbursed divided by eligible claim value; residual loss; report-to-acknowledgement and report-to-disposition times; false-positive blocks; appeal reversal rate; merchant exclusion; customer effort and comprehension.

Privacy and governance: raw cross-PSP data exchanged (default target zero); purpose-bound feature lineage; membership-inference/model-inversion and cross-silo leakage tests; poisoning/backdoor robustness; key/currentness/revocation propagation; auditable decisions and appeal correction.

All metrics need cohort, instrument, period, eligibility rule, label maturity and denominator. Publish uncertainty intervals and stratified results.

## 5. Competitive target and hard gates

**Do not optimise acceptance rate or AUC alone.** A useful system should improve amount-weighted outcomes while holding a pre-registered false-positive budget and avoiding exclusion.

Hard gates:
- 0 post-outcome feature leakage;
- 0 request, view, query, decline, report or name-match treated as a payment effect;
- 0 unqualified legal/collection-identity claims;
- 0 stale/refuted/unbound evidence treated as current positive proof;
- 0 silent denominator or eligibility-policy substitution;
- 0 raw inter-participant PII exchange in the default benchmark route;
- 100% unresolved labels, policy versions, and exclusions accounted for;
- every block has a deterministic reason and an appeal/correction path.

**Provisional engineering target:** p99 hot-path risk decision at or below 100 ms for a future real-time scoring path. This is a design target to measure, not a South African scheme SLA or a proven requirement for every RTP product. Fallbacks must fail safely without turning the risk model into a hidden payment gate.

## 6. Path to credible SOTA comparison

Phase 1: reproduce public aggregate baselines and run synthetic stress tests, including SynSEPA only after validating its internal row-count discrepancy and schema limitations.

Phase 2: freeze and publish data manifest, label provenance, time splits, grouped splits, model cards, calibration plots, false-positive/appeal results and confidence intervals. Have independent reviewers reproduce results.

Phase 3: with authorised PSPs, run shadow-mode inference on minimised, pseudonymised data. Do not block payments or train on cross-participant raw data. Compare identical decisions/cases against provider rules and the human-review baseline.

Phase 4: controlled trial only after legal, security, fairness, privacy, accessibility and consumer-recourse review. Measure realised losses, reimbursed value and residual harm, not only model scores.

## 7. Claim ceiling

This protocol plus synthetic data can establish reproducible test behaviour and reveal metric/coverage gaps. It cannot establish South African fraud prevalence, legal reimbursement rights, live provider capability, real-world model performance, or superiority over industry without authorised, representative, independently labelled, time-frozen data.


## 2026-11-04 language, selective-prediction and counterfactual extension

### South African language/modality coverage

The current constitutional Section 6 list, effective 26 June 2026, contains **12 official languages**, including South African Sign Language (SASL). The benchmark now lists all 12 separately, but marks every language/modality as **not yet evaluated** and makes no performance claim.

For the 11 written/spoken languages, evaluation must include native-speaker adjudication, written-name variants, transliteration distinctions, payer-visible material-field comprehension and accessibility tasks. For SASL, a signed visual presentation is a distinct modality: captions, text or audio are not presumed equivalent. A qualified SASL path requires native SASL users and independent review of the material fields. Language coverage is a measurement obligation, not a pretense of equal capability.

### Selective prediction / abstention

A model must be allowed to say “not enough evidence.” Report coverage and selective risk together, across fixed coverage settings (50%, 75%, 90%, 95% in the manifest). Abstention may route to human review, request more evidence, or present a warning while preserving the payer's choice under applicable rules. Do not reward a model for converting uncertainty into an automatic rejection or for reducing false positives by suppressing all low-confidence cases.

### Counterfactual evaluation

Freeze adjudicated pairs in two classes:
- **equivalent presentation variation**: case, whitespace, locale formatting or independently approved name variants that preserve account identity; decisions should remain safe and consistent, allowing a bounded close-match/review result rather than forcing a match;
- **material change**: beneficiary, amount, currency, fee, purpose or expiry mutations; prior acceptance must be invalidated and fresh exact confirmation required.

Include negative controls where the same name maps to another account, phonetic similarity is not sufficient for identity, and verification service unavailability remains indeterminate. Pairs are valid only with independent labels; do not synthesize equivalence by fuzzy string distance alone.

### New metric family

The manifest adds decision coverage, selective risk at fixed coverage, counterfactual equivalence consistency, material-change reconfirmation sensitivity and subgroup calibration error. All metrics remain separate; language/modality and merchant cohorts must be visible. No language-performance score is admissible until an adjudicated, sufficiently sized holdout exists.

Source for the language list: [Current Section 6 of the Constitution](https://www.saflii.org/content/Constitution-of-the-Republic-of-South-Africa_1996.html), reflecting commencement on 26 June 2026.
