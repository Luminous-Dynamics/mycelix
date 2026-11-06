# Payment Prevalidation Independent Adjudication Protocol v1

**Status:** synthetic protocol-conformance fixture. No real customer data, legal adjudication, or population-level performance estimate is included.

## Why this benchmark exists

Synthetic conformance tests show whether an evaluator rejects seeded mistakes. They cannot establish how often a real system falsely says a payment is ready, wrongly quarantines a legitimate merchant, or confuses a payee-name match with requester legitimacy or claim validity.

The benchmark separates three layers:

1. Frozen source evidence and a minimised case snapshot.
2. Independent adjudicator labels, blind to all system outputs.
3. Incumbent, conventional and Mycelix system assessments and decisions.

The evaluator's expected output is never a ground-truth label.

## Independent labels

Each case records subject/account binding, payee-name comparison, requester identity, requester authority, claim validity, profile eligibility, intent integrity, evidence freshness and final disposition separately.

A name match is not proof of requester authority or debt validity. A no-match is not itself proof of fraud. An unavailable or indeterminate result is not a match.

## Adjudication

Every case has two independent primary adjudicators. Both are blind to incumbent, conventional and Mycelix outputs. A third adjudicator is required when the current primary labels disagree on any dimension.

Corrections are append-only. A correction identifies its predecessor, retains the same adjudicator identity, supplies a reason and leaves the superseded record present. The correction does not count as a third independent vote.

Questions outside adjudicators' legal/contractual competence remain indeterminate or out of scope.

## Leakage-resistant split

Development, calibration and held-out test partitions must be disjoint by merchant, participant, payee identity and time block. Thresholds and primary endpoints are pre-registered and frozen before held-out evaluation. Only development and calibration partitions are available for tuning.

## Metrics

For each system, report confusion matrices separately for every label dimension, plus:
- false-ready count and denominator: system says safe-to-present or warn-and-permit-choice while adjudication says reconfirm, quarantine, reject, ineligible or indeterminate;
- false-quarantine count and denominator: system says quarantine, reject or ineligible while adjudication says safe-to-present or warn-and-permit-choice;
- payer-warning omission count and denominator;
- independent primary-adjudicator agreement and Cohen's kappa where defined;
- missing/indeterminate label counts and cluster counts.

Do not collapse these into one accuracy score. False-ready and false-quarantine are different risks. A system that prevents false-ready by blocking every legitimate small merchant is not a successful prevalidation system.

## Confidence intervals

The protocol specifies cluster bootstrap resampling by merchant and time block. The evaluator suppresses intervals on this synthetic fixture and whenever the minimum independent-cluster count is not met. Transaction-level Wilson intervals are not a substitute for clustered inference.

A real comparison must report numerators, denominators, missingness, cluster counts, resampling method/seed, sample size and interval for each endpoint.

## Baselines

Use identical evidence and independent labels for an incumbent participant/bank journey, a conventional provider integration and Mycelix. Compare false-ready, false-quarantine, warnings, manual review, user time and cost within matched case mix.

## Claim ceiling

A passing schema/evaluator/mutation suite proves only that this synthetic adjudication contract and its report behave as specified. It does not establish real-world accuracy, fraud prevention, legal debt validity, fairness, superiority or customer benefit. Those require authorised data, independent qualified adjudication, a pre-registered representative study and privacy/legal review.
