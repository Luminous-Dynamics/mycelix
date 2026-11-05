# South African Payment Fraud Response Feedback Contract v1

**Status:** synthetic-only recipient response contract; no live signal transport, payment action, legal determination or consumer outcome claim.

## Purpose

A recipient institution must be able to acknowledge a fraud signal, record its local review/action disposition, and return a minimised response without converting its operational activity into shared ground truth.

## Evidence ladder

`delivered != acknowledged != reviewed != action_taken != independently_adjudicated_outcome`

A manual review is not proof of fraud. A step-up challenge passed is not an exoneration. A temporary protective action is not a finding of guilt or a shared legal power. A local case closure is not necessarily a customer remedy.

The contract binds each response to the source signal ID/version/digest; recipient and actor authority; a recipient-pairwise subject reference; a local case commitment and policy revision; purpose; clock-qualified event times; receipt status; processing disposition; local action and local authority; outcome maturity/evidence; correction and appeal references; and minimised feedback-sharing rules.

## Three independent axes

1. **Receipt/processing:** delivered, acknowledged, queued, reviewed, no-action, escalated or delivery-failed.
2. **Local action:** none, step-up, manual review, case opened or protective action under recipient-local authority.
3. **Outcome maturity:** immature, unresolved, independently substantiated, not substantiated, authority determined, corrected, withdrawn or out of scope.

A recipient response can be valid while the underlying case remains unresolved. No-action never means exonerated. Only independently adjudicated evidence may support independently substantiated/not-substantiated; a court/official record and named authority are required for authority-determined.

## Privacy and authority boundary

- Every response uses a recipient-scoped pairwise subject reference and a local case commitment.
- Raw personal data and full statements are forbidden in shared feedback; use minimal reason codes and evidence commitments.
- Feedback is purpose/recipient-bound and time-limited.
- Local actions require local actor authority and a policy reference. A source signal cannot authorise the recipient's action.
- The response itself does not execute a payment, freeze funds, close an account or determine customer liability.
- Pending appeals and unresolved/immature outcomes remain explicit.
- Corrections and withdrawals are successor records; the source signal's state cannot be resurrected by a later recipient response.
- Recipient dispositions are not ground truth. Use #5830's independent adjudication protocol for false-positive/false-negative evaluation.

## Benchmark

Hard gates:
- 100% response binds to exact source signal ID/version/digest, recipient, purpose and local case commitment.
- 0 unacknowledged/delivery-failed events promoted to reviewed/actioned.
- 0 local actions without recipient-local authority.
- 0 step-up success or no-action treated as exoneration.
- 0 internal recipient fraud tag promoted to independent adjudication.
- 0 shared feedback outside purpose/recipient scope or after expiry.
- 0 raw personal information/full statements in shared feedback.
- 0 future or causally unordered timestamps admitted.
- 100% appeal/correction/withdrawal states have their required references.
- 100% duplicate acknowledgements remain idempotent.
- deterministic replay of all 32 synthetic response cases.

Operational studies may measure delivery-to-ack, ack-to-review, review-to-action, case maturity, response-correction latency, false-positive appeal reversal and local operator effort. Denominators and maturity must be disclosed. Do not optimise alert acceptance or local action rate without harm metrics.

## Composition

- #5833: source signal envelope, issuer/purpose/subject scope, freshness and correction.
- #1215: generic typed receipt semantics; this contract reuses that algebra and must not upgrade a receipt into a stronger claim.
- #5766: customer recourse and actual returned-value lifecycle.
- #5768: Request-to-Pay abuse and non-coercive consent.
- #5830: blinded independent ground truth and held-out evaluation.

## Claim ceiling

The corpus is synthetic. A passing evaluator proves only that this response contract rejects seeded false-action, false-adjudication, authority, privacy, replay and time-order mutations. It does not demonstrate a live participant exchange, fraud-detection accuracy, POPIA compliance, lawful action authority or customer benefit.
