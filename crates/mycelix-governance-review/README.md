# mycelix-governance-review

A governed boundary between advisory analysis and binding governance procedure.

## Problem

Mycelix already has useful analysis systems: ethics disclosures, collective reflection, concentration warnings, narrow-margin warnings, voting-bloc detection, Phi/reputation analysis, and future Symthaea recommendations.

Those systems can surface important risks. But a model or metric should not silently become a constitutional authority merely because it emits a severe score.

The current architecture contains paths where analysis can escalate thresholds or block proposal advancement. This crate defines the safer replacement contract.

## Core invariant

> An advisory signal may request review. It cannot itself rewrite a tally, revoke a right, change ballot weight, or impose an unbounded block.

A procedural intervention requires all of the following:

1. a review policy adopted **before** the event, bound to institution, jurisdiction, and exact rulebook;
2. a trigger type explicitly allowed by that policy;
3. a still-active review request bound to the original binding-tally digest;
4. an explicit reviewer authority grant carrying the policy's review capability;
5. a time-bounded disposition permitted by the protocol;
6. a host-verified authorization proof.

## Review policy

`ReviewPolicy` binds:

- institution and jurisdiction;
- exact rulebook;
- reviewer capability;
- allowed advisory trigger classes;
- maximum cooling period;
- maximum independent-review period;
- policy lifetime;
- authority grant / proof reference that adopted the policy.

This prevents an advisory engine from dynamically inventing a new power after seeing an outcome.

## Advisory review request

`AdvisoryReviewRequest` records:

- proposal;
- digest of the immutable binding tally;
- advisory signal ID;
- recognized trigger type;
- explanation/artifact reference;
- request lifetime.

The request is intentionally non-binding. It has no method or field that mutates the tally or removes a participant's rights.

## Authorized dispositions

An authorized reviewer may choose only bounded procedural responses:

- `DisclosureOnly` — publish the finding and leave finalization unchanged;
- `CoolingPeriod` — bounded pause, original tally preserved;
- `IndependentReview` — named forum with bounded deadline;
- `SecondBallot` — create a distinct successor proposal/ballot while preserving the original result in history.

There is intentionally no direct `RewriteTally`, `ChangeBallotWeight`, `RemoveVoter`, or `RejectBecauseModelSaidSo` disposition.

## Migration target

The current voting `GovernanceEthicsVerdict::Blocked` and collective-mirror circuit-breaker mechanisms should eventually emit advisory review requests rather than directly escalating the binding threshold or blocking advancement.

The adopted rulebook can still define meaningful safeguards, including cooling periods and independent review. The difference is that the procedural power comes from explicit institutional policy and review authority, not from the score itself.

This preserves the useful sensing layer while keeping the authority boundary legible, appealable, time-bounded, and auditable.
