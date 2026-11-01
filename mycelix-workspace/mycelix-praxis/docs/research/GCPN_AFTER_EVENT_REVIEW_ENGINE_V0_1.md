# GCPN After-Event Review Engine v0.1 — Stage B

**Status:** Rust source authored; compile/test/CI remain outstanding.  
**Parent issue:** #5656.  
**Specification PR:** #5657.  
**Package:** benchmarks/gcpn-after-event-review.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented review core

This increment turns part of the AER specification into a deterministic synthetic Rust library:

- plan milestones with stage, planned time and plan version;
- observed evidence events with event time, transaction/knowledge time and source reference;
- historical timeline reconstruction at a transaction-time cutoff;
- planned-versus-observed milestone classification;
- stage and plan-version mismatch detection;
- causal-claim levels separated into chronology, association, process contribution and causal effect;
- explicit causal fields for target, assumptions, confounders, outcome measurement, counterfactual and limitations;
- evaluator-only known-DGP causal truth, kept separate from the review claim;
- as-of evidence visibility check for causal claims;
- system-level findings with alternative explanations and unresolved state;
- recommendations with evidence references, system issue, owner role and measurable follow-up;
- independent summary counts, without a composite performance score.

The API has no personal blame, personal suitability or individual-ranking field.

## 2. Bitemporal reconstruction

Each evidence event preserves:
- event/valid time (event_at);
- transaction/knowledge time (recorded_at);
- source identifier;
- plan version;
- fixture verification status.

The reconstruct_as_of(events, T) function admits only events recorded at or before T and orders the visible evidence deterministically by event time and evidence ID.

An event that occurred at time 100 but was not recorded until time 300 is absent from a time-200 review snapshot. It appears in a later snapshot without being backdated into what the reviewer could have known earlier.

Plan version is part of the event/milestone match. A record associated with a different plan version or review stage cannot be silently counted as success against the selected milestone.

## 3. Causal calibration

The engine refuses to promote chronology or association into causal claims. A causal-effect claim must include:
- a target question;
- causal assumptions;
- confounder consideration;
- outcome measurement;
- counterfactual definition;
- data limitations;
- evidence IDs;
- non-empty proposed cause/effect.

The evaluator compares the complete claim against a separate synthetic known-DGP truth object. If the effect is not identified by the DGP, the assessment remains CausalEffectUnidentified. If the evidence supporting a claim was not visible at the selected as-of cutoff, CausalEvidenceNotVisibleAsOf is returned.

Known DGP truth is an evaluator fixture. It must not be disclosed to a reviewer during a blinded review trial.

This is a small conformance oracle, not a general causal inference engine. It does not estimate causal effects from observational data.

## 4. Recommendation traceability

Each recommendation must point to evidence visible at the review cutoff and include:
- a system-level issue;
- an owner role (not an individual identity);
- a follow-up due time;
- a measurable follow-up description.

The engine distinguishes missing evidence, unknown/not-visible evidence, missing system issue, missing role and missing follow-up. It does not issue recommendations or rank institutions.

## 5. Authored tests

Twelve Rust tests are included:
1. later-recorded evidence excluded from historical snapshot;
2. deterministic replay and event-time sorting;
3. plan comparison distinguishes on-time, late and missing;
4. unverified event is not counted as on-time success;
5. descriptive claim is not promoted to causality;
6. incomplete causal fields rejected;
7. unknown causal effect remains unidentified;
8. unsupported causal link rejected against known DGP;
9. recommendation requires visible evidence, system issue, role and follow-up;
10. summary preserves unresolved and incomplete status without composite score;
11. later-recorded causal evidence cannot support an earlier as-of claim;
12. plan-version/stage mismatch is not scored as success.

These are authored source tests, not executed results.

## 6. Limitations and next steps

Not implemented:
- full 13-scenario DGP fixture corpus;
- actual event log file format and persistence;
- multi-source causal graph generation and richer confounding mechanisms;
- selection/missing-not-at-random and survivorship-bias simulators;
- alternative explanation coverage/contradiction scoring;
- independent facilitator/reviewer usability study;
- privacy review and role-based access controls;
- robust recommendation dependency/closure workflow;
- integration with the bitemporal graph package.

The first version is a synthetic review-logic core. It must not be used to conduct an actual national AER or evaluate named agencies.

## 7. Validation status

- Isolated Cargo manifest, Rust library and CLI are on the implementation branch.
- Source-level checks and documented oracle semantics reviewed.
- No Cargo build, Rust test execution, CLI run or CI pass claimed because this runtime has no Cargo or rustc.
- No real event, institution, individual or outcome data are used.

## 8. Research alignment and non-claims

The direction is grounded in UNDRR/IFRC/ITU/WMO's After-Event Review: Methodological Guidance on Conducting After-Event Reviews of Early Warning Systems (15 July 2026): https://www.undrr.org/publication/documents-and-publications/after-event-review-aer-methodological-guidance-conducting

The guidance emphasizes structured, learning-oriented, non-punitive review across the warning value chain. This package supports evidence reconstruction; it does not replace facilitation, community participation, national institutions or review governance.

No live incident investigation, legal finding, individual blame, organization leaderboard, responder direction or causal lives-saved claim is introduced.
