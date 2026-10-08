# Continual-Adaptation Selection and Censoring Research v1

Status: research specification and dependency-light fixture model only.

This seam closes the longitudinal selection boundary identified by Mycelix #4667
and Symthaea #7094. It is intentionally separate from authoritative
qualification and must not be interpreted as a new trust root.

## Problem

A continually adapting process can change whether an attempted episode:

- remains observable;
- survives to a later evaluation horizon;
- reaches another decision point;
- remains eligible for a later decision;
- contributes a target measurement.

Therefore:

    observed-at-horizon != eligible-at-decision
    survived-to-evaluation != unaffected-by-update
    complete-trajectory-only != unbiased revision value

A harmful UPDATE that terminates an episode is still an attempted outcome.
Dropping it because the later measurement is absent is a selection operation.

Target-trial methodology treats eligibility, treatment/assignment, time zero,
follow-up, outcomes, and censoring as explicit design elements. Misalignment can
introduce selection and immortal-time bias, and artificial/informative censoring
requires explicit handling rather than silent complete-case filtering.

Research references:

- Hernan & Robins, *Using Big Data to Emulate a Target Trial When a Randomized
  Trial Is Not Available* (2016):
  https://pubmed.ncbi.nlm.nih.gov/26994063/
- Hernan et al., *Target Trial Emulation: A Framework for Causal Inference From
  Observational Data* (2022):
  https://pubmed.ncbi.nlm.nih.gov/36508210/
- TARGET reporting guideline development (2023):
  https://pmc.ncbi.nlm.nih.gov/articles/PMC10503363/
- Recent methodological review (2026):
  https://pubmed.ncbi.nlm.nih.gov/41793097/
- Example of cloning/censoring/weighting for protocol deviations:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC10053871/
- 2026 target-trial tutorial covering longitudinal censoring weights and
  diagnostics:
  https://arxiv.org/abs/2608.01625

## Executable model

The research fixture model records, for each attempted episode:

- eligibility at the frozen time zero;
- action / assignment;
- terminal state;
- observation end epoch;
- observation at the target horizon;
- censoring reason and explicit basis where needed;
- action-induced and outcome-dependent censoring flags;
- horizon completion;
- whether failure is itself an observed outcome.

It separately records decision points, so later trigger denominators cannot be
reconstructed solely from surviving trajectories.

The protocol binds an exact expected attempt-ID set and decision-ID set. The
reference verifier rejects incomplete or duplicate ledgers before positive
evaluation.

## Censoring classes

The current research dialect distinguishes:

- `None`;
- `AdministrativeCensoring`;
- `InfrastructureCensoring`;
- `ExogenousLoss`;
- `ActionInducedCensoring`;
- `OutcomeDependentCensoring`;
- `ObservationPolicyCensoring`;
- `CensoringUnknown`.

`AdministrativeCensoring` is the only class treated as non-informative by
default in this fixture profile. Infrastructure and exogenous loss require an
explicit predeclared basis. Action-induced, outcome-dependent,
observation-policy, and unknown censoring require a declared frozen
adjustment protocol or remain unresolved.

The verifier does not compute or validate an actual inverse-probability weight,
nor does it identify a correct causal model. It only verifies that an
adjustment protocol is declared in advance with a method identity and frozen
policy fields.

## Time-zero boundary

Time zero is part of the estimand definition.

The fixture protocol freezes `t0` before assignment. A later timestamp is not
allowed to retroactively define the eligible population or follow-up start.

The verifier therefore checks:

    attempt.time_zero_epoch == protocol.time_zero_epoch

before evaluating the claim.

This is intentionally a structural boundary. It does not prove that a physical
system's wall clock or event timestamp is externally trustworthy; that belongs
to the existing temporal/currentness evidence layer.

## Estimand separation

The research model distinguishes:

- `FullEpisodeValue`;
- `DecisionPointValue`;
- `SurvivorConditionalValue`;
- `PerProtocolPolicyValue`;
- `CensoringAdjustedValue`;
- `CensoringUnresolved`.

For full-population estimands, the analysis must include every attempt that was
eligible at time zero.

`SurvivorConditionalValue` is allowed only when the survivor selection rule is
explicit. It is therefore not interchangeable with `FullEpisodeValue`.

`CensoringAdjustedValue` is allowed only when a frozen adjustment protocol is
declared. This verifies protocol declaration, not identification.

`CensoringUnresolved` is a deliberate unresolved state, not a weak positive.

## Decision-point completeness

The decision ledger has its own exact ID census.

For every recorded decision point with `eligible=true`:

    decision.id ∈ analysis.trigger_denominator_ids

A missing eligible decision point is therefore a denominator defect even when
all surviving trajectories look internally consistent.

This catches a distinct failure mode from attempt completeness:

    #4571: did the attempt happen and terminate?
    #4667: did later observation/eligibility select which attempted decisions
           remained available for analysis?

Both can be necessary.

## Complete-case and survivor attacks

The fixed fixture corpus contains explicit cases for:

1. removing an attempted episode;
2. harmful UPDATE causing early action-induced termination;
3. silently dropping an incomplete trajectory;
4. changing the primary analysis to survivor selection;
5. explicitly declaring a survivor-conditional estimand;
6. enabling a complete-case filter;
7. classifying censoring after observing outcomes;
8. censoring with no declared non-informative basis;
9. censoring with an explicit predeclared basis;
10. declaring an unfrozen censoring adjustment;
11. declaring a frozen adjustment protocol;
12. deleting a later decision point;
13. excluding an eligible later decision from its trigger denominator;
14. violating time-zero alignment;
15. using an unknown selection rule;
16. demonstrating that complete-case filtering remains prohibited even when the
    included set itself is unchanged.

The corpus also preserves ordinary outcome failure as an observed outcome
rather than treating terminal failure as missing data.

## Generated compositional campaign

`CONTINUAL_ADAPTATION_SELECTION_CENSORING_GENERATED_PROPERTIES.json` is
generated deterministically in CI from seed `0x53435601`.

The frozen manifest records the exact serialized SHA-256, byte length, generator
version, seed, and case count. The current campaign contains 192 cases across
six families:

- 32 representation-invariance cases;
- 32 attempt-census completeness cases;
- 32 silent-survivor-selection cases;
- 32 informative-censoring cases;
- 32 timing/trigger-denominator cases;
- 32 compositional mutations.

The final family combines multiple attacks in one mutation sequence. The goal is
not to calculate a selection-bias score. The goal is to verify that adding a
second distortion cannot silently turn an invalid analysis positive.

The generator is deterministic and CI byte-compares its output against the
frozen manifest before executing it.

## Policy identity and liveness

The selection policy is claim-relevant research evidence.

The verifier receives the expected Git blob identity externally and recomputes
the policy blob hash from raw bytes. The fixed and generated corpora bind that
exact blob identity.

A six-case policy-liveness campaign mutates one declarative control at a time
and reruns both the Python and Node reference implementations. The campaign
currently exercises:

- full-population selection rule;
- survivor-selection compatibility;
- informative-censoring block;
- explicit basis requirement for non-administrative censoring;
- pre-outcome classification requirement;
- complete-case-filter compatibility.

A policy mutation that causes the declared verdict transition to disappear is
a development failure. This is a liveness test, not a claim of policy
correctness.

## Differential reference model

Two dependency-light implementations are maintained:

- `verify_selection_censoring.py`;
- `verify_selection_censoring.mjs`.

The CI lane runs both against the fixed and generated corpora and compares their
reports.

Agreement means only that the two research implementations agree on this
fixture dialect. It does not establish equivalence to the authoritative
qualification verifier, production runtime semantics, or an external causal
analysis package.

## Claim ceiling

A green research fixture run can establish only that the declared synthetic
fixture protocol detects the specified selection/censoring distortions under
the implemented dialect.

It does not establish:

- universal causal identification;
- absence of unmeasured confounding;
- correctness of a censoring model;
- correctness of any inverse-probability weights;
- physical efficacy;
- deployment safety;
- general intelligence or consciousness;
- legal, political, or institutional authority.

No production behavior, autonomous update authority, or qualification PASS is
introduced by this research seam.

## Composition

This seam is intended to compose with, rather than replace:

- #4548 matched HOLD/UPDATE revision value;
- #4550 trigger/regression confounds;
- #4551 synthetic vs non-branchable physical counterfactuals;
- #4565 adaptive peeking/multiplicity;
- #4566 interference/carryover;
- #4570 action-dependent observation/distribution shift;
- #4571 experiment-attempt completeness and negative evidence;
- #4577 source-to-target transportability;
- #4579 freshness and target-regime drift;
- #4598 intervention semantic consistency;
- #4599 measurement invariance;
- #4617 claim-local evidence ledger;
- #4618 evidence dependence/shared ancestry.

The central causal distinction remains:

    attempt completeness
        +
    selection/censoring integrity
        +
    declared estimand
        !=
    automatic causal truth

The authoritative verifier remains outside this research tree and retains the
final qualification boundary.
