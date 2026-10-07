# Continual Adaptation Qualification Spine v1

## Status

Research specification. Non-authoritative. No production autonomous-update permission is created.

## Purpose

Define a reusable evidence and evaluation spine for systems that detect environmental change and may propose or apply model/runtime revisions.

The spine composes:

- deterministic change detection;
- explicit revision proposal/selection/application states;
- matched no-update counterfactuals where branching is valid;
- retention and invariant checks;
- recurrence/re-adaptation tests;
- causal/statistical integrity controls;
- independent qualification.

## Core laws

```
detected change
!=
beneficial update

successful update
!=
useful adaptation

post-update improvement
!=
causal update effect

new-regime performance
!=
retained prior capability

synthetic fork value
!=
physical-world causal effect

matched fork
!=
automatically causal identification
```

## Evidence identity

Keep these independent:

```
RevisionSubject
EvaluationProtocol
QualificationVerifier
ExecutionEnvironment
EvaluationReceipt
```

A revision cannot change the verifier or evaluation policy that qualifies that same revision.

Every material evaluation input receives an exact identity/commitment:

- source/model/checkpoint;
- detector and revision profile;
- context/configuration;
- evaluation manifest;
- seed/randomness profile;
- simulator/world state where applicable;
- evaluator implementation;
- toolchain/environment where reproducibility requires it.

## Revision lifecycle

Use explicit states:

```
RevisionProposed
RevisionSelected
RevisionApplied
RevisionRejected
RevisionValueObserved
RevisionSuperseded
```

A proposed revision cannot acquire observed-value evidence before application.

A selected revision must be selected without using its final evaluation outcome.

## Experimental partitions

Keep these disjoint:

1. calibration/baseline data;
2. detection data;
3. revision-selection data;
4. final evaluation data.

The final evaluation partition is frozen before revision selection.

## Matched counterfactual ledger

When the world is branchable:

```
                         common parent
                              |
                    +---------+---------+
                    |                   |
                 HOLD                UPDATE
                    |                   |
                    +---------+---------+
                              |
                         common eval
                         identity
```

Bind:

- common parent state;
- change/detection episode;
- candidate revision;
- revision-input manifest;
- HOLD identity;
- UPDATE identity;
- evaluation identity;
- randomness/exogenous-input profile;
- metric profile;
- horizon profile.

A failed/diverged branch remains part of the attempted denominator.

## Counterfactual estimands

Distinguish:

### StateAnchoredValue

Both branches are evaluated from an identical restorable state and identical exogenous sequence.

### ClosedLoopValue

Both branches start from a common fork but later interact with an environment, allowing trajectories to diverge.

### NonBranchablePhysicalValue

No exact physical HOLD trajectory exists after the update. Use an explicitly different design such as randomized timing, blocking, comparable replicas, delayed updates, or historical reference.

These estimands must never be silently substituted for one another.

## Confound controls

The baseline campaign must address:

- regression-to-the-mean;
- trigger-selection bias;
- candidate-selection bias;
- update-frequency confounding;
- horizon dependence;
- repeated evaluator looks;
- optional stopping;
- multiple endpoints/candidates;
- fork interference/carryover;
- shared mutable memory;
- shared resources;
- shared evaluator/cache state;
- mutable RNG/exogenous streams.

## Trigger controls

Where meaningful, compare the production trigger with:

```
always update
never update
detector-triggered
rate-matched random trigger
irrelevant-signal trigger
```

Report separately:

- revision value;
- trigger-selection value;
- update frequency;
- missed-learning opportunities.

A trigger may not look superior merely because it spends more update opportunities.

## Decision-point census

Record every eligible decision point:

```
NoChangeDetected
ChangeDetected_NoUpdate
ChangeDetected_UpdateRejected
ChangeDetected_UpdateApplied
```

The denominator for trigger analysis is the complete frozen decision-point universe.

## Statistical integrity

Before evaluation, freeze:

- primary estimand;
- primary endpoint;
- protected endpoints;
- inferential unit;
- candidate universe/budget;
- maximum evaluation looks;
- evaluation horizons;
- stopping rule;
- seed/randomization policy;
- multiplicity/error-control strategy.

An unplanned post-hoc analysis is exploratory, not a replacement primary result.

Repeated observations from one trajectory must not silently become independent samples.

## Protected outcomes

A revision can show positive primary value while failing protected outcomes. That must remain a failure or conditional state under the declared policy.

Protect at least:

- prior-regime retention;
- invariants;
- calibration/residual behavior;
- false-alarm behavior;
- stability after adaptation.

## Recurrence

After a primary A -> B campaign, separately test:

```
A -> B -> A
```

Measure:

- recurrence recognition;
- relearning cost;
- retained A structure;
- retained B structure;
- update churn;
- oscillation;
- stability.

Do not make recurrence part of the first acceptance predicate until its estimator is independently frozen.

## Interference

For every branchable evaluation domain classify relevant state as:

```
SharedImmutable
SharedReadOnly
ForkLocal
ExternallyMutable
Unknown
```

Unknown mutable state blocks a strong StateAnchoredValue claim.

Distinguish:

```
common exogenous input
!=
shared mutable state
```

Any post-fork contamination invalidates the matched-pair label unless the declared estimand explicitly models the interference.

## Divergence accounting

Always expose:

```
attempted forks
successful paired forks
diverged UPDATE forks
diverged HOLD forks
unresolved forks
```

Do not drop inconvenient branches from the denominator.

## Resource normalization

Adaptation mechanisms must not win only through unconstrained resources.

Report or normalize:

- trainable parameter count;
- persistent state bytes;
- replay/storage bytes;
- update count;
- update latency;
- inference latency;
- message/traffic volume;
- peak memory;
- observation interaction budget.

## Scientific-source boundary

Prediction, simulation, replay, pseudo-labeling, and self-generated representations remain their original provenance class.

```
prediction agreement
!= new observation

simulation reconstruction
!= physical calibration evidence

self-generated label
!= independent validation
```

This is particularly important for sensor adaptation and physical-system qualification.

## Qualification progression

```
frozen research specification
        ↓
dependency-light deterministic reference model
        ↓
independent verifier
        ↓
adversarial mutation corpus
        ↓
exact-head hosted execution
        ↓
repeated matched campaign
        ↓
recurrence / retention campaign
        ↓
separate physical validation
```

No later rung upgrades the earlier evidence beyond its declared claim ceiling.

## Current composition

Mycelix:

- #4548 — matched HOLD/UPDATE revision value;
- #4550 — trigger and regression confound controls;
- #4551 — synthetic vs non-branchable physical counterfactuals;
- #4565 — adaptive peeking and multiplicity;
- #4566 — interference/carryover/shared-environment contamination.

Symthaea:

- #55 — stronger continual-compositional validation plan;
- #3361 — Phase-II continual-learning tournament;
- #3395 — uncertainty-guided metaplasticity;
- #3446 — context remapping and representational reserve;
- #5859 — sensor provenance-aware learning;
- #7029 — unified continual-adaptation evaluation spine.

## Claim ceiling

A qualified implementation of this spine establishes only that the declared synthetic evaluation protocol correctly enforces its identities, partitions, counterfactual structure, statistical controls, resource reporting, and protected outcomes.

It does not establish:

- real-world model truth;
- physical causal efficacy;
- generalization beyond the frozen evaluation distribution;
- biological equivalence;
- safe autonomous self-modification;
- consciousness;
- legal or political authority.
