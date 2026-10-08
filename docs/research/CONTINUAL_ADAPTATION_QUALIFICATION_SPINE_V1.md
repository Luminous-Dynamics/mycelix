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
- evaluator identity and state integrity;
- action-dependent observation and distribution-shift controls;
- independent qualification.

## Core laws

    detected change != beneficial update

    successful update != useful adaptation

    post-update improvement != causal update effect

    new-regime performance != retained prior capability

    synthetic fork value != physical-world causal effect

    matched fork != automatically causal identification

    independent evaluator != correct evaluator

    same initial state != same future observation process

## Evidence identity

Keep these independent:

    RevisionSubject
    EvaluationProtocol
    QualificationVerifier
    ExecutionEnvironment
    EvaluationReceipt

A revision cannot change the verifier or evaluation policy that qualifies that same revision.

Every material evaluation input receives an exact identity/commitment:

- source/model/checkpoint;
- detector and revision profile;
- context/configuration;
- evaluation manifest;
- seed/randomness profile;
- simulator/world state where applicable;
- evaluator implementation and state;
- toolchain/environment where reproducibility requires it.

## Revision lifecycle

Use explicit states:

    RevisionProposed
    RevisionSelected
    RevisionApplied
    RevisionRejected
    RevisionValueObserved
    RevisionSuperseded

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

    common parent
         |
    +----+----+
    |         |
    HOLD    UPDATE
    |         |
    +----+----+
         |
     common eval
     identity

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
- mutable RNG/exogenous streams;
- action-dependent observation;
- policy-induced distribution shift;
- censoring/attrition changes;
- irregular decision timing.

## Trigger controls

Where meaningful, compare the production trigger with:

    always update
    never update
    detector-triggered
    rate-matched random trigger
    irrelevant-signal trigger

Report separately:

- revision value;
- trigger-selection value;
- update frequency;
- missed-learning opportunities.

A trigger may not look superior merely because it spends more update opportunities.

## Decision-point census

Record every eligible decision point:

    NoChangeDetected
    ChangeDetected_NoUpdate
    ChangeDetected_UpdateRejected
    ChangeDetected_UpdateApplied

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

## Experiment-attempt completeness

A frozen protocol and independent evaluator are still insufficient when execution history can be selectively omitted.

The canonical campaign evidence must make this chain reconstructible:

    campaign plan
        ->
    started attempts
        ->
    terminal dispositions
        ->
    confirmatory analysis
        ->
    human-facing report

Before execution, commit the campaign plan, including:

- campaign identity;
- exact protocol/evaluator identities;
- candidate universe or generation budget;
- planned decision points/slots;
- seed/randomization policy;
- evaluation horizons;
- terminal taxonomy;
- stopping rules;
- retry policy.

For every started attempt, preserve:

- exact campaign and attempt identity;
- subject/candidate identity;
- evaluator and execution-environment identity;
- seed/randomization identity;
- terminal disposition;
- failure/divergence reason where available;
- artifact/trace roots.

The verifier should reconcile:

    planned slots
    =
    started attempts
    +
    explicit unstarted slots
    +
    declared protocol-level exclusions

Every started attempt must have exactly one terminal disposition.

### Retry and negative-evidence rules

A retry creates a new attempt identity and retains the original attempt.

Do not overwrite a failed, divergent, null, harmful, or infrastructure-indeterminate attempt with a successful retry.

Preserve:

- null results;
- negative deltas;
- harmful updates;
- no-op candidates;
- failed adaptation;
- divergence;
- invariant failures;
- safety-triggered stops;
- infrastructure-indeterminate attempts;
- rejected candidates;
- unused planned slots with an explicit reason.

Campaign supersession creates a new campaign identity. Historical plans and attempts remain immutable evidence.

Deleting, replacing, canceling, rerunning, or selectively reporting an attempt must leave a detectable gap in the campaign census rather than silently improving the confirmatory result.


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

    A -> B -> A

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

    SharedImmutable
    SharedReadOnly
    ForkLocal
    ExternallyMutable
    Unknown

Unknown mutable state blocks a strong StateAnchoredValue claim.

Distinguish:

    common exogenous input
    !=
    shared mutable state

Any post-fork contamination invalidates the matched-pair label unless the declared estimand explicitly models the interference.

## Evaluator integrity

Treat the evaluator as a first-class experimental subject rather than an invisible oracle.

A confirmatory evaluation must bind, as applicable:

- evaluator implementation/version;
- executable/container/build identity;
- metric definitions and orientation;
- preprocessing/transforms;
- aggregation rules;
- missing-data/censoring rules;
- outlier and failure handling;
- reference/oracle dataset or generator identity;
- hidden-test partition and access policy;
- threshold/tolerance profile;
- statistical method identity;
- multiplicity/peeking policy;
- evaluator configuration;
- evaluator cache/memory/model state.

Keep these roles separate:

    TheoremSubject
    QualificationVerifier
    EvaluationEvaluator
    EvaluationExecutionEnvironment
    QualificationReceipt

Where independence is required, the candidate must not modify evaluator code, evaluator thresholds, hidden references, evaluator memory/cache, failure classification, or final qualification decisions.

Self-evaluation may remain exploratory, but it is not independent qualification evidence.

### Evaluator mutation controls

The adversarial corpus should include:

1. threshold changes after observing a favorable Delta;
2. adding a metric after observing failure;
3. deleting an unfavorable metric;
4. UPDATE-only preprocessing;
5. candidate output warming evaluator cache/model state used for HOLD;
6. shared mutable evaluator RNG;
7. candidate output used to train a reference/evaluator;
8. hidden-test routing changed after intermediate results;
9. failure/divergence handling changed after inspection;
10. prior campaign results silently fed into the current score.

A confirmatory run must fail closed when evaluator identity/state continuity cannot be established.

### Selection firewall

Record:

- candidate generation identity;
- candidate universe/budget;
- selection rule;
- information available at selection time;
- evaluator information available before selection;
- whether any evaluator output crossed the selection boundary.

If the same held-out outcomes informed candidate selection and final confirmation, classify the result as exploratory unless a separate validation corpus exists.

### Hidden-reference rule

A protected reference corpus remains outside the candidate's optimization channel.

The receipt commits the reference identity, partition, access policy, sampling procedure, oracle version/digest, and write permissions.

## Divergence accounting

Always expose:

    attempted forks
    successful paired forks
    diverged UPDATE forks
    diverged HOLD forks
    unresolved forks

Do not drop inconvenient branches from the denominator.

## Action-dependent observation and distribution shift

An UPDATE can change the process that generates later observations. Consequently, a HOLD/UPDATE comparison over a long adaptive trajectory may mix:

- direct update effect;
- process-state mediation;
- observation-policy change;
- action-dependent missingness/censoring;
- changed future decision opportunities;
- passive environmental drift.

The evidence packet must keep these effects explicit.

### Time-indexed causal ledger

At each decision epoch, record where applicable:

- pre-treatment state;
- decision/trigger information;
- candidate/update action;
- post-action state;
- observation policy;
- observed data;
- missingness/censoring state;
- next decision eligibility;
- outcome.

Temporal order is part of the evidence identity.

Post-update variables must not be silently treated as baseline covariates. Whether they are confounders, mediators, outcomes, or part of a closed-loop policy value depends on the declared estimand.

### Observation-policy identity

Bind:

- sensor modalities;
- sampling frequency/timing;
- trigger-dependent sampling;
- feature extraction;
- filters/windowing;
- dropout rules;
- manual/operator observation rules where applicable;
- logging/capture conditions;
- data inclusion/exclusion;
- whether UPDATE changes any of the above.

A revision that changes what is observed changes the treatment environment as well as the process.

### Distribution-shift taxonomy

Report separately:

    PassiveDrift
    ActionInducedShift
    ObservationPolicyShift
    CensoringShift
    SupportViolation
    ShiftUnresolved

Do not collapse all distribution movement into one generic drift scalar.

### Support and positivity

For regime comparisons that use weighting, modeling, or transport across histories, declare support/positivity assumptions before execution.

Record diagnostics such as:

- observed history support by decision point;
- action frequency by stratum;
- trigger-rate differences;
- rare/unreachable histories;
- censoring/attrition by regime;
- extrapolation outside observed support.

Post-hoc weighting must not be introduced merely to turn an unfavorable result favorable.

A support failure narrows the estimand or blocks the affected inference rather than being normalized away.

### Passive-drift controls

Where the environment can drift without UPDATE, expose:

- time/sequence effects;
- batch/order effects;
- exogenous environment changes;
- maintenance/calibration changes;
- resource depletion;
- background process mutation.

An improvement that exists only because it happened later in the sequence is not automatically revision value.

### Action-dependent observation fixtures

Include synthetic fixtures where UPDATE intentionally:

1. changes the measurement distribution;
2. changes sensor trigger frequency;
3. changes missingness/censoring;
4. changes which future states are sampled;
5. changes event timing;
6. changes the opportunity set for later updates;
7. changes resource availability and observation cadence.

Expected behavior is explicit shift classification, estimand narrowing, or fail-closed inference where the declared protocol cannot support the intended claim.

### Timing and decision-point controls

Bind:

- decision timestamps/epochs;
- waiting time since the prior update;
- trigger-to-update latency;
- observation-to-decision latency;
- time under HOLD;
- time under UPDATE;
- whether timing is policy-controlled or outcome-responsive.

Two episodes with the same number of updates are not necessarily the same treatment regime when timing differs materially.

### Estimand families

Make room for:

- StateAnchoredValue;
- ClosedLoopValue;
- ObservationPolicyEffect;
- PhysicalSequentialRegimeValue.

For non-branchable physical systems, use a sequential experimental/observational design appropriate to the physical setting rather than relabeling it as a rewinded counterfactual.

## Transportability and target-regime boundary

Internal validity does not establish applicability beyond the evaluated regime.

Keep separate:

    source-domain validity
    in-domain generalization
    cross-regime generalization
    transported effect
    operational admissibility

Every extension claim binds a target-regime identity, including as applicable:

- population/process/environment identity;
- observation policy;
- action/update opportunity structure;
- support/covariate envelope;
- temporal regime;
- hardware/provider/runtime differences;
- protocol/measurement differences;
- explicit exclusions.

A target validation corpus is distinct evidence. It must remain outside candidate generation, selection, tuning, evaluator adaptation, threshold optimization, and model-choice feedback for the source campaign.

For causal adaptation claims, record whether effect heterogeneity may arise from baseline state, environment, sensor/process regime, architecture family, resource constraints, observation policy, operator/context, or temporal regime.

Declare the bridge assumptions needed for any transfer claim. Unknown or violated support/positivity, treatment-semantic compatibility, observation compatibility, or relevant effect-modifier coverage narrows or blocks the transported claim.

### Transport controls

Include fixtures for:

1. covariate shift with stable mechanism;
2. mechanism shift with stable predictive metrics;
3. observation-regime shift;
4. changed update-opportunity frequency;
5. causal-effect reversal in the target regime;
6. target histories outside source support;
7. hidden effect modifiers;
8. target-specific censoring/missingness;
9. stable OOD prediction metric with changed intervention effect;
10. operationally distinct conditions hidden behind superficially similar benchmarks.

A transport result must distinguish predictive/OOD performance from transportability of the intervention or update effect.

Use explicit claim states such as:

    SourceQualified
    InDomainHoldoutValidated
    CrossRegimeValidated
    TransportSupported
    UnsupportedTarget

A failed target validation is immutable evidence against the corresponding transfer hypothesis. Target feedback may motivate a new development campaign, but must not silently mutate the source qualification artifact.

Do not create a universal generalization or transportability scalar.


## Intervention-semantic consistency

A source and target regime must not silently treat the same nominal UPDATE label as the same intervention.

Bind, as applicable:

- intervention/update identity;
- exact action semantics;
- target surface;
- timing and duration;
- dose/bounds;
- preconditions;
- execution/runtime/actuation mapping;
- fallback/rejection behavior;
- resource budget;
- realized-versus-requested action.

Distinguish:

    SameInterventionVersion
    BoundedSemanticVariant
    MaterialInterventionVariant
    TranslationRequired
    InterventionUnresolved

A requested action and the realized action are separate evidence objects.

Direct transport is permitted only when intervention identity is compatible under the declared profile. Otherwise use an explicit intervention-translation hypothesis followed by target validation.

This boundary is required because treatment-version consistency is a foundational transport assumption; distinct versions of an exposure can represent different causal interventions. citeturn617824search4turn617824search5

## Measurement invariance

Source/target comparison also requires measurement compatibility.

Bind, as applicable:

- observable/construct identity;
- sensor/instrument identity;
- calibration profile;
- sampling timing/frequency;
- preprocessing/filtering;
- aggregation;
- unit/scale transformation;
- missingness/censoring semantics;
- reference/ground-truth source;
- measurement uncertainty;
- operator/manual scoring rules;
- threshold/tolerance semantics.

Distinguish:

    MeasurementInvariant
    BoundedMeasurementVariant
    CalibrationTranslationRequired
    MeasurementShifted
    MeasurementUnresolved

Do not infer comparability from metric name, unit, or score range alone.

When measurements differ, use:

    source measurement
        ->
    declared measurement mapping
        ->
    target measurement
        ->
    mapping validation

A mapping that predicts well does not automatically establish that causal-effect comparisons remain valid.

Transportability literature explicitly identifies differences in implementation and outcome measurement as sources of external-validity problems. citeturn617824search2turn617824search7


## Freshness and target-regime drift

Transport evidence is historical evidence about the exact validated subject, target regime, evaluator, observation process, and applicability profile.

Therefore:

    transport-supported
    !=
    permanently current

Current operational applicability requires a distinct currentness assessment.

A currentness profile should bind, as applicable:

- source qualification identity;
- target-regime identity;
- subject/runtime/hardware identity;
- evaluator/reference identity;
- observation-policy identity;
- environment/process profile;
- validation epoch/window;
- freshness horizon/policy;
- drift detection policy;
- counterexample/invalidation rules.

Track drift dimensions separately:

    SubjectDrift
    EnvironmentProcessDrift
    ObservationPolicyDrift
    TargetPopulationDrift
    EvaluatorReferenceDrift
    ProtocolPolicyDrift
    HardwareRuntimeDrift
    EvidenceCounterexampleDrift

Freshness triggers may include elapsed policy horizon, detected regime shift, support degradation, sensor/calibration change, evaluator/reference change, material hardware/runtime change, newly discovered failure modes, changed update semantics, or unexplained performance/calibration drift.

A trigger restricts or suspends the affected claim until the policy-required evidence is restored.

Do not refresh a freshness timestamp without new qualifying evidence. Do not let a later positive campaign silently erase a prior failed transport assessment; create a new lineage and preserve the historical result.

Existing Mycelix currentness, applicability, revocation, change-impact, and evidence-lineage mechanisms remain authoritative. This spine composes with them rather than defining a second freshness oracle.


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

If the evaluation mechanism itself is part of the proposed deployed architecture, account for its resource cost separately rather than hiding it in the baseline.

## Scientific-source boundary

Prediction, simulation, replay, pseudo-labeling, and self-generated representations remain their original provenance class.

    prediction agreement != new observation

    simulation reconstruction != physical calibration evidence

    self-generated label != independent validation

This is particularly important for sensor adaptation and physical-system qualification.

## Qualification progression

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

No later rung upgrades the earlier evidence beyond its declared claim ceiling.

## Current composition

Mycelix:

- #4548 — matched HOLD/UPDATE revision value;
- #4550 — trigger and regression confound controls;
- #4551 — synthetic vs non-branchable physical counterfactuals;
- #4565 — adaptive peeking and multiplicity;
- #4566 — interference/carryover/shared-environment contamination;
- #4569 — evaluator identity/state integrity and assessor-adaptation controls;
- #4571 — experiment-attempt completeness and immutable negative evidence;
- #4577 — source-to-target transportability and target-regime validation;
- #4570 — action-dependent observation and policy-induced distribution shift.
- #4579 — freshness and target-regime drift after transport validation;
- #4598 — intervention-semantic consistency across regimes;
- #4599 — measurement invariance across regimes;

Symthaea:

- #55 — stronger continual-compositional validation plan;
- #3361 — Phase-II continual-learning tournament;
- #3395 — uncertainty-guided metaplasticity;
- #3446 — context remapping and representational reserve;
- #5859 — sensor provenance-aware learning;
- #7029 — unified continual-adaptation evaluation spine;
- #7030 — adaptive cognition evaluation with evaluator-state and observation-shift integrity.
- #7041 — target-regime transport of continual-adaptation evidence;
- #7043 — freshness of transported adaptation evidence under regime drift;
- #7056 — intervention-version and measurement invariance for transported adaptation evidence;

## Claim ceiling

A qualified implementation of this spine establishes only that the declared synthetic evaluation protocol correctly enforces its identities, partitions, counterfactual structure, evaluator integrity, action-dependent observation controls, statistical controls, resource reporting, and protected outcomes.

It does not establish:

- real-world model truth;
- physical causal efficacy;
- generalization beyond the frozen evaluation distribution;
- absence of unmeasured confounding;
- biological equivalence;
- safe autonomous self-modification;
- consciousness;
- legal or political authority.
