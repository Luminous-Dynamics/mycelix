# SYM-CIVIC-002 — causal falsification benchmark v1

Status: synthetic research benchmark only
Parent: CIV-RES-004 / `424e9342fcea47f982d3c5b9f43ad9a4cce4f1a0`
Tracking issue: #3833
Program: #2006

## Purpose

Provide a reproducible research bridge from qualified CIV-RES observations into Symthaea analysis.

The benchmark tests a mechanism hypothesis. It does not assume that the hypothesis is true, does not convert model output into civic authority, and does not infer person-level danger.

## Hypothesis under test

> Structural adversity + lower reachable option-space + weaker buffering + higher lethal permeability -> greater probability of harmful/irreversible transitions.

The hypothesis is a candidate research statement. Synthetic cases are deliberately constructed to expose situations in which one or more implied mechanisms fail, interact differently, or become observationally indistinguishable from measurement artifacts.

## Required study binding

Every study instance must bind the upstream qualified CIV-RES-004 evidence cut before any causal analysis begins. The binding includes its qualifier commit, workflow run, fixture SHA-256, oracle SHA-256, and canonical receipt SHA-256.

Every study instance must bind:

`study_id + input_snapshot + cohort + geographic_scope + temporal_scope + estimand + model_identity + execution_identity + seed/randomness + uncertainty + missingness_policy + identification_assumptions + diagnostics + sensitivity + spillover/displacement + alternative_explanations + output_disposition`

The benchmark never treats an omitted uncertainty or identification assumption as evidence for the hypothesis.

## Counterexample corpus

CF-01 through CF-10 cover:

1. adversity changes while option-space remains stable;
2. option-space expands without outcome improvement;
3. high social connection exists inside a harmful environment;
4. nominal availability is not practical reachability;
5. multiple nominal pathways share a hidden dependency;
6. downstream lethal permeability changes outcome proxies without upstream crisis change;
7. identical current conditions have different prior trajectories;
8. model prediction is contradicted by later observation;
9. reporting/measurement changes mimic an outcome change;
10. an alternative specification removes the apparent intervention effect.

These are falsification and robustness cases, not evidence that any one mechanism always operates.

## Dispositions

Allowed output dispositions are:

`Supported`, `Contradicted`, `Inconclusive`, `Underpowered`, `Confounded`, `Invalidated`

`Inconclusive` is a valid scientific outcome.

## Semantic boundaries

`Observation != causal effect`
`Model estimate != observation`
`Correlation != intervention effect`
`Prediction != diagnosis`
`Recommendation != authority`
`Aggregate pattern != individual risk`
`Simulation != physical evidence`

The benchmark produces no combined violence, resilience, safety, or individual-danger score.

## Symthaea ownership

Symthaea may formulate hypotheses, execute synthetic analyses, estimate effects, quantify uncertainty, test alternative specifications, and produce falsification dispositions.

Symthaea may not mutate authoritative civic state, authorize interventions, adjudicate institutional claims, diagnose people, or assign individual danger labels.

## Qualification ceiling

PASS establishes only that this exact synthetic benchmark preserves the study-binding, counterexample, uncertainty, provenance, non-authority, and non-person-risk semantics.

It does not establish causal effectiveness in a real population, clinical validity, crime reduction, suicide prevention, violence reduction, municipal legitimacy, or deployment readiness.


## Mandatory causal-threat checks

Before any estimator can report a supported or contradicted mechanism, the study record must explicitly address:

- **Positivity / overlap:** treatment or intervention alternatives must have adequate support in the analyzed scope; structural absence must not be silently extrapolated.
- **Time-varying confounding and feedback:** prior interventions may affect later confounders, and those confounders may affect later interventions and outcomes; adjustment strategy must be declared rather than inferred from a static snapshot.
- **Network interference / spillover:** an intervention affecting one unit may change exposure or outcomes elsewhere in a connected population, so direct, spillover, and total effects must remain distinguishable.
- **Measurement error / misclassification:** observed exposures and outcomes may differ from their latent or intended constructs; sensitivity or validation assumptions must be explicit.
\nThese checks are required because overlap failures, time-varying confounding/feedback, interference, and measurement error can compromise causal identification or bias estimates. The benchmark treats these as analysis threats, not as evidence for or against the civic hypothesis.

## Executable case contract

Each CF-01 through CF-10 fixture now contains a stable seed, explicit design conditions, observed synthetic conditions, all four causal-threat declarations, identification assumptions, and a notes field. Fixtures do **not** contain a candidate scientific disposition or an embedded oracle verdict. The analysis layer must derive its scientific disposition from the study record and diagnostics.

A case fixture is therefore a reproducible adversarial input, not a pre-scored answer. This prevents the benchmark from becoming an oracle disguised as input data.

## Upstream evidence binding

The current synthetic benchmark input cut is fixed to CIV-RES-004 qualifier commit `424e9342fcea47f982d3c5b9f43ad9a4cce4f1a0`, workflow run `37040844685`, fixture `68c3266ee77f59afd5fb9e0d24f0caa13c19b56943102703fb783148239b09db`, oracle `e5fb8b63bd2ca0cc5a2a55649fa9fe9ce31590dc0818bf5bc13e14a1cfcb8722`, and canonical receipt `3e35ec4c9049ccfcabe164548b379ec45404509ae58162e1f765b886ca0bdce7`.

A later study using a different qualified cut must bind that cut explicitly. A model run must never silently substitute a newer or locally modified corpus.
