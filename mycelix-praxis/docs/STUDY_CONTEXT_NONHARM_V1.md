# Praxis Study Context Companion v1

**Status:** Typed contract and synthetic fixture proposal. No learner efficacy result is claimed.

## Purpose

The existing assessment-evidence v1 contract records baseline, practice, immediate post, transfer, and delayed-retention attempts with explicit assistance labels. It deliberately does not equate assisted performance with mastery or intervention benefit.

The October 2026 randomized university trial reported that course-integrated AI-tutor access was associated with lower final grades and substantially lower LMS participation in the studied setting. Another 2026 math field experiment reported faster recovery after mistakes but mixed delayed outcomes. These are different interventions and populations, not a universal verdict on AI. They motivate measuring the study context around the test scores rather than reading scores alone.

Sources:
- Liu et al., *The Effects of Course-Integrated AI Tutoring on Student Performance and Engagement: A Randomized University Trial*, IZA Discussion Paper 18989 / EdWorkingPaper 26-1598 (October 2026): https://edworkingpapers.com/ai26-1598
- Oreopoulos et al., *Making AI Tutoring Productive: Evidence from a Mastery-Based Math Practice Experiment*, NBER Working Paper 35621 (August 2026): https://www.nber.org/papers/w35621

## Contract files

- `crates/praxis-core/src/study_context.rs`: Rust structural validator and binding verifier.
- `examples/assessment-benchmarks/v1/study_context.schema.json`: strict JSON Schema 2020-12 shape.
- `examples/assessment-benchmarks/v1/study_context_synthetic.json`: synthetic-only positive, partial, unavailable, and missing-follow-up observations.
- `crates/praxis-core/tests/study_context.rs`: schema parity, negative controls, assignment invariance, missingness, and exact-artifact binding tests.

This is an **adjacent contract**, not a change to the existing assessment-evidence v1 wire format. Existing v1 remains authoritative for assessed attempts, scores, assistance mode on attempts, and retention/transfer task forms.

## What it records

Each opaque participant reference and study period separates:

- **Assigned arm**: the study condition the participant was assigned to. It does not infer actual use.
- **Support availability**: not offered, available, unavailable, or unknown.
- **Support uptake**: observed invocation count and seconds of use.
- **Ordinary learning activity**: scheduled, attempted, and completed learning tasks, plus independent-practice attempts.
- **Observation status**: complete, partial, not collected, lost to follow-up, or withdrawn.

Periods are baseline, intervention, 7-day follow-up, and 30-day follow-up. Protocol-specific tasks and outcome scores remain in the assessment artifact. Context measures cannot replace the primary independently scored, unaided transfer outcome or delayed retention.

## Missingness is not zero

A numeric zero means the measure was observed and the value was zero. A null means that particular measure was not available. A non-observed period uses null counters and unknown support availability; it must not be imputed as zero participation or zero help.

The validator rejects:
- a complete observation that omits a counter or has unknown support availability;
- partial records with no observed measurements;
- missing/lost/withdrawn records with counters that would disguise missingness as zero;
- attempts beyond scheduled opportunities or completions beyond attempts;
- positive support use when support was not offered, unavailable, or not recorded as available;
- duplicate participant-period records or arm changes across periods.

These are structural consistency checks. They do not establish the truth of self-reported or provider-reported telemetry.

## Binding to assessment evidence

`assessment_run_sha256` must be the SHA-256 of the **exact assessment artifact bytes**. The caller must compute that digest from the actual artifact and pass the independently observed value to `validate_study_context_binding`. The validator compares the declared digest with the supplied observed digest, then checks protocol ID, participant IDs, and assigned-arm correspondence. It validates digest syntax but does not itself hash a file or establish who produced the bytes.

The binding function also validates the supplied assessment run against the current assessment-evidence v1 validator. Caller-side artifact acquisition and the provenance of that acquisition remain separate claims.

## Synthetic fixture and limitations

The fixture contains 12 observations across three synthetic participants, three assigned arms, and multiple periods. It intentionally includes:
- a conventional no-AI comparison;
- AI support that was available and used;
- an assigned AI condition in which support was unavailable;
- a partial intervention record;
- a lost 30-day follow-up and a not-collected 30-day follow-up.

It does not contain real learner records, subgroup attributes, causal estimates, or learning-effect results. It is too small to estimate learning efficacy. Its passing tests establish only that the structural contract and failure rules behave as specified.

## Evaluation protocol implication

For any human-participant study, preregister the target capability, active comparator, primary unaided transfer outcome, delayed-retention timepoints, domain-specific meaningful-effect/non-inferiority margin, sample-size rationale, exclusions, missingness plan, subgroup analyses, and adverse-outcome rules before looking at outcomes. Measure assignment, actual availability, uptake, and ordinary task participation separately. A null result from an underpowered study is not proof of equivalence.

The study design requires appropriate consent, privacy review, and ethics review before real learner data are collected. Do not publish raw learner histories or use this contract to issue credentials or authorization.

## Current validation status

The exact-head Rust formatting/build/test workflow must qualify these additions. A queued or skipped run is not a pass. Even a structural PASS would not establish causal efficacy, learning benefit, non-harm, accessibility conformance, or production readiness.

Related research task: [PRAXIS-LEARNING-NONHARM-001 #5461](https://github.com/Luminous-Dynamics/mycelix/issues/5461).
