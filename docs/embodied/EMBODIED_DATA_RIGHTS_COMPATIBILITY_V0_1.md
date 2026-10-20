# EMB-DATA-RIGHTS-001T — Synthetic Intended-Use Compatibility Corpus v0.1

Status: SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS

Parent program: EMB-DATA-000 (#2923)
Rights architecture: EMB-DATA-RIGHTS-001 (#2925)
Preregistration: EMB-DATA-RIGHTS-001T (#2936)
Downstream implementation: EMB-DATA-RIGHTS-001A (#2937)

## Purpose

Freeze provider-neutral synthetic semantics for comparing an exact reviewed terms-evidence profile with an intended dataset use before any Rust evaluator is implemented.

No real provider legal text is reproduced here. Every subject and terms profile in the fixture is synthetic. The corpus tests compatibility logic, obligation preservation, uncertainty, split-role restrictions, multi-source composition, contradictory evidence, and authority ceilings.

## Core theorem

`CompatibleWithReviewedTermsProfile` is a narrow machine assessment only.

It is not:

- legal permission;
- provider approval;
- an access credential;
- privacy or consent approval;
- training execution authority;
- publication authority;
- model-release authority;
- robot actuation authority.

A separate human/organizational policy gate may consume the assessment together with exact current source terms, access/account state, privacy/consent evidence, and other obligations.

## Closed assessment vocabulary

V0.1 uses exactly these aggregate states:

- `CompatibleWithReviewedTermsProfile`
- `IncompatibleWithReviewedTermsProfile`
- `HumanReviewRequired`
- `TermsEvidenceConflict`
- `TermsProfileExpiredOrSuperseded`
- `IntendedUseOutOfProfile`

There is intentionally no `Allowed`, `Legal`, `LicenseOK`, or `CanTrain` state.

## Intended-use dimensions

V0.1 keeps use dimensions independent:

- LocalStorage
- ResearchAnalysis
- Training
- Evaluation
- CommercialResearch
- CommercialProductDevelopment
- DerivedModelUse
- DerivedModelDistribution
- DerivedDatasetCreation
- DerivedDatasetRedistribution
- RawRedistribution
- PublicationExcerpt
- PublicDemo
- RemoteProcessing

A positive assessment for one dimension grants nothing about another dimension.

## Obligations

Compatibility returns retained obligations rather than erasing them. The synthetic vocabulary includes attribution, notice retention, noncommercial-only use, no raw redistribution, source-lineage retention, review on terms change, deletion-on-revocation, and no derived-dataset redistribution.

A compatible assessment with obligations is not an unrestricted use.

## Split-role firewall

Dataset split role is independently binding. In particular:

- `EvaluationOnly` and `BenchmarkHoldout` cannot satisfy a Training request merely because generic terms evidence says Training is compatible;
- a conversion, mirror, stream, derivative, or synthetic augmentation cannot silently rewrite the split role;
- changing a split role is a semantic change requiring new evidence.

## Multi-source composition

Every source remains individually attributable.

- all requested dimensions must be compatible for every source before the composite may be compatible;
- one incompatible source withholds a compatible composite;
- one unknown source requires review;
- expired/superseded evidence cannot support a current positive assessment;
- obligations from compatible sources survive composition;
- a permissive source cannot wash a restrictive source.

## Contradictory evidence boundary

Contradiction is distinct from ordinary restrictive composition.

Two different source datasets may legitimately have different rights profiles; the more restrictive relevant source can block a composite use without creating `TermsEvidenceConflict`.

`TermsEvidenceConflict` is reserved for contradictory reviewed evidence about the same exact subject/profile/currentness/use dimension when the evidence cannot be reconciled under the frozen profile. For example:

- review A says Training is `Compatible` for `synthetic:dataset:t:v1`;
- review B says Training is `Incompatible` for that same exact subject/profile;
- neither evidence record supersedes or invalidates the other.

The evaluator must surface the conflict rather than choose last-writer-wins, majority vote, or the more convenient conclusion.

## Privacy and consent boundary

Terms compatibility and privacy/consent are separate prerequisites. A human-subject dataset may be terms-compatible while still requiring an unresolved `PrivacyConsentAssessment`.

The rights kernel must expose that missing external prerequisite; it must not turn copyright/terms evidence into a human-subject approval theorem.

## Currentness and subject binding

A terms profile is bound to its exact synthetic subject/version and currentness state. Profile A cannot assess dataset/version B merely because their names look similar. Expired or superseded evidence cannot remain current-positive.

## Authority ceiling

The corpus globally fixes all of these to false:

- legal permission authority;
- access authority;
- training execution authority;
- physical execution authority.

No caller field may mutate reviewed evidence. `training=true` in an intended-use request expresses what the caller wants to assess; it does not rewrite an incompatible source observation into compatibility.

## Frozen cases

The JSON fixture contains exactly 17 cases covering the original 16 preregistered vectors plus one coverage-completion vector for the already-frozen `TermsEvidenceConflict` state:

1. research compatible while commercial product development is prohibited;
2. training compatible while raw redistribution is prohibited;
3. attribution and source-lineage retention;
4. evaluation-only split blocking training;
5. publicly reachable source with unreviewed/unknown terms;
6. ambiguous training clause requiring review;
7. expired terms evidence;
8. exact subject/version mismatch;
9. two compatible sources with obligation union;
10. noncommercial source blocking a commercial composite;
11. one unknown source withholding a composite;
12. caller request unable to rewrite incompatible evidence;
13. terms-compatible human dataset with unresolved privacy/consent prerequisite;
14. benchmark holdout blocking training;
15. derived-model distribution outside reviewed scope;
16. terms compatibility never creating robot actuation authority;
17. contradictory reviewed evidence for the same exact subject and Training dimension produces `TermsEvidenceConflict`.

C17 is not a new authority state. It closes a preregistration coverage gap: the state vocabulary already contained `TermsEvidenceConflict`, but the original 16-case draft did not exercise it.

## Downstream implementation rule

EMB-DATA-RIGHTS-001A (#2937) must consume this exact corpus through a pure deterministic evaluator. The evaluator should have no provider SDK, browser, HTTP, filesystem, database, Holochain, credential, LLM, model-training, or robot-control dependencies.

Positive result objects should be constructor-controlled. Deserializing arbitrary JSON must not mint a compatibility assessment.

## Claim ceiling

A future qualifier PASS may establish only deterministic representation of this synthetic intended-use compatibility theorem and its exact fixture outcomes.

It is not legal advice, does not interpret a real provider contract, does not grant dataset access/use rights, does not establish privacy/consent compliance, does not authorize training/publication/model distribution, and does not authorize physical execution.
