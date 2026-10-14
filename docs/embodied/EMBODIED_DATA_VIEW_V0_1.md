# EMB-DATA-VIEW-001T — Synthetic Purpose-Constrained View Corpus v0.1

Status: SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS

Canonical parent: EMB-DATA-VIEW-001 (#3018)
Preregistration: EMB-DATA-VIEW-001T (#3019)
Qualified prerequisite evaluator sources: terms #2996; privacy-purpose #3015
Downstream product: EMB-DATA-VIEW-001A (#3039)

## Purpose

Freeze implementation-independent, provider-neutral semantics for producing or withholding modality-minimized embodied-data view plans before any Rust planner, provider parser, downloader, model runtime or robot runtime consumes them.

All fixtures are synthetic. They contain no real provider data, legal text, consent records, credentials, dataset bytes, storage access or effects.

## Core theorem

A view plan composes reviewed evidence. It does not manufacture authority.

`ViewPlanReadyUnderReviewedEvidence` means only that the exact synthetic inputs satisfy this frozen composition theorem.

It is not:
- legal permission;
- consent or privacy compliance;
- dataset-access authorization;
- parser qualification;
- data materialization;
- training authority;
- publication authority;
- robot or physical-execution authority.

## Closed aggregate dispositions

V0.1 uses exactly:
- `ViewPlanReadyUnderReviewedEvidence`
- `ViewWithheldTerms`
- `ViewWithheldPrivacyPurpose`
- `ViewWithheldSplitRole`
- `ViewWithheldTransformation`
- `ViewWithheldModality`
- `ViewWithheldExternalPrerequisite`
- `ViewEvidenceConflict`

Malformed or binding defects are typed errors rather than aggregate dispositions:
- `MalformedRequest`
- `SubjectBindingMismatch`
- `AssessmentBindingMismatch`
- `AuthorityFieldRejected`

## Deterministic precedence

Typed malformed/binding errors are evaluated before aggregate disposition.

For otherwise well-formed inputs, aggregate precedence is:

`EvidenceConflict > Terms > PrivacyPurpose > SplitRole > Transformation > Modality > ExternalPrerequisite > Ready`

Input/source ordering may not alter that outcome.

Case V14 freezes a specific overlap: when an exact reviewed transform is present but its declared output omits a selected modality, the result is `ViewWithheldTransformation`, not an input-order-dependent transformation/modality choice.

## Privacy applicability

Every source declares one of:
- `Required`
- `NotApplicableUnderReviewedSourceProfile`
- `UnknownRequiresReview`

Privacy non-applicability must come from the reviewed source profile. It cannot be inferred from a missing face/audio channel or a caller flag.

## Upstream assessment abstraction

The corpus uses bounded synthetic references (`terms_ref`, `privacy_ref`) and synthetic assessment states. It does not deserialize or mint qualified #2996/#3015 assessment objects.

Future product code must consume non-forgeable qualified assessment types or verified projections. An arbitrary string saying `Compatible` is not a qualified assessment.

## Modality minimization

The frozen modality vocabulary is:
- RgbVideo
- Audio
- Gaze
- HeadPose
- BodyPose
- HandPose
- ObjectPose
- Depth
- Imu
- SpatialMap
- DerivedEmbedding
- DerivedPoseOnly

For each source, the fixture preserves:
- available modalities;
- explicitly selected modalities;
- explicitly excluded/not-selected modalities;
- required transformation refs;
- transformation output modalities.

For every ready source view, every available-but-unselected modality must remain explicit excluded/not-selected provenance.

A provider exposing more channels, a storage converter exposing all fields, or a caller convenience flag cannot broaden the selected set.

## Split roles

Closed roles:
- Training
- Validation
- EvaluationOnly
- BenchmarkHoldout
- Unspecified

Training-oriented purposes cannot consume EvaluationOnly or BenchmarkHoldout evidence merely because terms/privacy evidence is positive.

## Transformations

A transformation is represented by an exact ref, reviewed state and declared output modality set.

A transform can narrow exposure but cannot erase source terms/privacy obligations or source lineage.

Same transform name with another exact profile/ref is a different semantic input.

## Multi-source composition

Every source remains individually attributable.

One permissive source cannot wash a restrictive, unknown or conflicting source.

Ready multi-source plans retain source-attributed obligations and nonclaims. Reordering the same source observations does not change canonical semantics.

## Semantic identity inputs

The semantic identity input set contains:
- canonical exact source subject set;
- upstream assessment refs/profile context;
- experiment purpose;
- split role per source;
- selected modalities;
- explicit excluded/not-selected modalities;
- exact transformation refs and output profiles;
- obligations and nonclaims;
- unresolved external prerequisites.

Temporary storage locators, shard names, cache paths, VRS filenames, ROS bag paths, Parquet row ranges and LeRobot episode indices are not semantic identity.

Changing selected modality, purpose, exact transform/profile ref, assessment ref or obligation/nonclaim context changes semantic identity inputs.

## Nonclaims

Nonclaims stay nonclaims.

For example, `ReidentificationImpossibleNotEstablished` may be retained as provenance. It must never be inverted into `ReidentificationImpossible=true`.

## Caller authority firewall

Caller-authored fields such as:
- `approved=true`
- `training_allowed=true`
- `privacy_ok=true`
- `include_all_available=true`

cannot mint or broaden a ready plan. Unknown permission-like authority fields are rejected or ignored for authority according to the frozen typed-error theorem.

## Frozen corpus

The JSON fixture contains exactly 30 cases V01–V30 covering:
1. minimized ready human-subject view;
2–3. terms withheld states;
4–7. privacy-purpose withholding/applicability;
8. explicit non-human privacy non-applicability;
9–10. split-role firewall;
11. selected modality unavailable;
12. malformed selected+excluded overlap;
13–14. transformation failures and precedence;
15–16. evidence conflicts;
17. unresolved external prerequisite;
18–20. multi-source composition;
21. source-order invariance;
22. storage-location identity exclusion;
23–25. modality/purpose/transform semantic identity changes;
26. caller authority-field rejection;
27. obligation retention;
28. nonclaim retention;
29. provider exposure cannot broaden selection;
30. absence of fetch/train/publish/control authority.

## Downstream product rule

Only after this exact source is independently qualified may #3039 implement the pure planner.

The product should be a small standalone crate with no Holochain, network, filesystem, provider SDK, data materializer, model-training runtime or robot-control dependency. Positive ready-plan types should be constructor-controlled and should not be deserializable from arbitrary JSON.

## Claim ceiling

A future qualifier PASS may establish only deterministic representation of this exact synthetic view-planning theorem and fixture outcomes.

It does not qualify any provider dataset/parser, grant rights or consent, establish privacy/legal compliance, fetch or materialize data, train or evaluate a model, prove a skill, authorize publication, or authorize humanoid action.
