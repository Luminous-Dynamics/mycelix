# EMB-DATA-VIEW-001T — Synthetic Purpose-Constrained Data View Corpus v0.1

Status: SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS

Parent: EMB-DATA-VIEW-001 (#3024)
Program: EMB-DATA-000 (#2923)
Preregistration: #3034

## Purpose

Freeze provider-neutral synthetic semantics for constructing a bounded embodied-data view from explicit purpose, modality and prerequisite evidence before any runtime planner/materializer exists.

The corpus uses synthetic rights/privacy prerequisite states only. It does not depend on or qualify EMB-DATA-RIGHTS-001A (#2937), EMB-DATA-PRIVACY-001 (#2948), any provider dataset, any downloader, any training system, or any robot runtime.

## Core theorem

`source modality available != modality required != modality admitted != modality exposed`

And:

`view plan exists != source access granted != data materialized != training authorized != publication authorized != physical execution authorized`

## Closed modality vocabulary

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

## Closed result vocabulary

- ViewPlanAdmittedUnderSyntheticPrerequisites
- PrerequisiteUnavailable
- RequestedModalityOutsideReviewedScope
- SplitRoleConflict
- RequiredTransformationUnavailable
- EvidenceConflict
- PurposeChangeRequiresNewView

The positive state is intentionally prerequisite-relative and synthetic. It is not a real-world authorization.

## Data minimization

The planner must start from the explicit requested modality set. Available-but-unrequested modalities remain excluded.

A convenience field such as `include_all_available=true` cannot broaden purpose or scope. If supported at all, it is subordinate to the explicit admitted set.

## Transformation firewall

Some modalities may be exposable only after an exact required transformation such as a reviewed redaction profile. A required transform must be present and bound by identity; the planner cannot substitute a similarly named transform or silently expose the raw source.

Conversion/repacking is not permission expansion. Excluded modalities remain excluded after format conversion.

## Purpose and identity

View identity binds at least:

- exact source subject;
- exact intended purpose;
- split role;
- requested/admitted modality set;
- required transformation profile;
- prerequisite evidence refs;
- retained obligations.

Storage path, shard, serialization or materialized byte layout is not semantic view identity.

Changing intended purpose requires a new view assessment and a new semantic view subject.

## Split firewall

EvaluationOnly and BenchmarkHoldout roles do not become training material through a view. Conversion, derivation or materialization must preserve source split semantics.

## Obligation firewall

Content-equivalent sources or outputs with different obligations remain distinct policy contexts. Attribution, no-redistribution, lineage and similar obligations are retained rather than collapsed.

## Authority ceiling

All of the following are globally false for this corpus:

- source access authority;
- data materialization authority;
- training authority;
- publication authority;
- physical execution authority.

## Frozen cases

The JSON fixture contains exactly 16 synthetic cases V01–V16 covering minimization, out-of-scope modalities, required transforms, missing prerequisites, split conflicts, purpose identity, obligation retention, conflict handling, conversion non-expansion and authority ceilings.

## Downstream rule

A future implementation must consume the frozen corpus through a pure deterministic planner. It should not require network, provider SDK, filesystem, database, Holochain, model-training or robot-control dependencies.

A positive view-plan object should be constructor-controlled. Arbitrary JSON deserialization must not mint an admitted view.

## Claim ceiling

A future qualifier PASS may establish only deterministic synthetic view-planning semantics for this exact docs/fixture subject. It does not grant dataset access/use rights, establish privacy compliance, materialize data, train a model, publish an artifact, establish learned capability or authorize humanoid action.
