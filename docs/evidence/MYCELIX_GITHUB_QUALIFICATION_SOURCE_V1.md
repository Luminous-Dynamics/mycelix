# MYC-EVID-001B GitHub Qualification Source V1

This adapter projects already-authenticated GitHub-shaped qualification evidence into the canonical MYC-EVID technical-evidence waist from MYC-EVID-001A.

## Authority boundary

The module is pure and stdlib-only. It has no HTTP client, token access, process execution, filesystem mutation, workflow dispatch, runner control, or repository-write authority.

```text
provider acquisition/authentication
!= source normalization
!= exact-subject attribution
!= theorem correctness
!= publication authority
```

## Identity separation

V1 keeps separate:

```text
run_head_sha
tested_subject_sha
tested_tree_sha
designated_subject_sha
source_evidence_id
run_id
run_attempt
```

`run_head_sha` is discovery/workflow metadata and never proves the tested subject by itself. A detached replay may record workflow head B while testing designated subject A. Tree equivalence does not imply commit identity.

## Exact-subject attribution is symmetric

The adapter requires exact tested-subject proof before attributing any semantic disposition to the designated commit.

```text
unproven subject + workflow success -> UNSUPPORTED
unproven subject + workflow failure -> UNSUPPORTED
```

This prevents both false-green and false-red exact-subject evidence.

## Exact-subject assertion profiles

V1 recognizes a closed set:

- `runtime_exact_checkout`: tested commit and workflow head both equal the designated subject;
- `detached_exact_replay`: tested commit equals the designated subject while workflow head may differ;
- `tree_equivalence_only`: tested tree matches the designated tree but exact commit identity is not proven;
- `merge_ref_only`: an observed tested merge/ref commit is retained but cannot qualify the designated head;
- `unproven`: no exact-subject proof is claimed.

Unknown assertion modes fail closed.

## Positive qualification requires a receipt

V1 has no profile switch that permits PASS from workflow success alone.

A PASS requires all of:

```text
exact tested subject proven
workflow completed/success
receipt digest present
receipt subject == tested subject
receipt profile == registered profile
```

A receipt cannot substitute for missing tested-subject evidence. Exact-subject success without a bound receipt projects `NOT_ASSESSED`, not PASS.

## Run-level failure is not theorem failure

V1 deliberately does not emit semantic `FAIL` from GitHub's run-level `conclusion=failure` alone. A run may fail during checkout, setup, dependency acquisition, artifact upload, or another non-theorem step.

Therefore:

```text
exact subject + run-level failure -> NOT_ASSESSED
```

Semantic RED requires a stronger typed step-level theorem-failure or negative-receipt composition profile. That belongs to the EVIDENCE-CI step-role/liveness line rather than this run-level source adapter.

Likewise `cancelled`, `skipped`, `timed_out`, and `action_required` remain `NOT_ASSESSED` when the exact subject is known and `UNSUPPORTED` when it is not.

## Run attempts and source evidence

`run_id` and positive `run_attempt` are retained separately.

`source_evidence_id` is mandatory and opaque. The adapter does not invent a second source commitment algorithm. It preserves the identifier supplied by the authenticated acquisition layer.

Two source objects may project to an identical legacy technical record while remaining distinct historical evidence because their source IDs and/or run attempts differ.

## Projection entry point

Authority-bearing projection is exposed through `project_github_technical_evidence_v1(profile, source, ...)`, which invokes the source normalizer internally.

The helper that projects an already-normalized object is private to the module. A caller must not be able to hand-author `disposition="PASS"` in a normalized dictionary and bypass exact-subject or receipt validation.

## Historical regression fixtures

The source tree freezes two real pull-request/default-checkout ambiguity cases:

- MYC-EVID-001A run `34912377654`;
- MYC-CAP-002A run `34959974354`.

Both runs report a PR head SHA and completed/success, while the workflows use default `actions/checkout` without an explicit head ref. The fixtures therefore preserve the run head but refuse literal exact-head PASS because the tested commit is not independently established.

## Currentness

Historical semantic disposition and currentness remain separate.

An exact historical PASS remains PASS for its exact subject while `designated_current_subject_sha` may move to a successor. The canonical MYC-EVID waist decides whether that PASS is current enough to support a `Qualified` claim.

## Claim ceiling

Even after independent qualification, this adapter may establish only bounded source-normalization and exact-subject positive-attribution semantics equivalent to:

`QualifiedExactSubjectGitHubQualificationSourceProjectionV1`

It does not establish GitHub incorruptibility, runner trust, semantic theorem failure from run-level failure, product correctness, deployment truth, publication authority, or cross-run/cross-attempt evidence composition.
