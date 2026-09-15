# REGEN-Q001 — Machine-Readable Qualification Receipt v1

Status: cross-cutting assurance contract; no scientific, governance, regulatory, or physical authority.

## Purpose

REGEN qualification evidence is currently visible in workflow logs and PR prose, but downstream gates should not have to infer the executed subject, dependency state, fixtures, assertions, or proposition from human text.

REGEN-Q001 defines a bounded machine-readable receipt for one qualification campaign.

It extends the semantics frozen by:

- REGEN-007 / #970 — evidence subject classes and exact-subject discipline;
- REGEN-008 / #972 — dependency-resolution states.

It deliberately uses a `REGEN-Q` namespace so the domain roadmap remains stable (`REGEN-011` is biomass provenance).

## Core theorem

```text
machine-readable receipt
!= authenticated receipt
!= implementation correctness beyond its assertions
!= scientific validity
!= governance authority
!= physical-action authority
```

A receipt says what subject was executed, under what recorded environment/dependency identity, which assertions ran, what result they produced, and what narrow proposition that campaign supports.

## Subject classes

The v1 subject class is one of:

- `ProductHead` — exact immutable product commit;
- `IntegrationMerge` — deliberate/provider-generated composition;
- `ReleaseArtifact` — immutable built artifact;
- `ExternalFixture` — exact external corpus/profile/dataset/protocol.

A result for one class must never be relabeled as another.

For a Git `ProductHead`, a `pass` receipt requires:

```text
declared_sha == observed_sha
```

The validator rejects a PASS if those values differ.

## Result vocabulary

- `pass` — campaign executed and every receipt assertion is `pass`;
- `fail` — campaign executed and at least one assertion is `fail`;
- `indeterminate` — campaign did not establish pass/fail, including queued/no-step/unavailable execution.

Therefore:

```text
queued
!= pass

workflow-created
!= executed

receipt-present
!= pass
```

## Dependency state

The receipt carries exactly one REGEN-008 state:

- `ProductFrozen`;
- `ExecutionResolved`;
- `Unresolved`.

Dependency identities are algorithm-qualified opaque strings. A dependency state records reproducibility identity only; it does not imply dependency security.

For `ProductFrozen` and `ExecutionResolved`, at least one dependency identity is required. `Unresolved` may carry none.

## Environment identity

The receipt records a bounded environment description and explicit toolchain strings. These strings are evidence labels, not a universal environment model. A stronger Nix/reproducibility capsule may later be referenced as a dependency/environment identity.

## Fixtures

Every consequential frozen external fixture used by a campaign should appear with:

- a stable receipt-local name;
- SHA-256 identity;
- optional Git blob identity when the fixture is checked into Git.

Fixture identity does not imply fixture truth, representativeness, licensing authority, or scientific adequacy.

## Assertions

Each assertion has a unique namespaced ID and one of:

- `pass`;
- `fail`;
- `indeterminate`.

The validator rejects duplicate assertion IDs.

The receipt-level result is constrained by assertion results:

- receipt `pass` -> executed and every assertion `pass`;
- receipt `fail` -> executed and at least one assertion `fail`;
- receipt `indeterminate` -> no PASS claim is permitted.

## Proposition and non-claims

Every receipt carries one non-empty `proposition`: the narrow statement that the campaign supports if it passes.

Every receipt also carries at least one explicit `non_claims` entry. This prevents generic `PASS` from becoming an ambient scientific or authority token.

Consumers MUST evaluate the proposition, subject class, dependency state and non-claims together.

## Authentication boundary

REGEN-Q001 v1 does not define signatures, trusted issuers, transparency logs, witness quorum, or remote-attestation authority.

A checked-in receipt can inherit ordinary Git repository provenance, but:

```text
valid receipt structure
!= authentic issuer
```

Future work may bind receipts to Xenia signing/attestation without changing this semantic distinction.

## Canonical files

- `schemas/regenerative/qualification-receipt-v1.schema.json` — language-neutral structural schema;
- `scripts/verify-regen-qualification-receipt.py` — independent standard-library semantic validator;
- `evidence/regenerative/qualification/regen-002-01ee859e.json` — first concrete receipt, representing the already-executed REGEN-002 ProductHead PASS;
- `.github/workflows/regen-qualification-receipt-ci.yml` — exact-head qualification for the receipt contract itself.

## First receipt

The first checked-in receipt binds the exact already-executed REGEN-002 campaign:

- ProductHead `01ee859e5d8679be14501245c8becc4ba3e10985`;
- Git tree `51386e356b270b4b0c5a4670b36184322243a380`;
- parent REGEN-000 `6e9903196b0de29ee9bca844b5c46d2ac7e3fb92`;
- workflow run `34961479829`;
- job `104355832650`;
- Rust/Cargo 1.96.0;
- `ExecutionResolved` Cargo.lock SHA-256 `fc3552c9b3c36cf4a0784ff8dccc053a88cb6980b213b47b178d76d2839b34c5`;
- exact-subject, format, default tests, serde tests, strict Clippy, lock recording and clean-checkout assertions all PASS.

The receipt does not upgrade that software result into agronomic or scientific evidence.

## Validator requirements

The independent validator uses Python standard library only and must reject at least:

- unknown top-level fields;
- malformed SHA-1/SHA-256 identities;
- unknown subject/dependency/result values;
- PASS with `executed=false`;
- ProductHead PASS with declared/observed SHA mismatch;
- PASS containing non-PASS assertions;
- FAIL without a failed assertion;
- duplicate assertion IDs;
- duplicate fixture names;
- resolved/frozen dependency state without any dependency identity;
- missing proposition;
- missing non-claim boundary.

Its self-test must contain both accepted and rejected mutations.

## Promotion rule

A receipt is evidence about its own declared proposition only.

```text
receipt A PASS
+ receipt B PASS
!= composite proposition PASS
```

unless a later campaign explicitly executes and qualifies that composition.

This prevents receipt aggregation from becoming a new authority shortcut.
