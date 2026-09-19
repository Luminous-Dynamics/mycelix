# FORGE-007B — Distinct approval quorum

`mycelix-forge-review-quorum` proves a narrow statement:

> one exact proposal has enough **distinct**, structurally authorized `Approve` reviews to satisfy the exact `ReviewSource` threshold of one exact authority epoch at one supplied observation time.

It intentionally does **not** say that the proposal may be merged.

## Inputs

The qualifier accepts only positive `StructurallyAuthorizedReview` values from FORGE-007. It then requires every supplied review to:

- belong to the same project;
- name the exact supplied `ChangeProposalId`;
- bind the proposal's exact authority epoch;
- bind the proposal's exact repository-policy state;
- carry decision `Approve`.

Reviewer principals are sorted canonically and passed to the existing authority-layer `evaluate_structural_quorum(..., ReviewSource, ...)` implementation.

## Fail-closed behavior

- duplicate reviewer principals are rejected rather than deduplicated;
- `RequestChanges` cannot count as approval;
- mixed proposals/authority/policy contexts fail;
- a reviewer no longer eligible at quorum observation time fails the positive theorem;
- below-threshold approval sets return an error rather than a positive unsatisfied object.

## Non-claims

A qualified approval quorum does not prove:

- absence of other `RequestChanges` reviews;
- trusted time;
- repository correctness;
- build qualification;
- merge policy satisfaction;
- merge authorization.

Those are separate policy/evidence layers.
