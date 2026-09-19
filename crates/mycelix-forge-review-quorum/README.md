# FORGE-007B — Distinct review quorum

`mycelix-forge-review-quorum` evaluates whether one exact change proposal has enough **distinct, authenticated, structurally authorized approvals** to satisfy the exact `ReviewSource` threshold in its bound authority epoch.

The protocol deliberately keeps these claims separate:

```text
ReviewStatement
      ↓
authenticated reviewer
      ↓
StructurallyAuthorizedReview
      ↓
distinct ReviewSource threshold
      ↓
ReviewQuorum
```

`ReviewQuorum` is **not** merge authority. A later merge-policy theorem must combine it with repository-policy, protected-history, CI/qualification and any additional merge requirements.

## Exact quorum context

Every accepted review must name the same:

- `ChangeProposalId` re-derived from the supplied proposal;
- project identity;
- authority epoch;
- repository-policy state;
- `Approve` decision.

The evaluator rechecks each reviewer's `ReviewSource` eligibility in the supplied `AuthorityEpoch` at that review's recorded observation time.

## Distinct reviewers

Reviewer principals are sorted canonically. Supplying the same principal twice is an error rather than implicit deduplication, so callers cannot accidentally mistake duplicate evidence for additional quorum weight.

## Fail-closed inputs

The evaluator rejects:

- duplicate reviewer principals;
- `RequestChanges` entries in a purported approval set;
- mixed proposal ids;
- mixed authority epochs;
- mixed repository-policy contexts;
- project mismatch;
- reviews no longer structurally eligible under the supplied epoch;
- missing `ReviewSource` threshold;
- approval count below threshold.

## Time boundary

The review observation times are not authenticated by this crate. They are the same externally supplied times already carried by FORGE-007 `StructurallyAuthorizedReview`.

The quorum theorem is therefore conditional on those observation times; it does not establish trusted time or timestamp authority.
