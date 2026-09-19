# FORGE-006 — Portable exact-subject change proposals

`mycelix-forge-proposal` defines the immutable object that reviewers approve or reject.

A proposal binds:

- stable project identity;
- claimed proposer `PrincipalId`;
- exact authority epoch;
- exact repository-policy state;
- exact target ref;
- exact base commit;
- exact proposed commit;
- claimed resulting tree;
- immutable change-intent / requirements commitment;
- sorted unique proposal dependencies.

Its `ChangeProposalId` is deterministic. There is no server id, hosting location, timestamp, comment thread, title, label, or random nonce in proposal identity.

## Why mutable collaboration metadata is excluded

Discussion comments, reviewer notes, display titles, labels and UI state may change while the code under review remains identical. Binding them into proposal identity would needlessly invalidate approvals.

Review-significant changes do invalidate identity:

```text
commit changes
or tree changes
or target ref changes
or authority/policy context changes
or immutable intent changes
or dependency set changes
        ↓
new ChangeProposalId
        ↓
old reviews no longer apply
```

## Claim boundary

A `ChangeProposal` is an exact **claim/review subject**. It does not establish that:

- `proposed_revision` is reachable from `base_revision`;
- `proposed_revision` really contains `resulting_tree`;
- the proposer controls `proposer`;
- the proposal satisfies repository policy;
- the proposer or reviewers are authorized;
- the proposal may be merged.

Those facts belong to repository verification, principal authentication, review authorization and merge-policy layers respectively.

## Validation

Protocol v1 rejects:

- base == proposed no-op transitions;
- mixed SHA-1/SHA-256 Git object formats inside one proposal;
- duplicate dependencies instead of silently deduplicating them;
- more than 256 dependencies.

Dependencies are normalized into deterministic canonical ordering after duplicate rejection.
