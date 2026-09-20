# FORGE-006 — Portable exact-subject change proposals

`mycelix-forge-proposal` defines the immutable object that reviewers approve or reject.

A proposal binds:

- stable project identity derived from the supplied `AuthorityEpoch`;
- claimed proposer `PrincipalId`;
- exact authority-epoch commitment;
- exact project-policy commitment;
- exact typed repository-policy-state commitment;
- exact target ref;
- exact base commit;
- exact proposed commit;
- claimed resulting tree;
- immutable change-intent / requirements commitment;
- sorted unique typed `ChangeProposalId` dependencies.

Its `ChangeProposalId` is deterministic. There is no server id, hosting location, timestamp, comment thread, title, label, or random nonce in proposal identity.

## Typed construction context

The production constructor takes live `AuthorityEpoch` and `RepositoryPolicyState` objects. The project is derived from the authority epoch, and construction fails if the repository-policy state belongs to another project.

```text
AuthorityEpoch(project A)
      +
RepositoryPolicyState(project A)
      ↓
ChangeProposal(project A)
```

A cross-project policy cannot be embedded accidentally through a loose hash argument.

Deserializing an existing proposal does not re-prove the external authority/policy objects. A serialized proposal remains a portable claim subject; later verification layers must resolve its commitments against retained authority/policy evidence.

## Why mutable collaboration metadata is excluded

Discussion comments, reviewer notes, display titles, labels and UI state may change while the code under review remains identical. Binding them into proposal identity would needlessly invalidate approvals.

Review-significant changes do invalidate identity:

```text
commit changes
or tree changes
or target ref changes
or authority/project/repository policy context changes
or immutable intent changes
or dependency set changes
        ↓
new ChangeProposalId
        ↓
old reviews no longer apply
```

## Dependency semantics

Dependencies are `ChangeProposalId`, not untyped hashes. Public construction accepts any order, rejects duplicates, and normalizes to sorted canonical order. Deserialization is stricter: non-canonical wire order is rejected rather than silently rewritten.

## Claim boundary

A `ChangeProposal` is an exact **claim/review subject**. It does not establish that:

- `proposed_revision` is reachable from `base_revision`;
- `proposed_revision` really contains `resulting_tree`;
- the named proposer controls `proposer`;
- the proposer is authorized;
- the proposal satisfies repository policy;
- a reviewer is authenticated or authorized;
- the proposal may be merged.

Those facts belong to repository verification, principal authentication, review authorization and merge-policy layers respectively.

## Validation

Protocol v1 rejects:

- base == proposed no-op transitions;
- mixed SHA-1/SHA-256 Git object formats inside one proposal;
- repository-policy state from another project;
- duplicate dependencies;
- non-canonical dependency ordering on the wire;
- more than 256 dependencies.
