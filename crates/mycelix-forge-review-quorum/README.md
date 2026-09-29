# Mycelix Forge Review Quorum

This crate establishes a distinct-reviewer approval quorum from reviews that have already crossed every earlier collaboration trust boundary:

```text
exact ReviewStatement
→ FORGE-005A evidence binding
→ structural ReviewSource eligibility
→ Xenia provider cryptographic provenance
→ project-policy trust in that exact Xenia verifier identity
→ ProjectPolicyTrustedReviewV1
→ distinct approval quorum
```

Raw reviews and provider-verified-but-project-untrusted reviews are not accepted by the quorum API.

## Positive theorem

`ProjectPolicyReviewQuorumV1` requires:

- one exact `ChangeProposalId`;
- one exact authority epoch;
- the proposal's exact project-policy and repository-policy commitments;
- only `ReviewDecision::Approve` statements;
- distinct reviewer principals;
- one common caller-supplied quorum observation time;
- every counted review observed no later than that common time;
- every reviewer structurally eligible for `ReviewSource` at that common time;
- approval count at or above the exact `ReviewSource` threshold.

Accepted approvals are canonically sorted by `PrincipalId`, so caller input order does not alter quorum identity.

## Time semantics

The common quorum observation time improves composition, but it is not trusted time.

This crate proves only:

```text
under the supplied immutable authority epoch
and at supplied time T
these distinct already-trusted approval statements satisfy threshold
```

It does not prove that T came from a trusted clock.

## Conflict/current-state boundary

A positive quorum also does not prove absence of:

- a later `RequestChanges` statement from one of the same principals;
- a withdrawal/supersession event;
- another conflicting review set;
- a newer authority or project-policy state.

Merge authorization must therefore consume review-currentness/supersession evidence separately rather than treating historical quorum as automatically current merge authority.

## Non-claims

A positive review quorum does not establish:

- trusted time;
- repository object correctness;
- CI/build qualification;
- gittuf/protected-history compliance;
- absence of later/opposing reviews;
- merge authorization;
- release authorization.
