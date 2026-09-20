# FORGE-008A — Proposal source verification

`mycelix-forge-proposal-source` turns the immutable Git claims in FORGE-006 into evidence-backed facts without making gittuf or a Git implementation the Forge protocol root of trust.

```text
ChangeProposal
  base / proposed / resulting tree claims
        +
exact RepositoryVerificationRequest
        +
QualifiedRepositoryVerification
        +
independent Git-source observation/verifier
        ↓
QualifiedProposalSource
```

## Exact protected-transition reconstruction

The qualifier reconstructs the expected `RepositoryVerificationRequest` from:

- repository adoption;
- proposal base revision;
- proposal proposed revision;
- proposal authority epoch;
- exact project policy;
- exact repository-policy state.

It then requires the reconstructed canonical bytes to equal the actual request that produced the supplied positive repository verification. This binds the proposal to the request's otherwise internal adoption/authority/policy context without re-parsing or weakening the repository contract.

## Git-source evidence

The independent source verifier is responsible for establishing Git semantics that repository-policy verification does not imply by itself:

- the proposed object is the intended commit object;
- the accepted ancestry/reachability relation from base to proposed;
- the proposed commit references the exact claimed resulting tree;
- those observations came from the exact named source/object-store state.

The raw `ProposalSourceObservation` binds base/proposed/tree plus ancestry and commit-tree evidence commitments. A `QualifiedProposalSource` exists only after a concrete `ProposalSourceVerifier` accepts those facts.

## Repository evidence remains explicit

The positive result commits to the exact repository verification consumed by this layer: request digest, adapter identity, observed tip, repository-policy state, capabilities, history/evidence commitments and monotonic-policy lineage evidence where present.

## Non-claims

`QualifiedProposalSource` does not establish proposer authentication, review quorum, absence of opposing reviews, build qualification or merge authorization. Those remain separate Forge evidence/policy layers.

A future concrete Git adapter can implement `ProposalSourceVerifier` against a portable bundle/object database using pinned Git plumbing without changing this protocol contract.
