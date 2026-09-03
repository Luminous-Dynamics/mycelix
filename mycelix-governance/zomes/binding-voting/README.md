# Binding Voting Runtime v0.1

This additive Holochain runtime is the binding civic-vote path for the score-independent governance contracts introduced by the institutional/rights/electorate work.

It does **not** replace or reinterpret legacy `Vote`, Phi-weighted, quadratic, score/ZK, collective-mirror, or ethics entries. Those remain available for analysis/experimentation, but this runtime never reads them while producing a binding tally.

## Authority chain

```text
verified ProposalAuthorityBinding
        |
        v
verified ElectionConfiguration
  - institution / jurisdiction / exact rulebook inherited from proposal context
  - frozen electorate snapshot + member-set commitment
  - exact membership-verifier policy
  - raw-count tally policy
  - DHT ballot-set finality policy
        |
        v
BindingBallotRecord
  - committing DID == voter DID
  - explicit Vote RightPermit
  - exact electorate membership evidence
  - no weight field
  - qualification valid through ballot close
        |
        v
latest valid source-chain ballot per voter
        |
        v
BallotSetCheckpoint
  - exact selected + superseded action hashes
  - finality policy / observer threshold / receipt
        |
        v
SnapshotBoundTally
  - one selected voter == one ballot
  - quorum denominator from frozen snapshot
  - approval/quorum from exact tally policy
```

## Why ballots stay append-only

A voter may cast again before close. Each cast is a new immutable source-chain action. At finalization the runtime groups valid ballots by voter and selects the highest Holochain `action_seq()` for that voter, with action hash as a deterministic tie-breaker.

This means:

- no globally mutable `current_vote` object;
- no DHT arrival-order dependence;
- no duplicate-vote denial-of-service;
- old choices remain auditable;
- re-voting is deterministic because each voter's source chain is ordered.

## Frozen electorate semantics

The electorate snapshot is created no later than ballot opening and commits the eligible population, exact membership set, eligibility predicate, verifier policy, and voting window.

At cast time the rights verifier must return a `RightPermit` and `ElectorateMembershipEvidence` bound to that frozen snapshot. Qualification must remain valid through ballot close.

After a ballot is accepted, tallying verifies the stored qualification receipt **as it existed at cast time**. It must not query mutable Phi/reputation/stake/model scores or silently invalidate a vote because an unrelated live score changed later.

If later evidence shows fraud, forged credentials, verifier compromise, or other invalidity, the remedy belongs in the explicit challenge/appeal/remedy system. It does not silently rewrite the historical tally.

## DHT finality

Closing time is not ballot-set finality. Holochain is eventually consistent, so one node can reach the close time before it has received every pre-close ballot.

`finalize_ballot_set` therefore cannot create a checkpoint from local observation alone. It submits the exact observed valid ballot actions to the rulebook-bound finality verifier. Only a response meeting the configured observer/finality policy can produce a `BallotSetCheckpoint`.

The checkpoint freezes the exact action set used for the binding tally. Late discovery of a ballot after a valid checkpoint is a challenge/remedy condition, not an implicit local retally.

## Fail-closed verifier boundary

The coordinator calls a local zome named:

`governance_rights_verifier`

Required functions:

- `verify_election_configuration`
- `verify_ballot_qualification`
- `verify_stored_ballot_receipt`
- `attest_ballot_set_finality`
- `verify_ballot_set_checkpoint`

This branch deliberately does **not** provide a permissive or mock implementation in the production DNA. Missing zome/function, call failure, decoding failure, denial, or an inexact echoed binding causes the binding operation to fail.

A follow-on verifier implementation must resolve the exact rulebook/trust policy and verify the relevant credentials, proofs, receipts, revocation-at-snapshot semantics, and finality observers. It must not use Phi, reputation, stake, wealth, or model score as a substitute for the explicit civic right.

## Stored entries are not authority by existence

Integrity validation enforces structure, append-only behavior, and author binding, but an agent can still attempt to publish structurally valid candidate entries directly.

Consumers therefore use only verified coordinator endpoints:

- `get_verified_election_configuration`
- `get_verified_ballot_set_checkpoint`
- `get_verified_binding_tally`

These re-establish the external verifier and proposal-authority bindings. A raw DHT entry is evidence to inspect, not proof of institutional authority.

## Remaining integration before merge-ready

1. Implement and independently test the real `governance_rights_verifier` boundary.
2. Add SweetConductor multi-agent tests for re-voting, DHT reorder, missing/late ballots, conflicting snapshots/checkpoints, forged receipts, and verifier outage.
3. Wire governance proposal finalization/execution to require `get_verified_binding_tally` for binding election classes.
4. Convert legacy Phi/quadratic/ethics outputs to explicitly advisory modes in user-facing APIs.
5. Rename/clarify the portable electorate revocation enum so its wording exactly matches frozen-snapshot runtime semantics before protocol freeze.
