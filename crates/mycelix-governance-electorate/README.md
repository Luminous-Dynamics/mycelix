# Mycelix Governance Electorate v0.1

`mycelix-governance-electorate` removes a subtle source of governance authority
from the tally caller: the electorate denominator.

A binding vote should not accept an arbitrary `eligible_voters = 100` parameter.
The number used for quorum must come from an immutable, institution-authorized
snapshot of the electorate under the exact proposal rulebook.

## Separation of concerns

Binding governance now separates three facts:

1. **Civic eligibility grant** — this principal has the `Vote` right.
2. **Electorate snapshot** — these are the population semantics and committed
   member set for this proposal.
3. **Binding ballot** — this eligible principal cast one choice.

None of these may silently stand in for another.

## Electorate snapshot

An `ElectorateSnapshot` binds:

- proposal;
- institution;
- optional jurisdiction;
- exact rulebook;
- eligible-person count;
- exact eligibility-criteria digest + profile;
- private member-set commitment + commitment profile;
- exact membership verifier-policy digest + policy profile/reference;
- membership proof format/profile;
- capture/open/close timestamps;
- explicit revocation semantics;
- institutional authority grant that froze the snapshot;
- host-verified snapshot proof/receipt.

The actual member list need not be published. A Merkle tree, accumulator, ZK set
commitment, credential registry snapshot, or another registered profile can back
the commitment.

## Verifier policy is not commitment format

A member-set root answers "which set was committed?" It does not answer "whose
membership proof should I trust?"

v0.1 therefore keeps these independent:

- `membership_commitment_profile` — how the population set is committed;
- `membership_verification_profile` — how one principal proves membership;
- `membership_verifier_policy_*` — which verifier roots/trust domains/policy
  qualify such a proof.

This prevents a self-created verifier from satisfying membership merely because
it can produce a structurally valid proof.

## Fixed denominator, revocation-aware ballots

v0.1 has one deliberately precise rule:

`RejectRevokedBallotKeepDenominator`

The snapshot denominator does not change silently while voting is open. A
revoked right may still prevent a ballot from qualifying, but changing the
population denominator requires a new, distinct ballot/snapshot. This preserves
both auditability and revocation safety.

## Tally API

`tally_snapshot_ballots()` has **no `eligible_voters` argument**.

Every ballot is supplied with:

- the binding ballot itself;
- a `RightPermit` for the exact institution/jurisdiction/rulebook;
- membership evidence for the exact electorate snapshot and verifier policy.

The function re-checks all structural bindings, then delegates raw-count tallying
to `mycelix-governance-rights` using the snapshot's authenticated denominator.

## Host responsibilities

This crate is intentionally wire-neutral. Production adapters must independently
verify:

- the snapshot authority grant and `snapshot_proof_ref`;
- the membership set commitment and its count;
- credential/grant revocation state according to the snapshot semantics;
- each membership proof under the exact verifier policy bound by the snapshot;
- verifier identity/trust-domain provenance rather than caller-provided labels;
- immutable persistence and proposal-context matching.

A non-empty proof reference or verifier name is never sufficient on its own.

## Runtime next step

The Holochain binding-vote adapter should require both:

1. `proposal_authority::get_verified_proposal_authority_context`; and
2. one immutable electorate snapshot whose institution/jurisdiction/rulebook and
   voting window exactly match that proposal context.

Only then should it admit a score-independent `BindingVoteV2`.
