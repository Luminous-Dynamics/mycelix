# Proposal Authority Runtime Adapter v0.1

This zome pair is the Holochain persistence/runtime adapter for the wire-neutral
`mycelix-governance-authority::ProposalAuthorityContext` contract.

It deliberately does **not** add fields to the existing `Proposal` entry. That
entry is mirrored by other zomes and changing its wire shape would create a
migration hazard.

## Authority chain

For binding governance, proposal activation becomes:

`Draft Proposal`
→ exact proposal/action bytes
→ immutable `ProposalAuthorityBinding`
→ institution + jurisdiction + exact rulebook
→ governing body
→ signing-policy id + digest + digest-profile id
→ `Active Proposal`

Later threshold-signing and execution adapters must consume the verified
context and remain fail-closed when it is absent, stale, expired, or ambiguous.

## Exact action identity

`actions_digest` intentionally uses the same profile currently used by the
fail-closed execution boundary:

`mycelix-governance-execution-authority-v1-blake3-exact-json`

The digest binds:

1. `mycelix-governance-execution-authority-v1\0`
2. proposal-id byte length as little-endian `u64`
3. exact proposal-id bytes
4. action JSON byte length as little-endian `u64`
5. exact action JSON bytes

Whitespace changes therefore change the digest. This is intentional until the
execution boundary and proposal adapter migrate together to a registered typed
canonical action profile.

## No retroactive authority

A new binding may be created only while the proposal is Draft. The adapter then
activates the proposal through the existing proposals coordinator.

An already-Active proposal is accepted only for an idempotent retry when an
equivalent context was already committed. A legacy Active proposal cannot bolt
on institutional authority after voting has begun.

## Append-only semantics

Authority bindings and their links are immutable. If Draft proposal content
changes, its old binding remains historical but stops matching because verified
lookup recomputes the current exact action digest.

Multiple equivalent bindings are harmless and collapse to one verified result.
Multiple conflicting contexts matching the same current proposal bytes are an
ambiguity and fail closed.

## Important non-claim

This adapter binds a proposal to a signing-policy **identity and digest**. It does
not prove that the signing policy was legitimately adopted. That proof remains
the responsibility of the signing-policy registry / institutional authority
adapter described in the parent governance-authority contract.

Likewise, this zome does not by itself make legacy voting binding. The
score-independent rights-floor runtime migration should consume
`get_verified_proposal_authority_context` in a later stacked PR.
