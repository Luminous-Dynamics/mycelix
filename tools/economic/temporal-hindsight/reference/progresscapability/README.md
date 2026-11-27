# Capability-derived temporal progress reference V0.1

This package structurally replays an append-only capability lifecycle and derives a progress antichain from the current active capability heads. It is intentionally separate from the basic progress algebra in the sibling progress-antichain package.

## Lifecycle

A capability is minted with generation 1 and an exact owner-local lower-bound vector. Advancement increments the generation, links to the prior capability-head digest and may only move coordinates forward. Release or revoke increments the generation, links to the prior head and consumes the active capability exactly once without rewriting its cut. Capability identifiers cannot be reused after release or revocation.

Every event has a contiguous log sequence, unique event identifier, previous-event digest and a domain-separated SHA-256 event commitment. Replay checks the complete sequence against an independently supplied event count and tail digest, then reconstructs active heads.

## Unknown outcomes

An event can carry a prepared transition with outcome OutcomeUnknown. That transition does not mutate the active set. It establishes a global reconciliation barrier: no further mint, advance, release, revoke or second unknown event is accepted until the exact pending event is reconciled as ReconciledApplied or ReconciledNotApplied.

An applied reconciliation executes the prepared transition after rechecking its predecessor and generation. A not-applied reconciliation leaves active state unchanged. This models the operational rule: unknown outcome means no new mutation until reconciliation.

## Derived progress

Active capability owner cuts are normalized to their minimal antichain. Dominated cuts are removed and incomparable cuts remain. A pending unknown outcome blocks derivation. An empty derived antichain means only that no declared capability remains after structurally replayed releases/revocations.

## Claim boundary

The package only establishes structural event-chain consistency relative to a supplied tail/count anchor. It does not authenticate that anchor, the issuer, key epoch, authority grant, checkpoint or source roster. It does not validate signatures, RFC 9162 signed tree heads, witnesses, DID key lifecycle, source-universe completeness or production authority.

Every replay and progress result keeps ledger, issuer-authority, checkpoint, progress and source-universe authentication at NotEvaluated. A signature over an event or Merkle path against a supplied root is not sufficient unless the root and issuer authority are independently qualified. See the existing temporal frontier profile and canonical-encoding contract in tools/economic/temporal-frontier-model/.

## Run

    go test ./...
    go test -race ./...
    go vet ./...
    go test -run '^$' -bench . -benchmem

The in-memory benchmark measures structural replay only. It excludes signature verification, checkpoint consistency, network calls, Holochain, distributed consensus, progress-query evaluation and source-roster proof cost.
