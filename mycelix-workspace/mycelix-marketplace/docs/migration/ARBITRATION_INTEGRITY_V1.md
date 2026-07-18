# Arbitration Integrity v1

## Status

This patch establishes a guarded, conflict-aware arbitration protocol and typed Leptos boundary. It does **not** declare arbitration production-ready, and the Leptos dispute action remains disabled until a live multi-agent conductor scenario passes.

## Security boundary

A dispute binds both the stable transaction Create action and the exact transaction revision that was already in `Disputed` state. The integrity zome verifies the transaction parties, the filing author, immutable case terms, legal case transitions, unique non-party arbitrators, vote authorship, and deterministic result derivation.

Votes bind the stable dispute root and the exact `UnderReview` or `Voting` revision that authorized them. Results bind the stable dispute root, the exact `Voting` revision, and every vote action used in the calculation. The result validator reloads those votes and recomputes the buyer-vote ratio, winner, loser, and recommended compensation.

## Conflict behavior

Dispute updates are reduced as an application-level tree. A single visible leaf is current. Multiple visible leaves are evidence of a conflict; reads expose every head and mutations fail closed rather than selecting a winner.

## Recoverability

`transactions.open_dispute` first moves the canonical transaction revision to `Disputed`, then calls `arbitration.file_dispute` idempotently. A failure after the transaction update is recoverable: retrying reuses the stable transaction root and existing Filed case.

Finalization is also recoverable. If a unique result entry and its dispute link exist but the dispute update did not complete—or the response was lost after the update—a retry binds or returns that result. Multiple case roots, duplicate arbitrator votes in the normal query path, or multiple linked result entries fail closed. An orphaned result whose link creation failed is not discoverable through this recovery path and remains a deliberate evidence gap.

## Reputation and assignment boundary

The guarded protocol intentionally uses one equal vote per assigned arbitrator. The historical `arbitrator_matl_score` wire field is fixed at `1.0` and validated as such. Reputation weighting will remain disabled until the integrity zome can validate a cryptographically bound score snapshot rather than trusting a coordinator response or browser-supplied value.

Arbitrator registration is currently permissionless. The coordinator deterministically chooses up to three locally visible registered agents after excluding the buyer, seller, and filer. This is a transparent pre-alpha policy, not a claim of globally fair or Sybil-resistant selection.

Filing a dispute creates no reputation side effect. Finalization projects exactly one winner and one loser event through the separately validated, idempotent reputation-event processor; it never mutates a legacy MATL score directly.

## Required live evidence before enabling the UI

The conductor scenario must demonstrate at minimum:

1. Buyer and seller create a transaction and obtain one canonical transaction root.
2. A non-party agent registers as an arbitrator.
3. A party opens a case and reloads it through the stable dispute root.
4. The assigned arbitrator sees one opportunity and casts one equal-weight vote.
5. A duplicate vote is rejected.
6. Finalization binds exactly the recorded vote hash and produces the expected winner and compensation.
7. Reload from the stable dispute root returns the resolved revision and unique result.
8. A deliberately concurrent dispute update is surfaced as multiple heads and blocks voting/finalization.
9. Filing creates no reputation event, while finalization creates exactly one winner and one loser event and rejects direct score mutation.

## Deliberate non-claims

This patch does not prove Byzantine tolerance, fair arbitrator selection, Sybil resistance, enforceable settlement, legal finality, or globally convergent conflict resolution. It creates a narrower auditable protocol on which those properties can be tested honestly.
