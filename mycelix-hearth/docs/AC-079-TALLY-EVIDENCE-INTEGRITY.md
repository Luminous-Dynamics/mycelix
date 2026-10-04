# AC-079: Typed Tally Evidence and Authenticated Decision Link Topology

AC-078 records the exact vote action hashes used by a finalized tally. AC-079 closes two remaining topology gaps.

## Integrity invariants

- DecisionToVotes and DecisionToVoteHistory targets must be valid registered Vote entries belonging to the base Decision, and the link author must equal the Vote voter.
- DecisionToOutcome targets must be valid registered DecisionOutcome entries belonging to the base Decision, and the link author must equal the outcome resolver.
- HearthToDecisions targets must be valid registered Decision entries belonging to the base Hearth, and the link author must equal the decision creator.

## Tally boundary

The coordinator and integrity layer both verify the authenticated application entry definition before a linked record is treated as a Vote. A record from another zome that merely deserializes to compatible bytes is not accepted as tally evidence.

Linked Votes whose decision reference does not match the tallying Decision are rejected rather than silently ignored.

## Participation boundary

participation_rate_bp is derived from the exact explicit vote evidence set stored on the DecisionOutcome. Raw mutable DHT link cardinality is no longer the source of that metric.

This prevents duplicate or malformed collection links from inflating the reported participation number.

## Qualification boundary

This establishes typed-link, base/target relationship, and link-author provenance. It still does not prove DHT-wide completeness, exactly-once distributed finalization, real-world identity, or substantive legitimacy.

Holochain validation remains deterministic and uses explicit record dependencies for this proof boundary.