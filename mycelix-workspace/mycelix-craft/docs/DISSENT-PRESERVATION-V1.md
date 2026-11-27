# Dissent-Preserving Need Records V1

**Status:** draft reference contract.  
**Implementation:** `craft-counterfactual-repair-dissent-v1`.  
**Related benchmark:** Evidence Assurance Metamorphic V1, MR-018.

## Principle

A system may help a group understand, compare and summarize material positions. It must not manufacture agreement by deleting minority positions, merging attributed concerns into one synthetic consensus statement, interpreting silence as assent, or treating an LLM summary as an authority decision.

Material positions remain separately identifiable, attributed, versioned and reconstructable throughout proposal revisions and decision review.

## Position record

Each position binds:
- stable position ID and revision;
- exact proposal artifact revision;
- stakeholder/group attribution artifact;
- statement digest;
- provenance artifact references;
- exact outcome, requirement or material-element scope;
- position state: Active, Disputed, Resolved or Withdrawn;
- exact resolution record when its state is Resolved or Withdrawn.

The host is responsible for verifying that the attributed actor or representative was entitled to speak for the named group, that participation and visibility rules were respected, and that provenance is authentic.

## Resolution record

An explicit resolution binds the exact proposal revision, the position IDs it addresses, a decision artifact, an authority artifact, and the resolution artifact/digest. It may resolve only positions that:
- exist;
- are bound to the same proposal revision;
- are not already resolved or withdrawn.

A generated summary, poll count, inferred majority, absent objection or model output is not a resolution. A legitimate compromise changes only the positions explicitly named in the authenticated decision. The original positions and their history remain available.

## Summary preservation

A summary carries a reference for every material position:
- Active and Disputed positions must be marked Preserved and retain the exact position digest.
- Resolved and Withdrawn positions must be marked ExplicitlyResolved and point to the exact resolution artifact, resolution ID and decision artifact attached to that position.
- Omitted, duplicated, unknown, stale-revision or incorrectly resolved position references fail validation.

A summary may shorten wording, cluster related subjects, or present alternatives together, but it must not collapse their attribution or lose their separate identity. Compression is not consensus.

## Limits and trust boundary

The reference core checks exact revisions, digests, identity coverage and resolution references. It does not authenticate a community vote, representative mandate, signature, quorum, legal competence or consent procedure. Those remain host and community-governance responsibilities, and their evidence must be retained as the authority artifact referenced by the resolution.

A position withdrawal is not agreement with another position. The contract distinguishes ExplicitlyWithdrawn from ExplicitlyResolved, and the position carries a PositionResolutionKind of Withdraw rather than Resolve. A withdrawal remains a separately attributed state and must not be represented as consensus or compromise.

## Acceptance tests

The current tests cover missing material positions, summary identity collapse, changed position digests, attempts to mark disputed positions as resolved without a decision, missing exact resolution references, cross-proposal resolution, unknown and duplicate position IDs, and repeated resolution attempts.
