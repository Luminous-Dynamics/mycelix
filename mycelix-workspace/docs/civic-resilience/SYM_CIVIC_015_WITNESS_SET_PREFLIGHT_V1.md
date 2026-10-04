# SYM-CIVIC-015 — witness-set proof boundary preflight v1

Status: design/research preflight only; **not qualified**

Parent subject: \`38218c2cdebb520b32cf908166d3f4177a7976a5\`

## Boundary

015 isolates the witness-set property that sits above a single checkpoint-aware witness:

\`named witnesses + exact identities + bound independence groups + explicit quorum policy => witness-scoped non-equivocation evidence\`

The September 2026 Checkpointed Local Log Internet-Draft distinguishes internal checkpoint consistency from fork detection, requires a checkpoint-aware witness to compare against its own last accepted checkpoint, requires named independent witnesses, forbids claims beyond the last witnessed checkpoint, and says multiple independent witnesses strengthen equivocation resistance.

The September 2026 SCITT Attestation Reconciliation Protocol draft (ARP-04) adds two mechanically checkable quorum constraints: the counted entries must have pairwise distinct Operating-Party Identifiers and pairwise distinct Verification Method References; a relying party must reject a set in which two entries resolve to the same key. The draft separately acknowledges that declared party identifiers do not mechanically prove real-world independence.

RFC 9943 defines SCITT non-equivocation around a consistent VDS sequence visible to relying parties.

## Model

The synthetic policy is 2-of-3 with at least two distinct operating parties and two distinct declared independence groups.

Each witness binds a Verification Method Reference, a public-key identity digest, an Operating-Party Identifier, and a declared independence group. Distinct public keys do not establish independence by themselves. Distinct Operating-Party Identifiers do not prove real-world independence either; common control and other operational relationships remain deployment-level properties. Producer-operated witnesses are excluded from the qualifying set.

A non-quorum conflicting witness does not destroy a k-of-n result when quorum remains intact. A conflict that reduces matching independent witnesses below quorum is insufficient. Duplicate conflicting attestations under the same witness identity are provenance failure.

Claims may not extend beyond the last checkpoint accepted by the witnesses supporting the quorum.

## Corpus

Exactly 18 cases; no expected-verdict fields.

Derived census:

\`REJECT_WITNESS_SET_PROVENANCE = 7\`
\`WITNESS_SET_SUFFICIENT = 3\`
\`WITNESS_SET_INSUFFICIENT = 6\`
\`WITNESS_SET_UNRESOLVED = 0\`

The corpus separately exercises duplicate Verification Method Reference, same-key, and same-operating-party quorum failure so the qualifier cannot collapse mechanically distinct identity layers into a generic independence-group count.

Metamorphic probes exercise quorum population collapse, independence-group collapse, stale witnesses, producer-operated witnesses, fork-free failure, non-critical extension mutation, last-checkpoint regression, quorum-preserving conflict, quorum-breaking conflict, same-witness conflict, same-operating-party quorum failure, duplicate verification-method failure, and frontier extension beyond the last witnessed checkpoint.

## Ceiling

A GREEN result would establish only this synthetic witness-set boundary. It would not establish global non-equivocation outside the named witness set, correctness of independence declarations, witness honesty, universe completeness, legal/judicial finality, institutional legitimacy, or execution authority.

Design rule:

\`Do not count signatures; count independently bound witness domains against an explicit quorum policy.\`

References:
- https://datatracker.ietf.org/doc/draft-hillier-scitt-arp/ (draft-hillier-scitt-arp-04, September 2026)
- https://datatracker.ietf.org/doc/draft-mih-scitt-checkpointed-local-log/ (draft-mih-scitt-checkpointed-local-log-01, September 2026)
- https://www.rfc-editor.org/rfc/rfc9943
