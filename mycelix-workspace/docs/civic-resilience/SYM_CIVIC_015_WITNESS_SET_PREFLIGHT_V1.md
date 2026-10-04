# SYM-CIVIC-015 — witness-set proof boundary preflight v1

Status: design/research preflight only; **not qualified**

Parent subject: \`38218c2cdebb520b32cf908166d3f4177a7976a5\`

## Boundary

015 isolates the witness-set property that sits above a single checkpoint-aware witness:

\`named witnesses + exact identities + bound independence groups + explicit quorum policy => witness-scoped non-equivocation evidence\`

The September 2026 Checkpointed Local Log Internet-Draft distinguishes internal checkpoint consistency from fork detection, requires a checkpoint-aware witness to compare against its own last accepted checkpoint, requires named independent witnesses, forbids claims beyond the last witnessed checkpoint, and says multiple independent witnesses strengthen equivocation resistance.

RFC 9943 defines SCITT non-equivocation around a consistent VDS sequence visible to relying parties.

## Model

The synthetic policy is 2-of-3 with at least two distinct independence groups.

Distinct public keys do not establish independence by themselves. Producer-operated witnesses are excluded from the qualifying set. The qualifier binds the declaration; actual operational independence remains an external deployment property.

A non-quorum conflicting witness does not destroy a k-of-n result when quorum remains intact. A conflict that reduces matching independent witnesses below quorum is insufficient. Duplicate conflicting attestations under the same witness identity are provenance failure.

Claims may not extend beyond the last checkpoint accepted by the witnesses supporting the quorum.

## Corpus

Exactly 16 cases; no expected-verdict fields.

Derived census:

\`REJECT_WITNESS_SET_PROVENANCE = 7\`
\`WITNESS_SET_SUFFICIENT = 3\`
\`WITNESS_SET_INSUFFICIENT = 6\`
\`WITNESS_SET_UNRESOLVED = 0\`

Metamorphic probes exercise quorum population collapse, independence-group collapse, stale witnesses, producer-operated witnesses, fork-free failure, non-critical extension mutation, last-checkpoint regression, quorum-preserving conflict, quorum-breaking conflict, same-witness conflict, and frontier extension beyond the last witnessed checkpoint.

## Ceiling

A GREEN result would establish only this synthetic witness-set boundary. It would not establish global non-equivocation outside the named witness set, correctness of independence declarations, witness honesty, universe completeness, legal/judicial finality, institutional legitimacy, or execution authority.

Design rule:

\`Do not count signatures; count independently bound witness domains against an explicit quorum policy.\`

References:
- https://datatracker.ietf.org/doc/draft-mih-scitt-checkpointed-local-log/
- https://www.rfc-editor.org/rfc/rfc9943
