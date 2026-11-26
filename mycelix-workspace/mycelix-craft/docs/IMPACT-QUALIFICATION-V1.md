# Craft Impact Qualification V1

- Work item: CRAFT-IMPACT-QUALIFICATION-001 (#6763)
- Status: scoped reference profile; integration and field qualification pending
- Version: 1.0-draft
- Date: 2026-11-26

## Ownership

This crate is deliberately narrower than a currentness or provenance platform.
It consumes exact change/event/artifact/dependency-edge/context references, owner-qualified leases from #181, semantic claim dispositions from Claim Support, owner-assigned common-cause groups and provenance from #6140, and independent reviewer/authority receipts.

It does not create event storage, a new lease algebra, a generic currentness field, evidence signatures, source authority, identity, semantic entailment or decision authorization.

The cross-domain invalidation benchmark and oracle belong to #6219. Evidence Lineage V1 (#6761/#6762) supplies a bounded structural graph evaluator. This profile decides whether one exact change can be declared not applicable to one exact Craft use and whether a declared support rule remains sufficiently supported.

## Scoped impact determination

A packet binds the exact change event and digest, changed artifact revision/digest, dependent artifact revision/digest, dependency edge ID/digest, deployment/use context, jurisdiction, population and use scope digests, status, typed justification, evidence references, assessor, independent reviews and an opaque owner-qualified lease reference.

Statuses:
- Affected: the change applies to this exact use; no exemption.
- NotAffected: the change is excluded only for the exact bound edge/event/context, with an allowed typed justification and evidence.
- UnderInvestigation: applicability remains unresolved; this is never silently treated as NotAffected.

A separate DomainAuthority review is required. High-consequence packets also require a distinct SafetyAuthority review. The assessor cannot approve their own packet. Reviewer approvals must bind the exact determination digest and carry current owner-qualified review leases.

A NotAffected packet never restores the upstream artifact, authorizes a decision, suppresses other edges, or survives a change to the bound event, artifact, edge, context, scope, reviewer authority, review digest or lease.

## Evidence sufficiency

Claim Support continues to own the semantic assessment of an exact claim. This crate evaluates only the sufficiency rule declared by the domain owner:
- AllOf: every named required evidence result must be DirectSupport.
- AnyOf: at least one complete, predeclared alternative support set must be DirectSupport.
- AtLeastIndependent: at least k named DirectSupport results must come from distinct, owner-qualified independence groups.

Stale, partial and not-assessable evidence do not count as direct support. Any candidate contradiction causes ConflictReviewRequired rather than being averaged away. A missing independence group fails closed for k-of-n evaluation.

Independence groups are assertions supplied by a qualified source/provenance assessor. Distinct IDs, documents, model calls or organizations do not by themselves prove independence. This crate checks group uniqueness; it cannot prove the groups correct.

## Interoperability and prior art

- CISA/NTIA VEX supplies scoped Affected, NotAffected, Fixed and UnderInvestigation patterns for product/vulnerability applicability. It is a design reference, not a general evidence-sufficiency standard.
- W3C Bitstring Status List v1.0 is the credential-status mechanism to consume for credential suspension/revocation; do not create a competing global status list.
- W3C PROV-DM is the provenance interchange foundation. This crate does not replace its entity/activity/agent/derivation vocabulary.
- Jama Connect and IBM DOORS Next already provide traceability and change-impact workflows. This profile is not a replacement for those workflows; it adds exact context-bound impact disposition and declared alternative evidence sufficiency to the Craft lifecycle.

## Limitations

No claim is made about correctness of owner-qualified lease, provenance or review issuers; actual statistical independence of source groups; legal authority of a reviewer; truth of source evidence; production Holochain integration; or real-world safety and service outcomes.

See IMPACT-QUALIFICATION-BENCHMARKS-V1.md for adversarial cases and targets.