# LEX-NET-028 — Typed Transition Evidence and Authority Separation v1

Status: executable research contract. This profile is language-neutral, deterministic, zero-network, and has no legal or external-effect authority.

## Governing theorem

`valid evidence != sufficient evidence != local authority != execution != external finality`

`portable evidence != portable authority`

LEX-NET does not define a scalar trust ladder. Evidence products prove bounded propositions and are intentionally incomparable unless an explicit local requirement names the proposition it needs.

## Qualified parent

This tranche is a direct child of qualified LEX-NET-032 exact head:

`117aa1e9c8701589d56dc55dc581f1191a97a071`

The parent established the frozen resource-bounded canonical-interpretation contract. This tranche does not weaken or reinterpret it.

## Product universes

v1 defines two disjoint product universes.

### EvidenceProduct

Frozen v1 kinds:

- `InterpretationEvidence`
- `TranslationEvidence`
- `QuarantineEvidence`
- `RecognitionEvidence`
- `CurrentnessEvidence`
- `ExternalProofEvidence`
- `IntentConfirmationEvidence`
- `ExecutionEvidence`
- `FinalityEvidence`
- `EvidenceAboutForeignAuthority`

Every positive evidence product states:

`product_universe = evidence`

`grants_local_authority = false`

`grants_external_effect_authority = false`

### AuthorityProduct

Frozen v1 kinds:

- `LocalCapabilityLease`
- `LocalAuthorization`

Authority products are locally minted execution-capacity objects. They are not subclasses of evidence products.

There is no generic `Receipt`, `TrustedObject`, `promote()`, `upgradeTrust()`, `asAuthority()`, or scalar `trust_level` conversion between the universes.

## Standards namespace boundary

External standards-defined proof objects keep their own names and semantics. A COSE/SCITT receipt can be represented as `ExternalProofEvidence`; LEX-NET does not redefine it as a generic workflow or authorization receipt.

## Domain-separated identity

Evidence commitments use:

`SHA256("LEX-NET/EVIDENCE/v1" || canonical_product_fields)`

Authority commitments use:

`SHA256("LEX-NET/AUTHORITY/v1" || canonical_authority_fields)`

Identical visible fields in different universes therefore do not share product identity.

## Requirement satisfaction

Consumers ask:

`Satisfies(requirement, evidence_set, evaluation_time)`

They do not ask whether evidence has a high enough score.

A requirement atom binds:

- exact evidence kind;
- subject;
- purpose;
- resource;
- action;
- exact profile commitment;
- accepted disposition;
- whether current satisfiability is required;
- required independent-source count.

The evaluator is closed-world:

1. every required atom must match explicitly;
2. unknown dimensions do not default positive;
3. duplicate commitments count once;
4. evidence sharing one `independence_group` does not become independent by repetition;
5. conflicting evidence about the same proposition yields `ConflictIndeterminate`;
6. majority count does not resolve conflict;
7. OR semantics require an explicit local `alternative_policy_id`;
8. evidence-set order does not affect the result;
9. unrelated extra evidence cannot strengthen a requirement;
10. fresh derived evidence cannot refresh a stale or missing currentness-sensitive predecessor.

## Historical validity versus current satisfiability

Evidence remains append-only historical evidence after expiry or revocation.

A new current decision recursively rechecks every currentness-sensitive predecessor at the explicit evaluation time.

`historically valid evidence != currently satisfiable evidence`

`fresh wrapper != fresh dependency`

## Explicit alternatives

Alternative evidence paths are permitted only when the requirement contains a non-empty locally selected `alternative_policy_id`.

Without that policy, presenting multiple evidence kinds is not implicit logical OR.

## Independence

Multiple products with the same commitment or the same `independence_group` count only once toward an independence threshold.

`N signatures != N independent sources`

unless the frozen provenance/requirement contract establishes the required independence.

## Evidence-to-authority mint boundary

Evidence can justify a local decision to mint authority, but minting is a separate operation:

`MintLocalCapability(local_domain, local_grant_commitment, satisfied_requirement, scope, lifetime, use_budget)`

The mint refuses unless the evidence requirement is `Satisfied`, local mint authorization is explicitly true, the local domain and issuing grant commitment are present, and the use budget is positive.

The output is a new authority-domain object. It is not promoted evidence.

A source-domain capability crossing a sovereignty boundary becomes only `EvidenceAboutForeignAuthority`; the destination must independently mint local authority.

## Consumption boundary

Authority is consumption-aware. A one-use capability cannot execute twice.

Consumption emits `ExecutionEvidence`. Execution evidence never turns back into reusable authority.

## Reference outcomes

- `Satisfied`
- `MissingRequiredEvidence`
- `ScopeMismatch`
- `ProfileMismatch`
- `DispositionMismatch`
- `StaleDependency`
- `MissingDependency`
- `ConflictIndeterminate`
- `IndependenceInsufficient`
- `AlternativePolicyRequired`
- `AuthorityProductRejected`
- `MintNotAuthorized`
- `MintPreconditionUnsatisfied`
- `AuthorityConsumed`

## Golden/adversarial corpus

The frozen corpus covers exact match, kind/scope/profile/disposition mismatch, stale and missing dependencies, duplicate/correlated versus independent evidence, unresolved conflicts, majority non-resolution, explicit versus implicit alternatives, authority-object rejection from the evidence evaluator, authorized/unauthorized minting, domain-separated mint identity, federation degradation to evidence, one-shot consumption, execution-evidence non-escalation, set-order invariance, unrelated evidence, and external-proof non-recognition.

## Non-escalation

This tranche does not establish factual truth.

This tranche does not establish legal recognition.

This tranche does not establish identity validity.

This tranche does not establish authorization.

This tranche does not establish execution success.

This tranche does not establish external finality.

This tranche does not establish production security.

No v1 evidence product grants local authority or external-effect authority.

A PASS establishes only the frozen typed-product, satisfaction, currentness, independence, mint-boundary, and consumption model.
