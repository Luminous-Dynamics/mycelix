# LEX-NET-028 — Typed Transition Evidence and Authority Separation v1 R2

Status: executable research contract. Language-neutral, deterministic, zero-network, and non-authoritative.

## Governing theorems

`valid evidence != sufficient evidence != local authority != execution != external finality`

`portable evidence != portable authority`

`Satisfied string != bound satisfaction evaluation`

`local authorization boolean != authority-domain authorization product`

A caller cannot acquire authority merely by naming a positive outcome.

## Qualified parent and lineage

R2 is a sibling replacement candidate directly on qualified LEX-NET-032 head:

`117aa1e9c8701589d56dc55dc581f1191a97a071`

R1 head `1fc40cf6f6c6a0afbad5ab9bd85070623a48e63e` remains frozen historical pre-execution evidence. R2 does not rewrite it.

## Disjoint product universes

### EvidenceProduct

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
- `SatisfactionEvaluation`

Every evidence product has:

`product_universe = evidence`

`grants_local_authority = false`

`grants_external_effect_authority = false`

### AuthorityProduct

- `LocalCapabilityLease`
- `LocalAuthorization`

Authority products are not evidence subclasses. No generic `Receipt`, `promote()`, `asAuthority()`, `upgradeTrust()`, scalar trust score, or implicit conversion crosses the universes.

External standards-defined receipt/proof terms keep their own semantics; a COSE/SCITT proof may enter only as `ExternalProofEvidence`.

## Domain-separated identities

R2 uses three domains:

`LEX-NET/EVIDENCE/v2`

`LEX-NET/AUTHORITY/v2`

`LEX-NET/REQUIREMENT/v1`

Requirement, evidence, and authority identity are therefore not interchangeable even when visible fields overlap.

## Closed-world satisfaction

Consumers evaluate:

`Satisfies(requirement, evidence_set, evaluation_time)`

Requirement commitments bind exact product kind, subject, purpose, resource, action, profile commitment, accepted disposition, currentness requirement, and independence count.

Rules:

1. an empty requirement is rejected;
2. a malformed/uncommitted requirement is rejected;
3. authority objects are rejected from the evidence evaluator;
4. exact duplicate evidence commitments count once;
5. the same evidence ID with different commitments is `EvidenceIdentityConflict`;
6. wrong kind/scope/profile/disposition does not default positive;
7. conflicting dispositions produce `ConflictIndeterminate`;
8. majority count does not resolve conflict;
9. evidence in one `independence_group` counts once;
10. alternatives require an explicit local `alternative_policy_id`;
11. evidence-set order is irrelevant;
12. unrelated evidence cannot strengthen a requirement;
13. current decisions recursively check currentness-sensitive predecessors;
14. a fresh sufficient candidate is not poisoned merely by an additional stale same-disposition candidate.

`N signatures != N independent sources`

`fresh wrapper != fresh dependency`

## Bound SatisfactionEvaluation

`evaluate(requirement, evidence_set, evaluation_time)` emits `SatisfactionEvaluation`.

The evaluation binds:

- the committed requirement;
- the exact deduplicated evidence commitments visible to the evaluator;
- evaluation time;
- evaluation outcome;
- decision scope;
- evidence-domain commitment.

Historical evidence remains append-only, but new decisions re-evaluate currentness.

## Authority-domain LocalAuthorization

Minting requires an immutable `LocalAuthorization` authority product, not a caller boolean.

It binds:

- local authority domain;
- local grant commitment;
- authorized operation (`mint-capability`);
- exact subject/purpose/resource/action scope;
- currentness;
- authority-domain commitment.

This profile models binding semantics only; it does not establish legitimacy of local grant inputs.

## Evidence-to-authority mint boundary

`MintLocalCapability(satisfaction_evaluation, local_authorization, use_budget)`

refuses unless:

- the `SatisfactionEvaluation` commitment verifies;
- its committed requirement verifies;
- the evaluation outcome is `Satisfied`;
- the `LocalAuthorization` authority commitment verifies;
- authorization operation is exactly `mint-capability`;
- authorization is current at the evaluation time;
- authorization scope exactly equals evaluation scope;
- local domain and local grant commitment are present;
- use budget is positive.

The minted `LocalCapabilityLease` binds both:

`satisfaction_evaluation_commitment`

and:

`mint_authorization_commitment`

It receives a new authority-domain identity. Evidence is never promoted in place.

A source-domain capability crossing a sovereignty boundary becomes only `EvidenceAboutForeignAuthority`; a destination must independently mint local authority.

## Append-only consumption

`LocalCapabilityLease` identity is immutable across use.

Consumption counts prior unique `ExecutionEvidence` products bound to the lease commitment. It emits a new `ExecutionEvidence` with a consumption index; it does not mutate or replace the lease.

A one-use capability cannot execute twice. Duplicate copies of one execution record do not consume two uses.

`execution evidence != reusable authority`

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
- `EvidenceIdentityConflict`
- `EmptyRequirementRejected`
- `EvaluationBindingMismatch`
- `MintAuthorizationMismatch`
- `MintPreconditionUnsatisfied`
- `AuthorityConsumed`

## Golden/adversarial corpus

The frozen 34-fixture corpus includes the R1 type/scope/currentness/conflict/independence cases plus: empty-requirement rejection; conflicting evidence IDs; bound satisfaction evaluation; bare-string mint rejection; tampered-evaluation rejection; unsatisfied-evaluation rejection; evidence-not-authorization rejection; wrong-scope/expired mint authorization; capability binding to both prerequisite commitments; append-only consumption; duplicate execution non-double-counting; and fresh-candidate survival in the presence of an extra stale same-disposition artifact.

## Non-escalation and nonclaims

This tranche does not establish factual truth.

This tranche does not establish legal recognition.

This tranche does not establish identity validity.

This tranche does not establish authorization.

This tranche does not establish execution success.

This tranche does not establish external finality.

This tranche does not establish production security.

This tranche does not establish legitimacy of local grant inputs.

A PASS establishes only the frozen product-universe, commitment-binding, satisfaction, currentness, independence, mint-boundary, and append-only consumption model.
