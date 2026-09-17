# LEX-NET-028 — Typed Transition Evidence and Authority Separation v1 R3

Status: executable research contract. Language-neutral, deterministic, zero-network, and non-authoritative.

## Governing theorems

`valid evidence != sufficient evidence != local authority != execution != external finality`

`portable evidence != portable authority`

`Satisfied string != bound satisfaction evaluation`

`local authorization boolean != authority-domain authorization product`

`satisfied alternative != permission to change decision scope`

`evaluation current at T != authority mintable after support horizon`

`lease validity <= evidence support horizon ∩ local authorization horizon`

`execution evidence != reusable authority`

A caller cannot acquire authority merely by naming a positive outcome, selecting a different alternative scope, replaying an old evaluation, or extending a capability beyond the evidence and authorization that supported it.

## Qualified parent and lineage

R3 is a sibling replacement candidate directly on qualified LEX-NET-032 exact head:

`117aa1e9c8701589d56dc55dc581f1191a97a071`

Historical pre-execution candidates remain frozen:

- R1: `1fc40cf6f6c6a0afbad5ab9bd85070623a48e63e`
- R2: `1aa79715c93f0fbdf2c11ed04594589d4119924f`

R3 does not rewrite either subject.

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

Authority products are not evidence subclasses. There is no generic `Receipt`, `promote()`, `asAuthority()`, `upgradeTrust()`, scalar trust score, or implicit conversion across universes.

External standards-defined proof/receipt terms keep their own semantics. A COSE/SCITT proof may enter only as `ExternalProofEvidence`.

## Domain-separated identities

R3 uses three disjoint commitment domains:

`LEX-NET/EVIDENCE/v3`

`LEX-NET/AUTHORITY/v3`

`LEX-NET/REQUIREMENT/v1`

Requirement, evidence, and authority identities are therefore not interchangeable even when visible fields overlap.

## Closed-world requirement scope

A v1 requirement represents one exact decision scope:

`(subject, purpose, resource, action)`

Every atom in a conjunction and every atom in every explicit alternative must use the same decision scope. A requirement whose alternatives change subject, purpose, resource, or action is:

`RequirementScopeMismatch`

This prevents a satisfied alternative for one action or resource from being rebound to another.

An empty or malformed/uncommitted requirement is rejected.

## Closed-world satisfaction

Consumers evaluate:

`Satisfies(requirement, evidence_set, evaluation_time)`

Rules:

1. authority products are rejected from the evidence evaluator;
2. exact duplicate evidence commitments count once;
3. the same evidence ID with different commitments is `EvidenceIdentityConflict`;
4. wrong kind/scope/profile/disposition does not default positive;
5. conflicting dispositions produce `ConflictIndeterminate`;
6. majority count does not resolve conflict;
7. one `independence_group` counts once;
8. alternatives require an explicit local `alternative_policy_id`;
9. evidence-set order is irrelevant;
10. unrelated evidence cannot strengthen a requirement;
11. current decisions recursively check currentness-sensitive predecessors;
12. a currentness-sensitive support object without an explicit horizon is not silently treated as perpetually current;
13. a fresh sufficient candidate is not poisoned merely by an additional stale same-disposition candidate;
14. a successful alternative is recorded by exact alternative index.

`N signatures != N independent sources`

`fresh wrapper != fresh dependency`

## Bound SatisfactionEvaluation

`evaluate(requirement, evidence_set, evaluation_time)` emits `SatisfactionEvaluation`.

The evaluation binds:

- the committed requirement;
- exact deduplicated evidence commitments visible to the evaluator;
- exact selected support commitments;
- selected predecessor closure;
- selected alternative index, when applicable;
- evaluation time;
- evaluation outcome;
- exact decision scope;
- `support_current_until`, the minimum currentness horizon across selected support and its required predecessor closure.

If no qualifying support is selected, no mintable support horizon exists.

Historical evidence remains append-only. New decisions re-evaluate currentness instead of refreshing stale dependencies through a newer wrapper.

## Authority-domain LocalAuthorization

Minting requires an immutable `LocalAuthorization`, not a caller boolean.

It binds:

- local authority domain;
- local grant commitment;
- operation exactly `mint-capability`;
- exact decision scope;
- exact `SatisfactionEvaluation` commitment it authorizes;
- authorization currentness horizon;
- maximum use budget;
- authority-domain commitment.

This profile models binding semantics only; it does not establish legitimacy of local grant inputs.

## Evidence-to-authority mint boundary

`MintLocalCapability(satisfaction_evaluation, local_authorization, mint_time, requested_current_until, requested_use_budget)`

refuses unless:

- the SatisfactionEvaluation commitment verifies;
- its Requirement commitment verifies;
- evaluation outcome is `Satisfied`;
- mint time is not before evaluation time;
- mint time is not after the selected support horizon;
- LocalAuthorization commitment verifies;
- operation is exactly `mint-capability`;
- LocalAuthorization binds the exact SatisfactionEvaluation commitment;
- authorization is current at mint time;
- authorization decision scope exactly equals evaluation scope;
- requested capability lifetime ends no later than both selected evidence support and LocalAuthorization;
- requested use budget is positive and no greater than the authorization maximum;
- local domain and local grant commitment are present.

The resulting immutable `LocalCapabilityLease` binds:

`satisfaction_evaluation_commitment`

and:

`mint_authorization_commitment`

It also binds mint time, expiry, and use budget. Evidence is never promoted in place.

A source-domain capability crossing a sovereignty boundary becomes only `EvidenceAboutForeignAuthority`; the destination must independently mint local authority.

## Append-only consumption

`LocalCapabilityLease` identity is immutable across use.

Consumption requires an explicit execution time inside the lease interval. It counts prior unique `ExecutionEvidence` products bound to the immutable lease commitment and emits a new `ExecutionEvidence` with a consumption index.

A one-use capability cannot execute twice. Duplicate copies of one execution record do not consume two uses. The same execution evidence ID with conflicting commitments is an integrity conflict.

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
- `RequirementScopeMismatch`
- `EvaluationBindingMismatch`
- `MintAuthorizationMismatch`
- `MintPreconditionUnsatisfied`
- `EvaluationNotCurrent`
- `LeaseLifetimeExceedsSupport`
- `AuthorityNotCurrent`
- `AuthorityConsumed`

## Golden/adversarial corpus

The frozen corpus covers exact matching, type/scope/profile/disposition mismatch, currentness and predecessor failure, stale-extra tolerance, independence, conflicts without majority voting, explicit alternatives, alternative-scope mismatch, empty requirements, authority/evidence separation, evidence-ID conflicts, bound evaluation/support horizon, forged/bare-string/unsatisfied mint attempts, exact evaluation-to-authorization binding, scope and authorization expiry, delayed minting, post-support mint rejection, lease-lifetime attenuation, use-budget attenuation, cross-domain degradation to evidence, append-only consumption, duplicate execution handling, execution expiry, order invariance, and external-proof non-recognition.

## Non-escalation and nonclaims

This tranche does not establish factual truth.

This tranche does not establish legal recognition.

This tranche does not establish identity validity.

This tranche does not establish authorization.

This tranche does not establish execution success.

This tranche does not establish external finality.

This tranche does not establish production security.

This tranche does not establish legitimacy of local grant inputs.

A PASS establishes only the frozen product-universe, commitment-binding, requirement-scope, satisfaction, currentness/support-horizon, independence, mint-attenuation, and append-only consumption model.
