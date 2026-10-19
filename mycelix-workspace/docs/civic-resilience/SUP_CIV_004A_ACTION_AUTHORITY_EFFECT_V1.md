# SUP-CIV-004A — Support action authority / execution / effect separation v1

Status: semantic contract only  
Parent subject: `main@a85369699099d4c7524e502e531735eed4ab36f4`  
Tracking issue: #2151  
Hardening program: #2030

## Purpose

Freeze the authority/effect boundary required before Commons Support autonomous-action records can be composed into Civic Resilience.

At the audited parent subject, Support actions contain fields including `approved`, `executed`, `success`, rollback data and `AutonomyLevel::FullAutonomous`; coordinator paths use Civic participation requirements including `civic_requirement_proposal()` around approval/execution operations.

Those may be meaningful legacy Support workflow semantics, but Civic adoption must not reinterpret them as institutional authorization or verified external effect.

This tranche changes no runtime behavior and does not determine which institution/policy may authorize any real action.

## Core separation theorem

```text
CivicEligibility
!= SupportActionProposal
!= SupportActionAuthorization
!= SupportExecutionRequest
!= SupportExecutionReceipt
!= SupportEffectObservation
!= OutcomeEffect
!= SupportRollbackReceipt
```

No stage inherits the next stage's meaning by existence or boolean mutation.

## Closed artifact vocabulary

The v1 semantic artifacts are:

```text
SupportActionProposal
SupportActionAuthorization
SupportExecutionRequest
SupportExecutionReceipt
SupportEffectObservation
SupportRollbackRequest
SupportRollbackReceipt
LegacySupportActionState
```

## SupportActionProposal

A proposal means only that one bounded action is suggested against one exact operational context.

Semantic refs:

```text
action_proposal_id
stable_ticket_or_case_ref
ticket_or_context_revision_ref
action_kind_ref
proposer_ref
proposal_reason_ref
predicted_effect_ref | None
risk_profile_ref | None
rollback_plan_ref | None
created_at_evidence_ref
limitations[]
```

```text
ActionProposal != AuthorityToExecute
SymthaeaProposal != Authorization
```

## SupportActionAuthorization

Authorization binds an externally owned authority/policy decision to one exact proposal and effect scope.

Semantic refs:

```text
action_authorization_id
action_proposal_ref
authority_source_ref
authorizer_ref
policy_or_delegation_ref
allowed_effect_scope_ref
constraints[]
valid_from_ref | None
expiry_ref | None
revocation_ref | None
provenance_ref
```

The contract intentionally contains no universal `approved=true` theorem.

```text
LegacyApprovedBool != InstitutionalAuthorization
```

A valid authorization for action A, target X or scope S cannot silently authorize action B, target Y or broader scope.

## Civic eligibility boundary

A Civic/Sovereign participation requirement may establish eligibility under its owning model. It does not manufacture institutional or effect authority.

```text
CivicEligibility != InstitutionalAuthorization
CivicRequirementSatisfied != DelegatedAuthority
CivicTier != InstitutionalRole
```

In particular, proposal eligibility is not an approval/execution theorem:

```text
civic_requirement_proposal() != AuthorityToApprove
civic_requirement_proposal() != AuthorityToExecute
```

A runtime may require both eligibility evidence and independent domain/institution authority evidence where both are relevant.

## SupportExecutionRequest

An execution request is the exact operation handed to one executor/tool/system under one authorization.

Semantic refs:

```text
execution_request_id
action_authorization_ref
executor_ref
target_ref
exact_operation_ref
input_or_parameter_commitment_ref
idempotency_or_nonce_ref
request_time_ref
constraints[]
```

```text
ExecutionRequest != ExecutionOccurred
```

The request must not broaden the authorization's target/effect scope.

## SupportExecutionReceipt

An execution receipt records evidence from the executor/runtime about what it attempted and observed.

Semantic refs:

```text
execution_receipt_id
execution_request_ref
executor_ref
execution_environment_ref
started_at_ref
completed_at_ref | None
exit_or_result_ref
artifact_or_log_refs[]
rollback_material_ref | None
limitations[]
```

```text
LegacyExecutedBool != ConfirmedExternalEffect
ExecutionReceipt != CompletionVerification
ExecutionReceipt != OutcomeEffect
```

The receipt can be evidence consumed by a later effect/completion verifier without becoming that verifier.

## SupportEffectObservation

An effect observation means that one process observed one bounded external/system effect predicate after execution.

Semantic refs:

```text
effect_observation_id
execution_receipt_ref
effect_predicate_ref
observation_method_ref
observer_ref
observation_time_ref
observation_result_ref
limitations[]
```

```text
SupportEffectObservation != CivicCompletionVerification
SupportEffectObservation != CausalOutcomeEffect
```

The observed predicate must stay explicit. Observing a service restarted does not prove a resident outcome improved; observing a file write does not prove the broader task succeeded.

## Success boundary

The current legacy `success` field must not be promoted into a universal success theorem.

```text
LegacySuccessBool != OutcomeEffect
LegacySuccessBool != CompletionVerification
LegacySuccessBool != BeneficiaryAcceptance
```

A future adapter may project it only as `LegacySupportActionState` with limitations unless stronger evidence exists.

## Autonomy boundary

`AutonomyLevel::FullAutonomous` remains a Support-domain configuration label, not unbounded Civic authority.

```text
AutonomyLevel::FullAutonomous != CivicAuthority
AutonomyLevel::FullAutonomous != UnboundedExecutionAuthority
```

Automation may reduce how often a human has to issue a request under an exact policy; it cannot expand the policy's authority scope.

## Prediction boundary

Prediction confidence is epistemic/operational metadata, not event truth or authorization.

```text
PredictionConfidence != EffectTruth
PredictionConfidence != IncidentTruth
PredictionConfidence != Authorization
```

A preemptive alert can trigger review/proposal flows without automatically becoming a factual incident or executable order.

## Rollback separation

Rollback plan, rollback request, rollback execution and rollback verification are separate.

`SupportRollbackRequest` binds:

```text
original_execution_receipt_ref
rollback_authorization_ref
rollback_operation_ref
executor_ref
reason_ref
request_time_ref
```

`SupportRollbackReceipt` binds:

```text
rollback_request_ref
executor_ref
result_ref
artifact_or_log_refs[]
observed_residual_effect_refs[]
limitations[]
```

```text
RollbackStepsPresent != RollbackPossible
LegacyRolledBackBool != RollbackVerified
RollbackReceipt != OriginalEffectErased
```

Partial and failed rollback must remain representable.

## Idempotency / replay boundary

Consequential execution requests must bind an idempotency/nonce/replay policy appropriate to the operation.

```text
SameAuthorizedRequestReplayed != NewAuthorization
```

A duplicate request must not cause an unintended duplicate effect merely because its authorization remains otherwise valid.

## Revocation / expiry boundary

Authorization validity is evaluated for the exact execution request under its policy.

```text
AuthorizationOnceValid != AuthorizationAlwaysValid
```

Expired/revoked authorization cannot be treated as current execution permission solely because a legacy `approved` field remains true.

## Legacy projection

Historical `AutonomousAction` records may retain useful audit evidence but are represented as:

```text
LegacySupportActionState
```

until stronger lineage is independently established.

Field renaming is not evidence upgrading.

```text
LegacyApprovedBool != SupportActionAuthorization
LegacyExecutedBool != SupportExecutionReceipt
LegacySuccessBool != SupportEffectObservation
LegacyRolledBackBool != SupportRollbackReceipt
```

## Required runtime refusals

A future executable tranche must cover at least:

- proposal-eligible actor cannot authorize solely because proposal requirement passed;
- authorization for action A cannot execute action B;
- authorization for target X cannot execute against target Y;
- expired/revoked authorization refuses execution;
- executor mismatch refuses execution;
- request parameters outside allowed scope refuse execution;
- replay/duplicate request obeys idempotency policy;
- execution receipt without observed target effect does not become effect success;
- legacy `success=true` cannot become outcome improvement;
- rollback request without rollback receipt cannot become verified rollback;
- partial/failed rollback remains partial/failed;
- `FullAutonomous` cannot bypass authorization scope;
- prediction confidence cannot become incident truth;
- legacy booleans cannot be upgraded to new artifacts without evidence.

## Required non-equivalences

The closed v1 registry is:

```text
CivicEligibility != InstitutionalAuthorization
CivicRequirementSatisfied != DelegatedAuthority
CivicTier != InstitutionalRole
civic_requirement_proposal() != AuthorityToApprove
civic_requirement_proposal() != AuthorityToExecute
ActionProposal != AuthorityToExecute
SymthaeaProposal != Authorization
LegacyApprovedBool != InstitutionalAuthorization
ExecutionRequest != ExecutionOccurred
LegacyExecutedBool != ConfirmedExternalEffect
ExecutionReceipt != CompletionVerification
ExecutionReceipt != OutcomeEffect
SupportEffectObservation != CivicCompletionVerification
SupportEffectObservation != CausalOutcomeEffect
LegacySuccessBool != OutcomeEffect
LegacySuccessBool != CompletionVerification
LegacySuccessBool != BeneficiaryAcceptance
AutonomyLevel::FullAutonomous != CivicAuthority
AutonomyLevel::FullAutonomous != UnboundedExecutionAuthority
PredictionConfidence != EffectTruth
PredictionConfidence != IncidentTruth
PredictionConfidence != Authorization
RollbackStepsPresent != RollbackPossible
LegacyRolledBackBool != RollbackVerified
RollbackReceipt != OriginalEffectErased
SameAuthorizedRequestReplayed != NewAuthorization
AuthorizationOnceValid != AuthorizationAlwaysValid
LegacyApprovedBool != SupportActionAuthorization
LegacyExecutedBool != SupportExecutionReceipt
LegacySuccessBool != SupportEffectObservation
LegacyRolledBackBool != SupportRollbackReceipt
```

## Deferred runtime decisions

Explicitly deferred:

- which authority source owns approval for each action class;
- institutional role/delegation representation;
- exact executor capability model;
- idempotency implementation;
- effect observation methods;
- risk classification;
- human-review requirements;
- action-specific rollback policy;
- Civic Resilience completion-verification composition.

## Continuation

```text
SUP-CIV-004A  authority/execution/effect semantic split     <- this tranche
SUP-CIV-004B  append-only proposal + authorization runtime
SUP-CIV-004C  execution/rollback receipts + replay controls
SUP-CIV-004D  legacy action projection/refusal tests
SUP-CIV-005   narrow Civic Resilience Support adapter
```

## Qualification claim ceiling

A PASS may establish only this exact semantic separation, closed non-equivalence registry and required refusal surface.

It does not establish institutional legitimacy, correctness of any real-world authorization, physical/system effect truth, Civic completion, outcome improvement, safety of arbitrary autonomous actions, privacy compliance, Johannesburg readiness or deployment readiness.
