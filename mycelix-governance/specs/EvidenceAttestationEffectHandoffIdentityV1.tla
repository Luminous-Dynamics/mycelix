---------------- MODULE EvidenceAttestationEffectHandoffIdentityV1 ----------------
EXTENDS Naturals

CONSTANTS
  DecisionId, RecordedDecisionId,
  IntentId, RecordedIntentId,
  OperationCommitment, RecordedOperationCommitment,
  UseCommitment, RecordedUseCommitment,
  Target, RecordedTarget,
  Adapter, RecordedAdapter,
  InvocationId, RecordedInvocationId

VARIABLES attemptCommitted

vars == <<attemptCommitted>>

Init == attemptCommitted = FALSE
CommitAttempt == attemptCommitted' = TRUE
Next == CommitAttempt

TypeOK == attemptCommitted \in BOOLEAN

DecisionIdentityConserved ==
  ~attemptCommitted \/ RecordedDecisionId = DecisionId

IntentIdentityConserved ==
  ~attemptCommitted \/ RecordedIntentId = IntentId

OperationIdentityConserved ==
  ~attemptCommitted \/ RecordedOperationCommitment = OperationCommitment

UseCommitmentConserved ==
  ~attemptCommitted \/ RecordedUseCommitment = UseCommitment

TargetConserved ==
  ~attemptCommitted \/ RecordedTarget = Target

AdapterConserved ==
  ~attemptCommitted \/ RecordedAdapter = Adapter

InvocationConserved ==
  ~attemptCommitted \/ RecordedInvocationId = InvocationId

EffectHandoffIdentityExact ==
  ~attemptCommitted \/
    /\ RecordedDecisionId = DecisionId
    /\ RecordedIntentId = IntentId
    /\ RecordedOperationCommitment = OperationCommitment
    /\ RecordedUseCommitment = UseCommitment
    /\ RecordedTarget = Target
    /\ RecordedAdapter = Adapter
    /\ RecordedInvocationId = InvocationId

=========================================================================
