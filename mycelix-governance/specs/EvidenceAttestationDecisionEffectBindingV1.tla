---------------- MODULE EvidenceAttestationDecisionEffectBindingV1 ----------------
EXTENDS Naturals

CONSTANTS
  DecisionId, RecordedDecisionId,
  BoundAuthorityEpoch, CurrentAuthorityEpoch,
  BoundRequestCommitment, CurrentRequestCommitment,
  BoundTarget, CurrentTarget,
  BoundPolicyEpoch, CurrentPolicyEpoch,
  BoundAdapterProfile, CurrentAdapterProfile,
  BoundInvocationId, CurrentInvocationId,
  DecisionIssuedAt, DecisionValidityUntil, CapabilityExpiry,
  CurrentTime

VARIABLES admitted

vars == <<admitted>>

Init ==
  admitted = FALSE

AdmitEffect ==
  admitted' = TRUE

Next ==
  AdmitEffect

TypeOK ==
  admitted \in BOOLEAN

DecisionIdentityBound ==
  ~admitted \/ RecordedDecisionId = DecisionId

DecisionIssuedBeforeEffect ==
  DecisionIssuedAt <= CurrentTime

AuthorityEpochBound ==
  ~admitted \/ CurrentAuthorityEpoch = BoundAuthorityEpoch

RequestCommitmentBound ==
  ~admitted \/ CurrentRequestCommitment = BoundRequestCommitment

TargetBound ==
  ~admitted \/ CurrentTarget = BoundTarget

PolicyEpochBound ==
  ~admitted \/ CurrentPolicyEpoch = BoundPolicyEpoch

AdapterIdentityBound ==
  ~admitted \/ CurrentAdapterProfile = BoundAdapterProfile

InvocationIdentityBound ==
  ~admitted \/ CurrentInvocationId = BoundInvocationId

CapabilityCurrentAtEffect ==
  ~admitted \/ CurrentTime < CapabilityExpiry

DecisionHorizonCurrentAtEffect ==
  ~admitted \/ CurrentTime < DecisionValidityUntil

=========================================================================
