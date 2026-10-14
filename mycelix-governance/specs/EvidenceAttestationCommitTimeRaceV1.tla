---------------- MODULE EvidenceAttestationCommitTimeRaceV1 ----------------
EXTENDS Naturals

CONSTANTS
  BoundAuthorityEpoch,
  BoundRequestCommitment,
  BoundTarget,
  BoundPolicyEpoch,
  BoundAdapterProfile,
  BoundInvocationId,
  BoundCapabilityExpiry,
  BoundTime

VARIABLES
  phase,
  currentAuthorityEpoch,
  currentRequestCommitment,
  currentTarget,
  currentPolicyEpoch,
  currentAdapterProfile,
  currentInvocationId,
  currentCapabilityExpiry,
  currentTime,
  preflightAuthorized,
  observedAuthorityEpoch,
  observedRequestCommitment,
  observedTarget,
  observedPolicyEpoch,
  observedAdapterProfile,
  observedInvocationId,
  observedCapabilityExpiry,
  observedTime,
  effectCommitted

vars ==
  <<phase, currentAuthorityEpoch, currentRequestCommitment, currentTarget,
    currentPolicyEpoch, currentAdapterProfile, currentInvocationId,
    currentCapabilityExpiry, currentTime, preflightAuthorized,
    observedAuthorityEpoch, observedRequestCommitment, observedTarget,
    observedPolicyEpoch, observedAdapterProfile, observedInvocationId,
    observedCapabilityExpiry, observedTime, effectCommitted>>

Init ==
  /\ phase = "preflight"
  /\ currentAuthorityEpoch = BoundAuthorityEpoch
  /\ currentRequestCommitment = BoundRequestCommitment
  /\ currentTarget = BoundTarget
  /\ currentPolicyEpoch = BoundPolicyEpoch
  /\ currentAdapterProfile = BoundAdapterProfile
  /\ currentInvocationId = BoundInvocationId
  /\ currentCapabilityExpiry = BoundCapabilityExpiry
  /\ currentTime = BoundTime
  /\ preflightAuthorized = FALSE
  /\ observedAuthorityEpoch = 0
  /\ observedRequestCommitment = ""
  /\ observedTarget = ""
  /\ observedPolicyEpoch = 0
  /\ observedAdapterProfile = ""
  /\ observedInvocationId = ""
  /\ observedCapabilityExpiry = 0
  /\ observedTime = 0
  /\ effectCommitted = FALSE

Preflight ==
  /\ phase = "preflight"
  /\ preflightAuthorized' = TRUE
  /\ observedAuthorityEpoch' = currentAuthorityEpoch
  /\ observedRequestCommitment' = currentRequestCommitment
  /\ observedTarget' = currentTarget
  /\ observedPolicyEpoch' = currentPolicyEpoch
  /\ observedAdapterProfile' = currentAdapterProfile
  /\ observedInvocationId' = currentInvocationId
  /\ observedCapabilityExpiry' = currentCapabilityExpiry
  /\ observedTime' = currentTime
  /\ phase' = "checked"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment,
                 currentTarget, currentPolicyEpoch, currentAdapterProfile,
                 currentInvocationId, currentCapabilityExpiry, currentTime,
                 effectCommitted>>

SafeCommit ==
  /\ phase = "checked"
  /\ preflightAuthorized
  /\ currentAuthorityEpoch = observedAuthorityEpoch
  /\ currentRequestCommitment = observedRequestCommitment
  /\ currentTarget = observedTarget
  /\ currentPolicyEpoch = observedPolicyEpoch
  /\ currentAdapterProfile = observedAdapterProfile
  /\ currentInvocationId = observedInvocationId
  /\ currentCapabilityExpiry = observedCapabilityExpiry
  /\ currentTime < currentCapabilityExpiry
  /\ currentTime >= observedTime
  /\ effectCommitted' = TRUE
  /\ phase' = "committed"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment,
                 currentTarget, currentPolicyEpoch, currentAdapterProfile,
                 currentInvocationId, currentCapabilityExpiry, currentTime,
                 preflightAuthorized, observedAuthorityEpoch,
                 observedRequestCommitment, observedTarget, observedPolicyEpoch,
                 observedAdapterProfile, observedInvocationId,
                 observedCapabilityExpiry, observedTime>>

Next ==
  Preflight \/ SafeCommit

TypeOK ==
  /\ phase \in {"preflight", "checked", "mutated", "committed"}
  /\ preflightAuthorized \in BOOLEAN
  /\ effectCommitted \in BOOLEAN

PreflightSnapshotExact ==
  phase # "preflight" =>
    /\ observedAuthorityEpoch = BoundAuthorityEpoch
    /\ observedRequestCommitment = BoundRequestCommitment
    /\ observedTarget = BoundTarget
    /\ observedPolicyEpoch = BoundPolicyEpoch
    /\ observedAdapterProfile = BoundAdapterProfile
    /\ observedInvocationId = BoundInvocationId
    /\ observedCapabilityExpiry = BoundCapabilityExpiry
    /\ observedTime = BoundTime

CommitOnlyAfterCheck ==
  effectCommitted => preflightAuthorized

AuthorityEpochRevalidated ==
  ~effectCommitted \/ currentAuthorityEpoch = observedAuthorityEpoch

RequestCommitmentRevalidated ==
  ~effectCommitted \/ currentRequestCommitment = observedRequestCommitment

TargetRevalidated ==
  ~effectCommitted \/ currentTarget = observedTarget

PolicyEpochRevalidated ==
  ~effectCommitted \/ currentPolicyEpoch = observedPolicyEpoch

AdapterIdentityRevalidated ==
  ~effectCommitted \/ currentAdapterProfile = observedAdapterProfile

InvocationIdentityRevalidated ==
  ~effectCommitted \/ currentInvocationId = observedInvocationId

CapabilityExpiryUnchangedAtCommit ==
  ~effectCommitted \/ currentCapabilityExpiry = observedCapabilityExpiry

CapabilityCurrentAtCommit ==
  ~effectCommitted \/ currentTime < currentCapabilityExpiry

CommitTimeMonotone ==
  ~effectCommitted \/ currentTime >= observedTime

=========================================================================
