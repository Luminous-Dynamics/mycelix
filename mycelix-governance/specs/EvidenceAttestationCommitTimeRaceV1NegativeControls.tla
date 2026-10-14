---------------- MODULE EvidenceAttestationCommitTimeRaceV1NegativeControls ----------------
EXTENDS EvidenceAttestationCommitTimeRaceV1

CONSTANT Control

RaceMutation ==
  /\ phase = "checked"
  /\ currentAuthorityEpoch' =
       IF Control = "authority-epoch" THEN currentAuthorityEpoch + 1 ELSE currentAuthorityEpoch
  /\ currentRequestCommitment' =
       IF Control = "request-commitment" THEN "changed" ELSE currentRequestCommitment
  /\ currentTarget' =
       IF Control = "target" THEN "target-changed" ELSE currentTarget
  /\ currentPolicyEpoch' =
       IF Control = "policy-epoch" THEN currentPolicyEpoch + 1 ELSE currentPolicyEpoch
  /\ currentAdapterProfile' =
       IF Control = "adapter-profile" THEN "adapter-changed" ELSE currentAdapterProfile
  /\ currentInvocationId' =
       IF Control = "invocation-identity" THEN "invocation-changed" ELSE currentInvocationId
  /\ currentCapabilityExpiry' =
       IF Control = "capability-expiry-change" THEN currentCapabilityExpiry + 10 ELSE currentCapabilityExpiry
  /\ currentTime' =
       IF Control = "time-expiry" THEN currentCapabilityExpiry + 1 ELSE currentTime
  /\ phase' = "mutated"
  /\ UNCHANGED <<preflightAuthorized, observedAuthorityEpoch,
                 observedRequestCommitment, observedTarget, observedPolicyEpoch,
                 observedAdapterProfile, observedInvocationId,
                 observedCapabilityExpiry, observedTime, effectCommitted>>

UnsafeCommit ==
  /\ phase = "mutated"
  /\ effectCommitted' = TRUE
  /\ phase' = "committed"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment, currentTarget,
                 currentPolicyEpoch, currentAdapterProfile, currentInvocationId,
                 currentCapabilityExpiry, currentTime, preflightAuthorized,
                 observedAuthorityEpoch, observedRequestCommitment, observedTarget,
                 observedPolicyEpoch, observedAdapterProfile, observedInvocationId,
                 observedCapabilityExpiry, observedTime>>

NegativeNext ==
  Preflight \/ RaceMutation \/ UnsafeCommit

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
