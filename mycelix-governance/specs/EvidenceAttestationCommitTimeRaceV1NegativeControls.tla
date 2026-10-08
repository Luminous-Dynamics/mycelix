---------------- MODULE EvidenceAttestationCommitTimeRaceV1NegativeControls ----------------
EXTENDS EvidenceAttestationCommitTimeRaceV1

CONSTANT Control

RaceMutation ==
  /\ phase = "checked"
  /\ currentAuthorityEpoch' =
       IF Control = "authority-epoch" THEN currentAuthorityEpoch + 1
       ELSE currentAuthorityEpoch
  /\ currentRequestCommitment' =
       IF Control = "request-commitment" THEN "changed"
       ELSE currentRequestCommitment
  /\ currentPolicyEpoch' =
       IF Control = "policy-epoch" THEN currentPolicyEpoch + 1
       ELSE currentPolicyEpoch
  /\ currentCapabilityExpiry' =
       IF Control = "capability-expiry" THEN currentTime
       ELSE currentCapabilityExpiry
  /\ currentTime' = currentTime
  /\ phase' = "mutated"
  /\ UNCHANGED <<preflightAuthorized, observedAuthorityEpoch,
                 observedRequestCommitment, observedPolicyEpoch,
                 observedCapabilityExpiry, observedTime,
                 effectCommitted>>

UnsafeCommit ==
  /\ phase = "mutated"
  /\ effectCommitted' = TRUE
  /\ phase' = "committed"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment,
                 currentPolicyEpoch, currentCapabilityExpiry,
                 currentTime, preflightAuthorized,
                 observedAuthorityEpoch, observedRequestCommitment,
                 observedPolicyEpoch, observedCapabilityExpiry,
                 observedTime>>

NegativeNext ==
  Preflight \/ RaceMutation \/ UnsafeCommit

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
