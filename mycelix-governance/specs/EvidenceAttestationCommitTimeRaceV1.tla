---------------- MODULE EvidenceAttestationCommitTimeRaceV1 ----------------
EXTENDS Naturals

CONSTANTS
  BoundAuthorityEpoch,
  BoundRequestCommitment,
  BoundPolicyEpoch,
  BoundCapabilityExpiry,
  BoundTime

VARIABLES
  phase,
  currentAuthorityEpoch,
  currentRequestCommitment,
  currentPolicyEpoch,
  currentCapabilityExpiry,
  currentTime,
  preflightAuthorized,
  observedAuthorityEpoch,
  observedRequestCommitment,
  observedPolicyEpoch,
  observedCapabilityExpiry,
  observedTime,
  effectCommitted

vars ==
  <<phase, currentAuthorityEpoch, currentRequestCommitment,
    currentPolicyEpoch, currentCapabilityExpiry, currentTime,
    preflightAuthorized, observedAuthorityEpoch,
    observedRequestCommitment, observedPolicyEpoch,
    observedCapabilityExpiry, observedTime, effectCommitted>>

Init ==
  /\ phase = "preflight"
  /\ currentAuthorityEpoch = BoundAuthorityEpoch
  /\ currentRequestCommitment = BoundRequestCommitment
  /\ currentPolicyEpoch = BoundPolicyEpoch
  /\ currentCapabilityExpiry = BoundCapabilityExpiry
  /\ currentTime = BoundTime
  /\ preflightAuthorized = FALSE
  /\ observedAuthorityEpoch = 0
  /\ observedRequestCommitment = ""
  /\ observedPolicyEpoch = 0
  /\ observedCapabilityExpiry = 0
  /\ observedTime = 0
  /\ effectCommitted = FALSE

Preflight ==
  /\ phase = "preflight"
  /\ preflightAuthorized' = TRUE
  /\ observedAuthorityEpoch' = currentAuthorityEpoch
  /\ observedRequestCommitment' = currentRequestCommitment
  /\ observedPolicyEpoch' = currentPolicyEpoch
  /\ observedCapabilityExpiry' = currentCapabilityExpiry
  /\ observedTime' = currentTime
  /\ phase' = "checked"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment,
                 currentPolicyEpoch, currentCapabilityExpiry,
                 currentTime, effectCommitted>>

SafeCommit ==
  /\ phase = "checked"
  /\ preflightAuthorized
  /\ currentAuthorityEpoch = observedAuthorityEpoch
  /\ currentRequestCommitment = observedRequestCommitment
  /\ currentPolicyEpoch = observedPolicyEpoch
  /\ currentTime < currentCapabilityExpiry
  /\ effectCommitted' = TRUE
  /\ phase' = "committed"
  /\ UNCHANGED <<currentAuthorityEpoch, currentRequestCommitment,
                 currentPolicyEpoch, currentCapabilityExpiry,
                 currentTime, preflightAuthorized,
                 observedAuthorityEpoch, observedRequestCommitment,
                 observedPolicyEpoch, observedCapabilityExpiry,
                 observedTime>>

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
    /\ observedPolicyEpoch = BoundPolicyEpoch
    /\ observedCapabilityExpiry = BoundCapabilityExpiry
    /\ observedTime = BoundTime

CommitRequiresCurrentRevalidation ==
  effectCommitted =>
    /\ preflightAuthorized
    /\ currentAuthorityEpoch = observedAuthorityEpoch
    /\ currentRequestCommitment = observedRequestCommitment
    /\ currentPolicyEpoch = observedPolicyEpoch
    /\ currentTime < currentCapabilityExpiry
    /\ currentTime >= observedTime

CommitOnlyAfterCheck ==
  effectCommitted => preflightAuthorized

=========================================================================
