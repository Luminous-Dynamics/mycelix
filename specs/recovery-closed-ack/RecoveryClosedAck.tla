---------------- MODULE RecoveryClosedAck ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    Candidates,
    MaxPolicyEpoch,
    MaxLsn

ASSUME /\ Candidates # {}
       /\ MaxPolicyEpoch \in Nat \ {0}
       /\ MaxLsn \in Nat \ {0}

NoCandidate == "no-candidate"

VARIABLES
    effectCommitted,
    effectEndLsn,
    databaseContainsEffect,
    candidateReplayLsn,
    candidateHasEffect,
    candidateTimelineCompatible,
    allowedCandidates,
    policyEpoch,
    leaseValid,
    externalHighWaterCommitted,
    oldPrimaryFenced,
    currentPrimaryWritable,
    formerPrimaryWritable,
    quarantined,
    finalityObserved,
    certificatePolicyEpoch,
    certificateEffectEndLsn,
    deliveryIdentityBound,
    unknownFinality,
    acked,
    ackPolicyEpoch,
    ackEvidence,
    promotedCandidate

vars ==
    << effectCommitted,
       effectEndLsn,
       databaseContainsEffect,
       candidateReplayLsn,
       candidateHasEffect,
       candidateTimelineCompatible,
       allowedCandidates,
       policyEpoch,
       leaseValid,
       externalHighWaterCommitted,
       oldPrimaryFenced,
       currentPrimaryWritable,
       formerPrimaryWritable,
       quarantined,
       finalityObserved,
       certificatePolicyEpoch,
       certificateEffectEndLsn,
       deliveryIdentityBound,
       unknownFinality,
       acked,
       ackPolicyEpoch,
       ackEvidence,
       promotedCandidate >>

Init ==
    /\ effectCommitted = FALSE
    /\ effectEndLsn = 0
    /\ databaseContainsEffect = FALSE
    /\ candidateReplayLsn = [c \in Candidates |-> 0]
    /\ candidateHasEffect = [c \in Candidates |-> FALSE]
    /\ candidateTimelineCompatible = [c \in Candidates |-> TRUE]
    /\ allowedCandidates = Candidates
    /\ policyEpoch = 1
    /\ leaseValid = TRUE
    /\ externalHighWaterCommitted = FALSE
    /\ oldPrimaryFenced = TRUE
    /\ currentPrimaryWritable = TRUE
    /\ formerPrimaryWritable = FALSE
    /\ quarantined = FALSE
    /\ finalityObserved = FALSE
    /\ certificatePolicyEpoch = 0
    /\ certificateEffectEndLsn = 0
    /\ deliveryIdentityBound = FALSE
    /\ unknownFinality = TRUE
    /\ acked = FALSE
    /\ ackPolicyEpoch = 0
    /\ ackEvidence = [
         recoveryClosed |-> FALSE,
         finalityObserved |-> FALSE,
         highWaterCommitted |-> FALSE,
         deliveryBound |-> FALSE,
         leaseValid |-> FALSE,
         endLsn |-> 0,
         policyEpoch |-> 0]
    /\ promotedCandidate = NoCandidate

EligibleCandidates ==
    {c \in Candidates :
        /\ candidateHasEffect[c]
        /\ candidateTimelineCompatible[c]
        /\ candidateReplayLsn[c] >= effectEndLsn}

RecoveryClosed ==
    ~effectCommitted
    \/ \A c \in allowedCandidates :
         /\ c \in EligibleCandidates
         /\ candidateReplayLsn[c] >= effectEndLsn

CommitProtectedEffect(lsn) ==
    /\ ~effectCommitted
    /\ currentPrimaryWritable
    /\ ~formerPrimaryWritable
    /\ ~quarantined
    /\ lsn \in 1..MaxLsn
    /\ effectCommitted' = TRUE
    /\ effectEndLsn' = lsn
    /\ databaseContainsEffect' = TRUE
    /\ finalityObserved' = FALSE
    /\ certificatePolicyEpoch' = 0
    /\ certificateEffectEndLsn' = 0
    /\ unknownFinality' = TRUE
    /\ UNCHANGED
         << candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            deliveryIdentityBound,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

ReplicateAndObserveCandidate(c, replayLsn, timelineCompatible) ==
    /\ c \in Candidates
    /\ effectCommitted
    /\ replayLsn \in candidateReplayLsn[c]..MaxLsn
    /\ candidateReplayLsn' =
         [candidateReplayLsn EXCEPT ![c] = replayLsn]
    /\ candidateHasEffect' =
         [candidateHasEffect EXCEPT ![c] = (replayLsn >= effectEndLsn)]
    /\ candidateTimelineCompatible' =
         [candidateTimelineCompatible EXCEPT ![c] = timelineCompatible]
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            unknownFinality,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

AdvancePromotionPolicy(newAllowedCandidates) ==
    /\ newAllowedCandidates \subseteq Candidates
    /\ newAllowedCandidates # {}
    /\ policyEpoch < MaxPolicyEpoch
    /\ IF effectCommitted
          THEN newAllowedCandidates \subseteq EligibleCandidates
          ELSE TRUE
    /\ allowedCandidates' = newAllowedCandidates
    /\ policyEpoch' = policyEpoch + 1
    /\ finalityObserved' = FALSE
    /\ certificatePolicyEpoch' = 0
    /\ certificateEffectEndLsn' = 0
    /\ unknownFinality' = TRUE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            deliveryIdentityBound,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

ObserveRecoveryFinality ==
    /\ effectCommitted
    /\ databaseContainsEffect
    /\ RecoveryClosed
    /\ leaseValid
    /\ oldPrimaryFenced
    /\ ~quarantined
    /\ finalityObserved' = TRUE
    /\ certificatePolicyEpoch' = policyEpoch
    /\ certificateEffectEndLsn' = effectEndLsn
    /\ externalHighWaterCommitted' = TRUE
    /\ unknownFinality' = FALSE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            deliveryIdentityBound,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

BindExactDelivery ==
    /\ effectCommitted
    /\ deliveryIdentityBound' = TRUE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            unknownFinality,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

AckSync ==
    /\ ~acked
    /\ effectCommitted
    /\ finalityObserved
    /\ certificatePolicyEpoch = policyEpoch
    /\ certificateEffectEndLsn = effectEndLsn
    /\ externalHighWaterCommitted
    /\ RecoveryClosed
    /\ leaseValid
    /\ oldPrimaryFenced
    /\ deliveryIdentityBound
    /\ ~unknownFinality
    /\ ~quarantined
    /\ acked' = TRUE
    /\ ackPolicyEpoch' = policyEpoch
    /\ ackEvidence' = [
         recoveryClosed |-> RecoveryClosed,
         finalityObserved |-> finalityObserved,
         highWaterCommitted |-> externalHighWaterCommitted,
         deliveryBound |-> deliveryIdentityBound,
         leaseValid |-> leaseValid,
         endLsn |-> effectEndLsn,
         policyEpoch |-> certificatePolicyEpoch]
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            unknownFinality,
            promotedCandidate >>

IsolateCurrentPrimary ==
    /\ currentPrimaryWritable
    /\ currentPrimaryWritable' = FALSE
    /\ formerPrimaryWritable' = TRUE
    /\ oldPrimaryFenced' = FALSE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            quarantined,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            unknownFinality,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

FenceFormerPrimary ==
    /\ formerPrimaryWritable
    /\ formerPrimaryWritable' = FALSE
    /\ oldPrimaryFenced' = TRUE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            currentPrimaryWritable,
            quarantined,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            unknownFinality,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

PromoteCandidate(c) ==
    /\ c \in allowedCandidates
    /\ ~currentPrimaryWritable
    /\ ~formerPrimaryWritable
    /\ oldPrimaryFenced
    /\ c \in EligibleCandidates
    /\ currentPrimaryWritable' = TRUE
    /\ formerPrimaryWritable' = FALSE
    /\ databaseContainsEffect' = TRUE
    /\ quarantined' = FALSE
    /\ promotedCandidate' = c
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            finalityObserved,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            unknownFinality,
            acked,
            ackPolicyEpoch >>

RestoreStaleBackup ==
    /\ effectCommitted
    /\ ~currentPrimaryWritable
    /\ databaseContainsEffect
    /\ databaseContainsEffect' = FALSE
    /\ currentPrimaryWritable' = FALSE
    /\ formerPrimaryWritable' = TRUE
    /\ oldPrimaryFenced' = FALSE
    /\ quarantined' = TRUE
    /\ finalityObserved' = FALSE
    /\ unknownFinality' = TRUE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            leaseValid,
            externalHighWaterCommitted,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

MarkLeaseUnknown ==
    /\ leaseValid
    /\ leaseValid' = FALSE
    /\ unknownFinality' = TRUE
    /\ finalityObserved' = FALSE
    /\ UNCHANGED
         << effectCommitted,
            effectEndLsn,
            databaseContainsEffect,
            candidateReplayLsn,
            candidateHasEffect,
            candidateTimelineCompatible,
            allowedCandidates,
            policyEpoch,
            externalHighWaterCommitted,
            oldPrimaryFenced,
            currentPrimaryWritable,
            formerPrimaryWritable,
            quarantined,
            certificatePolicyEpoch,
            certificateEffectEndLsn,
            deliveryIdentityBound,
            acked,
            ackPolicyEpoch,
            ackEvidence,
            promotedCandidate >>

Next ==
    \/ \E lsn \in 1..MaxLsn : CommitProtectedEffect(lsn)
    \/ \E c \in Candidates, lsn \in 0..MaxLsn, compatible \in BOOLEAN :
          ReplicateAndObserveCandidate(c, lsn, compatible)
    \/ \E allowed \in SUBSET Candidates : AdvancePromotionPolicy(allowed)
    \/ ObserveRecoveryFinality
    \/ BindExactDelivery
    \/ AckSync
    \/ IsolateCurrentPrimary
    \/ FenceFormerPrimary
    \/ \E c \in Candidates : PromoteCandidate(c)
    \/ RestoreStaleBackup
    \/ MarkLeaseUnknown

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ effectCommitted \in BOOLEAN
    /\ effectEndLsn \in 0..MaxLsn
    /\ databaseContainsEffect \in BOOLEAN
    /\ candidateReplayLsn \in [Candidates -> 0..MaxLsn]
    /\ candidateHasEffect \in [Candidates -> BOOLEAN]
    /\ candidateTimelineCompatible \in [Candidates -> BOOLEAN]
    /\ allowedCandidates \subseteq Candidates
    /\ policyEpoch \in 1..MaxPolicyEpoch
    /\ leaseValid \in BOOLEAN
    /\ externalHighWaterCommitted \in BOOLEAN
    /\ oldPrimaryFenced \in BOOLEAN
    /\ currentPrimaryWritable \in BOOLEAN
    /\ formerPrimaryWritable \in BOOLEAN
    /\ quarantined \in BOOLEAN
    /\ finalityObserved \in BOOLEAN
    /\ certificatePolicyEpoch \in 0..MaxPolicyEpoch
    /\ certificateEffectEndLsn \in 0..MaxLsn
    /\ deliveryIdentityBound \in BOOLEAN
    /\ unknownFinality \in BOOLEAN
    /\ acked \in BOOLEAN
    /\ ackPolicyEpoch \in 0..MaxPolicyEpoch
    /\ ackEvidence \in
         [recoveryClosed : BOOLEAN,
          finalityObserved : BOOLEAN,
          highWaterCommitted : BOOLEAN,
          deliveryBound : BOOLEAN,
          leaseValid : BOOLEAN,
          endLsn : 0..MaxLsn,
          policyEpoch : 0..MaxPolicyEpoch]
    /\ promotedCandidate \in (Candidates \cup {NoCandidate})

NoSplitBrain ==
    ~(currentPrimaryWritable /\ formerPrimaryWritable)

AckImpliesVerifiedFinality ==
    acked =>
      /\ effectCommitted
      /\ ackEvidence.recoveryClosed
      /\ ackEvidence.finalityObserved
      /\ ackEvidence.highWaterCommitted
      /\ ackEvidence.endLsn = effectEndLsn
      /\ ackEvidence.policyEpoch = ackPolicyEpoch
      /\ ackEvidence.policyEpoch \in 1..policyEpoch
      /\ ackEvidence.deliveryBound
      /\ ackEvidence.leaseValid

WritablePrimaryIsAdmitted ==
    currentPrimaryWritable =>
      /\ ~quarantined
      /\ ~formerPrimaryWritable
      /\ databaseContainsEffect = effectCommitted
      /\ oldPrimaryFenced

AcknowledgedEffectSurvivesEveryAllowedPromotion ==
    acked =>
      \A c \in allowedCandidates :
        /\ candidateHasEffect[c]
        /\ candidateTimelineCompatible[c]
        /\ candidateReplayLsn[c] >= effectEndLsn

QuarantinedRestoreCannotWrite ==
    quarantined => ~currentPrimaryWritable

AckedStaleRestoreMustStayQuarantined ==
    acked /\ ~databaseContainsEffect =>
      quarantined /\ ~currentPrimaryWritable

PromotionRequiresPhysicalFencing ==
    currentPrimaryWritable /\ promotedCandidate # NoCandidate =>
      /\ oldPrimaryFenced
      /\ ~formerPrimaryWritable

ExternalEvidenceIsMonotonic ==
    [](externalHighWaterCommitted => externalHighWaterCommitted')

=============================================================================
