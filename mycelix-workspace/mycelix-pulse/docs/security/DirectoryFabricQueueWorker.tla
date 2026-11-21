---------------- MODULE DirectoryFabricQueueWorker ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    Workers,
    NoWorker,
    CommandFenceGeneration,
    CommandExpiresAt,
    MaxDeliveries,
    MaxClaimEpoch,
    IgnoreOwnerFence,
    IgnoreWorkerClaimEpoch,
    AllowRetryWhileUnknown,
    AckOnDelivery

ASSUME /\ Workers # {}
       /\ NoWorker \notin Workers
       /\ CommandFenceGeneration > 0
       /\ CommandExpiresAt > 0
       /\ MaxDeliveries > 0
       /\ MaxClaimEpoch > 0
       /\ IgnoreOwnerFence \in BOOLEAN
       /\ IgnoreWorkerClaimEpoch \in BOOLEAN
       /\ AllowRetryWhileUnknown \in BOOLEAN
       /\ AckOnDelivery \in BOOLEAN

VARIABLES
    now,
    fenceGeneration,
    fenceAcknowledgedGeneration,
    deliveryCount,
    claimEpoch,
    activeWorker,
    effectState,
    effectCommits,
    commitWasCurrentFence,
    commitWasCurrentClaim,
    commitBeforeExpiry,
    retryWhileUnknown,
    brokerAcked,
    reconciliationTaskDurable

vars ==
    <<now, fenceGeneration, fenceAcknowledgedGeneration, deliveryCount,
      claimEpoch, activeWorker, effectState, effectCommits,
      commitWasCurrentFence, commitWasCurrentClaim, commitBeforeExpiry,
      retryWhileUnknown, brokerAcked, reconciliationTaskDurable>>

Init ==
    /\ now = 0
    /\ fenceGeneration = CommandFenceGeneration
    /\ fenceAcknowledgedGeneration = CommandFenceGeneration
    /\ deliveryCount = 0
    /\ claimEpoch = 0
    /\ activeWorker = NoWorker
    /\ effectState = "unstarted"
    /\ effectCommits = 0
    /\ commitWasCurrentFence = TRUE
    /\ commitWasCurrentClaim = TRUE
    /\ commitBeforeExpiry = TRUE
    /\ retryWhileUnknown = FALSE
    /\ brokerAcked = FALSE
    /\ reconciliationTaskDurable = FALSE

Deliver ==
    /\ ~brokerAcked
    /\ deliveryCount < MaxDeliveries
    /\ deliveryCount' = deliveryCount + 1
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    claimEpoch, activeWorker, effectState, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

Claim(w) ==
    /\ w \in Workers
    /\ deliveryCount > 0
    /\ ~brokerAcked
    /\ claimEpoch < MaxClaimEpoch
    /\ \/ effectState = "unstarted"
       \/ effectState = "confirmed_no_effect"
       \/ /\ AllowRetryWhileUnknown
          /\ effectState \in {"unknown", "unknown_task_durable"}
    /\ claimEpoch' = claimEpoch + 1
    /\ activeWorker' = w
    /\ retryWhileUnknown' =
          retryWhileUnknown \/ (effectState \in {"unknown", "unknown_task_durable"})
    /\ effectState' = "in_flight"
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, effectCommits, commitWasCurrentFence,
                    commitWasCurrentClaim, commitBeforeExpiry, brokerAcked,
                    reconciliationTaskDurable>>

AdvanceAndAcknowledgeFence ==
    /\ fenceGeneration = CommandFenceGeneration
    /\ fenceGeneration' = fenceGeneration + 1
    /\ fenceAcknowledgedGeneration' = fenceGeneration + 1
    /\ UNCHANGED <<now, deliveryCount, claimEpoch, activeWorker, effectState,
                    effectCommits, commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

Tick ==
    /\ now < CommandExpiresAt
    /\ now' = now + 1
    /\ UNCHANGED <<fenceGeneration, fenceAcknowledgedGeneration, deliveryCount,
                    claimEpoch, activeWorker, effectState, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

Commit(w, suppliedClaimEpoch) ==
    /\ activeWorker = w
    /\ w \in Workers
    /\ suppliedClaimEpoch > 0
    /\ (IgnoreWorkerClaimEpoch \/ suppliedClaimEpoch = claimEpoch)
    /\ (IgnoreOwnerFence \/ fenceGeneration = CommandFenceGeneration)
    /\ now < CommandExpiresAt
    /\ effectState = "in_flight"
    /\ effectCommits < 2
    /\ effectCommits' = effectCommits + 1
    /\ commitWasCurrentFence' = (fenceGeneration = CommandFenceGeneration)
    /\ commitWasCurrentClaim' = (suppliedClaimEpoch = claimEpoch)
    /\ commitBeforeExpiry' = (now < CommandExpiresAt)
    /\ effectState' = "committed"
    /\ activeWorker' = NoWorker
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

UnknownAfterNoEffect ==
    /\ effectState = "in_flight"
    /\ effectState' = "unknown"
    /\ activeWorker' = NoWorker
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

UnknownAfterPossibleEffect(w) ==
    /\ activeWorker = w
    /\ w \in Workers
    /\ effectState = "in_flight"
    /\ (IgnoreOwnerFence \/ fenceGeneration = CommandFenceGeneration)
    /\ now < CommandExpiresAt
    /\ effectCommits < 2
    /\ effectCommits' = effectCommits + 1
    /\ commitWasCurrentFence' = (fenceGeneration = CommandFenceGeneration)
    /\ commitWasCurrentClaim' = TRUE
    /\ commitBeforeExpiry' = TRUE
    /\ effectState' = "unknown"
    /\ activeWorker' = NoWorker
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

PersistReconciliationTask ==
    /\ effectState = "unknown"
    /\ reconciliationTaskDurable = FALSE
    /\ reconciliationTaskDurable' = TRUE
    /\ effectState' = "unknown_task_durable"
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, activeWorker, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked>>

ReconcileCommitted ==
    /\ effectState \in {"unknown", "unknown_task_durable"}
    /\ effectCommits = 1
    /\ effectState' = "committed"
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, activeWorker, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

ReconcileNoEffect ==
    /\ effectState \in {"unknown", "unknown_task_durable"}
    /\ effectCommits = 0
    /\ effectState' = "confirmed_no_effect"
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, activeWorker, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

RejectBeforeAttempt ==
    /\ deliveryCount > 0
    /\ effectState = "unstarted"
    /\ \/ now >= CommandExpiresAt
       \/ fenceGeneration > CommandFenceGeneration
    /\ effectState' = "rejected_before_attempt"
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, activeWorker, effectCommits,
                    commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown, brokerAcked,
                    reconciliationTaskDurable>>

AcknowledgeBroker ==
    /\ ~brokerAcked
    /\ \/ effectState \in {"committed", "confirmed_no_effect",
                           "rejected_before_attempt", "unknown_task_durable"}
       \/ /\ AckOnDelivery
          /\ deliveryCount > 0
    /\ brokerAcked' = TRUE
    /\ UNCHANGED <<now, fenceGeneration, fenceAcknowledgedGeneration,
                    deliveryCount, claimEpoch, activeWorker, effectState,
                    effectCommits, commitWasCurrentFence, commitWasCurrentClaim,
                    commitBeforeExpiry, retryWhileUnknown,
                    reconciliationTaskDurable>>

Next ==
    \/ Deliver
    \/ \E w \in Workers : Claim(w)
    \/ AdvanceAndAcknowledgeFence
    \/ Tick
    \/ \E w \in Workers, e \in 0..MaxClaimEpoch : Commit(w, e)
    \/ UnknownAfterNoEffect
    \/ \E w \in Workers : UnknownAfterPossibleEffect(w)
    \/ PersistReconciliationTask
    \/ ReconcileCommitted
    \/ ReconcileNoEffect
    \/ RejectBeforeAttempt
    \/ AcknowledgeBroker

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ now \in 0..CommandExpiresAt
    /\ fenceGeneration \in Nat
    /\ fenceAcknowledgedGeneration \in Nat
    /\ deliveryCount \in 0..MaxDeliveries
    /\ claimEpoch \in 0..MaxClaimEpoch
    /\ activeWorker \in Workers \cup {NoWorker}
    /\ effectState \in {"unstarted", "in_flight", "unknown",
                        "unknown_task_durable", "confirmed_no_effect",
                        "committed", "rejected_before_attempt"}
    /\ effectCommits \in 0..2
    /\ commitWasCurrentFence \in BOOLEAN
    /\ commitWasCurrentClaim \in BOOLEAN
    /\ commitBeforeExpiry \in BOOLEAN
    /\ retryWhileUnknown \in BOOLEAN
    /\ brokerAcked \in BOOLEAN
    /\ reconciliationTaskDurable \in BOOLEAN

AtMostOneEffectCommit ==
    effectCommits <= 1

NoCommitAfterFenceWasStale ==
    effectCommits = 0 \/ commitWasCurrentFence

NoCommitFromStaleWorkerClaim ==
    effectCommits = 0 \/ commitWasCurrentClaim

NoCommitAfterDecisionExpiry ==
    effectCommits = 0 \/ commitBeforeExpiry

NoRetryWhileUnknown ==
    retryWhileUnknown = FALSE

NoBrokerAckBeforeDurableDisposition ==
    ~brokerAcked
    \/ effectState \in {"committed", "confirmed_no_effect",
                        "rejected_before_attempt", "unknown_task_durable"}

UnknownIsNotTerminalSuccess ==
    ~(brokerAcked /\ effectState = "unknown")

FenceAcknowledgementNeverRegresses ==
    fenceAcknowledgedGeneration <= fenceGeneration

=============================================================================
