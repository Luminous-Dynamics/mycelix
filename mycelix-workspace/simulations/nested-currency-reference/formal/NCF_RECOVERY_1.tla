---------------- MODULE NCF_RECOVERY_1 ----------------
EXTENDS Naturals, Integers, TLC

CONSTANTS Capacity, MaxOps

ASSUME Capacity \in Nat \ {0}
ASSUME MaxOps \in Nat \ {0}

OldPhases == {"ACTIVE", "DRAINING", "FENCED", "RETIRED"}
NewPhases == {"ABSENT", "PREPARED", "ACTIVE"}
ResponseStates == {"NONE", "UNOBSERVED", "LOST", "ACKED"}
RetryResults == {"NONE", "IDEMPOTENT", "CONFLICT_REJECTED"}

VARIABLES
    oldPhase,
    newPhase,
    oldCapacity,
    newCapacity,
    oldOutstanding,
    oldPending,
    newOutstanding,
    oldSequence,
    terminalSequence,
    reconciledOutstanding,
    reconciled,
    fenced,
    preparedAmount,
    commitRecorded,
    commitCount,
    crashed,
    commitTransferId,
    commitPayloadAmount,
    commitReceiptId,
    responseState,
    retryTransferId,
    retryPayloadAmount,
    retryReceiptId,
    retryResult,
    conflictingRetryAttempted,
    conflictAccepted

vars ==
    << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
       oldPending, newOutstanding, oldSequence, terminalSequence,
       reconciledOutstanding, reconciled, fenced, preparedAmount,
       commitRecorded, commitCount, crashed, commitTransferId,
       commitPayloadAmount, commitReceiptId, responseState,
       retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
       conflictingRetryAttempted, conflictAccepted >>

Init ==
    /\ oldPhase = "ACTIVE"
    /\ newPhase = "ABSENT"
    /\ oldCapacity = Capacity
    /\ newCapacity = 0
    /\ oldOutstanding = 0
    /\ oldPending = 0
    /\ newOutstanding = 0
    /\ oldSequence = 0
    /\ terminalSequence = -1
    /\ reconciledOutstanding = 0
    /\ reconciled = FALSE
    /\ fenced = FALSE
    /\ preparedAmount = 0
    /\ commitRecorded = FALSE
    /\ commitCount = 0
    /\ crashed = FALSE
    /\ commitTransferId = "none"
    /\ commitPayloadAmount = 0
    /\ commitReceiptId = "none"
    /\ responseState = "NONE"
    /\ retryTransferId = "none"
    /\ retryPayloadAmount = 0
    /\ retryReceiptId = "none"
    /\ retryResult = "NONE"
    /\ conflictingRetryAttempted = FALSE
    /\ conflictAccepted = FALSE

OldSpend ==
    \E amount \in 1..Capacity:
        /\ ~crashed
        /\ oldPhase = "ACTIVE"
        /\ oldSequence < MaxOps
        /\ oldOutstanding + oldPending + amount <= oldCapacity
        /\ oldOutstanding' = oldOutstanding + amount
        /\ oldSequence' = oldSequence + 1
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity, oldPending,
               newOutstanding, terminalSequence, reconciledOutstanding,
               reconciled, fenced, preparedAmount, commitRecorded,
               commitCount, crashed, commitTransferId, commitPayloadAmount,
               commitReceiptId, responseState, retryTransferId,
               retryPayloadAmount, retryReceiptId, retryResult,
               conflictingRetryAttempted, conflictAccepted >>

CreatePending ==
    \E amount \in 1..Capacity:
        /\ ~crashed
        /\ oldPhase = "ACTIVE"
        /\ oldSequence < MaxOps
        /\ oldOutstanding + oldPending + amount <= oldCapacity
        /\ oldPending' = oldPending + amount
        /\ oldSequence' = oldSequence + 1
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
               newOutstanding, terminalSequence, reconciledOutstanding,
               reconciled, fenced, preparedAmount, commitRecorded,
               commitCount, crashed, commitTransferId, commitPayloadAmount,
               commitReceiptId, responseState, retryTransferId,
               retryPayloadAmount, retryReceiptId, retryResult,
               conflictingRetryAttempted, conflictAccepted >>

ResolveAllPending ==
    /\ ~crashed
    /\ oldPending > 0
    /\ oldOutstanding' = oldOutstanding + oldPending
    /\ oldPending' = 0
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, newOutstanding,
           oldSequence, terminalSequence, reconciledOutstanding,
           reconciled, fenced, preparedAmount, commitRecorded,
           commitCount, crashed, commitTransferId, commitPayloadAmount,
           commitReceiptId, responseState, retryTransferId,
           retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

BeginDrain ==
    /\ ~crashed
    /\ oldPhase = "ACTIVE"
    /\ oldPhase' = "DRAINING"
    /\ UNCHANGED
        << newPhase, oldCapacity, newCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, responseState,
           retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

OldSettlement ==
    \E amount \in 1..Capacity:
        /\ ~crashed
        /\ oldOutstanding >= amount
        /\ oldOutstanding' = oldOutstanding - amount
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity, oldPending,
               newOutstanding, oldSequence, terminalSequence,
               reconciledOutstanding, reconciled, fenced, preparedAmount,
               commitRecorded, commitCount, crashed, commitTransferId,
               commitPayloadAmount, commitReceiptId, responseState,
               retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
               conflictingRetryAttempted, conflictAccepted >>

Reconcile ==
    /\ ~crashed
    /\ oldPhase = "DRAINING"
    /\ oldPending = 0
    /\ oldPhase' = "DRAINING"
    /\ terminalSequence' = oldSequence
    /\ reconciledOutstanding' = oldOutstanding
    /\ reconciled' = TRUE
    /\ UNCHANGED
        << newPhase, oldCapacity, newCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, fenced, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, responseState,
           retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

Fence ==
    /\ ~crashed
    /\ oldPhase = "DRAINING"
    /\ reconciled
    /\ oldSequence = terminalSequence
    /\ oldPhase' = "FENCED"
    /\ fenced' = TRUE
    /\ UNCHANGED
        << newPhase, oldCapacity, newCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, responseState,
           retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

PrepareTransfer ==
    \E amount \in 1..Capacity:
        /\ ~crashed
        /\ oldPhase = "FENCED"
        /\ fenced
        /\ reconciled
        /\ newPhase = "ABSENT"
        /\ amount <= oldCapacity - reconciledOutstanding
        /\ newPhase' = "PREPARED"
        /\ preparedAmount' = amount
        /\ UNCHANGED
            << oldPhase, oldCapacity, newCapacity, oldOutstanding,
               oldPending, newOutstanding, oldSequence, terminalSequence,
               reconciledOutstanding, reconciled, fenced, commitRecorded,
               commitCount, crashed, commitTransferId, commitPayloadAmount,
               commitReceiptId, responseState, retryTransferId,
               retryPayloadAmount, retryReceiptId, retryResult,
               conflictingRetryAttempted, conflictAccepted >>

AbortPreparedTransfer ==
    /\ ~crashed
    /\ oldPhase = "FENCED"
    /\ newPhase = "PREPARED"
    /\ newPhase' = "ABSENT"
    /\ preparedAmount' = 0
    /\ UNCHANGED
        << oldPhase, oldCapacity, newCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, commitRecorded,
           commitCount, crashed, commitTransferId, commitPayloadAmount,
           commitReceiptId, responseState, retryTransferId,
           retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

CommitTransfer ==
    /\ ~crashed
    /\ oldPhase = "FENCED"
    /\ newPhase = "PREPARED"
    /\ ~commitRecorded
    /\ preparedAmount > 0
    /\ oldCapacity' = oldCapacity - preparedAmount
    /\ newCapacity' = newCapacity + preparedAmount
    /\ oldPhase' = "RETIRED"
    /\ newPhase' = "ACTIVE"
    /\ commitRecorded' = TRUE
    /\ commitCount' = commitCount + 1
    /\ commitTransferId' = "transfer-1"
    /\ commitPayloadAmount' = preparedAmount
    /\ commitReceiptId' = "receipt-1"
    /\ responseState' = "UNOBSERVED"
    /\ retryTransferId' = "none"
    /\ retryPayloadAmount' = 0
    /\ retryReceiptId' = "none"
    /\ retryResult' = "NONE"
    /\ conflictingRetryAttempted' = FALSE
    /\ conflictAccepted' = FALSE
    /\ UNCHANGED
        << oldOutstanding, oldPending, newOutstanding, oldSequence,
           terminalSequence, reconciledOutstanding, reconciled, fenced,
           preparedAmount, crashed >>

LoseCommitResponse ==
    /\ ~crashed
    /\ commitRecorded
    /\ responseState = "UNOBSERVED"
    /\ responseState' = "LOST"
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
           oldPending, newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, retryTransferId,
           retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

ObserveCommitResponse ==
    /\ ~crashed
    /\ commitRecorded
    /\ responseState = "UNOBSERVED"
    /\ responseState' = "ACKED"
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
           oldPending, newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, retryTransferId,
           retryPayloadAmount, retryReceiptId, retryResult,
           conflictingRetryAttempted, conflictAccepted >>

RetryCommittedTransfer ==
    /\ ~crashed
    /\ commitRecorded
    /\ responseState \in {"LOST", "ACKED"}
    /\ retryTransferId' = commitTransferId
    /\ retryPayloadAmount' = commitPayloadAmount
    /\ retryReceiptId' = commitReceiptId
    /\ retryResult' = "IDEMPOTENT"
    /\ responseState' = "ACKED"
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
           oldPending, newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, crashed, commitTransferId,
           commitPayloadAmount, commitReceiptId, conflictingRetryAttempted,
           conflictAccepted >>

RejectConflictingRetry ==
    \E alternateAmount \in 1..Capacity:
        /\ ~crashed
        /\ commitRecorded
        /\ responseState \in {"LOST", "ACKED"}
        /\ alternateAmount # commitPayloadAmount
        /\ retryTransferId' = commitTransferId
        /\ retryPayloadAmount' = alternateAmount
        /\ retryReceiptId' = "none"
        /\ retryResult' = "CONFLICT_REJECTED"
        /\ conflictingRetryAttempted' = TRUE
        /\ conflictAccepted' = FALSE
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
               oldPending, newOutstanding, oldSequence, terminalSequence,
               reconciledOutstanding, reconciled, fenced, preparedAmount,
               commitRecorded, commitCount, crashed, commitTransferId,
               commitPayloadAmount, commitReceiptId, responseState >>

NewSpend ==
    \E amount \in 1..Capacity:
        /\ ~crashed
        /\ newPhase = "ACTIVE"
        /\ commitRecorded
        /\ newOutstanding + amount <= newCapacity
        /\ newOutstanding' = newOutstanding + amount
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity,
               oldOutstanding, oldPending, oldSequence, terminalSequence,
               reconciledOutstanding, reconciled, fenced, preparedAmount,
               commitRecorded, commitCount, crashed, commitTransferId,
               commitPayloadAmount, commitReceiptId, responseState,
               retryTransferId, retryPayloadAmount, retryReceiptId, retryResult,
               conflictingRetryAttempted, conflictAccepted >>

Crash ==
    /\ ~crashed
    /\ crashed' = TRUE
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
           oldPending, newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, commitTransferId, commitPayloadAmount,
           commitReceiptId, responseState, retryTransferId, retryPayloadAmount,
           retryReceiptId, retryResult, conflictingRetryAttempted,
           conflictAccepted >>

Restart ==
    /\ crashed
    /\ crashed' = FALSE
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
           oldPending, newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, commitCount, commitTransferId, commitPayloadAmount,
           commitReceiptId, responseState, retryTransferId, retryPayloadAmount,
           retryReceiptId, retryResult, conflictingRetryAttempted,
           conflictAccepted >>

Next ==
    \/ OldSpend
    \/ CreatePending
    \/ ResolveAllPending
    \/ BeginDrain
    \/ OldSettlement
    \/ Reconcile
    \/ Fence
    \/ PrepareTransfer
    \/ AbortPreparedTransfer
    \/ CommitTransfer
    \/ LoseCommitResponse
    \/ ObserveCommitResponse
    \/ RetryCommittedTransfer
    \/ RejectConflictingRetry
    \/ NewSpend
    \/ Crash
    \/ Restart

TypeOK ==
    /\ oldPhase \in OldPhases
    /\ newPhase \in NewPhases
    /\ oldCapacity \in 0..Capacity
    /\ newCapacity \in 0..Capacity
    /\ oldOutstanding \in 0..Capacity
    /\ oldPending \in 0..Capacity
    /\ newOutstanding \in 0..Capacity
    /\ oldSequence \in 0..MaxOps
    /\ terminalSequence \in {-1} \cup (0..MaxOps)
    /\ reconciledOutstanding \in 0..Capacity
    /\ reconciled \in BOOLEAN
    /\ fenced \in BOOLEAN
    /\ preparedAmount \in 0..Capacity
    /\ commitRecorded \in BOOLEAN
    /\ commitCount \in 0..2
    /\ crashed \in BOOLEAN
    /\ commitTransferId \in {"none", "transfer-1"}
    /\ commitPayloadAmount \in 0..Capacity
    /\ commitReceiptId \in {"none", "receipt-1"}
    /\ responseState \in ResponseStates
    /\ retryTransferId \in {"none", "transfer-1"}
    /\ retryPayloadAmount \in 0..Capacity
    /\ retryReceiptId \in {"none", "receipt-1"}
    /\ retryResult \in RetryResults
    /\ conflictingRetryAttempted \in BOOLEAN
    /\ conflictAccepted \in BOOLEAN

CapacityConserved ==
    oldCapacity + newCapacity = Capacity

OldLiabilityBound ==
    oldOutstanding + oldPending <= oldCapacity

NewLiabilityBound ==
    newOutstanding <= newCapacity

AggregateOutstandingBound ==
    oldOutstanding + oldPending + newOutstanding <= Capacity

NoSuccessorSpendBeforeCommit ==
    newOutstanding > 0 => newPhase = "ACTIVE" /\ commitRecorded

NoDoubleCommit ==
    commitCount <= 1

FenceBeforeSuccessorActivation ==
    newPhase \in {"PREPARED", "ACTIVE"} => fenced

ReconciliationCutStable ==
    reconciled => oldSequence = terminalSequence

PreparedTransferWithinVerifiedRemainder ==
    newPhase = "PREPARED" =>
        preparedAmount <= oldCapacity - reconciledOutstanding

CommitReceiptBound ==
    commitRecorded =>
        /\ commitTransferId = "transfer-1"
        /\ commitPayloadAmount > 0
        /\ commitReceiptId = "receipt-1"

IdempotentRetryReturnsOriginalReceipt ==
    retryResult = "IDEMPOTENT" =>
        /\ retryReceiptId = commitReceiptId
        /\ retryTransferId = commitTransferId
        /\ retryPayloadAmount = commitPayloadAmount
        /\ commitCount = 1

ConflictingRetryDoesNotReturnReceipt ==
    retryResult = "CONFLICT_REJECTED" =>
        /\ retryReceiptId = "none"
        /\ retryTransferId = commitTransferId
        /\ retryPayloadAmount # commitPayloadAmount
        /\ ~conflictAccepted

NoConflictingReplayAccepted ==
    ~conflictAccepted

Spec == Init /\ [][Next]_vars

====
