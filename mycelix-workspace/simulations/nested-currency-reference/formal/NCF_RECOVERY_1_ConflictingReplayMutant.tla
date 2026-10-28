---------------- MODULE NCF_RECOVERY_1_ConflictingReplayMutant ----------------
EXTENDS NCF_RECOVERY_1

\* Deliberately broken: the same transfer ID is accepted with a different
\* amount payload after the original commit response was lost.
UnsafeConflictingReplay ==
    \E alternateAmount \in 1..Capacity:
        /\ ~crashed
        /\ commitRecorded
        /\ responseState = "LOST"
        /\ commitTransferId = "transfer-1"
        /\ alternateAmount # commitPayloadAmount
        /\ retryTransferId' = commitTransferId
        /\ retryPayloadAmount' = alternateAmount
        /\ retryReceiptId' = commitReceiptId
        /\ retryResult' = "IDEMPOTENT"
        /\ conflictingRetryAttempted' = TRUE
        /\ conflictAccepted' = TRUE
        /\ UNCHANGED
            << oldPhase, newPhase, oldCapacity, newCapacity, oldOutstanding,
               oldPending, newOutstanding, oldSequence, terminalSequence,
               reconciledOutstanding, reconciled, fenced, preparedAmount,
               commitRecorded, commitCount, crashed, commitTransferId,
               commitPayloadAmount, commitReceiptId, responseState >>

MutantNext == Next \/ UnsafeConflictingReplay
MutantSpec == Init /\ [][MutantNext]_vars

====
