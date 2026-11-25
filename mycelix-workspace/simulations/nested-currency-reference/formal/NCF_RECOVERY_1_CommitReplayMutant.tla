---------------- MODULE NCF_RECOVERY_1_CommitReplayMutant ----------------
EXTENDS NCF_RECOVERY_1

\* Deliberately broken: after a successful durable commit whose response was
\* lost, the same logical transfer is credited again. The retry must be
\* recognized as the original operation, not create new economic effects.
UnsafeReplayCommit ==
    /\ ~crashed
    /\ commitRecorded
    /\ responseState = "LOST"
    /\ newPhase = "ACTIVE"
    /\ commitCount = 1
    /\ commitTransferId = "transfer-1"
    /\ commitPayloadAmount = preparedAmount
    /\ newCapacity' = newCapacity + commitPayloadAmount
    /\ commitCount' = commitCount + 1
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, crashed, commitTransferId, commitPayloadAmount,
           commitReceiptId, responseState, retryTransferId, retryPayloadAmount,
           retryReceiptId, retryResult, conflictingRetryAttempted,
           conflictAccepted >>

MutantNext == Next \/ UnsafeReplayCommit
MutantSpec == Init /\ [][MutantNext]_vars

====
