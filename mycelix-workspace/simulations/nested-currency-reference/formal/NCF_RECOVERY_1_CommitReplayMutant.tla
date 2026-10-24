---------------- MODULE NCF_RECOVERY_1_CommitReplayMutant ----------------
EXTENDS NCF_RECOVERY_1

\* Deliberately broken: a retry after a committed response-loss repeats the
\* successor credit without debiting the predecessor or preserving commit ID.
UnsafeReplayCommit ==
    /\ ~crashed
    /\ commitRecorded
    /\ newPhase = "ACTIVE"
    /\ commitCount = 1
    /\ preparedAmount > 0
    /\ newCapacity' = newCapacity + preparedAmount
    /\ commitCount' = commitCount + 1
    /\ UNCHANGED
        << oldPhase, newPhase, oldCapacity, oldOutstanding, oldPending,
           newOutstanding, oldSequence, terminalSequence,
           reconciledOutstanding, reconciled, fenced, preparedAmount,
           commitRecorded, crashed >>

MutantNext == Next \/ UnsafeReplayCommit
MutantSpec == Init /\ [][MutantNext]_vars

====
