---------------- MODULE NCF_FEN_1_TimeoutMutant ----------------
EXTENDS NCF_FEN_1

\* Deliberately unsafe negative control: a timeout path reassigns capacity while
\* the old owner is merely draining, without complete reconciliation or a fence.
TimeoutReallocate ==
    \E amount \in 1..Capacity:
        /\ phase = "DRAIN_REQUESTED"
        /\ amount <= Capacity - outstanding - pending
        /\ phase' = "TRANSFERRED"
        /\ successorCapacity' = amount
        /\ oldWriteAccepted' = FALSE
        /\ UNCHANGED
            << outstanding, pending, opSeq, evidenceComplete, recSeq,
               recOutstanding, recPending, fenceSeq, successorOutstanding,
               rejectedStaleAttempts >>

MutantNext == Next \/ TimeoutReallocate
MutantSpec == Init /\ [][MutantNext]_vars

====
