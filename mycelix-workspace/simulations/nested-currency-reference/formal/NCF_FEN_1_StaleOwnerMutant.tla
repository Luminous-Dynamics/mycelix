---------------- MODULE NCF_FEN_1_StaleOwnerMutant ----------------
EXTENDS NCF_FEN_1

\* Deliberately unsafe negative control: a stale old owner is allowed to create
\* new exposure after successor capacity has already been transferred.
StaleOldObligation ==
    \E amount \in 1..Capacity:
        /\ phase = "TRANSFERRED"
        /\ opSeq < MaxSteps
        /\ outstanding + amount + pending <= Capacity
        /\ outstanding' = outstanding + amount
        /\ opSeq' = opSeq + 1
        /\ oldWriteAccepted' = TRUE
        /\ UNCHANGED
            << phase, pending, evidenceComplete, recSeq,
               recOutstanding, recPending, fenceSeq,
               successorCapacity, successorOutstanding,
               rejectedStaleAttempts >>

StaleOwnerMutantNext == Next \/ StaleOldObligation
StaleOwnerMutantSpec == Init /\ [][StaleOwnerMutantNext]_vars

====
