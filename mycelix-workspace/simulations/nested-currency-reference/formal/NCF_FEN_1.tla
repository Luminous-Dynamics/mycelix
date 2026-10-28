---------------- MODULE NCF_FEN_1 ----------------
EXTENDS Naturals, Integers, TLC

CONSTANTS Capacity, MaxSteps

ASSUME Capacity \in Nat \ {0}
ASSUME MaxSteps \in Nat \ {0}

Phases ==
    {"ACTIVE", "DRAIN_REQUESTED", "RECONCILED", "FENCED", "TRANSFERRED"}

VARIABLES
    phase,
    outstanding,
    pending,
    opSeq,
    evidenceComplete,
    recSeq,
    recOutstanding,
    recPending,
    fenceSeq,
    successorCapacity,
    successorOutstanding,
    oldWriteAccepted,
    rejectedStaleAttempts

vars ==
    << phase, outstanding, pending, opSeq, evidenceComplete,
       recSeq, recOutstanding, recPending, fenceSeq,
       successorCapacity, successorOutstanding,
       oldWriteAccepted, rejectedStaleAttempts >>

Init ==
    /\ phase = "ACTIVE"
    /\ outstanding = 0
    /\ pending \in 0..Capacity
    /\ opSeq = 0
    /\ evidenceComplete = FALSE
    /\ recSeq = -1
    /\ recOutstanding = 0
    /\ recPending = 0
    /\ fenceSeq = -1
    /\ successorCapacity = 0
    /\ successorOutstanding = 0
    /\ oldWriteAccepted = FALSE
    /\ rejectedStaleAttempts = 0

OldObligation ==
    \E amount \in 1..Capacity:
        /\ phase = "ACTIVE"
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

BeginDrain ==
    /\ phase = "ACTIVE"
    /\ phase' = "DRAIN_REQUESTED"
    /\ oldWriteAccepted' = FALSE
    /\ UNCHANGED
        << outstanding, pending, opSeq, evidenceComplete, recSeq,
           recOutstanding, recPending, fenceSeq, successorCapacity,
           successorOutstanding, rejectedStaleAttempts >>

OldSettlement ==
    \E amount \in 1..Capacity:
        /\ phase \in {"ACTIVE", "DRAIN_REQUESTED", "FENCED", "TRANSFERRED"}
        /\ outstanding >= amount
        /\ opSeq < MaxSteps
        /\ outstanding' = outstanding - amount
        /\ opSeq' = opSeq + 1
        /\ oldWriteAccepted' = FALSE
        /\ UNCHANGED
            << phase, pending, evidenceComplete, recSeq,
               recOutstanding, recPending, fenceSeq,
               successorCapacity, successorOutstanding,
               rejectedStaleAttempts >>

ResolveEvidence ==
    /\ phase = "DRAIN_REQUESTED"
    /\ evidenceComplete = FALSE
    /\ evidenceComplete' = TRUE
    /\ oldWriteAccepted' = FALSE
    /\ UNCHANGED
        << phase, outstanding, pending, opSeq, recSeq,
           recOutstanding, recPending, fenceSeq, successorCapacity,
           successorOutstanding, rejectedStaleAttempts >>

Reconcile ==
    /\ phase = "DRAIN_REQUESTED"
    /\ evidenceComplete = TRUE
    /\ phase' = "RECONCILED"
    /\ recSeq' = opSeq
    /\ recOutstanding' = outstanding
    /\ recPending' = pending
    /\ oldWriteAccepted' = FALSE
    /\ UNCHANGED
        << outstanding, pending, opSeq, evidenceComplete, fenceSeq,
           successorCapacity, successorOutstanding, rejectedStaleAttempts >>

Fence ==
    /\ phase = "RECONCILED"
    /\ recSeq = opSeq
    /\ phase' = "FENCED"
    /\ fenceSeq' = recSeq
    /\ oldWriteAccepted' = FALSE
    /\ UNCHANGED
        << outstanding, pending, opSeq, evidenceComplete, recSeq,
           recOutstanding, recPending, successorCapacity,
           successorOutstanding, rejectedStaleAttempts >>

Transfer ==
    \E amount \in 1..Capacity:
        /\ phase = "FENCED"
        /\ fenceSeq = recSeq
        /\ recOutstanding + recPending <= Capacity
        /\ amount <= Capacity - recOutstanding - recPending
        /\ phase' = "TRANSFERRED"
        /\ successorCapacity' = amount
        /\ oldWriteAccepted' = FALSE
        /\ UNCHANGED
            << outstanding, pending, opSeq, evidenceComplete, recSeq,
               recOutstanding, recPending, fenceSeq, successorOutstanding,
               rejectedStaleAttempts >>

SuccessorSpend ==
    \E amount \in 1..Capacity:
        /\ phase = "TRANSFERRED"
        /\ successorOutstanding + amount <= successorCapacity
        /\ outstanding + pending + successorOutstanding + amount <= Capacity
        /\ successorOutstanding' = successorOutstanding + amount
        /\ oldWriteAccepted' = FALSE
        /\ UNCHANGED
            << phase, outstanding, pending, opSeq, evidenceComplete, recSeq,
               recOutstanding, recPending, fenceSeq, successorCapacity,
               rejectedStaleAttempts >>

StaleOldAttempt ==
    /\ phase \in {"DRAIN_REQUESTED", "RECONCILED", "FENCED", "TRANSFERRED"}
    /\ rejectedStaleAttempts < MaxSteps
    /\ rejectedStaleAttempts' = rejectedStaleAttempts + 1
    /\ oldWriteAccepted' = FALSE
    /\ UNCHANGED
        << phase, outstanding, pending, opSeq, evidenceComplete, recSeq,
           recOutstanding, recPending, fenceSeq, successorCapacity,
           successorOutstanding >>

Next ==
    \/ OldObligation
    \/ BeginDrain
    \/ OldSettlement
    \/ ResolveEvidence
    \/ Reconcile
    \/ Fence
    \/ Transfer
    \/ SuccessorSpend
    \/ StaleOldAttempt

TypeOK ==
    /\ phase \in Phases
    /\ outstanding \in 0..Capacity
    /\ pending \in 0..Capacity
    /\ opSeq \in 0..MaxSteps
    /\ evidenceComplete \in BOOLEAN
    /\ recSeq \in -1..MaxSteps
    /\ recOutstanding \in 0..Capacity
    /\ recPending \in 0..Capacity
    /\ fenceSeq \in -1..MaxSteps
    /\ successorCapacity \in 0..Capacity
    /\ successorOutstanding \in 0..Capacity
    /\ oldWriteAccepted \in BOOLEAN
    /\ rejectedStaleAttempts \in 0..MaxSteps

ActualExposureBound ==
    outstanding + pending + successorOutstanding <= Capacity

RiskRightsConserved ==
    outstanding + pending + successorCapacity <= Capacity

SuccessorAllocationBound ==
    successorOutstanding <= successorCapacity

NoOldWriteOutsideActive ==
    oldWriteAccepted => phase = "ACTIVE"

NoFenceWithoutReconciliation ==
    fenceSeq # -1 =>
        /\ recSeq = fenceSeq
        /\ phase \in {"FENCED", "TRANSFERRED"}

NoTransferWithoutFence ==
    successorCapacity > 0 =>
        /\ phase = "TRANSFERRED"
        /\ fenceSeq # -1
        /\ fenceSeq = recSeq

ReconciliationIsBounded ==
    phase \in {"RECONCILED", "FENCED", "TRANSFERRED"} =>
        recOutstanding + recPending <= Capacity

NoSuccessorSpendBeforeTransfer ==
    successorOutstanding > 0 => phase = "TRANSFERRED"

Spec == Init /\ [][Next]_vars

====
