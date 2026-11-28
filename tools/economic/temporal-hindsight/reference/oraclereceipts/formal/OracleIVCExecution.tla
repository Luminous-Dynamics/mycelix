---------------- MODULE OracleIVCExecution ----------------
EXTENDS Naturals, FiniteSets, TLC

MaxSteps == 2
Counts == 0..MaxSteps

VARIABLES
    stepCount,
    stateHead,
    proofHead,
    adviceHead,
    receiptFrontier,
    receiptHighWater,
    pending,
    pendingBaseState,
    pendingBaseProof,
    pendingBaseAdvice,
    pendingBaseFrontier,
    pendingTargetState,
    pendingTargetProof,
    pendingTargetAdvice,
    pendingTargetFrontier

vars ==
    << stepCount, stateHead, proofHead, receiptFrontier, receiptHighWater,
       pending, pendingBaseState, pendingBaseProof, pendingBaseFrontier,
       pendingTargetState, pendingTargetProof, pendingTargetFrontier >>

Init ==
    /\ stepCount = 0
    /\ stateHead = 0
    /\ proofHead = 0
    /\ adviceHead = 0
    /\ receiptFrontier = 0
    /\ receiptHighWater = 0
    /\ pending = FALSE
    /\ pendingBaseState = 0
    /\ pendingBaseProof = 0
    /\ pendingBaseAdvice = 0
    /\ pendingBaseFrontier = 0
    /\ pendingTargetState = 0
    /\ pendingTargetProof = 0
    /\ pendingTargetAdvice = 0
    /\ pendingTargetFrontier = 0

ApplyKnownStep ==
    /\ ~pending
    /\ stepCount < MaxSteps
    /\ receiptFrontier < MaxSteps
    /\ stepCount' = stepCount + 1
    /\ stateHead' = stateHead + 1
    /\ proofHead' = proofHead + 1
    /\ adviceHead' = adviceHead + 1
    /\ receiptFrontier' = receiptFrontier + 1
    /\ receiptHighWater' = receiptFrontier + 1
    /\ UNCHANGED << pending, pendingBaseState, pendingBaseProof,
                    pendingBaseFrontier, pendingTargetState,
                    pendingTargetProof, pendingTargetAdvice, pendingTargetFrontier >>

PrepareUnknownStep ==
    /\ ~pending
    /\ stepCount < MaxSteps
    /\ receiptFrontier < MaxSteps
    /\ pending' = TRUE
    /\ pendingBaseState' = stateHead
    /\ pendingBaseProof' = proofHead
    /\ pendingBaseAdvice' = adviceHead
    /\ pendingBaseFrontier' = receiptFrontier
    /\ pendingTargetState' = stateHead + 1
    /\ pendingTargetProof' = proofHead + 1
    /\ pendingTargetAdvice' = adviceHead + 1
    /\ pendingTargetFrontier' = receiptFrontier + 1
    /\ UNCHANGED << stepCount, stateHead, proofHead,
                    receiptFrontier, receiptHighWater >>

ReconcileApplied ==
    /\ pending
    /\ stepCount' = stepCount + 1
    /\ stateHead' = pendingTargetState
    /\ proofHead' = pendingTargetProof
    /\ adviceHead' = pendingTargetAdvice
    /\ receiptFrontier' = pendingTargetFrontier
    /\ receiptHighWater' = pendingTargetFrontier
    /\ pending' = FALSE
    /\ UNCHANGED << pendingBaseState, pendingBaseProof,
                    pendingBaseFrontier, pendingTargetState,
                    pendingTargetProof, pendingTargetAdvice, pendingTargetFrontier >>

ReconcileNotApplied ==
    /\ pending
    /\ pending' = FALSE
    /\ UNCHANGED << stepCount, stateHead, proofHead,
                    receiptFrontier, receiptHighWater,
                    pendingBaseState, pendingBaseProof,
                    pendingBaseFrontier, pendingTargetState,
                    pendingTargetProof, pendingTargetFrontier >>

KeepState ==
    /\ stepCount = MaxSteps
    /\ UNCHANGED vars

Next ==
    \/ ApplyKnownStep
    \/ PrepareUnknownStep
    \/ ReconcileApplied
    \/ ReconcileNotApplied
    \/ KeepState

(* Mutant 1: a later execution step rewinds the receipt prefix. *)
MutantRegressFrontier ==
    /\ ~pending
    /\ receiptHighWater > 0
    /\ receiptFrontier' = 0
    /\ UNCHANGED << stepCount, stateHead, proofHead, receiptHighWater,
                    pending, pendingBaseState, pendingBaseProof,
                    pendingBaseFrontier, pendingTargetState,
                    pendingTargetProof, pendingTargetFrontier >>

(* Mutant 2: update the proof head without updating the corresponding state. *)
MutantProofOnly ==
    /\ ~pending
    /\ proofHead < MaxSteps
    /\ proofHead' = proofHead + 1
    /\ UNCHANGED << stepCount, stateHead, receiptFrontier,
                    receiptHighWater, pending, pendingBaseState,
                    pendingBaseProof, pendingBaseFrontier,
                    pendingTargetState, pendingTargetProof,
                    pendingTargetFrontier >>

(* Mutant 3: apply a step while the prior outcome is still Unknown. *)
MutantApplyWhilePending ==
    /\ pending
    /\ stepCount < MaxSteps
    /\ stepCount' = stepCount + 1
    /\ stateHead' = stateHead + 1
    /\ proofHead' = proofHead + 1
    /\ receiptFrontier' = receiptFrontier + 1
    /\ receiptHighWater' = receiptFrontier + 1
    /\ UNCHANGED << pending, pendingBaseState, pendingBaseProof,
                    pendingBaseFrontier, pendingTargetState,
                    pendingTargetProof, pendingTargetFrontier >>

MutantRegressionSpec == Init /\ [][Next \/ MutantRegressFrontier]_vars
MutantProofOnlySpec == Init /\ [][Next \/ MutantProofOnly]_vars
MutantPendingMutationSpec == Init /\ [][Next \/ MutantApplyWhilePending]_vars

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ stepCount \in Counts
    /\ stateHead \in Counts
    /\ proofHead \in Counts
    /\ adviceHead \in Counts
    /\ receiptFrontier \in Counts
    /\ receiptHighWater \in Counts
    /\ pending \in BOOLEAN
    /\ pendingBaseState \in Counts
    /\ pendingBaseProof \in Counts
    /\ pendingBaseAdvice \in Counts
    /\ pendingBaseFrontier \in Counts
    /\ pendingTargetState \in Counts
    /\ pendingTargetProof \in Counts
    /\ pendingTargetAdvice \in Counts
    /\ pendingTargetFrontier \in Counts

StateProofHeadsConsistent ==
    /\ stateHead = stepCount
    /\ proofHead = stepCount
    /\ adviceHead = stepCount

ReceiptFrontierMonotone ==
    receiptFrontier >= receiptHighWater

UnknownOutcomeFreezesHeads ==
    pending =>
        /\ stateHead = pendingBaseState
        /\ proofHead = pendingBaseProof
        /\ adviceHead = pendingBaseAdvice
        /\ receiptFrontier = pendingBaseFrontier

PendingTargetIsNext ==
    pending =>
        /\ pendingTargetState = stateHead + 1
        /\ pendingTargetProof = proofHead + 1
        /\ pendingTargetAdvice = adviceHead + 1
        /\ pendingTargetFrontier = receiptFrontier + 1

=============================================================================
