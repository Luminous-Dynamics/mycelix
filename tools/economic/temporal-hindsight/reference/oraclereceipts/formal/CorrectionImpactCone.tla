---------------- MODULE CorrectionImpactCone ----------------
EXTENDS Naturals, FiniteSets, TLC

Steps == {1, 2, 3}
EarliestAffected == 2
ImpactCone == {step \in Steps : step >= EarliestAffected}

VARIABLES
    activeSteps,
    invalidatedSteps,
    correctionCommitted,
    currentState,
    preImpactState,
    correctionInputState,
    correctionOutputState

vars == <<activeSteps, invalidatedSteps, correctionCommitted,
          currentState, preImpactState, correctionInputState,
          correctionOutputState>>

Init ==
    /\ activeSteps = Steps
    /\ invalidatedSteps = {}
    /\ correctionCommitted = FALSE
    /\ currentState = 3
    /\ preImpactState = 1
    /\ correctionInputState = 0
    /\ correctionOutputState = 4

CommitCompleteCorrection ==
    /\ ~correctionCommitted
    /\ invalidatedSteps' = ImpactCone
    /\ activeSteps' = activeSteps \ ImpactCone
    /\ correctionInputState' = preImpactState
    /\ currentState' = correctionOutputState
    /\ correctionCommitted' = TRUE
    /\ UNCHANGED <<preImpactState, correctionOutputState>>

(* Faulty implementation invalidates only the earliest affected step. *)
MutantInvalidateEarliestOnly ==
    /\ ~correctionCommitted
    /\ invalidatedSteps' = {EarliestAffected}
    /\ activeSteps' = activeSteps \ {EarliestAffected}
    /\ correctionInputState' = preImpactState
    /\ currentState' = correctionOutputState
    /\ correctionCommitted' = TRUE
    /\ UNCHANGED <<preImpactState, correctionOutputState>>

(* Faulty implementation recomputes from the stale current state head. *)
MutantRecomputeFromStaleState ==
    /\ ~correctionCommitted
    /\ invalidatedSteps' = ImpactCone
    /\ activeSteps' = activeSteps \ ImpactCone
    /\ correctionInputState' = currentState
    /\ currentState' = correctionOutputState
    /\ correctionCommitted' = TRUE
    /\ UNCHANGED <<preImpactState, correctionOutputState>>

KeepState ==
    /\ correctionCommitted
    /\ UNCHANGED vars

Next == CommitCompleteCorrection \/ KeepState
MutantNext == CommitCompleteCorrection \/ MutantInvalidateEarliestOnly \/ KeepState
MutantStaleStateNext == CommitCompleteCorrection \/ MutantRecomputeFromStaleState \/ KeepState

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][MutantNext]_vars
MutantStaleStateSpec == Init /\ [][MutantStaleStateNext]_vars

TypeOK ==
    /\ activeSteps \subseteq Steps
    /\ invalidatedSteps \subseteq Steps
    /\ correctionCommitted \in BOOLEAN
    /\ currentState \in {1, 3, 4}
    /\ preImpactState \in {1}
    /\ correctionInputState \in {0, 1, 3}
    /\ correctionOutputState \in {4}

HistoryPartition ==
    activeSteps \cap invalidatedSteps = {}

NoStaleActiveDescendant ==
    correctionCommitted => activeSteps \cap ImpactCone = {}

ExactImpactConeInvalidated ==
    correctionCommitted => invalidatedSteps = ImpactCone

ImpactConeIsSuffix ==
    ImpactCone = {step \in Steps : step >= EarliestAffected}

CorrectionStartsAtPreImpactState ==
    correctionCommitted => correctionInputState = preImpactState

CurrentStateMatchesRecomputedOutput ==
    correctionCommitted => currentState = correctionOutputState

=============================================================================
