---------------- MODULE CorrectionImpactCone ----------------
EXTENDS Naturals, FiniteSets, TLC

Steps == {1, 2, 3}
EarliestAffected == 2
ImpactCone == {step \in Steps : step >= EarliestAffected}

VARIABLES
    activeSteps,
    invalidatedSteps,
    correctionCommitted

vars == <<activeSteps, invalidatedSteps, correctionCommitted>>

Init ==
    /\ activeSteps = Steps
    /\ invalidatedSteps = {}
    /\ correctionCommitted = FALSE

CommitCompleteCorrection ==
    /\ ~correctionCommitted
    /\ invalidatedSteps' = ImpactCone
    /\ activeSteps' = activeSteps \ ImpactCone
    /\ correctionCommitted' = TRUE

(* Faulty implementation invalidates only the earliest affected step. *)
MutantInvalidateEarliestOnly ==
    /\ ~correctionCommitted
    /\ invalidatedSteps' = {EarliestAffected}
    /\ activeSteps' = activeSteps \ {EarliestAffected}
    /\ correctionCommitted' = TRUE

KeepState ==
    /\ correctionCommitted
    /\ UNCHANGED vars

Next == CommitCompleteCorrection \/ KeepState
MutantNext == CommitCompleteCorrection \/ MutantInvalidateEarliestOnly \/ KeepState

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ activeSteps \subseteq Steps
    /\ invalidatedSteps \subseteq Steps
    /\ correctionCommitted \in BOOLEAN

HistoryPartition ==
    activeSteps \cap invalidatedSteps = {}

NoStaleActiveDescendant ==
    correctionCommitted => activeSteps \cap ImpactCone = {}

ExactImpactConeInvalidated ==
    correctionCommitted => invalidatedSteps = ImpactCone

ImpactConeIsSuffix ==
    ImpactCone = {step \in Steps : step >= EarliestAffected}

=============================================================================
