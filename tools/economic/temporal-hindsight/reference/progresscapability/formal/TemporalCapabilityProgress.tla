---------------- MODULE TemporalCapabilityProgress ----------------
EXTENDS Naturals, FiniteSets, TLC

Caps == {"cap:1"}

VARIABLES active, unknownOutcome, progressClosed
vars == <<active, unknownOutcome, progressClosed>>

Init ==
    /\ active = Caps
    /\ unknownOutcome = FALSE
    /\ progressClosed = FALSE

PrepareUnknownRelease ==
    /\ active = Caps
    /\ ~unknownOutcome
    /\ ~progressClosed
    /\ unknownOutcome' = TRUE
    /\ UNCHANGED <<active, progressClosed>>

ReconcileApplied ==
    /\ unknownOutcome
    /\ unknownOutcome' = FALSE
    /\ active' = {}
    /\ UNCHANGED progressClosed

ReconcileNotApplied ==
    /\ unknownOutcome
    /\ unknownOutcome' = FALSE
    /\ UNCHANGED <<active, progressClosed>>

ReleaseKnown ==
    /\ ~unknownOutcome
    /\ active = Caps
    /\ active' = {}
    /\ UNCHANGED <<unknownOutcome, progressClosed>>

DeclareProgressClosed ==
    /\ ~unknownOutcome
    /\ active = {}
    /\ ~progressClosed
    /\ progressClosed' = TRUE
    /\ UNCHANGED <<active, unknownOutcome>>

KeepClosed ==
    /\ progressClosed
    /\ UNCHANGED vars

Next ==
    \/ PrepareUnknownRelease
    \/ ReconcileApplied
    \/ ReconcileNotApplied
    \/ ReleaseKnown
    \/ DeclareProgressClosed
    \/ KeepClosed

(* Faulty finalizer ignores both active capabilities and unresolved outcomes. *)
MutantDeclareProgressClosed ==
    /\ ~progressClosed
    /\ progressClosed' = TRUE
    /\ UNCHANGED <<active, unknownOutcome>>

MutantNext == Next \/ MutantDeclareProgressClosed

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ active \subseteq Caps
    /\ unknownOutcome \in BOOLEAN
    /\ progressClosed \in BOOLEAN

PendingRetainsCapability ==
    unknownOutcome => active = Caps

NoMutationCompletionWhileUnknown ==
    unknownOutcome => ~progressClosed

NoPrematureProgressClosure ==
    progressClosed => active = {}

=============================================================================
