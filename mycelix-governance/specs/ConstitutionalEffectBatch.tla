------------------------- MODULE ConstitutionalEffectBatch -------------------------
EXTENDS Naturals, FiniteSets

CONSTANT MaxStep

Actions == {"A0", "A1", "A2"}
NoAction == "None"

(* Representative capabilities used to exercise the protocol contract.
   These are NOT the runtime profile. The runtime profile remains conservative. *)
ReplaySafe == {"A0"}
Queryable == {"A0", "A1"}

ActionStates == {
    "Absent",
    "EffectPending",
    "InFlight",
    "UnknownOutcome",
    "ObservedSuccess",
    "ObservedNoEffectFailure"
}

OperationPhases == {
    "Uncommitted",
    "EffectPending",
    "Executing",
    "UnknownOutcome",
    "PartiallyCompleted",
    "FailedNoEffect",
    "Completed",
    "IntegrityHalted"
}

StepValues == 0..MaxStep

Index(a) ==
    CASE a = "A0" -> 0
      [] a = "A1" -> 1
      [] a = "A2" -> 2

PriorSucceeded(a, s) ==
    \A b \in Actions : Index(b) < Index(a) => s[b] = "ObservedSuccess"

AllSucceeded(s) ==
    \A a \in Actions : s[a] = "ObservedSuccess"

AnyUnknown(s) ==
    \E a \in Actions : s[a] = "UnknownOutcome"

AnyActionInFlight(s) ==
    \E a \in Actions : s[a] = "InFlight"

VARIABLES step, opPhase, opCommitted, outbox, actionState, inFlight,
          requestAttempts, forwardEffects, noEffectProof,
          failureSeen, integrityHalt, crashCount

vars == <<step, opPhase, opCommitted, outbox, actionState, inFlight,
          requestAttempts, forwardEffects, noEffectProof,
          failureSeen, integrityHalt, crashCount>>

Init ==
    /\ step = 0
    /\ opPhase = "Uncommitted"
    /\ opCommitted = FALSE
    /\ outbox = {}
    /\ actionState = [a \in Actions |-> "Absent"]
    /\ inFlight = NoAction
    /\ requestAttempts = [a \in Actions |-> 0]
    /\ forwardEffects = {}
    /\ noEffectProof = {}
    /\ failureSeen = FALSE
    /\ integrityHalt = FALSE
    /\ crashCount = 0

Advanceable == step < MaxStep
NextStep == step + 1

CommitBatch ==
    /\ Advanceable
    /\ ~opCommitted
    /\ ~integrityHalt
    /\ opCommitted' = TRUE
    /\ outbox' = Actions
    /\ actionState' = [a \in Actions |-> "EffectPending"]
    /\ opPhase' = "EffectPending"
    /\ step' = NextStep
    /\ UNCHANGED <<inFlight, requestAttempts, forwardEffects, noEffectProof,
                   failureSeen, integrityHalt, crashCount>>

StartAction(a) ==
    /\ Advanceable
    /\ opCommitted
    /\ ~integrityHalt
    /\ ~failureSeen
    /\ inFlight = NoAction
    /\ ~AnyUnknown(actionState)
    /\ actionState[a] = "EffectPending"
    /\ PriorSucceeded(a, actionState)
    /\ actionState' = [actionState EXCEPT ![a] = "InFlight"]
    /\ inFlight' = a
    /\ requestAttempts' = [requestAttempts EXCEPT ![a] = @ + 1]
    /\ opPhase' = "Executing"
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, forwardEffects, noEffectProof,
                   failureSeen, integrityHalt, crashCount>>

(* A logical provider effect observation. Multiple network deliveries are outside
   this abstraction unless the provider profile separately proves replay safety. *)
ExternalDeliver(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ inFlight = a
    /\ actionState[a] = "InFlight"
    /\ a \notin forwardEffects
    /\ forwardEffects' = forwardEffects \cup {a}
    /\ step' = NextStep
    /\ UNCHANGED <<opPhase, opCommitted, outbox, actionState, inFlight,
                   requestAttempts, noEffectProof, failureSeen,
                   integrityHalt, crashCount>>

ReceiveSuccessAck(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ inFlight = a
    /\ actionState[a] = "InFlight"
    /\ a \in forwardEffects
    /\ LET s2 == [actionState EXCEPT ![a] = "ObservedSuccess"]
       IN /\ actionState' = s2
          /\ opPhase' = IF AllSucceeded(s2)
                         THEN "Completed"
                         ELSE "PartiallyCompleted"
    /\ inFlight' = NoAction
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, requestAttempts, forwardEffects,
                   noEffectProof, failureSeen, integrityHalt, crashCount>>

ReceiveKnownNoEffectFailure(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ inFlight = a
    /\ actionState[a] = "InFlight"
    /\ a \notin forwardEffects
    /\ actionState' = [actionState EXCEPT ![a] = "ObservedNoEffectFailure"]
    /\ inFlight' = NoAction
    /\ failureSeen' = TRUE
    /\ opPhase' = IF forwardEffects = {}
                   THEN "FailedNoEffect"
                   ELSE "PartiallyCompleted"
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, requestAttempts, forwardEffects,
                   noEffectProof, integrityHalt, crashCount>>

LoseOrTimeoutAck(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ inFlight = a
    /\ actionState[a] = "InFlight"
    /\ actionState' = [actionState EXCEPT ![a] = "UnknownOutcome"]
    /\ inFlight' = NoAction
    /\ opPhase' = "UnknownOutcome"
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, requestAttempts, forwardEffects,
                   noEffectProof, failureSeen, integrityHalt, crashCount>>

ReconcileSuccess(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ actionState[a] = "UnknownOutcome"
    /\ a \in forwardEffects
    /\ LET s2 == [actionState EXCEPT ![a] = "ObservedSuccess"]
       IN /\ actionState' = s2
          /\ opPhase' = IF AllSucceeded(s2)
                         THEN "Completed"
                         ELSE "PartiallyCompleted"
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, inFlight, requestAttempts,
                   forwardEffects, noEffectProof, failureSeen,
                   integrityHalt, crashCount>>

ReconcileNoEffect(a) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ actionState[a] = "UnknownOutcome"
    /\ a \notin forwardEffects
    /\ a \in Queryable
    /\ actionState' = [actionState EXCEPT ![a] = "EffectPending"]
    /\ noEffectProof' = noEffectProof \cup {a}
    /\ opPhase' = IF forwardEffects = {}
                   THEN "EffectPending"
                   ELSE "PartiallyCompleted"
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, inFlight, requestAttempts,
                   forwardEffects, failureSeen, integrityHalt, crashCount>>

Crash ==
    /\ Advanceable
    /\ IF inFlight = NoAction
          THEN /\ actionState' = actionState
               /\ opPhase' = opPhase
          ELSE /\ actionState' = [actionState EXCEPT ![inFlight] = "UnknownOutcome"]
               /\ opPhase' = "UnknownOutcome"
    /\ inFlight' = NoAction
    /\ crashCount' = crashCount + 1
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, requestAttempts, forwardEffects,
                   noEffectProof, failureSeen, integrityHalt>>

ObserveContradiction ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ opCommitted
    /\ integrityHalt' = TRUE
    /\ opPhase' = "IntegrityHalted"
    /\ inFlight' = NoAction
    /\ step' = NextStep
    /\ UNCHANGED <<opCommitted, outbox, actionState, requestAttempts,
                   forwardEffects, noEffectProof, failureSeen, crashCount>>

Next ==
       CommitBatch
    \/ (\E a \in Actions : StartAction(a))
    \/ (\E a \in Actions : ExternalDeliver(a))
    \/ (\E a \in Actions : ReceiveSuccessAck(a))
    \/ (\E a \in Actions : ReceiveKnownNoEffectFailure(a))
    \/ (\E a \in Actions : LoseOrTimeoutAck(a))
    \/ (\E a \in Actions : ReconcileSuccess(a))
    \/ (\E a \in Actions : ReconcileNoEffect(a))
    \/ Crash
    \/ ObserveContradiction

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ step \in StepValues
    /\ opPhase \in OperationPhases
    /\ opCommitted \in BOOLEAN
    /\ outbox \subseteq Actions
    /\ actionState \in [Actions -> ActionStates]
    /\ inFlight \in Actions \cup {NoAction}
    /\ requestAttempts \in [Actions -> StepValues]
    /\ forwardEffects \subseteq Actions
    /\ noEffectProof \subseteq Actions
    /\ failureSeen \in BOOLEAN
    /\ integrityHalt \in BOOLEAN
    /\ crashCount \in StepValues

IntentBeforeAnyRequest ==
    \A a \in Actions :
        requestAttempts[a] > 0 => opCommitted /\ a \in outbox

EffectRequiresCommittedIntent ==
    forwardEffects # {} => opCommitted /\ forwardEffects \subseteq outbox

EffectsRespectActionOrder ==
    \A a \in forwardEffects :
        \A b \in Actions : Index(b) < Index(a) => b \in forwardEffects

ObservedSuccessRequiresEffect ==
    \A a \in Actions :
        actionState[a] = "ObservedSuccess" => a \in forwardEffects

KnownNoEffectFailureRequiresNoEffect ==
    \A a \in Actions :
        actionState[a] = "ObservedNoEffectFailure" => a \notin forwardEffects

UnknownOutcomeBlocksParallelWork ==
    AnyUnknown(actionState) => inFlight = NoAction /\ ~AnyActionInFlight(actionState)

UnknownOutcomeBlocksLaterActions ==
    \A a \in Actions :
        actionState[a] = "UnknownOutcome" =>
            \A b \in Actions :
                Index(b) > Index(a) => actionState[b] = "EffectPending"

NonReplayRetryRequiresNoEffectProof ==
    \A a \in (Actions \ ReplaySafe) :
        requestAttempts[a] > 1 => a \in noEffectProof

CompletedMeansAllActionsObserved ==
    opPhase = "Completed" => AllSucceeded(actionState)

FailedNoEffectMeansNoPhysicalEffect ==
    opPhase = "FailedNoEffect" => forwardEffects = {}

PartialCompletionIsTruthful ==
    opPhase = "PartiallyCompleted" => forwardEffects # {}

KnownFailureStopsForwardDispatch ==
    failureSeen => inFlight = NoAction /\ ~AnyActionInFlight(actionState)

IntegrityHaltIsExplicit ==
    integrityHalt => opPhase = "IntegrityHalted"

(* Named reachability predicates for a future exact-head qualifier. *)
PartialFailureReached ==
    failureSeen /\ forwardEffects # {} /\ opPhase = "PartiallyCompleted"

UnknownWithPriorEffectReached ==
    opPhase = "UnknownOutcome" /\ forwardEffects # {}

NonReplayRetryAfterNoEffectProofReached ==
    \E a \in (Actions \ ReplaySafe) :
        requestAttempts[a] >= 2 /\ a \in noEffectProof

UnreconcilableUnknownReached ==
    actionState["A2"] = "UnknownOutcome" /\ "A2" \notin Queryable

CompletedBatchReached ==
    opPhase = "Completed"

FailedNoEffectReached ==
    opPhase = "FailedNoEffect"

CrashIntoUnknownReached ==
    crashCount > 0 /\ opPhase = "UnknownOutcome"

NeverPartialFailureReached == ~PartialFailureReached
NeverUnknownWithPriorEffectReached == ~UnknownWithPriorEffectReached
NeverNonReplayRetryAfterNoEffectProofReached == ~NonReplayRetryAfterNoEffectProofReached
NeverUnreconcilableUnknownReached == ~UnreconcilableUnknownReached
NeverCompletedBatchReached == ~CompletedBatchReached
NeverFailedNoEffectReached == ~FailedNoEffectReached
NeverCrashIntoUnknownReached == ~CrashIntoUnknownReached

=============================================================================
