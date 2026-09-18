---------------------- MODULE ConstitutionalLifecycleQuiescence ----------------------
EXTENDS Naturals, FiniteSets

CONSTANT MaxSeq

Claims == {"A", "B"}
NoClaim == "None"
Statuses == {
    "Absent",
    "Pending",
    "Blocked",
    "Finalized",
    "Rejected",
    "RevokedClosed",
    "Halted"
}
LiveStatuses == {"Pending", "Blocked"}
TerminalStatuses == {"Finalized", "Rejected", "RevokedClosed", "Halted"}
QuiescenceClasses == {
    "HorizonReached",
    "IntegrityHalt",
    "ResolvedQuiescence",
    "AwaitingExternalEvidence",
    "ActiveResolution",
    "ProtocolStall"
}
ClockValues == 0..MaxSeq

VARIABLES clock, status, winner, revoked, fault, effect, terminalSeen, workSeen,
          finalityReady, revocationReady, closureReady, effectReady, faultReady

vars == <<clock, status, winner, revoked, fault, effect, terminalSeen, workSeen,
          finalityReady, revocationReady, closureReady, effectReady, faultReady>>

Init ==
    /\ clock = 0
    /\ status = [c \in Claims |-> "Absent"]
    /\ winner = NoClaim
    /\ revoked = FALSE
    /\ fault = FALSE
    /\ effect = NoClaim
    /\ terminalSeen = {}
    /\ workSeen = FALSE
    /\ finalityReady = [c \in Claims |-> FALSE]
    /\ revocationReady = FALSE
    /\ closureReady = FALSE
    /\ effectReady = FALSE
    /\ faultReady = FALSE

Advanceable == clock < MaxSeq
NextSeq == clock + 1
LiveClaims == {c \in Claims : status[c] \in LiveStatuses}
TerminalNow == {c \in Claims : status[c] \in TerminalStatuses}
PendingEffect == winner # NoClaim /\ effect = NoClaim
UnresolvedWork == LiveClaims # {} \/ PendingEffect

(***************************************************************************
External/environment events. These can make resolution possible, but they are
not themselves constitutional resolution progress.
***************************************************************************)
Submit(c) ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ status[c] = "Absent"
    /\ status' = [status EXCEPT ![c] = IF revoked THEN "Blocked" ELSE "Pending"]
    /\ workSeen' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, revoked, fault, effect, terminalSeen,
                   finalityReady, revocationReady, closureReady, effectReady, faultReady>>

OfferFinality(c) ==
    /\ Advanceable
    /\ ~fault
    /\ status[c] \in LiveStatuses
    /\ ~finalityReady[c]
    /\ finalityReady' = [finalityReady EXCEPT ![c] = TRUE]
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, effect, terminalSeen, workSeen,
                   revocationReady, closureReady, effectReady, faultReady>>

OfferRevocation ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ ~revoked
    /\ ~revocationReady
    /\ \E c \in Claims : status[c] = "Pending"
    /\ revocationReady' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, effect, terminalSeen, workSeen,
                   finalityReady, closureReady, effectReady, faultReady>>

OfferClosure ==
    /\ Advanceable
    /\ ~fault
    /\ revoked
    /\ ~closureReady
    /\ \E c \in Claims : status[c] = "Blocked"
    /\ closureReady' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, effect, terminalSeen, workSeen,
                   finalityReady, revocationReady, effectReady, faultReady>>

OfferEffectExecutor ==
    /\ Advanceable
    /\ ~fault
    /\ PendingEffect
    /\ ~effectReady
    /\ effectReady' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, effect, terminalSeen, workSeen,
                   finalityReady, revocationReady, closureReady, faultReady>>

OfferFaultEvidence ==
    /\ Advanceable
    /\ ~fault
    /\ UnresolvedWork
    /\ ~faultReady
    /\ faultReady' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, effect, terminalSeen, workSeen,
                   finalityReady, revocationReady, closureReady, effectReady>>

ExternalInputStep ==
       (\E c \in Claims : Submit(c))
    \/ (\E c \in Claims : OfferFinality(c))
    \/ OfferRevocation
    \/ OfferClosure
    \/ OfferEffectExecutor
    \/ OfferFaultEvidence

(***************************************************************************
Internal constitutional resolution. Preconditions are intentionally mirrored
by ResolutionObligation below, but that predicate is written independently so
mutation tests can remove an action and expose ProtocolStall.
***************************************************************************)
ResolveFinality(c) ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ status[c] \in LiveStatuses
    /\ finalityReady[c]
    /\ status' = [x \in Claims |->
          IF x = c THEN "Finalized"
          ELSE IF status[x] \in LiveStatuses THEN "Rejected"
          ELSE status[x]]
    /\ winner' = c
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<revoked, fault, effect, workSeen, finalityReady,
                   revocationReady, closureReady, effectReady, faultReady>>

ResolveRevocation ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ revocationReady
    /\ \E c \in Claims : status[c] = "Pending"
    /\ status' = [x \in Claims |->
          IF status[x] = "Pending" THEN "Blocked" ELSE status[x]]
    /\ revoked' = TRUE
    /\ revocationReady' = FALSE
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, fault, effect, terminalSeen, workSeen, finalityReady,
                   closureReady, effectReady, faultReady>>

ResolveClosure ==
    /\ Advanceable
    /\ ~fault
    /\ revoked
    /\ closureReady
    /\ \E c \in Claims : status[c] = "Blocked"
    /\ status' = [x \in Claims |->
          IF status[x] = "Blocked" THEN "RevokedClosed" ELSE status[x]]
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, revoked, fault, effect, workSeen, finalityReady,
                   revocationReady, closureReady, effectReady, faultReady>>

ResolveFault ==
    /\ Advanceable
    /\ ~fault
    /\ faultReady
    /\ UnresolvedWork
    /\ fault' = TRUE
    /\ status' = [x \in Claims |->
          IF status[x] \in LiveStatuses THEN "Halted" ELSE status[x]]
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, revoked, effect, workSeen, finalityReady,
                   revocationReady, closureReady, effectReady, faultReady>>

ApplyEffect ==
    /\ Advanceable
    /\ ~fault
    /\ PendingEffect
    /\ effectReady
    /\ effect' = winner
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, revoked, fault, terminalSeen, workSeen,
                   finalityReady, revocationReady, closureReady, effectReady, faultReady>>

InternalResolutionStep ==
       (\E c \in Claims : ResolveFinality(c))
    \/ ResolveRevocation
    \/ ResolveClosure
    \/ ResolveFault
    \/ ApplyEffect

FinalityResolutionRequired ==
    winner = NoClaim /\ \E c \in Claims : status[c] \in LiveStatuses /\ finalityReady[c]

RevocationResolutionRequired ==
    winner = NoClaim /\ revocationReady /\ \E c \in Claims : status[c] = "Pending"

ClosureResolutionRequired ==
    revoked /\ closureReady /\ \E c \in Claims : status[c] = "Blocked"

FaultResolutionRequired == faultReady /\ UnresolvedWork
EffectResolutionRequired == PendingEffect /\ effectReady

ResolutionObligation ==
    /\ Advanceable
    /\ ~fault
    /\ (FinalityResolutionRequired
        \/ RevocationResolutionRequired
        \/ ClosureResolutionRequired
        \/ FaultResolutionRequired
        \/ EffectResolutionRequired)

InternalStepEnabled == ENABLED InternalResolutionStep

ProtocolStall ==
    /\ Advanceable
    /\ ~fault
    /\ UnresolvedWork
    /\ ResolutionObligation
    /\ ~InternalStepEnabled

AwaitingExternalEvidence ==
    /\ Advanceable
    /\ ~fault
    /\ UnresolvedWork
    /\ ~ResolutionObligation

ActiveResolution ==
    /\ Advanceable
    /\ ~fault
    /\ UnresolvedWork
    /\ ResolutionObligation
    /\ InternalStepEnabled

ResolvedQuiescence == ~fault /\ ~UnresolvedWork

StateClass ==
    IF ~Advanceable THEN "HorizonReached"
    ELSE IF fault THEN "IntegrityHalt"
    ELSE IF ResolvedQuiescence THEN "ResolvedQuiescence"
    ELSE IF ProtocolStall THEN "ProtocolStall"
    ELSE IF AwaitingExternalEvidence THEN "AwaitingExternalEvidence"
    ELSE "ActiveResolution"

Next == ExternalInputStep \/ InternalResolutionStep
Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ clock \in ClockValues
    /\ status \in [Claims -> Statuses]
    /\ winner \in Claims \cup {NoClaim}
    /\ revoked \in BOOLEAN
    /\ fault \in BOOLEAN
    /\ effect \in Claims \cup {NoClaim}
    /\ terminalSeen \subseteq Claims
    /\ workSeen \in BOOLEAN
    /\ finalityReady \in [Claims -> BOOLEAN]
    /\ revocationReady \in BOOLEAN
    /\ closureReady \in BOOLEAN
    /\ effectReady \in BOOLEAN
    /\ faultReady \in BOOLEAN

AtMostOneFinalized == Cardinality({c \in Claims : status[c] = "Finalized"}) <= 1

WinnerConsistent ==
    /\ (winner = NoClaim) = ({c \in Claims : status[c] = "Finalized"} = {})
    /\ winner # NoClaim => status[winner] = "Finalized"

CompetitorsResolved ==
    winner # NoClaim =>
        \A c \in Claims \ {winner} : status[c] \notin LiveStatuses \cup {"Finalized"}

BlockedRequiresRevocation ==
    (\E c \in Claims : status[c] = "Blocked") => revoked

FaultHaltsLiveWork == fault => LiveClaims = {}

HaltedRequiresFault ==
    (\E c \in Claims : status[c] = "Halted") => fault

EffectRequiresFinalized == effect # NoClaim => status[effect] = "Finalized"
TerminalStatusesMonotonic == terminalSeen \subseteq TerminalNow
ClassificationTotal == StateClass \in QuiescenceClasses
NoPrematureProtocolQuiescence == ~ProtocolStall
AwaitingExternalHasInputEnabled == AwaitingExternalEvidence => ENABLED ExternalInputStep

(***************************************************************************
Named non-vacuity predicates. Qualifiers check their negations as expected
violations; they do not conflate reachability with safety.
***************************************************************************)
AwaitingExternalEvidenceReached == workSeen /\ AwaitingExternalEvidence
ActiveResolutionReached == workSeen /\ ActiveResolution
ResolvedAfterWorkReached == workSeen /\ ResolvedQuiescence
IntegrityHaltReached == workSeen /\ fault
ClosureResolutionReached == \E c \in Claims : status[c] = "RevokedClosed"
EffectResolutionReached == effect # NoClaim

NeverAwaitingExternalEvidenceReached == ~AwaitingExternalEvidenceReached
NeverActiveResolutionReached == ~ActiveResolutionReached
NeverResolvedAfterWorkReached == ~ResolvedAfterWorkReached
NeverIntegrityHaltReached == ~IntegrityHaltReached
NeverClosureResolutionReached == ~ClosureResolutionReached
NeverEffectResolutionReached == ~EffectResolutionReached

=============================================================================
