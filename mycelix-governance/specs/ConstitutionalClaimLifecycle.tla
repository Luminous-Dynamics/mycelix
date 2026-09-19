------------------------- MODULE ConstitutionalClaimLifecycle -------------------------
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
TerminalStatuses == {"Finalized", "Rejected", "RevokedClosed", "Halted"}
LiveStatuses == {"Pending", "Blocked"}
CoverageStates == {"Open", "Closed", "Fault"}
Seqs == 1..MaxSeq
ClockValues == 0..MaxSeq

VARIABLES clock, status, winner, finalityEffective, revoked, revocationSeq,
          coverage, fault, effect, terminalSeen

vars == <<clock, status, winner, finalityEffective, revoked, revocationSeq,
          coverage, fault, effect, terminalSeen>>

Init ==
    /\ clock = 0
    /\ status = [c \in Claims |-> "Absent"]
    /\ winner = NoClaim
    /\ finalityEffective = [c \in Claims |-> 0]
    /\ revoked = FALSE
    /\ revocationSeq = 0
    /\ coverage = "Open"
    /\ fault = FALSE
    /\ effect = NoClaim
    /\ terminalSeen = {}

Advanceable == clock < MaxSeq
NextSeq == clock + 1

TerminalNow == {c \in Claims : status[c] \in TerminalStatuses}

Submit(c) ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ status[c] = "Absent"
    /\ status' = [status EXCEPT ![c] = IF revoked THEN "Blocked" ELSE "Pending"]
    /\ clock' = NextSeq
    /\ terminalSeen' = terminalSeen
    /\ UNCHANGED <<winner, finalityEffective, revoked, revocationSeq,
                   coverage, fault, effect>>

Finalize(c, f) ==
    /\ Advanceable
    /\ ~fault
    /\ winner = NoClaim
    /\ status[c] \in LiveStatuses
    /\ f \in Seqs
    /\ (~revoked \/ f < revocationSeq)
    /\ (~revoked \/ coverage = "Open")
    /\ status' = [x \in Claims |->
          IF x = c THEN "Finalized"
          ELSE IF status[x] \in LiveStatuses THEN "Rejected"
          ELSE status[x]]
    /\ winner' = c
    /\ finalityEffective' = [finalityEffective EXCEPT ![c] = f]
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<revoked, revocationSeq, coverage, fault, effect>>

Revoke(r) ==
    /\ Advanceable
    /\ ~fault
    /\ r \in Seqs
    /\ IF winner # NoClaim /\ finalityEffective[winner] >= r
          THEN /\ fault' = TRUE
               /\ status' = [x \in Claims |->
                    IF status[x] \in LiveStatuses THEN "Halted" ELSE status[x]]
               /\ terminalSeen' = terminalSeen \cup
                    {x \in Claims : status'[x] \in TerminalStatuses}
               /\ revoked' = revoked
               /\ revocationSeq' = revocationSeq
          ELSE /\ fault' = fault
               /\ revoked' = TRUE
               /\ revocationSeq' = IF revocationSeq = 0 \/ r < revocationSeq THEN r ELSE revocationSeq
               /\ status' = [x \in Claims |->
                    IF status[x] = "Pending" THEN "Blocked" ELSE status[x]]
               /\ terminalSeen' = terminalSeen
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, finalityEffective, coverage, effect>>

CloseCoverage ==
    /\ Advanceable
    /\ ~fault
    /\ revoked
    /\ coverage = "Open"
    /\ coverage' = "Closed"
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, finalityEffective, revoked, revocationSeq,
                   fault, effect, terminalSeen>>

FaultCoverage ==
    /\ Advanceable
    /\ coverage = "Closed"
    /\ ~fault
    /\ coverage' = "Fault"
    /\ fault' = TRUE
    /\ status' = [x \in Claims |->
          IF status[x] \in LiveStatuses THEN "Halted" ELSE status[x]]
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, finalityEffective, revoked, revocationSeq, effect>>

ResolveClosed ==
    /\ Advanceable
    /\ ~fault
    /\ revoked
    /\ coverage = "Closed"
    /\ \E c \in Claims : status[c] = "Blocked"
    /\ status' = [x \in Claims |->
          IF status[x] = "Blocked" THEN "RevokedClosed" ELSE status[x]]
    /\ terminalSeen' = terminalSeen \cup
          {x \in Claims : status'[x] \in TerminalStatuses}
    /\ clock' = NextSeq
    /\ UNCHANGED <<winner, finalityEffective, revoked, revocationSeq,
                   coverage, fault, effect>>

ApplyEffect(c) ==
    /\ Advanceable
    /\ ~fault
    /\ status[c] = "Finalized"
    /\ effect = NoClaim
    /\ effect' = c
    /\ clock' = NextSeq
    /\ UNCHANGED <<status, winner, finalityEffective, revoked, revocationSeq,
                   coverage, fault, terminalSeen>>

Next ==
       (\E c \in Claims : Submit(c))
    \/ (\E c \in Claims : \E f \in Seqs : Finalize(c, f))
    \/ (\E r \in Seqs : Revoke(r))
    \/ CloseCoverage
    \/ FaultCoverage
    \/ ResolveClosed
    \/ (\E c \in Claims : ApplyEffect(c))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ clock \in ClockValues
    /\ status \in [Claims -> Statuses]
    /\ winner \in Claims \cup {NoClaim}
    /\ finalityEffective \in [Claims -> ClockValues]
    /\ revoked \in BOOLEAN
    /\ revocationSeq \in ClockValues
    /\ coverage \in CoverageStates
    /\ fault \in BOOLEAN
    /\ effect \in Claims \cup {NoClaim}
    /\ terminalSeen \subseteq Claims

AtMostOneFinalized == Cardinality({c \in Claims : status[c] = "Finalized"}) <= 1

WinnerConsistent ==
    /\ (winner = NoClaim) = ({c \in Claims : status[c] = "Finalized"} = {})
    /\ winner # NoClaim => status[winner] = "Finalized"

CompetitorsResolved ==
    winner # NoClaim =>
        \A c \in Claims \ {winner} : status[c] \notin LiveStatuses \cup {"Finalized"}

RejectedHasWinner ==
    \A c \in Claims : status[c] = "Rejected" => winner # NoClaim /\ c # winner

RevocationBlocksPending == revoked => \A c \in Claims : status[c] # "Pending"

BlockedRequiresRevocation ==
    (\E c \in Claims : status[c] = "Blocked") => revoked

FaultHaltsLiveWork == fault => \A c \in Claims : status[c] \notin LiveStatuses

HaltedRequiresFault ==
    (\E c \in Claims : status[c] = "Halted") => fault

RevokedClosedRequiresRevocation ==
    (\E c \in Claims : status[c] = "RevokedClosed") => revoked

RevokedClosedRequiresClosureHistory ==
    (\E c \in Claims : status[c] = "RevokedClosed") =>
        coverage \in {"Closed", "Fault"}

EffectRequiresFinalized == effect # NoClaim => status[effect] = "Finalized"

TerminalStatusesMonotonic == terminalSeen \subseteq TerminalNow

(***************************************************************************
Named reachability predicates. Qualifiers check their negations as expected
violations so safety and non-vacuity remain separate evidence classes.
***************************************************************************)
ConflictRejectedReached ==
    winner # NoClaim /\ \E c \in Claims \ {winner} : status[c] = "Rejected"

RevocationBlockedReached ==
    revoked /\ \E c \in Claims : status[c] = "Blocked"

RevokedClosedReached ==
    \E c \in Claims : status[c] = "RevokedClosed"

LatePreRevocationFinalityReached ==
    revoked /\ winner # NoClaim /\ finalityEffective[winner] < revocationSeq

FaultHaltedReached ==
    fault /\ \E c \in Claims : status[c] = "Halted"

HistoricalTerminalPreservedAfterFaultReached ==
    fault /\ \E c \in Claims : status[c] = "RevokedClosed"

NeverConflictRejectedReached == ~ConflictRejectedReached
NeverRevocationBlockedReached == ~RevocationBlockedReached
NeverRevokedClosedReached == ~RevokedClosedReached
NeverLatePreRevocationFinalityReached == ~LatePreRevocationFinalityReached
NeverFaultHaltedReached == ~FaultHaltedReached
NeverHistoricalTerminalPreservedAfterFaultReached == ~HistoricalTerminalPreservedAfterFaultReached

=============================================================================
