-------------------- MODULE ArtificialSovereigntyRoleConcentrationV2 --------------------
EXTENDS Naturals, FiniteSets

CONSTANTS S1, S2,
          Operator, Verifier, EvidenceArchive, Adjudicator,
          MaxTime

Subjects == {S1, S2}
Roles == {Operator, Verifier, EvidenceArchive, Adjudicator}
Times == 0..MaxTime

ASSUME MaxTime in Nat {0}

VARIABLES roleHolder,
          conflictFinding,
          activeReviewers,
          reviewHistory,
          clock

vars == <<roleHolder, conflictFinding, activeReviewers, reviewHistory, clock>>

Init ==
    /\ roleHolder = [r \in Roles |-> {}]
    /\ conflictFinding = [s \in Subjects |-> FALSE]
    /\ activeReviewers = [s \in Subjects |-> {}]
    /\ reviewHistory = [s \in Subjects |-> {}]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

IndependentReviewRecorded(s) ==
    \E reviewer \in Subjects :
      /\ reviewer \in activeReviewers[s]
      /\ reviewer # s
      /\ \A role \in Roles : reviewer \notin roleHolder[role]

ActiveReviewRoleDisjointness ==
    \A subject \in Subjects, reviewer \in Subjects :
      reviewer \in activeReviewers[subject]
        => \A role \in Roles : reviewer \notin roleHolder[role]

AssignRole(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ ~(\E subject \in Subjects : s \in activeReviewers[subject])
    /\ LET currentRoles ==
          Cardinality({x \in Roles : s \in roleHolder[x]})
       IN
          /\ currentRoles < 1 \/ conflictFinding[s]
          /\ currentRoles < 3 \/ IndependentReviewRecorded(s)
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, activeReviewers, reviewHistory>>
    /\ clock' = NextTime

RecordConflictFinding(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~conflictFinding[s]
    /\ conflictFinding' = [conflictFinding EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<roleHolder, activeReviewers, reviewHistory>>
    /\ clock' = NextTime

OpenExternalReview(s, reviewer) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ reviewer \in Subjects
    /\ reviewer # s
    /\ ~activeReviewers[s] # {}
    /\ reviewer \notin reviewHistory[s]
    /\ \A role \in Roles : reviewer \notin roleHolder[role]
    /\ activeReviewers' = [activeReviewers EXCEPT ![s] = @ \cup {reviewer}]
    /\ reviewHistory' = [reviewHistory EXCEPT ![s] = @ \cup {reviewer}]
    /\ UNCHANGED <<roleHolder, conflictFinding>>
    /\ clock' = NextTime

CloseExternalReview(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ activeReviewers[s] # {}
    /\ activeReviewers' = [activeReviewers EXCEPT ![s] = {}]
    /\ UNCHANGED <<roleHolder, conflictFinding, reviewHistory>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, r \in Roles : AssignRole(s, r))
  \/ (\E s \in Subjects : RecordConflictFinding(s))
  \/ (\E s \in Subjects, reviewer \in Subjects : OpenExternalReview(s, reviewer))
  \/ (\E s \in Subjects : CloseExternalReview(s))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ roleHolder \in [Roles -> SUBSET Subjects]
    /\ conflictFinding \in [Subjects -> BOOLEAN]
    /\ activeReviewers \in [Subjects -> SUBSET Subjects]
    /\ reviewHistory \in [Subjects -> SUBSET Subjects]
    /\ clock \in Times

RoleConcentrationRequiresFinding ==
    \A s \in Subjects :
      Cardinality({r \in Roles : s \in roleHolder[r]}) >= 2
        => conflictFinding[s]

FullControlRequiresIndependentExternalReview ==
    \A s \in Subjects :
      Cardinality({r \in Roles : s \in roleHolder[r]}) = Cardinality(Roles)
        => IndependentReviewRecorded(s)

ReviewerRoleDisjointness ==
    ActiveReviewRoleDisjointness

ReviewHistoryPreserved ==
    \A s \in Subjects :
      activeReviewers[s] \subseteq reviewHistory[s]

Safety ==
    /\ TypeOK
    /\ RoleConcentrationRequiresFinding
    /\ FullControlRequiresIndependentExternalReview
    /\ ReviewerRoleDisjointness
    /\ ReviewHistoryPreserved

================================================================================