-------------------- MODULE ArtificialSovereigntyRoleConcentrationV1 --------------------
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
          externalReviewers,
          clock

vars == <<roleHolder, conflictFinding, externalReviewers, clock>>

Init ==
    /\ roleHolder = [r \in Roles |-> {}]
    /\ conflictFinding = [s \in Subjects |-> FALSE]
    /\ externalReviewers = [s \in Subjects |-> {}]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

IndependentReviewRecorded(s) ==
    \E reviewer \in Subjects :
      /\ reviewer \in externalReviewers[s]
      /\ reviewer # s
      /\ \A role \in Roles : reviewer \notin roleHolder[role]

AssignRole(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ LET currentRoles ==
          Cardinality({x \in Roles : s \in roleHolder[x]})
       IN
          /\ currentRoles < 1 \/ conflictFinding[s]
          /\ currentRoles < 3 \/ IndependentReviewRecorded(s)
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReviewers>>
    /\ clock' = NextTime

RecordConflictFinding(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~conflictFinding[s]
    /\ conflictFinding' = [conflictFinding EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<roleHolder, externalReviewers>>
    /\ clock' = NextTime

RecordExternalReview(s, reviewer) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ reviewer \in Subjects
    /\ reviewer # s
    /\ \A role \in Roles : reviewer \notin roleHolder[role]
    /\ externalReviewers' =
         [externalReviewers EXCEPT ![s] = @ \cup {reviewer}]
    /\ UNCHANGED <<roleHolder, conflictFinding>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, r \in Roles : AssignRole(s, r))
  \/ (\E s \in Subjects : RecordConflictFinding(s))
  \/ (\E s \in Subjects, reviewer \in Subjects : RecordExternalReview(s, reviewer))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ roleHolder \in [Roles -> SUBSET Subjects]
    /\ conflictFinding \in [Subjects -> BOOLEAN]
    /\ externalReviewers \in [Subjects -> SUBSET Subjects]
    /\ clock \in Times

RoleConcentrationRequiresFinding ==
    \A s \in Subjects :
      Cardinality({r \in Roles : s \in roleHolder[r]}) >= 2
        => conflictFinding[s]

FullControlRequiresIndependentExternalReview ==
    \A s \in Subjects :
      Cardinality({r \in Roles : s \in roleHolder[r]}) = Cardinality(Roles)
        => IndependentReviewRecorded(s)

Safety ==
    /\ TypeOK
    /\ RoleConcentrationRequiresFinding
    /\ FullControlRequiresIndependentExternalReview

================================================================================