-------------------- MODULE ArtificialSovereigntyRoleConcentrationV1 --------------------
EXTENDS Naturals, FiniteSets

CONSTANTS S1, S2,
          Operator, Verifier, EvidenceArchive, Adjudicator,
          MaxTime

Subjects == {S1, S2}
Roles == {Operator, Verifier, EvidenceArchive, Adjudicator}
CriticalRoles == {Operator, Verifier, EvidenceArchive}
Times == 0..MaxTime

ASSUME MaxTime in Nat {0}

VARIABLES roleHolder,
          conflictFinding,
          externalReview,
          clock

vars == <<roleHolder, conflictFinding, externalReview, clock>>

Init ==
    /\ roleHolder = [r \in Roles |-> {}]
    /\ conflictFinding = [s \in Subjects |-> FALSE]
    /\ externalReview = [s \in Subjects |-> FALSE]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

AssignRole(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ LET currentCritical ==
          Cardinality({x \in CriticalRoles : s \in roleHolder[x]})
       IN
          /\ r \notin CriticalRoles
              \/ (currentCritical < 2 \/ conflictFinding[s])
          /\ r \notin CriticalRoles
              \/ (currentCritical < 3 \/ externalReview[s])
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReview>>
    /\ clock' = NextTime

RecordConflictFinding(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~conflictFinding[s]
    /\ conflictFinding' = [conflictFinding EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<roleHolder, externalReview>>
    /\ clock' = NextTime

RecordExternalReview(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ externalReview' = [externalReview EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<roleHolder, conflictFinding>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, r \in Roles : AssignRole(s, r))
  \/ (\E s \in Subjects : RecordConflictFinding(s))
  \/ (\E s \in Subjects : RecordExternalReview(s))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ roleHolder \in [Roles -> SUBSET Subjects]
    /\ conflictFinding \in [Subjects -> BOOLEAN]
    /\ externalReview \in [Subjects -> BOOLEAN]
    /\ clock \in Times

RoleConcentrationRequiresFinding ==
    \A s \in Subjects :
      Cardinality({r \in CriticalRoles : s \in roleHolder[r]}) >= 2
        => conflictFinding[s]

ExternalReviewRequiredForFullControlConcentration ==
    \A s \in Subjects :
      Cardinality({r \in CriticalRoles : s \in roleHolder[r]}) = Cardinality(CriticalRoles)
        => externalReview[s]

Safety ==
    /\ TypeOK
    /\ RoleConcentrationRequiresFinding
    /\ ExternalReviewRequiredForFullControlConcentration

================================================================================