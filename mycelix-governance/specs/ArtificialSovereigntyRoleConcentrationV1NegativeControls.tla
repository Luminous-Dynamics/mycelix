---------------- MODULE ArtificialSovereigntyRoleConcentrationV1NegativeControls ----------------
EXTENDS ArtificialSovereigntyRoleConcentrationV1

CONSTANT Control

BadAssignSecondWithoutFinding(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ Cardinality({x \in Roles : s \in roleHolder[x]}) = 1
    /\ ~conflictFinding[s]
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReviewers>>
    /\ clock' = NextTime

BadAssignFourthWithoutIndependentReview(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ Cardinality({x \in Roles : s \in roleHolder[x]}) = 3
    /\ conflictFinding[s]
    /\ ~IndependentReviewRecorded(s)
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReviewers>>
    /\ clock' = NextTime

BadSelfReviewFullControl(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ Cardinality({r \in Roles : s \in roleHolder[r]}) = 0
    /\ roleHolder' =
         [roleHolder EXCEPT
            ![Operator] = @ \cup {s},
            ![Verifier] = @ \cup {s},
            ![EvidenceArchive] = @ \cup {s},
            ![Adjudicator] = @ \cup {s}]
    /\ conflictFinding' = [conflictFinding EXCEPT ![s] = TRUE]
    /\ externalReviewers' = [externalReviewers EXCEPT ![s] = {s}]
    /\ clock' = NextTime

NegativeNext ==
      Next
  \/ IF Control = "role-conflict" THEN
        \E s \in Subjects, r \in Roles :
          BadAssignSecondWithoutFinding(s, r)
     ELSE FALSE
  \/ IF Control = "full-control-review" THEN
        \E s \in Subjects, r \in Roles :
          BadAssignFourthWithoutIndependentReview(s, r)
     ELSE FALSE
  \/ IF Control = "self-review-full-control" THEN
        \E s \in Subjects : BadSelfReviewFullControl(s)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
==============================================================