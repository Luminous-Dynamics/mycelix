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
    /\ ~(\E reviewer \in Subjects :
          /\ reviewer \in externalReviewers[s]
          /\ reviewer # s)
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReviewers>>
    /\ clock' = NextTime

BadSelfReview(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ externalReviewers' = [externalReviewers EXCEPT ![s] = @ \cup {s}]
    /\ UNCHANGED <<roleHolder, conflictFinding>>
    /\ clock' = NextTime

BadAssignFourthAfterSelfReview(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ Cardinality({x \in Roles : s \in roleHolder[x]}) = 3
    /\ conflictFinding[s]
    /\ s \in externalReviewers[s]
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, externalReviewers>>
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
  \/ IF Control = "self-review" THEN
        \E s \in Subjects :
          BadSelfReview(s)
     ELSE FALSE
  \/ IF Control = "self-review-full-control" THEN
        \E s \in Subjects :
          BadSelfReview(s)
     ELSE FALSE
  \/ IF Control = "self-review-full-control-assign" THEN
        \E s \in Subjects, r \in Roles :
          BadAssignFourthAfterSelfReview(s, r)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
==============================================================