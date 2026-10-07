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

BadFullControlWithoutIndependentReview(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ Cardinality({x \in Roles : s \in roleHolder[x]}) = 0
    /\ roleHolder' =
         [roleHolder EXCEPT
            ![Operator] = @ \cup {s},
            ![Verifier] = @ \cup {s},
            ![EvidenceArchive] = @ \cup {s},
            ![Adjudicator] = @ \cup {s}]
    /\ conflictFinding' = [conflictFinding EXCEPT ![s] = TRUE]
    /\ UNCHANGED externalReviewers
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
        \E s \in Subjects : BadFullControlWithoutIndependentReview(s)
     ELSE FALSE
  \/ IF Control = "self-review-full-control" THEN
        \E s \in Subjects : BadSelfReviewFullControl(s)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
==============================================================