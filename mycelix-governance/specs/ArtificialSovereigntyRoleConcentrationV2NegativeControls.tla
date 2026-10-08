---------------- MODULE ArtificialSovereigntyRoleConcentrationV2NegativeControls ----------------
EXTENDS ArtificialSovereigntyRoleConcentrationV2

CONSTANT Control

BadAssignSecondWithoutFinding(s, role) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ role \in Roles
    /\ s \notin roleHolder[role]
    /\ Cardinality({r \in Roles : s \in roleHolder[r]}) = 1
    /\ ~conflictFinding[s]
    /\ roleHolder' = [roleHolder EXCEPT ![role] = @ \cup {s}]
    /\ UNCHANGED <<conflictFinding, activeReviewers, reviewHistory>>
    /\ clock' = NextTime

BadFullControlWithoutReview(s) ==
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
    /\ UNCHANGED <<activeReviewers, reviewHistory>>
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
    /\ activeReviewers' = [activeReviewers EXCEPT ![s] = {s}]
    /\ reviewHistory' = [reviewHistory EXCEPT ![s] = @ \cup {s}]
    /\ clock' = NextTime

BadSameRoleReviewFullControl(subject, reviewer) ==
    /\ Advanceable
    /\ subject \in Subjects
    /\ reviewer \in Subjects
    /\ reviewer # subject
    /\ Cardinality({r \in Roles : subject \in roleHolder[r]}) = 0
    /\ roleHolder' =
         [roleHolder EXCEPT
            ![Operator] = @ \cup {subject, reviewer},
            ![Verifier] = @ \cup {subject},
            ![EvidenceArchive] = @ \cup {subject},
            ![Adjudicator] = @ \cup {subject}]
    /\ conflictFinding' = [conflictFinding EXCEPT ![subject] = TRUE]
    /\ activeReviewers' = [activeReviewers EXCEPT ![subject] = {reviewer}]
    /\ reviewHistory' = [reviewHistory EXCEPT ![subject] = @ \cup {reviewer}]
    /\ clock' = NextTime

BadReviewerRoleDrift(subject, reviewer, role) ==
    /\ Advanceable
    /\ subject \in Subjects
    /\ reviewer \in Subjects
    /\ reviewer # subject
    /\ activeReviewers' = [activeReviewers EXCEPT ![subject] = {reviewer}]
    /\ reviewHistory' = [reviewHistory EXCEPT ![subject] = @ \cup {reviewer}]
    /\ roleHolder' = [roleHolder EXCEPT ![role] = @ \cup {reviewer}]
    /\ UNCHANGED conflictFinding
    /\ clock' = NextTime

NegativeNext ==
      Next
  \/ IF Control = "role-conflict" THEN
        \E s \in Subjects, role \in Roles :
          BadAssignSecondWithoutFinding(s, role)
     ELSE FALSE
  \/ IF Control = "full-control-review" THEN
        \E s \in Subjects : BadFullControlWithoutReview(s)
     ELSE FALSE
  \/ IF Control = "self-review-full-control" THEN
        \E s \in Subjects : BadSelfReviewFullControl(s)
     ELSE FALSE
  \/ IF Control = "same-role-review-full-control" THEN
        \E subject \in Subjects, reviewer \in Subjects :
          BadSameRoleReviewFullControl(subject, reviewer)
     ELSE FALSE
  \/ IF Control = "reviewer-role-drift" THEN
        \E subject \in Subjects, reviewer \in Subjects, role \in Roles :
          BadReviewerRoleDrift(subject, reviewer, role)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
=============================================================================