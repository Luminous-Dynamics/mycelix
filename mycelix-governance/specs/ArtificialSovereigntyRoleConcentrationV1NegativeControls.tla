---------------- MODULE ArtificialSovereigntyRoleConcentrationV1NegativeControls ----------------
EXTENDS ArtificialSovereigntyRoleConcentrationV1

CONSTANT Control

BadAssign(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Roles
    /\ s \notin roleHolder[r]
    /\ roleHolder' = [roleHolder EXCEPT ![r] = @ \cup {s}]
    /\ conflictFinding' = conflictFinding
    /\ externalReview' = externalReview
    /\ clock' = NextTime

NegativeNext ==
      Next
  \/ IF Control = "role-conflict" THEN
        \E s \in Subjects, r \in CriticalRoles : BadAssign(s, r)
     ELSE FALSE
  \/ IF Control = "full-control-review" THEN
        \E s \in Subjects, r \in CriticalRoles : BadAssign(s, r)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
==============================================================