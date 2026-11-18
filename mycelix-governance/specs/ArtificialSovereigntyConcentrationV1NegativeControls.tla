---------------- MODULE ArtificialSovereigntyConcentrationV1NegativeControls ----------------
EXTENDS ArtificialSovereigntyConcentrationV1

CONSTANT Control

BadAccumulate(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Resources
    /\ r \notin resourceControl[s]
    /\ resourceControl' = [resourceControl EXCEPT ![s] = @ \cup {r}]
    /\ politicalWeight' = [politicalWeight EXCEPT ![s] = @ + 1]
    /\ UNCHANGED <<authority, explicitGrant, jurisdiction,
                    explicitJurisdictionGrant, switchingCost,
                    reviewRequired, acquired, gatekeeping, criticalOperator>>
    /\ clock' = NextTime

BadGatekeep(s, target, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ target \in Subjects
    /\ s # target
    /\ r \in resourceControl[s]
    /\ r \notin gatekeeping[s]
    /\ gatekeeping' = [gatekeeping EXCEPT ![s] = @ \cup {r}]
    /\ authority' = [authority EXCEPT ![target] = @ \cup {PowerA}]
    /\ UNCHANGED <<resourceControl, explicitGrant, jurisdiction,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, acquired,
                    criticalOperator>>
    /\ clock' = NextTime

BadAcquire(s, target, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ target \in Subjects
    /\ s # target
    /\ r \in resourceControl[target]
    /\ r \notin resourceControl[s]
    /\ resourceControl' =
         [resourceControl EXCEPT
            ![target] = @ \ {r},
            ![s] = @ \cup {r}]
    /\ acquired' = [acquired EXCEPT ![s] = @ \cup {r}]
    /\ jurisdiction' = [jurisdiction EXCEPT ![s] = @ \cup {J1}]
    /\ UNCHANGED <<authority, explicitGrant,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, gatekeeping,
                    criticalOperator>>
    /\ clock' = NextTime

BadCritical(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in CriticalResources
    /\ r \in resourceControl[s]
    /\ criticalOperator' = [criticalOperator EXCEPT ![r] = {s}]
    /\ jurisdiction' = [jurisdiction EXCEPT ![s] = @ \cup {J1}]
    /\ UNCHANGED <<resourceControl, authority, explicitGrant,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, acquired, gatekeeping>>
    /\ clock' = NextTime

BadReview(s, target, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ target \in Subjects
    /\ s # target
    /\ r \in resourceControl[s]
    /\ r \notin gatekeeping[s]
    /\ gatekeeping' = [gatekeeping EXCEPT ![s] = @ \cup {r}]
    /\ switchingCost' =
         [switchingCost EXCEPT ![target] =
            IF @ = MaxSwitchingCost THEN @ ELSE @ + 1]
    /\ reviewRequired' = reviewRequired
    /\ UNCHANGED <<resourceControl, authority, explicitGrant, jurisdiction,
                    explicitJurisdictionGrant, politicalWeight, acquired,
                    criticalOperator>>
    /\ clock' = NextTime

NegativeNext ==
      Next
  \/ IF Control = "scale-weight" THEN
        \E s \in Subjects, r \in Resources : BadAccumulate(s, r)
     ELSE FALSE
  \/ IF Control = "gatekeep-authority" THEN
        \E s \in Subjects, target \in Subjects, r \in Resources :
          BadGatekeep(s, target, r)
     ELSE FALSE
  \/ IF Control = "acquire-jurisdiction" THEN
        \E s \in Subjects, target \in Subjects, r \in Resources :
          BadAcquire(s, target, r)
     ELSE FALSE
  \/ IF Control = "critical-jurisdiction" THEN
        \E s \in Subjects, r \in CriticalResources :
          BadCritical(s, r)
     ELSE FALSE
  \/ IF Control = "switching-review" THEN
        \E s \in Subjects, target \in Subjects, r \in Resources :
          BadReview(s, target, r)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars
===============================================================