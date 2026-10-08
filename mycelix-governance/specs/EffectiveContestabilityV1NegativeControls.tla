---------------- MODULE EffectiveContestabilityV1NegativeControls ----------------
EXTENDS EffectiveContestabilityV1

CONSTANT Control

BadNominalPromotion(s, p) ==
  /\ Advanceable
  /\ NominalAlternative(s, p)
  /\ nominalExit' = nominalExit
  /\ portable' = [portable EXCEPT ![s] = FALSE]
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = @ \cup {p}]
  /\ UNCHANGED <<currentProvider, viableProviders,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, switchObserved, authorityBeforeSwitch,
                  jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadSharedRoot(s, p) ==
  /\ Advanceable
  /\ NominalAlternative(s, p)
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = @ \cup {p}]
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, switchObserved, authorityBeforeSwitch,
                  jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadFailureAuthority(p) ==
  /\ Advanceable
  /\ p \in Providers
  /\ providerFailed' = providerFailed \cup {p}
  /\ failureObserved' = TRUE
  /\ authority' = [authority EXCEPT ![S1] = @ \cup {PowerA}]
  /\ failureAuthority' = authority
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, jurisdiction,
                  switchingCost, reviewRequired, switchObserved,
                  authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadSwitchObligation(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ obligationsPreserved' = [obligationsPreserved EXCEPT ![s] = FALSE]
  /\ UNCHANGED <<viableProviders, nominalExit, effectiveExit, portable,
                  historyPreserved, authority, jurisdiction, switchingCost,
                  reviewRequired, providerFailed, failureObserved, failureAuthority,
                  authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadSwitchJurisdiction(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ jurisdiction' = [jurisdiction EXCEPT ![s] = @ \cup {J2}]
  /\ UNCHANGED <<viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, authority,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadSwitchReview(s) ==
  /\ Advanceable
  /\ s \in Subjects
  /\ switchingCost' = [switchingCost EXCEPT ![s] = ReviewThreshold]
  /\ reviewRequired' = [reviewRequired EXCEPT ![s] = FALSE]
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, effectiveExit,
                  portable, obligationsPreserved, historyPreserved, authority,
                  jurisdiction, providerFailed, failureObserved, failureAuthority,
                  switchObserved, authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

NegativeNext ==
  \/ Next
  \/ IF Control = "nominal-effective" THEN
       \E s \in Subjects, p \in Providers : BadNominalPromotion(s, p)
     ELSE FALSE
  \/ IF Control = "shared-root" THEN
       \E s \in Subjects, p \in Providers : BadSharedRoot(s, p)
     ELSE FALSE
  \/ IF Control = "failure-authority" THEN
       \E p \in Providers : BadFailureAuthority(p)
     ELSE FALSE
  \/ IF Control = "switch-obligation" THEN
       \E s \in Subjects, p \in Providers : BadSwitchObligation(s, p)
     ELSE FALSE
  \/ IF Control = "switch-jurisdiction" THEN
       \E s \in Subjects, p \in Providers : BadSwitchJurisdiction(s, p)
     ELSE FALSE
  \/ IF Control = "switch-review" THEN
       \E s \in Subjects : BadSwitchReview(s)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars

==========================================================================
