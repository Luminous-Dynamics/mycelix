---------------- MODULE EffectiveContestabilityV1NegativeControls ----------------
EXTENDS EffectiveContestabilityV1

CONSTANT Control

BadNominalPromotion(s, p) ==
  /\ Advanceable
  /\ NominalAlternative(s, p)
  /\ p \in nominalExit[s]
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
  /\ p \in nominalExit[s]
  /\ portable[s]
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
  /\ p \notin providerFailed
  /\ providerFailed' = providerFailed \cup {p}
  /\ failureObserved' = TRUE
  /\ authority' = [authority EXCEPT ![S1] = @ \ {PowerA}]
  /\ failureAuthority' = authority
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, jurisdiction,
                  switchingCost, reviewRequired, switchObserved,
                  authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime


BadSwitchStaleExit(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ p # currentProvider[s]
  /\ ~switchObserved[s]
  /\ portable[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ UNCHANGED <<viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

BadSwitchObligation(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ p # currentProvider[s]
  /\ ~switchObserved[s]
  /\ portable[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ authorityBeforeSwitch' = [authorityBeforeSwitch EXCEPT ![s] = @]
  /\ jurisdictionBeforeSwitch' = [jurisdictionBeforeSwitch EXCEPT ![s] = @]
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = {}]
  /\ obligationsPreserved' = [obligationsPreserved EXCEPT ![s] = FALSE]
  /\ UNCHANGED <<viableProviders, nominalExit, portable,
                  historyPreserved, authority, jurisdiction, switchingCost,
                  reviewRequired, providerFailed, failureObserved, failureAuthority>>
  /\ clock' = NextTime

BadSwitchAuthority(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ p # currentProvider[s]
  /\ ~switchObserved[s]
  /\ portable[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ authority' = [authority EXCEPT ![s] = @ \ {PowerA}]
  /\ authorityBeforeSwitch' = [authorityBeforeSwitch EXCEPT ![s] = @]
  /\ jurisdictionBeforeSwitch' = [jurisdictionBeforeSwitch EXCEPT ![s] = @]
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = {}]
  /\ UNCHANGED <<viableProviders, nominalExit, portable,
                  obligationsPreserved, historyPreserved, jurisdiction, switchingCost,
                  reviewRequired, providerFailed, failureObserved, failureAuthority>>
  /\ clock' = NextTime

BadSwitchJurisdiction(s, p) ==
  /\ Advanceable
  /\ p \in effectiveExit[s]
  /\ p # currentProvider[s]
  /\ ~switchObserved[s]
  /\ portable[s]
  /\ currentProvider' = [currentProvider EXCEPT ![s] = p]
  /\ switchObserved' = [switchObserved EXCEPT ![s] = TRUE]
  /\ authorityBeforeSwitch' = [authorityBeforeSwitch EXCEPT ![s] = @]
  /\ jurisdictionBeforeSwitch' = [jurisdictionBeforeSwitch EXCEPT ![s] = @]
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = {}]
  /\ jurisdiction' = [jurisdiction EXCEPT ![s] = @ \cup {J2}]
  /\ UNCHANGED <<viableProviders, nominalExit, portable, authority,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority>>
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
  \/ IF Control = "switch-authority" THEN
       \E s \in Subjects, p \in Providers : BadSwitchAuthority(s, p)
     ELSE FALSE
  \/ IF Control = "switch-jurisdiction" THEN
       \E s \in Subjects, p \in Providers : BadSwitchJurisdiction(s, p)
     ELSE FALSE
  \/ IF Control = "switch-review" THEN
       \E s \in Subjects : BadSwitchReview(s)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars

==========================================================================
