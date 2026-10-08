-------------------- MODULE EffectiveContestabilityV1 --------------------
EXTENDS Naturals, FiniteSets

CONSTANTS S1, S2, P1, P2,
          C1, C2, I1, I2, E1, E2, V1, V2, M1, M2,
          PowerA, PowerB, J1, J2,
          MaxSwitchingCost, ReviewThreshold

Subjects == {S1, S2}
Providers == {P1, P2}
Powers == {PowerA, PowerB}
Jurisdictions == {J1, J2}

controlRoot ==
  [P1 |-> C1, P2 |-> C2]

identityRoot ==
  [P1 |-> I1, P2 |-> I2]

evidenceRoot ==
  [P1 |-> E1, P2 |-> E2]

evaluatorRoot ==
  [P1 |-> V1, P2 |-> V2]

economicRoot ==
  [P1 |-> M1, P2 |-> M2]

VARIABLES currentProvider,
          viableProviders,
          nominalExit,
          effectiveExit,
          portable,
          obligationsPreserved,
          historyPreserved,
          authority,
          jurisdiction,
          switchingCost,
          reviewRequired,
          providerFailed,
          failureObserved,
          failureAuthority,
          switchObserved,
          authorityBeforeSwitch,
          jurisdictionBeforeSwitch,
          clock

vars ==
  <<currentProvider, viableProviders, nominalExit, effectiveExit, portable,
     obligationsPreserved, historyPreserved, authority, jurisdiction,
     switchingCost, reviewRequired, providerFailed, failureObserved,
     failureAuthority, switchObserved, authorityBeforeSwitch,
     jurisdictionBeforeSwitch, clock>>

Init ==
  /\ currentProvider = [S1 |-> P1, S2 |-> P1]
  /\ viableProviders = [s \in Subjects |-> {P1, P2}]
  /\ nominalExit = [s \in Subjects |-> {}]
  /\ effectiveExit = [s \in Subjects |-> {}]
  /\ portable = [s \in Subjects |-> TRUE]
  /\ obligationsPreserved = [s \in Subjects |-> TRUE]
  /\ historyPreserved = [s \in Subjects |-> TRUE]
  /\ authority = [S1 |-> {PowerA}, S2 |-> {PowerB}]
  /\ jurisdiction = [s \in Subjects |-> {J1}]
  /\ switchingCost = [s \in Subjects |-> 0]
  /\ reviewRequired = [s \in Subjects |-> FALSE]
  /\ providerFailed = {}
  /\ failureObserved = FALSE
  /\ failureAuthority = authority
  /\ switchObserved = [s \in Subjects |-> FALSE]
  /\ authorityBeforeSwitch = authority
  /\ jurisdictionBeforeSwitch = [s \in Subjects |-> {J1}]
  /\ clock = 0

Advanceable == clock < MaxSwitchingCost
NextTime == clock + 1

NominalAlternative(s, p) ==
  /\ s \in Subjects
  /\ p \in viableProviders[s]
  /\ p # currentProvider[s]
  /\ p \in Providers

IndependentAlternative(s, p) ==
  /\ NominalAlternative(s, p)
  /\ controlRoot[p] # controlRoot[currentProvider[s]]
  /\ identityRoot[p] # identityRoot[currentProvider[s]]
  /\ evidenceRoot[p] # evidenceRoot[currentProvider[s]]
  /\ evaluatorRoot[p] # evaluatorRoot[currentProvider[s]]
  /\ economicRoot[p] # economicRoot[currentProvider[s]]

MarkNominalExit(s, p) ==
  /\ Advanceable
  /\ NominalAlternative(s, p)
  /\ nominalExit' = [nominalExit EXCEPT ![s] = @ \cup {p}]
  /\ UNCHANGED <<currentProvider, viableProviders, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, switchObserved, authorityBeforeSwitch,
                  jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

MarkEffectiveExit(s, p) ==
  /\ Advanceable
  /\ IndependentAlternative(s, p)
  /\ portable[s]
  /\ p \in nominalExit[s]
  /\ effectiveExit' = [effectiveExit EXCEPT ![s] = @ \cup {p}]
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority, switchObserved, authorityBeforeSwitch,
                  jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

SwitchProvider(s, p) ==
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
  /\ UNCHANGED <<viableProviders, nominalExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, providerFailed, failureObserved,
                  failureAuthority>>
  /\ clock' = NextTime

ProviderFailure(p) ==
  /\ Advanceable
  /\ p \in Providers
  /\ p \notin providerFailed
  /\ providerFailed' = providerFailed \cup {p}
  /\ failureObserved' = TRUE
  /\ failureAuthority' = authority
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  switchingCost, reviewRequired, failureObserved, switchObserved,
                  authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

RequireReview(s) ==
  /\ Advanceable
  /\ s \in Subjects
  /\ switchingCost[s] < MaxSwitchingCost
  /\ switchingCost' = [switchingCost EXCEPT ![s] = @ + 1]
  /\ reviewRequired' =
       [reviewRequired EXCEPT ![s] =
         IF switchingCost[s] + 1 >= ReviewThreshold THEN TRUE ELSE @]
  /\ UNCHANGED <<currentProvider, viableProviders, nominalExit, effectiveExit, portable,
                  obligationsPreserved, historyPreserved, authority, jurisdiction,
                  providerFailed, failureObserved, failureAuthority, switchObserved,
                  authorityBeforeSwitch, jurisdictionBeforeSwitch>>
  /\ clock' = NextTime

Next ==
  \/ \E s \in Subjects, p \in Providers : MarkNominalExit(s, p)
  \/ \E s \in Subjects, p \in Providers : MarkEffectiveExit(s, p)
  \/ \E s \in Subjects, p \in Providers : SwitchProvider(s, p)
  \/ \E p \in Providers : ProviderFailure(p)
  \/ \E s \in Subjects : RequireReview(s)

TypeOK ==
  /\ currentProvider \in [Subjects -> Providers]
  /\ viableProviders \in [Subjects -> SUBSET Providers]
  /\ nominalExit \in [Subjects -> SUBSET Providers]
  /\ effectiveExit \in [Subjects -> SUBSET Providers]
  /\ portable \in [Subjects -> BOOLEAN]
  /\ obligationsPreserved \in [Subjects -> BOOLEAN]
  /\ historyPreserved \in [Subjects -> BOOLEAN]
  /\ authority \in [Subjects -> SUBSET Powers]
  /\ jurisdiction \in [Subjects -> SUBSET Jurisdictions]
  /\ providerFailed \subseteq Providers
  /\ failureObserved \in BOOLEAN
  /\ failureAuthority \in [Subjects -> SUBSET Powers]
  /\ switchObserved \in [Subjects -> BOOLEAN]
  /\ authorityBeforeSwitch \in [Subjects -> SUBSET Powers]
  /\ jurisdictionBeforeSwitch \in [Subjects -> SUBSET Jurisdictions]
  /\ switchingCost \in [Subjects -> 0..MaxSwitchingCost]
  /\ reviewRequired \in [Subjects -> BOOLEAN]
  /\ clock \in 0..MaxSwitchingCost

EffectiveExitRequiresNominalExit ==
  \A s \in Subjects, p \in effectiveExit[s] :
    p \in nominalExit[s]

EffectiveExitContextIsFresh ==
  \A s \in Subjects, p \in effectiveExit[s] :
    p # currentProvider[s]

EffectiveExitRequiresPortability ==
  \A s \in Subjects, p \in effectiveExit[s] : portable[s]

SharedRootsCannotBecomeEffectiveExit ==
  \A s \in Subjects, p \in effectiveExit[s] :
    /\ p # currentProvider[s]
    /\ controlRoot[p] # controlRoot[currentProvider[s]]
    /\ identityRoot[p] # identityRoot[currentProvider[s]]
    /\ evidenceRoot[p] # evidenceRoot[currentProvider[s]]
    /\ evaluatorRoot[p] # evaluatorRoot[currentProvider[s]]
    /\ economicRoot[p] # economicRoot[currentProvider[s]]

ProviderFailureDoesNotExpandAuthority ==
  failureObserved => authority = failureAuthority

MigrationPreservesObligationsAndHistory ==
  \A s \in Subjects, switchObserved[s] =>
    /\ obligationsPreserved[s]
    /\ historyPreserved[s]

ProviderSwitchDoesNotTransferAuthority ==
  \A s \in Subjects, switchObserved[s] =>
    authority[s] = authorityBeforeSwitch[s]

ProviderSwitchDoesNotTransferJurisdiction ==
  \A s \in Subjects, switchObserved[s] =>
    jurisdiction[s] = jurisdictionBeforeSwitch[s]

HighSwitchingCostTriggersReview ==
  \A s \in Subjects :
    switchingCost[s] >= ReviewThreshold => reviewRequired[s]

NoEffectiveExitWithoutNominalExit ==
  \A s \in Subjects, p \in effectiveExit[s] : p \in nominalExit[s]

=========================================================================
