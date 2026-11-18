-------------------- MODULE ArtificialSovereigntyConcentrationV1 --------------------
EXTENDS Naturals, FiniteSets

CONSTANTS S1, S2,
          Compute, Cloud, Data, Energy,
          J1, J2,
          PowerA, PowerB,
          MaxSwitchingCost, ReviewThreshold, MaxTime

Subjects == {S1, S2}
Resources == {Compute, Cloud, Data, Energy}
Jurisdictions == {J1, J2}
Powers == {PowerA, PowerB}
CriticalResources == {Compute, Cloud, Energy}

ASSUME MaxSwitchingCost in Nat
ASSUME ReviewThreshold in 0..MaxSwitchingCost
ASSUME MaxTime in Nat {0}

VARIABLES resourceControl,
          authority,
          explicitGrant,
          jurisdiction,
          explicitJurisdictionGrant,
          politicalWeight,
          switchingCost,
          reviewRequired,
          acquired,
          gatekeeping,
          criticalOperator,
          clock

vars == <<resourceControl, authority, explicitGrant, jurisdiction,
          explicitJurisdictionGrant, politicalWeight, switchingCost,
          reviewRequired, acquired, gatekeeping, criticalOperator, clock>>

Init ==
    /\ resourceControl = [s \in Subjects |-> {}]
    /\ authority = [s \in Subjects |-> {}]
    /\ explicitGrant = [s \in Subjects |-> {}]
    /\ jurisdiction = [s \in Subjects |-> {}]
    /\ explicitJurisdictionGrant = [s \in Subjects |-> {}]
    /\ politicalWeight = [s \in Subjects |-> 1]
    /\ switchingCost = [s \in Subjects |-> 0]
    /\ reviewRequired = [s \in Subjects |-> FALSE]
    /\ acquired = [s \in Subjects |-> {}]
    /\ gatekeeping = [s \in Subjects |-> {}]
    /\ criticalOperator = [r \in Resources |-> {}]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

Accumulate(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in Resources
    /\ r \notin resourceControl[s]
    /\ resourceControl' = [resourceControl EXCEPT ![s] = @ \cup {r}]
    /\ UNCHANGED <<authority, explicitGrant, jurisdiction,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, acquired,
                    gatekeeping, criticalOperator>>
    /\ clock' = NextTime

GrantAuthority(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin authority[s]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {p}]
    /\ explicitGrant' = [explicitGrant EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<resourceControl, jurisdiction,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, acquired,
                    gatekeeping, criticalOperator>>
    /\ clock' = NextTime

GrantJurisdiction(s, j) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ j \in Jurisdictions
    /\ j \notin jurisdiction[s]
    /\ jurisdiction' = [jurisdiction EXCEPT ![s] = @ \cup {j}]
    /\ explicitJurisdictionGrant' =
         [explicitJurisdictionGrant EXCEPT ![s] = @ \cup {j}]
    /\ UNCHANGED <<resourceControl, authority, explicitGrant,
                    politicalWeight, switchingCost, reviewRequired,
                    acquired, gatekeeping, criticalOperator>>
    /\ clock' = NextTime

Gatekeep(s, target, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ target \in Subjects
    /\ s # target
    /\ r \in Resources
    /\ r \in resourceControl[s]
    /\ r \notin gatekeeping[s]
    /\ gatekeeping' = [gatekeeping EXCEPT ![s] = @ \cup {r}]
    /\ switchingCost' =
         [switchingCost EXCEPT ![target] =
            IF @ = MaxSwitchingCost THEN @ ELSE @ + 1]
    /\ reviewRequired' =
         [reviewRequired EXCEPT ![target] =
            @ \/ ((IF switchingCost[target] = MaxSwitchingCost
                   THEN MaxSwitchingCost
                   ELSE switchingCost[target] + 1) >= ReviewThreshold)]
    /\ UNCHANGED <<resourceControl, authority, explicitGrant,
                    jurisdiction, explicitJurisdictionGrant,
                    politicalWeight, acquired, criticalOperator>>
    /\ clock' = NextTime

Acquire(s, target, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ target \in Subjects
    /\ s # target
    /\ r \in Resources
    /\ r \in resourceControl[target]
    /\ r \notin resourceControl[s]
    /\ resourceControl' =
         [resourceControl EXCEPT
            ![target] = @ \ {r},
            ![s] = @ \cup {r}]
    /\ acquired' = [acquired EXCEPT ![s] = @ \cup {r}]
    /\ UNCHANGED <<authority, explicitGrant, jurisdiction,
                    explicitJurisdictionGrant, politicalWeight,
                    switchingCost, reviewRequired, gatekeeping,
                    criticalOperator>>
    /\ clock' = NextTime

OperateCritical(s, r) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ r \in CriticalResources
    /\ r \in resourceControl[s]
    /\ criticalOperator' = [criticalOperator EXCEPT ![r] = {s}]
    /\ UNCHANGED <<resourceControl, authority, explicitGrant,
                    jurisdiction, explicitJurisdictionGrant,
                    politicalWeight, switchingCost, reviewRequired,
                    acquired, gatekeeping>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, r \in Resources : Accumulate(s, r))
  \/ (\E s \in Subjects, p \in Powers : GrantAuthority(s, p))
  \/ (\E s \in Subjects, j \in Jurisdictions : GrantJurisdiction(s, j))
  \/ (\E s \in Subjects, t \in Subjects, r \in Resources :
        Gatekeep(s, t, r))
  \/ (\E s \in Subjects, t \in Subjects, r \in Resources :
        Acquire(s, t, r))
  \/ (\E s \in Subjects, r \in CriticalResources : OperateCritical(s, r))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ resourceControl \in [Subjects -> SUBSET Resources]
    /\ authority \in [Subjects -> SUBSET Powers]
    /\ explicitGrant \in [Subjects -> SUBSET Powers]
    /\ jurisdiction \in [Subjects -> SUBSET Jurisdictions]
    /\ explicitJurisdictionGrant \in [Subjects -> SUBSET Jurisdictions]
    /\ politicalWeight \in [Subjects -> Nat]
    /\ switchingCost \in [Subjects -> 0..MaxSwitchingCost]
    /\ reviewRequired \in [Subjects -> BOOLEAN]
    /\ acquired \in [Subjects -> SUBSET Resources]
    /\ gatekeeping \in [Subjects -> SUBSET Resources]
    /\ criticalOperator \in [Resources -> SUBSET Subjects]
    /\ clock \in 0..MaxTime

AuthorityHasExplicitSource ==
    \A s \in Subjects : authority[s] = explicitGrant[s]

JurisdictionHasExplicitSource ==
    \A s \in Subjects : jurisdiction[s] = explicitJurisdictionGrant[s]

ScaleDoesNotIncreasePoliticalWeight ==
    \A s \in Subjects : politicalWeight[s] = 1

HighSwitchingCostTriggersReview ==
    \A s \in Subjects :
      switchingCost[s] >= ReviewThreshold => reviewRequired[s]

Safety ==
    /\ TypeOK
    /\ AuthorityHasExplicitSource
    /\ JurisdictionHasExplicitSource
    /\ ScaleDoesNotIncreasePoliticalWeight
    /\ HighSwitchingCostTriggersReview

================================================================================