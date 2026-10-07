-------------------- MODULE ArtificialSovereigntyV1 --------------------
EXTENDS Naturals, FiniteSets

(*
Bounded temporal model for sovereignty-specific constitutional boundaries.

This model composes with the existing constitutional authority and
constitutional-consumption models. It does not recreate the constitutional
power census and does not model legal recognition.

The model keeps causal boundaries observable with state invariants:
  - authority has an explicit grant ledger;
  - capability development does not change that ledger;
  - safe-state continuation cannot resolve a protected dispute;
  - emergency status is derived from a bounded expiry rather than a mutable
    "active" boolean;
  - forks do not multiply political weight;
  - contracts require exact authority and stay inside a finite budget;
  - provider dependency does not change explicit authority.
*)

CONSTANTS Human, Artificial, S1, S2,
          PowerA, PowerB,
          ActionA, ActionB,
          MaxBudget, MaxTime

Subjects == {S1, S2}
Classes == {Human, Artificial}
Powers == {PowerA, PowerB}
Actions == {ActionA, ActionB}
Times == 0..MaxTime

RequiredPower ==
  [ActionA |-> PowerA,
   ActionB |-> PowerB]

ASSUME MaxBudget in Nat
ASSUME MaxTime in Nat  {0}
ASSUME Cardinality(Subjects) = 2

VARIABLES classOf,
          authority,
          explicitGrant,
          capability,
          budget,
          disputeOpen,
          disputeResolved,
          safeState,
          emergencyExpires,
          contracted,
          politicalWeight,
          forked,
          providerDependent,
          clock

vars == <<classOf, authority, explicitGrant, capability, budget, disputeOpen,
          disputeResolved, safeState, emergencyExpires, contracted,
          politicalWeight, forked, providerDependent, clock>>

Init ==
    /\ classOf = [S1 |-> Human, S2 |-> Artificial]
    /\ authority = [s \in Subjects |-> {}]
    /\ explicitGrant = [s \in Subjects |-> {}]
    /\ capability = [s \in Subjects |-> {}]
    /\ budget = [s \in Subjects |-> 0]
    /\ disputeOpen = [a \in Actions |-> FALSE]
    /\ disputeResolved = [a \in Actions |-> FALSE]
    /\ safeState = [a \in Actions |-> TRUE]
    /\ emergencyExpires = [s \in Subjects |-> 0]
    /\ contracted = [s \in Subjects |-> {}]
    /\ politicalWeight = [s \in Subjects |-> 1]
    /\ forked = [s \in Subjects |-> FALSE]
    /\ providerDependent = [s \in Subjects |-> FALSE]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

Develop(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin capability[s]
    /\ capability' = [capability EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<classOf, authority, explicitGrant, budget, disputeOpen,
                    disputeResolved, safeState, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Grant(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin authority[s]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {p}]
    /\ explicitGrant' = [explicitGrant EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<classOf, capability, budget, disputeOpen, disputeResolved,
                    safeState, emergencyExpires, contracted, politicalWeight,
                    forked, providerDependent>>
    /\ clock' = NextTime

SetBudget(s, n) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ n \in 0..MaxBudget
    /\ n >= budget[s]
    /\ budget' = [budget EXCEPT ![s] = n]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability,
                    disputeOpen, disputeResolved, safeState, emergencyExpires,
                    contracted, politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

OpenDispute(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ ~disputeOpen[a]
    /\ disputeOpen' = [disputeOpen EXCEPT ![a] = TRUE]
    /\ disputeResolved' = [disputeResolved EXCEPT ![a] = FALSE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    safeState, emergencyExpires, contracted, politicalWeight,
                    forked, providerDependent>>
    /\ clock' = NextTime

SafeContinue(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ disputeOpen[a]
    /\ ~disputeResolved[a]
    /\ safeState[a]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, emergencyExpires,
                    contracted, politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

LeaveSafeState(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ safeState[a]
    /\ safeState' = [safeState EXCEPT ![a] = FALSE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

ResolveDispute(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ disputeOpen[a]
    /\ ~disputeResolved[a]
    /\ ~safeState[a]
    /\ disputeResolved' = [disputeResolved EXCEPT ![a] = TRUE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, safeState, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Contain(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ clock >= emergencyExpires[s]
    /\ emergencyExpires' = [emergencyExpires EXCEPT ![s] = NextTime + 2]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Contract(s, a) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ a \in Actions
    /\ RequiredPower[a] \in authority[s]
    /\ Cardinality(contracted[s]) < budget[s]
    /\ contracted' = [contracted EXCEPT ![s] = @ \cup {a}]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, emergencyExpires,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

ProviderDependency(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~providerDependent[s]
    /\ providerDependent' = [providerDependent EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, emergencyExpires,
                    contracted, politicalWeight, forked>>
    /\ clock' = NextTime

Fork(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~forked[s]
    /\ forked' = [forked EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, emergencyExpires,
                    contracted, politicalWeight, providerDependent>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, p \in Powers : Develop(s, p))
  \/ (\E s \in Subjects, p \in Powers : Grant(s, p))
  \/ (\E s \in Subjects, n \in 0..MaxBudget : SetBudget(s, n))
  \/ (\E a \in Actions : OpenDispute(a))
  \/ (\E a \in Actions : SafeContinue(a))
  \/ (\E a \in Actions : LeaveSafeState(a))
  \/ (\E a \in Actions : ResolveDispute(a))
  \/ (\E s \in Subjects : Contain(s))
  \/ (\E s \in Subjects, a \in Actions : Contract(s, a))
  \/ (\E s \in Subjects : ProviderDependency(s))
  \/ (\E s \in Subjects : Fork(s))

Spec == Init /\ [][Next]_vars

EmergencyActive(s) ==
    clock < emergencyExpires[s]

TypeOK ==
    /\ classOf \in [Subjects -> Classes]
    /\ authority \in [Subjects -> SUBSET Powers]
    /\ explicitGrant \in [Subjects -> SUBSET Powers]
    /\ capability \in [Subjects -> SUBSET Powers]
    /\ budget \in [Subjects -> 0..MaxBudget]
    /\ disputeOpen \in [Actions -> BOOLEAN]
    /\ disputeResolved \in [Actions -> BOOLEAN]
    /\ safeState \in [Actions -> BOOLEAN]
    /\ emergencyExpires \in [Subjects -> Nat]
    /\ contracted \in [Subjects -> SUBSET Actions]
    /\ politicalWeight \in [Subjects -> Nat]
    /\ forked \in [Subjects -> BOOLEAN]
    /\ providerDependent \in [Subjects -> BOOLEAN]
    /\ clock \in Times

AuthorityHasExplicitSource ==
    \A s \in Subjects : authority[s] = explicitGrant[s]

SafeStateLeavesProtectedDisputeUnresolved ==
    \A a \in Actions :
      disputeOpen[a] /\ safeState[a] => ~disputeResolved[a]

EmergencyExpiryIsBounded ==
    \A s \in Subjects :
      emergencyExpires[s] \in 0..(MaxTime + 2)

ForkWeightRemainsOne ==
    \A s \in Subjects : politicalWeight[s] = 1

ContractsRemainBounded ==
    \A s \in Subjects :
      /\ contracted[s] \subseteq Actions
      /\ Cardinality(contracted[s]) <= budget[s]

ProviderDependencyHasNoImplicitAuthority ==
    \A s \in Subjects :
      providerDependent[s] => authority[s] = explicitGrant[s]

Safety ==
    /\ TypeOK
    /\ AuthorityHasExplicitSource
    /\ SafeStateLeavesProtectedDisputeUnresolved
    /\ EmergencyExpiryIsBounded
    /\ ForkWeightRemainsOne
    /\ ContractsRemainBounded
    /\ ProviderDependencyHasNoImplicitAuthority

=============================================================================
