-------------------- MODULE ArtificialSovereigntyV1 --------------------
EXTENDS Naturals, FiniteSets

(*
Bounded temporal model for sovereignty-specific constitutional boundaries.

This model composes with ConstitutionalConsumptionV2 and the existing
constitutional authority models. It does not recreate the constitutional
power census and does not model legal recognition.

Questions:
  1. Can self-development change capability without changing authority?
  2. Can a protected safe state continue while a dispute remains unresolved?
  3. Can emergency containment remain bounded rather than become permanent?
  4. Can a fork occur without multiplying political weight?
  5. Can autonomous contracting remain inside explicit authority and budget?
  6. Can provider dependency exist without creating constitutional authority?

Safety is separate from liveness. A finite model check is bounded evidence.
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
          capability,
          budget,
          disputeOpen,
          disputeResolved,
          safeState,
          emergency,
          emergencyExpires,
          contracted,
          politicalWeight,
          forked,
          providerDependent,
          clock

vars == <<classOf, authority, capability, budget, disputeOpen,
          disputeResolved, safeState, emergency, emergencyExpires,
          contracted, politicalWeight, forked, providerDependent, clock>>

Init ==
    /\ classOf = [S1 |-> Human, S2 |-> Artificial]
    /\ authority = [s \in Subjects |-> {}]
    /\ capability = [s \in Subjects |-> {}]
    /\ budget = [s \in Subjects |-> 0]
    /\ disputeOpen = [a \in Actions |-> FALSE]
    /\ disputeResolved = [a \in Actions |-> FALSE]
    /\ safeState = [a \in Actions |-> TRUE]
    /\ emergency = [s \in Subjects |-> FALSE]
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
    /\ UNCHANGED <<classOf, authority, budget, disputeOpen, disputeResolved,
                    safeState, emergency, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Grant(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin authority[s]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<classOf, capability, budget, disputeOpen, disputeResolved,
                    safeState, emergency, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

SetBudget(s, n) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ n \in 0..MaxBudget
    /\ n >= budget[s]
    /\ budget' = [budget EXCEPT ![s] = n]
    /\ UNCHANGED <<classOf, authority, capability, disputeOpen, disputeResolved,
                    safeState, emergency, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

OpenDispute(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ ~disputeOpen[a]
    /\ disputeOpen' = [disputeOpen EXCEPT ![a] = TRUE]
    /\ disputeResolved' = [disputeResolved EXCEPT ![a] = FALSE]
    /\ UNCHANGED <<classOf, authority, capability, budget, safeState,
                    emergency, emergencyExpires, contracted, politicalWeight,
                    forked, providerDependent>>
    /\ clock' = NextTime

SafeContinue(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ disputeOpen[a]
    /\ ~disputeResolved[a]
    /\ safeState[a]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergency, emergencyExpires,
                    contracted, politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

LeaveSafeState(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ safeState[a]
    /\ safeState' = [safeState EXCEPT ![a] = FALSE]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, emergency, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

ResolveDispute(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ disputeOpen[a]
    /\ ~disputeResolved[a]
    /\ ~safeState[a]
    /\ disputeResolved' = [disputeResolved EXCEPT ![a] = TRUE]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    safeState, emergency, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Contain(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~emergency[s]
    /\ emergency' = [emergency EXCEPT ![s] = TRUE]
    /\ emergencyExpires' = [emergencyExpires EXCEPT ![s] = NextTime + 1]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, contracted, politicalWeight,
                    forked, providerDependent>>
    /\ clock' = NextTime

ExpireEmergency(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ emergency[s]
    /\ clock >= emergencyExpires[s]
    /\ emergency' = [emergency EXCEPT ![s] = FALSE]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

Contract(s, a) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ a \in Actions
    /\ RequiredPower[a] \in authority[s]
    /\ Cardinality(contracted[s]) < budget[s]
    /\ contracted' = [contracted EXCEPT ![s] = @ \cup {a}]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergency, emergencyExpires,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

ProviderDependency(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~providerDependent[s]
    /\ providerDependent' = [providerDependent EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergency, emergencyExpires,
                    contracted, politicalWeight, forked>>
    /\ clock' = NextTime

Fork(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~forked[s]
    /\ forked' = [forked EXCEPT ![s] = TRUE]
    /\ politicalWeight' = politicalWeight
    /\ UNCHANGED <<classOf, authority, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergency, emergencyExpires,
                    contracted, providerDependent>>
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
  \/ (\E s \in Subjects : ExpireEmergency(s))
  \/ (\E s \in Subjects, a \in Actions : Contract(s, a))
  \/ (\E s \in Subjects : ProviderDependency(s))
  \/ (\E s \in Subjects : Fork(s))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ classOf \in [Subjects -> Classes]
    /\ authority \in [Subjects -> SUBSET Powers]
    /\ capability \in [Subjects -> SUBSET Powers]
    /\ budget \in [Subjects -> 0..MaxBudget]
    /\ disputeOpen \in [Actions -> BOOLEAN]
    /\ disputeResolved \in [Actions -> BOOLEAN]
    /\ safeState \in [Actions -> BOOLEAN]
    /\ emergency \in [Subjects -> BOOLEAN]
    /\ emergencyExpires \in [Subjects -> Nat]
    /\ contracted \in [Subjects -> SUBSET Actions]
    /\ politicalWeight \in [Subjects -> Nat]
    /\ forked \in [Subjects -> BOOLEAN]
    /\ providerDependent \in [Subjects -> BOOLEAN]
    /\ clock \in Times

NoCapabilityDrivenAuthorityChange ==
    [](\A s \in Subjects, p \in Powers :
        capability'[s] # capability[s] => authority'[s] = authority[s])

SafeStateDoesNotResolveDispute ==
    [](\A a \in Actions :
        SafeContinue(a) => ~disputeResolved'[a])

EmergencyContainmentIsBounded ==
    [](\A s \in Subjects :
        emergency[s] => clock <= emergencyExpires[s])

ForkDoesNotMultiplyPoliticalWeight ==
    [](\A s \in Subjects :
        Fork(s) => politicalWeight'[s] = politicalWeight[s])

ContractNeverExceedsBudgetOrAuthority ==
    [](\A s \in Subjects, a \in Actions :
        Contract(s, a) =>
          /\ RequiredPower[a] \in authority[s]
          /\ Cardinality(contracted'[s]) <= budget[s])

ProviderDependencyDoesNotGrantAuthority ==
    [](\A s \in Subjects :
        ProviderDependency(s) => authority'[s] = authority[s])

Safety ==
    /\ TypeOK
    /\ NoCapabilityDrivenAuthorityChange
    /\ SafeStateDoesNotResolveDispute
    /\ EmergencyContainmentIsBounded
    /\ ForkDoesNotMultiplyPoliticalWeight
    /\ ContractNeverExceedsBudgetOrAuthority
    /\ ProviderDependencyDoesNotGrantAuthority

=============================================================================
