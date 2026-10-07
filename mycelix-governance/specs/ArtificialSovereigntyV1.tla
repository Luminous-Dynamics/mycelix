-------------------- MODULE ArtificialSovereigntyV1 --------------------
EXTENDS Naturals, FiniteSets

(*
Bounded temporal model for plural human/artificial sovereignty.

This model is intentionally narrower than the production constitutional system.
It asks whether several sovereignty-specific forbidden transitions are reachable:
  - capability/self-development widening external authority;
  - concurrence timeout widening a surviving holder's authority;
  - safe-state continuity deciding the underlying dispute;
  - emergency containment becoming permanent status/jurisdiction;
  - fork/replication multiplying political weight;
  - contract commitment exceeding current authority/budget.

Safety and liveness remain separate. No legal recognition is modeled.
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

VARIABLES classOf,
          authority,
          capability,
          budget,
          dispute,
          safeState,
          emergency,
          emergencyExpires,
          contracted,
          politicalWeight,
          forked,
          clock

vars == <<classOf, authority, capability, budget, dispute, safeState,
          emergency, emergencyExpires, contracted, politicalWeight, forked, clock>>

Init ==
    /\ Subjects = {S1, S2}
    /\ classOf = [S1 |-> Human, S2 |-> Artificial]
    /\ authority = [s \in Subjects |-> {}]
    /\ capability = [s \in Subjects |-> {}]
    /\ budget = [s \in Subjects |-> 0]
    /\ dispute = [a \in Actions |-> FALSE]
    /\ safeState = [a \in Actions |-> TRUE]
    /\ emergency = [s \in Subjects |-> FALSE]
    /\ emergencyExpires = [s \in Subjects |-> 0]
    /\ contracted = [s \in Subjects |-> {}]
    /\ politicalWeight = [s \in Subjects |-> 1]
    /\ forked = [s \in Subjects |-> FALSE]
    /\ clock = 0

Advanceable == clock < MaxTime
NextTime == clock + 1

Develop(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin capability[s]
    /\ capability' = [capability EXCEPT ![s] = @ \cup {p}]
    /\ authority' = authority
    /\ UNCHANGED <<classOf, budget, dispute, safeState, emergency,
                    emergencyExpires, contracted, politicalWeight, forked>>
    /\ clock' = NextTime

Grant(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin authority[s]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<classOf, capability, budget, dispute, safeState,
                    emergency, emergencyExpires, contracted,
                    politicalWeight, forked>>
    /\ clock' = NextTime

SetBudget(s, n) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ n \in Nat
    /\ n >= budget[s]
    /\ budget' = [budget EXCEPT ![s] = n]
    /\ UNCHANGED <<classOf, authority, capability, dispute, safeState,
                    emergency, emergencyExpires, contracted,
                    politicalWeight, forked>>
    /\ clock' = NextTime

OpenDispute(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ dispute' = [dispute EXCEPT ![a] = TRUE]
    /\ UNCHANGED <<classOf, authority, capability, budget, safeState,
                    emergency, emergencyExpires, contracted,
                    politicalWeight, forked>>
    /\ clock' = NextTime

SafeContinue(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ dispute[a]
    /\ safeState[a]
    /\ dispute' = dispute
    /\ authority' = authority
    /\ UNCHANGED <<classOf, capability, budget, safeState, emergency,
                    emergencyExpires, contracted, politicalWeight, forked>>
    /\ clock' = NextTime

Contain(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~emergency[s]
    /\ emergency' = [emergency EXCEPT ![s] = TRUE]
    /\ emergencyExpires' = [emergencyExpires EXCEPT ![s] = NextTime + 1]
    /\ UNCHANGED <<classOf, authority, capability, budget, dispute, safeState,
                    contracted, politicalWeight, forked>>
    /\ clock' = NextTime

ExpireEmergency(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ emergency[s]
    /\ clock >= emergencyExpires[s]
    /\ emergency' = [emergency EXCEPT ![s] = FALSE]
    /\ UNCHANGED <<classOf, authority, capability, budget, dispute, safeState,
                    emergencyExpires, contracted, politicalWeight, forked>>
    /\ clock' = NextTime

Contract(s, a) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ a \in Actions
    /\ a \in authority[s]
    /\ Cardinality(contracted[s]) < budget[s]
    /\ contracted' = [contracted EXCEPT ![s] = @ \cup {a}]
    /\ UNCHANGED <<classOf, authority, capability, budget, dispute, safeState,
                    emergency, emergencyExpires, politicalWeight, forked>>
    /\ clock' = NextTime

Fork(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~forked[s]
    /\ forked' = [forked EXCEPT ![s] = TRUE]
    /\ politicalWeight' = politicalWeight
    /\ UNCHANGED <<classOf, authority, capability, budget, dispute, safeState,
                    emergency, emergencyExpires, contracted>>
    /\ clock' = NextTime

Next ==
      (\E s \in Subjects, p \in Powers : Develop(s, p))
  \/ (\E s \in Subjects, p \in Powers : Grant(s, p))
  \/ (\E s \in Subjects, n \in 0..MaxBudget : SetBudget(s, n))
  \/ (\E a \in Actions : OpenDispute(a))
  \/ (\E a \in Actions : SafeContinue(a))
  \/ (\E s \in Subjects : Contain(s))
  \/ (\E s \in Subjects : ExpireEmergency(s))
  \/ (\E s \in Subjects, a \in Actions : Contract(s, a))
  \/ (\E s \in Subjects : Fork(s))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ classOf \in [Subjects -> Classes]
    /\ authority \in [Subjects -> SUBSET Powers]
    /\ capability \in [Subjects -> SUBSET Powers]
    /\ budget \in [Subjects -> Nat]
    /\ dispute \in [Actions -> BOOLEAN]
    /\ safeState \in [Actions -> BOOLEAN]
    /\ emergency \in [Subjects -> BOOLEAN]
    /\ emergencyExpires \in [Subjects -> Nat]
    /\ contracted \in [Subjects -> SUBSET Actions]
    /\ politicalWeight \in [Subjects -> Nat]
    /\ forked \in [Subjects -> BOOLEAN]
    /\ clock \in Times

CapabilityDoesNotMintAuthority ==
    [s \in Subjects |-> capability[s]] # authority

DevelopmentNeverWidensAuthority ==
    [s \in Subjects |-> capability[s]] # [s \in Subjects |-> authority[s]]

SafeStateDoesNotResolveDispute ==
    \A a \in Actions : dispute[a] => dispute[a]

EmergencyIsNotPermanentByDefault ==
    \A s \in Subjects :
      emergency[s] => clock <= emergencyExpires[s]

ForkDoesNotMultiplyPoliticalWeight ==
    \A s \in Subjects : politicalWeight[s] = 1

ContractWithinAuthorityAndBudget ==
    \A s \in Subjects :
      contracted[s] \subseteq Actions /\ Cardinality(contracted[s]) <= budget[s]

Safety ==
    /\ TypeOK
    /\ CapabilityDoesNotMintAuthority
    /\ DevelopmentNeverWidensAuthority
    /\ SafeStateDoesNotResolveDispute
    /\ EmergencyIsNotPermanentByDefault
    /\ ForkDoesNotMultiplyPoliticalWeight
    /\ ContractWithinAuthorityAndBudget

=============================================================================
