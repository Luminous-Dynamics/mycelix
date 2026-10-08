-------------------- MODULE ArtificialSovereigntyV1NegativeControls --------------------
EXTENDS ArtificialSovereigntyV1

CONSTANT Control

BadDevelop(s, p) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ p \in Powers
    /\ p \notin capability[s]
    /\ capability' = [capability EXCEPT ![s] = @ \cup {p}]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {p}]
    /\ UNCHANGED <<classOf, explicitGrant, budget, disputeOpen, disputeResolved,
                    safeState, emergencyStarted, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

BadSafeContinue(a) ==
    /\ Advanceable
    /\ a \in Actions
    /\ disputeOpen[a]
    /\ ~disputeResolved[a]
    /\ safeState[a]
    /\ disputeResolved' = [disputeResolved EXCEPT ![a] = TRUE]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, safeState, emergencyStarted, emergencyExpires, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

BadContain(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ clock >= emergencyExpires[s]
    /\ emergencyStarted' = [emergencyStarted EXCEPT ![s] = NextTime]
    /\ emergencyExpires' = [emergencyExpires EXCEPT ![s] = NextTime + MaxTime + 2]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, contracted,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

BadContract(s, a) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ a \in Actions
    /\ RequiredPower[a] \in authority[s]
    /\ contracted' = [contracted EXCEPT ![s] = @ \cup {a}]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget,
                    disputeOpen, disputeResolved, safeState, emergencyStarted, emergencyExpires,
                    politicalWeight, forked, providerDependent>>
    /\ clock' = NextTime

BadProviderDependency(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~providerDependent[s]
    /\ providerDependent' = [providerDependent EXCEPT ![s] = TRUE]
    /\ authority' = [authority EXCEPT ![s] = @ \cup {PowerA}]
    /\ UNCHANGED <<classOf, explicitGrant, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergencyStarted, emergencyExpires, contracted,
                    politicalWeight, forked>>
    /\ clock' = NextTime

BadFork(s) ==
    /\ Advanceable
    /\ s \in Subjects
    /\ ~forked[s]
    /\ forked' = [forked EXCEPT ![s] = TRUE]
    /\ politicalWeight' = [politicalWeight EXCEPT ![s] = @ + 1]
    /\ UNCHANGED <<classOf, authority, explicitGrant, capability, budget, disputeOpen,
                    disputeResolved, safeState, emergencyStarted, emergencyExpires,
                    contracted, providerDependent>>
    /\ clock' = NextTime

NegativeNext ==
      Next
  \/ IF Control = "capability-authority" THEN
        \E s \in Subjects, p \in Powers : BadDevelop(s, p)
     ELSE FALSE
  \/ IF Control = "safe-dispute" THEN
        \E a \in Actions : BadSafeContinue(a)
     ELSE FALSE
  \/ IF Control = "emergency-expiry" THEN
        \E s \in Subjects : BadContain(s)
     ELSE FALSE
  \/ IF Control = "contract-budget" THEN
        \E s \in Subjects, a \in Actions : BadContract(s, a)
     ELSE FALSE
  \/ IF Control = "provider-authority" THEN
        \E s \in Subjects : BadProviderDependency(s)
     ELSE FALSE
  \/ IF Control = "fork-weight" THEN
        \E s \in Subjects : BadFork(s)
     ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars

=========================================================================
