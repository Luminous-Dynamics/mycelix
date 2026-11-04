---- MODULE PulseAuthorityLifecycle ----
EXTENDS Naturals, FiniteSets, TLC

(*
Finite-state contract model for Pulse authorization lifecycle v1.

This is an abstract model, not a model of Holochain internals. The host
CapGrant state is modeled explicitly and separately from application records,
DHT/UI projections, recipient claims, source-chain effects, and caller
observations.

UseProjectionAuthority is a deliberate negative-control switch:
  FALSE = intended design; host decision uses the live host grant.
  TRUE  = unsafe mutant; host decision is inferred from the projection.

The mutant configuration should violate NoProjectionAuthority after a revoke
while the projection is still stale-active.
*)

CONSTANTS
    g1, g2,
    op1, op2,
    grantor, grantee, outsider,
    NoGrant, NoOp, NoAgent,
    DomainAllowedOps,
    UseProjectionAuthority,
    UseGrantIdempotency,
    MaxAttempts

Grants == {g1, g2}
Ops == {op1, op2}
Agents == {grantor, grantee, outsider}

ASSUME
    /\ NoGrant \notin Grants
    /\ NoOp \notin Ops
    /\ NoAgent \notin Agents
    /\ DomainAllowedOps \subseteq Ops
    /\ UseProjectionAuthority \in BOOLEAN
    /\ UseGrantIdempotency \in BOOLEAN
    /\ MaxAttempts \in Nat
    /\ MaxAttempts >= 2

VARIABLES
    hostActive,
    issued,
    retired,
    appState,
    projection,
    claims,
    operationGrant,
    operationOwner,
    attempts,
    callerOutcome,
    effectCount,
    hostAuthorizedOps,
    lastGrant,
    lastOperation,
    lastCaller,
    lastDecision,
    lastHostAuthorized,
    lastHostActiveAtCall,
    lastProjectionActiveAtCall,
    lastClaimAtCall,
    lastDomainAllowedAtCall,
    lastCallMadeEffect

vars ==
    << hostActive, issued, retired, appState, projection, claims,
       operationGrant, operationOwner, attempts, callerOutcome,
       effectCount, hostAuthorizedOps, lastGrant, lastOperation,
       lastCaller, lastDecision, lastHostAuthorized,
       lastHostActiveAtCall, lastProjectionActiveAtCall,
       lastClaimAtCall, lastDomainAllowedAtCall, lastCallMadeEffect >>

Init ==
    /\ hostActive = {}
    /\ issued = {}
    /\ retired = {}
    /\ appState = [g \in Grants |-> "absent"]
    /\ projection = [g \in Grants |-> "unknown"]
    /\ claims = {}
    /\ operationGrant = [op \in Ops |-> NoGrant]
    /\ operationOwner = [g \in Grants |-> NoOp]
    /\ attempts = [op \in Ops |-> 0]
    /\ callerOutcome = [op \in Ops |-> "none"]
    /\ effectCount = [op \in Ops |-> 0]
    /\ hostAuthorizedOps = {}
    /\ lastGrant = NoGrant
    /\ lastOperation = NoOp
    /\ lastCaller = NoAgent
    /\ lastDecision = "none"
    /\ lastHostAuthorized = FALSE
    /\ lastHostActiveAtCall = FALSE
    /\ lastProjectionActiveAtCall = FALSE
    /\ lastClaimAtCall = FALSE
    /\ lastDomainAllowedAtCall = FALSE
    /\ lastCallMadeEffect = FALSE

IssueGrant(g, op) ==
    /\ g \in Grants
    /\ op \in Ops
    /\ g \notin issued
    /\ operationGrant[op] = NoGrant
    /\ operationOwner[g] = NoOp
    /\ hostActive' = hostActive \cup {g}
    /\ issued' = issued \cup {g}
    /\ appState' = [appState EXCEPT ![g] = "active"]
    /\ projection' = [projection EXCEPT ![g] = "active"]
    /\ operationGrant' = [operationGrant EXCEPT ![op] = g]
    /\ operationOwner' = [operationOwner EXCEPT ![g] = op]
    /\ attempts' = [attempts EXCEPT ![op] = 1]
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "acknowledged"]
    /\ UNCHANGED
        << retired, claims, effectCount, hostAuthorizedOps,
           lastGrant, lastOperation, lastCaller, lastDecision,
           lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

RetryGrant(op) ==
    /\ op \in Ops
    /\ operationGrant[op] \in Grants
    /\ attempts[op] < MaxAttempts
    /\ attempts' = [attempts EXCEPT ![op] = @ + 1]
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "acknowledged"]
    /\ UNCHANGED
        << hostActive, issued, retired, appState, projection, claims,
           operationGrant, operationOwner, effectCount, hostAuthorizedOps,
           lastGrant, lastOperation, lastCaller, lastDecision,
           lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

(* Deliberately unsafe mutant: a lost acknowledgement retry mints a second
   CapGrant for the same logical request. The original grant stays live. *)
DuplicateGrantForRetry(g, op) ==
    /\ UseGrantIdempotency = FALSE
    /\ g \in Grants
    /\ op \in Ops
    /\ operationGrant[op] \in Grants
    /\ callerOutcome[op] = "unknown"
    /\ g \notin issued
    /\ operationOwner[g] = NoOp
    /\ attempts[op] < MaxAttempts
    /\ hostActive' = hostActive \cup {g}
    /\ issued' = issued \cup {g}
    /\ appState' = [appState EXCEPT ![g] = "active"]
    /\ projection' = [projection EXCEPT ![g] = "active"]
    /\ operationGrant' = [operationGrant EXCEPT ![op] = g]
    /\ operationOwner' = [operationOwner EXCEPT ![g] = op]
    /\ attempts' = [attempts EXCEPT ![op] = @ + 1]
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "acknowledged"]
    /\ UNCHANGED
        << retired, claims, effectCount, hostAuthorizedOps,
           lastGrant, lastOperation, lastCaller, lastDecision,
           lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

DropGrantAck(op) ==
    /\ op \in Ops
    /\ operationGrant[op] \in Grants
    /\ callerOutcome[op] = "acknowledged"
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "unknown"]
    /\ UNCHANGED
        << hostActive, issued, retired, appState, projection, claims,
           operationGrant, operationOwner, attempts, effectCount,
           hostAuthorizedOps, lastGrant, lastOperation, lastCaller,
           lastDecision, lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

DeliverClaim(g) ==
    /\ g \in hostActive
    /\ g \notin claims
    /\ claims' = claims \cup {g}
    /\ UNCHANGED
        << hostActive, issued, retired, appState, projection,
           operationGrant, operationOwner, attempts, callerOutcome,
           effectCount, hostAuthorizedOps, lastGrant, lastOperation,
           lastCaller, lastDecision, lastHostAuthorized,
           lastHostActiveAtCall, lastProjectionActiveAtCall,
           lastClaimAtCall, lastDomainAllowedAtCall, lastCallMadeEffect >>

RevokeGrant(g) ==
    /\ g \in hostActive
    /\ hostActive' = hostActive \ {g}
    /\ retired' = retired \cup {g}
    /\ appState' = [appState EXCEPT ![g] = "revoked"]
    /\ UNCHANGED
        << issued, projection, claims, operationGrant, operationOwner,
           attempts, callerOutcome, effectCount, hostAuthorizedOps,
           lastGrant, lastOperation, lastCaller, lastDecision,
           lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

ConvergeProjection(g) ==
    /\ g \in retired
    /\ projection[g] = "active"
    /\ projection' = [projection EXCEPT ![g] = "revoked"]
    /\ UNCHANGED
        << hostActive, issued, retired, appState, claims,
           operationGrant, operationOwner, attempts, callerOutcome,
           effectCount, hostAuthorizedOps, lastGrant, lastOperation,
           lastCaller, lastDecision, lastHostAuthorized,
           lastHostActiveAtCall, lastProjectionActiveAtCall,
           lastClaimAtCall, lastDomainAllowedAtCall, lastCallMadeEffect >>

Call(g, actor, op) ==
    /\ g \in issued
    /\ actor \in Agents
    /\ op \in Ops
    /\ operationGrant[op] = g
    /\ LET hostOK ==
            IF UseProjectionAuthority
            THEN projection[g] = "active"
                 /\ g \in claims
                 /\ actor = grantee
            ELSE g \in hostActive
                 /\ g \in claims
                 /\ actor = grantee
       IN
          /\ lastGrant' = g
          /\ lastOperation' = op
          /\ lastCaller' = actor
          /\ lastHostAuthorized' = hostOK
          /\ lastHostActiveAtCall' = g \in hostActive
          /\ lastProjectionActiveAtCall' = projection[g] = "active"
          /\ lastClaimAtCall' = g \in claims
          /\ lastDomainAllowedAtCall' = op \in DomainAllowedOps
          /\ lastDecision' =
                IF ~hostOK
                THEN "host-denied"
                ELSE IF op \in DomainAllowedOps
                     THEN "authorized"
                     ELSE "domain-rejected"
          /\ lastCallMadeEffect' =
                hostOK
                /\ op \in DomainAllowedOps
                /\ effectCount[op] = 0
          /\ hostAuthorizedOps' =
                IF hostOK THEN hostAuthorizedOps \cup {op}
                ELSE hostAuthorizedOps
          /\ effectCount' =
                IF hostOK /\ op \in DomainAllowedOps /\ effectCount[op] = 0
                THEN [effectCount EXCEPT ![op] = 1]
                ELSE effectCount
          /\ callerOutcome' =
                [callerOutcome EXCEPT ![op] = "acknowledged"]
          /\ UNCHANGED
                << hostActive, issued, retired, appState, projection,
                   claims, operationGrant, operationOwner, attempts >>

Timeout(op) ==
    /\ op \in Ops
    /\ callerOutcome[op] = "acknowledged"
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "unknown"]
    /\ UNCHANGED
        << hostActive, issued, retired, appState, projection, claims,
           operationGrant, operationOwner, attempts, effectCount,
           hostAuthorizedOps, lastGrant, lastOperation, lastCaller,
           lastDecision, lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

ReconcileOutcome(op) ==
    /\ op \in Ops
    /\ callerOutcome[op] = "unknown"
    /\ callerOutcome' = [callerOutcome EXCEPT ![op] = "acknowledged"]
    /\ UNCHANGED
        << hostActive, issued, retired, appState, projection, claims,
           operationGrant, operationOwner, attempts, effectCount,
           hostAuthorizedOps, lastGrant, lastOperation, lastCaller,
           lastDecision, lastHostAuthorized, lastHostActiveAtCall,
           lastProjectionActiveAtCall, lastClaimAtCall,
           lastDomainAllowedAtCall, lastCallMadeEffect >>

Next ==
    \/ \E g \in Grants, op \in Ops : IssueGrant(g, op)
    \/ \E op \in Ops : RetryGrant(op)
    \/ \E g \in Grants, op \in Ops : DuplicateGrantForRetry(g, op)
    \/ \E op \in Ops : DropGrantAck(op)
    \/ \E g \in Grants : DeliverClaim(g)
    \/ \E g \in Grants : RevokeGrant(g)
    \/ \E g \in Grants : ConvergeProjection(g)
    \/ \E g \in Grants, actor \in Agents, op \in Ops : Call(g, actor, op)
    \/ \E op \in Ops : Timeout(op)
    \/ \E op \in Ops : ReconcileOutcome(op)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ hostActive \subseteq Grants
    /\ issued \subseteq Grants
    /\ retired \subseteq Grants
    /\ claims \subseteq issued
    /\ hostAuthorizedOps \subseteq Ops
    /\ appState \in [Grants -> {"absent", "active", "revoked"}]
    /\ projection \in [Grants -> {"unknown", "active", "revoked"}]
    /\ operationGrant \in [Ops -> (Grants \cup {NoGrant})]
    /\ operationOwner \in [Grants -> (Ops \cup {NoOp})]
    /\ attempts \in [Ops -> 0..MaxAttempts]
    /\ callerOutcome \in [Ops -> {"none", "unknown", "acknowledged"}]
    /\ effectCount \in [Ops -> 0..1]
    /\ lastGrant \in Grants \cup {NoGrant}
    /\ lastOperation \in Ops \cup {NoOp}
    /\ lastCaller \in Agents \cup {NoAgent}
    /\ lastDecision \in {"none", "host-denied", "domain-rejected", "authorized"}
    /\ lastHostAuthorized \in BOOLEAN
    /\ lastHostActiveAtCall \in BOOLEAN
    /\ lastProjectionActiveAtCall \in BOOLEAN
    /\ lastClaimAtCall \in BOOLEAN
    /\ lastDomainAllowedAtCall \in BOOLEAN
    /\ lastCallMadeEffect \in BOOLEAN

NoProjectionAuthority ==
    lastHostAuthorized
    => (lastHostActiveAtCall /\ lastClaimAtCall /\ lastCaller = grantee)

StaleProjectionDoesNotAllow ==
    lastProjectionActiveAtCall /\ ~lastHostActiveAtCall
    => ~lastHostAuthorized

NoEffectWithoutHostAuthorization ==
    /\ \A op \in Ops :
        effectCount[op] = 1 => op \in hostAuthorizedOps
    /\ lastCallMadeEffect =>
        lastHostAuthorized
        /\ lastHostActiveAtCall
        /\ lastClaimAtCall
        /\ lastDomainAllowedAtCall

AtMostOneEffectPerLogicalOperation ==
    \A op \in Ops : effectCount[op] <= 1

RevocationMonotonic ==
    /\ retired \subseteq issued
    /\ retired \cap hostActive = {}
    /\ \A g \in retired : appState[g] = "revoked"

GrantIdentityInjective ==
    /\ \A g \in issued : operationOwner[g] \in Ops
    /\ \A g \in issued : operationGrant[operationOwner[g]] = g
    /\ \A op \in Ops :
        operationGrant[op] \in Grants
        => operationOwner[operationGrant[op]] = op

ClaimsBoundToIssuedGrant ==
    claims \subseteq issued

====
