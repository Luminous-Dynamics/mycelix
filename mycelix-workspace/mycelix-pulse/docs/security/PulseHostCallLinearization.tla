---- MODULE PulseHostCallLinearization ----
EXTENDS Naturals, FiniteSets, TLC

(*
Finite model of a protected host call racing with capability revocation.

Linearization points are explicit protocol events:
  - HostCallDecision: conductor evaluates the exact current CapGrant.
  - CommitHostRevocation: conductor commits deletion of that exact grant.

A call may have been invoked before revocation and still be denied if its
conductor decision occurs after the revoke commit. Conversely, a call whose
host decision occurs before the revoke commit may complete afterward.

No wall-clock ordering is used. `seq` records the model's
abstract durable event order.

DropCallAcknowledgement and RetryCall model a committed call whose response
was lost. The retry uses the same logical operation and must recover the
original durable decision/effect, not perform a second protected effect.

UseProjectionAuthority = TRUE is an unsafe mutant: it decides host authority
from a stale application projection, rather than the live host CapGrant.
*)

CONSTANTS
    grantee,
    outsider,
    NoAgent,
    UseProjectionAuthority,
    MaxAttempts

Agents == {grantee, outsider}

VARIABLES
    hostGrantActive,
    projectionActive,
    seq,
    callState,
    callCaller,
    callDecision,
    callDecisionSeq,
    callGrantActiveAtDecision,
    callProjectionActiveAtDecision,
    callInvokeSeq,
    callOutcome,
    callEffectCount,
    callAttempts,
    revokeState,
    revokeInvokeSeq,
    revokeCommitSeq,
    revokeCommitCount,
    revokeOutcome,
    revokeAttempts

vars ==
    << hostGrantActive, projectionActive, seq,
       callState, callCaller, callDecision, callDecisionSeq,
       callGrantActiveAtDecision, callProjectionActiveAtDecision,
       callInvokeSeq, callOutcome, callEffectCount, callAttempts,
       revokeState, revokeInvokeSeq, revokeCommitSeq,
       revokeCommitCount, revokeOutcome, revokeAttempts >>

Init ==
    /\ hostGrantActive = TRUE
    /\ projectionActive = TRUE
    /\ seq = 0
    /\ callState = "idle"
    /\ callCaller = NoAgent
    /\ callDecision = "none"
    /\ callDecisionSeq = 0
    /\ callGrantActiveAtDecision = FALSE
    /\ callProjectionActiveAtDecision = FALSE
    /\ callInvokeSeq = 0
    /\ callOutcome = "none"
    /\ callEffectCount = 0
    /\ callAttempts = 0
    /\ revokeState = "idle"
    /\ revokeInvokeSeq = 0
    /\ revokeCommitSeq = 0
    /\ revokeCommitCount = 0
    /\ revokeOutcome = "none"
    /\ revokeAttempts = 0

CallInvoke ==
    /\ callState = "idle"
    /\ callCaller' = grantee
    /\ callState' = "invoked"
    /\ seq' = seq + 1
    /\ callInvokeSeq' = seq + 1
    /\ callOutcome' = "pending"
    /\ callAttempts' = 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callEffectCount,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

RevokeInvoke ==
    /\ revokeState = "idle"
    /\ revokeState' = "invoked"
    /\ seq' = seq + 1
    /\ revokeInvokeSeq' = seq + 1
    /\ revokeOutcome' = "pending"
    /\ revokeAttempts' = 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeCommitSeq, revokeCommitCount >>

HostCallDecision ==
    /\ callState = "invoked"
    /\ callCaller = grantee
    /\ LET hostAllows ==
               IF UseProjectionAuthority
               THEN projectionActive
               ELSE hostGrantActive
       IN
           /\ callDecision' =
                IF hostAllows THEN "AUTHORIZED" ELSE "UNAUTHORIZED"
           /\ callDecisionSeq' = seq + 1
           /\ callGrantActiveAtDecision' = hostGrantActive
           /\ callProjectionActiveAtDecision' = projectionActive
           /\ callState' = "decided"
           /\ seq' = seq + 1
           /\ callEffectCount' =
                IF hostAllows THEN callEffectCount + 1 ELSE callEffectCount
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callCaller, callInvokeSeq, callOutcome, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

CommitHostRevocation ==
    /\ revokeState = "invoked"
    /\ hostGrantActive
    /\ hostGrantActive' = FALSE
    /\ revokeState' = "committed"
    /\ revokeCommitSeq' = seq + 1
    /\ revokeCommitCount' = revokeCommitCount + 1
    /\ seq' = seq + 1
    /\ UNCHANGED
        << projectionActive,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeInvokeSeq, revokeOutcome, revokeAttempts >>

ConvergeProjection ==
    /\ ~hostGrantActive
    /\ projectionActive
    /\ projectionActive' = FALSE
    /\ UNCHANGED
        << hostGrantActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

DropCallAcknowledgement ==
    /\ callState = "decided"
    /\ callOutcome = "pending"
    /\ callOutcome' = "unknown"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

RetrySameLogicalCall ==
    /\ callState = "decided"
    /\ callOutcome = "unknown"
    /\ callAttempts < MaxAttempts
    /\ callAttempts' = callAttempts + 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

RecoverCallOutcome ==
    /\ callState = "decided"
    /\ callOutcome = "unknown"
    /\ callOutcome' =
        IF callDecision = "AUTHORIZED" THEN "authorized" ELSE "denied"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

DropRevocationAcknowledgement ==
    /\ revokeState = "committed"
    /\ revokeOutcome = "pending"
    /\ revokeOutcome' = "unknown"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeAttempts >>

RetrySameLogicalRevocation ==
    /\ revokeState = "committed"
    /\ revokeOutcome = "unknown"
    /\ revokeAttempts < MaxAttempts
    /\ revokeAttempts' = revokeAttempts + 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome >>

RecoverRevocationOutcome ==
    /\ revokeState = "committed"
    /\ revokeOutcome = "unknown"
    /\ revokeOutcome' = "acknowledged"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callOutcome, callEffectCount, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeAttempts >>

Next ==
    \/ CallInvoke
    \/ RevokeInvoke
    \/ HostCallDecision
    \/ CommitHostRevocation
    \/ ConvergeProjection
    \/ DropCallAcknowledgement
    \/ RetrySameLogicalCall
    \/ RecoverCallOutcome
    \/ DropRevocationAcknowledgement
    \/ RetrySameLogicalRevocation
    \/ RecoverRevocationOutcome

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ hostGrantActive \in BOOLEAN
    /\ projectionActive \in BOOLEAN
    /\ seq \in Nat
    /\ callState \in {"idle", "invoked", "decided"}
    /\ callCaller \in Agents \cup {NoAgent}
    /\ callDecision \in {"none", "AUTHORIZED", "UNAUTHORIZED"}
    /\ callDecisionSeq \in Nat
    /\ callGrantActiveAtDecision \in BOOLEAN
    /\ callProjectionActiveAtDecision \in BOOLEAN
    /\ callInvokeSeq \in Nat
    /\ callOutcome \in {"none", "pending", "unknown", "authorized", "denied"}
    /\ callEffectCount \in 0..1
    /\ callAttempts \in 0..MaxAttempts
    /\ revokeState \in {"idle", "invoked", "committed"}
    /\ revokeInvokeSeq \in Nat
    /\ revokeCommitSeq \in Nat
    /\ revokeCommitCount \in 0..1
    /\ revokeOutcome \in {"none", "pending", "unknown", "acknowledged"}
    /\ revokeAttempts \in 0..MaxAttempts

CallDecisionIsLinearized ==
    callDecision = "AUTHORIZED"
    =>
        /\ callGrantActiveAtDecision
        /\ (revokeCommitSeq = 0 \/ callDecisionSeq < revokeCommitSeq)

NoPostRevokeAuthorization ==
    callDecision = "AUTHORIZED"
    =>
        (revokeCommitSeq = 0 \/ callDecisionSeq < revokeCommitSeq)

NoEffectWithoutHostGrant ==
    callEffectCount = 1
    =>
        /\ callDecision = "AUTHORIZED"
        /\ callGrantActiveAtDecision

OneEffectPerLogicalCall ==
    callEffectCount <= 1

OneCommitPerLogicalRevoke ==
    revokeCommitCount <= 1

RevokeIntentIsNotCommit ==
    revokeState = "invoked" => hostGrantActive

RetryDoesNotReexecuteCall ==
    callAttempts > 1
    =>
        /\ callDecision # "none"
        /\ callEffectCount \in 0..1

RetryDoesNotRecommitRevocation ==
    revokeAttempts > 1
    =>
        /\ revokeCommitCount = 1
        /\ ~hostGrantActive

RecoveredCallMatchesDurableDecision ==
    /\ callOutcome = "authorized" => callDecision = "AUTHORIZED"
    /\ callOutcome = "denied" => callDecision = "UNAUTHORIZED"

UnknownIsNotAnAuthorizationDecision ==
    callOutcome = "unknown"
    =>
        callDecision \in {"AUTHORIZED", "UNAUTHORIZED"}

HostDecisionNotProjectionDerived ==
    callDecision = "AUTHORIZED" => callGrantActiveAtDecision

====
