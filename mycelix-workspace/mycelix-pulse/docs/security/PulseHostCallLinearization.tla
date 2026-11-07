---- MODULE PulseHostCallLinearization ----
EXTENDS Naturals, FiniteSets, TLC

(*
Finite model of a protected host call racing with capability revocation.

The lifecycle has distinct protocol events:
  1. CallInvoke: the client sends a logical operation.
  2. HostCallDecision: the conductor evaluates the exact live CapGrant.
  3. BeginCallExecution: coordinator execution begins after admission.
  4. CommitCallEffect: the operation's durable effect is committed.
  5. AcknowledgeCall / DropCallAcknowledgement: the caller learns or loses
     the response.

CommitHostRevocation is the durable host grant deletion point.

A call invoked before revocation may still be denied if HostCallDecision
occurs after the revoke commit. A call admitted before revocation may complete
afterward; that is an already-authorized in-flight operation, not a new
post-revocation authorization.

No wall-clock ordering is used. `seq` records the abstract
protocol event order.

A lost ACK followed by RetrySameLogicalCall is a second physical attempt with
the same logical operation. It is re-authorized by the conductor and must not
create a second durable effect.

UseProjectionAuthority = TRUE is an unsafe mutant that derives the host
decision from a stale application projection.
DeduplicateLogicalCall = FALSE is an unsafe mutant that repeats a committed
effect on retry.
*)

CONSTANTS
    grantee,
    outsider,
    NoAgent,
    UseProjectionAuthority,
    DeduplicateLogicalCall,
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
    callExecutionSeq,
    callCommitSeq,
    callOutcome,
    callEffectCount,
    callAttempts,
    firstEffectGrantActiveAtDecision,
    firstEffectDecisionSeq,
    firstEffectCommitSeq,
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
       callInvokeSeq, callExecutionSeq, callCommitSeq,
       callOutcome, callEffectCount, callAttempts,
       firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
       firstEffectCommitSeq,
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
    /\ callExecutionSeq = 0
    /\ callCommitSeq = 0
    /\ callOutcome = "none"
    /\ callEffectCount = 0
    /\ callAttempts = 0
    /\ firstEffectGrantActiveAtDecision = FALSE
    /\ firstEffectDecisionSeq = 0
    /\ firstEffectCommitSeq = 0
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
    /\ callDecision' = "none"
    /\ callDecisionSeq' = 0
    /\ callGrantActiveAtDecision' = FALSE
    /\ callProjectionActiveAtDecision' = FALSE
    /\ callExecutionSeq' = 0
    /\ callCommitSeq' = 0
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callEffectCount, firstEffectGrantActiveAtDecision,
           firstEffectDecisionSeq, firstEffectCommitSeq,
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
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
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
           /\ callState' =
                IF hostAllows THEN "authorized" ELSE "denied"
           /\ callOutcome' =
                IF hostAllows THEN "pending" ELSE "denied"
           /\ seq' = seq + 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callCaller, callInvokeSeq, callExecutionSeq, callCommitSeq,
           callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

BeginCallExecution ==
    /\ callState = "authorized"
    /\ callState' = "executing"
    /\ seq' = seq + 1
    /\ callExecutionSeq' = seq + 1
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callCommitSeq, callOutcome,
           callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

CommitCallEffect ==
    /\ callState = "executing"
    /\ callState' = "committed"
    /\ seq' = seq + 1
    /\ callCommitSeq' = seq + 1
    /\ IF callEffectCount = 0
       THEN
           /\ callEffectCount' = 1
           /\ firstEffectGrantActiveAtDecision' = callGrantActiveAtDecision
           /\ firstEffectDecisionSeq' = callDecisionSeq
           /\ firstEffectCommitSeq' = seq + 1
       ELSE
           /\ callEffectCount' =
                IF DeduplicateLogicalCall
                THEN callEffectCount
                ELSE callEffectCount + 1
           /\ UNCHANGED
                << firstEffectGrantActiveAtDecision,
                   firstEffectDecisionSeq, firstEffectCommitSeq >>
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callExecutionSeq, callOutcome, callAttempts,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

AcknowledgeCall ==
    /\ callState = "committed"
    /\ callOutcome = "pending"
    /\ callDecision = "AUTHORIZED"
    /\ callOutcome' = "authorized"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

DropCallAcknowledgement ==
    /\ callState = "committed"
    /\ callOutcome = "pending"
    /\ callOutcome' = "unknown"
    /\ UNCHANGED
        << hostGrantActive, projectionActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeOutcome, revokeAttempts >>

RetrySameLogicalCall ==
    /\ callState = "committed"
    /\ callOutcome = "unknown"
    /\ callAttempts < MaxAttempts
    /\ callState' = "invoked"
    /\ callAttempts' = callAttempts + 1
    /\ seq' = seq + 1
    /\ callInvokeSeq' = seq + 1
    /\ callOutcome' = "pending"
    /\ callDecision' = "none"
    /\ callDecisionSeq' = 0
    /\ callGrantActiveAtDecision' = FALSE
    /\ callProjectionActiveAtDecision' = FALSE
    /\ callExecutionSeq' = 0
    /\ callCommitSeq' = 0
    /\ UNCHANGED
        << hostGrantActive, projectionActive,
           callCaller, callEffectCount,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
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
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeInvokeSeq, revokeOutcome, revokeAttempts >>

ConvergeProjection ==
    /\ ~hostGrantActive
    /\ projectionActive
    /\ projectionActive' = FALSE
    /\ UNCHANGED
        << hostGrantActive, seq,
           callState, callCaller, callDecision, callDecisionSeq,
           callGrantActiveAtDecision, callProjectionActiveAtDecision,
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
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
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
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
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
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
           callInvokeSeq, callExecutionSeq, callCommitSeq,
           callOutcome, callEffectCount, callAttempts,
           firstEffectGrantActiveAtDecision, firstEffectDecisionSeq,
           firstEffectCommitSeq,
           revokeState, revokeInvokeSeq, revokeCommitSeq,
           revokeCommitCount, revokeAttempts >>

Next ==
    \/ CallInvoke
    \/ RevokeInvoke
    \/ HostCallDecision
    \/ BeginCallExecution
    \/ CommitCallEffect
    \/ AcknowledgeCall
    \/ DropCallAcknowledgement
    \/ RetrySameLogicalCall
    \/ CommitHostRevocation
    \/ ConvergeProjection
    \/ DropRevocationAcknowledgement
    \/ RetrySameLogicalRevocation
    \/ RecoverRevocationOutcome

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ hostGrantActive \in BOOLEAN
    /\ projectionActive \in BOOLEAN
    /\ seq \in Nat
    /\ callState \in
        {"idle", "invoked", "authorized", "denied", "executing", "committed"}
    /\ callCaller \in Agents \cup {NoAgent}
    /\ callDecision \in {"none", "AUTHORIZED", "UNAUTHORIZED"}
    /\ callDecisionSeq \in Nat
    /\ callGrantActiveAtDecision \in BOOLEAN
    /\ callProjectionActiveAtDecision \in BOOLEAN
    /\ callInvokeSeq \in Nat
    /\ callExecutionSeq \in Nat
    /\ callCommitSeq \in Nat
    /\ callOutcome \in {"none", "pending", "unknown", "authorized", "denied"}
    /\ callEffectCount \in 0..2
    /\ callAttempts \in 0..MaxAttempts
    /\ firstEffectGrantActiveAtDecision \in BOOLEAN
    /\ firstEffectDecisionSeq \in Nat
    /\ firstEffectCommitSeq \in Nat
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
    callEffectCount > 0
    =>
        /\ firstEffectGrantActiveAtDecision
        /\ firstEffectDecisionSeq > 0
        /\ firstEffectCommitSeq > firstEffectDecisionSeq

PreAdmittedCallMayCommitAfterRevoke ==
    (callEffectCount > 0 /\ revokeCommitSeq > 0 /\ firstEffectCommitSeq > revokeCommitSeq)
    =>
        /\ firstEffectGrantActiveAtDecision
        /\ firstEffectDecisionSeq < revokeCommitSeq

OneEffectPerLogicalCall ==
    callEffectCount <= 1

OneCommitPerLogicalRevoke ==
    revokeCommitCount <= 1

RevokeIntentIsNotCommit ==
    revokeState = "invoked" => hostGrantActive

RetryDoesNotDuplicateCallEffect ==
    callAttempts > 1 => callEffectCount <= 1

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
