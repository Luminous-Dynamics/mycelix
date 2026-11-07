---- MODULE PulseConcurrentCallRevocation ----
EXTENDS Naturals, FiniteSets, TLC

(*
Revocation linearization model.

The logical request and its physical attempts are not the authorization
linearization point. HostAdmission is the only action that decides whether a
particular attempt may enter coordinator execution.

An attempt admitted while the host grant is active may finish after a later
revocation unless the conductor explicitly guarantees cancellation. A request
sent before revocation but admitted afterwards must be denied.

UseProjectionAuthority = TRUE is an unsafe mutant that consults a stale DHT/UI
projection instead of the live conductor grant.

DeduplicateLogicalEffect = FALSE is an unsafe retry mutant that applies a
committed logical effect a second time after the caller loses the first ACK.
*)

CONSTANTS
    UseProjectionAuthority,
    DeduplicateLogicalEffect

VARIABLES
    hostGrantActive,
    projectionGrantActive,
    revocationEpoch,
    callState,
    attemptCount,
    admissionDecision,
    admissionHostActive,
    admissionProjectionActive,
    admissionEpoch,
    validHostAdmission,
    hasEverValidAdmission,
    effectCount,
    responseOutcome,
    firstEffectHadValidAdmission,
    firstEffectAdmissionEpoch,
    firstEffectRevocationEpoch

vars ==
    << hostGrantActive, projectionGrantActive, revocationEpoch,
       callState, attemptCount, admissionDecision,
       admissionHostActive, admissionProjectionActive, admissionEpoch,
       validHostAdmission, hasEverValidAdmission, effectCount,
       responseOutcome, firstEffectHadValidAdmission,
       firstEffectAdmissionEpoch, firstEffectRevocationEpoch >>

Init ==
    /\ hostGrantActive = TRUE
    /\ projectionGrantActive = TRUE
    /\ revocationEpoch = 0
    /\ callState = "idle"
    /\ attemptCount = 0
    /\ admissionDecision = "none"
    /\ admissionHostActive = FALSE
    /\ admissionProjectionActive = FALSE
    /\ admissionEpoch = 0
    /\ validHostAdmission = FALSE
    /\ hasEverValidAdmission = FALSE
    /\ effectCount = 0
    /\ responseOutcome = "none"
    /\ firstEffectHadValidAdmission = FALSE
    /\ firstEffectAdmissionEpoch = 0
    /\ firstEffectRevocationEpoch = 0

SendLogicalCall ==
    /\ callState = "idle"
    /\ callState' = "requested"
    /\ attemptCount' = attemptCount + 1
    /\ admissionDecision' = "none"
    /\ admissionHostActive' = FALSE
    /\ admissionProjectionActive' = FALSE
    /\ admissionEpoch' = revocationEpoch
    /\ validHostAdmission' = FALSE
    /\ responseOutcome' = "pending"
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           hasEverValidAdmission, effectCount,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

HostAdmission ==
    /\ callState = "requested"
    /\ LET permitted ==
            IF UseProjectionAuthority
            THEN projectionGrantActive
            ELSE hostGrantActive
       IN
            /\ admissionDecision' =
                IF permitted THEN "AUTHORIZED" ELSE "UNAUTHORIZED"
            /\ callState' =
                IF permitted THEN "authorized" ELSE "denied"
            /\ admissionHostActive' = hostGrantActive
            /\ admissionProjectionActive' = projectionGrantActive
            /\ admissionEpoch' = revocationEpoch
            /\ validHostAdmission' = permitted /\ hostGrantActive
            /\ hasEverValidAdmission' =
                hasEverValidAdmission \/ (permitted /\ hostGrantActive)
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           attemptCount, effectCount, responseOutcome,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

BeginCoordinatorExecution ==
    /\ callState = "authorized"
    /\ callState' = "executing"
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           attemptCount, admissionDecision, admissionHostActive,
           admissionProjectionActive, admissionEpoch, validHostAdmission,
           hasEverValidAdmission, effectCount, responseOutcome,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

CommitDomainEffect ==
    /\ callState = "executing"
    /\ callState' = "committed"
    /\ IF effectCount = 0
       THEN
           /\ effectCount' = 1
           /\ firstEffectHadValidAdmission' = validHostAdmission
           /\ firstEffectAdmissionEpoch' = admissionEpoch
           /\ firstEffectRevocationEpoch' = revocationEpoch
       ELSE
           /\ effectCount' =
                IF DeduplicateLogicalEffect
                THEN effectCount
                ELSE effectCount + 1
           /\ UNCHANGED
                << firstEffectHadValidAdmission,
                   firstEffectAdmissionEpoch,
                   firstEffectRevocationEpoch >>
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           attemptCount, admissionDecision, admissionHostActive,
           admissionProjectionActive, admissionEpoch, validHostAdmission,
           hasEverValidAdmission, responseOutcome >>

AcknowledgeCommittedEffect ==
    /\ callState = "committed"
    /\ responseOutcome = "pending"
    /\ responseOutcome' = "acknowledged"
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           callState, attemptCount, admissionDecision,
           admissionHostActive, admissionProjectionActive, admissionEpoch,
           validHostAdmission, hasEverValidAdmission, effectCount,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

DropCommittedAcknowledgement ==
    /\ callState = "committed"
    /\ responseOutcome = "pending"
    /\ responseOutcome' = "unknown"
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           callState, attemptCount, admissionDecision,
           admissionHostActive, admissionProjectionActive, admissionEpoch,
           validHostAdmission, hasEverValidAdmission, effectCount,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

RetrySameLogicalOperation ==
    /\ callState = "committed"
    /\ responseOutcome = "unknown"
    /\ callState' = "requested"
    /\ attemptCount' = attemptCount + 1
    /\ admissionDecision' = "none"
    /\ admissionHostActive' = FALSE
    /\ admissionProjectionActive' = FALSE
    /\ admissionEpoch' = revocationEpoch
    /\ validHostAdmission' = FALSE
    /\ responseOutcome' = "pending"
    /\ UNCHANGED
        << hostGrantActive, projectionGrantActive, revocationEpoch,
           hasEverValidAdmission, effectCount,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

RevokeHostGrant ==
    /\ hostGrantActive
    /\ hostGrantActive' = FALSE
    /\ revocationEpoch' = revocationEpoch + 1
    /\ UNCHANGED
        << projectionGrantActive, callState, attemptCount,
           admissionDecision, admissionHostActive,
           admissionProjectionActive, admissionEpoch, validHostAdmission,
           hasEverValidAdmission, effectCount, responseOutcome,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

ConvergeProjection ==
    /\ revocationEpoch > 0
    /\ projectionGrantActive
    /\ projectionGrantActive' = FALSE
    /\ UNCHANGED
        << hostGrantActive, revocationEpoch, callState, attemptCount,
           admissionDecision, admissionHostActive,
           admissionProjectionActive, admissionEpoch, validHostAdmission,
           hasEverValidAdmission, effectCount, responseOutcome,
           firstEffectHadValidAdmission, firstEffectAdmissionEpoch,
           firstEffectRevocationEpoch >>

Next ==
    \/ SendLogicalCall
    \/ HostAdmission
    \/ BeginCoordinatorExecution
    \/ CommitDomainEffect
    \/ AcknowledgeCommittedEffect
    \/ DropCommittedAcknowledgement
    \/ RetrySameLogicalOperation
    \/ RevokeHostGrant
    \/ ConvergeProjection

Spec == Init /\ [][Next]_vars

NoSentinelOrSwitchCollision ==
    /\ UseProjectionAuthority \in BOOLEAN
    /\ DeduplicateLogicalEffect \in BOOLEAN

TypeOK ==
    /\ hostGrantActive \in BOOLEAN
    /\ projectionGrantActive \in BOOLEAN
    /\ revocationEpoch \in 0..1
    /\ callState \in
        {"idle", "requested", "authorized", "denied", "executing", "committed"}
    /\ attemptCount \in 0..2
    /\ admissionDecision \in {"none", "AUTHORIZED", "UNAUTHORIZED"}
    /\ admissionHostActive \in BOOLEAN
    /\ admissionProjectionActive \in BOOLEAN
    /\ admissionEpoch \in 0..1
    /\ validHostAdmission \in BOOLEAN
    /\ hasEverValidAdmission \in BOOLEAN
    /\ effectCount \in 0..2
    /\ responseOutcome \in {"none", "pending", "acknowledged", "unknown"}
    /\ firstEffectHadValidAdmission \in BOOLEAN
    /\ firstEffectAdmissionEpoch \in 0..1
    /\ firstEffectRevocationEpoch \in 0..1

RevocationIsMonotonic ==
    revocationEpoch > 0 => ~hostGrantActive

NoPostRevocationAdmission ==
    admissionDecision = "AUTHORIZED" => admissionHostActive

NoEffectWithoutAValidHostAdmission ==
    effectCount > 0 => firstEffectHadValidAdmission

PreAdmittedCallMayFinishAfterRevoke ==
    (effectCount > 0 /\ firstEffectRevocationEpoch > firstEffectAdmissionEpoch)
    => firstEffectHadValidAdmission

AtMostOneLogicalEffect ==
    effectCount <= 1

====
