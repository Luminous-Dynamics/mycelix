---- MODULE PulseAuthorityCheckpoint ----
EXTENDS Naturals, FiniteSets, TLC

(*
Bounded causal-freshness model.

The conductor's host grant and the observer's DHT/UI projection are separate.
A host probe uses hostActive directly. An AuthorityCheckpoint records the
minimum authority generation a caller requires for an application read.

UseCheckpoint = TRUE is the intended reader:
  if the local projection has not reached the requested checkpoint, return
  UNKNOWN. Do not label stale-active state as fresh.

UseCheckpoint = FALSE is the deliberate freshness-bypass mutant:
  it labels the current projection authoritative even when it is older than
  the requested checkpoint.

The checkpoint is a read-freshness fence, not a bearer token and not a grant.
*)

CONSTANTS
    grantor, grantee, outsider, NoAgent,
    UseCheckpoint

Agents == {grantor, grantee, outsider}

VARIABLES
    hostActive,
    authorityEpoch,
    canonicalRevoked,
    projectionEpoch,
    projectionState,
    minimumEpoch,
    caller,
    hostProbe,
    lastHostActiveAtProbe,
    readState,
    lastReadEpoch,
    lastRequiredEpoch,
    lastCanonicalRevoked,
    lastReadUsedCheckpoint

vars ==
    << hostActive, authorityEpoch, canonicalRevoked,
       projectionEpoch, projectionState, minimumEpoch,
       caller, hostProbe, lastHostActiveAtProbe, readState, lastReadEpoch,
       lastRequiredEpoch, lastCanonicalRevoked,
       lastReadUsedCheckpoint >>

Init ==
    /\ hostActive = TRUE
    /\ authorityEpoch = 0
    /\ canonicalRevoked = FALSE
    /\ projectionEpoch = 0
    /\ projectionState = "active"
    /\ minimumEpoch = 0
    /\ caller = NoAgent
    /\ hostProbe = "none"
    /\ lastHostActiveAtProbe = FALSE
    /\ readState = "none"
    /\ lastReadEpoch = 0
    /\ lastRequiredEpoch = 0
    /\ lastCanonicalRevoked = FALSE
    /\ lastReadUsedCheckpoint = FALSE

CommitHostRevocation ==
    /\ hostActive
    /\ hostActive' = FALSE
    /\ authorityEpoch' = authorityEpoch + 1
    /\ canonicalRevoked' = TRUE
    /\ minimumEpoch' = authorityEpoch + 1
    /\ UNCHANGED
        << projectionEpoch, projectionState, caller, hostProbe,
           lastHostActiveAtProbe, readState, lastReadEpoch, lastRequiredEpoch,
           lastCanonicalRevoked, lastReadUsedCheckpoint >>

ConvergeProjection ==
    /\ projectionEpoch < authorityEpoch
    /\ projectionEpoch' = authorityEpoch
    /\ projectionState' =
        IF canonicalRevoked THEN "revoked" ELSE "active"
    /\ UNCHANGED
        << hostActive, authorityEpoch, canonicalRevoked, minimumEpoch,
           caller, hostProbe, lastHostActiveAtProbe, readState, lastReadEpoch,
           lastRequiredEpoch, lastCanonicalRevoked,
           lastReadUsedCheckpoint >>

ProbeHostAsGrantee ==
    /\ caller = NoAgent
    /\ caller' = grantee
    /\ hostProbe' =
        IF hostActive THEN "AUTHORIZED" ELSE "UNAUTHORIZED"
    /\ lastHostActiveAtProbe' = hostActive
    /\ UNCHANGED
        << hostActive, authorityEpoch, canonicalRevoked,
           projectionEpoch, projectionState, minimumEpoch,
           readState, lastReadEpoch, lastRequiredEpoch,
           lastCanonicalRevoked, lastReadUsedCheckpoint >>

ReadWithCheckpoint ==
    /\ caller = grantee
    /\ LET freshEnough == projectionEpoch >= minimumEpoch
           acceptedProjection ==
               IF projectionState = "active"
               THEN "TRUSTED_ACTIVE"
               ELSE "TRUSTED_REVOKED"
           decision ==
               IF UseCheckpoint /\ ~freshEnough
               THEN "UNKNOWN"
               ELSE acceptedProjection
       IN
           /\ readState' = decision
           /\ lastReadEpoch' = projectionEpoch
           /\ lastRequiredEpoch' = minimumEpoch
           /\ lastCanonicalRevoked' = canonicalRevoked
           /\ lastReadUsedCheckpoint' = UseCheckpoint
           /\ UNCHANGED
                << hostActive, authorityEpoch, canonicalRevoked,
                   projectionEpoch, projectionState, minimumEpoch,
                   caller, hostProbe, lastHostActiveAtProbe >>

Next ==
    \/ CommitHostRevocation
    \/ ConvergeProjection
    \/ ProbeHostAsGrantee
    \/ ReadWithCheckpoint

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ hostActive \in BOOLEAN
    /\ authorityEpoch \in Nat
    /\ canonicalRevoked \in BOOLEAN
    /\ projectionEpoch \in Nat
    /\ projectionState \in {"active", "revoked"}
    /\ minimumEpoch \in Nat
    /\ caller \in Agents \cup {NoAgent}
    /\ hostProbe \in {"none", "AUTHORIZED", "UNAUTHORIZED"}
    /\ lastHostActiveAtProbe \in BOOLEAN
    /\ readState \in {"none", "UNKNOWN", "TRUSTED_ACTIVE", "TRUSTED_REVOKED"}
    /\ lastReadEpoch \in Nat
    /\ lastRequiredEpoch \in Nat
    /\ lastCanonicalRevoked \in BOOLEAN
    /\ lastReadUsedCheckpoint \in BOOLEAN

NoFalseFreshAuthority ==
    /\ readState = "TRUSTED_ACTIVE"
       => /\ ~canonicalRevoked
          /\ hostActive
          /\ projectionEpoch >= minimumEpoch
          /\ projectionState = "active"
    /\ readState = "TRUSTED_REVOKED"
       => /\ canonicalRevoked
          /\ ~hostActive
          /\ projectionEpoch >= minimumEpoch
          /\ projectionState = "revoked"

HostProbeIsNotProjectionDerived ==
    hostProbe = "AUTHORIZED" => lastHostActiveAtProbe

CheckpointNeverGrantsHostAuthority ==
    /\ readState = "TRUSTED_ACTIVE"
       => hostActive
    /\ readState = "UNKNOWN" => TRUE

StaleCheckpointReadIsUnknown ==
    lastReadUsedCheckpoint /\ lastReadEpoch < lastRequiredEpoch
    => readState = "UNKNOWN"

====
