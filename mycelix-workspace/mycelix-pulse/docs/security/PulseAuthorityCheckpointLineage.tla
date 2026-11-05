---- MODULE PulseAuthorityCheckpointLineage ----
EXTENDS Naturals, FiniteSets, TLC

(*
A scalar epoch is not a causal identity.

This model separates:
  - host CapGrant enforcement;
  - the canonical authority history (DNA/cell, author, ActionHash, epoch);
  - the checkpoint a caller requires;
  - the observer's projection history and proof that it descends from the checkpoint.

A foreign history may have the same numeric epoch as the required checkpoint.
A reader that compares only the integer can label stale/conflicting state fresh.

CheckLineage = TRUE is the intended reader: require matching DNA/cell and
author lineage plus a valid ancestry proof.

CheckLineage = FALSE is the deliberate lineage-bypass mutant. It compares
only epoch numbers and can accept a foreign active projection at the same
numeric height after revocation.

This is an abstract contract model, not a claim that Holochain exposes a
global DHT snapshot token or that one scalar can totally order all source
chains.
*)

CONSTANTS
    canonicalDna,
    foreignDna,
    grantor,
    grantee,
    NoAction,
    revokeAction,
    foreignAction,
    CheckLineage

VARIABLES
    hostActive,
    canonicalDnaId,
    canonicalAuthor,
    canonicalAction,
    canonicalEpoch,
    canonicalRevoked,
    requiredDna,
    requiredAuthor,
    requiredAction,
    requiredEpoch,
    projectionDna,
    projectionAuthor,
    projectionAction,
    projectionEpoch,
    projectionRevoked,
    projectionDescendsFromRequired,
    readDecision,
    lastReadObservedEpoch,
    lastReadRequiredEpoch,
    lastReadDnaMatches,
    lastReadAuthorMatches,
    lastReadAncestryProof,
    lastReadHostActive,
    lastReadCanonicalRevoked,
    lastReadProjectionRevoked

vars ==
    << hostActive,
       canonicalDnaId, canonicalAuthor, canonicalAction,
       canonicalEpoch, canonicalRevoked,
       requiredDna, requiredAuthor, requiredAction, requiredEpoch,
       projectionDna, projectionAuthor, projectionAction,
       projectionEpoch, projectionRevoked,
       projectionDescendsFromRequired,
       readDecision,
       lastReadObservedEpoch, lastReadRequiredEpoch,
       lastReadDnaMatches, lastReadAuthorMatches,
       lastReadAncestryProof, lastReadHostActive,
       lastReadCanonicalRevoked, lastReadProjectionRevoked >>

Init ==
    /\ hostActive = TRUE
    /\ canonicalDnaId = canonicalDna
    /\ canonicalAuthor = grantor
    /\ canonicalAction = NoAction
    /\ canonicalEpoch = 0
    /\ canonicalRevoked = FALSE
    /\ requiredDna = canonicalDna
    /\ requiredAuthor = grantor
    /\ requiredAction = NoAction
    /\ requiredEpoch = 0
    /\ projectionDna = canonicalDna
    /\ projectionAuthor = grantor
    /\ projectionAction = NoAction
    /\ projectionEpoch = 0
    /\ projectionRevoked = FALSE
    /\ projectionDescendsFromRequired = TRUE
    /\ readDecision = "none"
    /\ lastReadObservedEpoch = 0
    /\ lastReadRequiredEpoch = 0
    /\ lastReadDnaMatches = TRUE
    /\ lastReadAuthorMatches = TRUE
    /\ lastReadAncestryProof = TRUE
    /\ lastReadHostActive = TRUE
    /\ lastReadCanonicalRevoked = FALSE
    /\ lastReadProjectionRevoked = FALSE

CommitHostRevocation ==
    /\ hostActive
    /\ hostActive' = FALSE
    /\ canonicalEpoch' = canonicalEpoch + 1
    /\ canonicalRevoked' = TRUE
    /\ canonicalAction' = revokeAction
    /\ requiredDna' = canonicalDnaId
    /\ requiredAuthor' = canonicalAuthor
    /\ requiredAction' = revokeAction
    /\ requiredEpoch' = canonicalEpoch + 1
    /\ UNCHANGED
        << canonicalDnaId, canonicalAuthor,
           projectionDna, projectionAuthor, projectionAction,
           projectionEpoch, projectionRevoked,
           projectionDescendsFromRequired, readDecision,
           lastReadObservedEpoch, lastReadRequiredEpoch,
           lastReadDnaMatches, lastReadAuthorMatches,
           lastReadAncestryProof, lastReadHostActive,
           lastReadCanonicalRevoked, lastReadProjectionRevoked >>

ObserveForeignHistoryAtSameEpoch ==
    /\ canonicalRevoked
    /\ projectionEpoch < requiredEpoch
    /\ projectionDna' = foreignDna
    /\ projectionAuthor' = grantor
    /\ projectionAction' = foreignAction
    /\ projectionEpoch' = requiredEpoch
    /\ projectionRevoked' = FALSE
    /\ projectionDescendsFromRequired' = FALSE
    /\ UNCHANGED
        << hostActive, canonicalDnaId, canonicalAuthor,
           canonicalAction, canonicalEpoch, canonicalRevoked,
           requiredDna, requiredAuthor, requiredAction, requiredEpoch,
           readDecision, lastReadObservedEpoch, lastReadRequiredEpoch,
           lastReadDnaMatches, lastReadAuthorMatches,
           lastReadAncestryProof, lastReadHostActive,
           lastReadCanonicalRevoked, lastReadProjectionRevoked >>

ConvergeCanonicalProjection ==
    /\ projectionEpoch < canonicalEpoch
       \/ projectionDna # canonicalDnaId
       \/ projectionAuthor # canonicalAuthor
       \/ ~projectionDescendsFromRequired
    /\ projectionDna' = canonicalDnaId
    /\ projectionAuthor' = canonicalAuthor
    /\ projectionAction' = canonicalAction
    /\ projectionEpoch' = canonicalEpoch
    /\ projectionRevoked' = canonicalRevoked
    /\ projectionDescendsFromRequired' = TRUE
    /\ UNCHANGED
        << hostActive, canonicalDnaId, canonicalAuthor,
           canonicalAction, canonicalEpoch, canonicalRevoked,
           requiredDna, requiredAuthor, requiredAction, requiredEpoch,
           readDecision, lastReadObservedEpoch, lastReadRequiredEpoch,
           lastReadDnaMatches, lastReadAuthorMatches,
           lastReadAncestryProof, lastReadHostActive,
           lastReadCanonicalRevoked, lastReadProjectionRevoked >>

ReadWithLineageCheckpoint ==
    /\ LET dnaMatches == projectionDna = requiredDna
           authorMatches == projectionAuthor = requiredAuthor
           ancestryProof == projectionDescendsFromRequired
           epochFresh == projectionEpoch >= requiredEpoch
           lineageFresh == dnaMatches /\ authorMatches /\ ancestryProof
           fresh == epochFresh /\ (IF CheckLineage THEN lineageFresh ELSE TRUE)
           decision ==
               IF ~fresh THEN "UNKNOWN"
               ELSE IF projectionRevoked THEN "TRUSTED_REVOKED"
               ELSE "TRUSTED_ACTIVE"
       IN
           /\ readDecision' = decision
           /\ lastReadObservedEpoch' = projectionEpoch
           /\ lastReadRequiredEpoch' = requiredEpoch
           /\ lastReadDnaMatches' = dnaMatches
           /\ lastReadAuthorMatches' = authorMatches
           /\ lastReadAncestryProof' = ancestryProof
           /\ lastReadHostActive' = hostActive
           /\ lastReadCanonicalRevoked' = canonicalRevoked
           /\ lastReadProjectionRevoked' = projectionRevoked
    /\ UNCHANGED
        << hostActive, canonicalDnaId, canonicalAuthor,
           canonicalAction, canonicalEpoch, canonicalRevoked,
           requiredDna, requiredAuthor, requiredAction, requiredEpoch,
           projectionDna, projectionAuthor, projectionAction,
           projectionEpoch, projectionRevoked,
           projectionDescendsFromRequired >>

Next ==
    \/ CommitHostRevocation
    \/ ObserveForeignHistoryAtSameEpoch
    \/ ConvergeCanonicalProjection
    \/ ReadWithLineageCheckpoint

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ hostActive \in BOOLEAN
    /\ canonicalDnaId \in {canonicalDna, foreignDna}
    /\ canonicalAuthor = grantor
    /\ canonicalAction \in {NoAction, revokeAction}
    /\ canonicalEpoch \in Nat
    /\ canonicalRevoked \in BOOLEAN
    /\ requiredDna \in {canonicalDna, foreignDna}
    /\ requiredAuthor = grantor
    /\ requiredAction \in {NoAction, revokeAction}
    /\ requiredEpoch \in Nat
    /\ projectionDna \in {canonicalDna, foreignDna}
    /\ projectionAuthor \in {grantor}
    /\ projectionAction \in {NoAction, revokeAction, foreignAction}
    /\ projectionEpoch \in Nat
    /\ projectionRevoked \in BOOLEAN
    /\ projectionDescendsFromRequired \in BOOLEAN
    /\ readDecision \in {"none", "UNKNOWN", "TRUSTED_ACTIVE", "TRUSTED_REVOKED"}
    /\ lastReadObservedEpoch \in Nat
    /\ lastReadRequiredEpoch \in Nat
    /\ lastReadDnaMatches \in BOOLEAN
    /\ lastReadAuthorMatches \in BOOLEAN
    /\ lastReadAncestryProof \in BOOLEAN
    /\ lastReadHostActive \in BOOLEAN
    /\ lastReadCanonicalRevoked \in BOOLEAN
    /\ lastReadProjectionRevoked \in BOOLEAN

NoForeignLineageCanBeTrusted ==
    /\ readDecision = "TRUSTED_ACTIVE"
       => lastReadDnaMatches /\ lastReadAuthorMatches /\ lastReadAncestryProof
    /\ readDecision = "TRUSTED_REVOKED"
       => lastReadDnaMatches /\ lastReadAuthorMatches /\ lastReadAncestryProof

NoFalseFreshAuthority ==
    /\ readDecision = "TRUSTED_ACTIVE"
       => /\ lastReadHostActive
          /\ ~lastReadCanonicalRevoked
          /\ ~lastReadProjectionRevoked
          /\ lastReadObservedEpoch >= lastReadRequiredEpoch
          /\ lastReadDnaMatches
          /\ lastReadAuthorMatches
          /\ lastReadAncestryProof
    /\ readDecision = "TRUSTED_REVOKED"
       => /\ ~lastReadHostActive
          /\ lastReadCanonicalRevoked
          /\ lastReadProjectionRevoked
          /\ lastReadObservedEpoch >= lastReadRequiredEpoch
          /\ lastReadDnaMatches
          /\ lastReadAuthorMatches
          /\ lastReadAncestryProof

====
