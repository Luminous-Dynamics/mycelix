---- MODULE PulseAuthorityCausalFrontier ----
EXTENDS Naturals, FiniteSets, TLC

(*
Two-chain causal-frontier model.

A checkpoint over independent source chains is a vector/partial-order frontier,
not a globally ordered scalar.

The required checkpoint is the pair:
    chain A: exact action a1 at sequence 1
    chain B: exact action b1 at sequence 1

A foreign or incomplete projection at A=2, B=0 has the same aggregate height
(2) but has not observed B's required transition and does not descend from the
required A action. It must remain UNKNOWN.

CheckComponentWise = TRUE:
  require both exact chain heads, per-chain sequence lower bounds and ancestry.

CheckComponentWise = FALSE:
  deliberate mutant; compare only sum(projection sequence) >= sum(required
  sequence), allowing one chain's height to mask another chain's omission.

Host grants are modeled separately from projections and checkpoints.
*)

CONSTANTS
    a0, a1, a2,
    b0, b1,
    NoAction

ActionsA == {a0, a1, a2, a3}
ActionsB == {b0, b1, b2}

VARIABLES
    hostAActive,
    hostBActive,
    canonicalASeq,
    canonicalBSeq,
    canonicalAHead,
    canonicalBHead,
    canonicalARevoked,
    canonicalBRevoked,
    requiredASeq,
    requiredBSeq,
    requiredAHead,
    requiredBHead,
    projectionASeq,
    projectionBSeq,
    projectionAHead,
    projectionBHead,
    projectionARevoked,
    projectionBRevoked,
    projectionADescends,
    projectionBDescends,
    readDecision,
    lastReadASeq,
    lastReadBSeq,
    lastReadAHead,
    lastReadBHead,
    lastReadADescends,
    lastReadBDescends,
    lastReadARevoked,
    lastReadBRevoked,
    lastCanonicalARevoked,
    lastCanonicalBRevoked,
    lastHostAActive,
    lastHostBActive

vars ==
    << hostAActive, hostBActive,
       canonicalASeq, canonicalBSeq, canonicalAHead, canonicalBHead,
       canonicalARevoked, canonicalBRevoked,
       requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
       projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
       projectionARevoked, projectionBRevoked,
       projectionADescends, projectionBDescends, readDecision,
       lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
       lastReadADescends, lastReadBDescends,
       lastReadARevoked, lastReadBRevoked,
       lastCanonicalARevoked, lastCanonicalBRevoked,
       lastHostAActive, lastHostBActive >>

Init ==
    /\ hostAActive = TRUE
    /\ hostBActive = TRUE
    /\ canonicalASeq = 0
    /\ canonicalBSeq = 0
    /\ canonicalAHead = a0
    /\ canonicalBHead = b0
    /\ canonicalARevoked = FALSE
    /\ canonicalBRevoked = FALSE
    /\ requiredASeq = 0
    /\ requiredBSeq = 0
    /\ requiredAHead = a0
    /\ requiredBHead = b0
    /\ projectionASeq = 0
    /\ projectionBSeq = 0
    /\ projectionAHead = a0
    /\ projectionBHead = b0
    /\ projectionARevoked = FALSE
    /\ projectionBRevoked = FALSE
    /\ projectionADescends = TRUE
    /\ projectionBDescends = TRUE
    /\ readDecision = "none"
    /\ lastReadASeq = 0
    /\ lastReadBSeq = 0
    /\ lastReadAHead = a0
    /\ lastReadBHead = b0
    /\ lastReadADescends = TRUE
    /\ lastReadBDescends = TRUE
    /\ lastReadARevoked = FALSE
    /\ lastReadBRevoked = FALSE
    /\ lastCanonicalARevoked = FALSE
    /\ lastCanonicalBRevoked = FALSE
    /\ lastHostAActive = TRUE
    /\ lastHostBActive = TRUE

CommitRevokeA ==
    /\ hostAActive
    /\ hostAActive' = FALSE
    /\ canonicalASeq' = canonicalASeq + 1
    /\ canonicalAHead' = a1
    /\ canonicalARevoked' = TRUE
    /\ requiredASeq' = canonicalASeq + 1
    /\ requiredAHead' = a1
    /\ readDecision' =
        IF readDecision \in {"TRUSTED_ACTIVE", "TRUSTED_REVOKED"} THEN "STALE" ELSE readDecision
    /\ UNCHANGED
        << hostBActive, canonicalBSeq, canonicalBHead, canonicalBRevoked,
           requiredBSeq, requiredBHead,
           projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
           projectionARevoked, projectionBRevoked,
           projectionADescends, projectionBDescends,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

CommitRevokeB ==
    /\ hostBActive
    /\ hostBActive' = FALSE
    /\ canonicalBSeq' = canonicalBSeq + 1
    /\ canonicalBHead' = b1
    /\ canonicalBRevoked' = TRUE
    /\ requiredBSeq' = canonicalBSeq + 1
    /\ requiredBHead' = b1
    /\ readDecision' =
        IF readDecision \in {"TRUSTED_ACTIVE", "TRUSTED_REVOKED"} THEN "STALE" ELSE readDecision
    /\ UNCHANGED
        << hostAActive, canonicalASeq, canonicalAHead, canonicalARevoked,
           requiredASeq, requiredAHead,
           projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
           projectionARevoked, projectionBRevoked,
           projectionADescends, projectionBDescends,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

(* A later action on the same canonical chain is valid when its ancestry
   from the required checkpoint is established, even though its ActionHash
   differs from the checkpoint ActionHash. *)
CommitPostCheckpointAppendA ==
    /\ canonicalARevoked
    /\ canonicalASeq = requiredASeq
    /\ canonicalAHead = requiredAHead
    /\ canonicalASeq' = canonicalASeq + 1
    /\ canonicalAHead' = a2
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalBSeq, canonicalBHead, canonicalBRevoked,
           canonicalARevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
           projectionARevoked, projectionBRevoked,
           projectionADescends, projectionBDescends, readDecision,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

CommitPostCheckpointAppendB ==
    /\ canonicalBRevoked
    /\ canonicalBSeq = requiredBSeq
    /\ canonicalBHead = requiredBHead
    /\ canonicalBSeq' = canonicalBSeq + 1
    /\ canonicalBHead' = b2
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalASeq, canonicalAHead, canonicalARevoked,
           canonicalBRevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
           projectionARevoked, projectionBRevoked,
           projectionADescends, projectionBDescends, readDecision,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

ObserveScalarHeightForeignProjection ==
    /\ canonicalARevoked /\ canonicalBRevoked
    /\ projectionASeq < requiredASeq + requiredBSeq
    /\ projectionASeq' = requiredASeq + requiredBSeq
    /\ projectionBSeq' = 0
    /\ projectionAHead' = a3
    /\ projectionBHead' = b0
    /\ projectionARevoked' = FALSE
    /\ projectionBRevoked' = FALSE
    /\ projectionADescends' = FALSE
    /\ projectionBDescends' = FALSE
    /\ readDecision' =
        IF readDecision \in {"TRUSTED_ACTIVE", "TRUSTED_REVOKED"}
        THEN "CONFLICTING" ELSE readDecision
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalASeq, canonicalBSeq, canonicalAHead, canonicalBHead,
           canonicalARevoked, canonicalBRevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

ConvergeA ==
    /\ (projectionASeq < canonicalASeq
        \/ projectionAHead # canonicalAHead
        \/ ~projectionADescends)
    /\ projectionASeq' = canonicalASeq
    /\ projectionAHead' = canonicalAHead
    /\ projectionARevoked' = canonicalARevoked
    /\ projectionADescends' = TRUE
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalASeq, canonicalBSeq, canonicalAHead, canonicalBHead,
           canonicalARevoked, canonicalBRevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           projectionBSeq, projectionBHead, projectionBRevoked,
           projectionBDescends, readDecision,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

ConvergeB ==
    /\ (projectionBSeq < canonicalBSeq
        \/ projectionBHead # canonicalBHead
        \/ ~projectionBDescends)
    /\ projectionBSeq' = canonicalBSeq
    /\ projectionBHead' = canonicalBHead
    /\ projectionBRevoked' = canonicalBRevoked
    /\ projectionBDescends' = TRUE
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalASeq, canonicalBSeq, canonicalAHead, canonicalBHead,
           canonicalARevoked, canonicalBRevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           projectionASeq, projectionAHead, projectionARevoked,
           projectionADescends, readDecision,
           lastReadASeq, lastReadBSeq, lastReadAHead, lastReadBHead,
           lastReadADescends, lastReadBDescends,
           lastReadARevoked, lastReadBRevoked,
           lastCanonicalARevoked, lastCanonicalBRevoked,
           lastHostAActive, lastHostBActive >>

componentwiseFresh ==
    /\ projectionASeq >= requiredASeq
    /\ projectionBSeq >= requiredBSeq
    /\ projectionADescends
    /\ projectionBDescends

scalarFresh ==
    projectionASeq + projectionBSeq >= requiredASeq + requiredBSeq

ReadWithDecision(fresh) ==
    /\ readDecision' =
        IF ~fresh THEN "UNKNOWN"
        ELSE IF canonicalARevoked \/ canonicalBRevoked
             THEN "TRUSTED_REVOKED"
             ELSE "TRUSTED_ACTIVE"
    /\ lastReadASeq' = projectionASeq
    /\ lastReadBSeq' = projectionBSeq
    /\ lastReadAHead' = projectionAHead
    /\ lastReadBHead' = projectionBHead
    /\ lastReadADescends' = projectionADescends
    /\ lastReadBDescends' = projectionBDescends
    /\ lastReadARevoked' = projectionARevoked
    /\ lastReadBRevoked' = projectionBRevoked
    /\ lastCanonicalARevoked' = canonicalARevoked
    /\ lastCanonicalBRevoked' = canonicalBRevoked
    /\ lastHostAActive' = hostAActive
    /\ lastHostBActive' = hostBActive
    /\ UNCHANGED
        << hostAActive, hostBActive,
           canonicalASeq, canonicalBSeq, canonicalAHead, canonicalBHead,
           canonicalARevoked, canonicalBRevoked,
           requiredASeq, requiredBSeq, requiredAHead, requiredBHead,
           projectionASeq, projectionBSeq, projectionAHead, projectionBHead,
           projectionARevoked, projectionBRevoked,
           projectionADescends, projectionBDescends >>

ReadWithCausalFrontier ==
    ReadWithDecision(componentwiseFresh)

ReadWithScalarHeight ==
    ReadWithDecision(scalarFresh)

Next ==
    \/ CommitRevokeA
    \/ CommitRevokeB
    \/ CommitPostCheckpointAppendA
    \/ CommitPostCheckpointAppendB
    \/ ObserveScalarHeightForeignProjection
    \/ ConvergeA
    \/ ConvergeB
    \/ ReadWithCausalFrontier

MutantNext ==
    \/ CommitRevokeA
    \/ CommitRevokeB
    \/ ObserveScalarHeightForeignProjection
    \/ ConvergeA
    \/ ConvergeB
    \/ ReadWithScalarHeight

Spec == Init /\ [][Next]_vars

ScalarMutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ hostAActive \in BOOLEAN
    /\ hostBActive \in BOOLEAN
    /\ canonicalASeq \in Nat
    /\ canonicalBSeq \in Nat
    /\ canonicalAHead \in ActionsA
    /\ canonicalBHead \in ActionsB
    /\ canonicalARevoked \in BOOLEAN
    /\ canonicalBRevoked \in BOOLEAN
    /\ requiredASeq \in Nat
    /\ requiredBSeq \in Nat
    /\ requiredAHead \in ActionsA
    /\ requiredBHead \in ActionsB
    /\ projectionASeq \in Nat
    /\ projectionBSeq \in Nat
    /\ projectionAHead \in ActionsA
    /\ projectionBHead \in ActionsB
    /\ projectionARevoked \in BOOLEAN
    /\ projectionBRevoked \in BOOLEAN
    /\ projectionADescends \in BOOLEAN
    /\ projectionBDescends \in BOOLEAN
    /\ readDecision \in {"none", "UNKNOWN", "STALE", "CONFLICTING", "TRUSTED_ACTIVE", "TRUSTED_REVOKED"}
    /\ lastReadASeq \in Nat
    /\ lastReadBSeq \in Nat
    /\ lastReadAHead \in ActionsA
    /\ lastReadBHead \in ActionsB
    /\ lastReadADescends \in BOOLEAN
    /\ lastReadBDescends \in BOOLEAN
    /\ lastReadARevoked \in BOOLEAN
    /\ lastReadBRevoked \in BOOLEAN
    /\ lastCanonicalARevoked \in BOOLEAN
    /\ lastCanonicalBRevoked \in BOOLEAN
    /\ lastHostAActive \in BOOLEAN
    /\ lastHostBActive \in BOOLEAN

NoFalseFreshCausalFrontier ==
    /\ readDecision = "TRUSTED_ACTIVE"
       => /\ ~lastCanonicalARevoked
          /\ ~lastCanonicalBRevoked
          /\ lastHostAActive
          /\ lastHostBActive
          /\ lastReadAHead = requiredAHead
          /\ lastReadBHead = requiredBHead
          /\ lastReadADescends
          /\ lastReadBDescends
    /\ readDecision = "TRUSTED_REVOKED"
       => /\ (lastCanonicalARevoked \/ lastCanonicalBRevoked)
          /\ lastReadAHead = requiredAHead
          /\ lastReadBHead = requiredBHead
          /\ lastReadADescends
          /\ lastReadBDescends
          /\ (lastReadARevoked \/ lastReadBRevoked)

CausalFrontierRequiresEveryChain ==
    readDecision \in {"TRUSTED_ACTIVE", "TRUSTED_REVOKED"}
    => /\ lastReadASeq >= requiredASeq
       /\ lastReadBSeq >= requiredBSeq
       /\ lastReadAHead = requiredAHead
       /\ lastReadBHead = requiredBHead

====
