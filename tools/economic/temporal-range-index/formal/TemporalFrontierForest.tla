------------------------ MODULE TemporalFrontierForest ------------------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS OwnerA, OwnerB, Frontier0, Frontier1,
          OwnerSequenceCut, TreeSize, GapCardinality

Owners == {OwnerA, OwnerB}
Frontiers == {Frontier0, Frontier1}

ASSUME OwnerA # OwnerB
ASSUME DOMAIN OwnerSequenceCut = Owners
ASSUME DOMAIN TreeSize = Owners
ASSUME DOMAIN GapCardinality = Owners

VARIABLES acceptedFrontier,
          manifestFrontier,
          manifestOwners,
          proofOwners,
          projectionOwners,
          declaredForestComplete,
          projectionReplayComplete

vars == <<acceptedFrontier, manifestFrontier, manifestOwners, proofOwners,
          projectionOwners, declaredForestComplete, projectionReplayComplete>>

Init ==
    /\ acceptedFrontier = Frontier0
    /\ manifestFrontier = Frontier0
    /\ manifestOwners = {}
    /\ proofOwners = {}
    /\ projectionOwners = {}
    /\ declaredForestComplete = FALSE
    /\ projectionReplayComplete = FALSE

AddManifestOwner(owner) ==
    /\ owner \in Owners
    /\ owner \notin manifestOwners
    /\ manifestOwners' = manifestOwners \cup {owner}
    /\ UNCHANGED <<acceptedFrontier, manifestFrontier, proofOwners,
                   projectionOwners, declaredForestComplete, projectionReplayComplete>>

AddOwnerProof(owner) ==
    /\ owner \in Owners
    /\ owner \in manifestOwners
    /\ owner \notin proofOwners
    /\ proofOwners' = proofOwners \cup {owner}
    /\ UNCHANGED <<acceptedFrontier, manifestFrontier, manifestOwners,
                   projectionOwners, declaredForestComplete, projectionReplayComplete>>

ReplayOwnerProjection(owner) ==
    /\ owner \in Owners
    /\ owner \in manifestOwners
    /\ owner \notin projectionOwners
    /\ projectionOwners' = projectionOwners \cup {owner}
    /\ UNCHANGED <<acceptedFrontier, manifestFrontier, manifestOwners,
                   proofOwners, declaredForestComplete, projectionReplayComplete>>

AcceptDeclaredForest ==
    /\ manifestFrontier = acceptedFrontier
    /\ manifestOwners = Owners
    /\ proofOwners = Owners
    /\ declaredForestComplete = FALSE
    /\ declaredForestComplete' = TRUE
    /\ UNCHANGED <<acceptedFrontier, manifestFrontier, manifestOwners,
                   proofOwners, projectionOwners, projectionReplayComplete>>

AcceptProjectionReplay ==
    /\ declaredForestComplete
    /\ projectionOwners = Owners
    /\ projectionReplayComplete = FALSE
    /\ projectionReplayComplete' = TRUE
    /\ UNCHANGED <<acceptedFrontier, manifestFrontier, manifestOwners,
                   proofOwners, projectionOwners, declaredForestComplete>>

AdvanceFrontier ==
    /\ acceptedFrontier = Frontier0
    /\ acceptedFrontier' = Frontier1
    /\ declaredForestComplete' = FALSE
    /\ projectionReplayComplete' = FALSE
    /\ UNCHANGED <<manifestFrontier, manifestOwners, proofOwners, projectionOwners>>

Next ==
    \/ \E owner \in Owners : AddManifestOwner(owner)
    \/ \E owner \in Owners : AddOwnerProof(owner)
    \/ \E owner \in Owners : ReplayOwnerProjection(owner)
    \/ AcceptDeclaredForest
    \/ AcceptProjectionReplay
    \/ AdvanceFrontier

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ acceptedFrontier \in Frontiers
    /\ manifestFrontier \in Frontiers
    /\ manifestOwners \subseteq Owners
    /\ proofOwners \subseteq Owners
    /\ projectionOwners \subseteq Owners
    /\ declaredForestComplete \in BOOLEAN
    /\ projectionReplayComplete \in BOOLEAN

ExactFrontierBinding ==
    declaredForestComplete => manifestFrontier = acceptedFrontier

ExactOwnerCoverage ==
    declaredForestComplete => /\ manifestOwners = Owners
                               /\ proofOwners = Owners

ProjectionRequiresForest ==
    projectionReplayComplete => declaredForestComplete

ProjectionOwnerCoverage ==
    projectionReplayComplete => projectionOwners = Owners

OwnerCoordinateConservation ==
    \A owner \in Owners :
        TreeSize[owner] + GapCardinality[owner] = OwnerSequenceCut[owner]

NoStaleProjectionCertificate ==
    projectionReplayComplete => manifestFrontier = acceptedFrontier

=============================================================================
