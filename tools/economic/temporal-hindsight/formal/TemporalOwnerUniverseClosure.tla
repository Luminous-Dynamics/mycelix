---------------- MODULE TemporalOwnerUniverseClosure ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS OwnerA, OwnerB, Epoch0, Epoch1
Owners == {OwnerA, OwnerB}
ASSUME OwnerA # OwnerB
ASSUME Epoch0 # Epoch1

VARIABLES registryEpoch, liveUniverse, manifestEpoch,
          rosterCandidates, requiredOwners, excludedOwners,
          unknownOwners, exclusionProofs, frontierOwners,
          rosterClosed, frontierCovered, frontierEpoch, historicalCertEpoch

vars == <<registryEpoch, liveUniverse, manifestEpoch,
          rosterCandidates, requiredOwners, excludedOwners,
          unknownOwners, exclusionProofs, frontierOwners,
          rosterClosed, frontierCovered, frontierEpoch, historicalCertEpoch>>

Init ==
    /\ registryEpoch = Epoch0
    /\ liveUniverse = {OwnerA}
    /\ manifestEpoch = Epoch0
    /\ rosterCandidates = {OwnerA}
    /\ requiredOwners = {OwnerA}
    /\ excludedOwners = {}
    /\ unknownOwners = {}
    /\ exclusionProofs = {}
    /\ frontierOwners = {OwnerA}
    /\ rosterClosed = FALSE
    /\ frontierCovered = FALSE
    /\ frontierEpoch = Epoch0
    /\ historicalCertEpoch = Epoch0

RegisterOwnerB ==
    /\ registryEpoch = Epoch0
    /\ registryEpoch' = Epoch1
    /\ liveUniverse' = Owners
    /\ rosterClosed' = FALSE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<manifestEpoch, rosterCandidates, requiredOwners,
                   excludedOwners, unknownOwners, exclusionProofs,
                   frontierOwners, frontierEpoch, historicalCertEpoch>>

AddRosterCandidate(owner) ==
    /\ owner \in liveUniverse
    /\ owner \notin rosterCandidates
    /\ rosterCandidates' = rosterCandidates \cup {owner}
    /\ requiredOwners' = requiredOwners \cup {owner}
    /\ manifestEpoch' = registryEpoch
    /\ rosterClosed' = FALSE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, excludedOwners,
                   unknownOwners, exclusionProofs, frontierOwners,
                   historicalCertEpoch>>

MarkOwnerUnknown(owner) ==
    /\ owner \in rosterCandidates
    /\ unknownOwners' = unknownOwners \cup {owner}
    /\ rosterClosed' = FALSE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, requiredOwners, excludedOwners,
                   exclusionProofs, frontierOwners, frontierEpoch, historicalCertEpoch>>

ResolveUnknownAsRequired(owner) ==
    /\ owner \in unknownOwners
    /\ unknownOwners' = unknownOwners \ {owner}
    /\ requiredOwners' = requiredOwners \cup {owner}
    /\ excludedOwners' = excludedOwners \ {owner}
    /\ rosterClosed' = FALSE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, exclusionProofs, frontierOwners,
                   historicalCertEpoch>>

ExcludeOwnerWithProfileProof(owner) ==
    /\ owner \in requiredOwners
    /\ requiredOwners' = requiredOwners \ {owner}
    /\ excludedOwners' = excludedOwners \cup {owner}
    /\ exclusionProofs' = exclusionProofs \cup {owner}
    /\ rosterClosed' = FALSE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, unknownOwners, frontierOwners,
                   historicalCertEpoch>>

AddFrontierOwner(owner) ==
    /\ owner \in requiredOwners
    /\ owner \notin frontierOwners
    /\ frontierOwners' = frontierOwners \cup {owner}
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, requiredOwners, excludedOwners,
                   unknownOwners, exclusionProofs, rosterClosed,
                   historicalCertEpoch>>

CloseRoster ==
    /\ manifestEpoch = registryEpoch
    /\ rosterCandidates = liveUniverse
    /\ unknownOwners = {}
    /\ requiredOwners \cup excludedOwners = rosterCandidates
    /\ requiredOwners \cap excludedOwners = {}
    /\ excludedOwners \subseteq exclusionProofs
    /\ rosterClosed = FALSE
    /\ rosterClosed' = TRUE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, requiredOwners, excludedOwners,
                   unknownOwners, exclusionProofs, frontierOwners,
                   historicalCertEpoch>>

AcceptFrontier ==
    /\ rosterClosed
    /\ frontierOwners = requiredOwners
    /\ frontierCovered = FALSE
    /\ frontierCovered' = TRUE
    /\ frontierEpoch' = registryEpoch
    /\ UNCHANGED <<registryEpoch, liveUniverse, manifestEpoch,
                   rosterCandidates, requiredOwners, excludedOwners,
                   unknownOwners, exclusionProofs, frontierOwners,
                   rosterClosed, historicalCertEpoch>>

Next ==
    \/ RegisterOwnerB
    \/ \E owner \in Owners : AddRosterCandidate(owner)
    \/ \E owner \in Owners : MarkOwnerUnknown(owner)
    \/ \E owner \in Owners : ResolveUnknownAsRequired(owner)
    \/ \E owner \in Owners : ExcludeOwnerWithProfileProof(owner)
    \/ \E owner \in Owners : AddFrontierOwner(owner)
    \/ CloseRoster
    \/ AcceptFrontier

(* Faulty resolver: closes over its self-declared candidate roster without
   checking that it equals the independently enumerated live universe. *)
CloseRosterMutant ==
    /\ registryEpoch = Epoch1
    /\ rosterClosed = FALSE
    /\ manifestEpoch' = registryEpoch
    /\ rosterClosed' = TRUE
    /\ frontierCovered' = FALSE
    /\ UNCHANGED <<registryEpoch, liveUniverse, rosterCandidates,
                   requiredOwners, excludedOwners, unknownOwners,
                   exclusionProofs, frontierOwners, frontierEpoch, historicalCertEpoch>>

NextMutant == Next \/ CloseRosterMutant

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][NextMutant]_vars

TypeOK ==
    /\ registryEpoch \in {Epoch0, Epoch1}
    /\ manifestEpoch \in {Epoch0, Epoch1}
    /\ liveUniverse \subseteq Owners
    /\ rosterCandidates \subseteq Owners
    /\ requiredOwners \subseteq Owners
    /\ excludedOwners \subseteq Owners
    /\ unknownOwners \subseteq Owners
    /\ exclusionProofs \subseteq Owners
    /\ frontierOwners \subseteq Owners
    /\ rosterClosed \in BOOLEAN
    /\ frontierCovered \in BOOLEAN
    /\ frontierEpoch \in {Epoch0, Epoch1}
    /\ historicalCertEpoch \in {Epoch0, Epoch1}

RosterEpochBinding ==
    rosterClosed => manifestEpoch = registryEpoch

RosterUniverseEquality ==
    rosterClosed => rosterCandidates = liveUniverse

NoUnknownInClosedRoster ==
    rosterClosed => unknownOwners = {}

RosterDispositionPartition ==
    rosterClosed =>
      /\ requiredOwners \cup excludedOwners = rosterCandidates
      /\ requiredOwners \cap excludedOwners = {}
      /\ excludedOwners \subseteq exclusionProofs

FrontierRequiresRoster ==
    frontierCovered => rosterClosed

FrontierExactCoverage ==
    frontierCovered => frontierOwners = requiredOwners

FrontierRosterEpochBinding ==
    frontierCovered => /\ frontierEpoch = manifestEpoch
                      /\ frontierEpoch = registryEpoch

NoExcludedOwnerInFrontier ==
    frontierCovered => frontierOwners \cap excludedOwners = {}

HistoricalCertificateImmutable ==
    historicalCertEpoch = Epoch0

=============================================================================
