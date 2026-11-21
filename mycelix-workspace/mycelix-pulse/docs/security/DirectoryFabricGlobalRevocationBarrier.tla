---------------- MODULE DirectoryFabricGlobalRevocationBarrier ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    Owners,
    Workers,
    NoWorker,
    CommandGeneration,
    MaxGeneration,
    MaxClaimEpoch,
    MaxConcurrentEffects,
    MaxEffectCommits,
    MaxTopologyEpoch,
    IgnoreOwnerFence,
    IgnoreWorkerClaim,
    AckBeforeReadback,
    AckWithMissingOwner,
    IgnoreTopologyChange

ASSUME
    /\ Owners # {}
    /\ Workers # {}
    /\ NoWorker \notin Workers
    /\ CommandGeneration > 0
    /\ MaxGeneration > CommandGeneration
    /\ MaxClaimEpoch > 0
    /\ MaxConcurrentEffects > 0
    /\ MaxEffectCommits > 0
    /\ MaxTopologyEpoch > 0
    /\ IgnoreOwnerFence \in BOOLEAN
    /\ IgnoreWorkerClaim \in BOOLEAN
    /\ AckBeforeReadback \in BOOLEAN
    /\ AckWithMissingOwner \in BOOLEAN
    /\ IgnoreTopologyChange \in BOOLEAN

VARIABLES
    barrierStarted,
    globalBarrierAcknowledged,
    topologyEpoch,
    frozenTopologyEpoch,
    inventoryCurrent,
    ownerGeneration,
    fenceRequested,
    fenceLockHeld,
    fenceCommitted,
    fenceReadBack,
    ownerAcknowledged,
    claimEpoch,
    activeWorker,
    inFlightCount,
    inFlightGeneration,
    effectCommitCount,
    staleEffectCommitCount

vars ==
    <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
      frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
      fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
      ownerAcknowledged, claimEpoch, activeWorker, inFlightCount,
      inFlightGeneration, effectCommitCount, staleEffectCommitCount>>

Init ==
    /\ barrierStarted = FALSE
    /\ globalBarrierAcknowledged = FALSE
    /\ topologyEpoch = 0
    /\ frozenTopologyEpoch = 0
    /\ inventoryCurrent = TRUE
    /\ ownerGeneration = [o \in Owners |-> CommandGeneration]
    /\ fenceRequested = [o \in Owners |-> FALSE]
    /\ fenceLockHeld = [o \in Owners |-> FALSE]
    /\ fenceCommitted = [o \in Owners |-> FALSE]
    /\ fenceReadBack = [o \in Owners |-> FALSE]
    /\ ownerAcknowledged = [o \in Owners |-> FALSE]
    /\ claimEpoch = [o \in Owners |-> 0]
    /\ activeWorker = [o \in Owners |-> NoWorker]
    /\ inFlightCount = [o \in Owners |-> 0]
    /\ inFlightGeneration = [o \in Owners |-> CommandGeneration]
    /\ effectCommitCount = [o \in Owners |-> 0]
    /\ staleEffectCommitCount = [o \in Owners |-> 0]

StartBarrier ==
    /\ ~barrierStarted
    /\ ~globalBarrierAcknowledged
    /\ barrierStarted' = TRUE
    /\ frozenTopologyEpoch' = topologyEpoch
    /\ inventoryCurrent' = TRUE
    /\ fenceRequested' = [o \in Owners |-> TRUE]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, topologyEpoch, ownerGeneration,
          fenceLockHeld, fenceCommitted, fenceReadBack, ownerAcknowledged,
          claimEpoch, activeWorker, inFlightCount, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

InstallWorkerClaim(o, w) ==
    /\ o \in Owners
    /\ w \in Workers
    /\ ~fenceLockHeld[o]
    /\ inFlightCount[o] = 0
    /\ claimEpoch[o] < MaxClaimEpoch
    /\ claimEpoch' = [claimEpoch EXCEPT ![o] = @ + 1]
    /\ activeWorker' = [activeWorker EXCEPT ![o] = w]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
          ownerAcknowledged, inFlightCount, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

StartEffect(o, w, suppliedGeneration, suppliedClaimEpoch) ==
    /\ o \in Owners
    /\ w \in Workers
    /\ suppliedGeneration \in 1..MaxGeneration
    /\ suppliedClaimEpoch \in 0..MaxClaimEpoch
    /\ ~fenceLockHeld[o]
    /\ inFlightCount[o] < MaxConcurrentEffects
    /\ (inFlightCount[o] = 0 \/ inFlightGeneration[o] = suppliedGeneration)
    /\ (IgnoreOwnerFence
        \/ (suppliedGeneration = ownerGeneration[o]
            /\ (~fenceRequested[o] \/ fenceCommitted[o])))
    /\ (IgnoreWorkerClaim
        \/ (activeWorker[o] = w /\ claimEpoch[o] = suppliedClaimEpoch))
    /\ inFlightCount' = [inFlightCount EXCEPT ![o] = @ + 1]
    /\ inFlightGeneration' =
        [inFlightGeneration EXCEPT ![o] = suppliedGeneration]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
          ownerAcknowledged, claimEpoch, activeWorker, effectCommitCount,
          staleEffectCommitCount>>

CommitInFlightEffect(o) ==
    /\ o \in Owners
    /\ inFlightCount[o] > 0
    /\ effectCommitCount[o] < MaxEffectCommits
    /\ inFlightCount' = [inFlightCount EXCEPT ![o] = @ - 1]
    /\ effectCommitCount' = [effectCommitCount EXCEPT ![o] = @ + 1]
    /\ staleEffectCommitCount' =
        IF inFlightGeneration[o] < ownerGeneration[o]
        THEN [staleEffectCommitCount EXCEPT ![o] = @ + 1]
        ELSE staleEffectCommitCount
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
          ownerAcknowledged, claimEpoch, activeWorker, inFlightGeneration>>

AbortInFlightEffect(o) ==
    /\ o \in Owners
    /\ inFlightCount[o] > 0
    /\ inFlightCount' = [inFlightCount EXCEPT ![o] = @ - 1]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
          ownerAcknowledged, claimEpoch, activeWorker, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

AcquireFenceLock(o) ==
    /\ o \in Owners
    /\ barrierStarted
    /\ fenceRequested[o]
    /\ ~fenceCommitted[o]
    /\ ~fenceLockHeld[o]
    /\ inFlightCount[o] = 0
    /\ fenceLockHeld' = [fenceLockHeld EXCEPT ![o] = TRUE]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceCommitted, fenceReadBack, ownerAcknowledged,
          claimEpoch, activeWorker, inFlightCount, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

CommitFence(o) ==
    /\ o \in Owners
    /\ fenceLockHeld[o]
    /\ ownerGeneration[o] < MaxGeneration
    /\ ownerGeneration' = [ownerGeneration EXCEPT ![o] = @ + 1]
    /\ fenceCommitted' = [fenceCommitted EXCEPT ![o] = TRUE]
    /\ fenceLockHeld' = [fenceLockHeld EXCEPT ![o] = FALSE]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, fenceRequested,
          fenceReadBack, ownerAcknowledged, claimEpoch, activeWorker,
          inFlightCount, inFlightGeneration, effectCommitCount,
          staleEffectCommitCount>>

ReadBackFence(o) ==
    /\ o \in Owners
    /\ fenceCommitted[o]
    /\ ~fenceReadBack[o]
    /\ fenceReadBack' = [fenceReadBack EXCEPT ![o] = TRUE]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, ownerAcknowledged,
          claimEpoch, activeWorker, inFlightCount, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

AcknowledgeOwner(o) ==
    /\ o \in Owners
    /\ fenceCommitted[o]
    /\ ~ownerAcknowledged[o]
    /\ (fenceReadBack[o] \/ AckBeforeReadback)
    /\ ownerAcknowledged' = [ownerAcknowledged EXCEPT ![o] = TRUE]
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, topologyEpoch,
          frozenTopologyEpoch, inventoryCurrent, ownerGeneration,
          fenceRequested, fenceLockHeld, fenceCommitted, fenceReadBack,
          claimEpoch, activeWorker, inFlightCount, inFlightGeneration,
          effectCommitCount, staleEffectCommitCount>>

TopologyDrift ==
    /\ barrierStarted
    /\ ~globalBarrierAcknowledged
    /\ topologyEpoch < MaxTopologyEpoch
    /\ topologyEpoch' = topologyEpoch + 1
    /\ inventoryCurrent' = FALSE
    /\ UNCHANGED
        <<barrierStarted, globalBarrierAcknowledged, frozenTopologyEpoch,
          ownerGeneration, fenceRequested, fenceLockHeld, fenceCommitted,
          fenceReadBack, ownerAcknowledged, claimEpoch, activeWorker,
          inFlightCount, inFlightGeneration, effectCommitCount,
          staleEffectCommitCount>>

AllOwnersAcknowledged ==
    \A o \in Owners : ownerAcknowledged[o]

AcknowledgeGlobalBarrier ==
    /\ barrierStarted
    /\ ~globalBarrierAcknowledged
    /\ (AckWithMissingOwner \/ AllOwnersAcknowledged)
    /\ (IgnoreTopologyChange
        \/ (inventoryCurrent /\ topologyEpoch = frozenTopologyEpoch))
    /\ globalBarrierAcknowledged' = TRUE
    /\ UNCHANGED
        <<barrierStarted, topologyEpoch, frozenTopologyEpoch,
          inventoryCurrent, ownerGeneration, fenceRequested, fenceLockHeld,
          fenceCommitted, fenceReadBack, ownerAcknowledged, claimEpoch,
          activeWorker, inFlightCount, inFlightGeneration, effectCommitCount,
          staleEffectCommitCount>>

Next ==
    \/ StartBarrier
    \/ \E o \in Owners, w \in Workers : InstallWorkerClaim(o, w)
    \/ \E o \in Owners, w \in Workers,
          g \in 1..MaxGeneration, e \in 0..MaxClaimEpoch :
          StartEffect(o, w, g, e)
    \/ \E o \in Owners : CommitInFlightEffect(o)
    \/ \E o \in Owners : AbortInFlightEffect(o)
    \/ \E o \in Owners : AcquireFenceLock(o)
    \/ \E o \in Owners : CommitFence(o)
    \/ \E o \in Owners : ReadBackFence(o)
    \/ \E o \in Owners : AcknowledgeOwner(o)
    \/ TopologyDrift
    \/ AcknowledgeGlobalBarrier

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ barrierStarted \in BOOLEAN
    /\ globalBarrierAcknowledged \in BOOLEAN
    /\ topologyEpoch \in 0..MaxTopologyEpoch
    /\ frozenTopologyEpoch \in 0..MaxTopologyEpoch
    /\ inventoryCurrent \in BOOLEAN
    /\ ownerGeneration \in [Owners -> 1..MaxGeneration]
    /\ fenceRequested \in [Owners -> BOOLEAN]
    /\ fenceLockHeld \in [Owners -> BOOLEAN]
    /\ fenceCommitted \in [Owners -> BOOLEAN]
    /\ fenceReadBack \in [Owners -> BOOLEAN]
    /\ ownerAcknowledged \in [Owners -> BOOLEAN]
    /\ claimEpoch \in [Owners -> 0..MaxClaimEpoch]
    /\ activeWorker \in [Owners -> Workers \cup {NoWorker}]
    /\ inFlightCount \in [Owners -> 0..MaxConcurrentEffects]
    /\ inFlightGeneration \in [Owners -> 1..MaxGeneration]
    /\ effectCommitCount \in [Owners -> 0..MaxEffectCommits]
    /\ staleEffectCommitCount \in [Owners -> 0..MaxEffectCommits]

FenceLockExcludesInFlightEffects ==
    \A o \in Owners : fenceLockHeld[o] => inFlightCount[o] = 0

OwnerAckRequiresDurableReadBack ==
    \A o \in Owners :
        ownerAcknowledged[o] => fenceCommitted[o] /\ fenceReadBack[o]

GlobalAckRequiresExactOwnerSetAndCurrentInventory ==
    ~globalBarrierAcknowledged
    \/ (AllOwnersAcknowledged
        /\ inventoryCurrent
        /\ topologyEpoch = frozenTopologyEpoch)

NoStaleEffectAfterOwnerAck ==
    \A o \in Owners :
        ownerAcknowledged[o] => staleEffectCommitCount[o] = 0

NoStaleEffectAfterGlobalAck ==
    ~globalBarrierAcknowledged
    \/ \A o \in Owners : staleEffectCommitCount[o] = 0

=============================================================================
