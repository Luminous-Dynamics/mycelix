---------------- MODULE DirectoryFabricPostBarrierRouteAdmission ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    Owners,
    InitialOwners,
    CommandGeneration,
    RevokedGeneration,
    MaxOwnerEpoch,
    IgnoreRouteFence,
    IgnoreRecoveryHighWater

ASSUME
    /\ Owners # {}
    /\ InitialOwners # {}
    /\ InitialOwners \subseteq Owners
    /\ CommandGeneration > 0
    /\ RevokedGeneration > CommandGeneration
    /\ MaxOwnerEpoch > 1
    /\ IgnoreRouteFence \in BOOLEAN
    /\ IgnoreRecoveryHighWater \in BOOLEAN

VARIABLES
    globalBarrierAcknowledged,
    ownerEpoch,
    ownerGeneration,
    ownerHighWaterEpoch,
    ownerHighWaterGeneration,
    ownerReady,
    ownerReadBack,
    ownerFenceAcknowledged,
    routeState

vars ==
    <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
      ownerHighWaterEpoch, ownerHighWaterGeneration, ownerReady,
      ownerReadBack, ownerFenceAcknowledged, routeState>>

Init ==
    /\ globalBarrierAcknowledged = FALSE
    /\ ownerEpoch = [o \in Owners |-> 1]
    /\ ownerGeneration = [o \in Owners |-> CommandGeneration]
    /\ ownerHighWaterEpoch = [o \in Owners |-> 1]
    /\ ownerHighWaterGeneration = [o \in Owners |-> CommandGeneration]
    /\ ownerReady = [o \in Owners |-> TRUE]
    /\ ownerReadBack = [o \in Owners |-> TRUE]
    /\ ownerFenceAcknowledged = [o \in Owners |-> FALSE]
    /\ routeState =
        [o \in Owners |-> IF o \in InitialOwners THEN "active" ELSE "absent"]

InstallInitialFence(o) ==
    /\ o \in InitialOwners
    /\ ~ownerFenceAcknowledged[o]
    /\ ownerReady[o]
    /\ ownerGeneration[o] < RevokedGeneration
    /\ ownerGeneration' = [ownerGeneration EXCEPT ![o] = RevokedGeneration]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = FALSE]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerHighWaterEpoch,
          ownerHighWaterGeneration, ownerReady, ownerFenceAcknowledged,
          routeState>>

ReadBackInitialFence(o) ==
    /\ o \in InitialOwners
    /\ ownerReady[o]
    /\ ownerGeneration[o] = RevokedGeneration
    /\ ~ownerReadBack[o]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = TRUE]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
          ownerHighWaterEpoch, ownerHighWaterGeneration, ownerReady,
          ownerFenceAcknowledged, routeState>>

AcknowledgeInitialOwner(o) ==
    /\ o \in InitialOwners
    /\ ownerReady[o]
    /\ ownerGeneration[o] = RevokedGeneration
    /\ ownerReadBack[o]
    /\ ~ownerFenceAcknowledged[o]
    /\ ownerFenceAcknowledged' =
        [ownerFenceAcknowledged EXCEPT ![o] = TRUE]
    /\ ownerHighWaterEpoch' =
        [ownerHighWaterEpoch EXCEPT ![o] = ownerEpoch[o]]
    /\ ownerHighWaterGeneration' =
        [ownerHighWaterGeneration EXCEPT ![o] = ownerGeneration[o]]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
          ownerReady, ownerReadBack, routeState>>

AllInitialOwnersAcknowledged ==
    \A o \in InitialOwners : ownerFenceAcknowledged[o]

AcknowledgeGlobalBarrier ==
    /\ ~globalBarrierAcknowledged
    /\ AllInitialOwnersAcknowledged
    /\ globalBarrierAcknowledged' = TRUE
    /\ UNCHANGED
        <<ownerEpoch, ownerGeneration, ownerHighWaterEpoch,
          ownerHighWaterGeneration, ownerReady, ownerReadBack,
          ownerFenceAcknowledged, routeState>>

StageNewRoute(o) ==
    /\ o \in Owners \ InitialOwners
    /\ globalBarrierAcknowledged
    /\ routeState[o] = "absent"
    /\ routeState' = [routeState EXCEPT ![o] = "staged"]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
          ownerHighWaterEpoch, ownerHighWaterGeneration, ownerReady,
          ownerReadBack, ownerFenceAcknowledged>>

InstallAdmissionFence(o) ==
    /\ o \in Owners \ InitialOwners
    /\ routeState[o] = "staged"
    /\ ownerReady[o]
    /\ ownerGeneration[o] < RevokedGeneration
    /\ ownerGeneration' = [ownerGeneration EXCEPT ![o] = RevokedGeneration]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = FALSE]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerHighWaterEpoch,
          ownerHighWaterGeneration, ownerReady, ownerFenceAcknowledged,
          routeState>>

ReadBackAdmissionFence(o) ==
    /\ o \in Owners \ InitialOwners
    /\ routeState[o] = "staged"
    /\ ownerReady[o]
    /\ ownerGeneration[o] >= RevokedGeneration
    /\ ~ownerReadBack[o]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = TRUE]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
          ownerHighWaterEpoch, ownerHighWaterGeneration, ownerReady,
          ownerFenceAcknowledged, routeState>>

ActivateNewRoute(o) ==
    /\ o \in Owners \ InitialOwners
    /\ routeState[o] = "staged"
    /\ (IgnoreRouteFence
        \/ (ownerReady[o]
            /\ ownerReadBack[o]
            /\ ownerGeneration[o] >= RevokedGeneration))
    /\ routeState' = [routeState EXCEPT ![o] = "active"]
    /\ ownerHighWaterEpoch' =
        [ownerHighWaterEpoch EXCEPT ![o] = ownerEpoch[o]]
    /\ ownerHighWaterGeneration' =
        [ownerHighWaterGeneration EXCEPT ![o] = ownerGeneration[o]]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerEpoch, ownerGeneration,
          ownerReady, ownerReadBack, ownerFenceAcknowledged>>

RestoreOwnerFromStaleBackup(o) ==
    /\ o \in Owners
    /\ globalBarrierAcknowledged
    /\ routeState[o] = "active"
    /\ ownerReady[o]
    /\ (ownerFenceAcknowledged[o] \/ o \notin InitialOwners)
    /\ ownerEpoch' = [ownerEpoch EXCEPT ![o] = 0]
    /\ ownerGeneration' = [ownerGeneration EXCEPT ![o] = CommandGeneration]
    /\ ownerReady' = [ownerReady EXCEPT ![o] = FALSE]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = FALSE]
    /\ routeState' = [routeState EXCEPT ![o] = "paused"]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerHighWaterEpoch,
          ownerHighWaterGeneration, ownerFenceAcknowledged>>

RecoverOwner(o) ==
    /\ o \in Owners
    /\ routeState[o] = "paused"
    /\ ~ownerReady[o]
    /\ ownerHighWaterEpoch[o] < MaxOwnerEpoch
    /\ ownerEpoch' =
        [ownerEpoch EXCEPT ![o] =
            IF IgnoreRecoveryHighWater
            THEN @ + 1
            ELSE ownerHighWaterEpoch[o] + 1]
    /\ ownerGeneration' =
        [ownerGeneration EXCEPT ![o] =
            IF IgnoreRecoveryHighWater
            THEN @
            ELSE ownerHighWaterGeneration[o]]
    /\ ownerHighWaterEpoch' =
        IF IgnoreRecoveryHighWater
        THEN ownerHighWaterEpoch
        ELSE [ownerHighWaterEpoch EXCEPT ![o] = ownerHighWaterEpoch[o] + 1]
    /\ ownerHighWaterGeneration' =
        IF IgnoreRecoveryHighWater
        THEN ownerHighWaterGeneration
        ELSE [ownerHighWaterGeneration EXCEPT ![o] = ownerHighWaterGeneration[o]]
    /\ ownerReady' = [ownerReady EXCEPT ![o] = TRUE]
    /\ ownerReadBack' = [ownerReadBack EXCEPT ![o] = TRUE]
    /\ routeState' = [routeState EXCEPT ![o] = "active"]
    /\ UNCHANGED
        <<globalBarrierAcknowledged, ownerHighWaterEpoch,
          ownerHighWaterGeneration, ownerFenceAcknowledged>>

Next ==
    \/ \E o \in InitialOwners : InstallInitialFence(o)
    \/ \E o \in InitialOwners : ReadBackInitialFence(o)
    \/ \E o \in InitialOwners : AcknowledgeInitialOwner(o)
    \/ AcknowledgeGlobalBarrier
    \/ \E o \in Owners : StageNewRoute(o)
    \/ \E o \in Owners : InstallAdmissionFence(o)
    \/ \E o \in Owners : ReadBackAdmissionFence(o)
    \/ \E o \in Owners : ActivateNewRoute(o)
    \/ \E o \in Owners : RestoreOwnerFromStaleBackup(o)
    \/ \E o \in Owners : RecoverOwner(o)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ globalBarrierAcknowledged \in BOOLEAN
    /\ ownerEpoch \in [Owners -> 0..MaxOwnerEpoch]
    /\ ownerGeneration \in [Owners -> 1..RevokedGeneration]
    /\ ownerHighWaterEpoch \in [Owners -> 1..MaxOwnerEpoch]
    /\ ownerHighWaterGeneration \in [Owners -> 1..RevokedGeneration]
    /\ ownerReady \in [Owners -> BOOLEAN]
    /\ ownerReadBack \in [Owners -> BOOLEAN]
    /\ ownerFenceAcknowledged \in [Owners -> BOOLEAN]
    /\ routeState \in [Owners -> {"absent", "staged", "active", "paused"}]

GlobalBarrierRequiresEveryInitialOwner ==
    globalBarrierAcknowledged => AllInitialOwnersAcknowledged

NoActiveRouteBelowBarrierFence ==
    globalBarrierAcknowledged =>
        \A o \in Owners :
            routeState[o] = "active" =>
                ownerReady[o]
                /\ ownerReadBack[o]
                /\ ownerGeneration[o] >= RevokedGeneration

NoActiveRouteBelowAcknowledgedHighWater ==
    globalBarrierAcknowledged =>
        \A o \in Owners :
            routeState[o] = "active" =>
                (ownerEpoch[o] > ownerHighWaterEpoch[o])
                \/ /\ ownerEpoch[o] = ownerHighWaterEpoch[o]
                   /\ ownerGeneration[o] >= ownerHighWaterGeneration[o]

NoUnfencedNewRoute ==
    globalBarrierAcknowledged =>
        \A o \in Owners \ InitialOwners :
            routeState[o] = "active" =>
                ownerGeneration[o] >= RevokedGeneration
                /\ ownerReadBack[o]

=============================================================================
