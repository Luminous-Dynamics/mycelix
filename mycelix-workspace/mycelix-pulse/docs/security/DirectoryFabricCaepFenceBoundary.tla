---------------- MODULE DirectoryFabricCaepFenceBoundary ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Owners, PromoteCaepToFenceAck
ASSUME /\ Owners # {}
       /\ PromoteCaepToFenceAck \in BOOLEAN

VARIABLES ownerGeneration, caepSignals, fenceAcks, caepPromoted
vars == <<ownerGeneration, caepSignals, fenceAcks, caepPromoted>>

Init ==
    /\ ownerGeneration = [o \in Owners |-> 1]
    /\ caepSignals = {}
    /\ fenceAcks = {}
    /\ caepPromoted = {}

EmitCaepSignal(o) ==
    /\ o \in Owners
    /\ caepSignals' = caepSignals \cup {o}
    /\ IF PromoteCaepToFenceAck
          THEN /\ fenceAcks' = fenceAcks \cup {o}
               /\ caepPromoted' = caepPromoted \cup {o}
          ELSE /\ UNCHANGED fenceAcks
               /\ UNCHANGED caepPromoted
    /\ UNCHANGED ownerGeneration

InstallOwnerFence(o) ==
    /\ o \in Owners
    /\ ownerGeneration[o] = 1
    /\ ownerGeneration' = [ownerGeneration EXCEPT ![o] = 2]
    /\ UNCHANGED <<caepSignals, fenceAcks, caepPromoted>>

AcknowledgeOwnerFence(o) ==
    /\ o \in Owners
    /\ ownerGeneration[o] = 2
    /\ fenceAcks' = fenceAcks \cup {o}
    /\ UNCHANGED <<ownerGeneration, caepSignals, caepPromoted>>

Next ==
    \/ \E o \in Owners : EmitCaepSignal(o)
    \/ \E o \in Owners : InstallOwnerFence(o)
    \/ \E o \in Owners : AcknowledgeOwnerFence(o)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ ownerGeneration \in [Owners -> {1, 2}]
    /\ caepSignals \subseteq Owners
    /\ fenceAcks \subseteq Owners
    /\ caepPromoted \subseteq Owners

NoCaepSignalPromotedToFenceAck ==
    caepPromoted = {}

NoAckBeforeDurableOwnerFence ==
    \A o \in fenceAcks : ownerGeneration[o] = 2

FenceComplete ==
    fenceAcks = Owners

=============================================================================
