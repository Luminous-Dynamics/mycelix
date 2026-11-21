---------------- MODULE DirectoryFabricEffectInventory ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Owners, InventoryOwners, FenceableOwners,
          TrustDeclaredInventory, TrustNonFenceableRoutes

ASSUME /\ Owners # {}
       /\ InventoryOwners \subseteq Owners
       /\ FenceableOwners \subseteq Owners
       /\ TrustDeclaredInventory \in BOOLEAN
       /\ TrustNonFenceableRoutes \in BOOLEAN

VARIABLES installed, acknowledgements, claimComplete
vars == <<installed, acknowledgements, claimComplete>>

Init ==
    /\ installed = [o \in Owners |-> 1]
    /\ acknowledgements = {}
    /\ claimComplete = FALSE

InstallFence(o) ==
    /\ o \in InventoryOwners
    /\ installed[o] = 1
    /\ installed' = [installed EXCEPT ![o] = 2]
    /\ UNCHANGED <<acknowledgements, claimComplete>>

AckFence(o) ==
    /\ o \in InventoryOwners
    /\ installed[o] = 2
    /\ acknowledgements' = acknowledgements \cup {o}
    /\ UNCHANGED <<installed, claimComplete>>

ClaimStrongRevocationComplete ==
    /\ acknowledgements = InventoryOwners
    /\ (TrustDeclaredInventory \/ InventoryOwners = Owners)
    /\ (TrustNonFenceableRoutes \/ FenceableOwners = Owners)
    /\ claimComplete' = TRUE
    /\ UNCHANGED <<installed, acknowledgements>>

Next ==
    \/ \E o \in InventoryOwners : InstallFence(o)
    \/ \E o \in InventoryOwners : AckFence(o)
    \/ ClaimStrongRevocationComplete

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ installed \in [Owners -> 1..2]
    /\ acknowledgements \subseteq InventoryOwners
    /\ claimComplete \in BOOLEAN

CompleteClaimRequiresCompleteInventory ==
    claimComplete => InventoryOwners = Owners

CompleteClaimRequiresAtomicFenceForEveryRoute ==
    claimComplete => FenceableOwners = Owners

CompleteClaimRequiresEveryOwnerInstalledAndAcknowledged ==
    claimComplete =>
        /\ acknowledgements = Owners
        /\ \A o \in Owners : installed[o] = 2

=============================================================================
