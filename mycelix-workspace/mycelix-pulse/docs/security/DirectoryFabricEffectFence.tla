---------------- MODULE DirectoryFabricEffectFence ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Owners, AllowStaleAfterComplete
ASSUME /\ Owners # {}
       /\ AllowStaleAfterComplete \in BOOLEAN

VARIABLES installed, target, acknowledgements, tokens, lastSubmit
vars == <<installed, target, acknowledgements, tokens, lastSubmit>>

Init ==
    /\ installed = [o \in Owners |-> 1]
    /\ target = [o \in Owners |-> 2]
    /\ acknowledgements = {}
    /\ tokens = [o \in Owners |-> 1]
    /\ lastSubmit =
        [owner |-> CHOOSE o \in Owners : TRUE,
         token |-> 0,
         installed |-> 1,
         fenceComplete |-> FALSE,
         stale |-> FALSE,
         accepted |-> FALSE]

InstallFence(o) ==
    /\ o \in Owners
    /\ installed[o] < target[o]
    /\ installed' = [installed EXCEPT ![o] = target[o]]
    /\ acknowledgements' = acknowledgements \ {o}
    /\ UNCHANGED <<target, tokens, lastSubmit>>

AckFence(o) ==
    /\ o \in Owners
    /\ installed[o] = target[o]
    /\ acknowledgements' = acknowledgements \cup {o}
    /\ UNCHANGED <<installed, target, tokens, lastSubmit>>

SetToken(o, generation) ==
    /\ o \in Owners
    /\ generation \in 1..2
    /\ tokens' = [tokens EXCEPT ![o] = generation]
    /\ UNCHANGED <<installed, target, acknowledgements, lastSubmit>>

Submit(o) ==
    LET complete == acknowledgements = Owners
        stale == tokens[o] < installed[o]
        accepted ==
            IF complete /\ AllowStaleAfterComplete
            THEN TRUE
            ELSE tokens[o] = installed[o]
    IN
        /\ o \in Owners
        /\ lastSubmit' =
            [owner |-> o,
             token |-> tokens[o],
             installed |-> installed[o],
             fenceComplete |-> complete,
             stale |-> stale,
             accepted |-> accepted]
        /\ UNCHANGED <<installed, target, acknowledgements, tokens>>

Next ==
    \/ \E o \in Owners : InstallFence(o)
    \/ \E o \in Owners : AckFence(o)
    \/ \E o \in Owners : \E generation \in 1..2 : SetToken(o, generation)
    \/ \E o \in Owners : Submit(o)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ installed \in [Owners -> 1..2]
    /\ target \in [Owners -> {2}]
    /\ acknowledgements \subseteq Owners
    /\ tokens \in [Owners -> 1..2]
    /\ lastSubmit \in
        [owner : Owners,
         token : 0..2,
         installed : 1..2,
         fenceComplete : BOOLEAN,
         stale : BOOLEAN,
         accepted : BOOLEAN]

NoAckBeforeDurableInstall ==
    \A o \in acknowledgements : installed[o] = target[o]

CompleteMeansAllRequiredOwnersInstalled ==
    acknowledgements = Owners
        => \A o \in Owners : installed[o] = target[o]

NoStaleMutationAcceptanceAfterFence ==
    ~(lastSubmit.fenceComplete /\ lastSubmit.stale /\ lastSubmit.accepted)

=============================================================================
