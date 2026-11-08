---- MODULE PulseTransparencyWitness ----
EXTENDS Naturals, FiniteSets, TLC

(*
Bounded anti-equivocation model for a SCITT-compatible transparency checkpoint.

Both candidate roots are assumed to:
  - carry a valid signature from the same pinned Transparency Service key;
  - include the target signed statement;
  - carry a valid append-only consistency proof from the same prior checkpoint;
  - have the same log_id and tree_size.

Therefore the model deliberately studies the residual split-view problem:
the log may show different, individually plausible same-size roots to
different clients.

Witness profile:
  - four witnesses;
  - at most one Byzantine witness;
  - safe quorum = 3 of 4;
  - honest witnesses durably lock one root per (log_id, tree_size);
  - clients accept only a root with a matching quorum visible to that client.

The model is not a Merkle-proof implementation and does not prove witness
independence or key custody. It isolates quorum-intersection, anti-double-sign
and partition semantics.
*)

CONSTANTS
    w1, w2, w3, w4, badWitness,
    c1, c2, NoRoot,
    WitnessQuorum,
    EnforceWitnessLock,
    DurableWitnessLocks,
    StartPartitioned

Witnesses == {w1, w2, w3, w4}
HonestWitnesses == Witnesses \ {badWitness}
Clients == {c1, c2}
Roots == {"root-A", "root-B"}

VARIABLES
    logSignedRoots,
    inclusionVerifiedRoots,
    consistencyVerifiedRoots,
    witnessSigned,
    witnessLock,
    clientViews,
    accepted

vars ==
    << logSignedRoots, inclusionVerifiedRoots,
       consistencyVerifiedRoots, witnessSigned,
       witnessLock, clientViews, accepted >>

Init ==
    /\ logSignedRoots = {}
    /\ inclusionVerifiedRoots = {}
    /\ consistencyVerifiedRoots = {}
    /\ witnessSigned = [w \in Witnesses |-> {}]
    /\ witnessLock = [w \in Witnesses |-> NoRoot]
    /\ clientViews =
        IF StartPartitioned
        THEN [c1 |-> {w1, badWitness}, c2 |-> {w2, badWitness}]
        ELSE [c \in Clients |-> Witnesses]
    /\ accepted = [c \in Clients |-> NoRoot]

PublishRoot(root) ==
    /\ root \in Roots
    /\ root \notin logSignedRoots
    /\ logSignedRoots' = logSignedRoots \cup {root}
    /\ inclusionVerifiedRoots' = inclusionVerifiedRoots \cup {root}
    /\ consistencyVerifiedRoots' = consistencyVerifiedRoots \cup {root}
    /\ UNCHANGED << witnessSigned, witnessLock, clientViews, accepted >>

SignRoot(w, root) ==
    /\ w \in Witnesses
    /\ root \in logSignedRoots
    /\ \/ w = badWitness
       \/ ~EnforceWitnessLock
       \/ witnessLock[w] = NoRoot
       \/ witnessLock[w] = root
    /\ witnessSigned' =
        [witnessSigned EXCEPT ![w] = @ \cup {root}]
    /\ witnessLock' = [witnessLock EXCEPT ![w] = root]
    /\ UNCHANGED
        << logSignedRoots, inclusionVerifiedRoots,
           consistencyVerifiedRoots, clientViews, accepted >>

RestartWitness(w) ==
    /\ w \in HonestWitnesses
    /\ ~DurableWitnessLocks
    /\ witnessLock[w] # NoRoot
    /\ witnessLock' = [witnessLock EXCEPT ![w] = NoRoot]
    /\ UNCHANGED
        << logSignedRoots, inclusionVerifiedRoots,
           consistencyVerifiedRoots, witnessSigned, clientViews, accepted >>

PartitionClients ==
    /\ clientViews[c1] = Witnesses
    /\ clientViews[c2] = Witnesses
    /\ clientViews' =
        [c1 |-> {w1, badWitness}, c2 |-> {w2, badWitness}]
    /\ UNCHANGED
        << logSignedRoots, inclusionVerifiedRoots,
           consistencyVerifiedRoots, witnessSigned, witnessLock, accepted >>

HealClients ==
    /\ clientViews' = [c \in Clients |-> Witnesses]
    /\ UNCHANGED
        << logSignedRoots, inclusionVerifiedRoots,
           consistencyVerifiedRoots, witnessSigned, witnessLock, accepted >>

AcceptCheckpoint(client, root) ==
    /\ client \in Clients
    /\ root \in Roots
    /\ accepted[client] = NoRoot
    /\ root \in logSignedRoots
    /\ root \in inclusionVerifiedRoots
    /\ root \in consistencyVerifiedRoots
    /\ Cardinality(
         {w \in clientViews[client] : root \in witnessSigned[w]}
       ) >= WitnessQuorum
    /\ accepted' = [accepted EXCEPT ![client] = root]
    /\ UNCHANGED
        << logSignedRoots, inclusionVerifiedRoots,
           consistencyVerifiedRoots, witnessSigned,
           witnessLock, clientViews >>

Next ==
    \/ \E root \in Roots : PublishRoot(root)
    \/ \E w \in Witnesses, root \in Roots : SignRoot(w, root)
    \/ \E w \in HonestWitnesses : RestartWitness(w)
    \/ PartitionClients
    \/ HealClients
    \/ \E client \in Clients, root \in Roots :
         AcceptCheckpoint(client, root)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ logSignedRoots \subseteq Roots
    /\ inclusionVerifiedRoots \subseteq Roots
    /\ consistencyVerifiedRoots \subseteq Roots
    /\ witnessSigned \in [Witnesses -> SUBSET Roots]
    /\ witnessLock \in [Witnesses -> (Roots \cup {NoRoot})]
    /\ clientViews \in [Clients -> SUBSET Witnesses]
    /\ accepted \in [Clients -> (Roots \cup {NoRoot})]
    /\ WitnessQuorum \in 1..Cardinality(Witnesses)
    /\ EnforceWitnessLock \in BOOLEAN
    /\ DurableWitnessLocks \in BOOLEAN
    /\ StartPartitioned \in BOOLEAN

NoSentinelCollision ==
    /\ NoRoot \notin Roots
    /\ badWitness \in Witnesses
    /\ Cardinality(HonestWitnesses) = 3

NoSplitViewAccepted ==
    Cardinality(
      {root \in Roots : \E client \in Clients : accepted[client] = root}
    ) <= 1

AcceptedHasValidProofs ==
    \A client \in Clients :
      accepted[client] \in Roots =>
        /\ accepted[client] \in logSignedRoots
        /\ accepted[client] \in inclusionVerifiedRoots
        /\ accepted[client] \in consistencyVerifiedRoots

AcceptedHasWitnessQuorum ==
    \A client \in Clients :
      accepted[client] \in Roots =>
        Cardinality(
          {w \in clientViews[client] :
             accepted[client] \in witnessSigned[w]}
        ) >= WitnessQuorum

NoQuorumMeansUnknown ==
    \A client \in Clients, root \in Roots :
      Cardinality(
        {w \in clientViews[client] : root \in witnessSigned[w]}
      ) < WitnessQuorum
      => accepted[client] # root

HonestWitnessNoDoubleSign ==
    \A w \in HonestWitnesses :
      Cardinality(witnessSigned[w]) <= 1

WitnessLockMatchesIssuedSignature ==
    \A w \in HonestWitnesses :
      witnessLock[w] # NoRoot => witnessLock[w] \in witnessSigned[w]

QuorumIntersectionRule ==
    2 * WitnessQuorum > Cardinality(Witnesses) + 1

PartitionedViewsCannotReachQuorum ==
    StartPartitioned =>
      \A client \in Clients :
        Cardinality(clientViews[client]) < WitnessQuorum
        => accepted[client] = NoRoot

====
