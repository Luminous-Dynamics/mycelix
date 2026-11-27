------------------------ MODULE TemporalIntervalProof ------------------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Records, Query, ValidFrom, ValidTo, OpenEnd

ASSUME Records = {0, 1, 2, 3}
ASSUME OpenEnd \subseteq Records

Nodes == {"root", "left", "right", "leaf0", "leaf1", "leaf2", "leaf3"}
InternalNodes == {"root", "left", "right"}
Leaves == {"leaf0", "leaf1", "leaf2", "leaf3"}

Children == [
    "root"  |-> {"left", "right"},
    "left"  |-> {"leaf0", "leaf1"},
    "right" |-> {"leaf2", "leaf3"}
]

Desc(n) ==
    CASE n = "root"  -> Records
      [] n = "left"  -> {0, 1}
      [] n = "right" -> {2, 3}
      [] n = "leaf0" -> {0}
      [] n = "leaf1" -> {1}
      [] n = "leaf2" -> {2}
      [] n = "leaf3" -> {3}

RecordOf(n) ==
    CASE n = "leaf0" -> 0
      [] n = "leaf1" -> 1
      [] n = "leaf2" -> 2
      [] n = "leaf3" -> 3

Matches(r) ==
    /\ ValidFrom[r] <= Query
    /\ (r \in OpenEnd \/ Query < ValidTo[r])

MatchingRecords == {r \in Records : Matches(r)}

(* This is the summary-based pruning theorem written without hash details:
   prune only when every descendant begins after Q, or every descendant
   has a finite end at or before Q. *)
FutureOnly(n) ==
    \A r \in Desc(n) : ValidFrom[r] > Query

ExpiredOnly(n) ==
    /\ \A r \in Desc(n) : r \notin OpenEnd
    /\ \A r \in Desc(n) : ValidTo[r] <= Query

SafePrune(n) == FutureOnly(n) \/ ExpiredOnly(n)

VARIABLES pending, pruned, returned
vars == <<pending, pruned, returned>>

Init ==
    /\ pending = {"root"}
    /\ pruned = {}
    /\ returned = {}

Expand(n) ==
    /\ n \in pending
    /\ n \in InternalNodes
    /\ pending' = (pending \ {n}) \cup Children[n]
    /\ UNCHANGED <<pruned, returned>>

Prune(n) ==
    /\ n \in pending
    /\ SafePrune(n)
    /\ pending' = pending \ {n}
    /\ pruned' = pruned \cup {n}
    /\ UNCHANGED returned

ReturnMatchingLeaf(n) ==
    /\ n \in pending
    /\ n \in Leaves
    /\ RecordOf(n) \in MatchingRecords
    /\ pending' = pending \ {n}
    /\ returned' = returned \cup Desc(n)
    /\ UNCHANGED pruned

Next ==
    \/ \E n \in Nodes : Expand(n)
    \/ \E n \in Nodes : Prune(n)
    \/ \E n \in Nodes : ReturnMatchingLeaf(n)

Spec == Init /\ [][Next]_vars

PrunedCoverage == UNION {Desc(n) : n \in pruned}
PendingCoverage == UNION {Desc(n) : n \in pending}

TypeOK ==
    /\ pending \subseteq Nodes
    /\ pruned \subseteq Nodes
    /\ returned \subseteq Records

NoFalsePrune ==
    PrunedCoverage \cap MatchingRecords = {}

NoFalseReturn ==
    returned \subseteq MatchingRecords

CoveragePartition ==
    /\ returned \cap PrunedCoverage = {}
    /\ returned \cap PendingCoverage = {}
    /\ PrunedCoverage \cap PendingCoverage = {}
    /\ returned \cup PrunedCoverage \cup PendingCoverage = Records

CompleteAtTermination ==
    pending = {} => returned = MatchingRecords

=============================================================================
