---------------- MODULE NCF_ESCROW_1 ----------------
EXTENDS Naturals, Integers, TLC

CONSTANTS Capacity, MaxOps, Shards

ASSUME Capacity \in Nat \ {0}
ASSUME MaxOps \in Nat \ {0}
ASSUME Shards = {"s1", "s2"}

VARIABLES reserved, outstanding, partitioned, eventCount, partitionSnapshotReserved

vars == << reserved, outstanding, partitioned, eventCount, partitionSnapshotReserved >>

\* The finite model intentionally fixes two shards so the arithmetic is explicit.
TotalReserved == reserved["s1"] + reserved["s2"]
TotalOutstanding == outstanding["s1"] + outstanding["s2"]

Init ==
    /\ reserved = [s \in Shards |-> 0]
    /\ outstanding = [s \in Shards |-> 0]
    /\ partitioned = FALSE
    /\ eventCount = 0
    /\ partitionSnapshotReserved = [s \in Shards |-> 0]

Reserve(s, amount) ==
    /\ s \in Shards
    /\ amount \in 1..Capacity
    /\ partitioned = FALSE
    /\ reserved[s] = 0
    /\ TotalReserved + amount <= Capacity
    /\ reserved' = [reserved EXCEPT ![s] = amount]
    /\ UNCHANGED << outstanding, partitioned, eventCount, partitionSnapshotReserved >>

StartPartition ==
    /\ partitioned = FALSE
    /\ TotalReserved > 0
    /\ partitioned' = TRUE
    /\ partitionSnapshotReserved' = reserved
    /\ UNCHANGED << reserved, outstanding, eventCount >>

HealPartition ==
    /\ partitioned = TRUE
    /\ partitioned' = FALSE
    /\ UNCHANGED << reserved, outstanding, eventCount, partitionSnapshotReserved >>

SpendInSlice(s, amount) ==
    /\ s \in Shards
    /\ amount \in 1..Capacity
    /\ reserved[s] > 0
    /\ outstanding[s] + amount <= reserved[s]
    /\ eventCount < MaxOps
    /\ outstanding' = [outstanding EXCEPT ![s] = @ + amount]
    /\ eventCount' = eventCount + 1
    /\ UNCHANGED << reserved, partitioned, partitionSnapshotReserved >>

SettleInSlice(s, amount) ==
    /\ s \in Shards
    /\ amount \in 1..Capacity
    /\ outstanding[s] >= amount
    /\ eventCount < MaxOps
    /\ outstanding' = [outstanding EXCEPT ![s] = @ - amount]
    /\ eventCount' = eventCount + 1
    /\ UNCHANGED << reserved, partitioned >>

Next ==
    \/ \E s \in Shards, amount \in 1..Capacity: Reserve(s, amount)
    \/ StartPartition
    \/ HealPartition
    \/ \E s \in Shards, amount \in 1..Capacity: SpendInSlice(s, amount)
    \/ \E s \in Shards, amount \in 1..Capacity: SettleInSlice(s, amount)

TypeOK ==
    /\ reserved \in [Shards -> 0..Capacity]
    /\ outstanding \in [Shards -> 0..Capacity]
    /\ partitionSnapshotReserved \in [Shards -> 0..Capacity]
    /\ partitioned \in BOOLEAN
    /\ eventCount \in 0..MaxOps

ReservedCapacityBound ==
    TotalReserved <= Capacity

OutstandingWithinReservation ==
    \A s \in Shards: outstanding[s] <= reserved[s]

AggregateOutstandingWithinReservedBudget ==
    TotalOutstanding <= TotalReserved

AggregateOutstandingWithinCeiling ==
    TotalOutstanding <= Capacity

NoUnreservedOutstanding ==
    \A s \in Shards: outstanding[s] > 0 => reserved[s] > 0

PartitionedReservationsStayFixed ==
    partitioned => reserved = partitionSnapshotReserved

Spec == Init /\ [][Next]_vars

====
