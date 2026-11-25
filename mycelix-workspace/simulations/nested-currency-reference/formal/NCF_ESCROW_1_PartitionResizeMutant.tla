---------------- MODULE NCF_ESCROW_1_PartitionResizeMutant ----------------
EXTENDS NCF_ESCROW_1

\* Deliberately unsafe: mutate a shard's allocated risk rights while its
\* reservation vector is supposed to be frozen during a network partition.
PartitionResize(s, amount) ==
    /\ s \in Shards
    /\ amount \in 1..Capacity
    /\ partitioned = TRUE
    /\ reserved[s] # amount
    /\ reserved' = [reserved EXCEPT ![s] = amount]
    /\ UNCHANGED << outstanding, partitioned, eventCount, partitionSnapshotReserved >>

PartitionResizeMutantNext == Next \/ \E s \in Shards, amount \in 1..Capacity: PartitionResize(s, amount)
PartitionResizeMutantSpec == Init /\ [][PartitionResizeMutantNext]_vars

====
