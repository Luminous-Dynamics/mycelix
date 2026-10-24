---------------- MODULE NCF_ESCROW_1_DuplicateAllocationMutant ----------------
EXTENDS NCF_ESCROW_1

\* Deliberately unsafe: on a stale capacity snapshot, allocate the full global
\* ceiling to a second shard even though another shard already owns risk rights.
DuplicateAllocation(s) ==
    /\ s \in Shards
    /\ partitioned = FALSE
    /\ reserved[s] = 0
    /\ TotalReserved > 0
    /\ reserved' = [reserved EXCEPT ![s] = Capacity]
    /\ UNCHANGED << outstanding, partitioned, eventCount >>

DuplicateAllocationMutantNext == Next \/ \E s \in Shards: DuplicateAllocation(s)
DuplicateAllocationMutantSpec == Init /\ [][DuplicateAllocationMutantNext]_vars

====
