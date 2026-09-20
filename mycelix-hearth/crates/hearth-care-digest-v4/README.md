# Hearth Care Digest v4

Pure planner-grade household-work aggregation over already-classified authority evidence.

Digest v4 does not inspect raw DHT records or re-run canonicalization. It consumes:

- one concrete Care occurrence subject;
- its D1B assignment/time authority assessment; and
- its D1C completion authority assessment.

Only `Current assignment + Current time + Qualified completion` contributes to member workload totals. Stale, missing, conflicting, third-party, and performer-mismatch evidence remains visible through explicit counters.

Actual minutes and explicit occurrence estimates remain separate facts; missing actual duration is never guessed.
