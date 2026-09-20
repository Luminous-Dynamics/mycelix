# PSI-002C0 — deterministic admission reference model

Status: **draft reference model / no durable runtime / no enumeration-resistance claim**.

This crate freezes the pure transition algebra required before a durable admission ledger is implemented. It deliberately contains no database, filesystem, networking, threads, wall clock, randomness, authentication, or cryptography.

The model consumes only PSI-002B policy plus server-visible admission metadata. It never receives raw queried identifiers.

Core theorem:

```text
reference transition model matches registered invariants
!= durable atomic transaction demonstrated
!= crash recovery demonstrated
!= distributed consistency demonstrated
!= enumeration resistance
```

Rejected admissions return the exact prior state unchanged. Successful admissions consume the request commitment and request/element/concurrency budget in one pure state transition. Release is idempotent and never makes a consumed commitment reusable. Revocation and epoch closure are explicit transitions.

The model is intended to become the oracle for PSI-002C1 durable-backend differential tests.
