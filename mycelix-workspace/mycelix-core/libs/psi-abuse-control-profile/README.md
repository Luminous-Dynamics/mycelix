# PSI-002B — abuse-control composition profile

Status: **structural source / experimental / not compile-qualified / not an enumeration-resistance claim**.

This crate defines the policy boundary above PSI-002A. It does not modify the VOPRF construction and does not receive raw queried identifiers.

```text
bounded request budget
!= enumeration resistance

scoped capability
!= Sybil resistance

client authentication
!= client anonymity

blinded query
!= anonymous transport

required atomic accounting contract
!= durable atomic accounting demonstrated
```

The profile accounts only for server-visible admission metadata: capability identity, epoch, request count, blinded-element count, request commitment/replay status, revocation status, concurrency state, and admission timing.

Raw contact identifiers are forbidden at this boundary. The admission service cannot enforce “unique guessed contact” semantics over values it intentionally cannot see.

The normative runtime requirements are frozen in `RUNTIME-CONTRACT.md`. Any later executor must prove, at minimum, that revocation check, replay check, request/element/concurrency budget checks, commitment consumption, and counter reservation happen in one serializable compare-and-consume decision with no partial success state. Epoch rollover must use a fresh server-authorized ledger; clients cannot reset budgets by inventing an epoch identifier.

Even a valid structural profile returns false for enumeration resistance, Sybil resistance, anonymity, privacy-preserving accounting, durable atomic enforcement, production admission, and application authority.

Operational enforcement, durable storage, distributed consistency, capability issuance, revocation propagation, crash recovery, Sybil assumptions, transport privacy, and real-data consent remain separate evidence gates.
