# MYC-INT-007M — Second-Generation B→C Recursive Seeding Fixture

Status: design/fixture only. Child of MYC-INT-007K/L.

007M makes the N7 recursion theorem concrete by freezing a second-generation seed package produced by Node B for Node C, plus C's planned FreshNode bootstrap run.

```text
A-origin public provenance may remain visible
!= A authority
!= A secrets
!= A runtime dependency
!= A required for B→C
```

B's package remains the same seven-class authority-free seed contract, but its component generations are B-owned/admitted/adapted generations. Each component now declares an `origin_kind`: artifacts derived from B's accepted/adapted A→B state retain a `source_admission_ref` plus public provenance lineage, while components created locally at B carry no fabricated A lineage.

C independently admits or adapts B's package, generates fresh node/host/application identities, initializes C-owned governance, inherits no A/B standing or authority, and may operate before federation.

The fixture explicitly requires A to be unavailable before B emits the second-generation package and throughout C's bootstrap. No A online service, secret service, authority service, or governance service may be a required runtime dependency.

The future recursion condition continues:

```text
B seeds C
B later unavailable
C operates its declared local profile
C may emit C-owned seed package
C may seed D using the same contract
A and B are not authority ancestors
```

007M does not establish actual B→C execution, physical independence, economic self-sufficiency, legal compliance, social legitimacy, or real-world recursive community replication.