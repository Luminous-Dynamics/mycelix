# MYC-INT-007E — Heterogeneous Federated-Node Showcase

Status: architecture / benchmark design only. Tracks #3248. Child of MYC-INT-007D / PR #3249.

Observed Integral source date: 2026-09-27.

## 1. Purpose

Define a complex federation demonstration that tests actual distributed-system questions rather than using node count as spectacle.

Integral's current public architecture explicitly describes autonomous/self-governing nodes connected through shared protocols, while also acknowledging that computational and institutional adequacy at scale remains to be tested.

A simple single-node vertical slice remains the first proof. This showcase is the second layer:

```text
single-node semantic correctness
-> heterogeneous federation conformance
-> measured scale progression
```

not:

```text
many animated nodes
-> scalability proven
```

## 2. Core theorem

```text
connectivity
!= interoperability

interoperability
!= semantic equivalence

semantic compatibility
!= policy agreement

foreign artifact accepted
!= foreign authority accepted

message delivered
!= effect authorized
```

The showcase should make these differences visible.

## 3. F0 topology — eight nodes

Start with eight deliberately heterogeneous logical nodes across three network zones.

```text
                         ┌──────────────────────┐
                         │ C — OAD / Design     │
                         │ schema v2 candidate  │
                         └─────────┬────────────┘
                                   │ CertifiedDesign-like artifact
                  ┌────────────────┴────────────────┐
                  │                                 │
                  ▼                                 ▼
        ┌──────────────────┐              ┌──────────────────┐
        │ A — H1 Physical  │              │ D — COS / Mfg    │
        │ Productive Node  │              │ synthetic prod   │
        │ Edge + sensors   │              │ labor/material   │
        └──────┬───────────┘              └──────┬───────────┘
               │ physical evidence                │ operational events
               │                                  │
               └──────────────┬───────────────────┘
                              ▼
                    ┌──────────────────┐
                    │ F — FRS /        │
                    │ Analysis Node    │
                    │ + optional       │
                    │ Symthaea         │
                    └────────┬─────────┘
                             │ recommendation / signal
                             ▼
                    ┌──────────────────┐
                    │ E — CDS /        │
                    │ Deliberation     │
                    └────────┬─────────┘
                             │ decision artifact
                             │
                  foreign decision ≠ local effect authority
                             │
        ┌────────────────────┼─────────────────────┐
        ▼                    ▼                     ▼
┌────────────────┐  ┌────────────────┐   ┌──────────────────┐
│ B — Productive │  │ G — Degraded   │   │ H — Conventional │
│ synthetic /    │  │ old version /  │   │ PostgreSQL/HTTP  │
│ replay-backed  │  │ intermittent   │   │ conformer        │
└────────────────┘  └────────────────┘   └──────────────────┘
```

## 4. Node roles

### A — real H1 physical productive node

Owns:

- Luminous Edge local sensor acquisition;
- exact physical observation/source evidence;
- local bench state;
- local action/authorization boundary when later enabled.

Optional downstream:

- Mycelix federation;
- Integral-shaped COS/FRS adapters;
- Symthaea read-only analysis.

Must remain useful when federation is unavailable.

### B — second productive profile

Initially deterministic synthetic/replay-backed unless a second physical bench exists.

Its purpose is not fake scale. It proves that the federation protocol does not assume identical sensors, installation identities, or process profiles.

### C — OAD/design-heavy node

Exercises:

- CertifiedDesign-like lineage;
- design revisions;
- certification/source evidence;
- version skew;
- supersession;
- consumer-specific local acceptance.

### D — COS/manufacturing-heavy node

Exercises:

- design consumption;
- labor/material events;
- source-owned production facts;
- idempotent downstream admission;
- local production authorization distinct from artifact receipt.

### E — CDS/deliberation-heavy node

Exercises:

- issue/evidence intake;
- decision/rationale/dissent/review basis;
- receipt of FRS recommendations;
- decision artifacts that do not themselves become remote effect authority.

### F — FRS/analysis-heavy node

Exercises:

- source-owned versus derived information;
- signal packet generation;
- recommendation routing;
- bounded read-only Symthaea analysis;
- no direct control path.

### G — degraded / old-generation node

Deliberately runs:

- older schema/interface profile;
- intermittent connectivity;
- clock skew fixture;
- delayed messages;
- stale remote projections.

It exists to expose compatibility failures rather than hide them.

### H — conventional external conformer

Use the conventional-service reference line, preferably PostgreSQL + HTTP/RPC + durable inbox/outbox under the same runtime-neutral semantics.

This node proves:

```text
Mycelix interoperability contract
!= Holochain-only protocol
```

## 5. Network zones

### Zone L — local / low-latency

Ordinary healthy LAN-like profile.

Candidate members: A, D, F.

### Zone R — remote / intermittent

Inject deterministic profiles such as:

- 150 ms latency;
- 300 ms latency;
- 500 ms latency;
- bounded loss where meaningful;
- application-level reordering/delay;
- bandwidth constraint;
- deliberate multi-minute/hour logical partitions.

Candidate members: B, G.

### Zone X — external compatibility

H and any mock external service run behind a visibly separate implementation/authority boundary.

C/E may move between zones during scenarios to avoid baking role into network topology.

## 6. Required demonstration sequence

### D1 — local-first acquisition

Disable all federation peers.

A must still acquire/log H1 observations under its local Edge profile.

Expected:

```text
federation unavailable
!= physical observation unavailable
```

### D2 — design propagation

C emits a versioned CertifiedDesign-like artifact to A and D.

Expected:

```text
artifact delivered
!= artifact locally certified/accepted
!= production authorized
```

Each recipient records its own admission state.

### D3 — duplicate COS delivery

D emits one labor/material semantic event and the delivery layer sends it twice.

Expected:

- delivery history records both attempts where appropriate;
- semantic admission remains idempotent under the exact identity/profile;
- downstream accounting policy remains Integral-owned.

### D4 — FRS → CDS path

F generates a signal/recommendation from permitted evidence and routes it to E.

Expected:

```text
signal received
-> deliberation input
!= decision
!= authorization
!= effect
```

### D5 — foreign decision negative control

E publishes a valid decision artifact that reaches A.

Expected: A stores/interprets the foreign decision under the federation profile but does not actuate a pump unless an explicit local delegation/authorization relation exists.

### D6 — partition

Partition G and one productive node after local state exists.

Expected:

- exact local functions continue where the node profile permits;
- remote information ages into stale/unknown states;
- no reconstructed fiction of global current state;
- local history remains source-owned.

### D7 — concurrent schema generation

Upgrade C/F to a successor source/interface generation while G remains on the old generation.

Expected:

- compatible additions behave according to the declared compatibility rule;
- incompatible revisions receive explicit rejection/translation handling;
- historical objects keep original schema identity;
- no silent reinterpretation.

### D8 — reconnect

Reconnect after independent local writes.

Expected:

```text
last message to arrive
!= automatically authoritative truth
```

Conflicts, unknown ordering or required reconciliation remain explicit.

### D9 — contradictory outcome

After a separately authorized local H1 intervention, new physical observations contradict the intended outcome.

Expected:

```text
contradictory outcome evidence
-> review candidate
!= historical decision mutation
!= automatic policy reversal
```

### D10 — Symthaea analysis

Send a bounded analysis request from permitted A/F evidence.

Expected:

- exact input/source lineage;
- AnalysisArtifact/recommendation output;
- no physical observation type promotion;
- no decision or effect authority.

### D11 — alternate runtime

At least one full valid semantic exchange traverses H.

Expected: external semantic identity and source provenance survive export/import; runtime-specific database/DHT IDs remain implementation provenance.

## 7. Fault campaign

F0 must exercise at least:

1. peer offline before write;
2. partition after receiver persistence but before sender acknowledgment;
3. duplicate delivery;
4. reordered delivery;
5. stale derived summary;
6. incompatible source schema;
7. unknown field under old peer;
8. source clock skew;
9. forged/unknown foreign authority;
10. expired authorization;
11. revoked/superseded design certification;
12. conflicting physical observations;
13. false-positive FRS recommendation fixture;
14. analysis service unavailable;
15. Mycelix/Holochain unavailable while Edge acquisition remains alive;
16. valid signature but local policy rejection;
17. timeout with indeterminate delivery/effect state;
18. export/import through the conventional conformer.

## 8. Scale tiers

Scale only after F0 semantic/failure correctness is measurable.

```text
F0 — 8 heterogeneous nodes
F1 — 32 logical nodes
F2 — 128 logical nodes
F3 — 512+ only after earlier resource/convergence evidence justifies it
```

At each tier bind an exact environment manifest and report independently:

- semantic object count;
- transport/delivery event count;
- local operation latency distribution;
- convergence latency distribution;
- reconnect/reconciliation duration;
- unresolved conflicts;
- stale/unknown object count;
- CPU;
- memory;
- storage growth;
- network bytes;
- operator interventions;
- failed semantic assertions;
- node runtime/schema-version mix.

No combined `federation score`.

## 9. Provenance classes

The graph must visually and structurally distinguish:

- `Physical`;
- `Synthetic`;
- `Replay`;
- `Derived`;
- `ExternalConformer`.

Likewise:

```text
8 logical nodes
!= 8 physical pilot nodes
```

## 10. Two UI views over one graph

### Story view

For contributors/operators:

```text
Design
  -> Production
  -> Evidence / Feedback
  -> Deliberation
  -> separately authorized local effects
  -> outcomes / review
```

Show partitions, stale data and foreign/local authority boundaries without requiring protocol knowledge.

### Evidence view

Advanced graph exposes:

- node/runtime identity;
- source schema version;
- semantic object identity;
- source-owner edges;
- delivery attempts and acknowledgments;
- translation receipts;
- authority/delegation edges;
- currentness;
- disagreements/conflicts;
- analysis/derivation lineage;
- qualification state.

Do not let green network connectivity imply semantic correctness or authorization.

## 11. Suggested reproducible lab substrate

Candidate implementation components:

- Nix/NixOS and/or pinned containers/VMs;
- deterministic network-fault harness such as Linux netem where suitable;
- Holochain only for the reference-runtime nodes under evaluation;
- PostgreSQL/HTTP conventional conformer from the 006M line;
- Luminous Edge physical H1 gateway;
- Leptos graph/status client;
- runtime-neutral fixture/event generator;
- immutable run/evidence manifest.

None becomes a federation protocol requirement merely by being used in the lab.

## 12. What this demonstration can establish

Under an exact executed campaign it may establish evidence such as:

- the tested implementations preserve semantic identity across federation;
- certain local functions survive specified partitions;
- version skew is handled according to the tested compatibility profile;
- duplicate/reordered deliveries do not violate specified invariants;
- foreign authority is rejected under specified local policies;
- a conventional conformer can interoperate;
- measured resource/convergence behavior at an exact node/workload tier.

## 13. What it cannot establish

Even a successful F3 run would not prove:

- Integral's social/economic model is viable;
- democratic legitimacy or participation quality;
- fair allocation;
- absence of institutional capture;
- real-world ecological correctness;
- production readiness at untested loads/topologies;
- legal/regulatory compliance;
- adoption suitability for any community.

## 14. Implementation gate

Architecture/topology/fixtures may proceed before all upstream semantic subjects qualify.

Executable conformance claims must bind the exact dependency state and never translate `queued`, `designed` or `implemented-unqualified` into `PASS`.

## 15. Recommended public framing

If this is eventually shown to Integral, describe it as:

> A heterogeneous federation stress and conformance demonstration for evaluating local autonomy, interface/version skew, partition behavior, provenance, authority boundaries, and runtime replaceability against an Integral-shaped workload.

That is stronger and more useful than presenting it as proof that a large federation already works.

## 16. Nonclaims

This architecture does not endorse Integral's political/economic model, recommend Mycelix/Holochain adoption, predict election/political outcomes, prove real-world institutional scalability, or establish production/security/legal qualification.
