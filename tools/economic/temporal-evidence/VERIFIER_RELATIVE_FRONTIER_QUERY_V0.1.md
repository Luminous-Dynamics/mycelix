# Verifier-Relative Causal Frontier Query — Design v0.1

Status: research design; not implemented or production-qualified by the timestamp-relative kernel in this directory.

Related implementation: [PR #6679](https://github.com/Luminous-Dynamics/mycelix/pull/6679)  
Tracked design work: [Issue #6680](https://github.com/Luminous-Dynamics/mycelix/issues/6680)

## 1. Why a third axis is needed

Conventional bitemporal systems answer two important questions:

- **Valid time V:** when was a fact true in the modeled world?
- **System/transaction time T:** when did this database record the fact?

XTDB describes its automatically maintained system-time history alongside application-managed valid time. SQL Server system-versioned temporal tables retain row history and allow point-in-time reconstruction. These are strong baselines for regulated history and retrospective correction.

A distributed economic system needs a further distinction:

- **Verifier-relative knowledge:** what had verifier X actually incorporated into its accepted state?

A timestamp on a receipt is not proof that the receipt was included in a particular replicated checkpoint. Receipt time can differ from durable admission, and independent owners can advance at different rates. Therefore a wall-clock cutoff K is a useful v0.1 approximation, not a proof of distributed knowledge.

The research target is a query over:

`Q = (subject, valid_time V, verifier X, accepted_frontier F)`

K may be supplied as an optional reporting constraint, but it must not substitute for F membership.

## 2. Frontiers are partial-order cuts, not global clocks

A verifier frontier should describe a consistent cut across owner-scoped causal histories.

Conceptually:

`F_X = (verifier, checkpoint, owner_cuts, coverage, policy_epoch, root)`

Each owner cut contains a verified prefix or checkpoint position for that owner. The owner-local sequence and explicit dependency graph establish causal order. They are not elapsed time and must not be compared numerically across owners.

Frontier comparison is a partial order:

- `Equal`: same authenticated cut.
- `Before`: every covered owner position is at or before the other cut, with compatible roots/lineage.
- `After`: inverse of Before.
- `Concurrent`: neither cut contains the other.
- `Unknown`: one or both cuts lack enough authenticated lineage/coverage to compare.

No total order is fabricated for concurrent frontiers. A merge creates a new frontier only after validating both parent histories and all consistency constraints.

## 3. Coverage is as important as inclusion

An inclusion proof can show that a particular receipt exists under a committed root. It does not, by itself, prove that a missing receipt was absent.

A frontier therefore needs two independent proof obligations:

1. **Inclusion:** the selected knowledge receipt is a member of the authenticated frontier.
2. **Coverage/completeness:** the relevant owner prefix has been completely accounted for, with no unreported gaps that could hide another receipt or correction.

A proposed owner-cut record should bind:

- owner identity and history/profile version;
- contiguous sequence covered through;
- authenticated root/checkpoint digest for that prefix;
- explicit unresolved or missing ranges;
- parent checkpoint and consistency proof;
- proof reference for each queried receipt, or a range/absence witness where required.

If coverage is incomplete, the result is `Indeterminate`, not `NotKnown`. This prevents a partition, delayed sync, missing page, or unindexed range from being interpreted as proof that an event never arrived.

RFC 9162 provides a useful established building block: Merkle inclusion proofs establish membership under a root, while consistency proofs demonstrate append-only extension between tree heads. A Mycelix frontier adapter would still need to define owner identity, verifier acceptance, gap/coverage semantics, and causal dependencies; Certificate Transparency does not provide those application semantics by itself.

## 4. Knowledge receipt and frontier model

A future typed contract should distinguish an assertion from the evidence that a particular verifier accepted it.

### 4.1 Assertion

An assertion remains an immutable claim about a subject and dimension:

- assertion ID and canonical content digest;
- claim and subject IDs;
- dimension (occurred, valid-from, valid-to, source-recorded, receiver-observed, etc.);
- value, precision, quality and uncertainty interval;
- source-authority and source-evidence references;
- optional append-only supersession link.

### 4.2 Verifier knowledge receipt

A receipt should bind:

- receipt ID and digest;
- exact assertion ID and digest;
- verifier DID / identity;
- accepting policy version and authority epoch;
- owner-local event identity and causal predecessor(s);
- the accepted frontier/checkpoint ID;
- source/receiver time observations as separate optional dimensions;
- signature / witness reference and validation profile.

The receipt is a verifier's statement that an assertion entered its accepted knowledge state. The kernel must not treat a source's own timestamp or a transport acknowledgement as equivalent.

### 4.3 Frontier

A frontier should bind:

- verifier identity;
- checkpoint ID and canonical root digest;
- parent frontier IDs and a verifiable append-only consistency relation;
- owner-cut vector and coverage/gap declarations;
- causal-closure policy and version;
- signer / verification method and revocation context;
- creation/observation timestamps, retained as metadata rather than the ordering authority.

The owner-cut structure should reuse the repository's owner-scoped causal partial-order work rather than introduce a global sequence.

## 5. Query semantics

The v0.1 kernel uses verifier-specific receipt timestamps and a K cutoff. The v0.1 frontier design replaces that selector with authenticated membership under F:

1. Authenticate the verifier and frontier certificate.
2. Verify frontier lineage, owner cuts, coverage, and append-only consistency.
3. Verify the target assertion's receipt is included in F.
4. Verify required causal predecessors and correction lineage.
5. Resolve the maximal correction that is both causally admissible and included in F.
6. Evaluate valid-time V using source precision and uncertainty.
7. Return a certificate explaining the selected state and all relevant uncertainty.

Proposed outcomes:

| Outcome | Meaning |
|---|---|
| `Known` / `ValidAtQuery` | Required assertions and receipts are authenticated and covered by F; valid-time evaluation is determinate. |
| `NotKnown` | Frontier coverage is complete for the relevant owner prefix, and no qualifying receipt is present. |
| `Indeterminate` | Coverage, lineage, timestamp quality, or proof material is incomplete/uncertain. |
| `Conflict` | Competing valid branches, incompatible roots, or contradictory accepted evidence remain. |

A correction can replace the earlier as-known answer only when its receipt and required lineage are covered by the queried frontier. A later wall-clock timestamp or a source-side correction alone cannot rewrite the verifier's earlier frontier.

## 6. Proof-carrying query certificate

A qualified result should be independently replayable and include:

- query tuple `(subject, V, verifier, F)`;
- selected assertion ID(s) and content digests;
- accepted receipt IDs/digests and verifier identity;
- frontier root and parent lineage;
- inclusion and consistency proof references;
- owner coverage status and unresolved ranges;
- correction path plus known competing branches;
- clock profile, precision and uncertainty decision;
- policy/authority epoch references;
- canonical result digest and kernel version.

The certificate is an evidence result, not an authorization. A separate authority gate must still decide whether a journal, payment, transfer, policy mutation, or external effect is permitted.

## 7. Security and failure cases

The model must test and fail closed on:

- delayed or duplicated receipts;
- missing sequence ranges and incomplete owner prefixes;
- forks, split-brain frontiers, rollback attempts and inconsistent roots;
- concurrent (incomparable) frontiers;
- cross-verifier receipt replay;
- signature/key rotation and revocation;
- corrections whose parent is missing or not causally included;
- conflicting accepted correction branches;
- an apparent absence that lacks a completeness witness;
- time uncertainty crossing the valid-time boundary;
- digest-shaped but byte-mismatched evidence;
- policy/authority epoch changes while a receipt is in flight.

An absent proof is not an absence proof. An unknown outcome must not authorize a mutation.

## 8. Benchmark programme

The frontier extension adds to the timestamp-relative protocol in [BENCHMARK_PROTOCOL_V0.1.md](BENCHMARK_PROTOCOL_V0.1.md):

- owner cardinality and frontier width;
- frontier depth and merge fan-in;
- concurrent frontier ratio;
- unresolved coverage/gap density;
- inclusion and consistency proof size;
- receipt verification cost;
- append/update cost per owner;
- query cost under hot and random owner access;
- number of candidate correction branches;
- false-`NotKnown` and false-allow rates.

Hard gates:

1. 100% agreement with an independent small-state frontier oracle.
2. Zero false allows and zero false `NotKnown` under incomplete coverage.
3. No frontier ordering for concurrent cuts without a valid merge proof.
4. No cross-verifier leakage.
5. Deterministic replay of each query certificate from its committed inputs.
6. Proof-verification and data-lookup costs reported separately.

## 9. Staged qualification

| Stage | Deliverable | Claim ceiling |
|---|---|---|
| V0.1 | Current timestamp-relative structural selector | Query behavior over caller-supplied assertions; opaque frontier references only |
| V0.2 | Owner-cut/frontier model, coverage semantics, independent reference oracle, bounded model checks | Structural frontier semantics over synthetic evidence |
| V0.3 | Authenticated checkpoint roots, inclusion/consistency proofs, verifier receipts | Cryptographic frontier verification over test fixtures |
| V0.4 | Holochain persistence, independent replay, authority/policy adapter, adversarial recovery tests | Production qualification only after external review and exact-head evidence |

No stage inherits a stronger claim merely because the prior stage passes.

## References

- XTDB, *What is XTDB?*: https://docs.xtdb.com/intro/what-is-xtdb.html
- XTDB, *Time in XTDB*: https://docs.xtdb.com/about/time-in-xtdb.html
- Microsoft, *Temporal table considerations and limitations*: https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal-table-considerations-and-limitations
- RFC 9162, *Certificate Transparency Version 2.0*, Merkle inclusion and consistency proofs: https://www.rfc-editor.org/rfc/rfc9162.html
- DeCandia et al., *Dynamo: Amazon's highly available key-value store* (2007): https://www.amazon.science/publications/dynamo-amazons-highly-available-key-value-store

These references motivate comparator semantics and proof building blocks; they do not establish that this design is novel, implemented, or faster than those systems.
