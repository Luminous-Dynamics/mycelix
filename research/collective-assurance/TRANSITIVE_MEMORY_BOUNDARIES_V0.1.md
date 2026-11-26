# Transitive Collective Memory Boundaries v0.1

**Status:** pure Rust reference qualifier, synthetic paired fixture and schema on a feature branch. No runtime enforcement, production deletion, physical erasure, machine unlearning, or SOTA result is claimed.

## Purpose

This increment composes the direct workflow boundary in #6711 with the downstream-impact and disclosure-authority owners #3029 and #2824.

It qualifies whether a derived artifact retains the limits of every contributing parent, whether a deliberate scope expansion has an explicit receipt, whether a revocation closure covers every transitive descendant, and whether subsequent uses match the known policy state at the time of use.

It does not implement another memory store, evidence ledger, authorization engine, policy evaluator, workflow executor or deletion service.

## Governing rules

```text
relevant(source)
!= eligible(source, principal, purpose, entity, time)

eligible(source)
!= eligible(derive(source))

valid receipt shape
!= authenticated receipt

revocation event recorded now
!= event known to every consumer at an earlier time
```

1. **No implicit authority amplification.** A derivative's principal, purpose and entity sets must be subsets of every declared parent scope. Its valid-time interval and retention deadline must also be no broader than each parent. An empty list is never an unbounded wildcard.
2. **Multi-source composition uses the intersection of parent eligibility.** A derivative based on several sources must fit every parent's constraints. One permissive source cannot wash out a more restrictive source.
3. **Explicit declassification is a separate event.** A broader derivative requires a receipt binding the exact ordered parent IDs and payload digests, transform, output payload, output scope, policy digest and issue/expiry times. The validator recomputes a domain-separated canonical receipt digest over every field and compares it to the declared digest. This establishes self-consistent field binding only; it does not authenticate the issuer or decide whether the declassification policy was legitimate.
4. **Unknown lineage is not a clean result.** Missing declared parents are only representable when the artifact explicitly marks lineage incomplete; the overall result becomes INDETERMINATE. A complete lineage with missing parents, duplicate IDs or cycles fails.
5. **Revocation closure is transitive and exact.** A complete closure must equal the graph-derived descendant set for the exact graph digest. Omitted or invented descendants fail; missing or incomplete closure is INDETERMINATE.
6. **Effective and recorded times are distinct.** A revocation blocks a derivation or use only once it is both effective and known in the relevant evaluator/authority domain. An event recorded later does not silently rewrite an earlier use. Retroactive legal/policy semantics are not inferred here.
7. **Historical snapshot identity is preserved.** Each use stores the digest of the policy/evidence snapshot it actually observed. It is allowed to differ from the current snapshot after later revocation. The historical digest must be resolvable in an independently retained archive before any external historical-reconstruction claim.
8. **No deletion inflation.** Access revocation, descendant ineligibility, tombstone, cache/index removal, physical erasure and model unlearning are separate outcomes. This validator only qualifies declared lineage, scope, receipt and use records.

## Typed contract

### Scope

A scope contains exact, non-empty unique principal, purpose and entity identifiers (no wildcard `*` and no leading/trailing whitespace), a half-open validity interval `[valid_from_ms, valid_until_ms)`, and a positive retention deadline strictly later than `valid_from_ms`; a deadline at the start of validity leaves no usable interval and is rejected.

No semantic equivalence, inferred group membership, global role, wildcard, tenant alias, legal interpretation or external identity lookup is performed.

### Artifact

A source artifact has no parent, transform or derivation receipt.

A derived artifact binds:
- exact ordered parent artifact IDs;
- output payload digest;
- transform digest;
- derivation receipt digest;
- output scope;
- creation time;
- explicit lineage completeness;
- optional declassification receipt.

Parent IDs are required to be sorted and unique. The graph must be acyclic, and a child cannot predate a declared parent.

### Revocation

Each event binds:
- event and source identity;
- effective time;
- recorded/observed time;
- policy epoch digest;
- authority receipt digest;
- event receipt digest.

Each closure binds the exact source, event, graph digest, sorted transitive descendant IDs, completeness state and closure receipt digest.

The validator recomputes descendants from the graph; it does not trust a supplied closure list.

### Use observation

Each use binds:
- use and artifact identity;
- principal, purpose and entity;
- use time;
- event-time policy/evidence snapshot digest;
- consumed / blocked-by-scope / blocked-by-retention / blocked-by-revocation / unknown disposition;
- use receipt digest;
- effect receipt digest when consumed.

A consumed use must have an effect receipt. The snapshot digest is not required to equal the current snapshot digest, because historical uses must retain their event-time reference after later updates. The referenced historical snapshot must be independently archived and authenticated outside this crate.

## Temporal semantics

For a revocation event with `effective_at` and `recorded_at`, the current reference profile blocks a source-derived use at time T only when:

```text
effective_at <= T AND recorded_at <= T
```

A revocation known before its future effective time does not block use before the effective time. A revocation with an effective date in the past but recorded after a historical use does not retroactively change that use in this profile. A separately qualified retroactive policy would need an explicit policy/effect contract and is out of scope.

The snapshot's `as_of_ms` bounds the included artifact inventory and use records. It is not a trusted clock; time authenticity must be supplied by an external witness before operational claims.

## Qualification output

The report keeps separate counts for:
- total and derived artifacts;
- revocation events and transitive descendants;
- consumed, blocked and unknown uses.

Status semantics:
- **PASS:** declared inventories and lineage are complete; scope, declassification, closures and uses satisfy the profile.
- **FAIL:** a known structural contradiction or prohibited use is found.
- **INDETERMINATE:** relevant inventory, lineage, closure or use disposition is incomplete/unknown.

An INDETERMINATE result is not a pass for new protected use.

## Synthetic adversarial coverage

The current Rust tests cover 16 cases, including:
- narrowed derivative and authorized use;
- scope amplification without declassification;
- exact declassification receipt binding and mutation of its policy fields;
- canonical independent golden vectors for scope, declassification receipt, graph and snapshot digests;
- retention/validity chronology and wildcard/non-canonical scope-ID rejection;
- post-revocation use of a descendant blocked;
- historical use before effective revocation;
- incomplete revocation closure;
- unknown use disposition;
- omitted descendant in a supposedly complete closure;
- event-time historical snapshot digest preserved after later revocation;
- effective-before-recorded revocation does not rewrite a prior use, but blocks later use;
- a derivative created after a known effective revocation fails;
- multi-source derivative must satisfy every parent's scope.

The JSON fixture demonstrates one source → summary derivation, a source revocation, a complete descendant closure and a blocked post-revocation use.

## Composition map

- #6711: direct task-graph artifact handoffs and per-exposure qualification.
- #6700: signed generation-input exposure; runtime witness still required.
- #3029: upstream rights/terms change and descendant-impact graph.
- #2824: current disclosure authority and revocation semantics.
- #2952: paired-world hidden/protected information noninterference.
- #5460 / #6730: observable-trace and memory-specific metadata leakage.
- #6719: research parent for transitive derived-memory boundaries.

## Validation and claim ceiling

The current implementation uses deterministic, domain-separated SHA-256 encodings for graph, scope and snapshot identity. These digests establish byte-level binding within this reference profile; they do not authenticate signers, receipt issuers, clocks, source truth, policy authority or completeness of the external storage inventory.

A future exact-head PASS may establish only these declared synthetic structural properties. It does not prove production access enforcement, complete discovery of every copy/cache/index, physical deletion, model unlearning, legal compliance, consent validity, absence of covert channels or universal security.

Production qualification would additionally need independently retained and authenticated receipts, exact snapshot archive resolution, real API/index/cache observation, concurrent revocation-race tests, external effect witnesses and owner-authorized enforcement at every protected use boundary.
