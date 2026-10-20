# Claim-Ceiling Meet Reference V1

**Status:** executable contract candidate; exact-head qualification pending until the dedicated workflow reports its result.  
**Parent:** [Mycelix EPI-FABRIC-005 / issue #3475](https://github.com/Luminous-Dynamics/mycelix/issues/3475) and [EPI-FABRIC-002 / issue #3472](https://github.com/Luminous-Dynamics/mycelix/issues/3472).  
**Claim ceiling:** algebra/reference behavior only. No evidence admission, truth, source authentication, or action authority.

## Paradigm shift

Treat a claim ceiling as a partial order over *permitted claim forms and scopes*, not as a truth probability or a scalar trust score. A result must not become stronger because it was replicated, projected, summarized, embedded, cached, or moved between repositories.

The portable reference makes five choices explicit:

1. **Coupled claim atoms.** An allowed atom binds predicate, evidence class, derivation method, and intended use. This prevents a component-wise cross-product from inventing a claim combination that neither operand allowed.
2. **Finite, explicit scopes.** Subject IDs and contexts must be named. V1 has no wildcard meaning for missing scope. Half-open temporal intervals intersect by taking the later start and earlier end; a non-overlap is bottom.
3. **Conservative meet.** Allowed atoms and scopes intersect. Mandatory qualifications, independent-evidence controls, and provenance requirements union. Composition therefore narrows permissions while preserving all obligations.
4. **Action authority is a separate dimension.** Disjoint action scopes produce an empty action-scope set, but do not automatically erase an otherwise meaningful analytical claim. The result grants no authority: this is a ceiling algebra, not an authorization grant.
5. **Receipts bind bytes and limit claims.** The CLI hashes the exact input and both schema byte strings. A supplied source SHA is marked pinned only after matching the checkout's actual HEAD and confirming the tracked tree is clean.

This is a small reference policy algebra. It does not determine whether two sources are truly independent, whether a qualification was legitimately granted, whether a clock is trusted, or whether a domain-specific predicate/evidence combination is correct. Policy construction and canonical admission stay separately governed.

## Order and laws

Write (A \preceq B) when A is no broader than B: A allows a subset of B's claim atoms, subjects, contexts, time interval, and action scopes, while A preserves at least all qualifications, independence controls, and provenance requirements imposed by B.

For a compatible pair, (A \wedge B) is computed as the intersection of allowed scopes and the union of required controls.

The executable tests require:
- idempotence: (A \wedge A = A);
- commutativity: (A \wedge B = B \wedge A);
- associativity over defined non-bottom examples;
- non-amplification: the usable meet is no broader than either operand;
- obligation preservation: composition cannot drop a qualification, provenance requirement, or independence control;
- explicit bottom for incompatible atoms, subjects, contexts, or time windows;
- no accidental escalation where action scopes are disjoint.

V1 uses finite enumerated sets and integer time boundaries for determinism. It deliberately does not define an implicit top element, wildcard IDs, probabilistic confidence, or a numeric ranking over different evidence classes. Evidence classes such as a formal proof and an instrument measurement are not generally interchangeable on one total scale.

## Comparison with standards and the field

This work is complementary to, not a replacement for:
- **W3C PROV**, which gives a general model for entities, activities, agents, and provenance relations: https://www.w3.org/TR/prov-overview/
- **NIST AI Risk Management Framework**, which organizes risk management across design, development, deployment, use, and evaluation: https://www.nist.gov/itl/ai-risk-management-framework
- **W3C AI Computed Provenance Community Group**, whose 2026 draft charter emphasizes independently re-deriving how a conclusion is supported, and distinguishing proofs from records of grounds: https://github.com/w3c-cg/ai-computed-provenance/blob/main/documents/charter/charter.md

Those references motivate the boundary, but this PR does not claim W3C/NIST conformance. Its increment is an executable cross-project ceiling algebra with frozen inputs and reproducible outputs, rather than another descriptive vocabulary.

## Benchmark and acceptance plan

The targets below are proposed engineering gates, not measured results or industry-standard thresholds.

| Gate | Proposed acceptance |
|---|---|
| Algebra laws | 100% pass for frozen idempotence, commutativity, associativity, and no-broader-than properties |
| Negative controls | 100% of frozen incompatible-atom, disjoint-subject, empty-time, unknown-field, bad-SHA, and mismatched-checkout controls rejected or returned as explicit bottom |
| Golden-vector portability | Independent Rust and a second language produce identical disposition, dimensions, ceiling, and input digest for every vector |
| Mutation effectiveness | Every seeded mutation that widens a scope, removes a requirement, or upgrades evidence/use is detected by at least one named control |
| Determinism | Identical bytes, schema, executable and source identity yield byte-identical receipts |
| Scale | First profile at 10, 100, 1,000, and 10,000 atoms/scope identities; report p50/p95/max runtime and peak memory before setting a production SLO |
| Integration | A read-only consumer may import the same ceiling, but must not infer admission, truth, freshness or action authority from it |

### Next work

1. Run this dedicated exact-head workflow; inspect compiler, formatter, schema, and golden-vector output without treating skipped checks as passed.
2. Add an independent second-language golden-vector verifier and semantic mutants for widening each individual dimension.
3. Add an explicit policy profile for cross-domain transfers (formal result to engineering claim, simulated service capacity to physical delivery, model inference to measured observation). Such profiles should reject unsupported pairings rather than score them.
4. Integrate ceilings only at an adapter seam: Mycelix remains canonical for admission, while Symthaea and domain systems continue to own their proof/model/measurement semantics.
5. After a stable contract, benchmark projection and derivation non-amplification over long evidence chains. Check that caching, replication, and serialization round trips cannot alter the ceiling.

## Nonclaims

No truth, factual accuracy, legal ownership, causal identification, source independence, trusted time, calibration, evidence admission, repository governance, authorization, deployment or production safety is established. Passing synthetic vectors qualifies only this reference behavior.