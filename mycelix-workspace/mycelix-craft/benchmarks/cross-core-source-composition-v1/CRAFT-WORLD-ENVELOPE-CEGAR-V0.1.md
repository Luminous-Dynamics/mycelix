# Craft Finite-Domain World Envelope and CEGAR V0.1

**Status:** bounded research/reference implementation  
**Tracking:** [issue #6867](https://github.com/Luminous-Dynamics/mycelix/issues/6867)  
**Composition:** CDIE #6865 / PR #6866; benchmark #6845 / PR #6847

## 1. Research question

Does Craft's abstract uncertainty representation conservatively contain every concrete world allowed by its declared finite-domain model? Can it refine the representation to remove spurious decision ambiguity without dropping any concrete compatible world?

For a declared concrete compatible set \(W\) and the concretization of abstract cells \(\gamma(A)\), the required finite-domain invariant is:

\[
W \subseteq \gamma(A)
\]

An under-approximation is rejected. A coarse over-approximation may create false ambiguity; it must not create a false-safe conclusion.

## 2. Bounded v0.1 domain

The initial oracle uses only finite discrete dimensions and exhaustively enumerates their Cartesian product, with a 100,000-assignment guard.

- Dimensions declare stable IDs and finite values.
- Forbidden-combination constraints declare partial assignments excluded from the concrete compatible set, plus evidence/assumption references.
- Every full-domain assignment has an explicit action-loss and hard-violation record. This includes impossible assignments because they may appear as spurious abstract states.
- An abstract cell is a partial fixed assignment minus explicit excluded patterns.
- The union of abstract cells must contain every assignment that does not match a declared forbidden constraint.
- An abstract assignment that violates a declared constraint is spurious, not a concrete world.
- A refinement may add only a declared constraint's forbidden pattern to a selected cell.
- The verifier re-enumerates after refinement and rejects any loss of concrete coverage or change in concrete decision semantics.

The result separates concrete and abstract decision summaries. It reports a spurious decision-flip witness when an impossible abstract assignment recommends a different optimal action from a concrete world.

## 3. CEGAR loop

1. Start with a sound but coarse abstraction, often one cell covering the entire finite domain.
2. Compute the concrete compatible set from declared constraints.
3. Verify every concrete assignment is represented by at least one abstract cell.
4. Calculate decision summaries over both concrete and abstract sets.
5. If a spurious assignment causes a decision flip, retrieve the declared constraint that excludes it.
6. Refine a selected cell by applying that exact constraint.
7. Recheck coverage and require the concrete set and concrete decision summary to remain unchanged.
8. Stop when spurious ambiguity is removed, no admissible refinement remains, or a resource budget is exhausted. The last condition must return unresolved/NotEvaluated, never a narrower world set.

A refinement can reduce spurious states without resolving a genuine concrete decision disagreement. The second frozen fixture tests exactly this distinction.

## 4. Proof and receipt outputs

The evaluator reports:

- declared full-domain assignment count;
- concrete compatible-world count;
- abstract-world count and spurious-world count;
- whether abstract coverage is sound within the declared finite model;
- concrete and abstract robust-safe, invariant-optimum, minimax-loss and minimax-regret summaries;
- the smallest deterministic spurious decision-flip witness, including its assignment, competing action, concrete comparison assignment and violated constraint IDs.

A refinement receipt binds the domain/world digest, before/after abstraction-state digests, cell and constraint IDs, before/after abstract and spurious counts, and whether concrete semantics were preserved.

## 5. Adversarial fixtures

- **CWE-001 — spurious decision flip:** an impossible high-hazard/healthy-sensor combination creates an artificial action disagreement in the coarse abstraction. Applying the declared constraint removes the spurious assignment and reveals the concrete invariant action.
- **CWE-002 — genuine decision flip:** a real concrete disagreement remains after the same refinement. It must remain non-identified.

Tests additionally reject under-approximation, defaulting an unobserved dimension to a favorable value, unsupported narrowing, contradictory constraints that produce an empty concrete set, modified fixture ground truth, unresolved items with a stronger scope label, malformed JSON and repeated/unknown refinements.

## 6. Qualification targets

Proposed acceptance targets, not achieved field results:

- 100% coverage of every declared concrete assignment by the abstract union.
- Zero accepted under-approximations.
- 100% preservation of the concrete world set and concrete decision semantics after every refinement.
- 100% detection of the frozen spurious and genuine decision-flip cases.
- Zero false global claims when domain/source coverage is NotEvaluated.
- Report abstraction ratio, spurious-world rate, false ambiguity rate, refinement count, decision-ambiguity gain, enumeration work and worst-case regret separately.

## 7. Limits

`abstract_coverage_sound=true` proves only that the abstract cell union covers all compatible assignments generated from the supplied finite domains and declared constraints. It does not establish that the dimensions, values, constraints, source universe or loss model cover real-world possibilities.

`fixture_exact` is pinned to two synthetic domain/world digests and means exhaustive only for those frozen fixtures. `declared_assumptions` is conditional on the declared domain and constraints. `not_evaluated` never supports a global decision claim.

This module is not a general SMT solver, interval abstraction engine, production world-model, evidence authenticator, authorization service or executor. The constraint references are not cryptographically verified by this oracle.
