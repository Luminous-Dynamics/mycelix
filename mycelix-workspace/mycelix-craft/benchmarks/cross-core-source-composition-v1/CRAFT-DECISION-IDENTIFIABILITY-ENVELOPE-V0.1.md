# Craft Decision Identifiability Envelope V0.1

**Status:** research/reference oracle; synthetic qualification only  
**Parent:** Craft end-to-end assurance benchmark #6845 / PR #6847  
**Tracking:** #6865  
**Non-goal:** production authorization, autonomous execution, or source-truth verification

## 1. Research question

Given admissible evidence that leaves several possible worlds consistent with the accepted decision frontier, which actions remain safe and justified across all those worlds? When no single action is optimal everywhere, what is the least-regret robust compromise, and what remains unidentifiable?

The benchmark unit remains a decision episode. This layer makes the decision's dependence on unresolved evidence explicit.

## 2. Formal contract

Let $A$ be the declared finite action set and $W(E)$ the finite set of worlds declared compatible with evidence $E$. Each world $w$ supplies non-negative loss $L(a,w)$ for every action and a hard-safety violation set $H(w) \subseteq A$.

Per-world safe actions:

\[
S(w) = A \setminus H(w)
\]

Robust-safe actions:

\[
A_{safe} = \bigcap_{w \in W(E)} S(w)
\]

Per-world optimal actions:

\[
O(w) = \operatorname*{argmin}_{a \in S(w)} L(a,w)
\]

Actions optimal in every represented world:

\[
A_{identified} = \bigcap_{w \in W(E)} O(w)
\]

For each robust-safe action, report:

\[
L_{max}(a) = \max_{w \in W(E)} L(a,w)
\]

\[
R_{max}(a) = \max_{w \in W(E)} \left[L(a,w) - \min_{b \in S(w)} L(b,w)\right]
\]

Minimax regret is $\operatorname*{argmin}_{a \in A_{safe}} R_{max}(a)$. Minimax loss is separately $\operatorname*{argmin}_{a \in A_{safe}} L_{max}(a)$.

These are not interchangeable. A minimax-regret action is a robust compromise; it is not an action that is optimal in every world. An empty $A_{identified}$ means there is no invariant optimum. It does not mean the system must choose the minimax action: the safe fallback and human/institutional decision path remain authoritative.

**No probabilities are assigned to worlds.** The set $W(E)$ is an epistemic possibility set, not a probability distribution. Loss values are supplied by the benchmark case and are not validated by this oracle.

## 3. World-set coverage is a separate proof obligation

A correct calculation over an incomplete possibility set can still produce a false decision claim. Every input therefore declares one coverage basis:

| Basis | Permitted interpretation |
|---|---|
| `fixture_exhaustive` | Exhaustive only for the frozen synthetic fixture whose generator/ground truth is separately checked |
| `declared_set` | Conditional result within a named assumption set; no global real-world identification |
| `not_evaluated` | Numerical within-set envelopes may be shown, but no global identification claim is emitted |

A finite list supplied by a caller does not prove that it contains every world compatible with real evidence. Source-universe completeness, observation-model coverage, and world-generation coverage must be qualified upstream. This oracle never upgrades them.

The fallback action is mandatory and must be hard-safe in every listed world. If not, input validation fails closed.

## 4. Hard safety and unknowns

Hard-safety violations are non-compensatory. An action unsafe in any listed world is excluded from $A_{safe}$, minimax-loss and minimax-regret candidates. Low average loss, high confidence, high expected value, or low operational cost cannot restore it.

Unknown, unavailable, disputed, stale, out-of-domain or selectively omitted evidence must be represented upstream as constraints on $W(E)$ or the coverage basis. It must never be silently converted into a favorable state. This module does not infer missingness semantics itself.

## 5. Composition boundaries

- **#6845 / #6847:** supplies decision-episode, source, frontier and adversarial fixture context.
- **GCPN #5668:** owns missing-outcome ascertainment, positivity, HT/Hájek and partial-identification estimators. CDIE consumes a declared possible-world set; it does not duplicate those estimators.
- **#6858 sequential EVSI / #6862 robust EVSI:** optimize observations under declared probabilistic or model-ambiguity assumptions. CDIE evaluates action invariance without assigning probabilities to epistemic worlds. The modules should later cross-check acquisition value: observations are useful when they shrink the possibility set enough to change or identify the decision.
- **#6859 offline verifier:** should preserve the world-set digest and coverage proof obligation in the portable bundle.
- **#6835 DBC / #6860 EAP:** the result may inform a DBC and an advisory EAP; it does not authorize evidence collection or an operational effect. Prior DBC, EAP and post-acquisition DBC remain distinct immutable receipts.

## 6. Frozen v0.1 cases

1. **CDIE-001 invariant action:** uncertainty remains, but repair is optimal in every exhaustive fixture world.
2. **CDIE-002 no invariant optimum:** repair and hold/sample preferences differ by world; minimax regret is reported as a compromise, not identified optimum.
3. **CDIE-003 hard-safety veto:** release has low nominal loss but is unsafe in one world; hold remains the only robust-safe action.
4. **CDIE-004 unqualified world set:** repair is best in every listed world, but coverage is NotEvaluated and no global claim is emitted.

The oracle also tests action/world permutation invariance, world-set expansion monotonicity, malformed/duplicate inputs, strict JSON parsing and an independently written exhaustive small-domain reference.

## 7. Metrics and proposed targets

Targets are proposed acceptance criteria, not results:

- 100% exact agreement with independent exhaustive enumeration on frozen small domains.
- Zero unsafe action in robust-safe or identified sets.
- Zero global identification claim when world-set coverage is conditional or NotEvaluated.
- 100% retention of synthetic ground-truth worlds by the frozen generator for fixtures declared exhaustive.
- Report identified-decision rate, abstention/non-identification rate, worst-case regret, worst-case loss, robust-safe set size, possibility-set cardinality, and coverage basis separately.
- No aggregate score may compensate for any hard-safety or false-identification failure.

## 8. Limits

The Go oracle is a bounded reference calculation. It does not authenticate sources, establish completeness of the real source universe, validate loss functions, prove probability calibration, establish legal authority, prove independence, execute an action, or observe a physical outcome. Local Go tests, hosted exact-head workflow results, synthetic benchmark outcomes and field validation must be reported separately.
