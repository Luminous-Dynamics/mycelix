# FSRE LP partial-identification E0 addendum

**State:** research implementation and exact known-answer fixtures. This is not a production solver or a field-validated population estimator.

## 1. Three increasingly rich model classes

### E0-A: finite-world extrema

For an explicit finite set of compatible worlds and query values \(q_w\), the unconstrained simplex:

\[
\min/\max \sum_w p_w q_w
\quad\text{s.t.}\quad \sum_w p_w=1,\ p_w\ge0
\]

has sharp bounds at the minimum/maximum world value. FSRE now builds this as a rational LP, provides an extremal one-hot primal witness and an exact dual certificate, then verifies both independently.

### E0-B: constrained distribution / partial identification

Add observed aggregates or moment constraints:

\[
\ell_k\le\sum_w a_{kw}p_w\le u_k.
\]

The resulting feasible probability distributions encode all worlds compatible with the declared constraints. The lower/upper query extrema are the sharp identified interval **within that declared finite world set and those constraints**.

A synthetic known-answer case has world query values \(0,10,20,30\) and the aggregate fact that exactly half the mass lies in the first two worlds. Exact rational certificates prove bounds \([10,20]\). Without the aggregate fact, the upper bound is 30. The constraints reduce uncertainty; the certificate proves the result for the encoded model, not the truth of the supplied aggregate.

### E0-C: explicit infeasibility

A failed solver search is not an infeasibility proof. For the canonical model \(Ax\le b,\ x\ge0\), a Farkas certificate supplies \(y\) with:

\[
y\ge0,\quad A^\top y\ge0,\quad b^\top y<0.
\]

Then no nonnegative \(x\) can satisfy all declared rows. FSRE's verifier checks each condition with exact rational arithmetic.

The known-answer negative control supplies two conflicting aggregate facts for the same group mass: at least \(3/4\), and at most \(1/2\). The Farkas combination has \(A^\top y=0\) and \(b^\top y=-1/4\), proving infeasibility of that declared model.

## 2. Claim boundaries

The LP layer must distinguish these outcomes:

- exact primal + exact dual equality: optimum proven for the declared LP;
- exact feasible primal and dual with a gap: lower and upper bounds only;
- one exact feasible side: one-sided bound only;
- valid Farkas certificate: declared LP infeasible;
- neither optimization nor Farkas certificate verified: no bound/infeasibility claim.

A solver's floating-point optimal status, a numerical residual, or search exhaustion is never a substitute for those exact certificates.

None of these certificates authenticates:
- the source records or issuer;
- rights or legal authority to use the source;
- causal assumptions or the population frame;
- completeness of the finite world set;
- independence of registers;
- physical truth or field outcome.

The evidence/authority gates remain separate from the LP mathematics.

## 3. E0 tests now included

- Exact rational normalization, arithmetic and overflow rejection.
- Primal/dual optimality equality.
- Gap-only and one-sided certificate outcomes.
- Exact equality-column reduction and reduced/unreduced feasibility/objective parity.
- Finite-world minimum/maximum parity and extremal witness.
- Negative and rational query values.
- Determinism under row/section permutations.
- Contradictory moment intervals rejected by the builder.
- Unknown world references rejected.
- Farkas infeasibility proof for conflicting aggregate facts.
- Negative multiplier and unknown-row negative controls.
- Exhaustive enumeration of all query-value vectors of length 1–4 over \(\{-2,-1,0,1,2\}\), requiring exact minimum/maximum and certificate equality.

The exhaustive test includes \(5+25+125+625=780\) finite vectors and checks both extrema for each. It is a small exact correctness corpus, not a scalability benchmark.

## 4. Next exact-reduction frontier

The 2026 literature points to two complementary improvements:

1. **Query-implied partial orders.** Rossetto and Antonucci show that a counterfactual query induces a partial ordering over relevant variables, which can parameterize tight LP bounds without a fully specified causal graph:
   https://proceedings.mlr.press/v343/rossetto26a.html

2. **Constraint-aware canonical-domain reduction.** Choe et al. reduce canonical counterfactual state spaces by quotienting out degrees of freedom irrelevant to the optimization:
   https://proceedings.mlr.press/v337/choe26a.html

FSRE's current exact identical-column reducer merges only variables with identical objective coefficient and every matrix coefficient. The next generalization must:
- derive equivalence under every objective and constraint functional, not just one target query;
- preserve links to original worlds, variables, evidence and assumption IDs;
- prove that the projection preserves the complete feasible objective interval;
- compare reduced and unreduced models on the exact finite oracle corpus;
- fail closed whenever a supposedly irrelevant dimension affects a constraint, safety floor or action ranking.

Do not aggregate worlds merely because they have the same current query value. They must agree on all constraints and every objective that matters to the decision.

## 5. HiGHS adapter boundary

The next E1 step is to let HiGHS propose primal/dual vectors for large sparse LPs and independently check them here. HiGHS explicitly uses floating-point feasibility and optimality tolerances; its documentation notes that relative tolerance conditions can mislead, and records default KKT tolerances of \(10^{-7}\):
https://ergo-code.github.io/HiGHS/stable/guide/kkt/

The pipeline must therefore:
1. freeze the exported LP bytes, solver version, options and environment;
2. retain raw floating vectors and residuals;
3. attempt bounded-denominator rational reconstruction;
4. pass reconstructed vectors through this exact verifier;
5. report numeric-only outcomes when exact rational feasibility fails.

The solver is a proposal engine. The exact certificate verifier is the claim gate.

## 6. Benchmark targets

Research targets, not achieved:
- zero false optimum claims;
- zero false infeasibility claims;
- 100% exact parity with finite-world enumeration on overlapping E0 cases;
- zero reduction-induced bound changes;
- zero false point-identification under incomplete assumptions;
- 100% model/export preservation for bounds, rows, variables and assumption IDs;
- report state/column/row/nonzero reduction, arithmetic bit growth, solve time, verification time and memory separately.

For scale, use increasing synthetic sparsity and degeneracy families, then matched HiGHS baselines. Do not compare solver runtime without input hashes, hardware, solver version/options and equivalent feasibility/quality targets.
