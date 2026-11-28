# Craft Decision-Separating Observation Profile V0.1

**Status:** research/reference oracle; finite deterministic outcome maps only  
**Tracking:** [issue #6870](https://github.com/Luminous-Dynamics/mycelix/issues/6870)  
**Composition:** CDIE #6865, world-envelope CEGAR #6867, EAP #6860, sequential EVSI #6858, robust EVSI #6862

## 1. Research question

Given evidence-compatible worlds and a candidate observation, will every possible observation result leave a safe action that is optimal across all worlds consistent with that result?

This is a decision-resolution question, not an entropy question. An observation can have many distinct outcomes while failing to resolve an action. Conversely, a small binary observation can completely resolve a consequential choice.

## 2. Formal contract

Let W be a finite declared world set, A a finite action set, L(a,w) a non-negative loss, and H(w) the hard-safety violations in world w.

A candidate observation is a total deterministic mapping:

\[
y_o: W \rightarrow Y
\]

For each nonempty outcome branch y:

\[
W_y = \{w \in W : y_o(w)=y\}
\]

Within a branch, compute the robust-safe actions and the intersection of each world's safe argmin action set. The branch is decision-identified when at least one action remains both safe and optimal in every world in that branch.

The observation is **universally decision-separating within the listed worlds** only when every non-empty branch is decision-identified. The optimal action may differ between branches; the policy can condition its choice on the observed result.

This is not a global claim when world-set coverage is not qualified. v0.1 accepts only 'declared_set' and 'not_evaluated' scopes. The former is conditional on the supplied world set; the latter never upgrades to a global assurance claim.

No probabilities are attached to worlds or outcomes. 'identified_world_count_share' is a combinatorial numerator/denominator over the enumerated fixture; it is not an estimated probability, propensity or expected value.

## 3. Branch regret

For each branch, the oracle reports its own minimax loss and minimax regret over branch-robust-safe actions. It then reports the worst residual minimax regret across all outcome branches:

\[
R_o = \max_{y \in Y} \min_{a \in A_{safe}(W_y)} \max_{w \in W_y}
\left[L(a,w)-\min_{b \in S(w)}L(b,w)\right]
\]

The pre-observation minimax regret is reported separately. Their difference is a deterministic finite-world diagnostic, not EVSI: it has no outcome probabilities and does not subtract acquisition costs. Cost, delay and burden are separate metadata fields.

## 4. Why this complements active feature acquisition

The IJCAI 2026 survey by Aronsson, Rahbar and Chehreghani frames Active Feature Acquisition as a sequential POMDP and organizes methods into embedded cost-sensitive, model-based, model-free and hybrid methods. It also identifies evaluation under realistic missingness/logging and guarantees for non-myopic acquisition as open challenges. Craft should benchmark against that body of work rather than claim sequential acquisition itself is novel.

The Craft differentiator is to attach proof obligations to each observation partition:
- current world-set digest and coverage basis;
- total outcome mapping over every listed compatible world;
- branch-specific robust-safe and invariant-optimum sets;
- branch residual regret;
- failure-domain overlap and independence status kept separate;
- an auditable result that can be carried into an advisory EAP.

## 5. Frozen cases and adversarial controls

- **DSOP-001:** a binary lab outcome separates two worlds whose optimal actions differ; each branch has an invariant safe action, and the adaptive policy chooses different actions for the two results.
- **DSOP-002:** hard safety is branch-specific; 'release' may be safe in a clear branch but must be excluded in the unsafe branch.
- Same result for every world: no decision separation if the branch retains competing optima.
- Missing mapping for a compatible world: observation model incomplete and no separation claim.
- Shared failure-domain overlap: report 'SHARED_FAILURE_DOMAIN'; do not infer independent corroboration from distinct result labels.
- Unknown world IDs, duplicate IDs, empty outcomes and invalid costs are rejected.
- Input permutation does not alter canonical results.

## 6. Comparator plan

Matched baselines:
- mandatory fixed observation schedule;
- domain-expert next-best-observation checklist;
- random-safe candidate under equal constraints and resource budget;
- EIG greedy;
- one-step EVSI;
- sequential finite-horizon EVSI #6858;
- robust model-ambiguity EVSI #6862;
- active feature acquisition POMDP methods and cost-sensitive predictors from the IJCAI 2026 taxonomy.

DSOP is not intended to beat POMDPs on expected utility when calibrated transition/outcome probabilities are available. It supplies exact branch-wise decision-separation diagnostics when such probabilities are unavailable or not justified. EVSI remains responsible for expected decision value under declared probability models; robust EVSI remains responsible for declared model ambiguity.

## 7. Targets (proposed, not achieved)

- 100% exact agreement with independent exhaustive branch enumeration on frozen small domains.
- Zero universal-separation claims when any compatible world lacks an outcome mapping.
- Zero unsafe actions in any branch robust-safe set.
- Zero inference of independence from outcome diversity.
- 100% canonical parity under world/action/observation permutation.
- Report universal branch-separation rate, identified-world count share, worst residual branch regret, branch count, incomplete-map rate, acquisition cost, delay and burden separately.

## 8. Limits

The observation mapping is deterministic and supplied by a fixture or declared model. v0.1 does not validate sensor likelihoods, outcome calibration, source completeness, causal effect, legal authority or physical outcome. Branch frequencies are not probabilities. Shared failure domains can be detected only to the extent supplied; independence remains NOT_EVALUATED unless a separate lineage proof obligation is qualified. The oracle is not an acquisition executor.

## References

- Aronsson, L., Rahbar, A. & Chehreghani, M. (2026), *A Survey on Active Feature Acquisition Strategies*, IJCAI 2026, DOI: [10.24963/ijcai.2026/860](https://doi.org/10.24963/ijcai.2026/860).
- Yamao et al. (2026), *Robust Decision-Focused Learning via Worst-Case Regret Minimization*, UAI 2026, [PMLR 337](https://proceedings.mlr.press/v337/yamao26a.html). This is related work on robust decision loss under uncertainty sets and distribution shift; Craft should compare with it on a matched decision task, not rebrand the method.
