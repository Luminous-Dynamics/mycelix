# PRAXIS-NRES-009: Joint service-failure distribution verification

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. No real outage dependency, operational probability or public-service readiness has been measured.  
**Fixture:** `examples/courses/civic-resilience-joint-service-forecast-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-joint-service-forecast-conformance-001.json`.  
**Schema:** `schemas/nres/nres-joint-service-forecast-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/joint.rs`.

## 1. Research shift: from marginal risk to joint-system risk

NRES-008 asks whether a model predicts each service-floor breach probability accurately. That is necessary but not sufficient for a system of interdependent services. A forecast can have reasonable marginal probabilities while assigning the wrong probability to critical combinations, such as simultaneous electricity and communications failure or multi-service disruption involving health logistics.

NRES-009 evaluates the **full categorical distribution over all binary breach combinations** for a frozen set of services and horizon. For four services there are 16 possible states; each forecast must provide one probability for every state. This makes dependence information explicit and independently scoreable.

The evaluator does not generate outages or infer causal edges. It verifies the joint distribution emitted by a candidate and compares it with the evaluator-owned observed breach vector.

## 2. Why multiple proper scores are needed

The multivariate forecasting literature cautions that a single proper score may not reveal every relevant misspecification. Pic et al. (2025) develop aggregation/transformation-based multivariate scoring rules and discuss how variogram scoring can target dependence structure; they also show that different verification tools can expose different forecast defects: https://ascmo.copernicus.org/articles/11/23/2025/.

NRES-009 therefore reports complementary scores:

1. **Multiclass Brier score:** the sum of squared errors over all possible joint states.
2. **Joint log loss:** negative log probability assigned to the observed complete breach combination. If that probability is zero, the loss is infinite and is reported as null with an explicit zero-support count—not silently clipped.
3. **Energy score:** (E\|X-y\|_2 - \tfrac12 E\|X-X'\|_2), using the discrete joint distribution.
4. **Variogram score:** mean squared error between predicted and observed pairwise discordance probabilities.
5. **Pairwise joint-breach Brier score:** evaluates (P(X_i=1,X_j=1)) for each unordered service pair.
6. **Marginal Brier rows:** retain the NRES-008 per-service breach-probability view.

Energy and variogram scores are complementary, not interchangeable. A good aggregate energy score must not excuse poor dependence forecasts; a good variogram score does not excuse poor marginal probabilities. The 2026 review by Waghmare and Ziegel surveys the foundations and families of proper scoring rules: https://www.annualreviews.org/content/journals/10.1146/annurev-statistics-042424-050626.

## 3. State-space contract

The public fixture has four services, each assigned a stable bit index:
- bit 0: electricity supply;
- bit 1: communications access;
- bit 2: water-service continuity;
- bit 3: health-logistics continuity.

A bit value of one means the service breached its declared floor at least once in the fixed 24-hour horizon. State mask zero means no service breached; state mask 15 means all four breached. There are exactly (2^4=16) states.

For every case:
- the evaluator supplies one observed joint state mask;
- the candidate supplies all 16 state masks exactly once;
- every probability is finite and in [0,1];
- probabilities must sum to one within (10^{-12});
- the evaluator independently derives the marginal and pairwise probabilities.

The public fixture has eight fictional forecast/observation pairs. This is enough to exercise the scoring implementation, not to establish calibration or a meaningful generalization claim.

## 4. Scoring definitions

Let (F) be the candidate distribution over binary vectors (X\in\{0,1\}^d), and let (y) be the observed vector.

**Multiclass Brier**
\[
S_B(F,y)=\sum_{s\in\{0,1\}^d}(p_s-\mathbb{1}[s=y])^2.
\]

**Joint log loss**
\[
S_L(F,y)=-\log p_y.
\]
If (p_y=0), the score is infinite; NRES-009 emits a null numeric mean and an explicit count of zero-probability observations.

**Energy score**
\[
S_E(F,y)=\mathbb{E}_F\|X-y\|_2-\frac12\mathbb{E}_{F,F'}\|X-X'\|_2.
\]

**Variogram score (unit weights, order one)**
\[
S_V(F,y)=\frac{1}{\binom d2}\sum_{i<j}
\left(\mathbb{E}_F|X_i-X_j|-|y_i-y_j|\right)^2.
\]

**Pairwise joint-breach Brier**
\[
S_J(F,y)=\frac{1}{\binom d2}\sum_{i<j}
\left(P_F(X_i=1,X_j=1)-y_i y_j\right)^2.
\]

All measures are reported independently. There is no weighted composite or single “resilience score”.

## 5. Industry comparison and benchmark protocol

Use identical scenario IDs, service order, horizon, outcome definitions and hidden observations for:
1. a product-of-marginals independent baseline (the baseline that assumes services fail independently);
2. a transparent empirical joint-frequency baseline, with smoothing fixed using training data only;
3. a candidate simulator/forecast distribution;
4. candidate ablations that remove common-cause or dependency information.

The independent baseline is especially important: it may estimate every marginal correctly while getting joint tail events wrong. NRES-009 is designed to make that error visible.

Report:
- joint log loss and multiclass Brier;
- energy and variogram scores;
- pairwise co-breach Brier by service pair;
- per-service marginal Brier;
- joint-event reliability by preregistered subsets (e.g. all-critical breach, power-and-comms co-breach);
- worst-case service subsets and zero-support events;
- scenario-seed uncertainty intervals, invalid runs and missing outputs.

Confirmatory testing requires independently generated hidden scenarios, repeated seeds, an independent oracle, a second independently implemented evaluator and third-party verification. Do not fit the joint distribution and evaluate it on the same public cases.

## 6. Cross-project ownership

- **Symtropy:** owns stochastic scenario generation, dependency propagation, and the candidate ensemble/joint distribution. It must identify model/build, seed, scenario/graph digests and assumptions.
- **Praxis/NRES:** independently validates the full state space and computes the proper scores. It does not generate the simulated outcomes.
- **Mycelix:** owns source qualification, evidence custody, provenance and authority. A simulated distribution does not become an observed probability by being serialized.
- **Sol-Atlas:** may provide evidence-qualified spatial/lifeline references, but geographic proximity does not prove service dependence or live capacity.

A future adapter should carry the joint state encoding, service bit mapping, sample/ensemble identity, probability mass, scenario and graph digests, forecast time/horizon and simulated-versus-observed claim state.

## 7. Conformance and limits

Hard integrity gates:
- 100% of state masks present exactly once;
- probability mass within the frozen (10^{-12}) tolerance;
- all observed masks valid for the service dimension;
- all 13 negative cases rejected;
- all golden score vectors reproduced within (10^{-12});
- zero composite/personal scores accepted.

These are structural and numerical conformance gates, not predictive-performance targets. The public fixture is synthetic and has only eight cases. No causal, operational, calibration, real-world readiness or SOTA superiority claim is made.

## 8. Reproducibility

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --joint
```

A successful run establishes only fixture consistency and deterministic scoring. Independent scenario custody and out-of-sample evidence are required for confirmatory claims.
