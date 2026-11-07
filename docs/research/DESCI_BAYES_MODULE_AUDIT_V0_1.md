# Mycelix DeSci Bayesian Module: Source Audit and Qualification Boundary v0.1

**Status:** Source audit; remediation pending  
**Date:** 2026-11-07  
**Related issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5910  
**Primary source:** mycelix-desci/src/core/src/bayesian.rs  
**Export:** mycelix-desci/src/core/src/lib.rs

## 1. Executive summary

The DeSci crate exposes a Bayesian belief-network API, but the current implementation should be treated as an experimental/legacy prototype, not a qualified posterior-inference engine.

This audit is based on reading the source on 2026-11-07. It is not a runtime test report. No local Cargo compile, unit-test pass, CI pass, or downstream-consumer audit is claimed here.

The active Mycelix repository already contains more advanced domain-specific Bayesian experimental-design work (including the Terra Preta sequential design and covariance/SBC workstreams). Those separate programmes should not be conflated with this legacy DeSci module.

**Immediate boundary:** do not use this module's current outputs for authoritative claim promotion, scientific conclusion, trust/credential decisions, or dispute outcomes until its semantics are repaired and independently qualified.

## 2. Source-level findings

### 2.1 Sampling is not stochastic

The likelihood-weighting loop calls a helper that derives a floating-point value only from a node UUID. For a fixed node ID, each loop iteration therefore sees the same pseudo-random value.

Consequences:
- Increasing sample_count does not create independent or even varied draws.
- The result does not have ordinary likelihood-weighting Monte Carlo semantics.
- No RNG algorithm, seed, stream, effective sample size, or Monte Carlo uncertainty is reported.

A deterministic UUID hash may be useful as a test fixture, but it is not a sampling algorithm. Replace it with an explicit seeded PRNG for reproducibility, then report uncertainty; use a cryptographic RNG only if a security-specific use case actually requires one.

### 2.2 The belief-propagation path is not sum-product BP

The current loopy belief-propagation method delegates to a message routine that returns a scalar derived from the source node's current belief. It does not compute the factor-to-variable sum over the other variables in the factor scope.

Consequences:
- CPT factors and neighboring-variable messages do not participate in the required sum-product update.
- The result cannot be interpreted as a standard BP marginal.
- Existing tests that merely check the returned value is within [0,1] do not qualify the algorithm.

Either implement the actual factor-to-variable and variable-to-factor equations with known-answer tests, or remove/rename the BP API until that is done.

### 2.3 Partial evidence probability is not correctly marginalized

The evidence-probability routine multiplies local conditional terms and falls back to a node prior if a parent is not observed. A local prior substitution does not generally marginalize an unobserved parent over the joint model.

The implementation needs a proper exact reference calculation for partial evidence, such as exhaustive enumeration on tiny networks or variable elimination.

### 2.4 D-separation is only a simplified reachability approximation

The source labels its d-separation procedure simplified. General d-separation requires handling ancestral structure, collider activation, and conditioning on descendants of colliders. The current test exercises the function but discards the expected answer.

Do not expose its result as a correct conditional-independence certificate. Replace it with a reviewed algorithm and a truth-table suite covering chains, forks, colliders and collider descendants.

### 2.5 Graph construction can hide structural errors

Current edge construction can silently ignore unknown node IDs, does not reject directed cycles, and reinitializes child CPTs when parents are added. That can discard probabilities already specified by callers.

Required:
- typed errors for unknown endpoints and duplicate edges;
- cycle detection and a stable topological ordering;
- explicit parent ordering in each CPT;
- no implicit CPT reset; require a deliberate versioned migration when a parent set changes.

### 2.6 CPT values are silently clamped

Probability setters clamp values into [0.001, 0.999]. This prevents exact probabilities 0 and 1, silently changes caller-supplied models, and hides malformed values.

Use explicit validation:
- finite values in [0,1];
- exact 0 and 1 permitted;
- every conditional row sums to one within the declared tolerance;
- invalid values return typed errors instead of being modified.

Hard evidence should be represented by a delta likelihood or equivalent hard constraint, not an arbitrary near-certain value such as 0.999.

## 3. Required remediation sequence

### P0 — contain claim and API risk

1. Mark the module Experimental / Legacy in module and type documentation.
2. Document that current sampling, BP, partial evidence and d-separation outputs are not qualified.
3. Search the complete workspace and known downstream consumers before changing public exports.
4. Add a compile-time/deprecation or feature boundary only after call-site impact is known; do not silently break consumers.

### P1 — repair mathematical semantics

1. Replace panic/silent behavior with fallible constructors and typed errors.
2. Reject missing node IDs, cycles, duplicate edges and inconsistent parent/CPT ordering.
3. Preserve previously specified CPT rows or require explicit migration.
4. Implement proper CPT normalization and exact 0/1 probabilities.
5. Implement exact enumeration for tiny networks as the reference oracle.
6. Implement variable elimination for bounded discrete networks.
7. Implement actual seeded likelihood weighting or another named approximate inference method with Monte Carlo diagnostics.
8. Implement actual sum-product BP if needed, with factor scopes, messages and convergence status.
9. Implement d-separation with collider/descendant correctness.
10. Return explicit zero-evidence-mass / inconsistent-model results; never fabricate a uniform posterior.

### P2 — independent benchmark and differential qualification

Use pinned versions of pgmpy as an external discrete-BN reference. Do not treat the Python implementation as infallible; compare exact known-answer cases and manually derived fixtures.

Required fixtures:
- Asia network posterior queries;
- chain, fork and collider networks;
- collider opened by conditioning on a descendant;
- multiple parents with explicit CPT ordering;
- partially observed networks with hidden-parent marginalization;
- impossible evidence;
- exact 0/1 probabilities;
- disconnected components;
- cyclic graph and unknown-endpoint rejection;
- repeated seeded draws and different-seed stream checks;
- convergence against exact posterior values within Monte Carlo confidence intervals.

Metrics:
- posterior total-variation and maximum absolute error;
- evidence probability / log evidence error;
- calibration and proper predictive log/Brier scores;
- Monte Carlo effective sample size and uncertainty;
- BP convergence residual and iteration count;
- d-separation truth-table agreement;
- runtime, memory, and induced width;
- explicit failure/unsupported rates.

### P3 — connect to Mycelix's evidence and governance layers

A posterior may inform a claim, but does not make the claim true or legitimate.

- Evidence provenance and dependence must be supplied by the evidence layer.
- Unknown ancestry cannot become statistical independence by default.
- Statistical confidence cannot override conflicting independent evidence or governance rules.
- Model-generated evidence cannot authenticate its own assumptions.
- Inference receipts must bind model/evidence digests, algorithm/version, RNG/seed if stochastic, diagnostics and claim ceiling.
- A model update invalidates dependent inference receipts.

## 4. Cross-project relation

- Symthaea #10889 defines the cross-project factor-graph correctness and interoperability programme.
- Symthaea #10404 covers evidence dependence / false corroboration; its lineage semantics are relevant to Mycelix fusion.
- Mycelix #5601, #5632 and #5648 cover sequential Bayesian design, covariance-aware likelihoods, discrepancy and posterior calibration. They are separate, more advanced research tracks.
- Symtropy #4017 and #4027 cover correlated sensing and exact finite-horizon passive diagnosis.
- Sol-Atlas #582 and #583 cover event-factor scope and continuous spatial detection likelihoods.

Do not replace those specialized systems with this module. Reuse a qualified shared interface when available.

## 5. Acceptance criteria

- All tiny discrete reference posteriors match an independent exact enumerator.
- Every malformed graph, invalid CPT, cycle and unknown edge endpoint is rejected explicitly.
- Zero/one CPT entries are preserved.
- Fixed-seed sampling is reproducible; different seeds generate different streams; Monte Carlo error is reported.
- Likelihood weighting matches known posterior expectations within preregistered uncertainty limits.
- Sum-product BP matches exact inference on trees.
- Loopy BP is labelled approximate and never reports success solely because values remain in range.
- Partial evidence is correctly marginalized.
- D-separation results match the complete chain/fork/collider/descendant test matrix.
- No output is elevated to an authoritative DeSci claim solely because this module returned a number.

## 6. Claim ceiling

This source audit identifies implementation gaps; it does not prove that the module is used by production call sites. It does not claim the module has passed or failed runtime tests. The current outputs are not qualified posterior probabilities, Monte Carlo estimates, conditional-independence certificates, or evidence-authenticity decisions.

**Decision:** quarantine its authority-bearing use, repair the exact semantics first, and use differential known-answer tests before exposing any result as a qualified Bayesian inference.


## 2.7 Duplicate mirrored workspace copy — additional audit finding (2026-11-07)

The same DeSci Bayesian source exists at two paths:

- `mycelix-desci/src/core/src/bayesian.rs`
- `mycelix-workspace/mycelix-desci/src/core/src/bayesian.rs`

Both files currently have the same Git blob SHA (`3593a1788602f9bd10b59f15aaea139d6434abf7)) and identical contents. Both corresponding DeSci Cargo manifests declare standalone workspaces; their Cargo.lock files are also identical. The READMEs describe the same architectural refoundation and build boundary.

The outer `mycelix-workspace/Cargo.toml` includes `mycelix-desci/apps/leptos` as a workspace member, but does not list the nested Rust DeSci workspace itself as an outer member. Therefore, the two Rust workspace roots must be treated as separate build/test entry points until maintainers formally designate one canonical source and document the mirror-generation/synchronization process.

Repository code-index searches for `BayesianInference`, `BeliefNetwork::new`, `loopy_belief_propagation`, `likelihood_weighting`, and `d_separated` returned the two source copies, their public `lib.rs` re-exports, and in-module tests. No external invocation appeared in those searches. This is useful negative evidence, not proof that no consumer exists; generated, path-qualified, downstream, or non-indexed call sites may remain.

**Remediation implication:** the call-site and build-context audit must cover both paths. Do not fix only one copy. Either designate one canonical DeSci Rust workspace and remove/redirect the duplicate through a reviewed migration, or require both copies to remain byte-identical with an automated digest check and both standalone workspace test commands. Do not delete or consolidate the mirrored workspace until its consumers and build purpose are confirmed.
