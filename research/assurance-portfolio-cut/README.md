# ASSURE-PORTFOLIO-CUT-001 — Threat-profile completeness and common-cause cut sets

**Status:** research specification v0.1  
**Tracking issue:** [#5695](https://github.com/Luminous-Dynamics/mycelix/issues/5695)  
**Baseline model:** [assurance portfolio composition #5659](https://github.com/Luminous-Dynamics/mycelix/pull/5659)  
**Base commit:** `198b810ce429a645345e87158c8b7a0fcaa4fcbe`  
**Claim ceiling:** conditional finite-model risk analysis, not a proof of universal independence

## 1. Motivation and boundary

The v0.1 composition model supports exact claim/subject binding, evaluator and profile digests, authority snapshots, evidence validity, 15 fault-domain dimensions, required pairwise role/dimension separation and explicit reporting of shared/unknown dependencies.

Its result deliberately means only that assessments meet one caller-declared evidence profile. That boundary is correct and must not be weakened.

This extension addresses two adjacent questions:

1. **Profile completeness:** when an external policy says a claim is independently verified, did that policy actually declare the necessary role-to-role separations and account for unknown/shared dependencies?
2. **Common-cause cut sets:** under explicit fault-injection semantics, what smallest set of shared domain failures could invalidate every required evaluator in the portfolio?

Do not add majority voting, trust scores, a global independence boolean or a stronger meaning to `MeetsDeclaredProfile`.

## 2. Threat-profile contract

Introduce a separately versioned, digest-bound contract evaluated before portfolio composition. It must be maintained/pinned independently of the portfolio under qualification.

The contract binds:
- claim class and claim-language ceiling;
- exact claim identifier and subject digest;
- required roles and exact evaluator/profile authorization where needed;
- mandatory pairwise separation obligations by role pair and fault-domain dimension;
- explicit `shared_accepted_with_rationale` dispositions where sharing is permitted;
- explicit `not_applicable_with_rationale` dispositions for dimensions excluded by the threat model;
- unresolved/unknown dimensions and the policy for handling them;
- exact policy/verifier executable and schema digests;
- effective/expiry interval and authorization/revocation snapshot where relevant.

Every dimension must be accounted for as one of:
- mandatory distinct;
- explicitly accepted as shared, with rationale and residual risk;
- not applicable, with rationale and a claim-specific scope explanation;
- unknown/unresolved (which blocks any tier requiring that dimension).

The profile verifier must reject a claimed assurance tier if any mandatory obligation is omitted, unknown, observed shared or unsupported. A digest that differs is not itself proof of independent organizational ownership; the profile only verifies declared identity evidence.

### Tier vocabulary

Avoid an unqualified `independent=true`. Use bounded, policy-named labels such as:
- `profile_complete_for_declared_threat_model`;
- `required_separations_observed`;
- `shared_dependencies_explicitly_accepted`;
- `profile_incomplete`;
- `profile_expired`;
- `profile_invalid`.

These are facts about contract coverage and observed identifiers, not guarantees that evaluators are truly independent.

## 3. Common-cause dependency graph

Represent each required evaluator role (r) as depending on a set of fault-domain keys:

`D_r = {(dimension, identity)}`

A key identifies one declared common-cause domain, not an authenticated real-world principal. A missing identity must remain unknown; do not replace it with a fresh per-evaluator identity.

For the first finite threat model, define a fault event as compromise/unavailability/corruption of one domain key. Under the explicit conservative assumption that compromising any one dependency domain can invalidate that evaluator's output, a set (C) compromises role (r) when:

`C ∩ D_r ≠ ∅`

A cut set compromises all required roles when it hits every role's dependency set:

`∀r, C ∩ D_r ≠ ∅`

A minimal cut set is a cut set for which no proper subset is itself a cut set.

This is a deliberately bounded fault model. Extend only with separately specified failure semantics for:
- dependencies whose compromise changes results but does not make the evaluator unavailable;
- common-mode faults that affect only a subset of functions;
- shared dependencies that provide data but cannot alter verifier execution;
- time-correlated compromise and revocation propagation;
- adversarial source acquisition versus accidental operational failure.

For each reported cut set, emit exact domain dimension/identity, affected roles, assumed compromise effect, omitted/unknown dependencies and the exact threat-model/profile digest. Never collapse results into one opaque score.

## 4. Independent reference algorithm

For small portfolios, write a deliberately simple reference enumerator over finite fault-domain keys:

1. Build the role-to-domain dependency sets from the admitted manifest.
2. Enumerate candidate fault-domain subsets in increasing cardinality.
3. Keep a candidate only if it intersects every required role's dependency set.
4. Discard any candidate containing an already reported cut set.
5. Report every minimal cut set and the resulting minimum cardinality.
6. Return `unknown/incomplete` rather than a numeric cut size if any required role has an unknown or incomplete dependency set.

The production research implementation and reference enumerator must not share their cut-set search helper. Compare outputs as canonical sets, not ordering-sensitive strings.

The model currently has 15 domain dimensions. Several dimensions may share the same digest value while remaining distinct dimension-qualified keys; this must be explicit in the schema. Evaluator executable and profile identities may also be included as dedicated keys, without conflating them with the remaining fault dimensions.

## 5. Frozen benchmark

### Portfolio sizes
- 2 roles (compatibility with #5659's existing sweep);
- 3 and 4 roles (partial overlap and multi-way shared domains);
- 5 roles (bounded scaling and cut-set enumeration cost).

### Domain patterns
- all roles share one implementation/specification/test source;
- distinct implementation, shared expected-answer lineage;
- distinct implementation and answers, shared toolchain/CI provider;
- partially shared source-evidence and input-acquisition lineages;
- all unique known identities;
- one or more unknown identities;
- explicitly accepted common domain with rationale;
- incomplete profile omitting a mandatory role/domain pair;
- not-applicable declaration without a rationale;
- stale/expired profile or authorization snapshot.

### Fault injections
- compromise one shared domain at a time;
- compromise selected pairs and small multi-domain sets;
- suppress or corrupt one evaluator output;
- corrupt shared expected answers or test corpus;
- stale/revoke one evaluator authorization;
- make evidence unavailable after a profile was admitted;
- substitute evaluator, profile, policy or verifier executable identity;
- duplicate/reorder assessments and domain declarations.

### Metrics
- profile completeness disposition and exact missing obligations;
- known/shared/unknown/not-applicable dimensions per role;
- all minimal cut sets and minimum cut cardinality (where known);
- affected role sets under each cut;
- false profile-complete and false independence-language promotion counts;
- false `MeetsDeclaredProfile` count relative to independent expected dispositions;
- enumeration time, memory, state/partition counts and cutoff status;
- all denominators, fixture/generator/profile/verifier digests and seeds.

Do not compare the number of cut sets across different graph sizes without reporting the exact graph and attack semantics. Do not interpret a larger minimum cut cardinality as a probability of security or assurance.

## 6. Acceptance gates

- 100% profile-verifier agreement with the independent expected result on the frozen finite panel.
- Zero incomplete/omitted/shared/unknown mandatory obligations promoted to the corresponding separation tier.
- Zero false profile-complete, false independence-language promotion or false `MeetsDeclaredProfile` outcomes on the enumerated panel.
- Cut-set output exactly matches the independent reference on enumerable portfolios.
- Unknown/incomplete dependency sets never yield a numeric guarantee or a positive independence tier.
- Reordering and duplicate assessment replay do not improve profile completeness or reduce the cut set.
- A change in the threat model, mandatory separation matrix, identity manifest or verifier executable changes the pinned digest and invalidates prior qualification receipts.
- Claim/report wording always carries the exact finite threat-model scope.

## 7. Relationship to #5659 and trust-root separation

Do not modify the normative ASSURE kernel or reinterpret #5659's composition disposition. This should be a separate research-only crate/tool and report format.

The profile contract must be independently owned and pinned. A candidate must not select or rewrite the expected threat model used to qualify itself. This is related to, but distinct from, [#4853: make trusted self-test evaluators independent of PR-controlled test code](https://github.com/Luminous-Dynamics/mycelix/issues/4853).

The profile verifier can establish that a supplied manifest covers a declared threat model. It cannot establish that the manifest is authentic, complete in reality, or free of unknown common causes. That limitation must be stated in every report.

## 8. Claim ceiling

A passing result establishes only that the exact finite manifest, profile rules and cut-set semantics were evaluated correctly. It does not prove that distinct digest identities correspond to independent teams or infrastructure, authenticate a trust root, establish truth, eliminate unknown shared failure modes, or confer production qualification or action authority.


## Executable v0.1 model

The standalone Rust crate in this directory implements the first finite research slice:

- The claimed tier resolves to mandatory evaluator roles and pairwise role/domain separation obligations from a **separately supplied, caller-pinned assurance policy**. The caller's expected policy digest is an independent function argument; both the policy object and candidate profile must match it. The candidate profile cannot provide or weaken its own mandatory separation list; a policy-digest mismatch and unknown tier are rejected. The trusted caller remains responsible for verifying that the supplied policy bytes actually hash to the pinned digest before constructing the policy value.
- The profile checker requires every policy-required role to be represented and each role manifest to account for all 15 dimensions. Missing/unknown dimensions make the profile incomplete for this v0.1 strict tier. Shared and not-applicable declarations require non-empty rationales. Profile validity uses the half-open interval `valid_from <= now < valid_until` when an expiry is present; empty or reversed windows are rejected.
- Required pairwise role/domain separations pass only when both sides supply known, unequal digest identities. Equal, unknown, shared or not-applicable identities cannot satisfy a mandatory separation. A profile cannot remove a required role to weaken the policy.
- Repeated domain identities across roles are reported as observed shared domains even if an input labels them `Distinct`; an explicit shared label without a matching cross-role identity is incomplete. The report does not infer independence from different executable hashes.
- A dependency-free incremental minimal-hitting-set reference enumerates every minimal set of domain compromises that hits every role dependency set under the explicit assumption that compromise of any listed dependency invalidates that role's output.
- Each dependency role must classify every dimension as exactly one of known, not-applicable (with a non-empty rationale), or unknown. Unknown, omitted, contradictory or empty dependency manifests return `Incomplete` with no cut-set cardinality. State-budget exhaustion returns `BudgetExceeded` with no partial cut sets. It never publishes a partial list as a complete result.
- Cut-set reports bind the claim ID, subject digest, dependency-manifest digest and attack-model digest.

Run from the repository root:

```sh
cargo fmt --manifest-path research/assurance-portfolio-cut/Cargo.toml --check
cargo check --locked --all-targets --manifest-path research/assurance-portfolio-cut/Cargo.toml
cargo test --locked --all-targets --manifest-path research/assurance-portfolio-cut/Cargo.toml
cargo clippy --locked --all-targets --manifest-path research/assurance-portfolio-cut/Cargo.toml -- -D warnings
```

The unit panel covers caller-pinned policy identity and policy-owned role/separation requirements, attempted profile downgrade, complete/incomplete/expired and malformed-window profiles, shared/unknown/omitted domain classifications, observed common dependencies, exact 2-role and 3-role minimal cut sets, an independent brute-force differential panel, role-order invariance, unknown dependency fail-closed behavior, state-budget exhaustion without partial output, duplicate roles and deterministic role-pair normalization. The pinned CI workflow checks the exact PR head with formatting, locked compile, tests and strict Clippy.

These are finite model tests. Policy and identity manifests are still supplied data: passing profile and cut-set tests do not authenticate ownership, prove organizational independence, or establish that all real common-cause failures have been enumerated.

## Profile-to-cut-set binding and algorithmic comparison

The downstream `analyze_profile_cut_sets` entry point is stricter than the lower-level finite enumerator. It accepts an opaque `VerifiedProfile` token created by `check_profile`, not a caller-constructible report struct. Its inner report is private; the report fields are exposed read-only through deref. This prevents a caller from fabricating a public report with `CompleteForDeclaredThreatModel` and skipping the profile checker. It accepts only a `CompleteForDeclaredThreatModel` profile and checks claim, subject, profile and policy identifiers; exact policy-required role set; and every role's 15 domain classifications and identities against the dependency manifest before it calls `minimal_cut_sets`. A manifest that merely repeats a matching profile digest but substitutes a dependency identity is rejected. The manifest and attack-model digests are carried into the report; the caller must still verify those digests against the exact source bytes.

The incremental enumerator sorts role dependency sets by increasing cardinality, then role ID, before expansion. This provides a deterministic processing order and makes candidate-extension budget consumption invariant to the input ordering of roles. It is a practical ordering heuristic, not a proven globally optimal ordering.

### What "better than SOTA" should mean here

This v0.1 crate is **not** a state-of-the-art general hypergraph transversal engine. It is a small, dependency-free, budgeted reference implementation for assurance portfolios with a bounded number of evaluator roles and 15 named dependency dimensions.

The surrounding algorithmic problem is nontrivial:
- Bläsius et al. (Journal of Computer and System Sciences, 2022) give polynomial-delay and polynomial-space minimal-hitting-set enumeration results for hypergraphs arising in data profiling, using an extension problem.
- A 2026 preprint by Mary gives new conditional lower bounds for minimal-transversal enumeration in bounded-dimension hypergraphs; this warns against promising a universally polynomial or output-insensitive solution.

References:
- Bläsius et al. (2022), *Efficiently enumerating hitting sets of hypergraphs arising in data profiling*: https://doi.org/10.1016/j.jcss.2021.10.002
- Mary (2026 preprint), *Lower Bounds for Minimal-Transversal Enumeration of Bounded-Dimension Hypergraphs*: https://arxiv.org/abs/2610.28012

A fair next benchmark compares the current incremental algorithm against:
1. the independent brute-force subset enumerator already used for tiny differential tests;
2. a separately implemented classic minimal-transversal enumerator (for example, a Berge-style incremental baseline);
3. a SAT/CP-SAT model that finds one hitting set at a time and blocks previously emitted minimal sets, with exact inclusion-minimality checks;
4. an output-sensitive extension-based minimal-transversal implementation where a reproducible implementation is available.

Freeze the same hypergraph/dependency manifest and attack semantics. Report the entire output set, not just the minimum cardinality. Measure:
- exact output-set agreement and false/missing/duplicate cut sets;
- number of minimal cut sets and candidate extensions;
- p50/p95 time-to-first cut set and time-to-complete enumeration;
- peak memory and frontier high-water mark;
- output delay and cost per emitted minimal cut set;
- budget-exhausted cases, explicitly retaining the incomplete disposition;
- sensitivity to role order, domain order, overlap density and number of roles/dependency keys.

Start with role counts 2–5 and all 15 dimensions, then scale synthetic portfolios to 10, 20 and 50 roles while retaining bounded, explicit candidate-extension and memory ceilings. Use held-out generated graph families and publish their generator/corpus digests. An improvement target should be **at least 2× lower median candidate extensions on held-out structured portfolio families** than the naive input-order baseline, with exact output-set equality and no larger false-positive count. This is a research target, not an achieved result; do not claim superiority over published output-sensitive algorithms without a matched reproduction.

The current dependency manifest model treats a dependency key as a single point whose compromise can invalidate a role. Future work must distinguish dependencies that can invalidate execution, alter expected answers, corrupt only input data, or merely reduce availability; those are different fault semantics and require separate attack-model dimensions, not a single stronger score.

The added v0.2 tests also verify that a complete profile can only be combined with its exact dependency manifest, that substituted domain identities fail even when the digest labels are left unchanged, that incomplete profiles cannot be promoted into cut-set analysis, and that candidate-extension accounting is invariant under role input order.

