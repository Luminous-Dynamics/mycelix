# LIFE-JOINT-PRIVACY-001 — Compile against linkage and attribute inference together

Related work: [issue #5494](https://github.com/Luminous-Dynamics/mycelix/issues/5494), [transcript compiler #5490](https://github.com/Luminous-Dynamics/mycelix/pull/5490), and [robust transcript compiler #5493](https://github.com/Luminous-Dynamics/mycelix/pull/5493).

## Why another benchmark?

Candidate-set size and sensitive-attribute privacy are different dimensions. A target may retain 20 or 32 plausible matches while the observable metadata determines the target's protected label with high confidence. A compiler that only enforces a k-style candidate floor can therefore accept a transcript with severe attribute inference.

This Rust 2024 executable oracle adds a second independent hard constraint. It is a finite synthetic study, not a formal Bayesian guarantee or a credential implementation.

## Frozen model

- 512 synthetic subjects and four task answers per subject; eight balanced task transcripts of 64 subjects each.
- Each task transcript has 32 positive and 32 negative values for the hidden synthetic protected attribute.
- Two scenarios share identical task-answer transcripts.
- Reference scenario: count and index metadata are generated without intentional dependence on the protected attribute.
- Shifted scenario: count buckets split every task transcript into 20 low-count subjects (all positive) and 44 high-count subjects (12 positive and 32 negative); the coarse index-parity channel encodes the protected label. Exact index masks vary within parity classes.
- Every round chooses exact / two-bucket / padded count metadata and exact / coarse / omitted index metadata. The full plan space is 9^4 = 6,561.
- Typed ObserverView has required task answers and only count/index fields that the plan exposes. It never contains the protected attribute.

## Joint policy

A plan is feasible only when every declared scenario has both:

1. p05 transcript-level candidate support >=16 out of 512; and
2. maximum absolute deviation of any observer-group protected-label prevalence from 50% <=100,000 ppm (10 percentage points).

The group-lookup attacker is intentionally simple and fully inspectable. For every observer-view equivalence class, the evaluator computes positive/negative label counts, the largest absolute group posterior deviation from the fixed 50% baseline, and group-majority prediction accuracy. These are empirical fixture metrics; they are not estimates of real-world posterior probabilities or a universal attack bound.

The compiler minimizes abstract cost, then maximizes worst-scenario p05 support, then minimizes worst-scenario uplift, then uses the lexical plan label. The Pareto frontier separately considers cost, support and uplift. No blended score allows one privacy dimension to compensate for a failure in the other.

## The key counterexample

In the shifted scenario, a profile exposing only two-bucket counts and omitting index metadata produces candidate groups of 20 and 44 subjects. The p05 support is 20, so the k=16 floor passes. However, the 20-person group is 100% positive, so maximum attribute posterior uplift is 500,000 ppm (50 percentage points) and majority attacker accuracy is 812,500 ppm (81.25%). The joint compiler must reject that plan.

A coarse index class partitions into 32-person groups and can also pass k=16 while exposing the protected attribute exactly. The joint policy rejects it too.

Padding count metadata and omitting index metadata leaves only the 64-person task cohort observable, with 32 positive and 32 negative subjects. It satisfies both gates in the declared fixtures at 144 abstract overhead units. These are deterministic synthetic results, not measured credential costs or anonymity guarantees.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-joint-privacy/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-joint-privacy/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-joint-privacy/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-joint-privacy/Cargo.toml > report.csv
~~~

The dedicated exact-head workflow pins Rust and action revisions, asserts the checked-out commit, runs format/tests/strict Clippy, compares report bytes across repeated runs, and retains SHA-256 evidence.

## Industry comparison and limits

NIST SP 800-226 stresses evaluating the privacy guarantee actually offered and the hazards introduced in implementation: https://csrc.nist.gov/pubs/sp/800/226/final

The W3C BBS Cryptosuites v1.0 remains a Candidate Recommendation Draft and discusses structural linkage beyond disclosed claim values: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

OpenID4VP/OpenID4VCI with HAIP 1.0 has an official conformance programme; conformance is a separate protocol-correctness gate, not an attribute-inference guarantee: https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/

The next stage must use real outputs from version-pinned implementations and stronger calibrated attackers. This oracle does not establish differential privacy, anonymity, real-user risk, standards conformance, production security, or SOTA superiority.
