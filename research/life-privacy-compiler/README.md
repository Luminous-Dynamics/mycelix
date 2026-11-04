# LIFE-PRIVACY-COMPILER-001 — Compile for privacy before disclosure

Related work: [issue #5487](https://github.com/Luminous-Dynamics/mycelix/issues/5487), [structural fingerprint oracle #5485](https://github.com/Luminous-Dynamics/mycelix/issues/5485), and [credential interoperability #5447](https://github.com/Luminous-Dynamics/mycelix/issues/5447).

## Question

Given a verification task whose answer must remain exactly correct, can a disclosure planner choose the lowest-overhead presentation profile that satisfies a declared candidate-support floor, rather than minimizing claims and checking leakage only afterward?

This dependency-free Rust program is an exact-search oracle over a frozen synthetic population. It does not create credentials, implement cryptography, or choose a production protocol.

## Paradigm

Treat task correctness as a hard constraint. Treat observer-visible metadata as a first-class part of the release. Enumerate viable presentation profiles, compute tail candidate support under the frozen observer model, and return both the minimum-cost feasible choice and the cost-versus-privacy Pareto frontier. No composite score can trade away task correctness or silently average away low-support targets.

The W3C BBS Cryptosuites v1.0 Candidate Recommendation Draft describes residual correlation from statement counts, revealed-index patterns and unique identifiers. It recommends common-length padding where relevant. Those points motivate the modeled channels but do not make this oracle a BBS test: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

## Frozen synthetic model

- 512 synthetic subjects, with eight balanced three-bit task-answer signatures.
- Every strategy emits exactly the same three task-answer bits for a given subject.
- Statement count ranges from 8 through 31.
- Each subject has three optional index positions chosen from a universe of eight.
- The count channel can be exact, mapped to two buckets, or padded to one common value.
- The index channel can expose the exact mask, expose a coarser class, or be omitted from the observer view.
- A stable-ID negative control is measured but policy-ineligible because this task does not require unique identification.
- Observer candidate sets are computed from task answers and only the fields exposed by the selected strategy.

The profile transformations are abstract capabilities. They are not asserted to map one-to-one to existing SD-JWT or BBS implementations.

## Compilation objective

The policy requires p05 candidate support >= 16 out of 512, exact task-answer equivalence, and no stable identifier when unique identification is not required. Overhead values are synthetic ordinal units used only to rank profiles. They are not bytes, milliseconds, gas, memory or measured production cost.

Among eligible profiles satisfying the candidate-support floor, the compiler minimizes the declared overhead, then maximizes p05 support, then breaks ties lexically. The report separately includes every strategy, under-k and unique-subject counts, candidate support percentiles, the selected profile and the pairwise nondominated frontier.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-privacy-compiler/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-privacy-compiler/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-privacy-compiler/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-privacy-compiler/Cargo.toml
~~~

The exact-head workflow checks out and asserts the PR head SHA, pins Rust 1.96.0, runs format/tests/strict Clippy, compares two report outputs byte-for-byte and saves a digest-bound evidence receipt.

## Frozen acceptance criteria (not achieved results)

- Every strategy preserves all three task answer bits for all 512 subjects.
- The exact-count/exact-index baseline fails the p05 candidate floor.
- The stable-ID control identifies every subject uniquely and cannot be selected.
- The selected strategy satisfies p05 >= 16 and is the lowest-overhead feasible eligible strategy.
- The frontier exactly agrees with a separate pairwise dominance oracle.
- Report includes raw low-support and unique counts with denominators and is byte-deterministic.

## Industry comparison and claim boundary

The OpenID Foundation completed OpenID4VP and OpenID4VCI conformance suites for HAIP 1.0 in August 2026 and opened them for self-certification. A real comparison should use those official protocol/profile tests rather than treating this search oracle as conformance: https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/

The next stage is to define a protocol-backed capability adapter, run official SD-JWT/OpenID4VP vectors and the version-pinned BBS test suite, then measure actual presentation bytes/latency and feed real allowed observer outputs into the candidate-set evaluator. Until that is done, no standards conformance, anonymity, DP, production cost or SOTA claim is made.
