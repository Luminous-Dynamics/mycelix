# LIFE-DISCLOSURE-TRACE-001

## Question

Can an observer infer a synthetic protected label from the **combined trace of repeated interactions** even when the required task result is identical? This experiment isolates observable channels from credential cryptography and from the admission guard's internal decision procedure.

The work follows the adaptive guard (#5456), population-shift stress test (#5458), and uncertainty-set plan (#5459). It complements rather than replaces the standards/profile comparison in #5447.

## Paradigm

A per-query counterfactual check asks whether the admission decision changes when the target's *next* answer bit changes. Transcript noninterference asks a broader question: can an observer distinguish two otherwise-matched worlds from the history of public behavior over repeated sessions?

The frozen synthetic fixture includes 256 paired contexts. Each context has two counterfactual worlds that differ only in a protected Boolean label. Both worlds produce the same required task result (`ELIGIBLE`) over four sessions. Observable trace fields are controlled independently to expose the effect of admission sequences, reason details, response size, protocol round trips, status/schema lookups, and retry behavior.

Profiles include a no-secret-correlation control, one profile per channel, and a deliberately leaky all-channel positive control. These are abstract observer features, not claims that a particular production API behaves this way. The response-size and round-trip values are synthetic counters, not measured latency.

## Run

```sh
cargo fmt --manifest-path research/life-disclosure-observable-traces/Cargo.toml --all -- --check
cargo test --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml
cargo clippy --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml --all-targets -- -D warnings
cargo run --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml
```

The executable emits deterministic CSV. CI runs it twice, compares the outputs byte-for-byte, and records SHA-256 digests for the report and machine-readable channel manifest bound to the checked-out commit.

## Metrics

Each row identifies a synthetic profile and observer channel. It records the number of counterfactual pairs whose trace differs, the raw denominator and ppm mismatch rate, held-out balanced accuracy from a categorical baseline attacker, and exact task-result agreement. The joint view concatenates every observer-visible field. The attacker split is by paired-context ID (not by individual row), so the two worlds of one pair cannot straddle train and test.

Interpretation:
- A mismatch is evidence that the **declared observer view** differs in the paired fixture.
- An attacker's balanced accuracy above 500,000 ppm shows distinguishability under that fixed baseline.
- A baseline attack at chance is not proof that the traces are private; more powerful attacks and unmodeled channels may exist.
- A zero mismatch result is only about the specified channel manifest, profiles, and finite fixture.

## What is intentionally not modeled yet

- No SD-JWT, BBS, OpenID4VP, Holochain or production policy implementation.
- No real network capture, revocation endpoint, DNS/TLS metadata, or runtime timing distribution.
- No real personal data, population prevalence claim, formal information-flow proof, differential privacy guarantee, or anonymity result.
- No assumption that response normalization alone defeats traffic analysis; actual network and timing channels need empirical implementation-level tests.
- The simple categorical attacker is a falsifiable baseline, not the best possible attacker. A failed attack does not establish absence of leakage.

The machine-readable manifest explicitly marks wall-clock timing as unmeasured. Timing results must only be added from actual pinned implementations on matched hardware, using repeated trials and environmental metadata. Keep protocol conformance, required task utility, cryptographic linkability, observable policy leakage and cost on separate axes.

## Research anchors

- W3C Verifiable Credentials Data Model 2.0 Recommendation, which cautions that unlinkability does not remove correlation from disclosed content, identifiers, metadata and validation behavior: https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- W3C Data Integrity Threat Model v1.1 is a **Group Note Draft** dated 8 October 2026; its maturity must not be promoted to Recommendation: https://www.w3.org/TR/2026/DNOTE-vc-data-integrity-threat-model-1.1-20261008/
- OpenID for Verifiable Presentations 1.0 Final specifies client-id and nonce binding to prevent replay; replay resistance and longitudinal privacy are distinct properties: https://openid.net/specs/openid-4-verifiable-presentations-1_0.html
- Rogers, Roth, Ullman and Vadhan, *Privacy Odometers and Filters: Pay-as-you-Go Composition*: https://proceedings.neurips.cc/paper/6170-privacy-odometers-and-filters-pay-as-you-go-composition.pdf

## Qualification boundary

This is a dependency-free, deterministic synthetic evaluator. It measures an explicitly declared observer view and is useful for finding counterexamples and regression-testing trace normalization. It is not evidence of deployment risk, conformance, universal noninterference, privacy in out-of-model channels, or state-of-the-art superiority.
