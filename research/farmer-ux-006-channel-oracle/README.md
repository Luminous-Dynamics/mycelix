# FARMER-UX-006 channel continuity oracle

A standalone Rust reference verifier for the synthetic cross-channel contract at the repository root. It is deliberately not a production adapter, service evaluator, durable journal, Holochain zome or identity/consent provider.

## Run

From the Mycelix repository root:

```sh
cargo test --manifest-path research/farmer-ux-006-channel-oracle/Cargo.toml
cargo run --manifest-path research/farmer-ux-006-channel-oracle/Cargo.toml -- --root . --report-json research/farmer-ux-006-channel-oracle/target/channel-oracle-report.json
```

The focused workflow runs both commands and uploads the JSON report.

## What is executable here

- Full JSON Schema Draft 2020-12 schema meta-validation and instance validation of the 17-case FUXC corpus and the synthetic relay-envelope example.
- Semantic guards for every frozen case: continuity, event replay vs equivocation, actor/recorder attribution, consent revocation, source/time provenance, unknown outcomes, correction history, no-provider outcomes, farmer acceptance, disputes and denominator preservation.
- A deliberately small reference ledger: an exact same-event envelope replay is idempotent; any same-event-ID envelope mismatch—including a changed payload digest, request reference or asserting/recording role—is retained as a conflict and cannot overwrite the first-seen event; separate request references are not fuzzy-merged.
- Fail-closed promotion rules for the high-risk transitions covered in the contract.
- 10,000 deterministic schedules per verifier run. Each schedule reorders eight events and injects one exact replay plus one same-ID/different-digest conflict. The seed and shuffle algorithm are fixed; no randomness dependency is used.
- Mutation tests that remove required conflict, relay, consent and unknown-outcome protections and require the semantic verifier to reject the mutated corpus.
- A JSON report with SHA-256 digests of source fixtures and schemas, schema-engine results, semantic errors, schedule count, resolved standalone Cargo.lock digest (if Cargo generated/read one) and an explicit claim ceiling. The workflow logs whether a lock file existed before dependency resolution; a generated lock digest is evidence of that CI resolution, not a reviewed committed lock file.

## Research anchors (design hypotheses, not local impact claims)

- Gatti et al., *Not-so-digital platforms?* (Journal of Rural Studies, June 2026), reports from multi-year qualitative fieldwork in Ghana and Kenya that registrations/subscriptions can mask uneven or short-lived use and that peer networks, extension staff and low-tech channels can outperform poorly fitted platforms. This is a warning to test institutional fit, not a universal claim that digital services fail: https://doi.org/10.1016/j.jrurstud.2026.104210
- *Digital literacy training to promote diffusion of digital agricultural tools to smallholder farmers* (Food Policy, June 2026) reports a randomized intervention in Egypt where training affected uptake, use, trust and perceived usefulness, with heterogeneous effects by demographic characteristics. Therefore a field study should record training/assistance exposure separately from service quality rather than attribute every change to interface design: https://doi.org/10.1016/j.foodpol.2026.103108
- FAO's Digital Services Portfolio is designed for country/local tailoring and targets smallholders, family farmers and other rural actors. This supports localized task and language testing rather than a presumed universal workflow: https://www.fao.org/digital-services/about/

**Measurement implication:** registrations, subscriptions and page views are not farmer-service outcomes. The outcome layer should report eligible requests, provider availability, authoritative receipts, cancellations/no-shows, accepted service inside a farmer-declared useful window, quality disputes/rework and farmer-paid cost separately. FUXC's synthetic tests check the integrity of evidence and denominators; they cannot tell us whether any channel improves these outcomes in South Africa.

## Interpret results honestly

A passing model proves only the code and fixtures in this crate. It does not prove browser/IndexedDB durability, server-side idempotency, real actor identity or consent, secure field relaying, provider receipt, bookings, service fulfilment, farmer comprehension, accessibility conformance, or agricultural benefit. The schedule generator is a model-based stress test—not a statistically representative field study or exhaustive proof of all event histories.

The contract and this verifier remain a stacked draft. Transitive dependency lockfile reproducibility is not qualified until a reviewed `Cargo.lock` is committed. The model must be checked against the exact reviewed shared journal/service-evaluator contracts before production integration.


The oracle workflow checks out the pull-request head SHA explicitly, asserts the checked-out `HEAD` matches, and pins its GitHub Actions to commit SHAs. The Rust toolchain itself is still requested as the `stable` channel; the exact compiler version is emitted in the run logs. Use a reviewed committed lockfile and `--locked` as the next reproducibility gate before promoting this research crate beyond draft.
