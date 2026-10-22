# Life Disclosure Linkability Benchmark v1

Rust-first, dependency-free synthetic benchmark for measuring whether multiple life-continuity presentations can be paired across different recipients.

## Why this exists

Selective disclosure controls which claims are presented. It does not, by itself, guarantee that presentations are unlinkable. Identifiers, reusable proofs, lineage references, timestamps, and repeated behavior can still correlate sessions. The W3C Verifiable Credentials Data Model 2.0 explicitly discusses identifier-, signature-, metadata-, and validation-based correlation, and cautions that unlinkable signatures do not eliminate all other correlation signals:

- https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/

This harness makes a narrow part of that problem repeatable. It does **not** model production credentials, real cryptography, browser timing measurements, or actual people.

## Industry and standards baselines (checked 2026-10-22)

The next comparison layer should use existing interoperable mechanisms, not a custom Mycelix cryptosystem:

- **[W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/)** is a Recommendation (15 May 2025). It supplies the issuer/holder/verifier data model and explicitly discusses identifier-, signature-, metadata- and validation-based correlation.
- **[W3C VC Data Model 2.1](https://www.w3.org/TR/2026/WD-vc-data-model-2.1-20260930/)** is still a Working Draft (30 September 2026), not a Recommendation. Do not use it as a conformance claim.
- **[IETF RFC 9901, SD-JWT](https://www.rfc-editor.org/rfc/rfc9901.html)** is a Standards Track document published in November 2025 for selectively disclosing individual JSON/JWT claim elements. It is a concrete baseline for claim-level minimization, not a universal guarantee that multiple presentations cannot be correlated.
- **[OpenID for Verifiable Presentations 1.0](https://openid.net/specs/openid-4-verifiable-presentations-1_0.html)** is a Final Specification (July 2025) for the request/presentation protocol. A later integration benchmark should evaluate the protocol and credential format separately.
- **[W3C Data Integrity BBS Cryptosuites v1.0](https://www.w3.org/TR/vc-di-bbs/)** is a Candidate Recommendation Draft in the publication history as of 10 September 2026, not yet a W3C Recommendation. It specifically targets selective disclosure and unlinkable derived proofs; include it in a future comparison only with an independently reviewable implementation and supported issuer/profile.
- **[WebAuthn Level 3](https://www.w3.org/news/2026/web-authentication-an-api-for-accessing-public-key-credentials-level-3-is-now-a-w3c-recommendation/)** became a Recommendation on 25 August 2026. It is relevant to authentication and key lifecycle, but it is not a substitute for presentation unlinkability.

The strategic comparison is therefore: test SD-JWT and suitable VC/BBS profiles where actual adapters and issuer support exist; test each presentation protocol separately; and keep the cross-recipient attacker benchmark as an application-level composition test. Standards status is time-sensitive and must be rechecked before any conformance claim.

## Run

From the repository root:

```sh
cargo fmt --manifest-path research/life-disclosure-linkability/Cargo.toml --all -- --check
cargo test --locked --manifest-path research/life-disclosure-linkability/Cargo.toml
cargo clippy --locked --manifest-path research/life-disclosure-linkability/Cargo.toml --all-targets -- -D warnings
cargo run --release --locked --manifest-path research/life-disclosure-linkability/Cargo.toml -- --write-corpus-dir target/life-disclosure-linkability
sha256sum target/life-disclosure-linkability/*
```

The generated presentations contain synthetic features only. The `*-presentations.tsv` files are attacker-visible. The corresponding `*-ground-truth.tsv` files are labels for offline evaluation and must **never** be supplied to the attacker/model being measured. CI hashes both classes of file for reproducibility.

## Synthetic experiment

The frozen v1 fixture contains 120 synthetic subjects, 3 recipient domains, and 2 sessions per recipient. The evaluator examines cross-recipient pairs only: **172,800 pair trials per profile**. Pair labels derive from synthetic ground truth held separately from the attacker-visible feature table.

The profiles are intentionally staged:

1. `stable_global_id`: reusable subject identifier, proof, lineage metadata, and coarse behavioral feature.
2. `pairwise_id_only`: recipient-specific identifier, but reusable proof and other correlation surfaces remain.
3. `pairwise_id_fresh_proof`: identifier and proof are changed, but stable lineage/behavioral metadata remain.
4. `metadata_minimized`: pairwise identifier, fresh proof reference, and no extra envelope/behavior metadata, but the same claim set remains.
5. `predicate_only`: minimized auxiliary metadata plus one broad synthetic eligibility predicate. It demonstrates false matches from common claims; it is not asserted to be equivalent in utility to the quasi-identifying claim profile.

The three-claim profile uses age band, region band, and employment cluster. Their **combination** is unique for this small synthetic subject set by construction. This is a deliberate counterexample: cryptography cannot undo identification inherent in a disclosed claim combination.

## Metrics and interpretation

For each channel (claims, identifier, proof, envelope, behavior) and for the union of all channels, the executable reports:

- positive and negative cross-recipient pairs;
- true positives, false positives, false negatives and true negatives under a declared exact-match attacker rule;
- precision, recall, false-positive rate; and
- `binary_rule_auc = (TPR + TNR) / 2`, i.e. the AUC of that binary rule with tied scores. **This is not** an ROC-AUC measurement for a trained probabilistic attacker.
- A deterministic random-pair negative control that uses only presentation IDs and a fixed seed, never synthetic subject labels or feature values. Its metrics should remain near chance and are reported as `random_pair_baseline`.

The FNV-1a value printed by the Rust program is a non-cryptographic determinism fingerprint only. CI separately computes SHA-256 over every exported corpus file and binds its qualification record to the exact checked-out commit.

## Claim ceiling / limitations

- Passing unit tests shows this synthetic evaluator behaves as specified; it is not proof of unlinkability, anonymization, legal compliance, or production security.
- It uses exact-match rules rather than a trained, adaptive colluding-verifier attacker.
- It does not model high-resolution timing, browser fingerprints, network metadata, issuer collusion beyond shared observable values, or real cryptographic proof construction.
- The finite synthetic population and deterministic feature assignments are intentionally transparent and should not be mistaken for representative user data.
- Comparing profiles without a matched task-utility measure is incomplete. A stricter privacy profile may disclose less useful information. Report utility and task-equivalence separately.
- No global “privacy score” may hide a severe leakage channel. Review each channel and the full union separately.
- Do not use the corpus for real personal data; do not claim that pairwise identifiers alone provide unlinkability.

Issue: https://github.com/Luminous-Dynamics/mycelix/issues/5401
