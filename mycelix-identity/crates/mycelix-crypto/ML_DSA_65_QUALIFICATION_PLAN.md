# ML-DSA-65 Verifier Qualification Plan

Status: **candidate implementation; not qualified**.

This plan applies to `mldsa65_verify::verify_with_empty_context` under the narrow `mldsa-verify-rc` Cargo feature and to any consumer adapter, including Symthaea PR #7238. That feature enables only the RustCrypto `ml-dsa` primitive, rather than pulling in the unrelated Ed25519, KEM, and AEAD dependencies of `hybrid-rc`. The plan is deliberately independent from key authorization and protocol transcript qualification.

## 1. Immutable implementation inputs

| Input | Pin | Use |
|---|---|---|
| Provider candidate | `ml-dsa = 0.1.1`, as present in `mycelix-identity/Cargo.lock` | Candidate implementation under test |
| Independent verifier candidate | `libcrux-ml-dsa = 0.0.11`, upstream commit `42d68bd49b244b09bd02626c6ffa95e54dc64f91` | Test-only differential oracle; never a production dependency for this PR |
| Wycheproof source snapshot | `C2SP/wycheproof` commit `12fd3aaf33eb5fa1f52e026912ee00c054f9d984` | Independent positive/negative verification corpus |
| Wycheproof ML-DSA-65 file | `testvectors_v1/mldsa_65_verify_test.json`, Git blob SHA-1 `049f74be9785af926623e56530ab3aa9384179e8` | Source identity only; **not** a SHA-256 digest |

The Wycheproof file's SHA-256 is not yet recorded in a checked-in corpus lock. This is a deliberate open gate: compute SHA-256 from the actual bytes checked into the qualification fixture and store it before calling any corpus run reproducible. Do not substitute the Git blob SHA-1 for SHA-256. The source snapshot commit is immutable, but the digest of our exact fixture is still required.

Libcrux's release source exposes the explicit-context verification API. Its version 0.0.11 is newer than the version 0.0.9 fix for the known AVX2 `use_hint` edge case, but this version floor is not a qualification verdict. Keep its use test-only and pin the exact source revision and Cargo resolution.

## 2. Required verdicts

Run the complete pinned ML-DSA-65 Wycheproof verification corpus (210 cases at the referenced snapshot) against both implementations using the **same exact key, message, context, and signature bytes**.

- `valid`: must verify successfully.
- `invalid`: must be rejected.
- `acceptable`: report separately. Do not silently count it as valid or invalid; the harness must implement and record an explicit, reviewed policy for each such flag.
- RustCrypto and Libcrux disagreement on a `valid` or `invalid` case is a hard failure and must preserve the case input and both outcomes in the evidence packet.

Two essential sentinel vectors identified by the public community analysis are ML-DSA-65 Wycheproof tcId 19 (repeated hint index; reject) and tcId 61 (valid signature with the largest z coefficient below the limit; accept). The latter prevents a suite containing only negative tests from passing a verifier that rejects valid boundary signatures too aggressively.

Source: [ACVP-Server issue #470](https://github.com/usnistgov/ACVP-Server/issues/470). That is community research about the published vector coverage—not an official NIST validation result.

## 3. Dedicated adversarial cases

The pinned general corpus is necessary but not sufficient. Add focused cases with frozen input bytes and expected verdicts for:

1. Repeated indices in a hint-index list: reject.
2. Nonzero unused hint-index tail padding: reject.
3. Total hint weight exceeding `omega`: reject.
4. z infinity norm at or above the forbidden `gamma_1 - beta` bound: reject.
5. A valid near-boundary signature whose z coefficients remain below the bound: accept.
6. Correctly sized but malformed signature encodings; wrong key/signature lengths; modified message; modified signature: reject.

For any generated or transformed fixture, preserve its source vector ID (if derived from one), the exact transformation, expected verdict, raw input file SHA-256, and a deterministic case identifier. Never mutate an input during the test without recording the mutation.

## 4. Mutation sensitivity

Run verifier mutations against the same acceptance suite. The following mutants must be detected, and the test receipt must identify the killing case for each:

- Strict hint-index ordering relaxed from `<` to `<=`.
- Hint-weight bound removed or weakened.
- Infinity-norm check removed, or its bound tightened by one.
- Zero requirement for unused hint-index tail removed.

A mutation counts as killed only when the mutated build completes and a named test changes to its expected failing result. A build error, timeout, skipped vector, or unexercised mutant is **not** a kill.

## 5. Evidence packet and promotion gate

The qualification runner must emit machine-readable evidence containing:

- Exact repository commit and clean checkout identity.
- `Cargo.lock` SHA-256, resolved RustCrypto and Libcrux versions, Libcrux commit, rustc/cargo versions, target triple, features, and relevant environment.
- Upstream source commit, local fixture path, fixture SHA-256, corpus/test counts by expected verdict, and case-level results.
- Independent-oracle verdicts and any disagreement records.
- Mutation inventory, mutation IDs, build outcomes, killed/survived counts, and case-level killers.
- Final gate result derived from the required assertions rather than a manually supplied green flag.

Qualification requires the locked test command to pass on the exact head, every required corpus case to be exercised, all mandatory mutations to be killed, and no unexplained provider/oracle disagreement. Until these conditions are met, the verifier remains experimental and must not be represented as a production-qualified provider or cryptographic certification.

## 6. Execution commands

The normal provider seam test stays offline/lock-preserving:

```sh
cd mycelix-identity
cargo test --locked -p mycelix-crypto --features mldsa-verify-rc
```

The independent corpus target should be added only with a committed lockfile and committed fixture digests; do not generate or silently modify a lockfile during the qualification job. Keep the differential verifier as a dev-only dependency and use a separate, explicit qualification target so the production dependency graph does not acquire Libcrux merely to gain assurance evidence. The standalone provider tests should run with `mldsa-verify-rc`; test the high-level `hybrid-rc` composition separately.
