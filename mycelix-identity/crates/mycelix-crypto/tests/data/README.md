# Pinned ML-DSA-65 Verification Corpus

The adjacent `mldsa_65_verify_test.json` is vendored unmodified from C2SP
Wycheproof at commit
`12fd3aaf33eb5fa1f52e026912ee00c054f9d984`, path
`testvectors_v1/mldsa_65_verify_test.json`.

- Upstream Git blob SHA-1: `049f74be9785af926623e56530ab3aa9384179e8`
- Fixture SHA-256: `49ac366d76115eab56b7116f10d06e288e6f23fe6cfb90b26bfb2d731a8d1e02`
- UTF-8 size: 1,664,194 bytes
- Cases: 210 total; 79 expected valid and 131 expected invalid
- Required sentinels: tcId 19 rejects repeated hint indices; tcId 61 accepts the valid near-boundary signature

The source blob SHA-1 was recomputed over Git's `blob <size>\0<bytes>`
representation and matched the source object ID. The SHA-256 is computed over
the exact UTF-8 file bytes. The integration test recomputes that SHA-256 at
runtime before applying the corpus.

Do not modify the vendored corpus in place. A corpus update must change the
source commit, source blob ID, SHA-256, case counts, and reviewed expected
verdict policy together. The test intentionally fails if a case class such as
`acceptable` appears without an explicit policy.

The upstream C2SP Wycheproof project is distributed under Apache License 2.0;
see [LICENSE-C2SP-Apache-2.0.txt](LICENSE-C2SP-Apache-2.0.txt).

## Supplemental upstream candidate (not yet merged upstream)

A second fixture, `mldsa_65_verify_pr278_test.json`, is sourced from C2SP
Wycheproof PR [#278, “mldsa: regenerate FIPS 204 edge cases”](https://github.com/C2SP/wycheproof/pull/278),
head commit `8f654b7fe9cd9bf0825269df2a7541c52f0f9cb1`. This commit is
signature-verified by GitHub and the PR reports cross-checks with OpenSSL,
smoke tests with Go's standard-library implementation, and mutation checks.
The PR was still open at the time this fixture was pinned, so this file is
**supplemental proposed upstream data**, not a released/main-branch corpus.

- Candidate file Git blob SHA-1: `fa871a3c8c76b0cf871879243ad34fa2cbd79000`
- Candidate fixture SHA-256: `1ca235f61928421a173a171780beb00264ae9155dc33c52a11e1766cc7979034`
- UTF-8 size: 1,664,284 bytes
- Cases: 210 (79 expected valid and 131 expected invalid)

The C2SP issue [#276](https://github.com/C2SP/wycheproof/issues/276) reported
that some ML-DSA edge cases were stale under the final FIPS 204 key-expansion
derivation. The harness runs this candidate separately from the published
snapshot to preserve provenance and prevent us from treating an unmerged
correction as official upstream data. Do not replace either fixture without
reviewing its source commit, blob SHA-1, SHA-256, case counts, and expected
verdicts together.

## Empty-context adapter policy

The C2SP files exercise the general ML-DSA verification API and include a
`ctx` field in eight cases. This module intentionally exposes only the
RFC 9964 empty-context profile. The harness therefore honors source verdicts
and the vector context together: source-labeled valid vectors with an absent
or empty `ctx` must verify; source-labeled valid vectors with a non-empty
`ctx` must be rejected by this empty-context adapter; source-labeled invalid
vectors must all be rejected. At these pins, that means 77 valid empty-context
acceptances, two non-empty-context valid signatures rejected for this profile,
and 131 invalid rejections per corpus. This is deliberately different from
claiming that all 79 source-labeled valid vectors should pass an empty-context
API.
