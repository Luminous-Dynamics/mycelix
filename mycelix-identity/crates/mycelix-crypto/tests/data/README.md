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
