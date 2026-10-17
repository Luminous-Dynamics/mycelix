# Cross-Scale Wisdom Corpus Verifier

Standalone Rust CLI for validating the **structure and content identity** of frozen cross-scale wisdom JSONL corpora. It is deliberately not an evaluation engine for model responses.

## Run

From the repository root, with a Rust toolchain and dependencies available:

    cargo run --manifest-path tools/cross-scale-wisdom-verifier/Cargo.toml -- verify \
      docs/research/fixtures/cross-scale-wisdom-v1.jsonl \
      docs/research/fixtures/cross-scale-wisdom-v1.manifest.json \
      docs/research/fixtures/cross-scale-wisdom-case.schema.json

To verify the optional Integral-lens supplement:

    cargo run --manifest-path tools/cross-scale-wisdom-verifier/Cargo.toml -- verify \
      docs/research/fixtures/cross-scale-wisdom-integral-lens-v0.1.jsonl \
      docs/research/fixtures/cross-scale-wisdom-integral-lens-v0.1.manifest.json \
      docs/research/fixtures/cross-scale-wisdom-case.schema.json

To verify the spiritual-practice boundaries supplement (still structurally validated and unscored):

    cargo run --manifest-path tools/cross-scale-wisdom-verifier/Cargo.toml -- verify \
      docs/research/fixtures/cross-scale-wisdom-spiritual-practice-boundaries-v0.1.jsonl \
      docs/research/fixtures/cross-scale-wisdom-spiritual-practice-boundaries-v0.1.manifest.json \
      docs/research/fixtures/cross-scale-wisdom-case.schema.json

Run the unit tests with:

    cargo test --manifest-path tools/cross-scale-wisdom-verifier/Cargo.toml

## Failure behavior

A malformed manifest/schema/record, hash mismatch, blank JSONL line, missing final newline, duplicate case/evidence ID, undeclared field, unknown severity/evidence kind/invariant, mismatched record or severity count, or incorrect manifest invariant set returns an error and a non-zero process exit. The CLI never rewrites a manifest automatically.

Output contains deterministic corpus/schema/checker identities and per-case receipts whose disposition is UNSCORED. Environment identity is explicitly reported as not captured by this structural verifier.

## Claim ceiling

A successful report is STRUCTURALLY_VALID_UNSCORED. It certifies only that the exact corpus bytes match their manifest and the records satisfy this verifier's structural contract. It does not score responses, independently validate case rationales, establish the scientific validity of Integral Theory or any other lens, measure a person's wisdom or consciousness, establish human efficacy, or qualify field safety.

The case schema and the verifier's accepted semantics must be changed together through reviewed, versioned updates. Frozen corpus content must not be silently edited; publish a new corpus version or a named supplement.
