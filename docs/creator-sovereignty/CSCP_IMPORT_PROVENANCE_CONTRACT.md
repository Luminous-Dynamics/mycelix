# CSCP Import Provenance Contract (research draft)

**Status:** experimental internal contract; not a conformance claim for W3C PROV, DDEX, or ODRL.
**Related implementation:** public Mycelix draft PR #5182, isolated Rust crate module provenance.rs.
**Tracker:** [CSCP-PROV-001](https://github.com/Luminous-Dynamics/mycelix/issues/5187).
**Research checked:** 2026-10-18.

## Executive decision

Move from "this row came from platform X" to **field-level, replayable, transaction-bound provenance**. A creator should be able to inspect, for every imported fact:

- which exact source payload was observed;
- the source system and optional source record/message identity;
- the schema edition and parser/mapping versions that interpreted it;
- which enumerated source fields were mapped to which target paths;
- which fields were preserved only in raw source and which remain unsupported;
- what warnings or errors occurred;
- the digest of the passport before and after import;
- whether a rejected import left canonical state unchanged.

The import manifest is a sidecar with a canonical byte encoding and content digest. It is not an identity signature, a legal-rights determination, or proof that the source's assertions are true. The raw source bytes must remain in creator-controlled or appropriately access-controlled storage; the manifest keeps a reference, length and digest so retrieved bytes can be checked later.

## Standards posture

W3C PROV is the semantic reference for describing entities, activities, and agents involved in producing or transforming data. CSCP's source payload, import activity, parser/mapping version, and resulting snapshot can be mapped to that model in a future adapter, but the current compact Rust manifest does not claim PROV conformance.

- W3C, [PROV Overview](https://www.w3.org/TR/prov-overview/)
- W3C, [PROV Model Primer](https://www.w3.org/TR/prov-primer/)

W3C ODRL represents permissions, prohibitions, obligations, actions, parties, assets and constraints. It is a plausible adapter vocabulary for expressing some use-policy exchange cases, but the internal usage-grant evaluator remains the current narrow authorization model; an ODRL adapter must preserve prohibitions, duties/obligations and constraints rather than flattening a policy to a boolean allow flag.

- W3C, [ODRL Information Model 2.2](https://www.w3.org/TR/odrl-model/)

For music metadata/revenue exchange, CSCP should continue to use versioned, appropriately licensed DDEX adapters rather than inventing competing payload schemas. See the [DDEX Interoperability Contract](CSCP_DDEX_INTEROP_CONTRACT.md) and tracker [CSCP-INTEROP-001](https://github.com/Luminous-Dynamics/mycelix/issues/5186).

## Implemented sidecar model

The isolated Rust prototype exposes SourceDocument, FieldMapping, ImportDiagnostic, and ImportProvenanceManifest in the provenance module.

### SourceDocument

It records source system, optional source record ID, schema name/version, parser name/version, import timestamp, media type, raw-payload reference, exact raw byte length, and BLAKE3-256 computed from the payload bytes passed to capture(). It does not accept a caller-supplied digest, so the captured hash is computed by the library.

verify_payload() compares retrieved bytes' length and digest. It does not resolve the raw-payload reference, confirm durable retention, authenticate the source, or prove the bytes' meaning.

### Field disposition

Every parser-enumerated source field must carry one explicit treatment:

- Mapped: target path, mapping rule ID, and mapping rule version.
- RetainedRaw: not represented by the canonical model, but retained in the original payload; requires a field-level diagnostic.
- UnsupportedRetained: unsupported by this model, retained in raw source, and explicitly diagnosed.

There is no silent-discard treatment. Source inventory is labelled complete-as-reported-by-parser, partial, or unknown. The "complete" state reports the parser's assertion only; it cannot prove that parser code discovered every source field. Retaining the source payload makes later independent audits possible.

A future hardening increment should make the target path convention unambiguously identify both the target object and target field, and should add signed/attested parser identity only where a trusted signing/attestation model exists.

### Import outcome and snapshot digests

The manifest binds processing to the before/after passport digests:

- Imported requires complete-as-reported inventory, no diagnostics, and every declared field mapped.
- ImportedWithWarnings requires an after digest and visibly records non-mapped fields, partial/unknown inventory, or diagnostics.
- Rejected requires an error diagnostic and identical before/after snapshot digest. This encodes a required failure-atomicity invariant.

These are caller-recorded outcome assertions. The manifest itself does not commit a database transaction or independently observe the passport; the integration layer must enforce that the recorded snapshot digests accurately reflect persisted state.

## Required workflow

1. **Capture before mutation.** Store or retain the original payload in access-controlled storage. Compute byte length and BLAKE3 digest from those exact bytes. Record source/schema/parser versions.
2. **Validate and parse in quarantine.** Validate against the exact source schema/profile; do not mutate canonical state while validation is incomplete or failed.
3. **Enumerate source fields.** Create one field record per parser-enumerated field. Preserve original bytes and explicit paths. Unknown fields remain visible and diagnosed.
4. **Apply a versioned mapping.** Identify the exact rule and version for each mapped field. Retain missing/ambiguous data as unknown rather than inventing a default or identifier.
5. **Validate proposed canonical state.** Check cross-record references, identifier type, exact split arithmetic, consent scope, and currency/territory boundaries. Conflicts create review states rather than newest-wins overwrites.
6. **Commit atomically.** On rejection, after-digest must equal before-digest. On commit, calculate the post-import snapshot digest from the committed snapshot.
7. **Write an immutable sidecar.** Record diagnostics, before/after snapshot digests, source payload digest, and exact mapping/parser versions. Corrections should append a new manifest rather than rewriting the previous import history.
8. **Verify portability.** A receiver can verify the raw source bytes against the payload digest, decode the manifest, and re-encode to identical canonical bytes. A digest by itself does not authenticate the source or importer; a trusted signature/timestamp service is a separate design decision.

## Benchmark and release gates

Targets below are proposed gates, not measured results.

| Gate | Target | Why |
|---|---|---|
| Raw-payload re-verification | 100% authentic source-byte fixtures accepted; 100% one-byte mutation fixtures rejected | Prevent provenance/reference drift |
| Field inventory accounting | Each declared source path has exactly one treatment | No silent field omission |
| Unsupported-field disclosure | Every non-mapped field has a diagnostic and is still present in retained raw source | No silent lossy mapping |
| Import rejection atomicity | 100% rejected imports record identical before/after snapshot digest, independently confirmed by persisted store | No partial canonical mutation |
| Deterministic mapping | Same payload/schema/parser/mapping versions yield same canonical target state | Reproducible imports |
| Re-processing behavior | Re-import of same source/version is idempotent or produces an explicit duplicate/revision event; never duplicate money/rights silently | Safe retries |
| Parser upgrade | Parser/mapping version change produces a reviewable diff before replacing current values | No invisible schema drift |
| Manifest portability | Independent encoder/decoder produces byte-identical manifests for a diverse frozen corpus | Cross-language reproducibility |
| Digest correctness | Independent implementation matches domain-separated manifest digest on frozen fixtures | Integrity without authenticity overclaim |
| Source authenticity | No provenance manifest is labelled signed/authenticated without a verifiable signature and identity policy | Prevent false trust |
| Sensitive metadata access | 0 raw-payload, statement, or unpublished asset leaks across permission scopes | Creator privacy |
| DDEX schema validity | All export fixtures validate against the exact licensed/versioned source or recipient profile | No guessed interoperability |

## Known limitations / next implementation steps

- This sidecar is not currently embedded inside CreativeAssetPassportEnvelope; it is linked by before/after snapshot digests and should be stored next to the snapshot.
- A digest and caller-provided raw_payload_ref cannot prove that the referenced bytes remain available. Add retention/availability checks in the storage adapter.
- The source field inventory is only as complete as the parser says it is; independently enumerate unknown XML/JSON fields in adapter conformance tests.
- The current schema has a small Rust/Python byte vector but no broad cross-language corpus, BLAKE3 digest vector, fuzz campaign, or externally reviewed conformance result.
- Imported and Rejected are manifest assertions. The persistence/ingress layer must enforce transaction boundaries and prove the after digest it records.
- The sidecar has no digital signature, trusted timestamp, cryptographic sender identity, or legal-validity check. Add signing only after key management, identity binding, revocation, retention and threat model are specified.
- DDEX/ODRL/PROV adapters must use applicable normative schemas/specifications and honest conformance claims. Internal field-name similarity is not compliance.

