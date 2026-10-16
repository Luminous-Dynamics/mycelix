# Mycelix prediction-market resolution evidence v1

**Status:** schema/design contract only. This is not a market deployment, settlement adapter, identity provider, or legal eligibility determination.

## Why this boundary exists

The Symthaea Futures pipeline already has neutral forecast representations, proper scoring, calibration diagnostics, and replayable evidence. Mycelix should not reinvent probability scoring. It should establish a stable, auditable contract/evidence boundary that allows multiple forecast engines and venues to reason about the same event without confusing price, evidence, and authority.

This adds two schemas and four fixtures:

- `schemas/prediction_market_contract_v1.schema.json`: frozen event semantics and predeclared resolution rules.
- `schemas/prediction_market_resolution_event_v1.schema.json`: one signed, append-only claim in the resolution lifecycle.
- `fixtures/prediction_market_contract_v1.valid.json` and `fixtures/prediction_market_resolution_event_v1.proposed.valid.json`: synthetic structural examples using `example.invalid`.
- corresponding `*.invalid.json` negative controls for missing fallback policy and an illegal predecessor on sequence one.

## Paradigm shift: resolve the event, not just a ticker

A market must be anchored to an immutable event contract:

```
stable contract ID + revision
    + exact event wording + outcome semantics
    + event cutoff + resolution cutoff
    + primary source + fallback/void policy
    + revision policy + dispute + adjudication + appeal policy
    + canonical digest
```

Any semantic change is a new revision with a successor link. Once a market has opened, the contract should be immutable; if a correction is needed, record a new revision/event under a declared transition policy instead of overwriting the original bytes.

## Three separate evidence classes

```
ContractCommitment
    != SourceArtifactObservation
    != ResolutionDecision
    != AuthorizedFinalization
```

- **Contract commitment** freezes the event definition and policies.
- **Source artifact observation** identifies the exact fetched bytes, source profile, publication/retrieval times, revision reference, and upstream provenance family.
- **Resolution event** states that a named actor proposed, challenged, decided, or voided an outcome under a specified policy.
- **Finalization/settlement** is a separate downstream action requiring a policy-qualified authority receipt and its own controls. The schema does not grant it.

A content digest verifies byte identity only when the bytes are separately obtained and re-hashed. A URL does not prove authority; an issuer/principal string is not authenticated identity; a signature-shaped JSON object is not a verified signature; a source described as "official" is not guaranteed true. The schema checks shape, not cryptography, source truth, or authorization.

## Required validators beyond JSON Schema

An independent conformance checker must fail closed unless all applicable requirements hold:

1. Compute RFC 8785 JSON Canonicalization Scheme bytes for the contract with `contract_digest_sha256` omitted, then recompute SHA-256 and compare the declared digest.
2. Compute the resolution-event digest from RFC 8785 canonical bytes with `event_digest_sha256` and the separate `attestation` envelope omitted. The attestation must name that recomputed digest. This avoids circularly hashing a signature which itself signs the digest.
3. Verify each source artifact digest against retained/captured bytes, not just its declaration. Capture time and source revision are preserved; newer publications are successor evidence, not retroactive deletion of older publications.
4. Check semantic time ordering (event cutoff, source publication, source capture, resolution deadline, challenge window) under one declared UTC/time policy. JSON Schema's `format` is structural, not a cross-field temporal proof.
5. Verify issuer, signer credential, signature scheme and signature cryptographically; bind the signature to the recomputed canonical event digest.
6. Verify `authority_policy_id` and `authorization_evidence_ref` against the exact contract's allowed resolver/adjudicator roles and jurisdiction profile. Policy IDs and refs are not self-authenticating.
7. Check event indices are contiguous, predecessor digests match the preceding verified record, one contract digest is used consistently, and no fork/duplicate terminal finalization exists.
8. Distinguish proposal, challenge, decision, finalization and void states. A schema-valid proposal is not final truth. A decision without verified authority cannot finalize a market.

## Symthaea integration

Symthaea records a forecast only with an exact `contract_id + revision + contract_digest_sha256`, forecast timestamp, target horizon, model/version digest, input-data known-at time, probability/distribution, and preregistered scoring protocol. Score only against a qualified resolution outcome for the same immutable contract commitment.

Keep the resulting axes separate:

- forecast calibration and scoring uncertainty;
- source concentration / dependence uncertainty;
- price freshness and market liquidity;
- volume versus qualified risk transfer;
- resolution completeness, disputes, corrections and authority provenance.

Do not boil these into a universal credibility score. In particular, forecast skill is not identity, governance or settlement authority.

## Incentives and market integrity

Separate forecast accuracy rewards, liquidity-making compensation, trading P&L and reputation/governance signals. Do not award forecast credibility by turnover or transaction count. If a venue separately subsidizes liquidity, evaluate executable spread/depth, uptime, slippage, inventory risk and adverse selection after costs.

Mycelix may provide issuer identity references, policy-bound attestations, append-only dispute/resolution lineage, source provenance receipts and independent-review evidence. That is an evidence/coordination role, not a claim that decentralized consensus can determine external truth by itself.

## Compatibility and versioning

- v1 intentionally limits the outcome space to a fully specified binary event. Categorical, interval, conditional, composite and scalar outcomes need separately versioned schemas and scoring semantics.
- Unknown data remains unknown; it is not coerced to `false`, zero, non-violation, or finalized.
- Unsupported or ambiguous contracts should not enter a qualified market book until the contract or jurisdiction profile has a declared interpretation.
- JSON Schema defines the structural contract. The focused reference validator additionally recomputes the RFC 8785/SHA-256 contract and event commitments. It still does not verify signatures, source-artifact bytes, signer credentials, authority, venue interoperability, fairness, anti-manipulation effectiveness or compliance.

## Benchmark gates

All targets are proposals, not measured results:

1. **Contract corpus:** 100% accept on frozen valid fixtures and 100% reject on frozen structural negative controls; mutation each required field independently.
2. **Digest binding:** 100% reject after changing any committed field without recomputing the correct digest; independently recompute bytes/digests in a second implementation.
3. **Lineage:** 100% reject on duplicate sequence, gap, bad predecessor, fork, wrong contract digest, duplicate finalization, or a finalization with absent/unauthorized policy receipt.
4. **Source provenance:** detect same-origin republications as one upstream provenance family; missing source or retrieval time remains explicit.
5. **Resolution stress:** challenge/void/late-correction/primary-source-unavailable cases replay deterministically and preserve superseded evidence.
6. **Forecast join:** no score is computed for mismatched contract digests, event semantics, horizon, or unqualified/void outcome; report Brier/log score and uncertainty after real prospective outcomes only.
7. **Operational targets:** do not set latency, uptime or settlement-throughput SLOs until a measured implementation and named hardware/deployment profile exist.

## Relationship to existing Mycelix work

This is complementary to AC-021's regenerative-market substrate research and existing trust/evidence/finance primitives. It does not replace the substrate architecture, create a new token, or grant governance powers. It also reuses rather than duplicates Symthaea's Futures forecast/scoring types.

## Nonclaims

No live platform data, market-integrity finding, forecasting improvement, trading profitability, legal permission, source authenticity, independent adjudication, external-world truth, cryptographic validity, finalization authority or production readiness is established here.
