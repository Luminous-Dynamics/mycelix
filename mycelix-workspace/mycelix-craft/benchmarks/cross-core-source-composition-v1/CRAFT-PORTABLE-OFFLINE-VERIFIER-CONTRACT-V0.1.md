# Craft Portable Offline Assurance Bundle and Verifier Contract V0.1

**Status:** research contract and frozen test vectors; no production cryptographic verifier is claimed  
**Bundle purpose:** permit an independent relying party to evaluate an exported decision-evidence package without access to the producer's live database or control plane.

## 1. The core paradigm shift

A portable assurance bundle is not a "verified=true" badge. It is a **proof-obligation vector** whose dimensions are independently evaluated and whose unresolved components constrain the claims a relying party may make.

The verifier separates at least these questions:

1. Are the exact artifact bytes intact under the declared canonicalization and digest profile?
2. Does the signature validate under an identified key, and is that key bound to the claimed issuer?
3. Is the signed statement registered in a transparency service with a valid inclusion receipt?
4. Was the issuer/key/policy trusted at the relevant time under an explicit trust snapshot?
5. Are every cross-artifact reference, digest, schema, scope, frontier and time binding exact?
6. Does the declared source universe have independently supported coverage, or is the submitted corpus only a subset?
7. Does the OCEL/event projection preserve the declared semantics and provenance?
8. Does the authority receipt cover this principal, action, resource, scope and validity interval?
9. Is the independence claim supported by a failure-domain/dependency model?
10. Is the physical or service outcome independently observed and qualified?

Passing one obligation MUST NOT imply another. A digest proves byte identity relative to a chosen representation; a signature authenticates a statement by a key; transparency inclusion demonstrates registration. None by itself proves completeness, truth, independence, policy legitimacy or physical outcome.

## 2. Interoperability boundary

- Use OCEL 2.x as the object-centric event interchange boundary. Preserve event-to-object and object-to-object relationships, qualifiers, object/event attributes and time-varying object changes. Treat a projection that drops relations or changes object semantics as a semantic failure, not a harmless serialization detail.
- Consume SCITT-compatible signed statements and transparency receipts where available. Do not define a new signature envelope or transparency log in this workstream.
- Bind DBC, EAP, source admission, frontier, OCEL corpus/projection/replay, policy, authority, action plan, execution transcript and outcome receipts by explicit typed references and digests.
- DBC, EAP and post-acquisition DBC remain distinct immutable artifacts. The EAP is not embedded into or hashed by the prior DBC.

OCEL 2.1 pre-release 2.1.0pre5 (2026-10-01) adds compact CSV and bundled CSV/Parquet exchange forms; the verifier contract must avoid assuming JSON is the only valid OCEL transport. Its semantics still need to be preserved across those forms.

SCITT receipt verification is an interoperability mechanism, not a truth oracle. The relying party's trust decision about the transparency service and issuer remains explicit and policy-bound.

## 3. Bundle contents

A V0.1 bundle manifest MUST identify:

- bundle schema/profile ID and version;
- canonicalization profile and digest algorithm;
- bundle creation time, decision valid time and declared verification time;
- DBC reference and digest;
- optional EAP reference and digest, if a proposal exists;
- exact source/admission receipt references;
- source-universe declaration and its owner/inventory receipt, or explicit NotEvaluated;
- causal frontier and declared per-stream cuts;
- OCEL profile, source corpus, temporal projection and replay references;
- policy and authority receipts, including action/resource/scope/subject/time constraints;
- execution request/transcript and outcome references, with Unknown distinct from NotEvaluated;
- trust snapshot reference, issuer/key identity, key validity and revocation freshness metadata;
- transparency service identity, policy ID, signed statement ID, inclusion receipt and verification profile;
- optional failure-domain/dependency profile;
- content hashes and byte locations for all disclosed payloads;
- withheld/undisclosed payload declarations and disclosure policy;
- explicit claims, unknowns, conflicts, unsupported semantics and proof-obligation results.

Every reference is typed. Identical digests under different schemas do not make artifacts interchangeable. URI resolution is optional for verification: exported bytes and declared proof material must be sufficient for the selected offline profile.

## 4. Proof-obligation lattice

Each obligation has exactly one state:

- `EstablishedWithinProfile`
- `Rejected`
- `Indeterminate`
- `NotEvaluated`
- `UnsupportedByProfile`
- `NotApplicable`

The verifier MUST preserve the distinction between `Indeterminate` (conflicting or insufficient evidence) and `NotEvaluated` (no evaluation was performed). An absent proof MUST NOT default to Established.

Obligations are:

| Dimension | Meaning | What it does not prove |
|---|---|---|
| integrity | Exported bytes match declared canonical digest | Who authored them or whether content is true |
| authentication | Signature verifies and key maps to the asserted issuer | Issuer competence or honesty |
| transparency | Correct statement inclusion under selected log/profile | Completeness of all statements or truth |
| trust_at_time | Trust/revocation snapshot supports key at stated time | Universal trust or present-day validity |
| binding | Cross-artifact IDs, schemas, digests, scope and time match | Semantic correctness |
| frontier | Causal cuts, parent closure and ordering satisfy profile | Entire source universe exists |
| universe_coverage | Declared inventory coverage is independently supported | Unlisted sources do not exist unless the universe boundary is justified |
| semantic | OCEL/profile transforms preserve required event/object meaning | Truth of original source assertions |
| authority | Principal and action are within an exact authorization envelope | Physical execution or outcome |
| independence | Common-cause/failure-domain constraints justify separation | Truth or authenticity of source data |
| effect | Execution/outcome evidence supports a declared effect | Broader causal attribution |
| truth | Domain-specific validators support a bounded factual claim | Universal truth beyond the validator/profile |

## 5. Overall verdict reduction

The overall verdict is one of:

- `InvalidBundle`: a required integrity, signature, typed binding, schema, scope, or semantic invariant is rejected.
- `Indeterminate`: material required proof obligations are Indeterminate or required verification material is unavailable.
- `AdmissibleWithLimitations`: all mandatory integrity/binding checks pass, but one or more claim dimensions are NotEvaluated, Unsupported, or outside the profile. The output must enumerate the ceiling.
- `QualifiedWithinDeclaredProfile`: every proof obligation required by the exact profile is Established or explicitly NotApplicable, all declared claim ceilings are satisfied, and no unresolved mandatory evidence or conflicts remain.

An aggregate verdict is only a reduction for convenience. The complete proof-obligation vector is authoritative and must be emitted alongside it. No policy may silently convert NotEvaluated or Unsupported into Established.

## 6. Trust snapshot and temporal verification

A verifier must pin:
- trust snapshot digest and issuer;
- trust snapshot validity interval;
- key identifiers, key usage and issuer binding;
- revocation state and sequence/epoch;
- transparency service identity and verification profile;
- statement issuance, registration and relevant decision times;
- the rule used to evaluate trust at the decision's valid time.

An unavailable, stale, incomplete or unverifiable revocation snapshot yields Indeterminate or NotEvaluated according to the profile. It does not yield "not revoked". A newer revocation does not retroactively alter what was verifiable at an earlier time, but it may constrain present reliance. Preserve both as separate time-indexed results.

## 7. Source-universe coverage and selective disclosure

This is the central anti-overclaim requirement.

A source list proves only which sources are listed. A signed inventory proves which inventory was signed. A transparency receipt proves a statement was registered. None proves that the inventory contains every relevant source.

Completeness requires a separately declared universe boundary, an accountable owner/authority for that boundary, an inventory method, expected partitions/streams, per-partition cuts, and an independently checkable coverage/reconciliation receipt. When any component is absent, report universe coverage as NotEvaluated or Indeterminate and cap completeness claims.

Selective disclosure may prove that disclosed records are intact while remaining unable to prove that withheld records are irrelevant. A disclosure proof must state the selection rule and coverage scope; no inference from absence is permitted unless the relevant completeness proof is present.

## 8. Failure model and required adversarial tests

The verifier suite must include:

1. single-byte payload mutation;
2. signature by an unknown or wrong key;
3. valid signed statement with missing/invalid transparency inclusion;
4. stale or unavailable trust/revocation snapshot;
5. cross-schema digest substitution;
6. EAP bound to a different DBC frontier/scope;
7. missing source partition while source feed remains correctly signed;
8. selectively disclosed subset presented as complete corpus;
9. duplicate or equivocated statement IDs;
10. stale policy or authority scope mismatch;
11. event/object relation or object-change dropped during OCEL conversion;
12. causal parent missing from selected frontier;
13. same timestamp incorrectly treated as causal order;
14. shared calibration/provider dependency falsely claimed independent;
15. unknown execution outcome followed by same-scope mutation attempt;
16. valid transparency inclusion for a false or unqualified factual statement.

Positive controls must include a complete, valid reference bundle and an integrity-valid but claim-limited bundle. Negative controls must fail closed. Indeterminate controls must remain indeterminate; they must not be scored as either success or a safe pass.

## 9. Benchmarks and proposed targets

| Metric | Proposed target |
|---|---:|
| Integrity/binding mutation detection | 100% frozen KATs |
| Wrong-key rejection | 100% frozen KATs |
| Missing proof upgraded to Established | 0 |
| False source-universe completeness | 0 |
| False semantic-preservation claims | 0 |
| False authority claims | 0 |
| False independence claims | 0 |
| Unknown-outcome same-scope mutation violations | 0 |
| Offline verification requiring producer online access | 0 for offline profile |
| Deterministic verdict agreement | 100% across independent implementations |

Report verification time, peak memory, proof bundle size, disclosed payload size, number of receipts, Merkle/coverage proof sizes, and cold/warm execution separately. Do not compare signature throughput as a proxy for end-to-end assurance.

## 10. Privacy and minimization

The bundle should carry the minimum evidence needed for independent verification. Where selective disclosure or external payload pointers are used, retain proofs of disclosed-byte integrity and clearly state which universe/coverage claims remain unproven. Do not put personal or sensitive raw data into public transparency logs merely to make an audit easier.

## 11. V0.1 acceptance and boundaries

Acceptance requires:
- schema and machine-readable manifest;
- deterministic known-answer cases;
- explicit proof-obligation vector and claim-ceiling reduction;
- no hidden online lookup in the offline profile;
- exact binding of profile/schema/digest/scope/frontier/trust snapshot;
- explicit NotEvaluated/Indeterminate outcomes;
- no production key custody, no new transparency protocol, no autonomous effect.

V0.1 does not prove real-world issuer honesty, field calibration, source-universe truth, human legitimacy, legal validity beyond supplied authority material, actual physical outcomes, or independence without an adequate dependency/failure-domain model.
