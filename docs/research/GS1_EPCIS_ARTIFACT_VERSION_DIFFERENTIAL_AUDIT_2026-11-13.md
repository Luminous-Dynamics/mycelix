# GS1 EPCIS Artifact-Version Differential Audit — 2026-11-13

**Status:** source-page and rendered-schema inspection only; raw artifact bytes, content hashes and validator runs are not yet captured.  
**Owner:** [MYC-GS1-CONFORMANCE-001 #5395](https://github.com/Luminous-Dynamics/mycelix/issues/5395)  
**Parent programme:** [SUPPLYCHAIN-INTEROP-001 #6105](https://github.com/Luminous-Dynamics/mycelix/issues/6105)  
**Companion register:** [SUPPLYCHAIN_INTEROP_BASELINE_REGISTER_V0_1_2026-11-13.md](SUPPLYCHAIN_INTEROP_BASELINE_REGISTER_V0_1_2026-11-13.md)  
**Machine-readable record:** [GS1_EPCIS_ARTIFACT_VERSION_DIFFERENTIAL_AUDIT_2026-11-13.json](GS1_EPCIS_ARTIFACT_VERSION_DIFFERENTIAL_AUDIT_2026-11-13.json)

## Finding

Do not use a single label such as “EPCIS 2.0.1 artifacts” as a substitute for pinning the actual standards document, normative references, artifact URLs and artifact bytes independently.

The public GS1 index currently labels its combined page “EPCIS / CBV 2.0.1”. The GS1 archive says EPCIS 2.0.1 was published on 1 July 2025 and links to CBV 2.0.0. Yet the EPCIS PDF reached from that current page identifies itself as **EPCIS Standard, Release 2.0, Ratified, June 2022**, and its section 10.3 explicitly references the **2.0.0** JSON Schema and SHACL URLs. Separately, the current artifact index exposes an EPCIS JSON Schema whose declared `$id` is under the 2.0.1 path.

These are not interchangeable source identities. The right approach is a two-track lock:

1. **Normative conformance lock:** consume the exact 2.0.0 validator URLs named by the ratified EPCIS 2.0 text.
2. **Current-index compatibility lock:** capture the separately exposed current 2.0.1 artifact set and compare it to the normative set without silently substituting one for the other.

This version separation is itself a test target: a validator must prove which artifact bytes it consumed, not merely display a standard-version label.

## Verified source observations

### 1. Current GS1 artifact index

The current [EPCIS / CBV artifact index](https://ref.gs1.org/standards/epcis/artefacts) is headed “EPCIS / CBV 2.0.1”. It lists:
- EPCIS and CBV standard documents;
- an EPCIS JSON-LD context;
- an EPCIS JSON Schema;
- EPCIS SHACL shapes;
- EPCIS and CBV ontology files in JSON-LD and Turtle;
- additional REST, query and XML artifacts.

The [GS1 archive](https://ref.gs1.org/standards/epcis/archive) identifies version 2.0.1 as published 1 July 2025 and links to the versioned specification and artifact page. It lists CBV 2.0.0 alongside that EPCIS version.

The 2.0.1 JSON Schema is available from the current artifact index at:
- Link URL: `https://ref.gs1.org/standards/epcis/epcis-json-schema.json`
- Declared schema `$id`: `https://ref.gs1.org/standards/epcis/2.0.1/epcis-json-schema.json`
- Declared dialect: JSON Schema Draft 07 (`http://json-schema.org/draft-07/schema#`).

The distinction between the current page's link URL and the schema's declared `$id` must be retained in the source lock. Do not overwrite either with a guessed canonical URL.

### 2. Normative EPCIS 2.0 validation references

The [EPCIS specification PDF reached from the current 2.0.1 page](https://ref.gs1.org/standards/epcis/2.0.1/) identifies itself internally as “Release 2.0, Ratified, Jun 2022”, document version 2.0.

Its section 10.3 says:
- the JSON representation SHALL validate against `https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json`;
- the JSON-LD representation SHALL validate against `https://ref.gs1.org/standards/epcis/2.0.0/epcis-shacl.ttl`.

The same 2.0.0 references are repeated in section 14.7's JSON/JSON-LD conformance requirements. Section 10.3 therefore remains the controlling locator for the normative EPCIS 2.0 validation files in this audit.

The exact 2.0.0 JSON Schema URL is retrievable by the browser reader. The returned document declares:
- `$id`: `https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json`;
- `$schema`: `http://json-schema.org/draft-07/schema#`.

The JSON-LD context and SHACL direct URLs were reached, but the browser reader rejected their media types (`application/ld+json` and `text/turtle`) before rendering their contents. This does **not** establish an HTTP 404 or successful raw-byte capture. The same distinction applies to the ontology links.

### 3. Interpretation and open question

The publicly served 2.0.1 index, the internal metadata of the linked PDF, and the normative artifact references expose different version signals. This audit does not infer why GS1 presents them this way, nor does it claim that the 2.0.1 artifacts are invalid or non-normative for every use.

The safe engineering conclusion is narrower:
- retain the specification PDF revision and its normative references;
- retain the artifact-index page and every linked artifact URL;
- capture exact bytes, final URLs, media types, retrieval time, byte length and SHA-256;
- compare 2.0.0 and 2.0.1 artifacts before selecting an additional compatibility profile;
- ask GS1 for clarification if the source metadata still leaves the normative relationship ambiguous after byte capture.

Do not let a source page heading, HTTP alias, schema `$id`, or filename silently decide which validator is authoritative for a conformance claim.

## Required evidence lock

For each artifact and each version, record:

| Field | Required treatment |
|---|---|
| Standard document | Exact PDF URL, document version/date/status, page and clause |
| Artifact index | Exact landing-page URL, page title, retrieval time and digest if captured |
| Artifact locator | Exact anchor/link URL and declared `$id` separately |
| Retrieval | HTTP status, redirect chain, final URL and media type from an independent HTTP client |
| Bytes | Byte length and SHA-256 of the exact payload consumed |
| Rights | Source/license/usage decision; no assumption from public accessibility |
| Validator | Implementation, version, dialect, options and runtime |
| Receipt | Input digest, artifact digest, validator/configuration digest, result and diagnostics |

Unknown fields stay null/unknown. Browser-reader parse failures must not be encoded as server HTTP failures. A hash of a metadata manifest does not stand in for hashes of the normative payload files.

## Qualification sequence

1. Fetch the exact 2.0.0 context, JSON Schema, SHACL and CBV ontology artifacts using an independent HTTP client; preserve response metadata and bytes.
2. Fetch the current-index 2.0.1 artifact set independently and preserve it as a separate source generation.
3. Compare the artifacts structurally and semantically; report added, removed, changed and unchanged predicates/terms, with canonicalized diff rules and raw digests.
4. Validate the frozen positive and negative EPCIS fixtures using Draft-07 JSON Schema, JSON-LD expansion and SHACL as distinct stages.
5. Record separate receipts for each validator. Do not collapse them into a generic “GS1 valid” boolean.
6. Only after the normative source lock passes, test independent server capture/query/export/round-trip behaviour.

## Claim ceiling

This audit confirms public source-page contents and the rendered JSON Schema declarations described above. It does not claim that all source bytes were captured, that the artifacts passed validation, that Mycelix conforms to EPCIS/CBV, that any external server is conformant, or that any physical event occurred. No content hash, validator result, GS1 endorsement or SOTA result is claimed.


## 2026-11-14 artifact retrieval and mirror discrimination

A fresh web inspection opened the official [EPCIS/CBV 2.0.0 artifact index](https://ref.gs1.org/standards/epcis/2.0.0/artefacts). It lists the 2.0.0 JSON-LD context, JSON Schema and SHACL files, plus EPCIS and CBV ontology files. The exact [2.0.0 JSON Schema](https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json) rendered successfully in the browser reader as JSON and declares:
- `$id`: `https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json`
- `$schema`: JSON Schema Draft-07.

This confirms that the schema endpoint is browser-renderable now; it does **not** provide captured raw response bytes, final URL/header metadata, a byte length, or a SHA-256 digest. No validator was executed. The exact context and SHACL endpoints still fail at the browser layer with unsupported content-type (`application/ld+json` and `text/turtle` respectively); that is not an HTTP 404.

A repository-mirror check found `gs1/EPCIS/epcis-context.jsonld` at commit `6f71468231838e424bb5dfa836e006e8dc69615c`, blob SHA (Git object ID) `484faf6e256bdd159dbc4be442f68fb73018eb95`. The pinned commit message identifies it as **“Hosted public review draft v2.1 of epcis-context.jsonld”**. It is therefore not accepted as a substitute for the exact 2.0.0 context named by the normative artifact URL. Its Git blob ID is not a SHA-256 of the official 2.0.0 HTTP payload.

A direct local HTTP attempt failed before connection at DNS resolution for `ref.gs1.org`; no HTTP status can be inferred. Keep every raw-byte, final-response, digest, JSON-LD and SHACL field unknown until an independent HTTP client captures the exact official 2.0.0 bytes.

**Disposition:** improved locator/version knowledge, but no change to the raw-artifact acceptance gate. The 2.0.0 and 2.0.1 source sets remain separate; the GitHub v2.1 review draft remains a rejected substitute.
