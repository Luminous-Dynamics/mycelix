# AEROCOMMONS-QIF-INGEST-001 — sandboxed QIF validation and extraction

Issue: https://github.com/Luminous-Dynamics/mycelix/issues/6366  
Crate: `tools/aero-qif-ingest-qualification`  
Schema: `docs/aerocommons/AERO_QIF_INGEST_QUALIFICATION_V1.schema.json`

## Rationale

The QIF 3.0 ecosystem already provides XML Schema validation, generated language bindings, and normative XSLT checks. Rebuilding those would duplicate upstream validation and create another source of truth.

The remaining gap is the **qualification wrapper**: freeze all raw documents and validator dependencies, execute linked-document validation in a disposable offline worker, prove what source fields were extracted, and preserve each stage's result without collapsing them into one green check.

This work is separate from:
- #6361 / QIF xId trace resolution (normalized graph integrity);
- #6354/#6355 / adapter-run receipts (execution/artifact provenance);
- #6358/#6360 / adapter replay reproducibility;
- #6348/#6349 / cross-format semantic preservation.

## Upstream baseline inspected

Repositories:
- QIF community: https://github.com/QualityInformationFramework/qif-community
- QIF validation tools: https://github.com/QualityInformationFramework/qif-validation-tools

Pinned compatibility source baseline:
- Repository commit: `39843e5cc696c21ce0f8b20537d01893198f3854`
- validation-tools README blob: `6ac15d6780512481fab5f293dffcd46ac922de42`
- XSD validator `Schema Validation Tool/src/Program.cs`: blob `06041bc28404ae50db2cf06ac5b91907e4cd5686`
- XSLT runner `XSLT Check Tool/src/Program.cs`: blob `4ab71f660b835848a70acbc888d2f1523b03bb27`
- .NET project file: blob `c73e0d1692ab38cc8596bbd32ae080d903aaa778`
- check descriptions: blob `8b75f58a50d431ac0d6f56e4f94da280351e2efb`

The XSD checker and XSLT tool are separate programs. The XSLT checking set covers Format, Quality and Semantic categories and linked QIF documents; the 3+ preview includes proposed future checks. Keep the normative QIF 3.0 profile separate from the 3+ preview.

## Security findings that drive the design

The upstream XSLT runner enables `document()` and script execution and loads stylesheets via `XmlUrlResolver`. Its linked-document checks may read external QIF documents. That behavior is important for QIF but turns untrusted QIF/XSLT into a filesystem/network execution boundary if run with ambient privileges.

Both upstream helper projects currently target .NET Core 3.1. Microsoft ended support on **13 December 2022**. As of **20 November 2026**, .NET 10 is an active LTS release with support through **14 November 2028**; the supported-runtime port should target a pinned patched .NET 10 runtime and must demonstrate output-equivalence before promotion. Do not silently retarget and assume equivalent behavior.

Official runtime status:
- .NET support policy: https://dotnet.microsoft.com/en-us/platform/support/policy
- .NET Core 3.1 lifecycle: https://learn.microsoft.com/en-us/dotnet/core/whats-new/dotnet-core-3-1
- .NET 10 announcement/support: https://devblogs.microsoft.com/dotnet/announcing-dotnet-10/

## Offline execution contract

The crate's metadata verifier requires a declared profile with:
- network disabled;
- rootless execution;
- no host secrets;
- staged input read-only and output write-only;
- nonzero, bounded CPU, memory, wall-time, file-size, document-count and link-depth limits;
- a content-addressed policy artifact and a runner-attestation artifact.

But these fields are claims, not enforcement. The runner implementation must separately provide evidence that:
1. every input document, schema, stylesheet, include/import, helper script, parameter and extension object is in a frozen manifest;
2. URI resolution maps only to the staged QPId/path manifest; external HTTP(S), arbitrary `file://`, UNC paths and host filesystem paths are denied;
3. archive extraction rejects absolute paths, traversal, symlinks/hardlinks and expansion bombs before staging;
4. inputs are mounted read-only; outputs are written only to an isolated output directory;
5. the process runs rootless, with no host credentials, sockets, environment secrets or network;
6. CPU, memory, wall-time, input bytes, document count and linked-document depth are enforced by the runtime, not just declared in JSON;
7. runner measurement/attestation is independently issued and verified by a separate trust service.

The current Rust crate does not implement the runner. It validates the qualification envelope and verifies exact artifacts only.

## Stage model

| Stage | What it means | What it does not imply |
|---|---|---|
| XML well-formedness | XML parsing succeeds | QIF XSD validity |
| XSD structure | QIF XSD and same-document key/keyref checks pass | normative XSLT pass |
| XSLT format | QIF format checks pass | quality or semantic pass |
| XSLT quality | QIF quality checks pass | semantic or metrology validity |
| XSLT semantic | QIF semantic and linked-document checks pass | extractor fidelity |
| Extraction fidelity | every required source field is mapped or explicitly classified | source validity or cross-format preservation |
| External-reference resolution | normalized QIF trace graph resolves | raw extraction fidelity or physical truth |

Warning, Unknown, Failed, Blocked and NotRun remain explicit. A downstream Passed stage is rejected unless all declared prerequisites passed. A stage with Failed/Warning/Unknown requires reason codes; Blocked requires a reason and no fabricated tool/report evidence.

## Source URIs vs staged paths

A QIF external URI is preserved as inert provenance metadata and is optional where QIF permits it. The staged relative path is a separate required field. The verifier never requires the URI to equal the local path and never dereferences the URI. A path is constrained to relative ASCII path components without dot traversal, backslashes, drive prefixes, percent encoding or control characters. The worker must additionally defend against symlink races and path aliasing.

This separation avoids rewriting source evidence just because an offline runner uses a different local staging layout.

## What the first Rust increment does

- strict Serde structures with unknown-field rejection;
- document QPId uniqueness and exactly one Results document;
- Plan documents must be linked from Results through explicit local-reference-to-Plan bindings;
- URI metadata is retained but not fetched;
- safe relative staged paths and one-to-one path-to-Plan mapping (multiple Results references may alias the same staged Plan);
- XSD and XSLT entrypoint/dependency manifests, including entrypoint membership and duplicate-path detection;
- tool source commit SHA and executable content-artifact binding;
- runtime-image binding and explicit .NET Core 3.1 legacy classification;
- sandbox policy / runner-attestation references;
- all seven stage results exactly once, including NotRun and Blocked;
- stage/tool-type compatibility and prerequisite ordering;
- extraction accounting identity: mapped + unresolved + ambiguous + unsupported + dropped = total required;
- unique source locators, source QPId/document membership, valid Results-local external-reference binding and target/reason-code consistency for each mapping row;
- recomputed canonical source-mapping-manifest bytes and SHA-256 must match the declared manifest artifact exactly;
- no Passed extraction with unresolved, ambiguous, unsupported or dropped mappings;
- exact byte verification for every document, schema, stylesheet dependency, runtime, tool, policy/runner reference, stage report and extraction output;
- deterministic SHA-256 receipt over a versioned canonical JSON profile.

## Benchmark targets

Targets, not achieved results:
- 100% frozen-input/tool/schema/XSLT/report digest reproducibility;
- zero network and zero host-path access in hostile-input containment tests;
- 100% rejection of URI traversal, absolute path, symlink, archive bomb, cyclic-link and recursion-limit controls;
- zero source-to-normalized silent drops;
- 100% one-to-one accounting of every required source field as a unique source-locator row classified mapped/unresolved/ambiguous/unsupported/dropped;
- 100% agreement between typed mapping rows, declared disposition counts and the exact source-mapping-manifest bytes;
- zero duplicate source locators, invalid external-reference joins or mapped rows lacking normalized targets;
- zero false Passed extraction when any mapping is unresolved, ambiguous, unsupported or dropped;
- zero downstream Passed stages with an unpassed prerequisite;
- 100% XSD and normative XSLT outcome capture, warnings included;
- .NET 10 port output equivalence with the pinned compatibility implementation on a rights-cleared frozen corpus, or an adjudicated difference set with no silent divergence;
- stable normalized extraction and trace output under harmless input ordering changes.

Report separately: validation coverage by stage, linked-document coverage, extraction coverage, false-preservation/wrong-join rate, unsupported-field inventory, runner containment violations, report reproducibility, p50/p95 wall time and peak memory.

## Rights and standards boundary

The validation-tools repository uses the Boost Software License 1.0 for its own tooling. That does not automatically grant redistribution rights for every normative XSD, XSLT, or CAx-IF sample bundled elsewhere. Pin source and digests first; vendor/copy only after verifying the relevant artifact-specific rights and notices.

The current CI corpus must remain synthetic until the official CAx-IF Test 7/8 packages and normative QIF 3.0 schema/check bundles have rights-cleared acquisition and redistribution status.

## Claim ceiling

A successful verifier result proves structural consistency of this envelope and exact byte identity for every listed payload. It does not prove the validator actually executed, the worker was isolated, tools were authentic, QIF XML passed its XSD/XSLT checks, raw extraction was faithful, AP242/QIF meaning was preserved, metrology was valid, or a part is acceptable or airworthy.
