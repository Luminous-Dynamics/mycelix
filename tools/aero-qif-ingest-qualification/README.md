# aero-qif-ingest-qualification

Strict, content-addressed evidence envelope for QIF 3.0 validation, linked-document staging and source-to-normalized extraction qualification.

## Scope

This crate does **not** parse QIF XML, execute QIF validators, enforce a sandbox, verify host signatures, authenticate tool publishers or establish metrology truth. It checks that the declared source documents, complete XSD/XSLT dependency bundles, tools, runtime, sandbox-policy/runner-attestation references, stage reports and extraction outputs are internally consistent and match exact supplied bytes.

It deliberately composes the QIF community's published tooling rather than implementing a competing QIF parser or a substitute XSD/XSLT validator.

## Run

    cargo fmt --manifest-path tools/aero-qif-ingest-qualification/Cargo.toml -- --check
    cargo test --manifest-path tools/aero-qif-ingest-qualification/Cargo.toml
    cargo clippy --manifest-path tools/aero-qif-ingest-qualification/Cargo.toml --all-targets -- -D warnings

## Evidence stages

Each appears exactly once and remains separate:
1. XML well-formedness
2. QIF XSD structure and same-document key/keyref
3. XSLT format
4. XSLT quality
5. XSLT semantic/linked-document
6. source-to-normalized extraction fidelity
7. external-reference trace resolution

A later stage cannot claim Passed if its prerequisite did not Pass. Warning, Unknown, Failed, Blocked and NotRun remain distinct states; non-pass outcomes require explicit reason codes (Blocked and NotRun have their own evidence rules).

## Offline staging

Original QIF URIs are inert provenance metadata. They are **never dereferenced** by this crate. Each external Plan QPId is separately mapped to a safe relative path in the sealed staging root. The path validator rejects absolute paths, traversal components, backslashes, percent-encoding and non-ASCII path characters. The runner must independently enforce a read-only staged-input mount, write-only output mount, no network, rootless identity, no host secrets and hard resource limits.

The sandbox policy and runner attestation are pinned as artifacts but are not authenticated here. A declared `network=disabled` value is not proof of network isolation.

## Upstream QIF tools

The QIF community publishes QIF 3.0 language bindings and sample instances. Its validation-tools repository separately provides:
- XML Schema Validation Tool (structure and local keys/keyrefs)
- XSLT Check Tool (format, quality and semantic checks over linked files)

The pinned compatibility baseline and security analysis are in `docs/aerocommons/AERO_QIF_INGEST_001.md`. The upstream helper projects target .NET Core 3.1, which is unsupported; use it only in a disposable compatibility worker. A supported-runtime port must be compared against the pinned implementation before promotion.

## Receipt

`AeroQifIngestQualificationV1` binds a fixed schema version, all input and linked-document artifacts, XSD/XSLT dependency manifests, validator source commits and executable digests, runtime image, sandbox policy, runner attestation, individual stage reports and extraction mapping/trace outputs.

The receipt digest is SHA-256 over a versioned Rust-struct JSON serialization after deterministic sorting of unordered document/link/bundle/tool/stage arrays. It is **not** RFC 8785/JCS. Consumers must use this exact versioned canonicalization function.

## Claim ceiling

A successful call to `verify_ingest_payloads` proves only that the declared envelope is structurally consistent and all declared payload bytes match their content artifacts. It does not prove that a validator actually ran, a sandbox was enforced, a source URI was fetched correctly, QIF was schema-valid, an extractor faithfully represented raw XML, AP242 semantics were preserved, measurements were valid or a part is accepted.
