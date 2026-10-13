# mycelix-business-export-completeness

A small, dependency-light contract for evaluating structural completeness evidence from source-system exports.

The crate checks that a declared paginated export is internally coherent: pages are ordered, cursor hand-offs are continuous, one snapshot/query/schema/source-contract/window is used, the final page is explicitly terminal, and a provider coverage statement matches the manifest's declared scope and row count.

## What a successful assessment means

It means only that the supplied evidence is **structurally consistent** with the v0.1 contract and that a source statement claims coverage for the declared query. It does not prove the statement was actually issued by the named provider, that the provider query semantics include every economically relevant event, that the upstream system recorded every real-world event, or that a supplied metric has legal/accounting authority.

Authentication, issuer authority, independently issued control reconciliation, source-schema semantics, and real-world completeness remain separate qualification steps. The crate holds no provider credentials, performs no network access, and grants no authority or business-write capability.

The API deliberately rejects incomplete pagination chains and scope drift instead of inferring completeness from a successfully parsed file. It also emits deterministic SHA-256 digests over the ordered manifest and parsed coverage projection, plus a combined evidence digest. The additive `assess_export_completeness_with_raw_payloads` entry point checks that each supplied raw page byte slice matches its declared `payload_digest` and that the exact raw coverage-statement bytes match `statement_digest`. It returns the distinct `RawPayloadBoundExport` type with a domain-separated binding digest, so downstream evidence can distinguish byte-bound results from structural-only assessments. This binds supplied bytes to declared digests but does not authenticate the provider or prove that the caller-derived semantic projection was parsed from those same bytes.

See docs/business/HOSPITALITY_SOURCE_EXPORT_COMPLETENESS_CONTRACT_V0_1.md for the design boundary and proposed qualification ladder.
