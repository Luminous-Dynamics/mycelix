# EPI-013R-T — OSINT tool execution receipt corpus v0.1

## Purpose

Freeze a deterministic, language-independent receipt contract for [EPI-013R / #2779](https://github.com/Luminous-Dynamics/mycelix/issues/2779), composing the six capability profiles in [EPI-013T / PR #2776](https://github.com/Luminous-Dynamics/mycelix/pull/2776).

This is a docs/test-vector child only. It introduces no runtime collector, browser/network adapter, credential authority, policy engine, admission behavior or second provenance ontology.

## Governing distinctions

```
ToolCapabilityProfileV1 != ToolExecutionReceiptV1
profile capability != run observation
run success != coverage completeness
exact-scope completeness != world completeness
tool output != source authenticity
source authenticity != claim truth
receipt integrity != semantic correctness
receipt != EPI admission
receipt != collection/action authority
```

## Receipt fields

Each receipt binds the selected EPI-013 profile and corpus commitment; actual tool/provider version; execution instance; request commitment and protected reference; purpose and policy refs; input artifacts/corpus; time assertions; terminal status; attempted/observed pages and items; errors/degradation; actual coverage and its scoped evidence; output commitments and transform ancestry; declared versus observed disclosure surfaces; retention/cache evidence class; profile drift; and the `ExecutionObservationOnly` authority ceiling.

Raw queries, credentials, cookies, bearer tokens, authorization headers and private account identifiers are forbidden in generic receipt serialization. Opaque refs and commitments can still leak information, especially for low-entropy requests; publication requires separate OPSEC/privacy review.

## Coverage states

The frozen enum preserves:
`RankedTopKObserved`, `PaginationExhaustedUnderExactRunProfile`, `PartialDueToTimeout`, `PartialDueToTruncation`, `PartialDueToRateLimit`, `ExhaustiveWithinExactFiniteCorpus`, `AccountScopedObserved`, `TransformCompletedCoverageUnknown`, and `UnknownCoverage`.

There is no `WorldComplete` or generic `Exhaustive` state. Pagination exhaustion requires stable-snapshot and all-page evidence. Finite-corpus exhaustion requires matching corpus commitment/count and an all-entry scan. OCR remains derived and coverage-unknown.

## Deterministic fixture corpus

Schema: `mycelix:epi-013r-tool-execution-receipt:v0.1`

Corpus: `mycelix:epi-013r-tool-execution-receipt-corpus:v0.1`

Profile prerequisite: `mycelix:epi-013-tool-capability-corpus:v0.1`

Exact profile corpus SHA-256: `d499b680d6461dbffed9b4ea053d3758ee4bd4e7d3e26658d26334acddc35cf7`

Files:
- `EPI_013R_TOOL_EXECUTION_RECEIPTS_V0_1.schema.json`
- `fixtures/EPI_013R_TOOL_EXECUTION_RECEIPTS_V0_1.json`

The fixture includes 8 positive receipt cases and 12 metamorphic controls. Its compact UTF-8 SHA-256 is `fd6123f71b1c49e5bd9c9c2e6bad43914ad890c952fc1f43ad498c40140a0e37`. The schema SHA-256 is `423ef6a23a4329979c54e725c8be148282ac84bd292f5af2d6b63e646e5f07d2`. These digests identify exact authored bytes, not tool correctness.

Positive cases cover Top-K, complete pagination, timeout degradation, exact finite local corpus, OCR transform, authenticated account-scoped access, provider-version drift and unexpected disclosure. Metamorphic controls cover page deletion, timeout injection, finite-corpus mismatch, transform-version change, profile drift, secret-byte rejection, retention uncertainty and the non-upgrading effect of signatures.

## Standards relationship

RFC 9942 defines COSE receipts proving properties of verifiable data structures; RFC 9943 defines the SCITT transparency architecture for signed statements and independently verifiable receipts. They may later provide an integrity/inclusion envelope around a committed Mycelix receipt statement. They do not establish that a tool's returned content is true, that provider claims are accurate, or that semantic coverage labels are correct.

References:
- https://www.rfc-editor.org/rfc/rfc9942/
- https://www.rfc-editor.org/rfc/rfc9943/

## Symthaea relationship

Symthaea #5482 consumes exact execution receipts as read-only inputs and may produce bounded candidate interpretation. It must preserve receipt degradation and profile drift. It must not infer completeness from nominal profiles, turn zero results under partial coverage into absence claims, admit outputs into canonical EPI state, or gain collection/action authority.

```
selected capability profile
  -> separately authorized run
  -> ToolExecutionReceiptV1
  -> Symthaea candidate interpretation
  -> independent evidence/claim admission, if separately qualified
```

## State / claim ceiling

This corpus only freezes a strict shape and synthetic expected semantics. It does not prove real collection, source authenticity, truth, complete search, remote retention, privacy/legal compliance or live-adapter safety. No tool execution or collection authority is created.
