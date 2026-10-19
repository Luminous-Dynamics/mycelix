# Humanity Atlas evidence binding to Mycelix-DeSci v0.1

**Status:** integration proposal; documentation only. No production connector, scientific event, signed receipt, or live submission has been created by this document.

## Purpose

The Humanity Atlas scenario contract currently carries SHA-256 digests for exact input and evaluation-artifact bytes. Those digests detect byte changes when a trusted copy is available, but do not establish publisher identity, authority, review scope, or scientific validity.

Mycelix-DeSci already defines a canonical signed scientific-event path, content-addressed evidence artifacts, typed attestations, an append-only event stream, and independently signed receipt-time authority evidence. The Humanity Atlas should reuse that path rather than introduce a second signature format or use the legacy mutable claim endpoints.

Relevant existing implementation and protocol references:

- \`mycelix-desci/src/core/src/scientific_events.rs\` — \`ResearchObject\`, \`AtomicClaim\`, \`EvidenceArtifact\`, \`Attestation\`, \`ScientificEventPayload\`, signed canonical event envelopes and hash-chained claim streams.
- \`mycelix-desci/src/core/src/scientific_authority_audit.rs\` — \`ScientificAuthorityReceipt\`, exact event binding, receipt-time identity/authorization snapshot and prior-receipt chaining.
- \`mycelix-desci/src/core/src/authority_signing.rs\` — pluggable Ed25519 signer boundary; callers verify returned signatures before persistence.
- \`mycelix-desci/docs/AUTHORITY_RECEIPTS.md\` — receipt journal, crash recovery, historical validity and trust/rotation requirements.
- \`mycelix-desci/README.md\` — canonical-path architecture and explicit legacy compatibility boundary.

The project README describes Mycelix-DeSci as undergoing an architectural refoundation. Treat this as a proposed mapping against source-defined types, not proof that a public production service is deployed or ready for this integration.

## Core distinction: integrity, authority, scientific validity

These are three separate questions and must not be collapsed into one status.

1. **Integrity:** do these exact bytes match the declared artifact digest? SHA-256 is used by the current Humanity Atlas run contract. Mycelix-DeSci's \`ContentHash\` is a BLAKE3 digest. When bridging the same artifact, calculate and preserve both digests from the same frozen bytes; never relabel one algorithm as the other.
2. **Authority / provenance:** which authenticated actor submitted or reviewed these exact bytes, under what authorized role and policy revision, and when did the canonical event service accept the event? This is what the signed event and authority receipt can help establish.
3. **Scientific validity:** do the methods, evaluation data, metrics and results justify the model claim for its intended use? A signed event or authority receipt cannot answer this by itself. It records a governed action and its provenance; it does not certify the scientific conclusion.

A valid signature over a false evaluation report is still a valid signature over a false report. The UI must keep these distinctions visible.

## Proposed mapping

| Humanity Atlas concept | Mycelix-DeSci representation | Boundary |
|---|---|---|
| Scenario or model report package | \`ResearchObject\` with type \`Model\`, \`Workflow\`, \`Dataset\` or \`Software\`, as appropriate | A research object groups material; it is not itself an atomic truth claim. |
| Claim about model performance or a forecast evaluation | \`AtomicClaim\` scoped to the model version, intended-use domain, horizon and named metrics | Keep claims granular: e.g. "model X meets predeclared MAE threshold for region R over evaluation window W", not "model X is valid." |
| Exact validation report bytes | \`EvidenceArtifact\` attached with \`ScientificEventPayload::EvidenceAttached\` | Include content hash, media type, immutable/content-addressed locator, license and availability. The locator must resolve to the exact bytes used for the digest. |
| Reproduction of the reported run | \`AttestationKind::ComputationalReproduction\` with outcome and protocol reference | Requires evidence IDs and protocol reference. A reproduced execution proves repeatability under that setup, not real-world validity. |
| Independent reproduction in another environment | \`AttestationKind::IndependentReplication\` with outcome and protocol reference | Record independence criteria, environment and the exact model/input digests in the referenced protocol/report. |
| Review, critique, correction or withdrawal | The corresponding typed attestation/correction/withdrawal event | Preserve corrections and withdrawals in the event history; don't erase prior records. |
| Submitter, roles, policy and receipt time | \`ScientificAuthorityReceipt\` bound to the exact signed event and event hash | Receipt-time authorization and actor binding; not an independent review of empirical methods. |
| Model lifecycle/adoption decision | Existing model-governance mechanisms when their scope matches | A vote to adopt a model is normative authorization, not empirical calibration. |

## Required artifact manifest

The evaluation report should be a content-addressed artifact whose bytes are frozen before submission. Include a machine-readable manifest with at least:

- Humanity Atlas run ID and run schema version;
- scenario SHA-256, input dataset SHA-256, model/configuration SHA-256;
- BLAKE3 digest of each submitted artifact, separately from its SHA-256;
- source repository, immutable commit/tree identifiers and license;
- engine and adapter version/build identity;
- intended use, domain, horizon, target metrics, units and exclusions;
- evaluation window and time-cutoff data snapshot; a leakage statement explaining what information was available at forecast origin;
- baseline/benchmark results, metric definitions, sample sizes, calibration diagnostics when probabilities are emitted, uncertainty and sensitivity analysis;
- evaluation protocol version, predefined acceptance thresholds, failed slices and known limitations;
- reviewer/replicator independence statement and any conflicts of interest;
- artifact locators, media type, license and public/controlled/embargoed availability.

The manifest must not claim that all artifacts are available publicly if permissions or data licenses restrict them. For unavailable or controlled inputs, preserve the access classification, digest and verification route without publishing protected data.

## Canonical event sequence

The proposed workflow uses the existing scientific-event kernel and exact source-defined event types:

1. Propose a research object and atomic claim scoped to one model/version and one evaluable assertion.
2. Attach the frozen evaluation report and required supporting artifacts with \`EvidenceAttached\`.
3. Record computational reproduction and/or independent replication attestations only after those activities actually ran. Each attestation must reference the relevant evidence artifact IDs and protocol.
4. Submit each action through the signed canonical scientific-event path. Preserve the event ID, event hash, stream/claim ID and sequence number from the accepted append result.
5. Preserve the independently signed receipt-time authority receipt for each accepted event. Verify the event signature, receipt signature, exact event ID/hash binding, actor/key/role/organization resolution, receipt policy/revision and previous-receipt chain using the canonical verifier.
6. Export a reviewable evidence bundle containing the artifact manifest and the relevant event/receipt chain. The bundle is an index over canonical records, not a replacement source of truth.
7. Reconcile the exported digest/identifiers back to the Humanity Atlas run envelope. A consumer should fail closed if any required digest, event binding or receipt-chain check is missing or mismatched.

Do not submit through \`POST /api/v1/claims\` or the older mutable verify/provenance endpoints as a shortcut. The DeSci README explicitly identifies those as a legacy compatibility layer with mutation routes disabled by default; the canonical path is signed scientific events at \`/api/v1/scientific/events\`. Verify the current route/API schema and deployment readiness before building a runtime client.

## How this interacts with Humanity Atlas run classes

The v0.1 run schema is intentionally minimal. It stores SHA-256 digests on validation evidence but does not yet carry event IDs or signed authority receipts. Until a later version explicitly binds those fields, treat an \`artifact_ref\` as a locator to an exported evidence bundle and do not interpret it as proof that the bundle has been verified.

Recommended future run-contract revision:

- add a typed evidence reference with SHA-256 and algorithm label;
- add Mycelix-DeSci scientific event ID, event hash, stream/claim ID and sequence;
- add authority-receipt protocol/version, receipt hash, signer key ID/public key reference, verification status and verifier version;
- include the digest of the full export bundle and exact referenced event/receipt bytes;
- record supersession/revocation state as a new event rather than silently replacing a historical receipt.

The adapter should only report \`receipt_verified\` after checking signatures, binding fields, key-trust/revocation policy and receipt-chain integrity. If it cannot reach the verification material or the trust root is unavailable, show the record as unresolved/unverified; do not infer success from HTTP 2xx or from the presence of a URL.

Even after successful provenance verification, the Humanity Atlas scientific label rules still apply:

- \`calibrated_simulation\` requires an appropriate retrospective-evaluation artifact tied to the exact model, configuration and data digests. A signed receipt does not replace that evaluation.
- \`prospective_forecast\` requires a frozen forecast receipt with a precisely defined outcome/metric, issue time, deadline, resolution source, model/input digests, baseline and predeclared scoring rule. Evaluation results are added later as resolution records; do not overwrite the original prediction.
- Counterfactual-history reports should be marked as model-conditional and assessed for robustness across alternative causal structures; the unrealized full outcome cannot be directly scored.

## Security and governance constraints

- Use Mycelix-DeSci's canonical signing bytes and designated signer interfaces. Do not create a Humanity Atlas-specific Ed25519 serialization alongside the existing protocol.
- The event signer and receipt-service signer must remain distinct, as required by the authority-receipt design.
- Trust roots, active key epochs, revocation, credential registry revision and receipt chain must be independently verified; today's role assignments must not be retroactively substituted for receipt-time authorization.
- Keep event ID, event hash, artifact digest, authority snapshot hash, receipt hash and previous-receipt hash as distinct typed values. Do not compare or interchange hashes across algorithms or domains.
- Preserve the original report and event when a claim is corrected, withdrawn, superseded or disputed.
- Model-gov approval is a governance fact. Empirical validation evidence remains an independent requirement.
- Avoid promoting a model based on a single successful demonstration, hand-curated scenario or one sample. Require use-specific validation metrics and sensitivity/failure analysis as defined in the Humanity Atlas model-credibility gate.
- Before any production integration, threat-model event replay, stale registry revisions, compromised or rotated receipt keys, artifact locator mutability, missing receipts after journal cutover, receipt-chain gaps, untrusted receipt keys, out-of-scope attestations, and dishonest or correlated reviewers.

## Qualification plan

1. **Contract tests:** create synthetic artifacts and exercise both SHA-256 and BLAKE3 digests over the same bytes; tamper with bytes and verify failure.
2. **Canonical type mapping:** unit-test the manifest-to-\`EvidenceArtifact\` mapping, claim scope, and required protocol references for computational reproduction/independent replication.
3. **Signature/receipt tests:** use Mycelix-DeSci's existing test key patterns and canonical verifiers; reject wrong signer, changed event hash, changed artifact digest, missing receipt, stale/revoked key and broken previous-receipt chain.
4. **Crash/replay tests:** use existing authority-journal recovery semantics; ensure retries are idempotent and that an event cannot be represented as accepted without its required receipt.
5. **Cross-repository fixture:** bind a synthetic Humanity Atlas run envelope to a frozen evidence-manifest fixture and verified event/receipt fixtures. Mark all keys/actors as test-only; do not publish a synthetic fixture as real trust evidence.
6. **Real-model validation:** only after the runtime adapter exists, run a cutoff-safe historical evaluation with predeclared metrics and baselines. Preserve the report plus the canonical Mycelix provenance bundle. Keep the output as \`unvalidated_simulation\` until the intended-use evaluation passes its declared criteria.

**Current status:** this mapping is not implemented as a live connector; no Mycelix event, evaluation artifact or authority receipt is created by this document. It specifies the reuse path and the tests required before a production claim can be made.
