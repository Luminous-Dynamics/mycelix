# Mycelix supply-chain capability audit v0.1

**Snapshot:** 2026-11-12  
**Status:** source inspection / capability-claim audit  
**Tracking:** [Mycelix #6099](https://github.com/Luminous-Dynamics/mycelix/issues/6099)  
**Related:** [Mycelix #6098](https://github.com/Luminous-Dynamics/mycelix/issues/6098), [LEX-NET-015 #1110](https://github.com/Luminous-Dynamics/mycelix/issues/1110)  
**Claim class:** bounded public-source review; not a security audit, implementation certification or exhaustive code review.

## 1. Executive finding

There is useful supply-chain design and SDK work in Mycelix, but the publicly accessible material spans different maturity and execution paths. They must not be described as one already-qualified product.

The standalone repository `Luminous-Dynamics/mycelix-supplychain` is archived/read-only. Its README advertises signed DKG claims, Verifiable Credentials, Product Passports, replay-safe ingestion, selective disclosure via SD-JWT/BBS+, and a verifier UI/CLI. Inspection of the archived source found material gaps between those claims and code:

- the Rust crypto crate documents SD-JWT/BBS+ as future support and exposes Ed25519 primitives / compact-JWT creation, not a full selective-disclosure VC stack;
- `rust/service/src/vc.rs::verify_vc_jwt` is explicitly a TODO and returns “VC verification not yet implemented”;
- its `KeyPair::did()` formats a `did:key:` prefix followed by hex-encoded key bytes, while source comments identify this as a simple placeholder rather than a proper multicodec/multibase DID representation.

The archived code is a reference for historical design only. It is not the writable implementation target.

The active Mycelix monorepo has a TypeScript SDK supply-chain integration. It contains at least two different execution paths:
1. an in-memory `SupplyChainProvenanceService` built around local `Map` instances and `LocalBridge`;
2. a `MycelixSupplyChainClient` wrapper that calls the Holochain `supplychain` role's `inventory` and `logistics` zomes.

The active workspace inventory lists `mycelix-supplychain` as `unverified`. The monorepo README also says most clusters are pre-alpha, only a couple have live user-facing apps, and cluster maturity must be checked rather than inferred from tests or scaffolding.

## 2. Exact source surfaces inspected

### Archived standalone repository (read-only)
- `README.md`
- `rust/crypto/Cargo.toml`
- `rust/crypto/src/lib.rs`
- `rust/service/src/vc.rs`
- `rust/claim-model/src/lib.rs`
- `rust/service/Cargo.toml`
- `specs/openapi.yaml`
- `specs/examples/batch_produced.json`
- `ts/sdk/package.json`

### Active maintained monorepo
- `README.md`
- `mycelix-workspace/README.md`
- `mycelix-workspace/scripts/workspace-inventory.txt`
- `mycelix-workspace/CLAUDE.md`
- `mycelix-workspace/sdk-ts/src/integrations/supplychain/index.ts`
- `mycelix-workspace/sdk-ts/package.json`

The exact inspected active source blob for the SDK supply-chain integration was:
`265320f79df9b8d83438916e0f5417dc00d481b3`.

## 3. Capability and evidence matrix

| Capability claim | What the inspected source shows | Accurate status |
|---|---|---|
| Record supply-chain checkpoints | In-memory service stores `Checkpoint` objects in Maps and maintains a per-product ID list | Implemented as a local in-memory SDK/service path |
| Build a Holochain supply-chain client | Client wrapper calls the `supplychain` role's `inventory` and `logistics` zomes via `callZome` | Wrapper/source exists; underlying active hApp runtime and integration are not qualified by this inspection |
| Verify provenance chain | In-memory `verifyChain` checks caller-provided `evidence[].verified`, 72-hour gaps, local handler reputation and a chain-integrity threshold | Heuristic local consistency/status evaluation; not cryptographic event verification |
| Verify evidence | Checkpoint evidence exposes a `verified: boolean` input supplied by the caller | Assertion/metadata only unless a separate, qualified verifier owns that field |
| Cryptographic signature verification | Archived Rust crypto exposes Ed25519 signing and low-level verification helpers; service `verify_vc_jwt` remains a TODO | Partial cryptographic primitives; full VC-JWT verification incomplete |
| SD-JWT / BBS+ selective disclosure | Archived Rust crypto module says this is future support; no such implementation dependency was visible in its Cargo manifest | Not demonstrated in that archived implementation |
| Standards-conformant `did:key` | Archived helper emits `did:key:` + hex public key bytes and comments this is a placeholder | Do not claim standards-conformant DID encoding from this helper |
| DKG publication and lineage | Archived README/API describe projection into DKG claims and hash-linked lineage | Design/API surface exists; archived runtime cannot be updated or treated as current production capability |
| Product passport export / verifier UI | Archived README describes the feature | Requires exact deployed implementation and offline verifier evidence; not established by the source inspection |
| Holochain integration tests / deployed supply-chain app | Active workspace inventory says `mycelix-supplychain unverified` | Explicitly unverified in the inspected inventory |
| Production readiness | Active monorepo README describes Mycelix as pre-alpha and says most clusters lack deployed user-facing apps | Do not market this vertical as production-ready on this evidence |

## 4. Important implementation distinctions

### 4.1 In-memory SDK service

The local `SupplyChainProvenanceService` holds:
- `checkpoints: Map<string, Checkpoint>`;
- `handlerReputations: Map<string, ReputationScore>`;
- `productChains: Map<string, string[]>`;
- a `LocalBridge`.

Its `recordCheckpoint` method:
- accepts `productId`, optional `batchId`, location, optional coordinates, handler, action and evidence;
- reads the previous checkpoint ID from its in-memory product-chain list;
- copies the supplied evidence array into the checkpoint;
- constructs `handlerDid` as `did:mycelix:${handler}`;
- uses `Date.now()` and `Math.random()` for checkpoint ID/timestamp;
- updates local handler reputation positively when a checkpoint is recorded.

Those semantics are useful for a local demonstration or scaffold. They do not provide durable cross-node state, a cryptographic signer identity, independent evidence verification, replay-safe persistent ingestion or a standards-defined DID method.

### 4.2 In-memory `verifyChain`

The implementation marks a checkpoint as `verified` when any evidence item supplied with `verified: true` exists (including non-cryptographic types). It flags a time gap greater than 72 hours and low local handler reputation as suspicious. The returned overall boolean is based on `chain.chainIntegrity >= 0.8` and no weak links.

That function is therefore a local heuristic over stored fields. It does not verify an issuer's signature, a cryptographic source commitment, a standards-conformant credential, instrument calibration, a custody transfer, or whether an event physically occurred. Avoid phrases such as “cryptographically verifies provenance” or “confirms product provenance” for this path.

### 4.3 Holochain client wrapper

A separate client calls zomes:
- `supplychain.inventory.list_checkpoints`;
- `supplychain.logistics.verify_chain`;
- `supplychain.logistics.get_handler_profile`;
- `supplychain.logistics.is_handler_trusted`;
- `supplychain.logistics.update_handler`.

A wrapper method is not evidence that the backing zome is implemented, has the expected validation rules, persists the records, is correctly authorized, passes integration tests, or is deployed. The workspace inventory currently classifies the nested supplychain workspace as `unverified`. Keep this client path as its own qualification target.

## 5. What must change before stronger claims

### Identity and signatures
- Choose the supported DID method/profile and use its specified identifier encoding and resolution; do not build a DID from a string prefix.
- Bind issuer, key, algorithm, credential/proof type and exact payload representation.
- Verify signatures with an explicit algorithm allow-list.
- Include revocation/status, expiry and trust-policy evaluation.
- Add known-answer positive/negative vectors for wrong key, wrong issuer, changed payload, unknown algorithm, revoked/expired credential and malformed input.

### Evidence and event truth
- Replace caller-authored `verified: true` as an authoritative signal with a typed verification receipt from an identified, qualified verifier; preserve the original assertion separately.
- Keep event occurrence, observation, ingestion, issuance and verification times separate.
- Distinguish signed assertion, instrument reading, inspection, third-party certificate, custody handoff and independently observed physical event.
- Preserve contradictory claims, corrections, dispute and supersession rather than silently choosing one.

### Product and standards claims
- Do not claim SD-JWT/BBS+ until interoperable generation **and independent verification** pass external vectors and privacy/leakage tests.
- Do not claim NIST IR 8536 conformance based on a mapping document.
- Keep GS1 EPCIS, QIF, MTConnect and other sector profiles versioned and source-preserving.
- Do not call a valid lineage record proof of ownership, lawful origin, sustainability, compliance, product conformity or delivery.

### Operations and maturity
- Establish the active Holochain workspace owner, source revision, zome tests, integration harness and runtime/deployment evidence.
- Separate local-memory demo, SDK wrapper, hApp integration, durable federation and actual production service as different maturity levels.
- Keep the workspace inventory state `unverified` until the owner’s actual tests and gates pass; a documentation update must not change it.

## 6. Benchmark programme

### Deterministic software controls
- altered payload / source digest;
- wrong issuer or key, unsupported algorithm, expired/revoked credential;
- duplicate and replayed event;
- wrong product/lot/serial and wrong predecessor;
- missing chain segment and contradictory custody history;
- unknown standard version or required field;
- source transformation that drops a restrictive qualifier;
- caller-set `verified: true` with no verification receipt;
- attempted unauthorized field disclosure.

### Independent evaluation
Use deterministic cryptographic/schema/lineage oracles where possible. For subjective or physical truth claims, use qualified human/third-party source evidence. Do not let an agent or the same component that produced the claim grade its own output.

### Metrics
Report separate confusion matrices and hard gates for:
- signature/authentication correctness;
- identity and subject binding;
- event/link/lineage correctness;
- stale/revoked status;
- source preservation and standards semantic fidelity;
- selective-disclosure proof correctness and leakage;
- false “verified” claims;
- unknown/indeterminate handling;
- time, compute, payload and operating cost.

No weighted score may compensate for a false authentication, wrong subject, broken chain, unauthorized disclosure or a false verified state.

## 7. Claim ceiling

This audit is a source-bounded review of the named public files and revisions on 2026-11-12. It is not an exhaustive code review, penetration test, cryptographic analysis, Holochain integration test, standards conformance run, product safety assessment or deployed-system verification. The absence of a feature in the inspected archived Rust crate does not prove that no other Mycelix repository contains related research or components; it does mean the archived standalone README cannot be used as evidence that this particular implementation supplies those capabilities.

## Sources / code navigation

- Active SDK supply-chain integration: `mycelix-workspace/sdk-ts/src/integrations/supplychain/index.ts`
- Active workspace maturity inventory: `mycelix-workspace/scripts/workspace-inventory.txt`
- Active monorepo maturity statement: `README.md`
- Archived standalone README: https://github.com/Luminous-Dynamics/mycelix-supplychain (archived/read-only)
- Archived standalone verifier placeholder: https://github.com/Luminous-Dynamics/mycelix-supplychain/blob/main/rust/service/src/vc.rs
- NIST interoperability profile work: [#6098](https://github.com/Luminous-Dynamics/mycelix/issues/6098)
