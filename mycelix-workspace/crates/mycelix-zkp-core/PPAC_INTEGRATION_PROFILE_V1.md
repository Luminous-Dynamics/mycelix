# PPAC Integration Profile V1 — Research Draft

**Status:** Architecture / integration research only. Not a cryptographic protocol specification, implementation, qualification result, or production-admission rule.

**Owner:** Mycelix integration profile; semantic and institutional authority remain with existing domain owners.

**Tracking:** [PPAC-INT-001 / Mycelix #6230](https://github.com/Luminous-Dynamics/mycelix/issues/6230)

**Date:** 2026-11-15

## 1. Purpose

Privacy-Preserving Assurance Claims (PPAC) compose a private predicate proof with the provenance, authority, policy, consent, session and effect evidence needed for one institutional action.

The target property is:

> A verifier learns only the claims necessary for a specific decision, while an independent reviewer can reconstruct why the claim was accepted, who authorized the resulting action, which dependencies were current, and what effect was actually observed.

The profile composes existing Mycelix, Symthaea, Xenia and LISE owners. It does not introduce another ZKP backend, identity system, evidence ledger, consent authority, generic currentness service, or effect executor.

## 2. Non-equivalence rules

The following are separate propositions:

| Proposition | Does not establish |
|---|---|
| Envelope is structurally valid | Signature validity or proof validity |
| Envelope signature is valid | Original issuer truth or issuer authority |
| ZK proof is valid | Source authenticity, issuer legitimacy, or institutional authority |
| Issuer signature is valid | Issuer is authorized for this claim and jurisdiction |
| Credential is current | Requested policy is applicable |
| Operator is authenticated | Human consent to this exact operation |
| Consent is valid | Unlimited authority for the remainder of a session |
| Session capability is valid | Permission for an arbitrary effect |
| Action capability is consumed | Effect successfully occurred |
| Provider acknowledges an action | Independent observation of the external effect |
| LISE qualification is current | Future changes cannot invalidate it |

No boolean or scalar score may collapse these distinctions. Unsupported, stale, revoked, expired, contradicted and indeterminate states remain explicit.

## 3. Ownership map

| System | Owns | Must not own |
|---|---|---|
| Mycelix | Claims, issuer/credential references, policy and governance semantics, authority decisions, provenance, request identity, effect coordination and domain-owned receipts | A universal cryptographic backend or cross-domain authority boolean |
| Symthaea | Candidate proof computation, analysis, privacy-computation planning, proof cost/utility reasoning and exact computation identity | Issuer legitimacy, governance decisions, operator consent or production admission |
| Xenia | Human/operator/host authentication, consent and session integrity; the Symthaea receipt-attestation bridge (#359, #364, #369) and durable single-use issuance/replay owner (#435) | Institutional policy interpretation or Mycelix domain authority |
| LISE | Claim applicability, dependency impact, temporal/evidence frontier qualification, targeted invalidation and requalification | A duplicate identity or proof system, global currentness boolean, generic event store or domain authority |
| Symtropy | Physical/computational model evidence and qualified outcome observations when the effect requires them | Institutional authorization or treating simulation as an independently observed real-world effect |

The existing Xenia-Symthaea receipt path is especially relevant. Xenia #359 verifies the exact hybrid attestation over canonical Symthaea receipt bytes; #364 binds portable authority to the live Xenia trust boundary; #369 freezes the portable daemon receipt, including receipt UUID/digest, operator/key lineage, typed scope, enrollment/policy version, host fingerprint, nonce and expiry; #435 owns durable single-use issuance/replay semantics. Reuse this owner chain when a Symthaea proof-verification receipt crosses into a relying system. It establishes who attested to the exact Symthaea receipt under a bounded Xenia scope; it does not establish that the underlying claim is true, that Mycelix should admit the evidence, or that a consequential action is authorized.

The systems exchange versioned references and exact evidence digests, not mutable copies of each other's semantic authority.

## 4. Canonical request-to-effect binding

A complete PPAC decision must be reconstructable from the following binding set. This is a semantic checklist, not a new wire schema; reuse existing types and add only versioned extensions where an owner confirms a real gap.

| Binding | Required meaning |
|---|---|
| Request identity | Unique request and exact canonical request digest |
| Audience and purpose | Intended verifier/recipient and declared purpose |
| Subject binding | Pairwise or purpose-scoped subject reference; avoid a global stable identifier |
| Issuer evidence | Exact issuer identity, credential/source artifact digest, claim schema and source provenance |
| Predicate | Exact mathematical statement, public inputs, units and boundary semantics |
| Commitment | Exact commitment scheme/profile, randomness model and opening relation |
| Proof identity | Proof-system ID/version, circuit/AIR/guest image digest and configuration/security profile |
| Verifier identity | Verifier implementation/build digest and accepted verification policy |
| Challenge | Verifier-issued, audience-bound, request-bound, expiring and single-use challenge |
| Policy | Exact policy/profile digest and applicable authority generation |
| Dependencies | Issuer keys, trust roots, credential status, source lineage, verifier, program, policy and evidence-frontier refs |
| Session | Xenia session fingerprint, operator/host identity and authorization generation |
| Consent | Exact consent record, recipient, purpose, scope and revocation state |
| Action | Exact target, route/lane, operation and effect digest |
| Capability | One-time authority token or owner-native capability, exact scope and expiry |
| Effect | Attempt identity, terminal reconciliation and independent observation evidence |

The existing Mycelix AuthenticatedProof v2 signs its current envelope fields, including backend identifier, domain tag, client identifier, timestamp, nonce, public-input hash, proof digest and integer energy claim. It does not, by itself, establish all fields in this table. In particular, do not assume v2 already binds an audience, issuer-signed source statement, program/image identity, action digest, Xenia session or LISE dependency frontier.

The existing SupplyProofStatement binds a subject ID, policy hash, evidence commitment, public-values hash, issuer ID, validity interval and nonce. It is a domain-separated statement commitment, not a cryptographic proof or an issuer-verification result. Its stable subject ID must not be reused across presentations where doing so would create unwanted cross-verifier correlation.

Any extension must be versioned and canonical. Do not silently change the v2 signed transcript or reinterpret an existing hash field.

## 5. Verification is a vector, not a boolean

An application may only authorize when every required dimension for that specific use case is explicitly satisfied by the relevant owner.

| Dimension | Example states | Evidence owner |
|---|---|---|
| Canonical structure | valid / invalid / unsupported | Mycelix envelope validator |
| Envelope authentication | valid / invalid / unsupported | Authenticated envelope verifier |
| Issuer authentication | valid / invalid / unsupported | Credential/source verifier |
| Issuer authority | qualified / unqualified / indeterminate | Domain trust and authority policy |
| Predicate proof | valid / invalid / unsupported / verifier-error | Circuit/guest verifier |
| Program identity | exact / mismatch / unknown | Backend and trusted program registry |
| Credential status | current / revoked / expired / unknown | Credential status owner |
| Policy applicability | applicable / stale / superseded / indeterminate | Domain policy owner + LISE |
| Request binding | matched / mismatch / replay / expired | Verifier challenge owner |
| Session authentication | valid / invalid / revoked / expired | Xenia |
| Consent | approved / denied / revoked / expired / scope-mismatch | Xenia consent owner |
| Institutional authority | authorized / denied / indeterminate | Mycelix domain authority |
| One-time action capability | valid / spent / revoked / expired / mismatch | Effect owner |
| Effect outcome | applied / not-applied / partial / unknown / not-attempted | Effect owner and reconciler |
| Observation | independently-observed / provider-reported / incomplete / contradicted / absent | Independent observer/evidence owner |

A caller-constructible DTO, deserialized receipt, public trait implementation, detached report, or caller-supplied verified flag cannot mint any trusted state. Map these dimensions onto existing status vocabularies where available; do not introduce competing universal enums without an owner review.

## 6. Protocol sequence

### 6.1 Request and challenge

The Mycelix domain owner forms a canonical request binding purpose, audience, target, predicate, policy, action intent and expected program identity.

The verifier issues a challenge bound to that request digest and audience. Challenge issuance, expiry and consumption must have authoritative single-use semantics under concurrency, restart and network partition. A caller-chosen nonce is not proof of verifier issuance.

### 6.2 Issuer-backed private witness

The holder proves a predicate over a witness tied to issuer-authenticated source data. The implementation must choose and document one of these profiles:

1. The issuer credential and signature are selectively disclosed, with the verifier directly validating the disclosed signed claims; or
2. The proof circuit/guest verifies the issuer signature over the committed claim and proves the predicate over those same committed values.

A proof over a value supplied by the claimant proves only the relation over that value. It does not prove where the value originated. Issuer trust, key authority, jurisdiction and credential status remain separate LISE/domain qualifications even when signature verification is inside the proof.

### 6.3 Proof production and independent verification

Symthaea may select or execute a qualified privacy-computation plan, but it must be able to abstain with NoQualifiedPlan when privacy requirements cannot be satisfied by an admitted backend/profile.

The prover binds the exact public statement, request/challenge, audience, policy digest and program identity. The verifier checks the proof against its independently selected expected program identity. The verifier must not trust a program/image identifier supplied solely by the prover.

A proof receipt records the exact verifier, build, proof system, statement, public inputs, program identity, outcome and applicable dependencies. It does not authorize an institutional action.

### 6.4 LISE qualification

LISE evaluates claim applicability and currentness using owner-native temporal, source, authority and dependency semantics.

Relevant dependencies include:

- issuer identity, signing key, trust root and issuer authorization;
- original credential/source artifact and source lineage;
- credential status/revocation source and evidence frontier;
- predicate definition, circuit/AIR/guest image and toolchain;
- proof backend, cryptographic security profile and verifier build;
- policy, jurisdiction and domain authority generation;
- request/challenge state and verifier identity;
- Xenia operator, host, session, consent and revocation generation;
- effect route, target, capability-consumption owner and observer lineage.

Preserve historical validity at its original cutoff. Invalidate only the claims affected under the owning applicability rules. An incomplete change feed or unknown dependency must lead to indeterminate qualification, not optimistic currentness.

### 6.5 Xenia session and consent

Xenia authenticates the operator and host and binds the consent ceremony to the exact request. Reuse the Xenia session-level capability defined by #449 for permitted session lanes.

A session capability is not a blanket approval for all operations in that session. The exact effect requires a narrower one-time capability, owned and consumed by the system that owns the effect. It should bind the request and action digests, target, recipient, route/lane, permission, session fingerprint, consent record, proof and qualification receipt digests, authority generation, expiry and unique invocation identity.

The owner rechecks current authority and revocation at effect commit. Consent revocation, key rotation, policy changes, capability expiry, reconnect and rekey races must not permit a stale operation to reach the effect path.

### 6.6 Execution, observation and reconciliation

The effect owner records the attempt and terminal disposition. Unknown outcomes require reconciliation before retry. A different target or changed action requires a new request and consent decision.

An execution acknowledgement, provider status, local log, physical measurement and independent observer are different evidence classes. The final result must state which proposition each artifact supports and whether the expected action set was fully observed.

## 7. Privacy and correlation model

Privacy is an explicit information-flow property, not a consequence of choosing a ZKP library.

Required controls:

- Pairwise or purpose-scoped subject handles; avoid global stable subject IDs and repeated credential hashes.
- Hiding commitments for low-entropy values; a deterministic hash of an age, salary band or small finite amount can be brute-forced.
- Explicit commitment randomness, domain separation and opening semantics.
- Scoped nullifiers only where duplicate-spend or single-use semantics require them; document the scope and correlation properties.
- Evaluate leakage through revealed claim positions, issuer-key choice, proof size, timing, status-list access, IP/network metadata, request cadence, error classes and colluding verifiers.
- Avoid recording raw health, financial or identity values in ordinary logs.
- Use privacy classes and encrypted evidence capsules where raw canonical evidence must be retained; redacted summaries must not be convertible into authority-bearing canonical inputs.
- Do not claim coercion resistance or protection from repeated-query inference without a separate threat model and evidence.

## 8. Backend and implementation status at profile creation

These observations are source-level inspection, not fresh build or test results.

| Component | Current inspected boundary | Consequence |
|---|---|---|
| Symthaea RISC Zero balance guest | Real guest/host proving path. It evaluates balance >= threshold and commits sufficiency, threshold and caller-supplied nonce. | Promising relation-proof candidate. It does not yet bind an issuer-signed balance source or verifier-issued challenge. Symthaea #5842 tracks formal guest-relation refinement. |
| Symthaea Binius HDC harness | Current vendored standard prover/verifier path is described by repository documentation as non-zero-knowledge. Upstream Binius64 PR #2526 proposes masking relation targets with fresh one-time-pad keys; its PR description reports a transcript leak-check and a 2,043-test workspace run, and its review history includes approval after earlier requested changes. | Treat the vendored snapshot as computational-integrity only, not private-witness assurance. The Symthaea tree has not been shown to contain the upstream patch; PR #2526 explicitly omits a verifier-side wrong-key adversarial test; a byte-level transcript check is not by itself a zero-knowledge proof for every witness/query channel. Require merged source identity, independent privacy review and matched adversarial tests before integration. PR #7403's Iron Spartan smoke test is a statement-binding smoke test, not witness-privacy evidence. |
| Mycelix RISC Zero adapter | Capability matrix describes it as structural-only, with no linked verifier. | Do not call it an operational RISC Zero backend. |
| Mycelix HealthRangeAir | The inspected AIR discards public inputs and constrains bitness without constraining the intended range decomposition or binding the value commitment. | Already quarantined by #227; do not use for eligibility or authorization pending soundness repair and malicious-witness requalification. |
| Mycelix Winterfell AIR inventory | #1873 records the same verifier-theorem failure pattern in jurisdiction, review-integrity, recursive-aggregation and XOR proof code, in addition to #227's range circuit. | Reuse the umbrella census and per-circuit owners; do not assume the range defect is the only affected circuit. |
| Mycelix STARK security / privacy / verifier boundary | #1899 separates AIR soundness from witness privacy; #1905 owns measured security-profile qualification; #1919 covers malformed trace dimensions reaching verifier panic paths. | A repaired AIR alone is not a privacy proof, security-level claim or robust untrusted-proof verifier. Keep each qualification plane separate. |
| Mycelix selective disclosure | #5751 records a path that treats non-empty bytes plus a 32-byte commitment as proof success. | Unsupported predicates must fail closed until real proof verification is present. |
| Mycelix cross-DID | #5724 records caller-supplied proof bytes and nonce data without the required verifier-issued challenge and cryptographic proof verification. | Do not treat current submissions or link-resistance fixtures as verified ZK presentations. |
| Symthaea GIS proof examples | Pedagogical/simulated field arithmetic and simplified proof structures. | Not a production cryptographic proof backend. |
| Symthaea CycleIntegrityProof | Hash-chain commitment code with empty proof_bytes in the generator. | Commitments can establish artifact consistency, not that the cognitive computation was ZK-proven. |

## 9. Benchmark design

Use the matched external-comparison and workload protocol in [PPAC_BENCHMARK_PROTOCOL_V1.md](PPAC_BENCHMARK_PROTOCOL_V1.md). It separates credential disclosure, private eligibility, general zkVM execution, HDC and multi-party computation; it records dated industry references and forbids cross-workload headline comparisons. It adds time-to-trustworthy-decision and requalification amplification as full-system metrics.

Compose with LISE #6219 and Mycelix #6140 rather than building another generic evidence invalidation or effect-observation framework.

The PPAC-specific panel should contain positive, negative and ambiguous controls for:

- issuer signature valid/invalid, issuer authorized/unauthorized, credential expired/revoked;
- correct/incorrect subject and claim binding;
- correct/altered public inputs, threshold, commitment and program/image digest;
- malformed, arbitrary, empty and unsupported proof inputs;
- challenge from wrong verifier/audience, expired challenge, replay, concurrent duplicate and restart/partition;
- issuer-key rotation, trust-root rollback, stale status source and incomplete dependency/change feed;
- pairwise identity, stable-hash canaries, cross-verifier presentation correlation and metadata leakage;
- valid session with denied/revoked consent; changed action after consent; wrong resource, lane or recipient;
- revocation, expiry or policy-generation changes between decision and effect commit;
- successful, failed, partial, unknown and independently unobserved effects;
- historical proof remains valid for its old cutoff while present applicability is stale or indeterminate.

Hard gates, all prospective targets:

- Zero false acceptance of invalid proofs, wrong issuer/subject, altered public inputs/program identity, replay, stale authority, changed action or scope escalation in the frozen adversarial panel.
- Unsupported and indeterminate source/proof states never produce a positive satisfaction or authorization result.
- 100% agreement with frozen independent reference outcomes for the finite panel.
- No historical evidence deletion or hindsight rewrite.
- No unrelated claim invalidation on minimal-pair dependency mutations.
- Zero unauthorized disclosure of seeded protected attributes in the defined log/artifact projection suite.
- Independent second-verifier agreement before cross-institution deployment claims.

Report, separately: false accepts/rejects, proof and issuer verification latency, p50/p95/p99 proving and verification, proof and receipt size, CPU/GPU, peak memory, energy, cold-start, native vs WASM/Holochain runtime, challenge/revocation propagation, targeted requalification latency, disclosure count and cross-verifier linkability results.

Performance comparison requires identical mathematical statements, witness distribution, public inputs, security assumptions, proof mode, verifier and hardware. Do not compare Binius integrity-only timings with zero-knowledge proofs. Establish a measured baseline before fixing numerical targets. Thereafter optimize the full Pareto frontier and seek at least a twofold improvement in one resource dimension without weakening the same statement, security profile or privacy requirements. That is a research ambition, not a present result.

## 10. Industry and standards positioning

- W3C Verifiable Credentials Data Model 2.0 and Verifiable Credential Data Integrity 1.0 provide the standards foundation for credential and proof representation; conformance requires profile-specific implementation evidence.
- W3C Data Integrity BBS Cryptosuites v1.0 is a Candidate Recommendation Draft at the date of this profile, not a completed Recommendation. It is a selective-disclosure candidate, not a blanket anonymity guarantee. Its metadata and message-structure leakage must be tested.
- RISC Zero receipt verification binds execution to an expected guest ImageID. Mycelix must bind that program identity to its own trusted program and policy registry, not accept it from the prover.
- NIST treats ZKP, MPC, PSI, FHE, PIR and differential privacy as distinct, complementary privacy-enhancing techniques. Symthaea's planner should select among them based on the required information-flow property and qualified evidence.
- External proof-system performance numbers (including Ethereum block-proving benchmarks) are workload-specific context, not evidence that a backend will be faster for this PPAC predicate. All PPAC comparisons require a matched statement and environment.

## 11. Existing work map and next steps

| Existing owner | Follow-up needed for PPAC |
|---|---|
| Mycelix #5751 | Real selective-disclosure verifier and no-positive-result fail-closed gate |
| Mycelix #227 / #1873 | Specific HealthRangeAir quarantine and umbrella Winterfell AIR authority census |
| Mycelix #1899 / #1905 / #1919 | Separate witness privacy, measured STARK security and malformed-proof verifier safety |
| Mycelix #5724 | Verifier-issued challenge record, proof verification, audience binding and distributed replay semantics |
| Mycelix #2000 | Reuse authenticated qualification receipt; bind exact PPAC proof/statement and dependencies |
| Mycelix #2109 / #2110 | Keep privacy primitive/objective and qualification semantics canonical |
| Mycelix #6219 | Add PPAC dependency/change mutations to the owner-specific invalidation evaluator |
| Mycelix #4753 | Consume exact decision-to-effect freshness and commit-time authority checks |
| Mycelix #6140 / #6069 | Independent observation evidence and historical reconstruction |
| Symthaea #5842 / #12269 | Refine the exact balance guest relation, then bind the private value to issuer evidence and a verifier-issued request without overstating issuer authority or privacy |
| Symthaea #4860 | Plan only among PEC profiles with admitted evidence; support NoQualifiedPlan |
| Symthaea PR #7403 / issue #12274 | Review HDC semantic corrections, freeze the exact Binius/Iron Spartan vendor source and qualify the witness-hiding boundary before any ZK privacy claim |
| Xenia #359 / #364 / #369 / #435 / #449 / #450 | Reuse receipt attestation, live trust binding, portable authority/replay, lane capabilities, consent and independently verifiable session evidence |

### Runner-independent tranche

1. Freeze the owner map and this semantic contract.
2. Build a machine-readable matrix from each claim to its issuer, source, credential, circuit/program, verifier, policy, session, capability and effect-observation dependencies.
3. Use the synthetic PPAC casebook at PPAC_CASEBOOK_V1.json (24 cases) as the initial mutation specification; its expected dispositions remain provisional, unadjudicated and not run until an independent oracle review.
4. Map the existing Symthaea RISC Zero balance guest and Mycelix envelope/statement types without changing their current semantics.
5. Record exact blockers and establish a matched benchmark environment before numerical comparisons.
6. Keep all implementation and qualification claims at their current evidence tier until exact-head and independent checks exist.

## 12. Claim ceiling

This profile is an integration design and benchmark plan. It does not establish an operational PPAC verifier, source authenticity, issuer legitimacy, proof soundness or privacy for any backend, unlinkability, external security audit, legal compliance, Holochain runtime readiness, application authority, or observed effect. Each requires the separate implementation and owner-qualified evidence listed above.
