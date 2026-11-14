# DeSci Challenge-Set Escrow and Access Attestation RFC v0.1

**Status:** Draft architecture proposal  
**Issue:** [#6031](https://github.com/Luminous-Dynamics/mycelix/issues/6031)  
**CIVDYN integration:** [repair audit #4453](https://github.com/Luminous-Dynamics/symtropy/issues/4453), [draft implementation #4455](https://github.com/Luminous-Dynamics/symtropy/pull/4455)  
**Scope:** Authorization and auditability of access to sealed scientific challenge artifacts. This RFC does not claim cryptographic proof of non-disclosure.

## 1. Problem and security objective

Scientific repair and model-selection workflows can overfit challenge data if a developer, repair agent or model-selection process obtains challenge observations before the final evaluation. CIVDYN currently checks declared statistic IDs, observation digests, model identities and split identities. Those checks detect inconsistent receipts, but caller-declared digests cannot prove that challenge data were not obtained through another route.

Mycelix should provide a narrow, independently auditable access-control boundary:

> For a declared challenge artifact and evaluation purpose, identify who was authorized, which capability was granted, what release path was used, which evaluation/model/repair attempt it was bound to, and which events were recorded.

The system should make unauthorized registered-path access fail closed, and make authorized-path use tamper-evident and attributable. It cannot prove that an authorized recipient did not make an unlogged copy, photograph, transcription or out-of-band transfer. That residual risk must remain explicit.

## 2. Design principles

1. **Do not put raw challenge data on a public DHT.** Store encrypted content in a rights-appropriate artifact store or controlled data enclave. DHT records carry commitments, access metadata and attestations only.
2. **Purpose-bound capability, not broad membership.** An evaluator's general project membership is not permission to read every challenge artifact.
3. **Fail closed.** Unknown, expired, revoked, exhausted, ambiguous or mismatched authorization denies release.
4. **Append-only history.** Revocation, correction, challenge rotation and dispute create new events. Never overwrite or erase the history of prior access.
5. **Separate duties.** Custodian, model developer/repair agent, evaluator and independent reviewer are distinct roles. A principal may hold more than one role only under an explicit, auditable policy.
6. **Least disclosure.** Public verifiers should be able to check authorization and lineage without receiving the underlying observations.
7. **Explicit trust assumptions.** A valid signature proves a key signed a statement; it does not prove the statement's real-world truth or prove absence of other access paths.
8. **Reuse before adding.** Integrate existing DeSci provenance, scientific event, credential governance, dispute and identity primitives; do not introduce a parallel identity system or consensus ledger.

## 3. Actors and responsibilities

| Actor | Responsibility | Must not be able to do alone |
|---|---|---|
| Data rights holder / custodian | Defines access policy and artifact version; controls release | Certify its own model repair |
| Challenge steward | Seals statistic vocabulary, split and challenge artifact | Modify a frozen artifact silently |
| Model developer / repair agent | Proposes model and repair plan | Read sealed challenge data before authorized evaluation |
| Evaluation service | Executes the declared evaluation under a grant | Change the model or challenge after execution begins |
| Independent reviewer | Reviews protocol, access events and output receipt | Alter the original access history |
| Public/verifying party | Verifies signatures, policy lineage and output binding | Obtain private observations from the public record |

## 4. Versioned record types

The names below are conceptual v0.1 types; final Rust/HDI schemas must follow the repository's established serialization, hash, signature and entry-validation conventions.

### 4.1 ChallengeArtifactCommitment

Fields:
- artifact ID and immutable version;
- challenge-manifest digest and canonicalization/version identifier;
- encrypted artifact locator or controlled-enclave identifier (not a public raw-data URL);
- ciphertext digest and encryption/key-reference metadata;
- statistic-vocabulary commitment and source-family metadata;
- rights/consent/license policy identifier and permitted-purpose set;
- steward/custodian DID and signature;
- created-at evidence, expiry/retirement state and predecessor artifact reference.

The commitment must bind both the challenge manifest and the exact encrypted artifact. Rotating data, units, statistics or protocol creates a new artifact version and commitment.

### 4.2 AccessGrant

Fields:
- grant ID and unpredictable nonce;
- subject DID and authenticated credential reference;
- artifact ID/version and challenge-manifest digest;
- project, model digest, repair-attempt/evaluation-run ID;
- permitted operation (evaluate, verify, adjudicate, or a future versioned operation);
- explicit statistic/partition scope;
- purpose and protocol digest;
- maximum uses, expiry and output-handling policy;
- issuer DID, policy version and signature;
- superseded/revoked grant reference, if applicable.

The grant must bind the intended model and repair attempt. A grant for model A cannot be replayed for model B or a later repair attempt.

### 4.3 AccessEvent

Append-only event types:
- GrantIssued
- GrantDenied
- GrantRevoked
- GrantExpired
- ReleaseAuthorized
- EvaluationStarted
- EvaluationCompleted
- EvaluationAborted
- OutputReceiptCommitted
- ArtifactRotated
- AccessDisputed

Each event binds:
- event ID and event type/version;
- actor DID and credential reference;
- artifact/version and manifest digest;
- grant ID and nonce;
- project/model/repair/evaluation identity;
- purpose, operation and statistic scope;
- previous event/action hash and local source-chain position;
- signed time evidence and clock source/uncertainty where available;
- event payload digest and issuer signature.

Holochain source-chain order is authoritative only for the committing agent's chain. Do not invent a globally total order across agents. Cross-agent events should link causally through action hashes, grant IDs, release IDs and signed predecessor references. Conflicting histories must be preserved and surfaced for reconciliation.

### 4.4 EvaluationReceipt

Fields:
- evaluation run ID and protocol digest;
- exact artifact/version and challenge-manifest digest;
- exact grant and release-event IDs;
- baseline/repaired model digests and repair-plan digest;
- evaluator identity and execution-environment attestation (when available);
- input manifest and output receipt digests;
- statistic-level result references and failure/abortion state;
- evaluation service signature and optional independent reviewer signature.

A receipt records what the registered evaluator claims to have executed. It does not independently establish that the code is correct or that no off-path data access occurred.

### 4.5 Revocation and rotation

Revocation prevents future registered-path releases. It cannot reverse an already completed release or claw back an existing copy. Challenge rotation creates a new artifact version, key/capability lineage and manifest digest; old versions remain auditable and are not silently re-pointed to new bytes.

## 5. Authorization and release protocol

1. The challenge steward freezes the manifest and artifact, encrypts the data and commits the artifact commitment.
2. The custodian issues a purpose-bound grant for one subject, model, repair attempt, artifact version and operation.
3. The evaluator authenticates the subject credential and presents the grant nonce.
4. The authorization coordinator checks signature, issuer authority, expiry, revocation, artifact/manifest identity, model/attempt identity, operation, purpose, scope and use budget.
5. A consume-once/replay-protection mechanism atomically records the grant use before the release capability is returned. If the coordinator cannot establish whether consumption committed, the outcome is unknown and no second release is issued until reconciliation.
6. The controlled release path provides access to the encrypted artifact or enclave session. The release event is committed with the grant and artifact identities.
7. The evaluator executes the frozen protocol and commits an output receipt. Aborted/failed runs remain visible.
8. An independent verifier checks the full chain from artifact commitment through grant, release, run and output receipt.
9. Any dispute, revocation or rotation is appended as a new event.

A one-time grant must not be implemented as a read-then-write check that permits concurrent double consumption. The implementation must reuse an established Mycelix consume-once/challenge primitive if its semantics match this operation; otherwise the concurrency and uncertain-outcome protocol requires a separate design review.

## 6. Privacy and data minimization

- Never publish raw observations, private source records, decryption keys, signing seeds, access tokens or personal information in DHT entries, logs, CI output or GitHub issues.
- Prefer encrypted artifacts with narrow, short-lived release capabilities.
- Separate identity/authorization metadata from private observation content.
- Minimize statistic labels and source metadata where these could reveal sensitive cohort membership.
- Define retention and deletion at the encrypted artifact store. DHT attestations may be immutable; personal data should not be placed in them merely because the ledger is append-only.
- Support selective disclosure of the authorization chain where the reviewer does not need the underlying data.
- Record the jurisdiction, consent basis, license and permitted purpose associated with the artifact. Cryptographic authorization does not replace data-rights review.

## 7. Threat model

| Threat | Required control | Residual limitation |
|---|---|---|
| Forged grant or event | Signature and issuer-authority verification | Compromised signing key |
| Expired/revoked grant | Versioned status and revocation check at release time | Already released data remain copied |
| Model/attempt substitution | Bind grant and receipt to exact model/repair IDs | A recipient can lie about an off-path run |
| Manifest/artifact substitution | Bind canonical manifest and ciphertext digest | Compromised custodian before sealing |
| Scope escalation | Exact operation/statistic/partition scope; unknown scope denied | Scope semantics can be poorly designed |
| Replay/double consumption | Atomic consume-once nonce and replay ledger | Coordinator compromise / recovery ambiguity |
| Event deletion/reordering | Append-only signed links, action hashes and source-chain validation | Cross-agent clocks are not a global ordering oracle |
| Unauthorized bulk export | Enclave/capability limits, output policy, monitoring | Screenshots, manual transcription, covert exfiltration |
| Evaluator collusion | Role separation, independent review and challenge rotation | Colluding authorized parties |
| Custodian compromise | Multiple custodians or independent challenge steward for high-value sets | Governance threshold itself may be compromised |

## 8. Verification contract consumed by CIVDYN

CIVDYN should accept a challenge-access attestation only after a verifier checks:
- issuer/actor credentials against a versioned trust policy;
- all signatures and credential/revocation states;
- exact artifact version, manifest and ciphertext commitments;
- model, repair-attempt, protocol and evaluator identities;
- grant scope, expiry, use count and consume-once result;
- causal links between grant, release, execution and output receipt;
- absence of any declared diagnostic/challenge statistic or observation overlap.

The resulting verified attestation should be a structured receipt, not a Boolean supplied by the caller. CIVDYN should reject a missing, revoked, wrong-purpose, wrong-model, wrong-manifest, replayed or incomplete access receipt.

**Claim boundary:** a valid attestation establishes that the registered access path authorized and logged a particular release. It does not prove non-exposure outside that path. High-consequence scientific claims still require independent review, challenge rotation and governance controls.

## 9. Implementation sequence

1. Map existing Mycelix DeSci claims/provenance, scientific events, credential governance, dispute, storage and identity primitives; record exact modules to reuse.
2. Define versioned types and canonical encodings for commitments, grants, events and receipts.
3. Implement validation for signature, issuer authority, subject, scope, expiry, revocation and manifest/model binding.
4. Implement atomic consume-once release authorization, explicit unknown-outcome reconciliation and replay tests.
5. Implement append-only access-event lineage and independent receipt verification.
6. Add privacy/data-rights review and threat-model tests.
7. Build a CIVDYN fixture that rejects missing, substituted, revoked, replayed and wrong-purpose access receipts.
8. Conduct independent security review before any claim that escrow protects a real challenge set.

## 10. Conformance tests

At minimum:
- invalid signature and unauthorized issuer;
- wrong subject, model, repair attempt, purpose, protocol or statistic scope;
- expired/revoked/rotated artifact and grant;
- concurrent duplicate consume-once attempts;
- duplicate nonce and replayed release;
- out-of-order, missing, duplicated or conflicting event links;
- output receipt referencing a different release event;
- unknown policy/schema version;
- unauthorized export and missing data-rights basis;
- key-compromise recovery and revocation;
- successful authorized run with exact CIVDYN challenge receipt binding.

All failures must preserve a receipt of denial where safe, avoid releasing content, and distinguish DENIED, UNKNOWN_OUTCOME and AUTHORIZED. Unknown outcome must not be treated as denial followed by automatic retry, because the first release may have succeeded.

## 11. Non-goals

- This RFC does not create a global ledger or new consensus protocol.
- It does not store private challenge observations on a public DHT.
- It does not claim cryptographic proof that no one copied or observed data.
- It does not certify the scientific correctness of a model or repair.
- It does not replace legal, ethical, consent or independent-review processes.
