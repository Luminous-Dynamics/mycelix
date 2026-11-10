# Mycelix Identity: Assurance Boundaries, Ecosystem Integration, and Benchmark Plan

**Status:** Research proposal / draft benchmark contract  
**Version:** 0.1  
**Date:** 2026-11-10  
**Target:** Public Mycelix repository; active Holochain identity DNA and its relying parties  
**Base revision:** 4b9d7aa7f1a69fa19b1bbd19a903ff3f4e7c63a5

This document complements, and does not replace, the identity-specific source review in [IDENTITY_ASSURANCE_REVIEW_2026-11-10.md](../mycelix-workspace/mycelix-identity/docs/IDENTITY_ASSURANCE_REVIEW_2026-11-10.md) proposed by PR #6037. That review records concrete recovery defects and code-level mitigations. This document defines the wider identity model, cross-system trust boundaries, and a repeatable benchmark programme.

Nothing here claims that Mycelix is currently certified against NIST, W3C, FIDO, OpenID, or SPIFFE standards; that its deployed network has been penetration-tested; or that a DID proves the real-world identity of its controller.

## 1. Decision

Mycelix should become the identity and credential foundation for Luminous systems, but **identity, authentication, authority, trust, and evidence must remain separate typed concepts**.

The goal is not one universal login, one global reputation score, or one database row that represents a person, device, service, organization, and AI agent interchangeably.

The target model is a **federated identity-and-evidence graph**:

- a subject has one or more stable, scoped identifiers;
- controllers and key epochs establish who can act for that subject under a defined method;
- authenticators prove possession or presence for a particular ceremony;
- credentials carry issuer-attributed claims with schema, scope, validity, and status;
- capabilities and delegations authorize exact actions within exact boundaries;
- evidence receipts bind observations, decisions, and state transitions to their subjects and source;
- policy evaluates the evidence for a particular relying party and operation.

A successful identity check must never silently become an authorization grant. A valid signature proves that a key signed a message; it does not prove that the claim is true, that the key is uncompromised, or that the signer had authority for the action.

## 2. Ontology: what is being identified?

Model the following as distinct subject classes. They may be related, but they must not be silently collapsed:

| Subject class | Examples | Identity question |
|---|---|---|
| Human / natural person | user, operator, trustee | Which person or pseudonymous persona is represented, and what proofing was actually performed? |
| Organization / collective | company, cooperative, council, community | Which legal, delegated, or member-governed controller can make which assertions? |
| Device / machine | laptop, server, robot, vehicle, boot target | Which physical or logical machine is this, and how was its enrollment bound to it? |
| Workload / service | Nixward daemon, API service, executor | Which running workload is presenting the credential, in which trust domain and audience? |
| Software artifact | binary, Nix closure, firmware, container image | Which exact artifact digest is the subject of the provenance statement? |
| AI agent / model instance | Symthaea instance, model build, tool executor | Which model/build and runtime instance acted, and what authority was separately granted? |
| Non-human or plural entity | ecological system, habitat, collective stewarded resource | Which claims are recorded, by whom, and under what stewardship relationship without silently promoting a claim into legal personhood or authority? |

### 2.1 Stable identity is not a content digest or a key

Keep at least these identifiers distinct:

- **Subject ID:** stable opaque identity within a declared method and scope.
- **Key ID / key epoch:** a particular public key and its lifecycle interval.
- **Controller relationship:** the policy-defined relationship authorizing an entity or key to update or act for the subject.
- **Instance ID:** one installed or running representation of a device, service, or agent.
- **Artifact digest:** immutable identity of a particular binary, firmware image, build output, or configuration closure.
- **Credential ID:** one issuer-signed claim object or status subject.
- **Transaction ID:** one exact consequential operation and its receipts.

A normal key rotation changes the key epoch, not automatically the subject identity. A software update changes an artifact digest, not automatically the device identity. A fork or independently governed successor is an explicit lineage event, not an accidental consequence of changing a hash.

The currently documented form did:mycelix:<agent_pub_key> should be treated as an **agent-key-derived identifier**. By itself, it is not proof that the controller is a particular natural person, legal organization, physical device, or unique participant. Any stronger statement requires separately specified enrollment evidence and a verifier policy.

### 2.2 Relationships are first-class

Use explicit, scoped, time-bounded relationships such as:

- controls
- operates
- enrolled-on
- issued-by
- held-by
- delegated-to
- authorized-for
- attested-by
- member-of
- steward-of
- supersedes
- recovered-by
- revoked-by
- derived-from

Every relationship needs an issuer/author, evidence reference, validity interval or event boundary, scope, and a revocation/supersession rule. A relationship must not inherit broader authority merely because it is adjacent in the graph.

## 3. Non-negotiable assurance boundaries

| Concept | What it establishes | What it does not establish |
|---|---|---|
| Identifier | A name for a subject within a method/scope | Real-world identity, uniqueness, trust, or authority |
| Proof of control | A controller can currently use a key or authenticator | That only one party has the secret, or that the key is uncompromised |
| Identity proofing | Evidence binds a subject to a claimed real-world identity at a declared assurance level | Ongoing authorization for every action |
| Authentication | A claimant completed a defined protocol ceremony | Permission to mutate a system or make a financial decision |
| Credential | An issuer made a scoped, verifiable claim | Truth beyond the issuer's claim and evidence |
| Authorization | A policy permits an exact subject/action/resource/context | That the action will succeed or be safe |
| Trust assessment | A relying party's policy interpretation of evidence | An intrinsic, universal property or single scalar score |
| Provenance | A source and history are attributable and tamper-evident | That the original statement is factually correct |
| Observation | A named source reported a state or event | Independent verification unless the source and method justify it |

**Required invariant:** identity verification, credential verification, authorization, execution, and independent outcome verification are separate decisions with separately inspectable evidence.

Trust should be represented as a set of typed, scoped claims and observations with provenance, freshness, uncertainty, and policy—not as one universal reputation number. A relying party may derive a decision from that evidence, but the derived decision must identify its policy version and inputs.

## 4. Privacy and plural identity

The system should support multiple identities and personas for the same controller without requiring a public global correlation key.

Required design properties:

1. Pairwise or purpose-scoped identifiers where protocol semantics allow them.
2. Selective disclosure of only the attributes required by a verifier.
3. No raw phone number, email, biometric template, recovery secret, or private key in DHT entries, logs, or public receipts.
4. No stable cross-domain identifier unless the subject has explicitly chosen that linkage or a narrowly defined legal requirement applies.
5. Explicit separation between claims about an entity and claims about its legal status, personhood, membership, or stewardship.
6. Unknown, unavailable, or stale evidence remains unresolved; it does not silently become valid.
7. A privacy review must evaluate metadata linkability and correlation, not only whether a secret field is encrypted.

A biometric may be used locally by an authenticator to unlock a credential, but a biometric hash supplied by a caller is not proof of possession and should not be treated as a remotely verifiable factor.

## 5. Lifecycle state machines

### 5.1 Subject and controller lifecycle

Define and test explicit transitions for enrollment, proofing, active use, key rotation, controller delegation, compromise, suspension, recovery, deactivation, and archival.

The subject's lifecycle and each key's lifecycle are independent. The system must retain enough history to explain which key was valid for which operation at the time it occurred.

### 5.2 Authenticator lifecycle

A newly added factor is **pending** until a real verification ceremony succeeds. Enrollment, verification, activation, suspension, replacement, and removal must be distinct transitions.

For WebAuthn assertions, verification must bind a fresh single-use challenge to the relying party, reject unexpected origins, check RP ID hash, credential ownership, authenticator flags, signature, and applicable backup/counter semantics. Do not turn a credential ID or a stored hash into a success signal.

### 5.3 Recovery lifecycle

Recovery is a high-risk authority transition, not an ordinary profile update. A recovery request must bind:

- target subject/DID;
- old controller/key epoch;
- proposed replacement controller/key epoch;
- request ID and single-use nonce;
- policy and threshold version;
- audience and expiry;
- each distinct factor/trustee proof;
- exact evidence digest;
- final claim/continuity receipt.

The flow must be retryable and idempotent. It must never report a DID claim as complete before the DID registry confirms the same request ID and intended replacement controller. Unknown cross-zome outcomes must remain pending until reconciled.

A threshold must count independent factors or trustees under an explicit independence policy, not multiple strings or identities that share one compromise domain. A local-chain lookup must not determine whether a remote trustee's valid DHT vote exists.

### 5.4 Credentials and revocation

Credential checks must cover issuer authorization, subject binding, schema and context version, issuance/expiry, suspension, terminal revocation, status authenticity, freshness, and unavailable status.

A verifier must distinguish:
- valid and current;
- valid but suspended;
- revoked;
- expired or not-yet-valid;
- malformed or unverifiable;
- status unavailable or too stale to decide.

Do not collapse the latter five states into a boolean success/failure without retaining the reason. Unknown status is not valid status.

## 6. Luminous integration contract

Mycelix should provide identity, key/controller lifecycle, credential verification, and portable evidence references. It should **not** become the privileged executor for every system.

| System | Identity/evidence role | Must remain separate |
|---|---|---|
| Mycelix | Stable subject IDs, controller/key epochs, credentials, recovery, revocation, trust-policy inputs and verifiable identity events | DHT identity does not by itself prove a natural person or authorize an operation |
| Nixward | Bind an exact ChangePlan to its target machine identity, owner/operator authority, policy version, freshness, and transaction evidence | A Mycelix identity or credential alone must not authorize a host mutation; exact-plan authorization remains separate |
| Sovereign Boot | Bind boot target, generation lineage, boot attempt, measured state, health result, and Last-Known-Good promotion evidence | Identity/recovery service unavailability must not prevent boot; boot safety stays local and fail-open |
| Luminous Platform | Link identity, software/build provenance, authorized transaction, observation, verification, recovery, and fleet evidence into one independently traversable graph | A digest, signature, or provenance receipt does not by itself prove an assertion is true |
| Symthaea | Identify model/build/runtime and attribute proposals, explanations, and tool requests to the generating instance | HDC similarity, confidence, active inference, Phi, or model identity is never an authorization primitive |
| Symtropy | Identify the target, executor, simulation/model revision, physical observation source, and resulting state receipt | A simulation result is not a physical observation; an executor cannot verify its own success alone |

### 6.1 Nixward binding

A Nixward system transaction should reference, at minimum:

- target machine subject and the exact target binding/digest;
- owner/operator principal and the authority credential or capability used;
- exact ChangePlan digest and nonce;
- policy version, expiry and replay-protection outcome;
- pre-state and recovery binding digests;
- exact applied command/derivation/closure identity;
- post-state observation and independent verification receipt;
- promotion or recovery result.

The subject ID names the target. The authorization proves permission for the exact transaction. The transaction evidence records what happened. None substitutes for the others.

### 6.2 Sovereign Boot boundary

Sovereign Boot should use locally verifiable, bounded trust material for boot-time decisions. Network identity services may contribute fresh evidence when available, but no remote Mycelix lookup may be a boot prerequisite. Cached material must have explicit version, expiry, revocation/freshness behavior, and failure semantics.

The recovery observer must not become a boot dependency or obtain authority to promote an unqualified generation merely because its own process is healthy.

### 6.3 Workload identity

For short-lived service-to-service identity, prefer an interoperable workload identity mechanism such as SPIFFE/SPIRE where it fits. Do not stretch a long-lived Mycelix DID into an ephemeral workload token. A Mycelix subject may bind the durable service owner/lineage; SPIFFE SVIDs can authenticate a live workload instance inside a trust domain. The binding between them must itself be attested and scoped.

## 7. Standards and industry comparison

No single industry benchmark measures all of identity, privacy, recovery, workload identity, authorization, and auditability. Compare Mycelix against the strongest relevant standards and conformance programmes by layer.

| Layer | External reference | What to adopt or test | Important boundary |
|---|---|---|---|
| Human proofing, authentication, federation | NIST SP 800-63-4 family (A-4, B-4, C-4) | Map policy profiles to IAL/AAL/FAL requirements where applicable; test enrollment, authenticator lifecycle, federation and assertions | NIST guidance is a risk-based profile, not one universal trust score or a claim of certification |
| Decentralized identifiers | W3C DID Core 1.0 | DID syntax, controller rules, verification relationships, proof-of-control and resolution method boundaries | DID Core is not a DID-method certification; Mycelix needs a method-specific conformance suite |
| Verifiable credentials | W3C VC Data Model 2.0 and VC Data Integrity 1.0 | Issuer/holder/verifier roles, claim integrity, schema/version handling, selective disclosure and cryptographic proof validation | Signature validity does not make the issuer's claim true |
| Passkeys / MFA | W3C WebAuthn Level 3 and FIDO certification programmes | Challenge, origin, RP ID, authenticator flags, attestation policy, credential lifecycle and interoperable negative tests | Local user verification does not establish a natural person's identity in every authenticator configuration |
| Credential protocols | OpenID4VCI and OpenID4VP High Assurance Interoperability Profile (HAIP) | Run the OpenID Foundation conformance suites and record the exact profile/version and result | Data-model support alone is not protocol interoperability |
| Workload identity | SPIFFE specifications and SPIRE implementations | Workload API, X.509-SVID/JWT-SVID validation, trust-domain scoping, rotation, audience and bundle updates | A workload ID is not a human identity or an application authorization policy |
| Signed statement transparency | IETF RFC 9943 (SCITT architecture) and applicable VDS profiles | Issuer attribution, signed statements, registration receipts, append-only history, non-equivocation and replayable audit | Transparency makes statements accountable; it does not prove their factual accuracy |
| System execution provenance | in-toto / SLSA, where applicable | Bind artifacts to build provenance and verification evidence | Build provenance is not owner authorization for installation or execution |

The current standards baseline has advanced beyond the identity review's older references: WebAuthn Level 3 became a W3C Recommendation on 2026-08-25; the OpenID Foundation announced HAIP conformance suites for OpenID4VP and OpenID4VCI on 2026-08-07; and IETF RFC 9943 was published in June 2026. The benchmark should pin exact versions rather than using the floating word “latest”.

## 8. Benchmark programme

Create a public, versioned corpus with:
- immutable fixture IDs and SHA-256 digests;
- explicit attacker capability and trust assumptions;
- valid positive controls as well as attack/negative controls;
- deterministic seeds and bounded generated state spaces;
- independent expected-outcome oracles;
- exact source revision, dependency lockfile and test runner version;
- machine-readable output containing test ID, input digest, result, reason, duration and evidence references;
- a declared coverage boundary and a separate list of untested cases.

Report safety, privacy, availability, interoperability, and performance independently. **Do not collapse them into a single weighted “identity score”.**

### 8.1 Required test families

| Suite | Example adversarial cases | Initial acceptance criterion |
|---|---|---|
| ID-BIND | Foreign DID, author/owner mismatch, controller mismatch, update/delete by non-controller, cross-zome identity substitution | Every frozen unauthorized mutation rejected at integrity boundary |
| KEY-LIFE | Wrong key, key substitution, stale key, unauthorized rotation, algorithm/key-type mismatch, downgrade, KID collision, replay | Every frozen invalid proof rejected; supported valid rotations remain verifiable with historical event attribution |
| MFA-ENROL | Pending factor promoted without proof, wrong challenge, wrong origin/RP ID, replayed assertion, invalid signature, unapproved attestation | No factor becomes active without the required proof and policy decision |
| RECOVERY | Forged/empty/stale/cross-DID proof, duplicate anchors, duplicated trustee vote, colluding subset below threshold, replacement-key mismatch, retry and timeout races | No below-policy threshold succeeds; all honest replicas converge on the same terminal outcome |
| MULTI-AGENT | Requester and trustees on different conductors, reordered/delayed/duplicated messages, partition, restart, DHT replication lag, rejoin | Deterministic, explainable convergence; unresolved outcomes stay unresolved rather than being guessed |
| DID-METHOD | Invalid DID syntax, resolver mismatch, duplicate method IDs, invalid verification relationships, unknown method version | Exact method-specific conformance vectors pass; malformed/unsupported vectors fail explicitly |
| VC-STATUS | Invalid issuer, wrong subject, expired/not-yet-valid, suspended/revoked, stale/forged/unavailable status, schema downgrade | No revoked or invalid credential accepted; unavailable status cannot become “valid” by default |
| PRIVACY | Cross-context correlation, identifier enumeration, raw PII/secret leakage in DHT/logs/receipts, excessive disclosure | Zero prohibited disclosure in the frozen corpus; quantify residual metadata linkability |
| TRANSPARENCY | Missing receipt, invalid receipt, inconsistent checkpoint, stale/forked log view, replayed registration, issuer equivocation | Receipt inclusion and consistency verified; detected inconsistency blocks trust in the affected view |
| MACHINE-BIND | Substitute machine, cloned machine image, stale machine credential, wrong generation, wrong artifact digest, cross-target plan replay | No authorization or promotion crosses the exact machine/plan/artifact boundary |
| OFFLINE | Mycelix/DHT/network unavailable, stale trust bundle, clock skew, revoked cached key, unknown revocation state | Boot remains possible; no privileged mutation or unqualified promotion is inferred from missing evidence |

### 8.2 Multi-conductor recovery as a release blocker

The most important near-term distributed test is not another single-agent unit test.

Construct a frozen SweetConductor scenario with at least:
- one requester;
- multiple trustees, each on a distinct agent/conductor;
- one proposed replacement agent;
- an adversarial duplicate-vote sender;
- delayed and reordered DHT/message delivery;
- at least one partition/restart/rejoin cycle.

Assert that:
1. votes are authenticated to the actual trustee action author;
2. one trustee cannot contribute twice under alternate encodings or duplicate delivery;
3. the threshold uses distinct trustee identities and the policy version fixed by the request;
4. each honest conductor derives the same vote set and decision from equivalent DHT evidence;
5. cancellation, rejection, expiry and approval races have deterministic resolution;
6. the DID claim is idempotent and bound to the exact request ID and replacement controller;
7. no endpoint reports completion when the claim operation failed or remains unknown.

This is a P0 integration gate. Until it passes, social recovery may be described as implemented only to the exact extent demonstrated by the currently passing tests.

### 8.3 Performance and scale

Measure the following separately for each operation and deployment topology:

- local signature verification and proof validation: p50/p95/p99 latency and CPU cost;
- DID resolution: local-cache, same-conductor, remote-conductor, cold-start and partitioned paths;
- credential verification: proof cost by format, credential size and disclosure profile;
- enrollment and key rotation: completion latency, failure rate and propagation delay;
- recovery: threshold decision latency, claim completion latency and retry/reconciliation cost;
- revocation: publication-to-observation propagation and stale-cache behavior;
- transparency: registration throughput, receipt generation/verification and audit cost;
- state growth: bytes per subject, credential, key epoch, vote and receipt; pruning/retention effects;
- resource cost: CPU, memory, storage, network bytes and energy on declared hardware.

Do not set a performance target before publishing a reproducible baseline. After the baseline, set service-class targets and improve one dimension at a time without weakening negative-control behavior. Report cold and warm paths separately. No result should be compared with a vendor or public benchmark unless protocol, cryptography, topology, hardware, workload and correctness envelope are matched.

### 8.4 Scorecard

Publish a multi-axis scorecard, not a leaderboard scalar:

1. **Safety:** unauthorized acceptances, rejected negative controls, mutation-boundary coverage.
2. **Correctness:** valid-case acceptance, state-machine completeness, deterministic replay.
3. **Distributed behavior:** convergence, fork/equivocation detection, partition and rejoin semantics.
4. **Privacy:** unwanted disclosure and cross-context linkability.
5. **Interoperability:** normative conformance and independently tested cross-implementation exchange.
6. **Availability:** valid-operation completion under declared faults, separate from safety.
7. **Performance:** latency, throughput and resource costs by workload.
8. **Evidence quality:** reproducibility, source attribution, independent oracle and receipt completeness.

A zero-failure run on a small frozen corpus means exactly that: zero failures on that corpus and revision. It is not a proof of universal security.

## 9. Priority order

### P0 — now

1. Complete and exact-head validate the existing identity security branches:
   - PR #6037: recovery configuration ownership and fail-closed proofless self-recovery;
   - PR #6043: cryptographic envelope algorithm/type safety;
   - PR #6044: pending MFA enrollment until verification.
2. Keep self-recovery disabled until versioned proof-of-possession receipts can be verified at the integrity boundary.
3. Add the root-workflow conductor/SweetConductor gate for the active nested identity workspace.
4. Add the multi-conductor recovery and claim-completion tests in §8.2.
5. Reconcile the legacy root identity path and the active nested Holochain identity workspace so one cannot silently drift or escape CI.

### P1 — identity fundamentals

6. Complete DID controller/verification-relationship and key-rotation negative controls.
7. Complete credential issuer/subject/status and revocation negative controls.
8. Separate local browser key custody from Holochain agent key custody; test backup/restore, encryption, lock/unlock, origin changes, and tampering.
9. Publish the Mycelix DID method specification and its independent resolution/conformance vectors.
10. Add privacy/linkability testing for pairwise identifiers and selective disclosure.

### P2 — ecosystem federation

11. Define a portable machine identity binding for Nixward and Sovereign Boot.
12. Evaluate SPIFFE/SPIRE for short-lived service/workload credentials rather than creating an incompatible replacement.
13. Bind transaction and boot receipts to identity key epochs, policy versions, artifact/generation digests, and exact target identifiers.
14. Add SCITT-compatible signed statement and receipt exchange for software/build evidence where an external transparency service is available.

## 10. Evidence and release discipline

For every published result, record:

- repository and exact commit SHA;
- branch/ref and whether the result is from main, a PR head, or a local worktree;
- locked toolchain and dependencies;
- command and environment;
- test corpus version and digest;
- positive and negative control counts;
- pass, fail, skipped, unavailable, and not-run counts separately;
- conductor count and topology for distributed tests;
- external verifier or review identity, when applicable.

A queued, cancelled, skipped, or unavailable workflow is not a pass. A source-level review is not a penetration test. A unit test is not a distributed test. A local build is not proof of deployed behavior. A cryptographic signature is not an independent truth oracle.

Do not claim NIST, W3C, FIDO, OpenID, SPIFFE, or SCITT conformance until the applicable normative tests are run against a pinned specification version and the result is independently reproducible.

## 11. External references

- [NIST SP 800-63-4 Implementation Resources](https://pages.nist.gov/800-63-4-Implementation-Resources/) — final July 2025, with identity proofing, authentication, federation, assurance and implementation resources.
- [NIST SP 800-63A-4: Identity Proofing and Enrollment](https://csrc.nist.gov/pubs/sp/800/63/a/4/final).
- [NIST SP 800-63B-4: Authentication and Authenticator Management](https://csrc.nist.gov/pubs/sp/800/63/b/4/final).
- [NIST SP 800-63C-4: Federation and Assertions](https://csrc.nist.gov/pubs/sp/800/63/c/4/final).
- [W3C Decentralized Identifiers (DIDs) v1.0](https://www.w3.org/TR/did/).
- [W3C Verifiable Credentials Data Model v2.0](https://www.w3.org/TR/vc-data-model/).
- [W3C Verifiable Credential Data Integrity 1.0](https://www.w3.org/TR/vc-data-integrity/).
- [W3C Web Authentication Level 3](https://www.w3.org/TR/webauthn-3/) — W3C Recommendation, 25 August 2026.
- [FIDO Alliance User Authentication Certification Programmes](https://fidoalliance.org/fido-user-authentication-certification-programs/).
- [OpenID Foundation: OpenID4VP and OpenID4VCI HAIP conformance suites](https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/) — announcement, 7 August 2026.
- [SPIFFE Workload API](https://spiffe.io/docs/latest/spiffe-specs/spiffe_workload_api/).
- [SPIFFE X.509-SVID](https://spiffe.io/docs/latest/spiffe-specs/x509-svid/).
- [IETF RFC 9943: An Architecture for Trustworthy and Transparent Digital Supply Chains](https://www.rfc-editor.org/rfc/rfc9943/) — June 2026.
- [SLSA specification](https://slsa.dev/spec/).

---

**Release decision:** This proposal is suitable for review and for building the benchmark corpus. It does not authorize autonomous identity recovery, host mutation, boot promotion, or trust-score-based access. The P0 blockers above remain release blockers until their exact executable evidence is attached.
