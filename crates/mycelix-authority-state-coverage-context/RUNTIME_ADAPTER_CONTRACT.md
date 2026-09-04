# Mycelix Authority State Runtime Adapter v0.1 — Normative Contract

Status: **runtime integration contract only; no authority/effect zome is enabled by this document**

This contract specifies how a future Holochain 0.6 runtime may obtain the evidence required by:

- `mycelix-authority-state-source`;
- `mycelix-authority-state-coverage`; and
- `mycelix-authority-state-coverage-context`.

The core rule is:

> Local DHT visibility is not proof of authoritative completeness.

The runtime must prove one fresh, policy-qualified source head and, when required, independent witness coverage before producing `VerifiedAuthorityStateCoverage`.

## 1. Runtime roles remain separate

The implementation must keep at least these logical roles distinct:

1. **challenge issuer** — creates fresh policy-bound verifier challenges;
2. **authority-state source** — owns/publishes the append-only source state being covered;
3. **transition-authority verifier** — proves each generation transition was institutionally authorized;
4. **source-head verifier** — verifies the exact source response and source identity;
5. **witness agent(s)** — independently observe the claimed source chain/head;
6. **witness trust-domain verifier** — classifies observers under the institution-adopted trust policy; and
7. **coverage composer** — invokes the pure #94/#96 qualification kernels.

One process may host more than one role operationally, but the authority evidence and provider identities must remain explicit. Co-location never implies authority equivalence.

## 2. Challenge issuance uses host randomness, not caller bytes

A coordinator zome may use Holochain's host-provided randomness and system time to generate challenge entropy.

The runtime challenge path must:

- resolve the exact current institution-adopted `CoverageTrustContextPolicy`;
- resolve the exact referenced `AuthorityCoveragePolicy`;
- generate fresh entropy inside the coordinator using the Holochain host randomness API;
- commit a digest of that entropy into `CoverageChallenge.nonce_digest`;
- bind the exact context-policy and coverage-policy digests/profiles;
- bind the exact authority subject;
- bind exact challenge issue/expiry times; and
- bind the exact challenge issuer.

Caller-supplied random bytes or caller-selected nonce digests are forbidden.

## 3. Randomness proof must be replayable locally

The runtime must be able to re-establish that the exact `nonce_digest` came from the approved challenge-generation path.

A recommended v0.1 implementation is:

1. create a **private source-chain entropy record** containing the raw host-generated random bytes and issuance metadata;
2. use that immutable record/action identity as `randomness_proof_ref`;
3. construct the public challenge from the entropy digest and policy context; and
4. later re-query the issuer's own private source chain to verify the exact entropy record before constructing `VerifiedCoverageChallenge`.

The entropy bytes themselves must never be placed on the DHT.

Challenge issuance and its local proof records should occur in one zome call so Holochain's source-chain transaction semantics commit or reject the complete issuance operation together.

## 4. Challenge issuer identity is explicit

`challenge_issuer_ref` must identify the exact issuing workload/agent authority.

The challenge verifier must re-establish:

- the exact challenge record;
- the exact issuing author;
- the exact entropy proof record;
- the exact nonce digest; and
- the exact policy/context bindings.

A non-empty issuer string or application-supplied `verified=true` field is insufficient.

## 5. Remote source-head challenge endpoint

The configured authority-state source should expose a narrowly scoped remote-call endpoint similar to:

`respond_authority_state_head_challenge(challenge)`

The endpoint must be capability protected. An unrestricted grant is not the default assumption.

The verifier must treat remote-call network failure, capability revocation/Unauthorized, decode failure, timeout/unreachability, or unexpected zome/function mapping as denial for current coverage.

Availability failure is not evidence that the old cached head remains current.

## 6. Source response snapshots its exact local chain head

The source responder must use its current source-chain state at the time of the challenge response.

The response must bind at least:

- exact challenge digest;
- exact authority subject;
- logical authoritative-source identity;
- exact source identity/key profile;
- exact latest authority-state generation for that subject;
- exact latest authority-state transition digest;
- exact transition/status record reference;
- exact source-chain head action hash;
- source-chain head sequence/timestamp in the runtime envelope;
- response/expiry time; and
- cryptographic proof/signature over the exact response.

The pure #94 source-head identity may carry the chain head as a reference string; the runtime envelope should retain typed Holochain chain-head fields for verification/audit rather than discarding them.

## 7. Source head is derived from authored history, not a mutable cache

The source must determine the latest authority-state transition by querying its own append-only authored source-chain records for the exact subject.

It must not answer from:

- a mutable in-memory cache;
- a DHT link whose recency is assumed;
- caller-provided generation;
- highest timestamp from unverified records; or
- a mutable `current_generation` field without replayable lineage.

The exact returned transition must itself be independently qualified by the transition-authority verifier before later current-authority use.

## 8. Source chain head and authority transition head are distinct

The latest authority-state transition need not be the latest action on the source chain.

The response therefore binds both:

- **authority transition head** — latest qualified authority-state transition for the subject; and
- **source-chain head** — current source-chain action head at response time.

A witness can then verify that it observed the source's public activity through the claimed source-chain head and that no later authority-state transition for the subject occurs within that covered range.

## 9. Source proof uses the configured source identity

The source response must be signed/verified against the exact source identity committed by the adopted coverage policy.

A signature under another locally available key, caller key, old source key, or same-name source is denial.

Key/profile changes require a newly adopted policy or explicit source-identity transition; they do not silently carry forward.

## 10. Witnesses independently verify the claimed source chain

A witness must not merely echo the source response.

For the exact source agent/head, the witness should use Holochain public source-chain activity primitives to independently establish a valid contiguous view through the claimed chain head.

The witness must deny when activity is:

- forked;
- invalid;
- incomplete for the required range;
- missing the claimed chain head;
- unavailable; or
- inconsistent with the source response.

Where `must_get_agent_activity` can prove a deterministic contiguous range through the claimed head, prefer that stronger result over recency heuristics.

## 11. Witnesses verify authority transitions inside the covered range

The witness must inspect the public authority-state transition actions/entries for the exact subject through the claimed source-chain head.

It must prove that:

- the claimed transition head exists in that range;
- its transition digest/status record match the source response; and
- there is no later authority-state transition for that subject within the covered chain range.

Failure to retrieve required public records/entries is denial.

Authority-state transition entries needed for coverage must therefore be public; private transition data cannot support independent witness coverage.

## 12. Witness coverage is not global DHT finality

A positive witness statement means:

> This independently verified observer saw a valid contiguous public source-chain view through the exact challenged source head and found the claimed authority transition head consistent within that range.

It does **not** mean:

- every DHT authority has integrated all ops;
- every network peer agrees;
- no later source-chain action can ever be written; or
- the entire Holochain network has reached global consensus.

The challenge/lease bounds intentionally make coverage a short-lived current-state claim.

## 13. Trust-domain classification is a separate provider

A witness does not self-declare its independence class.

For every witness observation, the runtime must call the institution-adopted trust-domain verifier named by the exact `CoverageTrustContextPolicy`.

That verifier must bind:

- exact witness observation identity;
- exact observer ID;
- exact trust-domain ID/ref;
- exact witness-trust policy digest/profile;
- exact classification proof; and
- bounded current validity.

Only then may the runtime construct `VerifiedWitnessTrustBinding`.

## 14. Same operator running many keys does not create independence

Trust-domain policy must be capable of grouping multiple observer keys/processes under one independence domain.

The runtime must not infer independence from:

- different AgentPubKeys;
- different IP/transport addresses;
- different conductor instances;
- different process IDs; or
- different claimed organization names.

#94's `max_per_trust_domain` and `min_trust_domains` are enforced only after trusted classification.

## 15. Current coverage is challenge-scoped and short-lived

All source/witness/trust evidence must be valid for the same exact challenge.

The final lease is the minimum of all dependency horizons.

At or after lease expiry, current authority requires a new challenge and fresh qualification.

A cached positive coverage receipt cannot be extended because network calls are unavailable.

## 16. Source compromise and refusal are distinct from coverage

Witness coverage can detect inconsistency between a claimed source head and independently visible authored history.

It cannot force a compromised/refusing source to author a legitimate revocation transition that it never wrote.

Institutional designs requiring resilience against source refusal/compromise should use a stronger source architecture (for example governed source rotation, replicated/threshold source authority, or independently writable revocation authority) rather than pretending witness quorum solves source liveness.

The coverage layer must state this limitation explicitly.

## 17. Source rotation is governed authority

Changing the logical authoritative source, source identity/key, or source-verification profile requires an explicit institution-authorized policy/source transition.

A runtime may not fail over to another reachable agent and preserve the same `authoritative_source_ref` unless that failover mechanism is itself part of the verified logical-source identity and policy.

## 18. Local DHT queries are supporting evidence only

DHT `get`, links, or locally integrated activity may help discover records.

They may not independently create `VerifiedAuthorityStateCoverage`.

A query that returns no generation N+1 record proves only that this node did not return one in that query.

It does not prove generation N is authoritative current state.

## 19. Historical verification uses immutable receipts, not fresh challenge reuse

Historical/as-of verification should preserve:

- immutable transition records/proofs;
- historical source response;
- historical witness observations;
- historical trust classifications; and
- the challenge/context effective at that event time.

A historical receipt may establish that coverage was valid then.

It cannot satisfy a new live challenge or recreate current authority after revocation/expiry.

## 20. Fail-closed runtime status

Before provisioning the runtime, expose a status endpoint that distinguishes:

- pure contracts installed;
- challenge generator available;
- context/policy verifier available;
- source responder reachable/authorized;
- source proof verifier available;
- witness verifier available;
- trust-domain verifier available;
- coverage composer available; and
- **current coverage authority operational**.

Code presence alone must never imply operational authority.

## 21. No external effects

The authority-state runtime only supplies current/historical authority freshness evidence.

It must not execute governance actions, mutate lifecycle effects, or bypass the separate execution-authority/effect-safety chain.

## 22. Required adversarial tests before enablement

At minimum, Sweettest/multi-agent coverage must include:

- caller-selected/stale challenge replay;
- challenge randomness-proof rebinding;
- wrong challenge issuer;
- source response under wrong source key/profile;
- source response for another policy/context;
- source-chain head mismatch;
- forked source-chain activity;
- incomplete activity range;
- withheld later authored revocation detected by witness;
- witness echo without independent source-chain retrieval;
- same observer under conflicting trust domains;
- multiple keys mapped to one trust domain;
- insufficient trust-domain diversity;
- source/witness/trust verifier outage;
- capability revocation / Unauthorized remote call;
- policy/source rotation during an active lease;
- lease expiry during qualification;
- historical proof after later revocation; and
- proof that historical coverage cannot satisfy a new current challenge.

External effects remain disabled until these authority-state tests and the downstream execution authority stack are green.
