# Authority State Challenge Runtime v0.1 — Normative Invariants

Status: **implemented runtime candidate; deliberately unprovisioned in the binding governance DNA**

This zome pair implements only the challenge-issuance role from the authority-state runtime contract.

It does not implement the authority-state source, source-head verifier, witness verifier, trust-domain verifier, coverage composer, governance executor, or any external effect.

## 1. Context authority is mandatory

Challenge issuance calls exactly one independent context provider:

`authority_state_context_policy_verifier.resolve_challenge_context(subject)`

A positive context receipt must bind the exact subject, context-policy digest/profile, coverage-policy digest/profile, bounded challenge lifetime, context lifetime, designated challenge issuer, verification provenance and current lease.

If the provider zome/function is absent, undecodable, stale, inexact, or designates another issuer, issuance denies.

Caller input contains only the requested exact `AuthoritySubjectRef`. Caller-provided context/policy authority is not accepted.

## 2. Caller nonce authority is impossible

The issuance API has no entropy/nonce field.

Entropy is generated inside the coordinator using Holochain's host `random_bytes` function.

The caller cannot select:

- entropy bytes;
- nonce digest;
- randomness-proof ref;
- issue timestamp;
- expiry; or
- challenge issuer.

## 3. Raw entropy is private by construction

`ChallengeEntropyRecord` is registered with:

`#[entry_type(visibility = "private")]`

The raw 32-byte entropy remains only on the issuer's local source chain and is never copied into `AuthorityStateChallengeRecord`.

Every Holochain action is still public; revealing the private entry's create-action hash as a proof reference does not reveal the private entry content.

## 4. Entropy digest is exact and domain separated

The private record commits the exact 32 entropy bytes through `entropy_nonce_digest` using the registered entropy digest profile/domain.

Integrity validation recomputes the digest.

Malformed entropy length, zero/mismatched policy binding, wrong protocol, malformed subject, wrong issuer, or nonce mismatch denies.

## 5. Private proof identity is the exact create action

The public `CoverageChallenge.randomness_proof_ref` must equal the exact `ChallengeEntropyRecord` create-action hash.

It cannot be an arbitrary proof label.

During local verification, the coordinator re-queries its own source chain including private entries and requires:

- the exact entropy action;
- a Create action authored by the current issuer;
- exact private entropy entry bytes;
- recomputed nonce digest;
- exact subject;
- exact context/coverage policy digests/profiles; and
- exact issuer binding.

Generic DHT visibility is not used to recover private entropy.

## 6. Public challenge is immutable audit provenance

`AuthorityStateChallengeRecord` is public and append-only.

Update/delete operations are invalid.

The create action author must equal `challenge.challenge_issuer_ref` as a `did:mycelix:` identifier.

The challenge must already be issued and still live when its public record is committed.

Existence of the public challenge record is not authority by itself.

## 7. Issuance time comes from the committed entropy action

The coordinator does not predict the timestamp Holochain will assign to the private write.

After creating the private entry it queries the exact local action and derives `CoverageChallenge.issued_at_ms` from that committed action timestamp.

The challenge expiry is bounded by the minimum of:

- issue time + context-authorized maximum challenge lifetime;
- context semantic expiry; and
- current context verification lease.

If the context expires during issuance, the whole call fails.

## 8. Source-chain transaction semantics are relied on only locally

Private entropy creation and public challenge creation occur in one zome invocation.

Holochain source-chain transaction semantics provide local commit/rollback behavior for the call.

This does not imply distributed finality, global atomicity, or successful source/witness coverage.

## 9. Verification is issuer-local

`verify_issued_authority_state_challenge` requires the challenge record to have been authored by the current local agent because only that agent can access the corresponding private entropy entry.

Another agent cannot manufacture a positive randomness receipt from the public action hash alone.

## 10. Current context is re-resolved during verification

A valid immutable entropy proof is not timeless current authority.

Before returning `VerifiedCoverageChallenge`, verification calls the independent context provider again and requires the same exact subject/context/coverage-policy/issuer semantics to remain current.

Policy/source rotation or context replacement therefore invalidates stale challenge authority even before nominal challenge expiry.

## 11. Verified challenge receipt exactly echoes proof identity

A positive `VerifiedCoverageChallenge` contains:

- exact public challenge;
- `verified_nonce_digest == challenge.nonce_digest`;
- `verified_randomness_proof_ref == challenge.randomness_proof_ref`;
- `verified_challenge_issuer_ref == challenge.challenge_issuer_ref`;
- deterministic verification reference; and
- current verification time.

This matches the pure #96 trust-context contract.

## 12. Status does not probe authority with synthetic data

`challenge_runtime_status` is declarative only.

It explicitly reports:

- private entropy enabled;
- caller nonce authority disabled;
- required context-provider name;
- no provider probe without a real subject; and
- `operational = false`.

Code presence or provider-name knowledge never implies operational authority.

## 13. Deliberately unprovisioned

The zome pair is part of the Rust workspace only so native/WASM compilation can qualify it.

`mycelix-governance/dna/dna.yaml` MUST NOT contain:

- `authority_state_challenge_integrity`; or
- `authority_state_challenge`.

The context-policy verifier is also not provisioned yet.

Therefore the binding governance DNA cannot invoke this challenge runtime from this tranche.

## 14. No remote source/witness behavior yet

This tranche does not call remote peers, expose the future source-head challenge responder, inspect another agent's activity, classify witness trust domains, or manufacture coverage.

Those remain separate authority roles under the runtime contract.

## 15. No advisory or execution authority

Phi, consciousness, reputation, stake, Guardian status, caller identity, model output and Symthaea recommendations cannot:

- select entropy;
- choose policy/context;
- designate the challenge issuer;
- extend challenge expiry;
- verify private entropy; or
- create coverage/execution authority.

## 16. Required qualification before provisioning

At minimum:

- rustfmt;
- native tests;
- warnings-denied Clippy;
- integrity + coordinator WASM builds;
- static proof that entropy entry visibility is private;
- static proof that caller input contains no nonce/entropy field;
- static proof that context provider failure is fail-closed;
- static proof that private proof retrieval uses local source-chain `query`;
- static proof that synthetic status probing is absent; and
- multi-agent/Sweettest coverage with a real context-policy verifier.

External effects remain disabled.
