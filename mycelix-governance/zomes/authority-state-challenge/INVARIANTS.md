# Authority-State Probe Runtime v0.1 — Normative Invariants

Status: **implemented runtime candidate; deliberately unprovisioned in the binding governance DNA**

This zome pair implements only fresh read-only evidence probing.

A probe is **not authority**.

It does not prove that the candidate coverage/context policies are current, does not prove the target authority subject is current, and cannot authorize governance, mutation, voting, lifecycle claims, or external effects.

## 1. Bootstrap-cycle rule

The runtime MUST NOT require current operational coverage/context policy freshness merely to generate the probe used to discover that freshness.

The invalid dependency is:

`probe -> current policy freshness -> probe`.

`issue_authority_state_probe` therefore performs no call to:

- `authority_state_context_policy_verifier`;
- `authority_current_freshness_verifier`;
- an issuer-grant verifier; or
- any execution-authority provider.

The probe becomes useful only when later constitution/root or operational qualification accepts the exact policy identities it carries.

## 2. Caller policy selection is candidate evidence, not authority

The request may identify:

- one exact `AuthoritySubjectRef`;
- one exact candidate context-policy digest;
- one exact candidate coverage-policy digest; and
- a bounded requested lifetime.

The runtime supplies the registered policy profiles itself.

Caller-selected candidate digests are **not** treated as current policy. Later #96/root qualification must recompute and accept those exact identities independently.

## 3. Caller nonce authority is impossible

The issuance API has no entropy, nonce, randomness-proof, timestamp, or issuer field.

Entropy is generated inside the coordinator using Holochain host `random_bytes`.

The caller cannot select:

- entropy bytes;
- nonce digest;
- randomness-proof ref;
- issue timestamp; or
- probe author/provenance.

The caller may request a shorter lifetime, bounded by `MAX_PROBE_LIFETIME_MS`. A requested lifetime grants no authority and later policy qualification may reject it as too long for the accepted context.

## 4. Raw entropy is private by construction

`ChallengeEntropyRecord` is registered with:

`#[entry_type(visibility = "private")]`

The raw 32-byte entropy remains only on the probe author's local source chain and is never copied into the public record.

Every Holochain action remains public; revealing the private entry's action hash does not reveal the private entry content.

## 5. Candidate policy identities are exact

The private entropy record and public `CoverageChallenge` both commit:

- exact context-policy digest + registered `CONTEXT_POLICY_PROFILE`;
- exact coverage-policy digest + registered `POLICY_IDENTITY_PROFILE`; and
- exact target subject.

Zero digests or wrong profiles deny.

The runtime does not verify currentness of those identities during probe creation.

## 6. Entropy proof is exact and domain separated

The private record commits exactly 32 entropy bytes through `entropy_nonce_digest` under the registered entropy profile/domain.

Integrity validation recomputes the nonce digest.

The public challenge `randomness_proof_ref` must equal the exact private entropy create-action hash.

## 7. Author identity is provenance only

The private entropy `issued_by` and public `challenge_issuer_ref` must equal the committing agent as `did:mycelix:<AgentPubKey>`.

This prevents provenance substitution.

It does **not** mean that the author holds an institutional grant or that the probe is authoritative.

No `AuthorityGrant` is required merely to collect read-only freshness evidence.

Operational deployments may rate-limit or capability-protect the endpoint for abuse resistance, but such access control is not governance authority.

## 8. Public probe is immutable audit provenance

`AuthorityStateChallengeRecord` is public and append-only.

Update/delete operations are invalid.

The public record stores only:

- protocol;
- exact `CoverageChallenge`; and
- exact private entropy action hash.

It deliberately contains no `context_verification_ref`, because probe creation does not claim context authority.

## 9. Committed issue time

The coordinator does not predict Holochain's action timestamp.

After committing private entropy, it queries the exact local record and derives `CoverageChallenge.issued_at_ms` from the committed action timestamp.

Expiry is `issued_at_ms + requested_lifetime_ms`, with overflow denial and the hard v0.1 maximum.

Later accepted context/root policy may impose a shorter valid window; #96 rejects probes outside that policy window.

## 10. Private proof verification is issuer-local

`verify_issued_authority_state_probe` requires the public probe to have been authored by the current local agent because only that agent can access the private entropy content.

It queries the author's local source chain with entries included and requires:

- exact private entropy action;
- Create action authored by the probe author;
- exact entropy bytes;
- recomputed nonce digest;
- exact subject;
- exact candidate policy digests/profiles; and
- exact author provenance.

Generic DHT visibility is not used to recover private entropy.

## 11. Probe verification proves randomness/provenance only

A positive `VerifiedCoverageChallenge` from this runtime means only that:

- the public challenge exists and is live;
- the private entropy proof exists locally;
- the nonce digest recomputes;
- the exact candidate policy identities are bound into the entropy/probe pair; and
- the author/provenance echo is exact.

It does **not** mean either candidate policy is current.

There is intentionally no current-context lookup in probe verification.

## 12. Later authority qualification remains mandatory

Before probe evidence can contribute to `VerifiedAuthorityFreshness`, later layers must independently prove at least:

- the exact accepted bootstrap/operational context policy;
- the exact accepted coverage policy;
- source-head evidence under the exact challenge;
- witness/trust bindings when required;
- complete covered transition lineage; and
- exact current-state projection.

For control-plane policy subjects this must be rooted through the current constitution/bootstrap root from #111/#109, not recursively through the operational policy plane.

## 13. Status semantics

`probe_runtime_status` is declarative only.

It reports:

- private entropy enabled;
- caller nonce authority disabled;
- candidate policy selection allowed;
- candidate policy selection grants no authority;
- probe grants no authority; and
- `operational = false` while unprovisioned.

No provider is probed with synthetic data.

## 14. Deliberately unprovisioned

The zome pair remains in the Rust workspace only for native/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain:

- `authority_state_challenge_integrity`; or
- `authority_state_challenge`.

Therefore the binding governance DNA cannot invoke the probe runtime from this tranche.

## 15. No remote source/witness behavior yet

This tranche does not:

- call remote authority sources;
- inspect another agent's activity;
- classify witness trust domains;
- qualify coverage;
- project current state; or
- manufacture freshness.

Those remain separate roles.

## 16. No advisory or effect authority

Phi, consciousness, reputation, stake, Guardian status, caller identity, model output and Symthaea recommendations cannot:

- select entropy;
- turn candidate policy IDs into current policy;
- transform a probe into authority;
- extend a qualified policy window;
- create lifecycle claims; or
- execute effects.

## 17. Required qualification before provisioning

At minimum:

- rustfmt;
- native tests;
- warnings-denied Clippy;
- integrity + coordinator WASM builds;
- private-entry visibility test;
- caller-selected nonce/replay denial;
- exact candidate-context binding tests;
- probe-expiry tests;
- bootstrap-root/context rotation tests;
- proof that no current-policy/issuer-grant call occurs during probe issuance; and
- Sweettest proving probe existence alone never creates current authority.

External effects remain disabled.
