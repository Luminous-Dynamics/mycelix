# Mycelix Identity Assurance Review — 2026-11-10

## Scope and claim boundary

This review targets the active Holochain identity DNA in
\`mycelix-workspace/mycelix-identity\`. It does not attribute results to the
legacy 0TML identity prototype, the separate root \`mycelix-identity\` crate,
or to Mycelix network-consensus/BFT properties.

This is a source review plus executable integrity tests. It is not a penetration
test of deployed conductors, a cryptographic audit, a DID-method conformance
certification, or proof of distributed identity uniqueness.

## Changes in this review branch

### Self-recovery configuration ownership

The recovery integrity zome previously checked that \`config.owner\` matched
the committing agent but did not require \`config.did\` to be the DID derived
from that same agent. A modified coordinator could therefore commit a recovery
configuration whose owner field was the attacker while the configuration
named a victim DID.

The branch now:
- binds the recovery-config DID to the author’s agent key at the integrity
  layer, not just in the coordinator;
- validates nonzero anchor thresholds, upper threshold bounds, non-empty
  anchor values and duplicate-free anchor lists;
- validates recovery-config updates against the original record;
- makes DID, owner and creation time immutable;
- requires update timestamps to advance;
- prevents previously enrolled anchors from being silently removed or
  replaced; and
- makes social-recovery supersession monotonic.

The coordinator also rejects a foreign DID at the API boundary and rejects
duplicate anchor enrollment before attempting an update. The integrity zome
remains authoritative if a coordinator is modified or bypassed.

### Proofless self-recovery disabled

The existing \`VerificationAnchor\` enum stores strings such as phone/email
hashes, passkey credential IDs, device-attestation hashes and biometric hashes.
The existing coordinator treats equality between a supplied string and an
enrolled value as a successful verification. It does not verify a signed
challenge, WebAuthn assertion, trusted device attestation chain, or any other
proof that the caller controls the factor.

The request entry also has no cryptographic possession-proof receipt. As a
result, the integrity zome cannot independently distinguish a real proof from
a caller-asserted anchor value. It previously allowed a Pending request to be
updated to Approved without a proof object.

This branch therefore fails closed:
- self-recovery requests cannot be created with claimed verified anchors;
- v1 cannot append or remove entries in the \`verified_anchors\` field;
- Pending → Approved is rejected by integrity validation;
- the coordinator's initiate, verify and execute self-recovery endpoints
  return an explicit unavailable error until proof-backed v2 exists.

**Social Recovery uses separate entry types and is not disabled by this
change.** This is an intentional compatibility trade-off: users must use the
separate trustee-mediated recovery path while proof-backed self-recovery is
being designed and independently tested.

### CI now reaches the active workspace

The monorepo root workflow's identity path filter previously included
\`mycelix-identity/**\` but not
\`mycelix-workspace/mycelix-identity/**\`. The latter is where the active
Holochain identity DNA lives. Changes in that tree could therefore miss the
root identity test job.

The root workflow is updated to:
- trigger the identity path filter for the active nested Holochain workspace;
- run \`cargo fmt --all --check\` in that workspace;
- run \`cargo test -p recovery_integrity --lib\`;
- compile the \`recovery\` coordinator crate on the host; and
- build \`recovery_integrity\` and \`recovery\` for
  \`wasm32-unknown-unknown\`.

The Holochain conductor/SweetConductor tests are not yet made a required PR
gate. They need a dedicated, correctly located root workflow job that builds
the identity WASM zomes, packs the DNA and invokes the nested test package.
A workflow file stored at
\`mycelix-workspace/.github/workflows/\` is not executed by GitHub Actions as
a repository workflow because it is not in the repository-root
\`.github/workflows/\` directory.

## Remaining identity blockers

These are deliberately not represented as fixed by this branch.

### P0 — proof of possession for recovery factors

A secure v2 design should not treat knowledge of a hashed phone number or
email address as possession. Each proof needs a fresh, single-use challenge
bound to:
- the target DID;
- the exact recovery request and replacement agent key;
- the factor type and enrolled factor identifier;
- an expiry and nonce;
- a verifier-defined audience/context; and
- the exact evidence digest.

Phone/email should use an independent delivery and challenge-verification
mechanism. Passkeys should use WebAuthn assertion verification including
challenge, RP ID hash, origin, flags, signature and monotonic credential
counter. Device attestations need a verified certificate/measurement chain.
Biometrics should never be a raw template or a caller-supplied hash masquerading
as proof. A threshold must count distinct factor classes/independent trust
domains, not merely multiple values stored by one attacker.

### P0 — social recovery multi-agent state convergence

The recovery integrity source documents that the coordinator's
\`check_and_update_request_status\` searches the local source chain using
\`query()\`. A trustee other than the request creator may not find the
request record locally, so a cross-agent threshold vote may fail to promote
the request even when votes are present on the DHT.

Add a multi-conductor test in which the request initiator and each trustee are
different agents. Verify that:
- votes are authenticated to the trustee agent at integrity level;
- duplicate votes cannot count twice;
- every trustee observes the same vote set;
- threshold calculation is deterministic and independent of local-chain
  placement and message order;
- timeout/rejection/cancellation races resolve consistently; and
- only the designated replacement agent can claim the DID.

### P0 — self-recovery completion and DID-claim consistency

The current self-recovery executor writes the request as Completed before
making a cross-zome call to \`did_registry.claim_recovered_did\`, then ignores
the call result. The call payload currently appears to be only the DID string,
while the DID registry claim input expects a request ID plus DID. This can
leave a recovery request marked Completed even when the DID transfer failed.

Design a retryable two-stage transition (for example, proof-approved →
claim-pending → claim-confirmed) with exact request identity, idempotency and
an independently checkable claim receipt. Never swallow an unauthorized,
not-found, decode or network error and report completion. Do not solve this by
moving the status change earlier in the sequence.

### P1 — DID key lifecycle

Add adversarial tests for:
- DID ID/controller/committer mismatch;
- verification methods whose controller is not the DID controller;
- duplicate method IDs and duplicate authentication/keyAgreement references;
- key ID reuse and algorithm/key-length mismatch;
- rotation to a key for which the subject has not demonstrated possession;
- old-key deactivation and stale-key replay;
- key agreement rotation preserving historical decryptability while excluding
  deprecated keys from new encryption; and
- simultaneous DID updates/key rotations.

A public key's syntactic validity is not proof that its owner possesses the
corresponding private key. New authentication keys should be proven using a
fresh challenge signature bound to the DID and rotation plan.

### P1 — credentials and revocation

Build a cross-zome adversarial suite that verifies issuer authorization,
subject binding, schema version, expiry, status-list authenticity, suspension
semantics, terminal revocation, duplicate status entries, stale status reads,
and revocation propagation. Verifiers should treat unknown/unavailable status
as unresolved rather than silently valid.

### P1 — local key custody

The browser-local identity implementation should be evaluated separately from
the Holochain agent key. Plain localStorage persistence without passphrase
protection is not acceptable as the default for long-lived identity secrets.
Test encrypted persistence, lock/unlock, wrong passphrase, storage tampering,
origin changes, backup/restore and key zeroization behavior. Do not claim that
a local encrypted key store protects against a compromised browser origin.

## Benchmark programme

Use fixed seeds, frozen fixtures, explicit threat models and independent
expected outcomes. Report metrics separately rather than collapsing identity
safety into one weighted score.

| Dimension | Required measurement | Initial acceptance target |
|---|---|---|
| DID ownership binding | Unauthorized DID create/update/deactivation attempts | 100% rejected |
| Key lifecycle | Unauthorized rotation, key substitution, replay | 100% rejected |
| Recovery proof | Wrong, stale, replayed, cross-DID and forged factor evidence | 100% rejected |
| Threshold recovery | Duplicate votes, duplicate factors, colluding subsets | No threshold reached below policy |
| Revocation | Revoked/suspended/expired/unknown credential cases | No revoked credential accepted |
| Convergence | Multi-conductor request/vote/status observations | Identical terminal outcome for all honest agents |
| Recovery availability | Valid recovery under node/network partitions | Report completion rate and latency; no safety shortcut |
| Privacy | Raw identifiers/secrets in DHT entries, logs and receipts | Zero unintended disclosure |
| Compatibility | Prior DID/VC MessagePack and JSON fixtures | 100% supported fixtures preserved |

The 100% negative-control target means every enumerated attack case in a
frozen corpus, not a claim that all possible attacks are known. Publish corpus
size, test IDs, coverage boundaries and independent review results.

## Immediate next steps

1. Complete exact-head compilation, formatting and unit tests for the modified
   recovery integrity/coordinator crates.
2. Add a root-level identity conductor/SweetConductor job and dedicated tests
   for foreign recovery configs, proofless anchor approval, trustee voting
   across agents and DID claim completion.
3. Design a versioned cryptographic possession-proof envelope and verifier;
   do not re-enable self-recovery until its integrity checks are executable.
4. Run the broader DID, key rotation, VC, revocation and local key-custody
   negative-control suite.
5. Keep identity assurance separate from trust scoring, reputation, agent
   autonomy and the network's BFT assumptions.

## Scope note

Passing these tests does not establish Mycelix consensus safety, Sybil
resistance, 34% BFT fault tolerance, cryptographic implementation correctness,
or security of deployed devices. Those require separate, independently
specified and adversarially tested programmes.
